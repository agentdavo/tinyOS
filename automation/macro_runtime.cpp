// SPDX-License-Identifier: MIT OR Apache-2.0

#include "macro_runtime.hpp"

#include "miniOS.hpp"
#include "cnc/offsets.hpp"
#include "motion/motion.hpp"
#include "probe_runtime.hpp"
#include "machine/toolpods.hpp"
#include "util.hpp"

namespace macros {

namespace {

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

const char* next_line(const char* p, const char* end, char* out, size_t out_size) {
    size_t i = 0;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    while (p < end && *p != '\r' && *p != '\n' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    while (p < end && *p != '\r' && *p != '\n') ++p;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    return p;
}

const char* field_value(const char* line, const char* key, char* scratch, size_t scratch_size) {
    if (!line || !key || !scratch || scratch_size == 0) return nullptr;
    const char* p = line;
    while (*p && *p != '\t') ++p;
    while (*p == '\t') {
        ++p;
        const char* field = p;
        while (*p && *p != '\t') ++p;
        const char* eq = field;
        while (eq < p && *eq != '=') ++eq;
        if (eq >= p) continue;
        const size_t key_len = static_cast<size_t>(eq - field);
        if (kernel::util::kstrlen(key) == key_len &&
            kernel::util::kmemcmp(field, key, key_len) == 0) {
            const char* value = eq + 1;
            const size_t value_len = static_cast<size_t>(p - value);
            const size_t copy = value_len + 1 < scratch_size ? value_len : scratch_size - 1;
            for (size_t i = 0; i < copy; ++i) scratch[i] = value[i];
            scratch[copy] = '\0';
            return scratch;
        }
    }
    return nullptr;
}

long parse_long(const char* s, long fallback = 0) {
    if (!s || !*s) return fallback;
    bool neg = false;
    if (*s == '-') {
        neg = true;
        ++s;
    }
    long value = 0;
    while (*s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        ++s;
    }
    return neg ? -value : value;
}

float parse_float(const char* s, float fallback = 0.0f) {
    if (!s || !*s) return fallback;
    bool neg = false;
    if (*s == '-') {
        neg = true;
        ++s;
    }
    float value = 0.0f;
    while (*s >= '0' && *s <= '9') {
        value = value * 10.0f + static_cast<float>(*s - '0');
        ++s;
    }
    if (*s == '.') {
        ++s;
        float place = 0.1f;
        while (*s >= '0' && *s <= '9') {
            value += static_cast<float>(*s - '0') * place;
            place *= 0.1f;
            ++s;
        }
    }
    return neg ? -value : value;
}

machine::SymbolType parse_type(const char* s) {
    if (!s || kernel::util::kstrcmp(s, "bool") == 0) return machine::SymbolType::Bool;
    if (kernel::util::kstrcmp(s, "float") == 0) return machine::SymbolType::Float;
    return machine::SymbolType::Int;
}

CompareOp parse_compare(const char* s) {
    if (!s || kernel::util::kstrcmp(s, "eq") == 0) return CompareOp::Equal;
    if (kernel::util::kstrcmp(s, "ne") == 0) return CompareOp::NotEqual;
    if (kernel::util::kstrcmp(s, "lt") == 0) return CompareOp::Less;
    if (kernel::util::kstrcmp(s, "le") == 0) return CompareOp::LessEqual;
    if (kernel::util::kstrcmp(s, "gt") == 0) return CompareOp::Greater;
    return CompareOp::GreaterEqual;
}

ComputeOp parse_compute(const char* s) {
    if (!s || kernel::util::kstrcmp(s, "copy") == 0) return ComputeOp::Copy;
    if (kernel::util::kstrcmp(s, "add") == 0) return ComputeOp::Add;
    if (kernel::util::kstrcmp(s, "sub") == 0) return ComputeOp::Subtract;
    if (kernel::util::kstrcmp(s, "mul") == 0) return ComputeOp::Multiply;
    if (kernel::util::kstrcmp(s, "div") == 0) return ComputeOp::Divide;
    if (kernel::util::kstrcmp(s, "avg") == 0) return ComputeOp::Average;
    return ComputeOp::None;
}

machine::SymbolValue parse_symbol_value(machine::SymbolType type, const char* s) {
    machine::SymbolValue value{};
    switch (type) {
        case machine::SymbolType::Bool:
            value.bool_value = parse_long(s, 0) != 0;
            value.int_value = value.bool_value ? 1 : 0;
            value.float_value = value.bool_value ? 1.0f : 0.0f;
            break;
        case machine::SymbolType::Int:
            value.int_value = static_cast<int32_t>(parse_long(s, 0));
            value.bool_value = value.int_value != 0;
            value.float_value = static_cast<float>(value.int_value);
            break;
        case machine::SymbolType::Float:
            value.float_value = parse_float(s, 0.0f);
            value.bool_value = value.float_value != 0.0f;
            value.int_value = static_cast<int32_t>(value.float_value);
            break;
    }
    return value;
}

bool parse_bool(const char* s) {
    return parse_long(s, 0) != 0;
}

bool symbol_numeric_value(const char* name, float& out) {
    if (!name || !*name) return false;
    return machine::g_registry.get_float(name, out);
}

} // namespace

Runtime g_runtime;

bool Runtime::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    macro_count_ = 0;
    step_count_ = 0;
    const char* p = buf;
    const char* end = buf + len;
    char line[256];
    while (p < end) {
        p = next_line(p, end, line, sizeof(line));
        char* cursor = line;
        while (is_space(*cursor)) ++cursor;
        if (*cursor == '\0' || *cursor == '#') continue;
        char* rest = cursor;
        while (*rest && *rest != '\t') ++rest;
        if (*rest == '\t') *rest++ = '\0';
        const char* record = cursor;
        char scratch_a[64];
        char scratch_b[64];
        char scratch_c[64];
        char scratch_d[64];
        if (kernel::util::kstrcmp(record, "symbol") == 0) {
            const char* name = field_value(rest, "name", scratch_a, sizeof(scratch_a));
            const auto type = parse_type(field_value(rest, "type", scratch_b, sizeof(scratch_b)));
            const auto initial = parse_symbol_value(type, field_value(rest, "initial", scratch_c, sizeof(scratch_c)));
            const bool writable = parse_bool(field_value(rest, "writable", scratch_d, sizeof(scratch_d)));
            (void)machine::g_registry.define_symbol(name, type, initial, writable, false);
            continue;
        }
        if (kernel::util::kstrcmp(record, "macro") == 0) {
            if (macro_count_ >= MAX_MACROS) continue;
            auto& macro = macros_[macro_count_++];
            kernel::util::k_snprintf(macro.id, sizeof(macro.id), "%s", field_value(rest, "id", scratch_a, sizeof(scratch_a)));
            kernel::util::k_snprintf(macro.title, sizeof(macro.title), "%s",
                                     field_value(rest, "title", scratch_b, sizeof(scratch_b)) ? field_value(rest, "title", scratch_b, sizeof(scratch_b)) : macro.id);
            macro.mcode = static_cast<uint16_t>(parse_long(field_value(rest, "mcode", scratch_c, sizeof(scratch_c)), 0));
            macro.first_step = step_count_;
            macro.step_count = 0;
            continue;
        }
        if (kernel::util::kstrcmp(record, "step") == 0 && macro_count_ > 0 && step_count_ < MAX_STEPS) {
            auto& step = steps_[step_count_++];
            const char* type_name = field_value(rest, "type", scratch_a, sizeof(scratch_a));
            if (type_name && kernel::util::kstrcmp(type_name, "set") == 0) step.kind = StepKind::Set;
            else if (type_name && kernel::util::kstrcmp(type_name, "delay") == 0) step.kind = StepKind::Delay;
            else if (type_name && kernel::util::kstrcmp(type_name, "wait") == 0) step.kind = StepKind::Wait;
            else if (type_name && kernel::util::kstrcmp(type_name, "move") == 0) step.kind = StepKind::Move;
            else if (type_name && kernel::util::kstrcmp(type_name, "pod_select") == 0) step.kind = StepKind::PodSelect;
            else if (type_name && kernel::util::kstrcmp(type_name, "tool_assign") == 0) step.kind = StepKind::ToolAssign;
            else if (type_name && kernel::util::kstrcmp(type_name, "capture_axis") == 0) step.kind = StepKind::CaptureAxis;
            else if (type_name && kernel::util::kstrcmp(type_name, "capture_probe") == 0) step.kind = StepKind::CaptureProbe;
            else if (type_name && kernel::util::kstrcmp(type_name, "compute") == 0) step.kind = StepKind::Compute;
            else if (type_name && kernel::util::kstrcmp(type_name, "set_work_offset") == 0) step.kind = StepKind::SetWorkOffset;
            else if (type_name && kernel::util::kstrcmp(type_name, "sphere_calibrate") == 0) step.kind = StepKind::SphereCalibrate;
            else if (type_name && kernel::util::kstrcmp(type_name, "alarm") == 0) step.kind = StepKind::Alarm;
            step.value_type = parse_type(field_value(rest, "value_type", scratch_b, sizeof(scratch_b)));
            kernel::util::k_snprintf(step.symbol, sizeof(step.symbol), "%s",
                                     field_value(rest, "symbol", scratch_c, sizeof(scratch_c)) ? field_value(rest, "symbol", scratch_c, sizeof(scratch_c)) : "");
            kernel::util::k_snprintf(step.source, sizeof(step.source), "%s",
                                     field_value(rest, "src", scratch_b, sizeof(scratch_b)) ? field_value(rest, "src", scratch_b, sizeof(scratch_b)) : "");
            kernel::util::k_snprintf(step.source_b, sizeof(step.source_b), "%s",
                                     field_value(rest, "src_b", scratch_c, sizeof(scratch_c)) ? field_value(rest, "src_b", scratch_c, sizeof(scratch_c)) : "");
            kernel::util::k_snprintf(step.pod_id, sizeof(step.pod_id), "%s",
                                     field_value(rest, "pod", scratch_d, sizeof(scratch_d)) ? field_value(rest, "pod", scratch_d, sizeof(scratch_d)) : "");
            kernel::util::k_snprintf(step.axis, sizeof(step.axis), "%s",
                                     field_value(rest, "axis", scratch_d, sizeof(scratch_d)) ? field_value(rest, "axis", scratch_d, sizeof(scratch_d)) : "");
            kernel::util::k_snprintf(step.message, sizeof(step.message), "%s",
                                     field_value(rest, "message", scratch_a, sizeof(scratch_a)) ? field_value(rest, "message", scratch_a, sizeof(scratch_a)) : "");
            step.value = parse_symbol_value(step.value_type, field_value(rest, "value", scratch_b, sizeof(scratch_b)));
            step.alt_value = parse_symbol_value(machine::SymbolType::Int, field_value(rest, "position", scratch_c, sizeof(scratch_c)));
            step.compare = parse_compare(field_value(rest, "compare", scratch_d, sizeof(scratch_d)));
            step.compute = parse_compute(field_value(rest, "op", scratch_a, sizeof(scratch_a)));
            step.timeout_ms = static_cast<uint32_t>(parse_long(field_value(rest, "timeout_ms", scratch_a, sizeof(scratch_a)), 0));
            step.scale = parse_float(field_value(rest, "scale", scratch_b, sizeof(scratch_b)), 1.0f);
            step.bias = parse_float(field_value(rest, "bias", scratch_c, sizeof(scratch_c)), 0.0f);
            step.direction = parse_float(field_value(rest, "direction", scratch_a, sizeof(scratch_a)), 0.0f);
            step.relative = !field_value(rest, "relative", scratch_d, sizeof(scratch_d)) ||
                            parse_bool(field_value(rest, "relative", scratch_d, sizeof(scratch_d)));
            ++macros_[macro_count_ - 1].step_count;
        }
    }
    return macro_count_ != 0;
}

bool Runtime::select(size_t idx) noexcept {
    if (idx >= macro_count_) return false;
    selected_ = idx;
    return true;
}

bool Runtime::find_macro(const char* id, size_t& idx_out) const noexcept {
    if (!id || !*id) return false;
    for (size_t i = 0; i < macro_count_; ++i) {
        if (kernel::util::kstrcmp(macros_[i].id, id) == 0) {
            idx_out = i;
            return true;
        }
    }
    return false;
}

uint64_t Runtime::now_us() const noexcept {
    auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return tm ? tm->get_system_time_us() : 0;
}

int Runtime::resolve_axis(const char* axis_name) const noexcept {
    if (!axis_name || !*axis_name) return -1;
    switch (axis_name[0]) {
        case 'X': case 'x': return 0;
        case 'Y': case 'y': return 1;
        case 'Z': case 'z': return 2;
        case 'A': case 'a': return 3;
        default: return -1;
    }
}

bool Runtime::compare_symbol(const Step& step) const noexcept {
    switch (step.value_type) {
        case machine::SymbolType::Bool: {
            bool value = false;
            if (!machine::g_registry.get_bool(step.symbol, value)) return false;
            switch (step.compare) {
                case CompareOp::Equal: return value == step.value.bool_value;
                case CompareOp::NotEqual: return value != step.value.bool_value;
                default: return false;
            }
        }
        case machine::SymbolType::Int: {
            int32_t value = 0;
            if (!machine::g_registry.get_int(step.symbol, value)) return false;
            switch (step.compare) {
                case CompareOp::Equal: return value == step.value.int_value;
                case CompareOp::NotEqual: return value != step.value.int_value;
                case CompareOp::Less: return value < step.value.int_value;
                case CompareOp::LessEqual: return value <= step.value.int_value;
                case CompareOp::Greater: return value > step.value.int_value;
                case CompareOp::GreaterEqual: return value >= step.value.int_value;
            }
            return false;
        }
        case machine::SymbolType::Float: {
            float value = 0.0f;
            if (!machine::g_registry.get_float(step.symbol, value)) return false;
            switch (step.compare) {
                case CompareOp::Equal: return value == step.value.float_value;
                case CompareOp::NotEqual: return value != step.value.float_value;
                case CompareOp::Less: return value < step.value.float_value;
                case CompareOp::LessEqual: return value <= step.value.float_value;
                case CompareOp::Greater: return value > step.value.float_value;
                case CompareOp::GreaterEqual: return value >= step.value.float_value;
            }
            return false;
        }
    }
    return false;
}

bool Runtime::apply_step(size_t channel, const Step& step) noexcept {
    auto& st = channels_[channel];
    switch (step.kind) {
        case StepKind::Set:
            if (step.value_type == machine::SymbolType::Bool) return machine::g_registry.set_bool(step.symbol, step.value.bool_value);
            if (step.value_type == machine::SymbolType::Int) return machine::g_registry.set_int(step.symbol, step.value.int_value);
            return machine::g_registry.set_float(step.symbol, step.value.float_value);
        case StepKind::Delay:
            st.wait_deadline_us = now_us() + static_cast<uint64_t>(step.timeout_ms) * 1000ULL;
            return true;
        case StepKind::Wait:
            if (compare_symbol(step)) return true;
            if (st.wait_deadline_us == 0 && step.timeout_ms != 0) {
                st.wait_deadline_us = now_us() + static_cast<uint64_t>(step.timeout_ms) * 1000ULL;
            }
            return false;
        case StepKind::Move: {
            const int axis = resolve_axis(step.axis);
            if (axis < 0) return false;
            motion::g_motion.move_to(static_cast<size_t>(axis), step.alt_value.int_value);
            return true;
        }
        case StepKind::PodSelect:
            return machine::toolpods::g_service.select_station(step.pod_id, static_cast<size_t>(step.value.int_value));
        case StepKind::ToolAssign:
            return machine::toolpods::g_service.assign_virtual_tool(step.pod_id,
                static_cast<size_t>(step.value.int_value),
                static_cast<uint16_t>(step.alt_value.int_value));
        case StepKind::CaptureAxis: {
            const int axis = resolve_axis(step.axis);
            if (axis < 0 || !step.symbol[0]) return false;
            const int32_t actual =
                motion::g_motion.axis(static_cast<size_t>(axis)).actual_pos.load(std::memory_order_relaxed);
            return machine::g_registry.set_int(step.symbol, actual);
        }
        case StepKind::CaptureProbe: {
            const int axis = resolve_axis(step.axis);
            if (axis < 0 || !step.symbol[0]) return false;
            const int32_t actual_counts =
                motion::g_motion.axis(static_cast<size_t>(axis)).actual_pos.load(std::memory_order_relaxed);
            float radius_mm = 0.0f;
            (void)machine::g_registry.get_float("probe_stylus_radius_mm", radius_mm);
            const float surface_mm = static_cast<float>(actual_counts) * 0.01f + step.direction * radius_mm;
            if (step.value_type == machine::SymbolType::Float) {
                return machine::g_registry.set_float(step.symbol, surface_mm);
            }
            return machine::g_registry.set_int(step.symbol, static_cast<int32_t>(surface_mm * 100.0f));
        }
        case StepKind::Compute: {
            if (!step.symbol[0]) return false;
            float a = step.value.float_value;
            float b = step.alt_value.float_value;
            if (step.source[0] && !symbol_numeric_value(step.source, a)) return false;
            if (step.source_b[0] && !symbol_numeric_value(step.source_b, b)) return false;
            float result = a;
            switch (step.compute) {
                case ComputeOp::Copy: result = a; break;
                case ComputeOp::Add: result = a + b; break;
                case ComputeOp::Subtract: result = a - b; break;
                case ComputeOp::Multiply: result = a * b; break;
                case ComputeOp::Divide: result = (b != 0.0f) ? (a / b) : 0.0f; break;
                case ComputeOp::Average: result = (a + b) * 0.5f; break;
                case ComputeOp::None: default: return false;
            }
            result = result * step.scale + step.bias;
            return machine::g_registry.set_float(step.symbol, result);
        }
        case StepKind::SetWorkOffset: {
            if (!step.symbol[0]) return false;
            const int axis = resolve_axis(step.axis);
            if (axis < 0) return false;
            float correction = 0.0f;
            if (!symbol_numeric_value(step.symbol, correction)) return false;
            correction = correction * step.scale + step.bias;
            int32_t configured_work = -1;
            (void)machine::g_registry.get_int("probe_offset_work_index", configured_work);
            const size_t active_work = cnc::offsets::g_service.active_work();
            const size_t work = configured_work >= 0 &&
                                        configured_work < static_cast<int32_t>(cnc::offsets::WORK_OFFSET_COUNT)
                                    ? static_cast<size_t>(configured_work)
                                    : active_work;
            int32_t policy = 0;
            (void)machine::g_registry.get_int("probe_offset_policy", policy);
            const auto& table = cnc::offsets::g_service.work_offsets();
            const bool replace = policy == 1 || !step.relative;
            const float base = replace ? 0.0f : table[work].value.axis[static_cast<size_t>(axis)];
            return cnc::offsets::g_service.set_work_axis(work, static_cast<size_t>(axis), base + correction);
        }
        case StepKind::SphereCalibrate:
            return probe::g_runtime.start_reference_sphere();
        case StepKind::Alarm:
            kernel::util::k_snprintf(st.message, sizeof(st.message), "%s", step.message);
            machine::g_registry.set_bool("macro_fault", true);
            machine::g_registry.set_int("macro_fault_code", static_cast<int32_t>(st.macro_index + 1));
            st.fault = true;
            st.active = false;
            return false;
        case StepKind::None:
        default:
            return true;
    }
}

bool Runtime::start(size_t channel, size_t idx) noexcept {
    if (channel >= MAX_CHANNELS || idx >= macro_count_) return false;
    auto& st = channels_[channel];
    st = ChannelState{};
    st.active = true;
    st.macro_index = idx;
    machine::g_registry.set_bool("macro_busy", true);
    machine::g_registry.set_bool("macro_fault", false);
    machine::g_registry.set_int("macro_fault_code", 0);
    return true;
}

bool Runtime::start_by_id(size_t channel, const char* id) noexcept {
    size_t idx = 0;
    return find_macro(id, idx) && start(channel, idx);
}

bool Runtime::start_mcode(size_t channel, uint16_t mcode) noexcept {
    for (size_t i = 0; i < macro_count_; ++i) {
        if (macros_[i].mcode == mcode) return start(channel, i);
    }
    return false;
}

bool Runtime::stop(size_t channel) noexcept {
    if (channel >= MAX_CHANNELS) return false;
    channels_[channel].active = false;
    channels_[channel].fault = false;
    channels_[channel].wait_deadline_us = 0;
    machine::g_registry.set_bool("macro_busy", false);
    return true;
}

bool Runtime::tick_channel(size_t channel) noexcept {
    if (channel >= MAX_CHANNELS) return false;
    auto& st = channels_[channel];
    if (!st.active) return !st.fault;
    if (st.macro_index >= macro_count_) {
        st.fault = true;
        st.active = false;
        return false;
    }
    const auto& macro = macros_[st.macro_index];
    if (st.step_index >= macro.step_count) {
        st.active = false;
        st.wait_deadline_us = 0;
        machine::g_registry.set_bool("macro_busy", false);
        return true;
    }
    const Step& step = steps_[macro.first_step + st.step_index];
    if (step.kind == StepKind::Delay && st.wait_deadline_us != 0) {
        if (now_us() < st.wait_deadline_us) return true;
        st.wait_deadline_us = 0;
        ++st.step_index;
        return true;
    }
    if (step.kind == StepKind::Wait) {
        if (compare_symbol(step)) {
            st.wait_deadline_us = 0;
            ++st.step_index;
            return true;
        }
        if (st.wait_deadline_us != 0 && now_us() >= st.wait_deadline_us) {
            st.fault = true;
            st.active = false;
            kernel::util::k_snprintf(st.message, sizeof(st.message), "timeout waiting for %s", step.symbol);
            machine::g_registry.set_bool("macro_fault", true);
            machine::g_registry.set_int("macro_fault_code", static_cast<int32_t>(st.macro_index + 1));
            machine::g_registry.set_bool("macro_busy", false);
            return false;
        }
        (void)apply_step(channel, step);
        return true;
    }
    if (step.kind == StepKind::SphereCalibrate) {
        if (st.wait_deadline_us == 0) {
            if (!apply_step(channel, step)) {
                st.fault = true;
                st.active = false;
                machine::g_registry.set_bool("macro_fault", true);
                machine::g_registry.set_int("macro_fault_code", static_cast<int32_t>(st.macro_index + 1));
                kernel::util::k_snprintf(st.message, sizeof(st.message), "sphere calibration start failed");
                return false;
            }
            st.wait_deadline_us = 1;
            return true;
        }
        if (probe::g_runtime.active()) return true;
        st.wait_deadline_us = 0;
        if (probe::g_runtime.fault()) {
            st.fault = true;
            st.active = false;
            machine::g_registry.set_bool("macro_fault", true);
            machine::g_registry.set_int("macro_fault_code", static_cast<int32_t>(st.macro_index + 1));
            kernel::util::k_snprintf(st.message, sizeof(st.message), "%s", probe::g_runtime.message());
            return false;
        }
        kernel::util::k_snprintf(st.message, sizeof(st.message), "%s", probe::g_runtime.message());
        probe::g_runtime.clear_done();
        ++st.step_index;
        return true;
    }
    if (!apply_step(channel, step)) return !st.fault;
    ++st.step_index;
    return true;
}

void Runtime::tick() noexcept {
    machine::g_registry.sync_from_runtime();
    bool any_active = false;
    for (size_t channel = 0; channel < MAX_CHANNELS; ++channel) {
        (void)tick_channel(channel);
        any_active |= channels_[channel].active;
    }
    machine::g_registry.set_bool("macro_busy", any_active);
}

bool Runtime::channel_done(size_t channel) const noexcept {
    return channel < MAX_CHANNELS && !channels_[channel].active && !channels_[channel].fault;
}

bool Runtime::channel_fault(size_t channel) const noexcept {
    return channel < MAX_CHANNELS && channels_[channel].fault;
}

void Runtime::thread_entry(void*) {
    for (;;) {
        g_runtime.tick();
        auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
        if (tm) tm->wait_until_ns((tm->get_system_time_us() + 1000ULL) * 1000ULL);
    }
}

} // namespace macros
