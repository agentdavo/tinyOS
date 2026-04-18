// SPDX-License-Identifier: MIT OR Apache-2.0

#include "ladder_runtime.hpp"

#include "miniOS.hpp"
#include "util.hpp"

namespace ladder {

namespace {

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

const char* next_line(const char* p, const char* end, char* out, size_t out_size) {
    size_t i = 0;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    while (p < end && *p != '\r' && *p != '\n' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    while (p < end && *p != '\r' && *p != '\n') ++p;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    return p;
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

machine::SymbolValue parse_value(machine::SymbolType type, const char* s) {
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
            value.int_value = static_cast<int32_t>(value.float_value);
            value.bool_value = value.float_value != 0.0f;
            break;
    }
    return value;
}

} // namespace

Runtime g_runtime;

bool Runtime::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    rung_count_ = 0;
    const char* p = buf;
    const char* end = buf + len;
    char line[256];
    while (p < end) {
        p = next_line(p, end, line, sizeof(line));
        if (line[0] == '\0' || line[0] == '#') continue;
        char* rest = line;
        while (*rest && *rest != '\t') ++rest;
        if (*rest == '\t') *rest++ = '\0';
        char scratch_a[64];
        char scratch_b[64];
        char scratch_c[64];
        if (kernel::util::kstrcmp(line, "rung") != 0 || rung_count_ >= MAX_RUNGS) continue;
        auto& rung = rungs_[rung_count_++];
        const auto type = parse_type(field_value(rest, "type", scratch_a, sizeof(scratch_a)));
        rung.type = type;
        rung.compare = parse_compare(field_value(rest, "compare", scratch_b, sizeof(scratch_b)));
        kernel::util::k_snprintf(rung.id, sizeof(rung.id), "%s", field_value(rest, "id", scratch_c, sizeof(scratch_c)));
        kernel::util::k_snprintf(rung.lhs, sizeof(rung.lhs), "%s", field_value(rest, "lhs", scratch_a, sizeof(scratch_a)));
        kernel::util::k_snprintf(rung.rhs_symbol, sizeof(rung.rhs_symbol), "%s",
                                 field_value(rest, "rhs_symbol", scratch_b, sizeof(scratch_b)) ? field_value(rest, "rhs_symbol", scratch_b, sizeof(scratch_b)) : "");
        kernel::util::k_snprintf(rung.out, sizeof(rung.out), "%s", field_value(rest, "out", scratch_c, sizeof(scratch_c)));
        rung.rhs_value = parse_value(type, field_value(rest, "rhs", scratch_a, sizeof(scratch_a)));
        rung.true_value = parse_value(type, field_value(rest, "true", scratch_b, sizeof(scratch_b)));
        rung.false_value = parse_value(type, field_value(rest, "false", scratch_c, sizeof(scratch_c)));
    }
    return rung_count_ != 0;
}

bool Runtime::evaluate(const Rung& rung) const noexcept {
    if (rung.type == machine::SymbolType::Bool) {
        bool lhs = false;
        if (!machine::g_registry.get_bool(rung.lhs, lhs)) return false;
        bool rhs = rung.rhs_value.bool_value;
        if (rung.rhs_symbol[0] != '\0') (void)machine::g_registry.get_bool(rung.rhs_symbol, rhs);
        switch (rung.compare) {
            case CompareOp::Equal: return lhs == rhs;
            case CompareOp::NotEqual: return lhs != rhs;
            default: return false;
        }
    }
    if (rung.type == machine::SymbolType::Int) {
        int32_t lhs = 0;
        if (!machine::g_registry.get_int(rung.lhs, lhs)) return false;
        int32_t rhs = rung.rhs_value.int_value;
        if (rung.rhs_symbol[0] != '\0') (void)machine::g_registry.get_int(rung.rhs_symbol, rhs);
        switch (rung.compare) {
            case CompareOp::Equal: return lhs == rhs;
            case CompareOp::NotEqual: return lhs != rhs;
            case CompareOp::Less: return lhs < rhs;
            case CompareOp::LessEqual: return lhs <= rhs;
            case CompareOp::Greater: return lhs > rhs;
            case CompareOp::GreaterEqual: return lhs >= rhs;
        }
    }
    float lhs = 0.0f;
    if (!machine::g_registry.get_float(rung.lhs, lhs)) return false;
    float rhs = rung.rhs_value.float_value;
    if (rung.rhs_symbol[0] != '\0') (void)machine::g_registry.get_float(rung.rhs_symbol, rhs);
    switch (rung.compare) {
        case CompareOp::Equal: return lhs == rhs;
        case CompareOp::NotEqual: return lhs != rhs;
        case CompareOp::Less: return lhs < rhs;
        case CompareOp::LessEqual: return lhs <= rhs;
        case CompareOp::Greater: return lhs > rhs;
        case CompareOp::GreaterEqual: return lhs >= rhs;
    }
    return false;
}

void Runtime::drive_output(const Rung& rung, bool state) noexcept {
    const auto& value = state ? rung.true_value : rung.false_value;
    switch (rung.type) {
        case machine::SymbolType::Bool:
            (void)machine::g_registry.set_bool(rung.out, value.bool_value);
            break;
        case machine::SymbolType::Int:
            (void)machine::g_registry.set_int(rung.out, value.int_value);
            break;
        case machine::SymbolType::Float:
            (void)machine::g_registry.set_float(rung.out, value.float_value);
            break;
    }
}

void Runtime::tick() noexcept {
    machine::g_registry.sync_from_runtime();
    for (size_t i = 0; i < rung_count_; ++i) {
        drive_output(rungs_[i], evaluate(rungs_[i]));
    }
}

void Runtime::thread_entry(void*) {
    for (;;) {
        g_runtime.tick();
        auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
        if (tm) tm->wait_until_ns((tm->get_system_time_us() + 1000ULL) * 1000ULL);
    }
}

} // namespace ladder
