// SPDX-License-Identifier: MIT OR Apache-2.0

#include "machine_topology.hpp"

#include "config/tsv.hpp"

namespace machine::topology {

namespace detail {

bool eq(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

void copy_sv(char* dst, size_t cap, std::string_view src) noexcept {
    if (!dst || cap == 0) return;
    size_t n = src.size();
    if (n >= cap) n = cap - 1;
    for (size_t i = 0; i < n; ++i) dst[i] = src[i];
    dst[n] = '\0';
}

Role parse_role(std::string_view value) noexcept {
    if (eq(value, "servo")) return Role::Servo;
    if (eq(value, "coupler")) return Role::Coupler;
    if (eq(value, "din") || eq(value, "digital_input")) return Role::DigitalInput;
    if (eq(value, "dout") || eq(value, "digital_output")) return Role::DigitalOutput;
    if (eq(value, "ain") || eq(value, "analog_input")) return Role::AnalogInput;
    if (eq(value, "aout") || eq(value, "analog_output")) return Role::AnalogOutput;
    if (eq(value, "encoder")) return Role::Encoder;
    if (eq(value, "probe")) return Role::Probe;
    if (eq(value, "sensor")) return Role::Sensor;
    return Role::Unknown;
}

bool parse_master(std::string_view value, uint8_t& out) noexcept {
    if (eq(value, "0") || eq(value, "a") || eq(value, "ec_a")) {
        out = 0;
        return true;
    }
    if (eq(value, "1") || eq(value, "b") || eq(value, "ec_b")) {
        out = 1;
        return true;
    }
    return false;
}

bool parse_axis(std::string_view value, uint8_t& out) noexcept {
    if (value.empty()) return false;
    if (value.size() == 1 && value[0] >= '0' && value[0] <= '9') {
        out = static_cast<uint8_t>(value[0] - '0');
        return true;
    }
    uint64_t numeric = 0;
    bool numeric_ok = true;
    for (char c : value) {
        if (c < '0' || c > '9') {
            numeric_ok = false;
            break;
        }
        numeric = numeric * 10u + static_cast<uint64_t>(c - '0');
    }
    if (numeric_ok && numeric <= 0xFFu) {
        out = static_cast<uint8_t>(numeric);
        return true;
    }

    switch (value[0]) {
        case 'X': case 'x': out = 0; return true;
        case 'Y': case 'y': out = 1; return true;
        case 'Z': case 'z': out = 2; return true;
        case 'A': case 'a': out = 3; return true;
        case 'B': case 'b': out = 17; return true;
        case 'C': case 'c': out = 16; return true;
        default: return false;
    }
}

struct LoadCtx {
    Service* service = nullptr;
    bool ok = true;
};

void record_cb(const config::Record& rec, void* raw_ctx) noexcept {
    auto* ctx = static_cast<LoadCtx*>(raw_ctx);
    if (!ctx || !ctx->service || !eq(rec.type, "binding")) return;

    auto& service = *ctx->service;
    if (service.count_ >= Service::MAX_BINDINGS) {
        ctx->ok = false;
        return;
    }

    const auto role_sv = rec.get("role");
    const Role role = parse_role(role_sv);
    uint32_t slave_u32 = 0;
    uint32_t channel_u32 = 0;
    uint8_t master_id = 0;

    if (role == Role::Unknown || !parse_master(rec.get("master"), master_id) ||
        !rec.get_u32("slave", slave_u32) || slave_u32 > 0xFFu) {
        ctx->ok = false;
        return;
    }

    Binding candidate{};
    candidate.used = true;
    candidate.role = role;
    candidate.master_id = master_id;
    candidate.slave_index = static_cast<uint8_t>(slave_u32);
    candidate.axis_index = 0xFF;
    if (rec.get_u32("channel", channel_u32) && channel_u32 <= 0xFFu) {
        candidate.channel = static_cast<uint8_t>(channel_u32);
    }
    copy_sv(candidate.name, sizeof(candidate.name), rec.get("name"));
    copy_sv(candidate.axis_name, sizeof(candidate.axis_name), rec.get("axis_name"));

    const auto axis_sv = rec.get("axis");
    if (!axis_sv.empty()) {
        uint8_t axis = 0xFF;
        if (!parse_axis(axis_sv, axis)) {
            ctx->ok = false;
            return;
        }
        candidate.axis_index = axis;
        if (candidate.axis_name[0] == '\0') {
            copy_sv(candidate.axis_name, sizeof(candidate.axis_name), axis_sv);
        }
    }

    if (candidate.role == Role::Servo && candidate.axis_index == 0xFF) {
        ctx->ok = false;
        return;
    }

    for (size_t i = 0; i < service.count_; ++i) {
        const auto& existing = service.bindings_[i];
        if (!existing.used) continue;
        if (existing.master_id == candidate.master_id &&
            existing.slave_index == candidate.slave_index) {
            ctx->ok = false;
            return;
        }
        if (existing.role == Role::Servo && candidate.role == Role::Servo &&
            existing.axis_index == candidate.axis_index) {
            ctx->ok = false;
            return;
        }
    }

    service.bindings_[service.count_++] = candidate;
}

} // namespace detail

Service g_service;

void Service::reset() noexcept {
    count_ = 0;
    for (auto& binding : bindings_) binding = Binding{};
}

bool Service::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    kernel::core::ScopedLock lock(lock_);
    reset();
    detail::LoadCtx ctx{this, true};
    const bool parsed = config::parse(buf, len, &detail::record_cb, &ctx);
    if (!parsed || !ctx.ok) {
        reset();
        return false;
    }
    return true;
}

size_t Service::binding_count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return count_;
}

const Binding* Service::binding(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return idx < count_ ? &bindings_[idx] : nullptr;
}

bool Service::has_servo_bindings() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (bindings_[i].used && bindings_[i].role == Role::Servo) return true;
    }
    return false;
}

const Binding* Service::servo_binding_for_axis(size_t axis_idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (!bindings_[i].used || bindings_[i].role != Role::Servo) continue;
        if (bindings_[i].axis_index == axis_idx) return &bindings_[i];
    }
    return nullptr;
}

const Binding* Service::servo_binding_for_slave(uint8_t master_id, size_t slave_index) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (!bindings_[i].used || bindings_[i].role != Role::Servo) continue;
        if (bindings_[i].master_id == master_id && bindings_[i].slave_index == slave_index) {
            return &bindings_[i];
        }
    }
    return nullptr;
}

int Service::axis_for_slave(uint8_t master_id, size_t slave_index) const noexcept {
    const Binding* found = servo_binding_for_slave(master_id, slave_index);
    return found ? static_cast<int>(found->axis_index) : -1;
}

} // namespace machine::topology
