// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "core.hpp"

#include <cstddef>
#include <cstdint>

namespace config { struct Record; }

namespace machine::topology {

namespace detail {
void record_cb(const config::Record& rec, void* raw_ctx) noexcept;
}

enum class Role : uint8_t {
    Unknown = 0,
    Servo,
    Coupler,
    DigitalInput,
    DigitalOutput,
    AnalogInput,
    AnalogOutput,
    Encoder,
    Probe,
    Sensor,
};

struct Binding {
    bool used = false;
    Role role = Role::Unknown;
    uint8_t master_id = 0;
    uint8_t slave_index = 0;
    uint8_t axis_index = 0xFF;
    uint8_t channel = 0;
    char name[24]{};
    char axis_name[8]{};
};

class Service {
public:
    static constexpr size_t MAX_BINDINGS = 64;

    bool load_tsv(const char* buf, size_t len) noexcept;
    size_t binding_count() const noexcept;
    const Binding* binding(size_t idx) const noexcept;
    bool has_servo_bindings() const noexcept;
    const Binding* servo_binding_for_axis(size_t axis_idx) const noexcept;
    const Binding* servo_binding_for_slave(uint8_t master_id, size_t slave_index) const noexcept;
    int axis_for_slave(uint8_t master_id, size_t slave_index) const noexcept;

private:
    friend void detail::record_cb(const config::Record& rec, void* raw_ctx) noexcept;
    void reset() noexcept;

    Binding bindings_[MAX_BINDINGS]{};
    size_t count_ = 0;
    mutable kernel::core::Spinlock lock_;
};

extern Service g_service;

} // namespace machine::topology
