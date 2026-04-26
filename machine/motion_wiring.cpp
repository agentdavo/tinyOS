// SPDX-License-Identifier: MIT OR Apache-2.0

#include "motion_wiring.hpp"

#include "devices/device_db.hpp"
#include "ethercat/master.hpp"
#include "machine_registry.hpp"
#include "machine_topology.hpp"
#include "motion/motion.hpp"
#include "util.hpp"

#include <cstring>

namespace machine::wiring {

const devices::DeviceEntry* default_servo_device() noexcept {
    for (size_t i = 0; i < devices::g_device_db.device_count(); ++i) {
        const auto* dev = devices::g_device_db.at(i);
        if (dev && dev->type == devices::DeviceType::Servo) return dev;
    }
    return nullptr;
}

void resolve_servo_limits(const devices::DeviceEntry* dev,
                          int32_t& vmax,
                          int32_t& accel) noexcept {
    vmax = 0;
    accel = 0;
    if (!dev) dev = default_servo_device();
    if (!dev) return;

    uint64_t value = 0;
    if (devices::g_device_db.get_od_default(dev->id, 0x6081, 0, value)) {
        vmax = static_cast<int32_t>(value);
    }
    if (devices::g_device_db.get_od_default(dev->id, 0x6083, 0, value)) {
        accel = static_cast<int32_t>(value);
    }
}

bool hook_servo_axis(ethercat::Master& master,
                     size_t slave_index,
                     size_t axis_index,
                     const devices::DeviceEntry* dev) noexcept {
    if (axis_index >= motion::MAX_AXES || slave_index >= ethercat::MAX_SLAVES) return false;
    int32_t vmax = 0;
    int32_t accel = 0;
    resolve_servo_limits(dev, vmax, accel);
    auto& slave = master.slave(slave_index);
    motion::g_motion.axis(axis_index).hook_drive(&slave.drive, slave.station_addr, vmax, accel);
    return true;
}

size_t wire_motion_axes_from_topology(kernel::hal::UARTDriverOps* uart) noexcept {
    for (size_t axis = 0; axis < motion::MAX_AXES; ++axis) {
        motion::g_motion.axis(axis).hook_drive(nullptr, 0, 0, 0);
    }

    const auto* default_dev = default_servo_device();
    int32_t vmax = 0;
    int32_t accel = 0;
    resolve_servo_limits(default_dev, vmax, accel);

    if (uart) {
        char buf[112];
        kernel::util::k_snprintf(
            buf, sizeof(buf),
            "[motion] default servo limits: vmax=%ld accel=%ld (0=compiled-default)\n",
            static_cast<long>(vmax), static_cast<long>(accel));
        uart->puts(buf);
    }

    size_t wired = 0;
    for (size_t i = 0; i < machine::topology::g_service.binding_count(); ++i) {
        const auto* binding = machine::topology::g_service.binding(i);
        if (!binding || !binding->used || binding->role != machine::topology::Role::Servo) continue;
        ethercat::Master& master = (binding->master_id == 0) ? ethercat::g_master_a : ethercat::g_master_b;
        if (hook_servo_axis(master, binding->slave_index, binding->axis_index, default_dev)) {
            ++wired;
        }
    }

    if (uart) {
        char buf[112];
        kernel::util::k_snprintf(
            buf, sizeof(buf),
            "[motion] topology servo bindings applied: %zu\n",
            wired);
        uart->puts(buf);
    }
    return wired;
}

bool wire_mpg(uint8_t master_id,
              uint16_t slave_index,
              uint8_t byte_offset,
              uint8_t counts_per_detent,
              kernel::hal::UARTDriverOps* uart) noexcept {
    if (master_id > 1 || slave_index >= ethercat::MAX_SLAVES) return false;
    if (counts_per_detent == 0) counts_per_detent = 4;
    motion::g_motion.wire_mpg_source(master_id, slave_index, byte_offset,
                                     counts_per_detent);
    if (uart) {
        char buf[112];
        kernel::util::k_snprintf(
            buf, sizeof(buf),
            "[motion] mpg bound: master=%u slave=%u off=%u cpd=%u\n",
            (unsigned)master_id, (unsigned)slave_index,
            (unsigned)byte_offset, (unsigned)counts_per_detent);
        uart->puts(buf);
    }
    return true;
}

bool wire_mpg_from_signal(const char* signal_name,
                          kernel::hal::UARTDriverOps* uart) noexcept {
    const char* want = (signal_name && *signal_name) ? signal_name : "mpg_handwheel";
    for (size_t i = 0; i < machine::g_registry.signal_binding_count(); ++i) {
        const auto* b = machine::g_registry.signal_binding(i);
        if (!b || !b->used) continue;
        if (b->source != machine::Registry::SignalSource::EcTxS32) continue;
        if (std::strncmp(b->name, want, machine::Registry::MAX_NAME_LEN) != 0) continue;
        // The TSV `offset` field carries a *bit* offset for tx_s32 (see
        // machine_registry::read_process_bits); MPG only supports byte-
        // aligned encoder slots, so reject misalignment loudly here
        // rather than silently producing rotated counters at runtime.
        if ((b->offset & 7u) != 0) {
            if (uart) uart->puts("[motion] mpg signal offset is not byte-aligned\n");
            return false;
        }
        const uint8_t byte_off = static_cast<uint8_t>(b->offset / 8u);
        return wire_mpg(b->master, b->slave, byte_off, /*counts_per_detent=*/4, uart);
    }
    if (uart) {
        char buf[112];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "[motion] no ec.tx_s32 signal named '%s' found\n", want);
        uart->puts(buf);
    }
    return false;
}

} // namespace machine::wiring
