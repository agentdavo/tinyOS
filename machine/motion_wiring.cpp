// SPDX-License-Identifier: MIT OR Apache-2.0

#include "motion_wiring.hpp"

#include "devices/device_db.hpp"
#include "ethercat/master.hpp"
#include "machine_topology.hpp"
#include "motion/motion.hpp"
#include "util.hpp"

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

} // namespace machine::wiring
