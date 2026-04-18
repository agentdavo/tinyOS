// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>

namespace devices { struct DeviceEntry; }
namespace ethercat { class Master; }
namespace kernel::hal { struct UARTDriverOps; }

namespace machine::wiring {

const devices::DeviceEntry* default_servo_device() noexcept;
void resolve_servo_limits(const devices::DeviceEntry* dev,
                          int32_t& vmax,
                          int32_t& accel) noexcept;
bool hook_servo_axis(ethercat::Master& master,
                     size_t slave_index,
                     size_t axis_index,
                     const devices::DeviceEntry* dev = nullptr) noexcept;
size_t wire_motion_axes_from_topology(kernel::hal::UARTDriverOps* uart = nullptr) noexcept;

} // namespace machine::wiring
