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

// MPG (manual pulse generator / handwheel) wiring. The motion kernel reads
// a 32-bit signed quadrature counter from the slave's TxPDO at `byte_offset`
// once per master cycle; deltas drive the operator-selected axis at
// `velocity_scale_cps_per_detent` counts/sec per detent. Returns false if
// the indices are out of range.
bool wire_mpg(uint8_t master_id,
              uint16_t slave_index,
              uint8_t byte_offset,
              uint8_t counts_per_detent = 4,
              kernel::hal::UARTDriverOps* uart = nullptr) noexcept;

// Look for an `ec.tx_s32` signal binding named `signal_name` (default
// "mpg_handwheel") in the registry and apply it to the motion kernel.
// Lets `devices/embedded_signals.tsv` declare the wiring instead of
// hard-coding it. Returns false if no matching binding is found or the
// binding's slave index is out of range.
bool wire_mpg_from_signal(const char* signal_name = nullptr,
                          kernel::hal::UARTDriverOps* uart = nullptr) noexcept;

} // namespace machine::wiring
