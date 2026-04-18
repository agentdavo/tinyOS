// SPDX-License-Identifier: MIT OR Apache-2.0

#ifndef MINIOS_USB_HPP
#define MINIOS_USB_HPP

#include "hal.hpp"

namespace kernel::usb {

bool init(kernel::hal::USBHostControllerOps* ops) noexcept;
bool initialized() noexcept;
kernel::hal::usb::ControllerInfo controller_info() noexcept;
bool port_status(uint32_t one_based_port, kernel::hal::usb::PortStatus& out) noexcept;
bool set_port_power(uint32_t one_based_port, bool on) noexcept;
bool reset_port(uint32_t one_based_port) noexcept;
void dump_status(kernel::hal::UARTDriverOps* uart) noexcept;

} // namespace kernel::usb

#endif // MINIOS_USB_HPP
