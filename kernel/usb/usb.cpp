// SPDX-License-Identifier: MIT OR Apache-2.0

#include "usb.hpp"
#include "miniOS.hpp"
#include "util.hpp"

namespace kernel::usb {
namespace {

kernel::hal::USBHostControllerOps* g_usb_ops = nullptr;
kernel::hal::usb::ControllerInfo g_info{};
bool g_initialized = false;

void ensure_initialized() noexcept {
    if (g_initialized || !kernel::g_platform) return;
    auto* volatile stable_ops = g_usb_ops ? g_usb_ops : kernel::g_platform->get_usb_ops();
    auto* ops = stable_ops;
    if (!ops) return;
    g_usb_ops = ops;
    if (!ops->init()) return;
    g_usb_ops = ops;
    g_info = stable_ops->get_info();
    g_initialized = g_info.present && g_info.initialized;
}

const char* controller_kind_name(kernel::hal::usb::ControllerKind kind) noexcept {
    switch (kind) {
        case kernel::hal::usb::ControllerKind::UHCI: return "uhci";
        case kernel::hal::usb::ControllerKind::OHCI: return "ohci";
        case kernel::hal::usb::ControllerKind::EHCI: return "ehci";
        case kernel::hal::usb::ControllerKind::XHCI: return "xhci";
        case kernel::hal::usb::ControllerKind::Unknown: break;
    }
    return "unknown";
}

const char* port_speed_name(kernel::hal::usb::PortSpeed speed) noexcept {
    switch (speed) {
        case kernel::hal::usb::PortSpeed::Low: return "low";
        case kernel::hal::usb::PortSpeed::Full: return "full";
        case kernel::hal::usb::PortSpeed::High: return "high";
        case kernel::hal::usb::PortSpeed::Super: return "super";
        case kernel::hal::usb::PortSpeed::Unknown: break;
    }
    return "unknown";
}

} // namespace

bool init(kernel::hal::USBHostControllerOps* ops) noexcept {
    auto* volatile stable_ops = ops;
    g_usb_ops = ops;
    g_info = {};
    g_initialized = false;
    if (!stable_ops) return false;
    if (!stable_ops->init()) return false;
    g_usb_ops = ops;
    g_info = stable_ops->get_info();
    g_initialized = g_info.present && g_info.initialized;
    return g_initialized;
}

bool initialized() noexcept {
    ensure_initialized();
    return g_initialized;
}

kernel::hal::usb::ControllerInfo controller_info() noexcept {
    ensure_initialized();
    auto* volatile stable_ops = g_usb_ops;
    if (stable_ops) g_info = stable_ops->get_info();
    return g_info;
}

bool port_status(uint32_t one_based_port, kernel::hal::usb::PortStatus& out) noexcept {
    ensure_initialized();
    if (!g_usb_ops || one_based_port == 0) return false;
    return g_usb_ops->get_port_status(one_based_port - 1, out);
}

bool set_port_power(uint32_t one_based_port, bool on) noexcept {
    ensure_initialized();
    if (!g_usb_ops || one_based_port == 0) return false;
    return g_usb_ops->set_port_power(one_based_port - 1, on);
}

bool reset_port(uint32_t one_based_port) noexcept {
    ensure_initialized();
    if (!g_usb_ops || one_based_port == 0) return false;
    return g_usb_ops->reset_port(one_based_port - 1);
}

void dump_status(kernel::hal::UARTDriverOps* uart) noexcept {
    if (!uart) return;
    const auto info = controller_info();
    if (!g_usb_ops || !info.present) {
        uart->puts("usb: unavailable\n");
        return;
    }

    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "usb: %s driver=%s transport=%s ports=%lu hotplug=%s qemu=%s enum=%s\n",
                             controller_kind_name(info.kind),
                             info.driver_name ? info.driver_name : "unknown",
                             info.transport ? info.transport : "unknown",
                             static_cast<unsigned long>(info.port_count),
                             info.hotplug_capable ? "yes" : "no",
                             info.qemu_virtual ? "yes" : "no",
                             info.enum_state ? info.enum_state : "none");
    uart->puts(buf);

    if (info.active_port != 0 || info.device_addressed || info.device_descriptor_read) {
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "  device port=%lu addressed=%s desc=%s vid=%04x pid=%04x class=%02x kbd=%s reports=%lu last_len=%u ep=%u cc=%u pending=%s tev=%lu pcev=%lu\n",
                                 static_cast<unsigned long>(info.active_port),
                                 info.device_addressed ? "yes" : "no",
                                 info.device_descriptor_read ? "yes" : "no",
                                 static_cast<unsigned>(info.device_vendor_id),
                                 static_cast<unsigned>(info.device_product_id),
                                 static_cast<unsigned>(info.device_class),
                                 info.keyboard_connected ? "yes" : "no",
                                 static_cast<unsigned long>(info.keyboard_reports),
                                 static_cast<unsigned>(info.keyboard_last_report_len),
                                 static_cast<unsigned>(info.keyboard_endpoint_dci),
                                 static_cast<unsigned>(info.keyboard_last_completion),
                                 info.keyboard_poll_pending ? "yes" : "no",
                                 static_cast<unsigned long>(info.transfer_events),
                                 static_cast<unsigned long>(info.port_change_events));
        uart->puts(buf);
    }

    for (uint32_t port = 1; port <= info.port_count; ++port) {
        kernel::hal::usb::PortStatus status{};
        if (!port_status(port, status)) continue;
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "  port%lu power=%s conn=%s en=%s speed=%s reset=%s overcur=%s\n",
                                 static_cast<unsigned long>(port),
                                 status.powered ? "on" : "off",
                                 status.connected ? "yes" : "no",
                                 status.enabled ? "yes" : "no",
                                 port_speed_name(status.speed),
                                 status.reset_in_progress ? "yes" : "no",
                                 status.over_current ? "yes" : "no");
        uart->puts(buf);
    }
}

} // namespace kernel::usb
