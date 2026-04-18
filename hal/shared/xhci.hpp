// SPDX-License-Identifier: MIT OR Apache-2.0

#ifndef HAL_SHARED_XHCI_HPP
#define HAL_SHARED_XHCI_HPP

#include "hal.hpp"
#include "hal/shared/pci.hpp"
#include <cstddef>
#include <cstdint>

namespace hal::shared::xhci {

struct TRB {
    uint64_t parameter = 0;
    uint32_t status = 0;
    uint32_t control = 0;
};

struct ERSTEntry {
    uint64_t ring_segment_base = 0;
    uint32_t ring_segment_size = 0;
    uint32_t reserved = 0;
};

struct SetupPacket {
    uint8_t request_type = 0;
    uint8_t request = 0;
    uint16_t value = 0;
    uint16_t index = 0;
    uint16_t length = 0;
} __attribute__((packed));

struct DeviceDescriptor {
    uint8_t length = 0;
    uint8_t descriptor_type = 0;
    uint16_t bcd_usb = 0;
    uint8_t device_class = 0;
    uint8_t device_subclass = 0;
    uint8_t device_protocol = 0;
    uint8_t max_packet_size0 = 0;
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uint16_t bcd_device = 0;
    uint8_t manufacturer = 0;
    uint8_t product = 0;
    uint8_t serial_number = 0;
    uint8_t num_configurations = 0;
} __attribute__((packed));

struct ConfigurationDescriptor {
    uint8_t length = 0;
    uint8_t descriptor_type = 0;
    uint16_t total_length = 0;
    uint8_t num_interfaces = 0;
    uint8_t configuration_value = 0;
    uint8_t configuration = 0;
    uint8_t attributes = 0;
    uint8_t max_power = 0;
} __attribute__((packed));

struct InterfaceDescriptor {
    uint8_t length = 0;
    uint8_t descriptor_type = 0;
    uint8_t interface_number = 0;
    uint8_t alternate_setting = 0;
    uint8_t num_endpoints = 0;
    uint8_t interface_class = 0;
    uint8_t interface_subclass = 0;
    uint8_t interface_protocol = 0;
    uint8_t interface = 0;
} __attribute__((packed));

struct EndpointDescriptor {
    uint8_t length = 0;
    uint8_t descriptor_type = 0;
    uint8_t endpoint_address = 0;
    uint8_t attributes = 0;
    uint16_t max_packet_size = 0;
    uint8_t interval = 0;
} __attribute__((packed));

struct ControllerState {
    static constexpr size_t COMMAND_RING_TRBS = 16;
    static constexpr size_t EVENT_RING_TRBS = 16;
    static constexpr size_t CONTROL_RING_TRBS = 16;
    static constexpr size_t INTERRUPT_RING_TRBS = 16;
    static constexpr size_t DCBAA_ENTRIES = 256;
    static constexpr size_t MAX_SCRATCHPAD_BUFFERS = 8;
    static constexpr size_t CONTEXT_BYTES = 2048;

    pci::HostBridge host{};
    pci::DeviceInfo pci{};
    uintptr_t mmio_base = 0;
    uintptr_t op_base = 0;
    uint8_t cap_length = 0;
    uint16_t hci_version = 0;
    uint32_t max_slots = 0;
    uint32_t max_ports = 0;
    uint32_t max_scratchpad_buffers = 0;
    uint32_t context_size = 32;
    uint32_t doorbell_offset = 0;
    uint32_t runtime_offset = 0;
    uint32_t pagesize = 0;
    uint8_t command_cycle = 1;
    uint8_t event_cycle = 1;
    uint8_t control_cycle = 1;
    uint8_t last_command_completion = 0;
    uint8_t command_ring_index = 0;
    uint8_t event_ring_index = 0;
    uint8_t control_ring_index = 0;
    uint8_t slot_id = 0;
    uint8_t slot_speed = 0;
    uint8_t observed_port = 0;
    uint8_t addressed_port = 0;
    uint8_t keyboard_config_value = 0;
    uint8_t keyboard_interface_number = 0;
    uint8_t keyboard_endpoint_address = 0;
    uint8_t keyboard_endpoint_dci = 0;
    uint8_t keyboard_interval = 0;
    uint8_t interrupt_ring_index = 0;
    uint8_t interrupt_cycle = 1;
    DeviceDescriptor device_descriptor{};
    uint16_t keyboard_max_packet = 0;
    uintptr_t keyboard_transfer_trb = 0;
    uint32_t keyboard_report_count = 0;
    uint32_t transfer_event_count = 0;
    uint32_t port_change_event_count = 0;
    uint8_t keyboard_last_report_len = 0;
    uint8_t keyboard_last_completion = 0;
    bool command_ring_ready = false;
    bool control_ring_ready = false;
    bool controller_running = false;
    bool present = false;
    bool initialized = false;
    bool device_addressed = false;
    bool descriptor_read = false;
    bool keyboard_candidate = false;
    bool keyboard_configured = false;
    bool keyboard_poll_pending = false;
    bool keyboard_connected = false;
    bool key_states[128]{};

    alignas(64) TRB command_ring[COMMAND_RING_TRBS]{};
    alignas(64) TRB event_ring[EVENT_RING_TRBS]{};
    alignas(64) TRB control_ring[CONTROL_RING_TRBS]{};
    alignas(64) TRB interrupt_ring[INTERRUPT_RING_TRBS]{};
    alignas(64) ERSTEntry erst[1]{};
    alignas(64) uint64_t dcbaa[DCBAA_ENTRIES]{};
    alignas(64) uint64_t scratchpad_ptrs[MAX_SCRATCHPAD_BUFFERS]{};
    alignas(4096) uint8_t scratchpad_pages[MAX_SCRATCHPAD_BUFFERS][4096]{};
    alignas(64) uint8_t device_context[CONTEXT_BYTES]{};
    alignas(64) uint8_t input_context[CONTEXT_BYTES]{};
    alignas(64) uint8_t control_buffer[256]{};
    alignas(64) uint8_t config_buffer[512]{};
    alignas(64) uint8_t keyboard_report[8]{};
    alignas(64) uint8_t keyboard_prev_report[8]{};
    SetupPacket setup_packet{};
};

bool init_from_pci(const pci::HostBridge& host,
                   ControllerState& state,
                   kernel::hal::usb::ControllerInfo& info);
void refresh_info(ControllerState& state,
                  kernel::hal::usb::ControllerInfo& info);
void poll(ControllerState& state);
bool keyboard_connected(const ControllerState& state);
bool key_pressed(const ControllerState& state, uint8_t key);
bool get_port_status(const ControllerState& state,
                     uint32_t port_index,
                     kernel::hal::usb::PortStatus& out);
bool set_port_power(ControllerState& state, uint32_t port_index, bool on);
bool reset_port(ControllerState& state, uint32_t port_index);

} // namespace hal::shared::xhci

#endif // HAL_SHARED_XHCI_HPP
