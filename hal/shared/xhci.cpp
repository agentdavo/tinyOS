// SPDX-License-Identifier: MIT OR Apache-2.0

#include "hal/shared/xhci.hpp"
#include "miniOS.hpp"
#include "util.hpp"

namespace hal::shared::xhci {
namespace {

constexpr uint8_t XHCI_CLASS = 0x0c;
constexpr uint8_t XHCI_SUBCLASS = 0x03;
constexpr uint8_t XHCI_PROGIF = 0x30;
constexpr uint16_t QEMU_PCI_VENDOR = 0x1b36;
constexpr uint16_t QEMU_XHCI_DEVICE = 0x000d;

constexpr uint32_t XHCI_HCSPARAMS1 = 0x04;
constexpr uint32_t XHCI_HCSPARAMS2 = 0x08;
constexpr uint32_t XHCI_HCCPARAMS1 = 0x10;
constexpr uint32_t XHCI_DBOFF = 0x14;
constexpr uint32_t XHCI_RTSOFF = 0x18;

constexpr uint32_t XHCI_USBCMD = 0x00;
constexpr uint32_t XHCI_USBSTS = 0x04;
constexpr uint32_t XHCI_PAGESIZE = 0x08;
constexpr uint32_t XHCI_DNCTRL = 0x14;
constexpr uint32_t XHCI_CRCR = 0x18;
constexpr uint32_t XHCI_DCBAAP = 0x30;
constexpr uint32_t XHCI_CONFIG = 0x38;

constexpr uint32_t XHCI_USBCMD_RS = 1u << 0;
constexpr uint32_t XHCI_USBCMD_HCRST = 1u << 1;
constexpr uint32_t XHCI_USBCMD_INTE = 1u << 2;
constexpr uint32_t XHCI_USBSTS_HCH = 1u << 0;
constexpr uint32_t XHCI_USBSTS_EINT = 1u << 3;
constexpr uint32_t XHCI_USBSTS_CNR = 1u << 11;

constexpr uint32_t XHCI_IMAN = 0x00;
constexpr uint32_t XHCI_IMOD = 0x04;
constexpr uint32_t XHCI_ERSTSZ = 0x08;
constexpr uint32_t XHCI_ERSTBA = 0x10;
constexpr uint32_t XHCI_ERDP = 0x18;
constexpr uint32_t XHCI_INTERRUPTER0 = 0x20;
constexpr uint64_t XHCI_ERDP_EHB = 1ULL << 3;
constexpr uint32_t XHCI_IMAN_IE = 1u << 1;

constexpr uint32_t XHCI_PORTSC_BASE = 0x400;
constexpr uint32_t XHCI_PORTSC_STRIDE = 0x10;

constexpr uint32_t PORTSC_CCS = 1u << 0;
constexpr uint32_t PORTSC_PED = 1u << 1;
constexpr uint32_t PORTSC_OCA = 1u << 3;
constexpr uint32_t PORTSC_PR  = 1u << 4;
constexpr uint32_t PORTSC_PP  = 1u << 9;
constexpr uint32_t PORTSC_SPEED_SHIFT = 10;
constexpr uint32_t PORTSC_SPEED_MASK  = 0x0fu << PORTSC_SPEED_SHIFT;
constexpr uint32_t PORTSC_RW1C_MASK =
    (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) |
    (1u << 21) | (1u << 22) | (1u << 23);

constexpr uint32_t TRB_TYPE_SHIFT = 10;
constexpr uint32_t TRB_TYPE_MASK = 0x3fu << TRB_TYPE_SHIFT;
constexpr uint32_t TRB_TYPE_LINK = 6u;
constexpr uint32_t TRB_TYPE_ENABLE_SLOT_CMD = 9u;
constexpr uint32_t TRB_TYPE_ADDRESS_DEVICE_CMD = 11u;
constexpr uint32_t TRB_TYPE_CONFIGURE_ENDPOINT_CMD = 12u;
constexpr uint32_t TRB_TYPE_NOOP_CMD = 23u;
constexpr uint32_t TRB_TYPE_NORMAL = 1u;
constexpr uint32_t TRB_TYPE_SETUP_STAGE = 2u;
constexpr uint32_t TRB_TYPE_DATA_STAGE = 3u;
constexpr uint32_t TRB_TYPE_STATUS_STAGE = 4u;
constexpr uint32_t TRB_TYPE_TRANSFER_EVENT = 32u;
constexpr uint32_t TRB_TYPE_COMMAND_COMPLETION_EVENT = 33u;
constexpr uint32_t TRB_TYPE_PORT_STATUS_CHANGE_EVENT = 34u;
constexpr uint32_t TRB_CYCLE = 1u << 0;
constexpr uint32_t TRB_LINK_TOGGLE_CYCLE = 1u << 1;
constexpr uint32_t TRB_ENT = 1u << 1;
constexpr uint32_t TRB_ISP = 1u << 2;
constexpr uint32_t TRB_CHAIN = 1u << 4;
constexpr uint32_t TRB_IOC = 1u << 5;
constexpr uint32_t TRB_IDT = 1u << 6;
constexpr uint32_t TRB_COMPLETION_CODE_SHIFT = 24;
constexpr uint32_t TRB_COMPLETION_CODE_MASK = 0xffu << TRB_COMPLETION_CODE_SHIFT;
constexpr uint32_t TRB_TRANSFER_LEN_MASK = (1u << 17) - 1u;
constexpr uint32_t TRB_DIR_IN = 1u << 16;
constexpr uint32_t TRB_TRT_SHIFT = 16;
constexpr uint32_t TRB_TRT_NONE = 0u;
constexpr uint32_t TRB_TRT_OUT = 2u;
constexpr uint32_t TRB_TRT_IN = 3u;
constexpr uint32_t TRB_BSR = 1u << 9;
constexpr uint32_t TRB_SLOT_ID_SHIFT = 24;
constexpr uint32_t TRB_SLOT_ID_MASK = 0xffu << TRB_SLOT_ID_SHIFT;
constexpr uint32_t TRB_ENDPOINT_ID_SHIFT = 16;
constexpr uint32_t TRB_ENDPOINT_ID_MASK = 0x1fu << TRB_ENDPOINT_ID_SHIFT;
constexpr uint8_t TRB_COMPLETION_SUCCESS = 1;
constexpr uint8_t USB_DESCRIPTOR_TYPE_DEVICE = 1;
constexpr uint8_t USB_DESCRIPTOR_TYPE_CONFIGURATION = 2;
constexpr uint8_t USB_DESCRIPTOR_TYPE_INTERFACE = 4;
constexpr uint8_t USB_DESCRIPTOR_TYPE_ENDPOINT = 5;
constexpr uint8_t USB_REQUEST_GET_DESCRIPTOR = 6;
constexpr uint8_t USB_REQUEST_SET_CONFIGURATION = 9;
constexpr uint8_t USB_REQUEST_SET_PROTOCOL = 0x0b;
constexpr uint8_t EP_TYPE_CONTROL = 4;
constexpr uint8_t EP_TYPE_INTERRUPT_IN = 7;
constexpr uint8_t USB_CLASS_HID = 3;
constexpr uint8_t USB_ENDPOINT_XFER_INT = 3;

inline uint32_t mmio_read32(uintptr_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

inline uint64_t mmio_read64(uintptr_t addr) {
    return *reinterpret_cast<volatile uint64_t*>(addr);
}

inline void mmio_write32(uintptr_t addr, uint32_t value) {
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

inline void mmio_write64(uintptr_t addr, uint64_t value) {
    *reinterpret_cast<volatile uint64_t*>(addr) = value;
}

inline uintptr_t operational_addr(const ControllerState& state, uint32_t offset) {
    return state.op_base + offset;
}

inline uintptr_t runtime_addr(const ControllerState& state, uint32_t offset) {
    return state.mmio_base + state.runtime_offset + offset;
}

inline uintptr_t doorbell_addr(const ControllerState& state, uint32_t index) {
    return state.mmio_base + state.doorbell_offset + static_cast<uintptr_t>(index) * 4u;
}

inline uintptr_t portsc_addr(const ControllerState& state, uint32_t port_index) {
    return state.op_base + XHCI_PORTSC_BASE +
           static_cast<uintptr_t>(port_index) * XHCI_PORTSC_STRIDE;
}

void delay_us(uint64_t us) {
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (timer) {
        timer->wait_until_ns((timer->get_system_time_us() + us) * 1000ULL);
        return;
    }
    uint64_t i = 0;
    while (i < (us * 64ULL)) {
        asm volatile("" ::: "memory");
        ++i;
    }
}

template <typename Predicate>
bool wait_until(Predicate&& pred, uint64_t timeout_us) {
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    const uint64_t deadline = timer ? (timer->get_system_time_us() + timeout_us) : timeout_us;
    uint64_t spins = 0;
    while (!pred()) {
        kernel::util::cpu_relax();
        if (timer) {
            if (timer->get_system_time_us() >= deadline) return false;
        } else if (++spins >= (timeout_us * 1024ULL)) {
            return false;
        }
    }
    return true;
}

kernel::hal::usb::PortSpeed decode_speed(uint32_t portsc) {
    const uint32_t psi = (portsc & PORTSC_SPEED_MASK) >> PORTSC_SPEED_SHIFT;
    switch (psi) {
        case 1: return kernel::hal::usb::PortSpeed::Full;
        case 2: return kernel::hal::usb::PortSpeed::Low;
        case 3: return kernel::hal::usb::PortSpeed::High;
        case 4: return kernel::hal::usb::PortSpeed::Super;
        default: return kernel::hal::usb::PortSpeed::Unknown;
    }
}

uint32_t scratchpad_count(uint32_t hcsparams2) {
    const uint32_t low = (hcsparams2 >> 21) & 0x1fu;
    const uint32_t high = (hcsparams2 >> 27) & 0x1fu;
    return low | (high << 5);
}

void* input_context_ptr(uint8_t* base, uint32_t context_size, uint32_t index) {
    return base + static_cast<size_t>(index) * context_size;
}

void* device_context_ptr(uint8_t* base, uint32_t context_size, uint32_t index) {
    return base + static_cast<size_t>(index) * context_size;
}

uint32_t raw_port_speed(const ControllerState& state, uint32_t port_index) {
    return (mmio_read32(portsc_addr(state, port_index)) & PORTSC_SPEED_MASK) >> PORTSC_SPEED_SHIFT;
}

uint32_t ep0_max_packet_for_speed(uint32_t speed) {
    switch (speed) {
        case 3: return 64;   // High-speed
        case 4: return 512;  // SuperSpeed
        case 1: [[fallthrough]]; // Full-speed
        case 2: [[fallthrough]]; // Low-speed
        default: return 8;
    }
}

void write_context32(uint8_t* ctx, size_t index, uint32_t value) {
    kernel::util::kmemcpy(ctx + index * sizeof(uint32_t), &value, sizeof(value));
}

void write_context64(uint8_t* ctx, size_t dword_index, uint64_t value) {
    kernel::util::kmemcpy(ctx + dword_index * sizeof(uint32_t), &value, sizeof(value));
}

void build_command_ring(ControllerState& state) {
    kernel::util::kmemset(state.command_ring, 0, sizeof(state.command_ring));
    const uintptr_t ring_base = reinterpret_cast<uintptr_t>(&state.command_ring[0]);
    TRB& link = state.command_ring[ControllerState::COMMAND_RING_TRBS - 1];
    link.parameter = ring_base;
    link.control = (TRB_TYPE_LINK << TRB_TYPE_SHIFT) | 1u | TRB_LINK_TOGGLE_CYCLE;
    state.command_ring_index = 0;
    state.command_cycle = 1;
}

void build_control_ring(ControllerState& state) {
    kernel::util::kmemset(state.control_ring, 0, sizeof(state.control_ring));
    const uintptr_t ring_base = reinterpret_cast<uintptr_t>(&state.control_ring[0]);
    TRB& link = state.control_ring[ControllerState::CONTROL_RING_TRBS - 1];
    link.parameter = ring_base;
    link.control = (TRB_TYPE_LINK << TRB_TYPE_SHIFT) | TRB_CYCLE | TRB_LINK_TOGGLE_CYCLE;
    state.control_ring_index = 0;
    state.control_cycle = 1;
    state.control_ring_ready = true;
}

void build_interrupt_ring(ControllerState& state) {
    kernel::util::kmemset(state.interrupt_ring, 0, sizeof(state.interrupt_ring));
    const uintptr_t ring_base = reinterpret_cast<uintptr_t>(&state.interrupt_ring[0]);
    TRB& link = state.interrupt_ring[ControllerState::INTERRUPT_RING_TRBS - 1];
    link.parameter = ring_base;
    link.control = (TRB_TYPE_LINK << TRB_TYPE_SHIFT) | TRB_CYCLE | TRB_LINK_TOGGLE_CYCLE;
    state.interrupt_ring_index = 0;
    state.interrupt_cycle = 1;
    state.keyboard_transfer_trb = 0;
    state.keyboard_poll_pending = false;
}

void build_event_ring(ControllerState& state) {
    kernel::util::kmemset(state.event_ring, 0, sizeof(state.event_ring));
    kernel::util::kmemset(state.erst, 0, sizeof(state.erst));
    state.erst[0].ring_segment_base = reinterpret_cast<uintptr_t>(&state.event_ring[0]);
    state.erst[0].ring_segment_size = ControllerState::EVENT_RING_TRBS;
    state.event_ring_index = 0;
    state.event_cycle = 1;
}

void reset_controller_state(ControllerState& state, const pci::HostBridge& host) {
    state.host = host;
    state.pci = {};
    state.mmio_base = 0;
    state.op_base = 0;
    state.cap_length = 0;
    state.hci_version = 0;
    state.max_slots = 0;
    state.max_ports = 0;
    state.max_scratchpad_buffers = 0;
    state.context_size = 32;
    state.doorbell_offset = 0;
    state.runtime_offset = 0;
    state.pagesize = 0;
    state.command_cycle = 1;
    state.event_cycle = 1;
    state.control_cycle = 1;
    state.last_command_completion = 0;
    state.command_ring_index = 0;
    state.event_ring_index = 0;
    state.control_ring_index = 0;
    state.slot_id = 0;
    state.slot_speed = 0;
    state.observed_port = 0;
    state.addressed_port = 0;
    state.keyboard_config_value = 0;
    state.keyboard_interface_number = 0;
    state.keyboard_endpoint_address = 0;
    state.keyboard_endpoint_dci = 0;
    state.keyboard_interval = 0;
    state.interrupt_ring_index = 0;
    state.interrupt_cycle = 1;
    state.device_descriptor = {};
    state.keyboard_max_packet = 0;
    state.keyboard_transfer_trb = 0;
    state.keyboard_report_count = 0;
    state.transfer_event_count = 0;
    state.port_change_event_count = 0;
    state.keyboard_last_report_len = 0;
    state.keyboard_last_completion = 0;
    state.command_ring_ready = false;
    state.control_ring_ready = false;
    state.controller_running = false;
    state.present = false;
    state.initialized = false;
    state.device_addressed = false;
    state.descriptor_read = false;
    state.keyboard_candidate = false;
    state.keyboard_configured = false;
    state.keyboard_poll_pending = false;
    state.keyboard_connected = false;
    kernel::util::kmemset(state.key_states, 0, sizeof(state.key_states));
    kernel::util::kmemset(state.keyboard_report, 0, sizeof(state.keyboard_report));
    kernel::util::kmemset(state.keyboard_prev_report, 0, sizeof(state.keyboard_prev_report));
}

bool reset_controller(ControllerState& state) {
    uint32_t usbcmd = mmio_read32(operational_addr(state, XHCI_USBCMD));
    if (usbcmd & XHCI_USBCMD_RS) {
        mmio_write32(operational_addr(state, XHCI_USBCMD), usbcmd & ~XHCI_USBCMD_RS);
        if (!wait_until([&]() {
                return (mmio_read32(operational_addr(state, XHCI_USBSTS)) & XHCI_USBSTS_HCH) != 0;
            }, 100000)) {
            return false;
        }
    }

    mmio_write32(operational_addr(state, XHCI_USBCMD), XHCI_USBCMD_HCRST);
    if (!wait_until([&]() {
            return (mmio_read32(operational_addr(state, XHCI_USBCMD)) & XHCI_USBCMD_HCRST) == 0;
        }, 100000)) {
        return false;
    }
    if (!wait_until([&]() {
            return (mmio_read32(operational_addr(state, XHCI_USBSTS)) & XHCI_USBSTS_CNR) == 0;
        }, 100000)) {
        return false;
    }
    return true;
}

bool setup_runtime(ControllerState& state) {
    state.pagesize = mmio_read32(operational_addr(state, XHCI_PAGESIZE));
    if ((state.pagesize & 0x1u) == 0) return false;

    build_command_ring(state);
    build_event_ring(state);
    build_control_ring(state);
    build_interrupt_ring(state);
    kernel::util::kmemset(state.dcbaa, 0, sizeof(state.dcbaa));
    kernel::util::kmemset(state.scratchpad_ptrs, 0, sizeof(state.scratchpad_ptrs));
    kernel::util::kmemset(state.device_context, 0, sizeof(state.device_context));
    kernel::util::kmemset(state.input_context, 0, sizeof(state.input_context));

    if (state.max_scratchpad_buffers > ControllerState::MAX_SCRATCHPAD_BUFFERS) return false;
    if (state.max_scratchpad_buffers != 0) {
        for (uint32_t i = 0; i < state.max_scratchpad_buffers; ++i) {
            state.scratchpad_ptrs[i] = reinterpret_cast<uintptr_t>(&state.scratchpad_pages[i][0]);
        }
        state.dcbaa[0] = reinterpret_cast<uintptr_t>(&state.scratchpad_ptrs[0]);
    }

    mmio_write32(operational_addr(state, XHCI_DNCTRL), 0);
    mmio_write64(operational_addr(state, XHCI_DCBAAP),
                 reinterpret_cast<uintptr_t>(&state.dcbaa[0]));
    mmio_write64(operational_addr(state, XHCI_CRCR),
                 reinterpret_cast<uintptr_t>(&state.command_ring[0]) | 1ULL);

    mmio_write32(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_IMAN), XHCI_IMAN_IE);
    mmio_write32(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_IMOD), 0);
    mmio_write32(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_ERSTSZ), 1);
    mmio_write64(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_ERSTBA),
                 reinterpret_cast<uintptr_t>(&state.erst[0]));
    mmio_write64(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_ERDP),
                 reinterpret_cast<uintptr_t>(&state.event_ring[0]));

    const uint32_t slots_to_enable = state.max_slots == 0 ? 0 : 1;
    mmio_write32(operational_addr(state, XHCI_CONFIG), slots_to_enable);
    state.command_ring_ready = true;
    return slots_to_enable != 0;
}

bool start_controller(ControllerState& state) {
    uint32_t usbcmd = mmio_read32(operational_addr(state, XHCI_USBCMD));
    usbcmd |= XHCI_USBCMD_RS | XHCI_USBCMD_INTE;
    mmio_write32(operational_addr(state, XHCI_USBCMD), usbcmd);
    if (!wait_until([&]() {
            return (mmio_read32(operational_addr(state, XHCI_USBSTS)) & XHCI_USBSTS_HCH) == 0;
        }, 100000)) {
        return false;
    }
    state.controller_running = true;
    return true;
}

bool next_event(ControllerState& state, TRB& out) {
    TRB& trb = state.event_ring[state.event_ring_index];
    if ((trb.control & 1u) != state.event_cycle) return false;
    out = trb;
    kernel::util::kmemset(&trb, 0, sizeof(trb));

    ++state.event_ring_index;
    if (state.event_ring_index >= ControllerState::EVENT_RING_TRBS) {
        state.event_ring_index = 0;
        state.event_cycle ^= 1u;
    }
    mmio_write64(runtime_addr(state, XHCI_INTERRUPTER0 + XHCI_ERDP),
                 reinterpret_cast<uintptr_t>(&state.event_ring[state.event_ring_index]) | XHCI_ERDP_EHB);
    if (mmio_read32(operational_addr(state, XHCI_USBSTS)) & XHCI_USBSTS_EINT) {
        mmio_write32(operational_addr(state, XHCI_USBSTS), XHCI_USBSTS_EINT);
    }
    return true;
}

void refresh_observed_port(ControllerState& state) {
    state.observed_port = 0;
    for (uint32_t port = 0; port < state.max_ports; ++port) {
        const uint32_t portsc = mmio_read32(portsc_addr(state, port));
        if (portsc & PORTSC_CCS) {
            state.observed_port = static_cast<uint8_t>(port + 1u);
            return;
        }
    }
}

bool wait_for_event_type(ControllerState& state,
                         uint32_t wanted_type,
                         TRB& out,
                         uint64_t timeout_us) {
    TRB event{};
    return wait_until([&]() {
        if (!next_event(state, event)) return false;
        const uint32_t type = (event.control & TRB_TYPE_MASK) >> TRB_TYPE_SHIFT;
        if (type == TRB_TYPE_PORT_STATUS_CHANGE_EVENT) {
            refresh_observed_port(state);
            return false;
        }
        if (type != wanted_type) return false;
        out = event;
        return true;
    }, timeout_us);
}

bool submit_command(ControllerState& state,
                    uint32_t trb_type,
                    uint64_t parameter = 0,
                    uint32_t status = 0,
                    uint32_t extra_control = 0,
                    uint8_t* completion_code = nullptr,
                    uint8_t* slot_id = nullptr) {
    if (!state.command_ring_ready || !state.controller_running) return false;

    TRB cmd{};
    cmd.parameter = parameter;
    cmd.status = status;
    cmd.control = (trb_type << TRB_TYPE_SHIFT) | (extra_control & ~1u) | state.command_cycle;

    const uintptr_t trb_addr = reinterpret_cast<uintptr_t>(&state.command_ring[state.command_ring_index]);
    state.command_ring[state.command_ring_index] = cmd;

    ++state.command_ring_index;
    if (state.command_ring_index >= ControllerState::COMMAND_RING_TRBS - 1) {
        state.command_ring_index = 0;
        state.command_cycle ^= 1u;
    }

    if (mmio_read32(operational_addr(state, XHCI_USBSTS)) & XHCI_USBSTS_EINT) {
        mmio_write32(operational_addr(state, XHCI_USBSTS), XHCI_USBSTS_EINT);
    }
    mmio_write32(doorbell_addr(state, 0), 0);

    TRB event{};
    if (!wait_for_event_type(state, TRB_TYPE_COMMAND_COMPLETION_EVENT, event, 100000)) return false;
    if (event.parameter != trb_addr) {
        // QEMU sometimes reports a completion TRB pointer that doesn't exactly
        // match what we queued, but the completion code / slot id remain usable.
    }

    const uint8_t cc = static_cast<uint8_t>((event.status & TRB_COMPLETION_CODE_MASK) >> TRB_COMPLETION_CODE_SHIFT);
    state.last_command_completion = cc;
    if (completion_code) *completion_code = cc;
    if (slot_id) *slot_id = static_cast<uint8_t>((event.control & TRB_SLOT_ID_MASK) >> TRB_SLOT_ID_SHIFT);
    return cc == TRB_COMPLETION_SUCCESS;
}

bool submit_enable_slot(ControllerState& state, uint8_t* slot_id_out) {
    uint8_t cc = 0;
    uint8_t slot = 0;
    if (!submit_command(state, TRB_TYPE_ENABLE_SLOT_CMD, 0, 0, 0, &cc, &slot)) return false;
    if (slot == 0) return false;
    state.slot_id = slot;
    if (slot_id_out) *slot_id_out = slot;
    return true;
}

void prepare_address_device_context(ControllerState& state, uint32_t port_index) {
    kernel::util::kmemset(state.input_context, 0, sizeof(state.input_context));
    kernel::util::kmemset(state.device_context, 0, sizeof(state.device_context));

    uint8_t* input_control = state.input_context;
    uint8_t* slot_ctx = static_cast<uint8_t*>(input_context_ptr(state.input_context, state.context_size, 1));
    uint8_t* ep0_ctx = static_cast<uint8_t*>(input_context_ptr(state.input_context, state.context_size, 2));

    write_context32(input_control, 1, 0x3u); // add slot + ep0

    const uint32_t speed = state.slot_speed;
    const uint32_t max_packet = ep0_max_packet_for_speed(speed);
    write_context32(slot_ctx, 0, (speed << 20) | (1u << 27));
    write_context32(slot_ctx, 1, static_cast<uint32_t>((port_index + 1u) << 16));

    write_context32(ep0_ctx, 1, (3u << 1) | (EP_TYPE_CONTROL << 3) | (max_packet << 16));
    write_context64(ep0_ctx, 2, reinterpret_cast<uintptr_t>(&state.control_ring[0]) | 1ULL);
    write_context32(ep0_ctx, 4, 8u);

    state.dcbaa[state.slot_id] = reinterpret_cast<uintptr_t>(&state.device_context[0]);
}

bool submit_address_device(ControllerState& state, uint32_t port_index) {
    prepare_address_device_context(state, port_index);
    uint8_t cc = 0;
    if (!submit_command(state,
                        TRB_TYPE_ADDRESS_DEVICE_CMD,
                        reinterpret_cast<uintptr_t>(&state.input_context[0]),
                        0,
                        static_cast<uint32_t>(state.slot_id) << TRB_SLOT_ID_SHIFT,
                        &cc,
                        nullptr)) {
        return false;
    }
    state.device_addressed = true;
    state.addressed_port = static_cast<uint8_t>(port_index + 1u);
    return true;
}

bool next_transfer_event(ControllerState& state, TRB& out) {
    return wait_for_event_type(state, TRB_TYPE_TRANSFER_EVENT, out, 100000);
}

TRB& queue_control_trb(ControllerState& state, uint8_t& cycle) {
    cycle = state.control_cycle;
    TRB& trb = state.control_ring[state.control_ring_index];
    kernel::util::kmemset(&trb, 0, sizeof(trb));
    ++state.control_ring_index;
    if (state.control_ring_index >= ControllerState::CONTROL_RING_TRBS - 1) {
        state.control_ring_index = 0;
        state.control_cycle ^= 1u;
    }
    return trb;
}

bool submit_control_transfer(ControllerState& state,
                             const SetupPacket& packet,
                             void* data,
                             size_t data_len,
                             bool data_in) {
    if (!state.device_addressed || !state.control_ring_ready) return false;
    uint8_t cycle = 0;
    state.setup_packet = packet;
    uint64_t setup_data = 0;
    kernel::util::kmemcpy(&setup_data, &state.setup_packet, sizeof(state.setup_packet));
    TRB& setup = queue_control_trb(state, cycle);
    setup.parameter = setup_data;
    setup.status = 8u;
    setup.control = (TRB_TYPE_SETUP_STAGE << TRB_TYPE_SHIFT) |
                    (((data_len == 0 ? TRB_TRT_NONE : (data_in ? TRB_TRT_IN : TRB_TRT_OUT))) << TRB_TRT_SHIFT) |
                    TRB_IDT | cycle;

    if (data_len != 0) {
        TRB& data_trb = queue_control_trb(state, cycle);
        data_trb.parameter = reinterpret_cast<uintptr_t>(data);
        data_trb.status = static_cast<uint32_t>(data_len) & TRB_TRANSFER_LEN_MASK;
        data_trb.control = (TRB_TYPE_DATA_STAGE << TRB_TYPE_SHIFT) |
                           (data_in ? TRB_DIR_IN : 0u) |
                           TRB_ISP | cycle;
    }

    TRB& status = queue_control_trb(state, cycle);
    status.control = (TRB_TYPE_STATUS_STAGE << TRB_TYPE_SHIFT) |
                     (data_in ? 0u : TRB_DIR_IN) |
                     TRB_IOC | cycle;

    mmio_write32(doorbell_addr(state, state.slot_id), 1u);

    TRB event{};
    if (!next_transfer_event(state, event)) return false;
    const uint8_t cc = static_cast<uint8_t>((event.status & TRB_COMPLETION_CODE_MASK) >> TRB_COMPLETION_CODE_SHIFT);
    return cc == TRB_COMPLETION_SUCCESS;
}

bool submit_get_device_descriptor(ControllerState& state) {
    kernel::util::kmemset(state.control_buffer, 0, sizeof(state.control_buffer));
    const SetupPacket packet{
        .request_type = 0x80,
        .request = USB_REQUEST_GET_DESCRIPTOR,
        .value = static_cast<uint16_t>(USB_DESCRIPTOR_TYPE_DEVICE << 8),
        .index = 0,
        .length = sizeof(DeviceDescriptor),
    };
    if (!submit_control_transfer(state, packet, &state.control_buffer[0], sizeof(DeviceDescriptor), true)) return false;

    kernel::util::kmemcpy(&state.device_descriptor,
                          &state.control_buffer[0],
                          sizeof(DeviceDescriptor));
    if (state.device_descriptor.length < 8 || state.device_descriptor.descriptor_type != USB_DESCRIPTOR_TYPE_DEVICE) {
        return false;
    }
    state.descriptor_read = true;
    return true;
}

uint8_t endpoint_dci(uint8_t endpoint_address) {
    const uint8_t ep_num = endpoint_address & 0x0fu;
    const bool in = (endpoint_address & 0x80u) != 0;
    return static_cast<uint8_t>(ep_num * 2u + (in ? 1u : 0u));
}

uint32_t interrupt_interval(uint32_t speed, uint8_t b_interval) {
    if (b_interval == 0) return 0;
    if (speed >= 3) return static_cast<uint32_t>(b_interval - 1u);
    return static_cast<uint32_t>(b_interval);
}

bool fetch_configuration_descriptor(ControllerState& state) {
    kernel::util::kmemset(state.config_buffer, 0, sizeof(state.config_buffer));
    ConfigurationDescriptor header{};
    SetupPacket packet{
        .request_type = 0x80,
        .request = USB_REQUEST_GET_DESCRIPTOR,
        .value = static_cast<uint16_t>(USB_DESCRIPTOR_TYPE_CONFIGURATION << 8),
        .index = 0,
        .length = sizeof(ConfigurationDescriptor),
    };
    if (!submit_control_transfer(state, packet, &header, sizeof(header), true)) return false;
    const uint16_t total = header.total_length;
    if (header.length < sizeof(ConfigurationDescriptor) || total < sizeof(ConfigurationDescriptor)) return false;
    const size_t fetch_len = total < sizeof(state.config_buffer) ? total : sizeof(state.config_buffer);
    packet.length = static_cast<uint16_t>(fetch_len);
    if (!submit_control_transfer(state, packet, &state.config_buffer[0], fetch_len, true)) return false;
    return true;
}

bool parse_boot_keyboard(ControllerState& state) {
    state.keyboard_candidate = false;
    state.keyboard_config_value = 0;
    state.keyboard_interface_number = 0;
    state.keyboard_endpoint_address = 0;
    state.keyboard_endpoint_dci = 0;
    state.keyboard_interval = 0;
    state.keyboard_max_packet = 0;

    const uint8_t* p = &state.config_buffer[0];
    const ConfigurationDescriptor* config = reinterpret_cast<const ConfigurationDescriptor*>(p);
    const size_t total = config->total_length < sizeof(state.config_buffer) ? config->total_length : sizeof(state.config_buffer);
    uint8_t current_if = 0xff;
    bool keyboard_if = false;

    for (size_t off = 0; off + 2 <= total;) {
        const uint8_t len = p[off];
        const uint8_t type = p[off + 1];
        if (len < 2 || off + len > total) break;
        if (type == USB_DESCRIPTOR_TYPE_CONFIGURATION && len >= sizeof(ConfigurationDescriptor)) {
            const auto* desc = reinterpret_cast<const ConfigurationDescriptor*>(p + off);
            state.keyboard_config_value = desc->configuration_value;
        } else if (type == USB_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(InterfaceDescriptor)) {
            const auto* desc = reinterpret_cast<const InterfaceDescriptor*>(p + off);
            current_if = desc->interface_number;
            keyboard_if = desc->interface_class == USB_CLASS_HID &&
                          desc->interface_subclass == 1u &&
                          desc->interface_protocol == 1u;
        } else if (type == USB_DESCRIPTOR_TYPE_ENDPOINT && keyboard_if && current_if != 0xff && len >= sizeof(EndpointDescriptor)) {
            const auto* desc = reinterpret_cast<const EndpointDescriptor*>(p + off);
            if ((desc->endpoint_address & 0x80u) != 0 &&
                (desc->attributes & 0x03u) == USB_ENDPOINT_XFER_INT) {
                state.keyboard_candidate = true;
                state.keyboard_interface_number = current_if;
                state.keyboard_endpoint_address = desc->endpoint_address;
                state.keyboard_endpoint_dci = endpoint_dci(desc->endpoint_address);
                state.keyboard_interval = desc->interval;
                state.keyboard_max_packet = desc->max_packet_size & 0x07ffu;
                return true;
            }
        }
        off += len;
    }
    return false;
}

bool submit_set_configuration(ControllerState& state) {
    const SetupPacket packet{
        .request_type = 0x00,
        .request = USB_REQUEST_SET_CONFIGURATION,
        .value = state.keyboard_config_value,
        .index = 0,
        .length = 0,
    };
    return state.keyboard_config_value != 0 && submit_control_transfer(state, packet, nullptr, 0, false);
}

bool submit_set_boot_protocol(ControllerState& state) {
    const SetupPacket packet{
        .request_type = 0x21,
        .request = USB_REQUEST_SET_PROTOCOL,
        .value = 0,
        .index = state.keyboard_interface_number,
        .length = 0,
    };
    return submit_control_transfer(state, packet, nullptr, 0, false);
}

bool submit_configure_keyboard_endpoint(ControllerState& state) {
    if (!state.keyboard_candidate || state.keyboard_endpoint_dci == 0) return false;
    kernel::util::kmemset(state.input_context, 0, sizeof(state.input_context));
    build_interrupt_ring(state);

    const uint32_t dci = state.keyboard_endpoint_dci;
    uint8_t* input_control = state.input_context;
    uint8_t* slot_ctx = static_cast<uint8_t*>(input_context_ptr(state.input_context, state.context_size, 1));
    uint8_t* ep_ctx = static_cast<uint8_t*>(input_context_ptr(state.input_context, state.context_size, dci + 1u));
    uint8_t* out_slot_ctx = static_cast<uint8_t*>(device_context_ptr(state.device_context, state.context_size, 0));
    kernel::util::kmemcpy(slot_ctx, out_slot_ctx, state.context_size);

    write_context32(input_control, 1, (1u << 0) | (1u << dci));

    uint32_t slot0 = 0;
    kernel::util::kmemcpy(&slot0, slot_ctx, sizeof(slot0));
    slot0 &= ~(0x1fu << 27);
    slot0 |= (dci & 0x1fu) << 27;
    write_context32(slot_ctx, 0, slot0);

    write_context32(ep_ctx, 0, interrupt_interval(state.slot_speed, state.keyboard_interval) << 16);
    write_context32(ep_ctx, 1,
                    (3u << 1) |
                    (EP_TYPE_INTERRUPT_IN << 3) |
                    (static_cast<uint32_t>(state.keyboard_max_packet) << 16));
    write_context64(ep_ctx, 2, reinterpret_cast<uintptr_t>(&state.interrupt_ring[0]) | 1ULL);
    write_context32(ep_ctx, 4,
                    (static_cast<uint32_t>(state.keyboard_max_packet) << 16) |
                    (state.keyboard_max_packet ? state.keyboard_max_packet : 8u));

    uint8_t cc = 0;
    return submit_command(state,
                          TRB_TYPE_CONFIGURE_ENDPOINT_CMD,
                          reinterpret_cast<uintptr_t>(&state.input_context[0]),
                          0,
                          static_cast<uint32_t>(state.slot_id) << TRB_SLOT_ID_SHIFT,
                          &cc,
                          nullptr);
}

uint8_t hid_usage_to_linux_key(uint8_t usage) {
    switch (usage) {
        case 0x04: return 30;  // A
        case 0x05: return 48;  // B
        case 0x06: return 46;  // C
        case 0x07: return 32;  // D
        case 0x08: return 18;  // E
        case 0x09: return 33;  // F
        case 0x0a: return 34;  // G
        case 0x0b: return 35;  // H
        case 0x0c: return 23;  // I
        case 0x0d: return 36;  // J
        case 0x0e: return 37;  // K
        case 0x0f: return 38;  // L
        case 0x10: return 50;  // M
        case 0x11: return 49;  // N
        case 0x12: return 24;  // O
        case 0x13: return 25;  // P
        case 0x14: return 16;  // Q
        case 0x15: return 19;  // R
        case 0x16: return 31;  // S
        case 0x17: return 20;  // T
        case 0x18: return 22;  // U
        case 0x19: return 47;  // V
        case 0x1a: return 17;  // W
        case 0x1b: return 45;  // X
        case 0x1c: return 21;  // Y
        case 0x1d: return 44;  // Z
        case 0x1e: return 2;
        case 0x1f: return 3;
        case 0x20: return 4;
        case 0x21: return 5;
        case 0x22: return 6;
        case 0x23: return 7;
        case 0x24: return 8;
        case 0x25: return 9;
        case 0x26: return 10;
        case 0x27: return 11;
        case 0x28: return 28;
        case 0x29: return 1;
        case 0x2a: return 14;
        case 0x2b: return 15;
        case 0x2c: return 57;
        case 0x2d: return 12;
        case 0x2e: return 13;
        case 0x2f: return 26;
        case 0x30: return 27;
        case 0x31: return 43;
        case 0x33: return 39;
        case 0x34: return 40;
        case 0x35: return 41;
        case 0x36: return 51;
        case 0x37: return 52;
        case 0x38: return 53;
        case 0x49: return 110;
        case 0x4a: return 102;
        case 0x4b: return 104;
        case 0x4c: return 111;
        case 0x4d: return 107;
        case 0x4e: return 109;
        case 0x4f: return 106;
        case 0x50: return 105;
        case 0x51: return 108;
        case 0x52: return 103;
        case 0x53: return 83;
        case 0x54: return 98;
        case 0x55: return 55;
        case 0x56: return 74;
        case 0x57: return 78;
        case 0x58: return 96;
        case 0x59: return 79;
        case 0x5a: return 80;
        case 0x5b: return 81;
        case 0x5c: return 75;
        case 0x5d: return 76;
        case 0x5e: return 77;
        case 0x5f: return 71;
        case 0x60: return 72;
        case 0x61: return 73;
        case 0x62: return 82;
        case 0x63: return 83;
        default: return 0;
    }
}

void set_modifier_keys(ControllerState& state, uint8_t modifiers) {
    state.key_states[29] = (modifiers & (1u << 0)) != 0;
    state.key_states[42] = (modifiers & (1u << 1)) != 0;
    state.key_states[56] = (modifiers & (1u << 2)) != 0;
    state.key_states[97] = (modifiers & (1u << 4)) != 0;
    state.key_states[54] = (modifiers & (1u << 5)) != 0;
    state.key_states[100] = (modifiers & (1u << 6)) != 0;
}

void update_keyboard_report(ControllerState& state, size_t report_len) {
    if (report_len < 2) return;
    state.keyboard_last_report_len = static_cast<uint8_t>(report_len);
    ++state.keyboard_report_count;
    set_modifier_keys(state, state.keyboard_report[0]);
    for (size_t i = 2; i < sizeof(state.keyboard_prev_report); ++i) {
        const uint8_t key = hid_usage_to_linux_key(state.keyboard_prev_report[i]);
        if (key != 0) state.key_states[key] = false;
    }
    for (size_t i = 2; i < report_len && i < sizeof(state.keyboard_report); ++i) {
        const uint8_t key = hid_usage_to_linux_key(state.keyboard_report[i]);
        if (key != 0) state.key_states[key] = true;
    }
    kernel::util::kmemcpy(state.keyboard_prev_report, state.keyboard_report, sizeof(state.keyboard_prev_report));
}

bool arm_keyboard_poll(ControllerState& state) {
    if (!state.keyboard_configured || state.keyboard_poll_pending) return false;
    TRB& trb = state.interrupt_ring[state.interrupt_ring_index];
    kernel::util::kmemset(&trb, 0, sizeof(trb));
    trb.parameter = reinterpret_cast<uintptr_t>(&state.keyboard_report[0]);
    trb.status = (state.keyboard_max_packet ? state.keyboard_max_packet : sizeof(state.keyboard_report)) & TRB_TRANSFER_LEN_MASK;
    trb.control = (TRB_TYPE_NORMAL << TRB_TYPE_SHIFT) | TRB_ISP | TRB_IOC | state.interrupt_cycle;
    state.keyboard_transfer_trb = reinterpret_cast<uintptr_t>(&trb);
    ++state.interrupt_ring_index;
    if (state.interrupt_ring_index >= ControllerState::INTERRUPT_RING_TRBS - 1) {
        state.interrupt_ring_index = 0;
        state.interrupt_cycle ^= 1u;
    }
    state.keyboard_poll_pending = true;
    mmio_write32(doorbell_addr(state, state.slot_id), state.keyboard_endpoint_dci);
    return true;
}

bool initialize_ports(ControllerState& state) {
    for (uint32_t port = 0; port < state.max_ports; ++port) {
        const uintptr_t reg = portsc_addr(state, port);
        const uint32_t cur = mmio_read32(reg);
        if ((cur & PORTSC_PP) == 0) {
            uint32_t next = cur & PORTSC_RW1C_MASK;
            next |= PORTSC_PP;
            mmio_write32(reg, next);
        }
    }

    delay_us(20000);
    (void)wait_until([&]() {
        refresh_observed_port(state);
        return state.observed_port != 0;
    }, 500000);

    for (uint32_t port = 0; port < state.max_ports; ++port) {
        const uint32_t portsc = mmio_read32(portsc_addr(state, port));
        if ((portsc & PORTSC_CCS) == 0 || (portsc & PORTSC_PED) != 0) continue;
        uint32_t next = portsc & PORTSC_RW1C_MASK;
        if (portsc & PORTSC_PP) next |= PORTSC_PP;
        next |= PORTSC_PR;
        mmio_write32(portsc_addr(state, port), next);
        if (!wait_until([&]() {
                const uint32_t updated = mmio_read32(portsc_addr(state, port));
                return (updated & PORTSC_PR) == 0;
            }, 100000)) {
            return false;
        }
    }
    refresh_observed_port(state);
    return true;
}

int first_connected_port(const ControllerState& state) {
    for (uint32_t port = 0; port < state.max_ports; ++port) {
        const uint32_t portsc = mmio_read32(portsc_addr(state, port));
        if (portsc & PORTSC_CCS) return static_cast<int>(port);
    }
    return -1;
}

void update_info_from_state(const ControllerState& state,
                            kernel::hal::usb::ControllerInfo& info) {
    info.kind = kernel::hal::usb::ControllerKind::XHCI;
    info.driver_name = "qemu-xhci-pci";
    info.transport = "pci";
    info.enum_state = state.keyboard_configured ? "configured" :
                      (state.descriptor_read ? "descriptor" :
                       (state.device_addressed ? "addressed" :
                       (state.slot_id != 0 ? "slot" :
                        (state.observed_port != 0 ? "connected" : "idle"))));
    info.register_base = state.mmio_base;
    info.irq = state.pci.irq_line == 0xffu ? 0u : state.pci.irq_line;
    info.port_count = state.max_ports;
    info.active_port = state.addressed_port ? state.addressed_port : state.observed_port;
    info.device_vendor_id = state.device_descriptor.vendor_id;
    info.device_product_id = state.device_descriptor.product_id;
    info.device_class = state.device_descriptor.device_class;
    info.keyboard_last_report_len = state.keyboard_last_report_len;
    info.keyboard_endpoint_dci = state.keyboard_endpoint_dci;
    info.keyboard_last_completion = state.keyboard_last_completion;
    info.keyboard_reports = state.keyboard_report_count;
    info.transfer_events = state.transfer_event_count;
    info.port_change_events = state.port_change_event_count;
    info.present = state.present;
    info.initialized = state.initialized;
    info.hotplug_capable = true;
    info.qemu_virtual = true;
    info.device_addressed = state.device_addressed;
    info.device_descriptor_read = state.descriptor_read;
    info.keyboard_connected = state.keyboard_connected;
    info.keyboard_poll_pending = state.keyboard_poll_pending;
}

void process_runtime_events(ControllerState& state) {
    TRB event{};
    while (next_event(state, event)) {
        const uint32_t type = (event.control & TRB_TYPE_MASK) >> TRB_TYPE_SHIFT;
        if (type == TRB_TYPE_PORT_STATUS_CHANGE_EVENT) {
            ++state.port_change_event_count;
            refresh_observed_port(state);
            continue;
        }
        if (type == TRB_TYPE_TRANSFER_EVENT && state.keyboard_poll_pending) {
            const uint8_t cc = static_cast<uint8_t>((event.status & TRB_COMPLETION_CODE_MASK) >> TRB_COMPLETION_CODE_SHIFT);
            const uint8_t slot_id = static_cast<uint8_t>((event.control & TRB_SLOT_ID_MASK) >> TRB_SLOT_ID_SHIFT);
            const uint8_t endpoint_id = static_cast<uint8_t>((event.control & TRB_ENDPOINT_ID_MASK) >> TRB_ENDPOINT_ID_SHIFT);
            const bool pointer_match = event.parameter == state.keyboard_transfer_trb;
            const bool endpoint_match =
                slot_id == state.slot_id &&
                endpoint_id == state.keyboard_endpoint_dci;
            if (pointer_match || endpoint_match) {
                ++state.transfer_event_count;
                state.keyboard_last_completion = cc;
            }
            if ((pointer_match || endpoint_match) && cc == TRB_COMPLETION_SUCCESS) {
                const uint32_t remaining = event.status & TRB_TRANSFER_LEN_MASK;
                const size_t requested = state.keyboard_max_packet ? state.keyboard_max_packet : sizeof(state.keyboard_report);
                const size_t actual = requested >= remaining ? requested - remaining : 0;
                update_keyboard_report(state, actual);
                state.keyboard_connected = true;
            }
            if (pointer_match || endpoint_match) {
                state.keyboard_poll_pending = false;
                state.keyboard_transfer_trb = 0;
            }
        }
    }
}

void maybe_enumerate(ControllerState& state) {
    if (!state.present || !state.initialized || !state.controller_running) return;
    process_runtime_events(state);
    refresh_observed_port(state);
    if (state.observed_port == 0) return;

    const uint32_t port_index = static_cast<uint32_t>(state.observed_port - 1u);
    if (state.slot_speed == 0) state.slot_speed = static_cast<uint8_t>(raw_port_speed(state, port_index));

    if (state.slot_id == 0) {
        uint8_t slot_id = 0;
        if (!submit_enable_slot(state, &slot_id)) return;
    }
    if (!state.device_addressed) {
        if (!submit_address_device(state, port_index)) return;
    }
    if (!state.descriptor_read) {
        (void)submit_get_device_descriptor(state);
    }
    if (!state.descriptor_read || state.keyboard_configured) return;
    if (!state.keyboard_candidate) {
        if (!fetch_configuration_descriptor(state)) return;
        if (!parse_boot_keyboard(state)) return;
    }
    if (!submit_set_configuration(state)) return;
    (void)submit_set_boot_protocol(state);
    if (!submit_configure_keyboard_endpoint(state)) return;
    state.keyboard_configured = true;
    state.keyboard_connected = true;
    (void)arm_keyboard_poll(state);
}

} // namespace

bool init_from_pci(const pci::HostBridge& host,
                   ControllerState& state,
                   kernel::hal::usb::ControllerInfo& info) {
    reset_controller_state(state, host);
    info = {};
    const bool by_class = pci::find_device_by_class(host, XHCI_CLASS, XHCI_SUBCLASS, XHCI_PROGIF, state.pci);
    const bool by_qemu_id = !by_class &&
        pci::find_device_by_vendor_device(host, QEMU_PCI_VENDOR, QEMU_XHCI_DEVICE, state.pci);
    if (!by_class && !by_qemu_id) return false;
    if (state.pci.bar0_is_io) return false;

    const bool bar0_unassigned =
        state.pci.bar0 == 0 ||
        state.pci.bar0 == ~0ULL ||
        state.pci.bar0 == 0xfffffffffffffff0ULL ||
        state.pci.bar0 == 0xfffffff0ULL;
    if (bar0_unassigned) {
        if (host.mmio_base == 0 || state.pci.bar0_size == 0) return false;
        const uint64_t aligned = (host.mmio_base + state.pci.bar0_size - 1u) & ~(state.pci.bar0_size - 1u);
        if (!pci::assign_bar0(host, state.pci.addr, aligned)) return false;
        if (!pci::probe_device(host, state.pci.addr, state.pci)) return false;
    }
    if (state.pci.bar0_is_io || state.pci.bar0 == 0 || state.pci.bar0 == ~0ULL) return false;
    if (!pci::enable_memory_busmaster(host, state.pci.addr)) return false;

    state.mmio_base = static_cast<uintptr_t>(state.pci.bar0);
    const uint32_t cap0 = mmio_read32(state.mmio_base + 0x00);
    state.cap_length = static_cast<uint8_t>(cap0 & 0xffu);
    state.hci_version = static_cast<uint16_t>((cap0 >> 16) & 0xffffu);
    state.op_base = state.mmio_base + state.cap_length;
    const uint32_t hcsparams1 = mmio_read32(state.mmio_base + XHCI_HCSPARAMS1);
    const uint32_t hcsparams2 = mmio_read32(state.mmio_base + XHCI_HCSPARAMS2);
    const uint32_t hccparams1 = mmio_read32(state.mmio_base + XHCI_HCCPARAMS1);
    state.max_slots = hcsparams1 & 0xffu;
    state.max_ports = (hcsparams1 >> 24) & 0xffu;
    state.max_scratchpad_buffers = scratchpad_count(hcsparams2);
    state.context_size = (hccparams1 & (1u << 2)) ? 64u : 32u;
    state.doorbell_offset = mmio_read32(state.mmio_base + XHCI_DBOFF) & ~0x3u;
    state.runtime_offset = mmio_read32(state.mmio_base + XHCI_RTSOFF) & ~0x1fu;

    if (state.cap_length < 0x20u || state.max_ports == 0u) return false;
    if (!reset_controller(state)) return false;
    if (!setup_runtime(state)) return false;
    if (!start_controller(state)) return false;

    uint8_t completion = 0;
    if (!submit_command(state, TRB_TYPE_NOOP_CMD, 0, 0, 0, &completion, nullptr)) return false;
    if (!initialize_ports(state)) return false;

    const int connected_port = first_connected_port(state);
    if (connected_port >= 0) {
        state.observed_port = static_cast<uint8_t>(connected_port + 1u);
        state.slot_speed = static_cast<uint8_t>(raw_port_speed(state, static_cast<uint32_t>(connected_port)));
        uint8_t slot_id = 0;
        if (submit_enable_slot(state, &slot_id) &&
            submit_address_device(state, static_cast<uint32_t>(connected_port))) {
            (void)submit_get_device_descriptor(state);
        }
    }

    state.present = true;
    state.initialized = true;
    update_info_from_state(state, info);
    return true;
}

void refresh_info(ControllerState& state,
                  kernel::hal::usb::ControllerInfo& info) {
    maybe_enumerate(state);
    update_info_from_state(state, info);
}

void poll(ControllerState& state) {
    maybe_enumerate(state);
    process_runtime_events(state);
    if (state.keyboard_configured && !state.keyboard_poll_pending) {
        (void)arm_keyboard_poll(state);
    }
}

bool keyboard_connected(const ControllerState& state) {
    return state.keyboard_connected;
}

bool key_pressed(const ControllerState& state, uint8_t key) {
    return key < sizeof(state.key_states) ? state.key_states[key] : false;
}

bool get_port_status(const ControllerState& state,
                     uint32_t port_index,
                     kernel::hal::usb::PortStatus& out) {
    if (!state.initialized || port_index >= state.max_ports) return false;
    const uint32_t portsc = mmio_read32(portsc_addr(state, port_index));
    out.powered = (portsc & PORTSC_PP) != 0;
    out.connected = (portsc & PORTSC_CCS) != 0;
    out.enabled = (portsc & PORTSC_PED) != 0;
    out.over_current = (portsc & PORTSC_OCA) != 0;
    out.reset_in_progress = (portsc & PORTSC_PR) != 0;
    out.speed = decode_speed(portsc);
    return true;
}

bool set_port_power(ControllerState& state, uint32_t port_index, bool on) {
    if (!state.initialized || port_index >= state.max_ports) return false;
    const uintptr_t reg = portsc_addr(state, port_index);
    const uint32_t cur = mmio_read32(reg);
    uint32_t next = cur & PORTSC_RW1C_MASK;
    if ((cur & PORTSC_PP) != 0 || on) next |= PORTSC_PP;
    if (!on) next &= ~PORTSC_PP;
    mmio_write32(reg, next);
    return true;
}

bool reset_port(ControllerState& state, uint32_t port_index) {
    if (!state.initialized || port_index >= state.max_ports) return false;
    const uintptr_t reg = portsc_addr(state, port_index);
    const uint32_t cur = mmio_read32(reg);
    uint32_t next = cur & PORTSC_RW1C_MASK;
    if (cur & PORTSC_PP) next |= PORTSC_PP;
    next |= PORTSC_PR;
    mmio_write32(reg, next);
    return wait_until([&]() { return (mmio_read32(reg) & PORTSC_PR) == 0; }, 100000);
}

} // namespace hal::shared::xhci
