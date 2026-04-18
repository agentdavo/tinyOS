// SPDX-License-Identifier: MIT OR Apache-2.0

#include "hal/shared/virtio_input.hpp"
#include "miniOS.hpp"
#include "util.hpp"

namespace hal::shared::input {

namespace {

alignas(64) VirtIOInput g_input_driver;

uint32_t input_now_ms() {
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!timer) return 0;
    return static_cast<uint32_t>(timer->get_system_time_ns() / 1000000ULL);
}

void input_log(const char* msg) {
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        kernel::g_platform->get_uart_ops()->puts(msg);
    }
}

void input_logf(const char* fmt, unsigned long a, unsigned long b = 0) {
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        char buf[128];
        kernel::util::k_snprintf(buf, sizeof(buf), fmt, a, b);
        kernel::g_platform->get_uart_ops()->puts(buf);
    }
}

void input_log_event_count(const char* tag, uint32_t count) {
    if ((count & 0xFFu) != 1u) return;
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf), "[input] %s events=%lu\n",
                                 tag, static_cast<unsigned long>(count));
        kernel::g_platform->get_uart_ops()->puts(buf);
    }
}

inline uint32_t mmio_r32(uint64_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

inline void mmio_w32(uint64_t addr, uint32_t v) {
    *reinterpret_cast<volatile uint32_t*>(addr) = v;
}

} // namespace

uint32_t VirtIOInputDevice::mmio_read(uint32_t off) const {
    return mmio_r32(base_ + off);
}

void VirtIOInputDevice::mmio_write(uint32_t off, uint32_t val) {
    mmio_w32(base_ + off, val);
}

bool VirtIOInputDevice::setup_queue(uint32_t queue_idx) {
    mmio_write(VMMIO_QUEUE_SEL, queue_idx);
    const uint32_t max = mmio_read(VMMIO_QUEUE_NUM_MAX);
    if (max == 0 || max < VIRTQ_SIZE) return false;
    mmio_write(VMMIO_QUEUE_NUM, VIRTQ_SIZE);

    const uint64_t desc_pa = reinterpret_cast<uint64_t>(&event_queue_.desc[0]);
    const uint64_t avail_pa = reinterpret_cast<uint64_t>(&event_queue_.avail);
    const uint64_t used_pa = reinterpret_cast<uint64_t>(&event_queue_.used);

    mmio_write(VMMIO_QUEUE_DESC_LO, static_cast<uint32_t>(desc_pa));
    mmio_write(VMMIO_QUEUE_DESC_HI, static_cast<uint32_t>(desc_pa >> 32));
    mmio_write(VMMIO_QUEUE_DRIVER_LO, static_cast<uint32_t>(avail_pa));
    mmio_write(VMMIO_QUEUE_DRIVER_HI, static_cast<uint32_t>(avail_pa >> 32));
    mmio_write(VMMIO_QUEUE_DEVICE_LO, static_cast<uint32_t>(used_pa));
    mmio_write(VMMIO_QUEUE_DEVICE_HI, static_cast<uint32_t>(used_pa >> 32));
    mmio_write(VMMIO_QUEUE_READY, 1);
    return true;
}

void VirtIOInputDevice::post_event_buffers() {
    for (size_t i = 0; i < VIRTQ_SIZE; ++i) {
        event_queue_.desc[i].addr = reinterpret_cast<uint64_t>(&event_bufs_[i]);
        event_queue_.desc[i].len = sizeof(VirtioInputEvent);
        event_queue_.desc[i].flags = VIRTQ_DESC_F_WRITE;
        event_queue_.desc[i].next = 0;
        event_queue_.avail.ring[i] = static_cast<uint16_t>(i);
    }
    __atomic_store_n(&event_queue_.avail.idx, static_cast<uint16_t>(VIRTQ_SIZE), __ATOMIC_RELEASE);
    mmio_write(VMMIO_QUEUE_NOTIFY, 0);
}

bool VirtIOInputDevice::init(uint64_t slot_base) {
    base_ = slot_base;

    if (mmio_read(VMMIO_MAGIC) != 0x74726976) return false;
    if (mmio_read(VMMIO_VERSION) != 2) return false;
    if (mmio_read(VMMIO_DEVICE_ID) != VIRTIO_DEV_ID_INPUT) return false;

    mmio_write(VMMIO_STATUS, 0);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);

    mmio_write(VMMIO_DEV_FEAT_SEL, 1);
    const uint64_t dev_feat_hi = mmio_read(VMMIO_DEV_FEAT);
    if ((dev_feat_hi & 1u) == 0) {
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    mmio_write(VMMIO_DRV_FEAT_SEL, 0);
    mmio_write(VMMIO_DRV_FEAT, 0);
    mmio_write(VMMIO_DRV_FEAT_SEL, 1);
    mmio_write(VMMIO_DRV_FEAT, 1);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK);
    if ((mmio_read(VMMIO_STATUS) & VIRTIO_STATUS_FEAT_OK) == 0) {
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    if (!setup_queue(0)) {
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
                             VIRTIO_STATUS_FEAT_OK | VIRTIO_STATUS_DRIVER_OK);
    post_event_buffers();
    initialized_ = true;
    return true;
}

void VirtIOInputDevice::handle_event(const VirtioInputEvent& event, InputState& state, MouseEvent& mouse_event) {
    static uint32_t keyboard_events = 0;
    static uint32_t mouse_events = 0;
    static uint32_t touch_events = 0;
    const uint32_t now_ms = input_now_ms();
    switch (event.type) {
        case EV_KEY:
            if (event.code < MAX_KEYBOARD_KEYS) {
                if (!state.keyboard_connected) input_log("[input] keyboard connected\n");
                state.keyboard_connected = true;
                state.key_states[event.code] = event.value != 0;
                input_log_event_count("keyboard", ++keyboard_events);
            } else if (event.code >= BTN_MOUSE && event.code <= BTN_MOUSE + 7) {
                if (!state.mouse_connected) input_log("[input] mouse connected\n");
                state.mouse_connected = true;
                if (event.value) state.mouse_buttons |= 1u << (event.code - BTN_MOUSE);
                else state.mouse_buttons &= static_cast<uint8_t>(~(1u << (event.code - BTN_MOUSE)));
                mouse_event.buttons = state.mouse_buttons;
                mouse_event.timestamp_ms = now_ms;
                input_log_event_count("mouse", ++mouse_events);
            } else if (event.code == BTN_TOUCH) {
                if (!state.touch_connected) input_log("[input] touch connected\n");
                state.touch_connected = true;
                state.touch_active = event.value != 0;
                state.last_touch.pressed = state.touch_active;
                state.last_touch.timestamp_ms = now_ms;
                input_log_event_count("touch", ++touch_events);
            }
            break;
        case EV_REL:
            if (!state.mouse_connected) input_log("[input] mouse connected\n");
            state.mouse_connected = true;
            if (event.code == REL_X) {
                state.mouse_x += static_cast<int32_t>(event.value);
                mouse_event.dx = static_cast<int32_t>(event.value);
            } else if (event.code == REL_Y) {
                state.mouse_y += static_cast<int32_t>(event.value);
                mouse_event.dy = static_cast<int32_t>(event.value);
            } else if (event.code == REL_WHEEL) {
                mouse_event.wheel += static_cast<int32_t>(event.value);
            }
            mouse_event.timestamp_ms = now_ms;
            input_log_event_count("mouse", ++mouse_events);
            break;
        case EV_ABS:
            if (event.code == ABS_X) {
                if (!state.mouse_connected) input_log("[input] mouse connected\n");
                state.mouse_connected = true;
                state.mouse_x = static_cast<int32_t>(event.value);
                state.last_touch.x = static_cast<int32_t>(event.value);
                mouse_event.timestamp_ms = now_ms;
                input_log_event_count("mouse", ++mouse_events);
            } else if (event.code == ABS_Y) {
                if (!state.mouse_connected) input_log("[input] mouse connected\n");
                state.mouse_connected = true;
                state.mouse_y = static_cast<int32_t>(event.value);
                state.last_touch.y = static_cast<int32_t>(event.value);
                mouse_event.timestamp_ms = now_ms;
                input_log_event_count("mouse", ++mouse_events);
            } else if (event.code == ABS_MT_POSITION_X) {
                if (!state.touch_connected) input_log("[input] touch connected\n");
                state.touch_connected = true;
                state.last_touch.x = static_cast<int32_t>(event.value);
                state.last_touch.timestamp_ms = now_ms;
                input_log_event_count("touch", ++touch_events);
            } else if (event.code == ABS_MT_POSITION_Y) {
                if (!state.touch_connected) input_log("[input] touch connected\n");
                state.touch_connected = true;
                state.last_touch.y = static_cast<int32_t>(event.value);
                state.last_touch.timestamp_ms = now_ms;
                input_log_event_count("touch", ++touch_events);
            } else if (event.code == ABS_MT_SLOT) {
                if (!state.touch_connected) input_log("[input] touch connected\n");
                state.touch_connected = true;
                state.last_touch.touch_id = static_cast<uint8_t>(event.value & 0xFFu);
                state.last_touch.timestamp_ms = now_ms;
                input_log_event_count("touch", ++touch_events);
            } else if (event.code == ABS_MT_TRACKING_ID) {
                if (!state.touch_connected) input_log("[input] touch connected\n");
                state.touch_connected = true;
                const int32_t tracking_id = static_cast<int32_t>(event.value);
                state.touch_active = tracking_id >= 0;
                state.last_touch.pressed = state.touch_active;
                if (tracking_id >= 0) state.last_touch.touch_id = static_cast<uint8_t>(tracking_id & 0xFF);
                state.last_touch.timestamp_ms = now_ms;
                input_log_event_count("touch", ++touch_events);
            }
            break;
        case EV_SYN:
        default:
            break;
    }
}

void VirtIOInputDevice::poll(InputState& state, MouseEvent& mouse_event) {
    if (!initialized_) return;

    const uint16_t used_idx = __atomic_load_n(&event_queue_.used.idx, __ATOMIC_ACQUIRE);
    bool processed = false;
    while (event_queue_.last_used != used_idx) {
        const uint16_t ring_pos = event_queue_.last_used % VIRTQ_SIZE;
        const VirtqUsedElem& elem = event_queue_.used.ring[ring_pos];
        const uint16_t desc_idx = static_cast<uint16_t>(elem.id & (VIRTQ_SIZE - 1));
        handle_event(event_bufs_[desc_idx], state, mouse_event);

        event_queue_.desc[desc_idx].len = sizeof(VirtioInputEvent);
        event_queue_.desc[desc_idx].flags = VIRTQ_DESC_F_WRITE;
        event_queue_.desc[desc_idx].next = 0;
        const uint16_t avail_pos = event_queue_.avail.idx % VIRTQ_SIZE;
        event_queue_.avail.ring[avail_pos] = desc_idx;
        __atomic_store_n(&event_queue_.avail.idx,
                         static_cast<uint16_t>(event_queue_.avail.idx + 1),
                         __ATOMIC_RELEASE);
        ++event_queue_.last_used;
        processed = true;
    }

    const uint32_t int_status = mmio_read(VMMIO_INT_STATUS);
    if (int_status != 0) mmio_write(VMMIO_INT_ACK, int_status);
    if (processed) mmio_write(VMMIO_QUEUE_NOTIFY, 0);
}

void VirtIOInput::configure_bus(uint64_t base, size_t stride, size_t slot_count) {
    bus_base_ = base;
    bus_stride_ = stride;
    bus_slot_count_ = slot_count;
}

bool VirtIOInput::init(uintptr_t base, uint32_t irq) {
    base_ = base;
    irq_ = irq;
    state_ = {};
    last_mouse_ = {};
    device_count_ = 0;

    for (size_t i = 0; i < bus_slot_count_ && device_count_ < MAX_INPUT_DEVS; ++i) {
        const uint64_t slot_base = bus_base_ + i * bus_stride_;
        if (mmio_r32(slot_base + VMMIO_MAGIC) != 0x74726976) continue;
        if (mmio_r32(slot_base + VMMIO_DEVICE_ID) != VIRTIO_DEV_ID_INPUT) continue;
        if (devices_[device_count_].init(slot_base)) {
            ++device_count_;
            input_logf("[input] virtio-input ready slot=0x%lx count=%lu\n",
                       static_cast<unsigned long>(slot_base),
                       static_cast<unsigned long>(device_count_));
        } else {
            input_logf("[input] virtio-input failed slot=0x%lx\n",
                       static_cast<unsigned long>(slot_base));
        }
    }
    if (device_count_ == 0) input_log("[input] no virtio-input devices discovered\n");
    return true;
}

void VirtIOInput::poll() {
    last_mouse_.dx = 0;
    last_mouse_.dy = 0;
    last_mouse_.wheel = 0;
    last_mouse_.buttons = state_.mouse_buttons;
    for (size_t i = 0; i < device_count_; ++i) {
        devices_[i].poll(state_, last_mouse_);
    }
}

bool VirtIOInput::get_key(uint8_t key) const {
    return key < MAX_KEYBOARD_KEYS ? state_.key_states[key] : false;
}

void VirtIOInput::get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) {
    x = state_.mouse_x;
    y = state_.mouse_y;
    buttons = state_.mouse_buttons;
}

void VirtIOInput::get_touch_position(int32_t& x, int32_t& y, bool& pressed) {
    x = state_.last_touch.x;
    y = state_.last_touch.y;
    pressed = state_.touch_active;
}

void configure_virtio_input_bus(uint64_t base, size_t stride, size_t slot_count) {
    g_input_driver.configure_bus(base, stride, slot_count);
}

bool init_virtio_input() {
    return g_input_driver.init(0, 0);
}

VirtIOInput* get_input_driver() {
    return &g_input_driver;
}

} // namespace hal::shared::input
