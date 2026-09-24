// SPDX-License-Identifier: MIT OR Apache-2.0
// Virtio input driver for QEMU - keyboard, mouse, and touch input.

#ifndef HAL_SHARED_VIRTIO_INPUT_HPP
#define HAL_SHARED_VIRTIO_INPUT_HPP

#include "virtio_mmio.hpp"
#include "hal.hpp"

#include <cstddef>
#include <cstdint>

namespace hal::shared::input {

// Transport registers, status bits and descriptor flags (virtio_mmio.hpp).
using namespace ::hal::shared::virtio_mmio;

constexpr size_t MAX_INPUT_DEVS = 4;
constexpr size_t MAX_KEYBOARD_KEYS = 128;
constexpr size_t MAX_TOUCH_POINTS = 4;
constexpr uint32_t VIRTIO_DEV_ID_INPUT = 18;

// QEMU's virtio-input queue size. 16 buffers filled within one 100 ms UI
// tick of pointer motion (3 events per move), and the device then drops.
constexpr size_t VIRTQ_SIZE = 64;

constexpr uint16_t EV_SYN = 0x00;
constexpr uint16_t EV_KEY = 0x01;
constexpr uint16_t EV_REL = 0x02;
constexpr uint16_t EV_ABS = 0x03;
constexpr uint16_t SYN_REPORT = 0;
constexpr uint16_t BTN_LEFT = 0x110;
constexpr uint16_t BTN_MOUSE = 0x110;
constexpr uint16_t BTN_RIGHT = 0x111;
constexpr uint16_t BTN_MIDDLE = 0x112;
constexpr uint16_t BTN_TOUCH = 0x14a;
constexpr uint16_t REL_X = 0x00;
constexpr uint16_t REL_Y = 0x01;
constexpr uint16_t REL_WHEEL = 0x08;
constexpr uint16_t ABS_X = 0x00;
constexpr uint16_t ABS_Y = 0x01;
constexpr uint16_t ABS_MT_SLOT = 0x2f;
constexpr uint16_t ABS_MT_POSITION_X = 0x35;
constexpr uint16_t ABS_MT_POSITION_Y = 0x36;
constexpr uint16_t ABS_MT_TRACKING_ID = 0x39;

struct KeyEvent {
    bool pressed;
    uint8_t key;
    uint32_t timestamp_ms;
};

struct MouseEvent {
    int32_t dx;
    int32_t dy;
    int32_t wheel;
    uint8_t buttons;
    uint32_t timestamp_ms;
};

struct TouchEvent {
    int32_t x;
    int32_t y;
    uint8_t touch_id;
    bool pressed;
    uint32_t timestamp_ms;
};

struct InputState {
    bool keyboard_connected = false;
    bool mouse_connected = false;
    bool touch_connected = false;
    bool key_states[MAX_KEYBOARD_KEYS] = {};
    int32_t mouse_x = 0;
    int32_t mouse_y = 0;
    uint8_t mouse_buttons = 0;
    TouchEvent last_touch{};
    bool touch_active = false;
    // Last pointer motion came from an absolute device (tablet / touch),
    // whose coordinates are normalised to 0..kAbsRange.
    bool pointer_absolute = false;
    // Ordered key / pointer-button edges since the consumer last drained.
    static constexpr size_t QUEUE_LEN = 64;
    kernel::hal::InputEvent queue[QUEUE_LEN]{};
    uint32_t q_head = 0;   // next to pop
    uint32_t q_tail = 0;   // next to push
    // Primary-button / touch state at the last EV_SYN. The edge is queued at
    // EV_SYN so it carries the frame's final coordinates.
    uint8_t buttons_at_syn = 0;
    bool touch_at_syn = false;

    void push(const kernel::hal::InputEvent& ev) {
        if (q_tail - q_head >= QUEUE_LEN) ++q_head;   // full: drop the oldest
        queue[q_tail % QUEUE_LEN] = ev;
        ++q_tail;
    }
};

// Canonical range absolute pointer coordinates are normalised to, whatever
// the device reports in its ABS_INFO (QEMU's tablet happens to use 0..32767).
constexpr int32_t kAbsRange = 32767;

struct VirtqDesc {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} __attribute__((packed));

struct VirtqAvail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[VIRTQ_SIZE];
    uint16_t used_event;
} __attribute__((packed));

struct VirtqUsedElem {
    uint32_t id;
    uint32_t len;
} __attribute__((packed));

struct VirtqUsed {
    uint16_t flags;
    uint16_t idx;
    VirtqUsedElem ring[VIRTQ_SIZE];
    uint16_t avail_event;
} __attribute__((packed));

struct VirtioInputEvent {
    uint16_t type;
    uint16_t code;
    uint32_t value;
} __attribute__((packed));

class VirtIOInputDevice {
public:
    bool init(uint64_t slot_base);
    void poll(InputState& state, MouseEvent& mouse_event);
    bool initialized() const { return initialized_; }

private:
    struct Queue {
        alignas(16) VirtqDesc desc[VIRTQ_SIZE];
        alignas(2) VirtqAvail avail;
        alignas(4) VirtqUsed used;
        uint16_t last_used = 0;
    };

    uint32_t mmio_read(uint32_t off) const;
    void mmio_write(uint32_t off, uint32_t val);
    bool setup_queue(uint32_t queue_idx);
    void post_event_buffers();
    void handle_event(const VirtioInputEvent& event, InputState& state, MouseEvent& mouse_event);
    // Reads ABS_INFO for X/Y (and the multitouch position axes) from the
    // device config space. Axes the device doesn't report keep 0..kAbsRange.
    void query_abs_ranges();
    int32_t normalize_abs(int axis, int32_t value) const;

    uint64_t base_ = 0;
    // [0]=X [1]=Y [2]=MT_X [3]=MT_Y
    int32_t abs_min_[4] = {0, 0, 0, 0};
    int32_t abs_max_[4] = {kAbsRange, kAbsRange, kAbsRange, kAbsRange};
    bool initialized_ = false;
    alignas(4096) Queue event_queue_{};
    alignas(64) VirtioInputEvent event_bufs_[VIRTQ_SIZE]{};
};

class VirtIOInput {
public:
    void configure_bus(uint64_t base, size_t stride, size_t slot_count);
    bool init(uintptr_t base, uint32_t irq);
    void poll();

    bool is_keyboard_connected() const { return state_.keyboard_connected; }
    bool is_mouse_connected() const { return state_.mouse_connected; }
    bool is_touch_connected() const { return state_.touch_connected; }

    bool get_key(uint8_t key) const;
    const MouseEvent& get_mouse() const { return last_mouse_; }
    const TouchEvent& get_touch() const { return state_.last_touch; }
    
    void get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons);
    void get_touch_position(int32_t& x, int32_t& y, bool& pressed);
    uint32_t pointer_abs_range() const { return state_.pointer_absolute ? kAbsRange : 0; }
    bool next_event(kernel::hal::InputEvent& ev) {
        if (state_.q_head == state_.q_tail) return false;
        ev = state_.queue[state_.q_head % InputState::QUEUE_LEN];
        ++state_.q_head;
        return true;
    }

private:
    uintptr_t base_ = 0;
    uint32_t irq_ = 0;
    uint64_t bus_base_ = 0x0A000000ULL;
    size_t bus_stride_ = 0x200;
    size_t bus_slot_count_ = 32;
    InputState state_{};
    MouseEvent last_mouse_{};
    VirtIOInputDevice devices_[MAX_INPUT_DEVS]{};
    size_t device_count_ = 0;
};

void configure_virtio_input_bus(uint64_t base, size_t stride, size_t slot_count);
bool init_virtio_input();
VirtIOInput* get_input_driver();

} // namespace hal::shared::input

#endif
