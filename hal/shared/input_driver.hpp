// SPDX-License-Identifier: MIT OR Apache-2.0
// Platform InputOps for QEMU virt, shared by both arches: every virtio-input
// device (keyboard / tablet / mouse) plus an optional xHCI USB keyboard.
// It used to be two near-identical per-arch classes, and the arm64 copy
// never polled xHCI, so a USB keyboard only worked on rv64.

#ifndef HAL_SHARED_INPUT_DRIVER_HPP
#define HAL_SHARED_INPUT_DRIVER_HPP

#include "hal.hpp"
#include "hal/shared/xhci.hpp"

namespace hal::shared::input {

class InputDriver final : public kernel::hal::InputOps {
public:
    // `usb` may be null (no xHCI keyboard path); it is only polled after
    // the platform's USB controller has initialised it.
    explicit InputDriver(::hal::shared::xhci::ControllerState* usb) : usb_(usb) {}
    bool init() override;
    void poll() override;
    bool is_keyboard_connected() override;
    bool is_mouse_connected() override;
    bool is_touch_connected() override;
    bool get_key_state(uint8_t key) override;
    void get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) override;
    void get_touch_position(int32_t& x, int32_t& y, bool& pressed) override;
    uint32_t pointer_abs_range() override;
    bool next_event(kernel::hal::InputEvent& ev) override;

private:
    ::hal::shared::xhci::ControllerState* usb_ = nullptr;
};

} // namespace hal::shared::input

#endif
