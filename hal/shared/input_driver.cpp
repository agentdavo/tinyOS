// SPDX-License-Identifier: MIT OR Apache-2.0

#include "hal/shared/input_driver.hpp"
#include "hal/shared/virtio_input.hpp"

namespace hal::shared::input {

bool InputDriver::init() {
    return init_virtio_input();
}

void InputDriver::poll() {
    if (auto* driver = get_input_driver()) driver->poll();
    if (usb_ && usb_->initialized) ::hal::shared::xhci::poll(*usb_);
}

bool InputDriver::is_keyboard_connected() {
    if (auto* driver = get_input_driver()) {
        if (driver->is_keyboard_connected()) return true;
    }
    return usb_ && usb_->initialized && ::hal::shared::xhci::keyboard_connected(*usb_);
}

bool InputDriver::is_mouse_connected() {
    auto* driver = get_input_driver();
    return driver ? driver->is_mouse_connected() : false;
}

bool InputDriver::is_touch_connected() {
    auto* driver = get_input_driver();
    return driver ? driver->is_touch_connected() : false;
}

bool InputDriver::get_key_state(uint8_t key) {
    if (auto* driver = get_input_driver()) {
        if (driver->get_key(key)) return true;
    }
    return usb_ && usb_->initialized && ::hal::shared::xhci::key_pressed(*usb_, key);
}

void InputDriver::get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) {
    auto* driver = get_input_driver();
    if (!driver) {
        x = 0;
        y = 0;
        buttons = 0;
        return;
    }
    driver->get_mouse_position(x, y, buttons);
}

void InputDriver::get_touch_position(int32_t& x, int32_t& y, bool& pressed) {
    auto* driver = get_input_driver();
    if (!driver) {
        x = 0;
        y = 0;
        pressed = false;
        return;
    }
    driver->get_touch_position(x, y, pressed);
}

uint32_t InputDriver::pointer_abs_range() {
    auto* driver = get_input_driver();
    return driver ? driver->pointer_abs_range() : 0;
}

bool InputDriver::next_event(kernel::hal::InputEvent& ev) {
    // virtio-input queues its edges. The xHCI keyboard only exposes held
    // state; the UI's per-tick diff still covers it.
    auto* driver = get_input_driver();
    return driver && driver->next_event(ev);
}

} // namespace hal::shared::input
