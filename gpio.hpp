// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file gpio.hpp
 * @brief GPIO subsystem header for miniOS v1.7.
 * @details
 * Defines GPIO management for miniOS. Updated in v1.7 with improved error handling
 * and modern C++20 practices.
 *
 * @version 1.7
 * @see gpio.cpp, core.hpp, hal.hpp
 */

#ifndef GPIO_HPP
#define GPIO_HPP

#include "core.hpp"
#include "hal.hpp"

namespace gpio {

class GPIOManager {
public:
    GPIOManager() = default;
    bool configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode);
    bool set_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state);
    kernel::hal::gpio::PinState read_pin(uint32_t bank, uint32_t pin);
private:
    struct GPIOBank {
        kernel::core::Spinlock lock;
    };
    std::array<GPIOBank, kernel::core::GPIO_BANKS> banks_;
};

extern GPIOManager g_gpio_manager;

} // namespace gpio

#endif // GPIO_HPP