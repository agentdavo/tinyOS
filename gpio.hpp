// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file gpio.hpp
 * @brief GPIO subsystem header for miniOS v1.7.
 * @details
 * Defines a GPIO manager for controlling 4 banks of 64 pins each, with per-bank threads for
 * real-time interrupt handling, input/output modes, state read/write, and interrupt support via
 * HAL. Integrates with CLI for pin configuration and control. Updated in v1.7 with improved error
 * handling, clearer documentation, and modern C++20 practices, retaining all v1.6 functionality
 * including threaded banks and interrupt counting.
 *
 * C++20 features:
 * - std::atomic for thread-safe state
 * - std::optional for safer returns
 *
 * @version 1.7
 * @see gpio.cpp, miniOS.hpp, util.hpp, cli.hpp
 */

#ifndef GPIO_HPP
#define GPIO_HPP

#include "miniOS.hpp"
#include <atomic>
#include <array>
#include <optional>

namespace gpio {

constexpr size_t NUM_BANKS = 4;
constexpr size_t PINS_PER_BANK = 64;
constexpr size_t MAX_INTERRUPTS = 8;

/**
 * @brief GPIO manager class.
 */
class GPIOManager {
public:
    GPIOManager();

    /**
     * @brief Initializes the GPIO subsystem with per-bank threads.
     * @param ops GPIO driver operations from HAL
     * @return True if initialized, false otherwise
     */
    bool init(kernel::hal::gpio::GPIODriverOps* ops);

    /**
     * @brief Configures a GPIO pin as input or output.
     * @param bank Bank index (0-3)
     * @param pin Pin index (0-63)
     * @param mode Input or output mode
     * @return True if configured, false if invalid or failed
     */
    bool configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode);

    /**
     * @brief Sets a GPIO pin state (high/low).
     * @param bank Bank index (0-3)
     * @param pin Pin index (0-63)
     * @param state High or low state
     * @return True if set, false if invalid or failed
     */
    bool set_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state);

    /**
     * @brief Reads a GPIO pin state.
     * @param bank Bank index (0-3)
     * @param pin Pin index (0-63)
     * @return Pin state (HIGH or LOW), or LOW if invalid
     */
    kernel::hal::gpio::PinState read_pin(uint32_t bank, uint32_t pin);

    /**
     * @brief Enables interrupt on a GPIO pin.
     * @param bank Bank index (0-3)
     * @param pin Pin index (0-63)
     * @param rising_edge True for rising edge, false for falling edge
     * @return True if enabled, false if invalid or limit reached
     */
    bool enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge);

private:
    /**
     * @brief Per-bank data structure.
     */
    struct GPIOBank {
        kernel::Spinlock lock; ///< Bank lock
        std::array<std::atomic<bool>, PINS_PER_BANK> pin_output_enabled; ///< Output mode flags
        std::array<std::atomic<bool>, PINS_PER_BANK> pin_state; ///< Pin states (true=high)
        std::array<std::atomic<bool>, MAX_INTERRUPTS> interrupt_enabled; ///< Interrupt flags
        std::array<std::atomic<bool>, MAX_INTERRUPTS> interrupt_rising_edge; ///< Rising edge flags
        std::array<uint8_t, MAX_INTERRUPTS> interrupt_pins; ///< Pins with interrupts
        uint8_t interrupt_count = 0; ///< Number of active interrupts
    };

    std::array<GPIOBank, NUM_BANKS> banks_; ///< Per-bank data
    std::array<kernel::TCB*, NUM_BANKS> bank_threads_; ///< Per-bank threads
    kernel::hal::gpio::GPIODriverOps* gpio_ops_; ///< HAL GPIO operations
    std::atomic<bool> initialized_; ///< Initialization state

    /**
     * @brief Validates bank and pin indices.
     * @param bank Bank index
     * @param pin Pin index
     * @return True if valid, false otherwise
     */
    bool is_valid_pin(uint32_t bank, uint32_t pin) const noexcept;

    /**
     * @brief Handles interrupts for a bank.
     * @param bank Bank index
     */
    void handle_interrupt(uint8_t bank);

    /**
     * @brief Per-bank thread entry point.
     * @param arg Bank index
     */
    static void bank_thread_entry(void* arg);
};

extern GPIOManager g_gpio_manager; ///< Global GPIO manager instance

} // namespace gpio

#endif // GPIO_HPP