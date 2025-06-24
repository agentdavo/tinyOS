// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file gpio.cpp
 * @brief GPIO subsystem implementation for miniOS v1.7.
 * @details
 * Implements a GPIO manager for controlling 4 banks of 64 pins each, with per-bank threads for
 * real-time interrupt handling (5ms deadline), input/output configuration, state read/write, and
 * interrupt support via HAL. Integrates with CLI for pin management. Updated in v1.7 with restored
 * v1.6 features (threaded banks, interrupt counting), improved error handling, clearer diagnostics,
 * and modern C++20 practices.
 *
 * C++20 features:
 * - std::atomic for thread-safe state
 * - std::string_view for string formatting
 *
 * @version 1.7
 * @see gpio.hpp, miniOS.hpp, util.hpp, cli.hpp
 */

#include "gpio.hpp"
#include "util.hpp"
#include <cstring>

namespace gpio {

GPIOManager g_gpio_manager;

GPIOManager::GPIOManager() : gpio_ops_(nullptr), initialized_(false) {
    for (auto& thread : bank_threads_) thread = nullptr;
    for (auto& bank : banks_) {
        for (auto& pin : bank.pin_output_enabled) pin = false;
        for (auto& pin : bank.pin_state) pin = false;
        for (size_t i = 0; i < MAX_INTERRUPTS; ++i) {
            bank.interrupt_enabled[i] = false;
            bank.interrupt_rising_edge[i] = false;
            bank.interrupt_pins[i] = 0;
        }
        bank.interrupt_count = 0;
    }
}

bool GPIOManager::is_valid_pin(uint32_t bank, uint32_t pin) const noexcept {
    return bank < NUM_BANKS && pin < PINS_PER_BANK;
}

bool GPIOManager::init(kernel::hal::gpio::GPIODriverOps* ops) {
    if (initialized_.load(std::memory_order_relaxed) || !ops || !kernel::g_platform || !kernel::g_scheduler_ptr) {
        return false;
    }
    gpio_ops_ = ops;
    initialized_.store(true, std::memory_order_relaxed);

    // Create per-bank threads with 5ms deadline
    for (uint8_t i = 0; i < NUM_BANKS; ++i) {
        char name[16];
        kernel::util::k_snprintf(name, sizeof(name), "GPIOBank%u", i);
        bank_threads_[i] = kernel::g_scheduler_ptr->create_thread(bank_thread_entry,
                                                                 reinterpret_cast<void*>(static_cast<uintptr_t>(i)),
                                                                 6, -1, name, false, 5000);
        if (!bank_threads_[i]) {
            initialized_.store(false, std::memory_order_relaxed);
            if (kernel::g_platform) {
                kernel::g_platform->panic("GPIO Thread Creation Failed", __FILE__, __LINE__);
            }
            return false;
        }
    }
    return true;
}

bool GPIOManager::configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode) {
    if (!initialized_.load(std::memory_order_relaxed) || !gpio_ops_ || !is_valid_pin(bank, pin)) {
        return false;
    }
    kernel::ScopedLock lock(banks_[bank].lock);
    if (!gpio_ops_->configure_pin(bank, pin, mode)) return false;
    banks_[bank].pin_output_enabled[pin].store(mode == kernel::hal::gpio::PinMode::OUTPUT, std::memory_order_release);
    return true;
}

bool GPIOManager::set_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state) {
    if (!initialized_.load(std::memory_order_relaxed) || !gpio_ops_ || !is_valid_pin(bank, pin)) {
        return false;
    }
    kernel::ScopedLock lock(banks_[bank].lock);
    if (!banks_[bank].pin_output_enabled[pin].load(std::memory_order_acquire)) return false;
    if (!gpio_ops_->set_pin(bank, pin, state)) return false;
    banks_[bank].pin_state[pin].store(state == kernel::hal::gpio::PinState::HIGH, std::memory_order_release);
    return true;
}

kernel::hal::gpio::PinState GPIOManager::read_pin(uint32_t bank, uint32_t pin) {
    if (!initialized_.load(std::memory_order_relaxed) || !gpio_ops_ || !is_valid_pin(bank, pin)) {
        return kernel::hal::gpio::PinState::LOW;
    }
    kernel::ScopedLock lock(banks_[bank].lock);
    kernel::hal::gpio::PinState state = gpio_ops_->read_pin(bank, pin);
    banks_[bank].pin_state[pin].store(state == kernel::hal::gpio::PinState::HIGH, std::memory_order_release);
    return state;
}

bool GPIOManager::enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) {
    if (!initialized_.load(std::memory_order_relaxed) || !gpio_ops_ || !is_valid_pin(bank, pin) ||
        banks_[bank].interrupt_count >= MAX_INTERRUPTS) {
        return false;
    }
    kernel::ScopedLock lock(banks_[bank].lock);
    if (!gpio_ops_->enable_interrupt(bank, pin, rising_edge)) return false;
    uint8_t idx = banks_[bank].interrupt_count++;
    banks_[bank].interrupt_enabled[idx].store(true, std::memory_order_release);
    banks_[bank].interrupt_rising_edge[idx].store(rising_edge, std::memory_order_release);
    banks_[bank].interrupt_pins[idx] = pin;
    return true;
}

void GPIOManager::handle_interrupt(uint8_t bank) {
    if (bank >= NUM_BANKS || !gpio_ops_) return;
    kernel::ScopedLock lock(banks_[bank].lock);
    for (uint8_t i = 0; i < banks_[bank].interrupt_count; ++i) {
        if (banks_[bank].interrupt_enabled[i].load(std::memory_order_acquire)) {
            uint8_t pin = banks_[bank].interrupt_pins[i];
            kernel::hal::gpio::PinState state = gpio_ops_->read_pin(bank, pin);
            bool rising = banks_[bank].interrupt_rising_edge[i].load(std::memory_order_acquire);
            if ((rising && state == kernel::hal::gpio::PinState::HIGH) ||
                (!rising && state == kernel::hal::gpio::PinState::LOW)) {
                kernel::trace_event("GPIO:Interrupt", (bank << 8) | pin, rising);
            }
        }
    }
}

void GPIOManager::bank_thread_entry(void* arg) {
    uint8_t bank = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(arg));
    if (!kernel::g_platform || !kernel::g_scheduler_ptr) return;
    while (true) {
        g_gpio_manager.handle_interrupt(bank);
        kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
    }
}

} // namespace gpio