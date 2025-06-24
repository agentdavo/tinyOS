// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file miniOS.hpp
 * @brief Main internal kernel header for miniOS v1.7.
 * @details
 * Includes core RTOS headers and global definitions essential for kernel operation.
 * This file is used by kernel modules themselves.
 *
 * @version 1.7
 */
#ifndef MINIOS_HPP
#define MINIOS_HPP

#include "core.hpp" // Defines kernel::core types
#include "hal.hpp"  // Defines kernel::hal interfaces
#include <cstdint>  // For uintptr_t
#include <cstddef>  // For size_t

// Convenient global aliases for common kernel constants
inline constexpr size_t MAX_NAME_LENGTH = kernel::core::MAX_NAME_LENGTH;
inline constexpr size_t GPIO_BANKS = kernel::hal::gpio::NUM_BANKS;
inline constexpr size_t GPIO_PINS_PER_BANK = kernel::hal::gpio::PINS_PER_BANK;

// Global kernel variables (defined in kernel_globals.cpp)
namespace kernel {
    extern hal::Platform* g_platform;
    extern core::Scheduler* g_scheduler_ptr;
    extern core::Spinlock g_trace_lock; // Used by global trace_event and dump_trace_buffer in core.cpp
    extern core::Spinlock g_irq_handler_lock; // Used by hal_irq_handler

    // Function for platform-specific memory protection (MPU/MMU)
    void configure_memory_protection(core::TCB* tcb, bool enable_for_task);

    /**
     * @brief Records a legacy global trace event.
     * @param event_str Description of the event. Must be a long-lived string literal.
     * @param arg1 First argument for the event.
     * @param arg2 Second argument for the event.
     * @note This is distinct from trace::TraceManager.
     */
    void trace_event(const char* event_str, uintptr_t arg1, uintptr_t arg2);

    void dump_trace_buffer(hal::UARTDriverOps* uart_ops);
    void get_kernel_stats(hal::UARTDriverOps* uart_ops);


} // namespace kernel

#endif // MINIOS_HPP