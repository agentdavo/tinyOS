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

// Global kernel variables (defined in kernel_globals.cpp)
namespace kernel {
    extern hal::Platform* g_platform;
    extern core::Scheduler* g_scheduler_ptr;

    // Function for platform-specific memory protection (MPU/MMU)
    void configure_memory_protection(core::TCB* tcb, bool enable_for_task);

    void get_kernel_stats(hal::UARTDriverOps* uart_ops);


} // namespace kernel

#endif // MINIOS_HPP