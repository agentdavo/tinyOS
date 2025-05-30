// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file kernel_globals.cpp
 * @brief Minimal definitions of global kernel variables and early UART output.
 * @details
 *   - Definitions for g_platform, g_scheduler_ptr, spinlocks, etc.
 *   - Stubs for configure_memory_protection.
 *   - Minimal early UART output for debugging/panic.
 *   - __dso_handle for C++ runtime linking.
 */

#include "miniOS.hpp"   // For kernel::g_platform, g_scheduler_ptr, etc
#include "core.hpp"     // For kernel::core types if needed
#include "hal.hpp"      // For kernel::hal types if needed
#include "util.hpp"     // For k_snprintf etc (optional, for future)

namespace kernel {

// Global platform pointer (set by platform init, used everywhere)
hal::Platform* g_platform = nullptr;

// Global scheduler pointer
core::Scheduler* g_scheduler_ptr = nullptr;

// Global spinlocks for IRQ/tracing
core::Spinlock g_trace_lock;
core::Spinlock g_irq_handler_lock;

// Stub for memory protection (do nothing in minimal)
void configure_memory_protection(core::TCB* tcb, bool enable_for_task) {
    (void)tcb;
    (void)enable_for_task;
}

} // namespace kernel
