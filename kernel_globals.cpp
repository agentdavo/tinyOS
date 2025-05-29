// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file kernel_globals.cpp
 * @brief Definitions of global kernel variables and stubs for miniOS v1.7.
 * @details
 * Provides definitions for global variables declared in miniOS.hpp and stub
 * implementations for functions required by the minimal kernel.
 *
 * @version 1.7
 * @see miniOS.hpp, core.hpp, hal.hpp
 */

#include "core.hpp"   // For kernel::core types if needed for stubs
#include "hal.hpp"    // For kernel::hal types if needed for stubs
#include "util.hpp"
#include "miniOS.hpp" // For declarations of g_platform, g_scheduler_ptr, etc.

namespace kernel {

hal::Platform* g_platform = nullptr;
core::Scheduler* g_scheduler_ptr = nullptr;
core::Spinlock g_trace_lock;
core::Spinlock g_irq_handler_lock;

void configure_memory_protection(core::TCB* tcb, bool enable_for_task) {
    (void)tcb; 
    (void)enable_for_task; 

    // Example logging (only if platform and UART are up)
    // if (g_platform && g_platform->get_uart_ops()) {
    //     char buf[128];
    //     kernel::util::k_snprintf(buf, sizeof(buf), "MemProtect: %s for TCB:%p Name:'%s'\n",
    //                   enable_for_task ? "ON" : "OFF",
    //                   (void*)tcb,
    //                   (tcb && tcb->name[0] != '\0') ? tcb->name : "N/A");
    //     g_platform->get_uart_ops()->puts(buf);
    // }
}

} // namespace kernel

// Define __dso_handle for C++ runtime.
// For a statically linked bare-metal kernel, this is usually just a non-extern definition.
// The 'extern "C"' is for name mangling.
extern "C" {
    void* __dso_handle = nullptr; // Definition, not extern declaration + definition
}