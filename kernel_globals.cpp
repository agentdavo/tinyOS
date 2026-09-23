// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file kernel_globals.cpp
 * @brief Definitions of global kernel variables.
 * @details
 *   - g_platform, g_scheduler_ptr, the fp self-test scrub flags.
 *   - Stub for configure_memory_protection.
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

// fp_context_selftest hook (see hal.hpp). Shared so both arches' IRQ
// handlers read the same per-core flag.
volatile uint8_t hal::g_fp_scrub_in_irq[core::MAX_CORES] = {};

// Stub for memory protection (do nothing in minimal)
void configure_memory_protection(core::TCB* tcb, bool enable_for_task) {
    (void)tcb;
    (void)enable_for_task;
}

} // namespace kernel
