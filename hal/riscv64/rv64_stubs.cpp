// SPDX-License-Identifier: MIT OR Apache-2.0
// rv64-specific context switch shim. The stubs are provided by
// cpp_runtime_stubs.cpp + freestanding_stubs.cpp in CORE_CPP.

#include <cstddef>
#include <cstdint>

#include "miniOS.hpp"

// TCB_Rv64 is now a typedef for kernel::core::TCB (see rv64_sched.cpp) —
// no cast required. Keep this shim so the shared scheduler's context-switch
// seam calls into the rv64-specific asm implementation.
extern "C" void cpu_context_switch_rv64(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb);

extern "C" void cpu_context_switch_impl(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    cpu_context_switch_rv64(old_tcb, new_tcb);
}

namespace kernel {
namespace hal {
namespace sync {

void barrier_dmb() { asm volatile("fence" ::: "memory"); }
void barrier_dsb() { asm volatile("fence" ::: "memory"); }
void barrier_isb() { asm volatile("fence" ::: "memory"); }

} // namespace sync

void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    cpu_context_switch_impl(old_tcb, new_tcb);
}

} // namespace hal
} // namespace kernel
