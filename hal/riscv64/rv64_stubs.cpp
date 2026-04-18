// SPDX-License-Identifier: MIT OR Apache-2.0
// rv64-specific context switch shim. The stubs are provided by
// cpp_runtime_stubs.cpp + freestanding_stubs.cpp in CORE_CPP.

#include <cstddef>
#include <cstdint>

#include "miniOS.hpp"

namespace hal::qemu_virt_rv64 {
struct TCB_Rv64;
}

extern "C" void cpu_context_switch_rv64(hal::qemu_virt_rv64::TCB_Rv64* old_tcb, hal::qemu_virt_rv64::TCB_Rv64* new_tcb);

extern "C" void cpu_context_switch_impl(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    cpu_context_switch_rv64(reinterpret_cast<hal::qemu_virt_rv64::TCB_Rv64*>(old_tcb),
                            reinterpret_cast<hal::qemu_virt_rv64::TCB_Rv64*>(new_tcb));
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
