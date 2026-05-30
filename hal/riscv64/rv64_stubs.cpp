// SPDX-License-Identifier: MIT OR Apache-2.0
// rv64-specific glue between the shared scheduler and the rv64 trap/context
// switch primitives in cpu_rv64.S.
//
// Two callers reach cpu_context_switch_impl:
//
//   (A) Voluntary: Scheduler::yield() -> Scheduler::schedule() from a thread
//       context (no in-flight trap). cpu_context_switch_rv64 saves callee-saved
//       state into old_tcb and reloads from new_tcb.
//
//   (B) Preemptive: trap_entry (cpu_rv64.S) saves the full interrupted state
//       into g_per_cpu_data[hart].current_thread, then calls into C; the C
//       handler may invoke Scheduler::preemptive_tick which calls
//       cpu_context_switch_impl. The trap-exit asm reloads from
//       g_per_cpu_data[hart].current_thread (which the scheduler has by then
//       repointed at the new TCB), so the call has nothing to do here — it
//       must just return so the C call chain unwinds to the trap exit.
//
// hal::qemu_virt_rv64::g_irq_in_progress[hart] discriminates the two cases;
// the trap dispatcher sets it across the preemptive path.

#include <cstddef>
#include <cstdint>

#include "miniOS.hpp"
#include "core.hpp"
#include "hal_qemu_rv64.hpp"

extern "C" void cpu_context_switch_rv64(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb);

// Per-hart "we're inside the trap dispatcher" flag. Mirrors arm64's
// g_irq_in_progress[] role: lets cpu_context_switch_impl know the outgoing
// thread's state was already spilled by the trap entry path so skip the
// voluntary save/restore. Defined in hal_qemu_rv64.cpp; declared here so the
// shim can read it without dragging the namespace in.
namespace hal::qemu_virt_rv64 {
extern "C" volatile uint64_t g_irq_in_progress[MAX_HARTS];
} // namespace hal::qemu_virt_rv64

// Arch-parity guard: cpu_rv64.S indexes g_irq_in_progress[] by (hartid * 8)
// for every hart the scheduler runs. If MAX_HARTS ever drops below MAX_CORES
// the trap handler would index past the array, so pin the relationship at
// compile time rather than discover it as a memory stomp at boot.
static_assert(hal::qemu_virt_rv64::MAX_HARTS >= kernel::core::MAX_CORES,
              "g_irq_in_progress[] must cover every core the rv64 trap asm indexes");

extern "C" void cpu_context_switch_impl(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    uint64_t hart;
    asm volatile("csrr %0, mhartid" : "=r"(hart));
    if (hart < hal::qemu_virt_rv64::MAX_HARTS &&
        hal::qemu_virt_rv64::g_irq_in_progress[hart]) {
        return;
    }
    cpu_context_switch_rv64(old_tcb, new_tcb);
}

// is_dedicated_rt_core lives in hal.cpp on arm64; rv64 doesn't link hal.cpp
// (it would pull in arm64-tied symbols), so re-provide the same logic here.
// Keep behaviour bit-for-bit identical so dedicated-RT-core gating works the
// same on both arches.
#include "machine/runtime_placement.hpp"

namespace kernel {
namespace hal {

bool is_dedicated_rt_core(uint32_t core_id) noexcept {
    machine::placement::Config cfg{};
    machine::placement::g_service.snapshot(cfg);
    const uint32_t num_cores = g_platform ? g_platform->get_num_cores() : kernel::core::MAX_CORES;
    const auto sanitize = [&](uint8_t requested) noexcept -> uint32_t {
        return machine::placement::g_service.sanitize_core(requested, num_cores);
    };

    const uint32_t ec_a_core = sanitize(cfg.ec_a_core);
#if MINIOS_FAKE_SLAVE
    const uint32_t rt_peer_core = sanitize(cfg.fake_slave_core);
#else
    const uint32_t rt_peer_core = sanitize(cfg.ec_b_core);
#endif
    if (core_id != ec_a_core && core_id != rt_peer_core) {
        return false;
    }

    const uint32_t shared_general_cores[] = {
        sanitize(cfg.cli_core),
        sanitize(cfg.uart_io_core),
        sanitize(cfg.ui_core),
        sanitize(cfg.motion_core),
        sanitize(cfg.gcode_core),
        sanitize(cfg.macro_core),
        sanitize(cfg.ladder_core),
        sanitize(cfg.probe_core),
        sanitize(cfg.bus_config_core),
    };
    for (uint32_t shared_core : shared_general_cores) {
        if (shared_core == core_id) return false;
    }
    return true;
}

namespace sync {
void barrier_dmb() { asm volatile("fence rw, rw" ::: "memory"); }
void barrier_dsb() { asm volatile("fence rw, rw" ::: "memory"); }
void barrier_isb() { asm volatile("fence rw, rw" ::: "memory"); }
} // namespace sync

void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    cpu_context_switch_impl(old_tcb, new_tcb);
}

} // namespace hal
} // namespace kernel
