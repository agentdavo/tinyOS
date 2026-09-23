// SPDX-License-Identifier: MIT OR Apache-2.0
// rv64-specific glue between the shared scheduler and the rv64 trap/context
// switch primitives in cpu_rv64.S.
//
// Two callers reach cpu_context_switch_impl:
//
//   (A) Voluntary: Scheduler::yield() -> Scheduler::schedule() from a thread
//       context (no in-flight trap). cpu_context_switch_rv64 saves all GPRs +
//       FP state into old_tcb and restores them from new_tcb.
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
// `iorw`: these also order memory against device (MMIO) accesses, e.g. a
// virtqueue update before the QUEUE_NOTIFY doorbell. `fence rw, rw` only
// orders memory against memory.
void barrier_dmb() { asm volatile("fence iorw, iorw" ::: "memory"); }
void barrier_dsb() { asm volatile("fence iorw, iorw" ::: "memory"); }
void barrier_isb() { asm volatile("fence iorw, iorw" ::: "memory"); }
} // namespace sync

void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    cpu_context_switch_impl(old_tcb, new_tcb);
}

void fp_scrub_registers() noexcept {
    // The compiler preserves the callee-saved fs0-fs11 around this call, so
    // the effective scrub is the ft*/fa* registers and fcsr.
    const uint64_t junk = 0xA5A5A5A5A5A5A5A5ULL;
    asm volatile(
        ".irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31\n"
        "fmv.d.x f\\n, %0\n"
        ".endr\n"
        "fscsr   zero\n"
        :: "r"(junk)
        : "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9", "f10", "f11",
          "f12", "f13", "f14", "f15", "f16", "f17", "f18", "f19", "f20", "f21", "f22",
          "f23", "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31", "memory");
}

uint32_t fp_context_selftest(uint64_t duration_us) noexcept {
    uint64_t hart;
    asm volatile("csrr %0, mhartid" : "=r"(hart));
    uint64_t start;
    asm volatile("rdtime %0" : "=r"(start));
    const uint64_t end = start + (::hal::qemu_virt_rv64::TIMEBASE_HZ / 1'000'000ULL) * duration_us;
    const uint64_t p0 = 0x0123456789ABCDEFULL, p1 = 0xFEDCBA9876543210ULL,
                   p2 = 0x1111222233334444ULL, p3 = 0x5555666677778888ULL,
                   p4 = 0x99990000AAAABBBBULL, p5 = 0xCCCCDDDDEEEEFFFFULL,
                   p6 = 0x0F1E2D3C4B5A6978ULL, p7 = 0x8796A5B4C3D2E1F0ULL;
    uint64_t bad;
    if (hart < kernel::core::MAX_CORES) g_fp_scrub_in_irq[hart] = 1;
    // One asm block so the compiler can't touch the registers between load
    // and check. f8/f9/f18/f27 are callee-saved (fs0/fs1/fs2/fs11).
    asm volatile(
        "fmv.d.x f0,  %[p0]\n"
        "fmv.d.x f7,  %[p1]\n"
        "fmv.d.x f8,  %[p2]\n"
        "fmv.d.x f9,  %[p3]\n"
        "fmv.d.x f18, %[p4]\n"
        "fmv.d.x f27, %[p5]\n"
        "fmv.d.x f28, %[p6]\n"
        "fmv.d.x f31, %[p7]\n"
        "1:\n"
        "rdtime  t0\n"
        "bltu    t0, %[end], 1b\n"
        "li      %[bad], 0\n"
        "fmv.x.d t0, f0\n  xor t0, t0, %[p0]\n  snez t0, t0\n                  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f7\n  xor t0, t0, %[p1]\n  snez t0, t0\n  slli t0, t0, 1\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f8\n  xor t0, t0, %[p2]\n  snez t0, t0\n  slli t0, t0, 2\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f9\n  xor t0, t0, %[p3]\n  snez t0, t0\n  slli t0, t0, 3\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f18\n xor t0, t0, %[p4]\n  snez t0, t0\n  slli t0, t0, 4\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f27\n xor t0, t0, %[p5]\n  snez t0, t0\n  slli t0, t0, 5\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f28\n xor t0, t0, %[p6]\n  snez t0, t0\n  slli t0, t0, 6\n  or %[bad], %[bad], t0\n"
        "fmv.x.d t0, f31\n xor t0, t0, %[p7]\n  snez t0, t0\n  slli t0, t0, 7\n  or %[bad], %[bad], t0\n"
        : [bad] "=&r"(bad)
        : [p0] "r"(p0), [p1] "r"(p1), [p2] "r"(p2), [p3] "r"(p3),
          [p4] "r"(p4), [p5] "r"(p5), [p6] "r"(p6), [p7] "r"(p7), [end] "r"(end)
        : "t0", "f0", "f7", "f8", "f9", "f18", "f27", "f28", "f31", "memory");
    if (hart < kernel::core::MAX_CORES) g_fp_scrub_in_irq[hart] = 0;
    return static_cast<uint32_t>(bad);
}

} // namespace hal
} // namespace kernel
