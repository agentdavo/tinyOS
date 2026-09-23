// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal.cpp
 * @brief Hardware abstraction layer entry point for miniOS v1.7.
 */

#include "hal.hpp"
#include "core.hpp"
#include "miniOS.hpp"
#include "machine/runtime_placement.hpp"
#include "util.hpp"
#include "diag/cpu_load.hpp"
#include <cstdint>

extern "C" {
    void cpu_context_switch_impl(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb);
}

namespace kernel {
namespace hal {

// Arch-parity guard: cpu_arm64.S reserves g_irq_in_progress as
// `.skip 8 * MAX_CORES_ASM` with MAX_CORES_ASM hardcoded to 4. If the C-side
// MAX_CORES grows past that, the IRQ entry/exit asm would index past the
// reserved BSS. Keep them locked together at compile time.
static_assert(kernel::core::MAX_CORES <= 4,
              "cpu_arm64.S reserves g_irq_in_progress for 4 cores (MAX_CORES_ASM); "
              "bump MAX_CORES_ASM in cpu_arm64.S if MAX_CORES grows");

// Defined in the platform-specific HAL translation unit.
extern Platform& get_platform_instance();
Platform* get_platform() { return &get_platform_instance(); }

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

void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    if (!new_tcb) {
        if (kernel::g_platform) {
            kernel::g_platform->panic("cpu_context_switch called with NULL new_tcb", __FILE__, __LINE__);
        } else { for(;;); }
    }
    cpu_context_switch_impl(old_tcb, new_tcb);
}

void fp_scrub_registers() noexcept {
    // The compiler preserves the callee-saved low halves of v8-v15 around
    // this call, so the effective scrub is v0-v7, v16-v31 and FPSR.
    asm volatile(
        "movi   v0.16b,  #0xA5\n  movi v1.16b,  #0xA5\n  movi v2.16b,  #0xA5\n  movi v3.16b,  #0xA5\n"
        "movi   v4.16b,  #0xA5\n  movi v5.16b,  #0xA5\n  movi v6.16b,  #0xA5\n  movi v7.16b,  #0xA5\n"
        "movi   v8.16b,  #0xA5\n  movi v9.16b,  #0xA5\n  movi v10.16b, #0xA5\n  movi v11.16b, #0xA5\n"
        "movi   v12.16b, #0xA5\n  movi v13.16b, #0xA5\n  movi v14.16b, #0xA5\n  movi v15.16b, #0xA5\n"
        "movi   v16.16b, #0xA5\n  movi v17.16b, #0xA5\n  movi v18.16b, #0xA5\n  movi v19.16b, #0xA5\n"
        "movi   v20.16b, #0xA5\n  movi v21.16b, #0xA5\n  movi v22.16b, #0xA5\n  movi v23.16b, #0xA5\n"
        "movi   v24.16b, #0xA5\n  movi v25.16b, #0xA5\n  movi v26.16b, #0xA5\n  movi v27.16b, #0xA5\n"
        "movi   v28.16b, #0xA5\n  movi v29.16b, #0xA5\n  movi v30.16b, #0xA5\n  movi v31.16b, #0xA5\n"
        "msr    fpsr, xzr\n"
        ::: "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11",
            "v12", "v13", "v14", "v15", "v16", "v17", "v18", "v19", "v20", "v21", "v22",
            "v23", "v24", "v25", "v26", "v27", "v28", "v29", "v30", "v31", "memory");
}

uint32_t fp_context_selftest(uint64_t duration_us) noexcept {
    uint64_t mpidr;
    asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    const uint32_t core = static_cast<uint32_t>(mpidr & 0xFF);
    uint64_t freq, start;
    asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    asm volatile("isb; mrs %0, cntvct_el0" : "=r"(start));
    const uint64_t end = start + (freq / 1'000'000ULL) * duration_us;
    const uint64_t p0 = 0x0123456789ABCDEFULL, p1 = 0xFEDCBA9876543210ULL,
                   p2 = 0x1111222233334444ULL, p3 = 0x5555666677778888ULL,
                   p4 = 0x99990000AAAABBBBULL, p5 = 0xCCCCDDDDEEEEFFFFULL,
                   p6 = 0x0F1E2D3C4B5A6978ULL, p7 = 0x8796A5B4C3D2E1F0ULL;
    uint64_t bad;
    if (core < core::MAX_CORES) g_fp_scrub_in_irq[core] = 1;
    // One asm block so the compiler can't touch the registers between load
    // and check. d8/d15 are callee-saved (v8-v15), the rest caller-saved.
    asm volatile(
        "fmov   d0,  %[p0]\n"
        "fmov   d7,  %[p1]\n"
        "fmov   d8,  %[p2]\n"
        "fmov   d15, %[p3]\n"
        "fmov   d16, %[p4]\n"
        "fmov   d23, %[p5]\n"
        "fmov   d24, %[p6]\n"
        "fmov   d31, %[p7]\n"
        "1:\n"
        "mrs    x9, cntvct_el0\n"
        "cmp    x9, %[end]\n"
        "b.lo   1b\n"
        "mov    %[bad], #0\n"
        "fmov   x9, d0\n  cmp x9, %[p0]\n  cset x10, ne\n  orr %[bad], %[bad], x10\n"
        "fmov   x9, d7\n  cmp x9, %[p1]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #1\n"
        "fmov   x9, d8\n  cmp x9, %[p2]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #2\n"
        "fmov   x9, d15\n cmp x9, %[p3]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #3\n"
        "fmov   x9, d16\n cmp x9, %[p4]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #4\n"
        "fmov   x9, d23\n cmp x9, %[p5]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #5\n"
        "fmov   x9, d24\n cmp x9, %[p6]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #6\n"
        "fmov   x9, d31\n cmp x9, %[p7]\n  cset x10, ne\n  orr %[bad], %[bad], x10, lsl #7\n"
        : [bad] "=&r"(bad)
        : [p0] "r"(p0), [p1] "r"(p1), [p2] "r"(p2), [p3] "r"(p3),
          [p4] "r"(p4), [p5] "r"(p5), [p6] "r"(p6), [p7] "r"(p7), [end] "r"(end)
        : "x9", "x10", "v0", "v7", "v8", "v15", "v16", "v23", "v24", "v31", "cc", "memory");
    if (core < core::MAX_CORES) g_fp_scrub_in_irq[core] = 0;
    return static_cast<uint32_t>(bad);
}

extern "C" void hal_irq_handler(uint32_t core_id) {
    // No global lock here — GICC/timer registers are per-CPU, and the scheduler
    // takes its own per-core locks when needed. Taking a global ScopedISRLock
    // across preemptive_tick() is a trap: cpu_context_switch_impl does `eret`
    // into the new thread, so the scoped-lock destructor never runs and the
    // lock stays held forever.
    if (core_id >= core::MAX_CORES || !kernel::g_platform || !kernel::g_platform->get_irq_ops() ||
        !kernel::g_platform->get_timer_ops()) {
        for (;;) asm volatile("nop");
        return;
    }
    // Per-core IRQ counter for the `top` CLI command.
    diag::g_core_counters[core_id].irqs.fetch_add(1, std::memory_order_relaxed);
    if (g_fp_scrub_in_irq[core_id]) fp_scrub_registers();
    IRQControllerOps* irq_ops = kernel::g_platform->get_irq_ops();
    TimerDriverOps* timer_ops = kernel::g_platform->get_timer_ops();
    uint32_t irq_id = irq_ops->ack_irq(core_id);

    if (irq_id < 1020) {
        if (irq_id == kernel::hal::SYSTEM_TIMER_IRQ) {
            // Acknowledge + end_irq BEFORE the schedule call, because schedule
            // may eret into a new thread and never return to this frame.
            timer_ops->ack_core_timer_interrupt(core_id);
            irq_ops->end_irq(core_id, irq_id);
            if (kernel::hal::is_dedicated_rt_core(core_id)) {
                // Tickless RT core: the timer IRQ is strictly a WFI wake for
                // wait_until_ns. Don't call preemptive_tick — we deliberately
                // keep ticks_total at 0 on these cores so `top` shows zero
                // scheduler ticks as the tickless proof.
                return;
            }
            if (kernel::g_scheduler_ptr) {
                kernel::g_scheduler_ptr->preemptive_tick(core_id);
            }
            return;
        }
        kernel::g_platform->handle_device_irq(core_id, irq_id);
        irq_ops->end_irq(core_id, irq_id);
    }
    // irq_id == 1023 = spurious.
}

namespace sync {
#if defined(__aarch64__)
    void barrier_dmb() { asm volatile("dmb sy"  ::: "memory"); }
    void barrier_dsb() { asm volatile("dsb sy"  ::: "memory"); }
    void barrier_isb() { asm volatile("isb"     ::: "memory"); }
#elif defined(__riscv)
    // rv64 equivalents. `fence iorw, iorw` covers the ordering dmb/dsb
    // provide on arm64, including memory vs device (MMIO) accesses — a
    // plain `fence rw, rw` would not order a virtqueue update before the
    // doorbell store. fence.i (the isb analogue) needs Zifencei, which the
    // rv64imafdc baseline doesn't advertise, so isb uses the same fence.
    void barrier_dmb() { asm volatile("fence iorw, iorw" ::: "memory"); }
    void barrier_dsb() { asm volatile("fence iorw, iorw" ::: "memory"); }
    void barrier_isb() { asm volatile("fence iorw, iorw" ::: "memory"); }
#else
    void barrier_dmb() { asm volatile(""             ::: "memory"); }
    void barrier_dsb() { asm volatile(""             ::: "memory"); }
    void barrier_isb() { asm volatile(""             ::: "memory"); }
#endif
}

} // namespace hal
} // namespace kernel
