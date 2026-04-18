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
                // wait_wfi_until_ns. Don't fire software timers (they run on
                // core 0/1) and don't call preemptive_tick — we deliberately
                // keep ticks_total at 0 on these cores so `top` shows zero
                // scheduler ticks as the tickless proof.
                return;
            }
            timer_ops->hardware_timer_irq_fired(core_id);
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
    // rv64 equivalents. `fence rw, rw` covers the load/store ordering
    // dmb/dsb provide on arm64; `fence.i` is the instruction-cache sync
    // analogue of isb (also fences rw so ordered vs subsequent loads).
    void barrier_dmb() { asm volatile("fence rw, rw" ::: "memory"); }
    void barrier_dsb() { asm volatile("fence rw, rw" ::: "memory"); }
    // fence.i requires the Zifencei extension; our `-march=rv64imafdc`
    // baseline doesn't advertise it. `fence rw, rw` serves as a
    // conservative fallback — stronger than isb but legal everywhere.
    void barrier_isb() { asm volatile("fence rw, rw" ::: "memory"); }
#else
    void barrier_dmb() { asm volatile(""             ::: "memory"); }
    void barrier_dsb() { asm volatile(""             ::: "memory"); }
    void barrier_isb() { asm volatile(""             ::: "memory"); }
#endif
}

} // namespace hal
} // namespace kernel
