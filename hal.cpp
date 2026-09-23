// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal.cpp
 * @brief Arch-neutral HAL glue shared by both ports: platform lookup,
 *        dedicated-RT-core policy, context-switch entry, barriers.
 *        arm64 IRQ dispatch lives in hal/arm64/hal_arm64_irq.cpp.
 */

#include "hal.hpp"
#include "core.hpp"
#include "miniOS.hpp"
#include "machine/runtime_placement.hpp"
#include "util.hpp"
#include <cstdint>

extern "C" {
    // arm64: cpu_arm64.S. rv64: rv64_stubs.cpp (wraps cpu_rv64.S).
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
