// SPDX-License-Identifier: MIT OR Apache-2.0
// Low-level "wait until absolute time" primitives for RT threads on a core
// that owns its physical timer. Two flavours:
//
//   wait_spin_until_ns(target_ns) — busy-poll CNTPCT_EL0. Simplest, works
//       anywhere, but burns CPU (hurts WSL2 because Hyper-V thinks the VCPU
//       is busy and keeps dispatching it).
//
//   wait_wfi_until_ns(target_ns)  — program CNTP_TVAL_EL0 for the delta and
//       issue WFI. Halts the core until the per-CPU physical-timer IRQ
//       fires, then returns. Best for dedicated-core RT threads where the
//       scheduler isn't driving CNTP_* at the same time.
//
// Note: the existing scheduler currently owns CNTP_* for its 200 µs tick. If
// you use wait_wfi_until_ns on a core that also hosts the scheduler tick,
// expect the scheduler to re-arm the timer behind your back. These helpers
// are intended for cores running exactly one RT worker with the scheduler
// tick disabled (planned for cores 2 and 3 once `ec_a`/`ec_b` are the sole
// residents).

#ifndef HAL_ARM64_RT_WAIT_HPP
#define HAL_ARM64_RT_WAIT_HPP

#include <cstdint>

namespace hal::rt {

inline uint64_t now_ns() noexcept {
    uint64_t ticks, freq;
    asm volatile("mrs %0, cntpct_el0" : "=r"(ticks));
    asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return (ticks * 1'000'000'000ULL) / freq;
}

inline uint64_t cntfrq() noexcept {
    uint64_t f; asm volatile("mrs %0, cntfrq_el0" : "=r"(f)); return f;
}

inline void wait_spin_until_ns(uint64_t target_ns) noexcept {
    while (now_ns() < target_ns) asm volatile("yield");
}

// Arm the EL0 physical timer to fire at target_ns, then WFI. Caller must be
// the sole user of CNTP on this core. Returns once the timer IRQ is taken.
inline void wait_wfi_until_ns(uint64_t target_ns) noexcept {
    const uint64_t now = now_ns();
    if (now >= target_ns) return;
    const uint64_t delta_ns = target_ns - now;
    const uint64_t freq     = cntfrq();
    // ticks = delta_ns * freq / 1e9. Integer-safe: delta_ns fits in 34 bits
    // for any realistic deadline, freq is ~26 bits; product fits in 64.
    const uint64_t ticks    = (delta_ns * freq) / 1'000'000'000ULL;
    asm volatile("msr cntp_tval_el0, %0" :: "r"(ticks));
    // Enable, unmasked. (ENABLE=bit0=1, IMASK=bit1=0)
    asm volatile("msr cntp_ctl_el0,  %0" :: "r"(1ULL));
    asm volatile("dsb sy");
    asm volatile("wfi");
    // After wake: the IRQ handler will have run and acked the timer.
}

// Arm via WFE (wait-for-event). Useful if event-signalling is wired through
// another path; for our setup the timer IRQ path is the same as WFI.
inline void wait_wfe_until_ns(uint64_t target_ns) noexcept {
    wait_wfi_until_ns(target_ns);
}

} // namespace hal::rt

#endif
