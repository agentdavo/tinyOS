// SPDX-License-Identifier: MIT OR Apache-2.0
// Low-level "wait until absolute time" primitives for RT threads on a hart
// that owns its CLINT mtimecmp slot. Mirrors hal/arm64/rt_wait.hpp.
//
//   wait_spin_until_ns(target_ns) — busy-poll mtime via rdtime. Burns the
//       hart; fine for determinism microbenchmarks, bad for WSL2 dispatch.
//
//   wait_wfi_until_ns(target_ns)  — program this hart's mtimecmp for the
//       deadline, enable MTIE in mie and MIE in mstatus, issue wfi. Halts
//       the hart until the timer IRQ fires. Caller must be the sole user of
//       this hart's mtimecmp slot.
//
// wfi can wake spuriously per spec; callers must re-check the deadline.

#ifndef HAL_RV64_RT_WAIT_HPP
#define HAL_RV64_RT_WAIT_HPP

#include <cstdint>

namespace hal::rt {

// QEMU virt: 10 MHz timebase. Override by editing or hooking via TimerDriver.
constexpr uint64_t RV64_TIMEBASE_HZ = 10'000'000;
constexpr uint64_t CLINT_MTIMECMP_BASE = 0x02004000;

inline uint64_t now_ticks() noexcept {
    uint64_t t; asm volatile("rdtime %0" : "=r"(t)); return t;
}

inline uint64_t now_ns() noexcept {
    return (now_ticks() * 1'000'000'000ULL) / RV64_TIMEBASE_HZ;
}

inline uint32_t hartid() noexcept {
    uint64_t h; asm volatile("csrr %0, mhartid" : "=r"(h));
    return static_cast<uint32_t>(h);
}

inline void wait_spin_until_ns(uint64_t target_ns) noexcept {
    while (now_ns() < target_ns) asm volatile("nop");
}

inline void wait_wfi_until_ns(uint64_t target_ns) noexcept {
    const uint32_t h = hartid();
    auto* mtimecmp = reinterpret_cast<volatile uint64_t*>(
        CLINT_MTIMECMP_BASE + 8ULL * h);
    for (;;) {
        const uint64_t now_t  = now_ticks();
        const uint64_t now_n  = (now_t * 1'000'000'000ULL) / RV64_TIMEBASE_HZ;
        if (now_n >= target_ns) return;
        const uint64_t dns    = target_ns - now_n;
        const uint64_t dtick  = (dns * RV64_TIMEBASE_HZ) / 1'000'000'000ULL;
        // 8-byte MMIO store — do NOT split into two 4-byte writes.
        *mtimecmp = now_t + (dtick ? dtick : 1);
        // mie.MTIE = bit 7, mstatus.MIE = bit 3.
        uint64_t mtie = 1ULL << 7;
        asm volatile("csrs mie, %0" :: "r"(mtie) : "memory");
        asm volatile("csrsi mstatus, 0x8" ::: "memory");
        asm volatile("wfi");
        // Loop: re-check deadline (spurious wake or early mtimecmp overshoot).
    }
}

inline void wait_wfe_until_ns(uint64_t target_ns) noexcept {
    wait_wfi_until_ns(target_ns);
}

} // namespace hal::rt

#endif // HAL_RV64_RT_WAIT_HPP
