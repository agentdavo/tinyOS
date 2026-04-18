// SPDX-License-Identifier: MIT OR Apache-2.0
// Per-core activity counters used by the `top` / `ttop` CLI commands.
//
// Three counters per core:
//   * irqs        — bumped on every entry to hal_irq_handler.
//   * ticks_total — bumped on every preemptive_tick.
//   * ticks_busy  — bumped on every preemptive_tick where the running
//                   thread is not this core's idle thread.
//
// `busy %` is computed as (ticks_busy * 100) / ticks_total.
//
// All counters are std::atomic<uint64_t> with relaxed ordering — we only need
// monotonic increments and a coherent read; readers tolerate a small skew.
//
// Freestanding C++20.

#ifndef DIAG_CPU_LOAD_HPP
#define DIAG_CPU_LOAD_HPP

#include "core.hpp"
#include <array>
#include <atomic>
#include <cstdint>

namespace diag {

struct CoreCounters {
    std::atomic<uint64_t> irqs{0};
    std::atomic<uint64_t> ticks_busy{0};
    std::atomic<uint64_t> ticks_total{0};
};

extern std::array<CoreCounters, kernel::core::MAX_CORES> g_core_counters;

} // namespace diag

#endif // DIAG_CPU_LOAD_HPP
