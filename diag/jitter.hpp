// SPDX-License-Identifier: MIT OR Apache-2.0
// Jitter tracker for periodic RT threads (base / motion(servo) / EC master).
//
// Each periodic thread owns a JitterTracker and calls `sample(now_ns)` at the
// top of every period. The tracker records the interval since the previous
// sample and maintains running max_interval, max_jitter, and the most recent
// interval — all in nanoseconds.
//
// Intended use (RT plane: base + three 200 µs periodic threads):
//   namespace rt {
//     JitterTracker base_jitter  {25000};   //  25 µs base thread
//     JitterTracker motion_jitter{200000};  // 200 µs servo / motion kernel
//     JitterTracker ecat_a_jitter{200000};  // 200 µs EtherCAT master A
//     JitterTracker ecat_b_jitter{200000};  // 200 µs EtherCAT master B
//   }
//   // in the thread loop:
//   rt::base_jitter.sample(now_ns());
//   ... do work ...
//
// Design notes:
//   * Lock-free. Writers are assumed single-producer (one thread per tracker);
//     readers (the CLI dump command on another core) use relaxed atomics and
//     see monotonically non-decreasing max_* values.
//   * ns, not µs: at 25 µs period, a 1 µs jitter is 4% — we need fine
//     resolution. ARM generic timer at 62.5 MHz resolves 16 ns.
//   * 0 is reserved for "no sample yet" for last_start_ns_.
//
// Freestanding C++20.

#ifndef DIAG_JITTER_HPP
#define DIAG_JITTER_HPP

#include "hal.hpp"
#include <atomic>
#include <cstdint>

namespace diag {

class JitterTracker {
public:
    // `period_ns` is the nominal cyclic period the thread is aiming for;
    // jitter is reported relative to it.
    constexpr explicit JitterTracker(uint64_t period_ns) noexcept
        : period_ns_(period_ns) {}

    // Call at the start of every period with monotonic time in nanoseconds.
    void sample(uint64_t now_ns) noexcept;

    // Clear running maxima, keep period_ns_.
    void reset() noexcept;

    // --- Read-only accessors ---
    uint64_t period_ns()        const noexcept { return period_ns_; }
    uint64_t count()            const noexcept { return count_.load(std::memory_order_relaxed); }
    uint64_t last_interval_ns() const noexcept { return last_interval_ns_.load(std::memory_order_relaxed); }
    uint64_t max_interval_ns()  const noexcept { return max_interval_ns_.load(std::memory_order_relaxed); }
    uint64_t max_jitter_ns()    const noexcept { return max_jitter_ns_.load(std::memory_order_relaxed); }
    uint64_t min_interval_ns()  const noexcept { return min_interval_ns_.load(std::memory_order_relaxed); }
    uint64_t overruns()         const noexcept { return overruns_.load(std::memory_order_relaxed); }

    // Emit one human-readable line to `uart`. `label` is prepended, e.g. "base".
    void dump(kernel::hal::UARTDriverOps* uart, const char* label) const noexcept;

private:
    const uint64_t period_ns_;
    std::atomic<uint64_t> last_start_ns_{0};
    std::atomic<uint64_t> count_{0};
    std::atomic<uint64_t> last_interval_ns_{0};
    std::atomic<uint64_t> max_interval_ns_{0};
    std::atomic<uint64_t> min_interval_ns_{UINT64_MAX};
    std::atomic<uint64_t> max_jitter_ns_{0};
    // Times the observed interval exceeded 2 * period_ns (a missed deadline in
    // the "roughly one cycle late" sense — useful as a hard-fault signal).
    std::atomic<uint64_t> overruns_{0};
};

// A well-known set of trackers shared by the RT threads. Declared in a namespace
// so they're findable from the CLI and telemetry paths without passing pointers
// around.
namespace rt {
    extern JitterTracker base;    //  25 µs — step / fast I/O
    extern JitterTracker motion;  // 200 µs — servo / motion kernel
    extern JitterTracker ecat_a;  // 200 µs — EtherCAT master A
    extern JitterTracker ecat_b;  // 200 µs — EtherCAT master B
}

} // namespace diag

#endif // DIAG_JITTER_HPP
