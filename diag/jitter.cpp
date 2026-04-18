// SPDX-License-Identifier: MIT OR Apache-2.0
#include "jitter.hpp"
#include "util.hpp"

namespace diag {

namespace rt {
// Default periods (ns). The RT plane is uniformly 200 µs with a single 25 µs
// base thread for step generation / fast I/O. Drives whose spec minimum cycle
// exceeds 200 µs (e.g. ClearPath-EC at 250 µs) must be driven every N-th
// frame; that's a per-device config issue, not a global period.
JitterTracker base  {   25'000};   //  25 µs
JitterTracker motion{  200'000};   // 200 µs (servo)
JitterTracker ecat_a{  200'000};   // 200 µs
JitterTracker ecat_b{  200'000};   // 200 µs
} // namespace rt

void JitterTracker::sample(uint64_t now_ns) noexcept {
    // exchange current → new, read the previous start in one shot.
    uint64_t prev = last_start_ns_.exchange(now_ns, std::memory_order_acq_rel);
    if (prev == 0) {
        // First sample — only the start time is recorded; no interval yet.
        return;
    }
    if (now_ns < prev) {
        // Clock skew / wraparound — ignore this sample.
        return;
    }
    const uint64_t interval = now_ns - prev;
    last_interval_ns_.store(interval, std::memory_order_relaxed);
    count_.fetch_add(1, std::memory_order_relaxed);

    // Update max_interval (single-producer per tracker, but use CAS for robustness
    // in case of future multi-producer use).
    uint64_t mi = max_interval_ns_.load(std::memory_order_relaxed);
    while (interval > mi &&
           !max_interval_ns_.compare_exchange_weak(mi, interval,
                                                   std::memory_order_relaxed)) {
        // retry with refreshed mi
    }

    // min_interval
    uint64_t mn = min_interval_ns_.load(std::memory_order_relaxed);
    while (interval < mn &&
           !min_interval_ns_.compare_exchange_weak(mn, interval,
                                                   std::memory_order_relaxed)) {
    }

    // Jitter = |interval - period_ns_|.
    uint64_t jitter;
    if (interval >= period_ns_) jitter = interval - period_ns_;
    else                         jitter = period_ns_ - interval;
    uint64_t mj = max_jitter_ns_.load(std::memory_order_relaxed);
    while (jitter > mj &&
           !max_jitter_ns_.compare_exchange_weak(mj, jitter,
                                                 std::memory_order_relaxed)) {
    }

    // Hard overrun: interval ≥ 2 × period → we slipped a whole cycle.
    if (interval >= period_ns_ * 2) {
        overruns_.fetch_add(1, std::memory_order_relaxed);
    }
}

void JitterTracker::reset() noexcept {
    last_start_ns_.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);
    last_interval_ns_.store(0, std::memory_order_relaxed);
    max_interval_ns_.store(0, std::memory_order_relaxed);
    min_interval_ns_.store(UINT64_MAX, std::memory_order_relaxed);
    max_jitter_ns_.store(0, std::memory_order_relaxed);
    overruns_.store(0, std::memory_order_relaxed);
}

void JitterTracker::dump(kernel::hal::UARTDriverOps* uart,
                         const char* label) const noexcept {
    if (!uart) return;
    uint64_t cnt   = count_.load(std::memory_order_relaxed);
    uint64_t last  = last_interval_ns_.load(std::memory_order_relaxed);
    uint64_t maxi  = max_interval_ns_.load(std::memory_order_relaxed);
    uint64_t mini  = min_interval_ns_.load(std::memory_order_relaxed);
    uint64_t maxj  = max_jitter_ns_.load(std::memory_order_relaxed);
    uint64_t over  = overruns_.load(std::memory_order_relaxed);
    if (mini == UINT64_MAX) mini = 0; // no samples yet

    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  %s  period=%lluns  n=%llu  last=%lluns  min=%lluns  max=%lluns  "
        "maxJitter=%lluns  overruns=%llu\n",
        label ? label : "?",
        (unsigned long long)period_ns_,
        (unsigned long long)cnt,
        (unsigned long long)last,
        (unsigned long long)mini,
        (unsigned long long)maxi,
        (unsigned long long)maxj,
        (unsigned long long)over);
    uart->puts(buf);
}

} // namespace diag
