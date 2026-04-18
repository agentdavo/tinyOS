// SPDX-License-Identifier: MIT OR Apache-2.0
#include "histogram.hpp"
#include "util.hpp"

#include <limits>

namespace diag {

namespace {

// 15 finite edges + 1 overflow bucket → 16 slots.
constexpr uint64_t kEdges[HIST_BUCKETS - 1] = {
    10ULL, 50ULL, 100ULL, 200ULL, 500ULL,
    1000ULL, 2000ULL, 5000ULL, 10000ULL, 20000ULL,
    50000ULL, 100000ULL, 200000ULL, 500000ULL, 1000000ULL
};

size_t bucket_for(uint64_t us) noexcept {
    for (size_t i = 0; i < HIST_BUCKETS - 1; ++i) {
        if (us <= kEdges[i]) return i;
    }
    return HIST_BUCKETS - 1;
}

// Atomic min: store v if v < cur.
inline void atomic_min(std::atomic<uint64_t>& a, uint64_t v) noexcept {
    uint64_t cur = a.load(std::memory_order_relaxed);
    while (v < cur && !a.compare_exchange_weak(cur, v,
                                               std::memory_order_relaxed,
                                               std::memory_order_relaxed)) {
        // cur is updated; loop retries.
    }
}

// Atomic max: store v if v > cur.
inline void atomic_max(std::atomic<uint64_t>& a, uint64_t v) noexcept {
    uint64_t cur = a.load(std::memory_order_relaxed);
    while (v > cur && !a.compare_exchange_weak(cur, v,
                                               std::memory_order_relaxed,
                                               std::memory_order_relaxed)) {
    }
}

} // namespace

const uint64_t* LatencyHistogram::bucket_edges_us() noexcept {
    return kEdges;
}

void LatencyHistogram::record(uint64_t us) noexcept {
    counts_[bucket_for(us)].fetch_add(1, std::memory_order_relaxed);
    count_.fetch_add(1, std::memory_order_relaxed);
    sum_us_.fetch_add(us, std::memory_order_relaxed);
    atomic_min(min_us_, us);
    atomic_max(max_us_, us);
}

void LatencyHistogram::reset() noexcept {
    for (auto& c : counts_) c.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);
    sum_us_.store(0, std::memory_order_relaxed);
    min_us_.store(UINT64_MAX, std::memory_order_relaxed);
    max_us_.store(0, std::memory_order_relaxed);
}

void LatencyHistogram::dump(kernel::hal::UARTDriverOps* uart, const char* label) const {
    if (!uart) return;
    const uint64_t count = count_.load(std::memory_order_relaxed);
    const uint64_t sum   = sum_us_.load(std::memory_order_relaxed);
    const uint64_t mn    = min_us_.load(std::memory_order_relaxed);
    const uint64_t mx    = max_us_.load(std::memory_order_relaxed);

    char buf[160];
    if (count == 0) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %s: no samples\n", label ? label : "hist");
        uart->puts(buf);
        return;
    }
    const uint64_t avg = sum / count;
    const uint64_t mn_disp = (mn == UINT64_MAX) ? 0 : mn;
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  %s: n=%llu min=%lluus avg=%lluus max=%lluus\n",
        label ? label : "hist",
        (unsigned long long)count,
        (unsigned long long)mn_disp,
        (unsigned long long)avg,
        (unsigned long long)mx);
    uart->puts(buf);

    // Column-style bucket dump.
    for (size_t i = 0; i < HIST_BUCKETS; ++i) {
        uint64_t c = counts_[i].load(std::memory_order_relaxed);
        if (c == 0) continue;
        if (i == HIST_BUCKETS - 1) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "    >%lluus: %llu\n",
                (unsigned long long)kEdges[HIST_BUCKETS - 2],
                (unsigned long long)c);
        } else if (i == 0) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "    <=%lluus: %llu\n",
                (unsigned long long)kEdges[0],
                (unsigned long long)c);
        } else {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "    (%llu..%llu]us: %llu\n",
                (unsigned long long)kEdges[i - 1],
                (unsigned long long)kEdges[i],
                (unsigned long long)c);
        }
        uart->puts(buf);
    }
}

} // namespace diag
