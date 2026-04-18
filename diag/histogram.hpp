// SPDX-License-Identifier: MIT OR Apache-2.0
// Lock-free latency histogram for production telemetry.
//
// Buckets (microseconds, upper-inclusive):
//   [0,10], (10,50], (50,100], (100,200], (200,500], (500,1k], (1k,2k],
//   (2k,5k], (5k,10k], (10k,20k], (20k,50k], (50k,100k], (100k,200k],
//   (200k,500k], (500k,1M], (1M,inf)
// → 16 buckets total. Each counter + sum/min/max is a std::atomic<uint64_t>
// updated with relaxed ordering (cross-core loss is acceptable for a
// diagnostic counter; we never gate control flow on these numbers).

#ifndef DIAG_HISTOGRAM_HPP
#define DIAG_HISTOGRAM_HPP

#include "hal.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace diag {

constexpr size_t HIST_BUCKETS = 16;

class LatencyHistogram {
public:
    LatencyHistogram() noexcept = default;

    // Record one observation, in microseconds.
    void record(uint64_t us) noexcept;

    // Dump a compact table to the given UART. `label` is the name printed in
    // the header (e.g. "ec0:cycle"). Safe to call concurrently with record().
    void dump(kernel::hal::UARTDriverOps* uart, const char* label) const;

    // Reset all counters to zero (for "ec_hist clear" or test harnesses).
    void reset() noexcept;

    static const uint64_t* bucket_edges_us() noexcept;

private:
    // Cache-line isolate the hot summary counters so min/max CAS loops don't
    // bounce the same line that the bucket counters touch. 64-byte alignment
    // matches cortex-a53's L1 line size.
    alignas(64) std::array<std::atomic<uint64_t>, HIST_BUCKETS> counts_{};
    alignas(64) std::atomic<uint64_t> count_{0};
    std::atomic<uint64_t> sum_us_{0};
    alignas(64) std::atomic<uint64_t> min_us_{UINT64_MAX};
    std::atomic<uint64_t> max_us_{0};
};

} // namespace diag

#endif
