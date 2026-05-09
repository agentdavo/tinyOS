// SPDX-License-Identifier: MIT OR Apache-2.0

#include "klog.hpp"
#include "hal.hpp"
#include "util.hpp"

#include <atomic>

namespace kernel { namespace klog {

static_assert((RING_SIZE & (RING_SIZE - 1)) == 0,
              "RING_SIZE must be a power of 2");

static char g_ring[RING_SIZE];
// Total bytes ever written. Wraps via mask. Single-writer (callers
// hold the platform UART lock when calling record), so a plain atomic
// store with release / load with acquire is sufficient for the dump
// reader to see a consistent snapshot.
static std::atomic<size_t> g_total{0};

void record(const char* data, size_t n) noexcept {
    if (!data || n == 0) return;
    const size_t total = g_total.load(std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i) {
        g_ring[(total + i) & (RING_SIZE - 1)] = data[i];
    }
    g_total.store(total + n, std::memory_order_release);
}

void dump(kernel::hal::UARTDriverOps* uart) noexcept {
    if (!uart) return;
    const size_t total = g_total.load(std::memory_order_acquire);
    if (total == 0) {
        uart->puts("[klog: empty]\n");
        return;
    }
    const bool wrapped = (total >= RING_SIZE);
    const size_t avail = wrapped ? RING_SIZE : total;
    const size_t start = wrapped ? (total & (RING_SIZE - 1)) : 0;

    // Emit in two segments if the ring has wrapped. Use small line
    // buffers to amortise the per-byte putc dispatch and stay below
    // the line-atomicity invariant of UARTDriver::puts (which holds
    // its lock for the duration of one puts() call).
    char buf[129];
    size_t buf_len = 0;
    auto flush = [&]() {
        if (buf_len == 0) return;
        buf[buf_len] = '\0';
        uart->puts(buf);
        buf_len = 0;
    };
    auto emit = [&](size_t from, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            buf[buf_len++] = g_ring[from + i];
            if (buf_len == sizeof(buf) - 1) flush();
        }
    };
    if (start + avail <= RING_SIZE) {
        emit(start, avail);
    } else {
        const size_t first = RING_SIZE - start;
        emit(start, first);
        emit(0, avail - first);
    }
    flush();

    char footer[64];
    const int n = kernel::util::k_snprintf(
        footer, sizeof(footer),
        "\n[klog: %lu bytes resident, %lu total]\n",
        static_cast<unsigned long>(avail),
        static_cast<unsigned long>(total));
    if (n > 0) uart->puts(footer);
}

size_t tail(char* out, size_t cap) noexcept {
    if (!out || cap == 0) return 0;
    const size_t total = g_total.load(std::memory_order_acquire);
    if (total == 0) return 0;
    const bool wrapped = (total >= RING_SIZE);
    const size_t avail = wrapped ? RING_SIZE : total;
    const size_t take  = avail < cap ? avail : cap;
    // Walk backwards from the write head by `take` bytes.
    const size_t end_pos = total & (RING_SIZE - 1);
    // start_pos is "end_pos - take" mod ring; copy in two segments if
    // the slice wraps.
    const size_t start_pos = (end_pos + RING_SIZE - take) & (RING_SIZE - 1);
    if (start_pos + take <= RING_SIZE) {
        for (size_t i = 0; i < take; ++i) out[i] = g_ring[start_pos + i];
    } else {
        const size_t first = RING_SIZE - start_pos;
        for (size_t i = 0; i < first; ++i) out[i] = g_ring[start_pos + i];
        for (size_t i = 0; i < take - first; ++i) out[first + i] = g_ring[i];
    }
    return take;
}

}} // namespace kernel::klog
