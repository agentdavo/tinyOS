// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file trace.hpp
 * @brief Task tracing subsystem header for miniOS v1.7.
 * @details
 * Defines a lightweight, thread-safe tracing system for debugging thread execution in SMP
 * environments. Records events (e.g., thread creation, scheduling, yield, exit, custom) in a
 * fixed-size buffer, with support for enabling/disabling tracing, dumping to UART, and clearing
 * via CLI. Updated in v1.7 for modularity and compatibility with low-latency audio RTOS.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 * - std::string_view for string operations
 *
 * @version 1.7
 * @see trace.cpp, core.hpp, hal.hpp
 */

#ifndef TRACE_HPP
#define TRACE_HPP

#include "core.hpp"
#include "hal.hpp"
#include <span>
#include <atomic>
#include <array>
#include <string_view>
#include <cstdint>

namespace trace {

constexpr size_t MAX_TRACE_EVENTS = 64;

enum class EventType {
    THREAD_CREATE,
    THREAD_SCHEDULE,
    THREAD_YIELD,
    THREAD_EXIT,
    CUSTOM
};

struct TraceEvent {
    uint64_t timestamp_us;
    EventType type;
    const kernel::core::TCB* tcb;
    std::string_view name;
    uint64_t value;
};

class TraceManager {
public:
    TraceManager() : enabled_(false) {}

    void init() noexcept { clear_trace(); }

    void set_enabled(bool enable) noexcept { enabled_.store(enable, std::memory_order_relaxed); }

    bool is_enabled() const noexcept { return enabled_.load(std::memory_order_relaxed); }

    bool record_event(const kernel::core::TCB* tcb, EventType type, std::string_view name, uint64_t value = 0) noexcept;

    void dump_trace(kernel::hal::UARTDriverOps* uart_ops) const;

    void clear_trace() noexcept;

private:
    std::atomic<bool> enabled_;
    std::array<TraceEvent, MAX_TRACE_EVENTS> buffer_;
    std::atomic<size_t> buffer_idx_{0};
};

extern TraceManager g_trace_manager;

} // namespace trace

#endif // TRACE_HPP