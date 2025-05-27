// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file trace.hpp
 * @brief Task tracing subsystem header for miniOS v1.7.
 * @details
 * Defines a lightweight, thread-safe tracing system for debugging thread execution in SMP
 * environments. Records events (e.g., thread creation, scheduling, yield, exit, custom) in a
 * fixed-size buffer, with support for enabling/disabling tracing, dumping to UART, and clearing
 * the buffer. Integrates with CLI for trace control. Updated in v1.7 with improved error handling,
 * clearer documentation, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 * - std::string_view for string operations
 *
 * @version 1.7
 * @see trace.cpp, miniOS.hpp, util.hpp, cli.hpp
 */

#ifndef TRACE_HPP
#define TRACE_HPP

#include "miniOS.hpp"
#include <span>
#include <atomic>
#include <array>
#include <string_view>

namespace trace {

constexpr size_t MAX_TRACE_EVENTS = 1024;

/**
 * @brief Trace event types.
 */
enum class EventType {
    THREAD_CREATE,    ///< Thread creation
    THREAD_SCHEDULE,  ///< Thread scheduled
    THREAD_YIELD,     ///< Thread yielded
    THREAD_EXIT,      ///< Thread exited
    CUSTOM            ///< User-defined event
};

/**
 * @brief Trace event structure.
 */
struct TraceEvent {
    uint64_t timestamp_us; ///< Event timestamp (microseconds)
    EventType type;       ///< Event type
    const kernel::TCB* tcb; ///< Associated thread control block
    std::string_view name; ///< Event name (e.g., thread name, custom label)
    uint64_t value;       ///< Optional event value
};

/**
 * @brief Tracing subsystem class.
 */
class TraceManager {
public:
    TraceManager() : enabled_(false) {}

    /**
     * @brief Initializes the tracing subsystem.
     */
    void init() noexcept { clear_trace(); }

    /**
     * @brief Enables or disables tracing.
     * @param enable True to enable, false to disable
     */
    void set_enabled(bool enable) noexcept { enabled_.store(enable, std::memory_order_relaxed); }

    /**
     * @brief Checks if tracing is enabled.
     * @return True if enabled, false otherwise
     */
    bool is_enabled() const noexcept { return enabled_.load(std::memory_order_relaxed); }

    /**
     * @brief Records a trace event.
     * @param tcb Thread control block (nullptr for non-thread events)
     * @param type Event type
     * @param name Event name
     * @param value Optional event value
     * @return True if recorded, false if buffer full or disabled
     */
    bool record_event(const kernel::TCB* tcb, EventType type, std::string_view name, uint64_t value = 0) noexcept;

    /**
     * @brief Dumps the trace buffer to UART.
     * @param uart_ops UART driver for output
     */
    void dump_trace(kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Clears the trace buffer.
     */
    void clear_trace() noexcept;

private:
    std::atomic<bool> enabled_; ///< Tracing enabled state
    std::array<TraceEvent, MAX_TRACE_EVENTS> buffer_; ///< Trace event buffer
    std::atomic<size_t> buffer_idx_{0}; ///< Current buffer index
};

extern TraceManager g_trace_manager; ///< Global trace manager instance

} // namespace trace

#endif // TRACE_HPP