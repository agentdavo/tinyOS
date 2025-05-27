// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file trace.cpp
 * @brief Task tracing subsystem implementation for miniOS v1.7.
 * @details
 * Implements a thread-safe tracing system for debugging SMP thread execution, recording events
 * (thread creation, scheduling, yield, exit, custom) in a fixed-size buffer. Supports enabling/
 * disabling tracing, dumping to UART, and clearing the buffer via CLI commands. Updated in v1.7
 * with improved error handling, clearer diagnostics, and modern C++20 practices, retaining all
 * v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer access
 * - std::atomic for thread-safe operations
 * - std::string_view for string handling
 *
 * @version 1.7
 * @see trace.hpp, miniOS.hpp, util.hpp, cli.hpp
 */

#include "trace.hpp"
#include "util.hpp"
#include <cstring>

namespace trace {

TraceManager g_trace_manager;

bool TraceManager::record_event(const kernel::TCB* tcb, EventType type, std::string_view name, uint64_t value) noexcept {
    if (!enabled_.load(std::memory_order_relaxed) || !kernel::g_platform) return false;
    size_t idx = buffer_idx_.fetch_add(1, std::memory_order_relaxed);
    if (idx >= MAX_TRACE_EVENTS) {
        buffer_idx_.fetch_sub(1, std::memory_order_relaxed);
        return false; // Buffer full
    }
    buffer_[idx] = TraceEvent{
        .timestamp_us = kernel::g_platform->get_timer_ops()->get_system_time_us(),
        .type = type,
        .tcb = tcb,
        .name = name,
        .value = value
    };
    return true;
}

void TraceManager::dump_trace(kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return;
    size_t size = std::min(buffer_idx_.load(std::memory_order_relaxed), MAX_TRACE_EVENTS);
    if (size == 0) {
        uart_ops->puts("Trace buffer empty\n");
        return;
    }

    uart_ops->puts("\n--- Trace Buffer ---\n");
    for (size_t i = 0; i < size; ++i) {
        const auto& event = buffer_[i];
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%llu", event.timestamp_us);
        uart_ops->puts("[Trace] ");
        uart_ops->puts(buf);
        uart_ops->puts(" us: ");
        switch (event.type) {
            case EventType::THREAD_CREATE: uart_ops->puts("THREAD_CREATE"); break;
            case EventType::THREAD_SCHEDULE: uart_ops->puts("THREAD_SCHEDULE"); break;
            case EventType::THREAD_YIELD: uart_ops->puts("THREAD_YIELD"); break;
            case EventType::THREAD_EXIT: uart_ops->puts("THREAD_EXIT"); break;
            case EventType::CUSTOM: uart_ops->puts("CUSTOM"); break;
        }
        if (event.tcb && event.tcb->name[0]) {
            uart_ops->puts(", Thread=");
            uart_ops->puts(event.tcb->name);
        }
        if (!event.name.empty()) {
            uart_ops->puts(", Name=");
            uart_ops->puts(event.name.data());
        }
        if (event.value != 0) {
            uart_ops->puts(", Value=");
            std::snprintf(buf, sizeof(buf), "0x%llx", event.value);
            uart_ops->puts(buf);
        }
        uart_ops->puts("\n");
    }
    uart_ops->puts("--- End Trace ---\n");
}

void TraceManager::clear_trace() noexcept {
    buffer_idx_.store(0, std::memory_order_relaxed);
    for (auto& event : buffer_) {
        event = TraceEvent{};
    }
}

} // namespace trace