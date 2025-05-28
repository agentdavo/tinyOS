// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file trace.cpp
 * @brief Task tracing subsystem implementation for miniOS v1.7.
 * @details
 * Implements thread-safe event tracing for debugging, with events stored in a fixed-size
 * buffer. Supports CLI integration for trace control. Updated in v1.7 for modularity and
 * low-latency RTOS compatibility.
 *
 * @version 1.7
 * @see trace.hpp, core.hpp, hal.hpp, miniOS.hpp
 */

#include "trace.hpp"
#include "core.hpp"
#include "hal.hpp"
#include "miniOS.hpp"
#include <cstdio>
#include <atomic>

namespace trace {

TraceManager g_trace_manager;

bool TraceManager::record_event(const kernel::core::TCB* tcb, EventType type,
                                std::string_view name, uint64_t value) noexcept {
    if (!enabled_.load(std::memory_order_relaxed) || !kernel::g_platform) return false;

    size_t idx = buffer_idx_.fetch_add(1, std::memory_order_relaxed);
    if (idx >= MAX_TRACE_EVENTS) {
        buffer_idx_.fetch_sub(1, std::memory_order_relaxed);
        return false;
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

    if (buffer_idx_.load(std::memory_order_relaxed) == 0) {
        uart_ops->puts("Trace buffer empty\n");
        return;
    }

    uart_ops->puts("\n--- Trace Buffer ---\n");

    for (size_t i = 0; i < std::min(buffer_idx_.load(std::memory_order_relaxed), MAX_TRACE_EVENTS); ++i) {
        const auto& event = buffer_[i];
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%lu", event.timestamp_us);
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
        } else if (!event.name.empty()) {
            uart_ops->puts(", Name=");
            uart_ops->puts(event.name.data());
        }

        if (event.value) {
            uart_ops->puts(", Value=");
            std::snprintf(buf, sizeof(buf), "0x%lx", event.value);
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