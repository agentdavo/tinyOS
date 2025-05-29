// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file trace.cpp
 * @brief Task tracing subsystem implementation for miniOS v1.7.
 */

#include "trace.hpp"
#include "core.hpp"
#include "hal.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include <algorithm>
#include <atomic>

namespace trace {

TraceManager g_trace_manager;

bool TraceManager::record_event(const kernel::core::TCB* tcb, EventType type,
                                std::string_view name_sv, uint64_t value) noexcept {
    if (!enabled_.load(std::memory_order_relaxed) || !kernel::g_platform || !kernel::g_platform->get_timer_ops()) { // Added timer_ops check
        return false;
    }

    // Using the lock from TraceManager if it exists, or a global one if necessary
    // Assuming TraceManager has its own lock_ for buffer access (as per your trace.hpp)
    // kernel::core::ScopedLock guard(lock_); // If lock_ is a member of TraceManager

    // For this example, let's assume no internal lock in TraceManager and rely on atomics for buffer_idx_
    // or if more complex, it should have its own kernel::core::Spinlock member.
    // For now, let's use the simpler approach from your previous code.

    size_t idx = buffer_idx_.fetch_add(1, std::memory_order_relaxed);
    // Simple wrap-around for ring buffer behavior
    idx %= MAX_TRACE_EVENTS; 

    // buffer_[idx] = TraceEvent { ... } // This direct assignment can be problematic if TraceEvent has non-trivial members
    // Corrected approach:
    TraceEvent& current_event = buffer_[idx];
    current_event.timestamp_us = kernel::g_platform->get_timer_ops()->get_system_time_us();
    current_event.type = type;
    current_event.tcb = tcb;

    // Copy name safely if name is std::array<char, KERNEL_MAX_NAME_LENGTH> in TraceEvent
    // If TraceEvent.name is std::string_view, this is fine:
    current_event.name = name_sv; 
    // If TraceEvent.name is char array like in my proposed trace.hpp:
    // kernel::util::safe_strcpy(current_event.name_copy.data(), name_sv.data(), current_event.name_copy.size());
    
    current_event.value = value;
    return true;
}

void TraceManager::dump_trace(kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return;

    size_t current_buffer_idx = buffer_idx_.load(std::memory_order_relaxed);
    size_t count = kernel::util::min(current_buffer_idx, MAX_TRACE_EVENTS); // Number of valid entries if not wrapping
                                                                  // Or if wrapping, this is just total events written mod MAX

    if (count == 0 && current_buffer_idx < MAX_TRACE_EVENTS) { // If it hasn't wrapped and idx is 0
        uart_ops->puts("Trace buffer empty\n");
        return;
    }

    uart_ops->puts("\n--- Trace Buffer (TraceManager) ---\n");
    
    // If it's a ring buffer, we need to print from head to tail.
    // This simple iteration prints the first 'count' entries, or all if it wrapped.
    // For a true ring buffer, this print logic needs to be smarter.
    // Assuming for now it's a simple array that fills up and then wraps.
    size_t start_idx = 0;
    size_t num_to_print = count;

    if (current_buffer_idx >= MAX_TRACE_EVENTS) { // It has wrapped at least once
        start_idx = current_buffer_idx % MAX_TRACE_EVENTS; // Oldest entry is at current_idx
        num_to_print = MAX_TRACE_EVENTS;
    }


    for (size_t i = 0; i < num_to_print; ++i) {
        size_t actual_idx = (start_idx + i) % MAX_TRACE_EVENTS;
        const auto& event = buffer_[actual_idx];
        
        // Check if the event looks initialized (e.g., timestamp is not 0, if that's a valid check)
        // Or if TraceEvent has a 'valid' flag. For now, print if name/tcb suggests it's used.
        if (event.tcb == nullptr && event.name.empty() && event.timestamp_us == 0 && i >= current_buffer_idx && current_buffer_idx < MAX_TRACE_EVENTS) {
            // Likely an uninitialized entry if buffer hasn't filled yet.
            continue;
        }

        char line_buf[256]; // Main buffer for the line
        char temp_buf[64];  // Temporary buffer for numbers

        kernel::util::k_snprintf(line_buf, sizeof(line_buf), "[TraceEvt %zu] %llu us: ",
                                 actual_idx,
                                 static_cast<unsigned long long>(event.timestamp_us));

        switch (event.type) {
            case EventType::THREAD_CREATE: kernel::util::kstrcat(line_buf, "CREATE ", sizeof(line_buf)); break;
            case EventType::THREAD_SCHEDULE: kernel::util::kstrcat(line_buf, "SCHED  ", sizeof(line_buf)); break;
            case EventType::THREAD_YIELD: kernel::util::kstrcat(line_buf, "YIELD  ", sizeof(line_buf)); break;
            case EventType::THREAD_EXIT: kernel::util::kstrcat(line_buf, "EXIT   ", sizeof(line_buf)); break;
            case EventType::CUSTOM: kernel::util::kstrcat(line_buf, "CUSTOM ", sizeof(line_buf)); break;
            default: kernel::util::kstrcat(line_buf, "UNKNOWN", sizeof(line_buf)); break;
        }

        if (event.tcb && event.tcb->name[0]) {
            kernel::util::kstrcat(line_buf, ", Thread=", sizeof(line_buf));
            kernel::util::kstrcat(line_buf, event.tcb->name, sizeof(line_buf));
        } else if (!event.name.empty()) {
            // string_view might not be null terminated for kstrcat
            // Create a temporary null-terminated string for name if needed, or ensure kstrcat handles non-null-terminated view
            char name_temp_buf[kernel::core::MAX_NAME_LENGTH + 1];
            size_t name_len = kernel::util::min(event.name.length(), kernel::core::MAX_NAME_LENGTH);
            kernel::util::kmemcpy(name_temp_buf, event.name.data(), name_len);
            name_temp_buf[name_len] = '\0';
            kernel::util::kstrcat(line_buf, ", Name=", sizeof(line_buf));
            kernel::util::kstrcat(line_buf, name_temp_buf, sizeof(line_buf));
        }

        if (event.value != 0 || event.type == EventType::CUSTOM) { 
            kernel::util::uint64_to_hex_str(event.value, temp_buf, sizeof(temp_buf));
            kernel::util::kstrcat(line_buf, ", Val=", sizeof(line_buf));
            kernel::util::kstrcat(line_buf, temp_buf, sizeof(line_buf));
        }
        kernel::util::kstrcat(line_buf, "\n", sizeof(line_buf));
        uart_ops->puts(line_buf);
    }

    uart_ops->puts("--- End Trace (TraceManager) ---\n");
}

void TraceManager::clear_trace() noexcept {
    // Assuming single-producer for buffer_idx modification or that it's locked externally if multi-producer clear
    buffer_idx_.store(0, std::memory_order_relaxed);
    // For safety, clear the buffer content if needed, though new writes will overwrite.
    // This depends on how 'empty' entries are detected in dump_trace.
    for (auto& event_entry : buffer_) {
        event_entry = TraceEvent{}; // Reset to default
    }
}

} // namespace trace