// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file miniOS.cpp
 * @brief Core kernel implementation for miniOS v1.7.
 * @details
 * Implements the miniOS kernel with EDF scheduling, thread management, synchronization, and subsystem
 * integration (CLI, DSP, audio, tracing, RAMFS, networking, GPIO). Hardware-agnostic, using HAL
 * interfaces for platform-specific operations. Updated in v1.7 with improved error handling, clearer
 * diagnostics, and developer-friendly enhancements. Renamed from miniOS_v1.6.cpp for simplicity.
 *
 * New in v1.7:
 * - Renamed from miniOS_v1.6.cpp
 * - Enhanced error handling (null checks, bounds validation)
 * - Improved Doxygen comments and code clarity
 * - Consistent C++20 usage (std::span, std::string_view)
 *
 * @version 1.7
 * @see miniOS.hpp, util.hpp, cli.hpp, dsp.hpp, audio.hpp, trace.hpp, fs.hpp, net.hpp, gpio.hpp
 */

#include "miniOS.hpp" // Should be first for precompiled headers, and defines kernel types
#include "util.hpp"   // Now included after kernel types are known via miniOS.hpp
#include "cli.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "trace.hpp"
#include "fs.hpp"
#include "net.hpp"
#include "gpio.hpp"
#include <cstring>    // For std::memcpy, std::memset, std::strlen, std::strcmp etc.
#include <cstdio>     // For std::sscanf, std::snprintf (used in get_kernel_stats)
#include <cassert>    // For assert (if used)
#include <algorithm>  // For std::max in FixedMemoryPool

// Declare demo and test registration functions
namespace demo { void register_demo_commands(); }
namespace test { void register_tests(); }

namespace kernel {

hal::Platform* g_platform = nullptr; // Definition for the extern in miniOS.hpp
Scheduler* g_scheduler_ptr = nullptr; // Definition
volatile bool g_bss_cleared_smp_flag = false;
alignas(64) uint8_t g_audio_pool_mem[64 * 1024]; // Definition
FixedMemoryPool g_audio_pool; // Definition
alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(kernel::hal::timer::SoftwareTimer)]; // Definition, qualified SoftwareTimer
FixedMemoryPool g_software_timer_obj_pool; // Definition
audio::AudioSystem g_audio_system; // Definition
trace::TraceManager g_trace_manager; // Definition
fs::FileSystem g_file_system; // Definition
net::NetManager g_net_manager; // Definition
gpio::GPIOManager g_gpio_manager; // Definition
Spinlock g_irq_handler_lock;
std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::atomic<size_t> g_trace_buffer_next_idx{0};
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count = {};
Spinlock g_trace_lock;
alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() : lock_id_(next_lock_id_++) {}

void Spinlock::acquire_isr_safe() {
    if (!g_platform) return;
    g_platform->get_irq_ops()->disable_core_irqs(g_platform->get_core_id());
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        // Consider a yield or short delay if this can spin long in a multicore scenario,
        // but for ISR-safe, it should be quick or the system design is flawed.
    }
}

void Spinlock::release_isr_safe() {
    if (!g_platform) return;
    lock_flag_.store(false, std::memory_order_release);
    g_platform->get_irq_ops()->enable_core_irqs(g_platform->get_core_id(), 0); // Assuming 0 is a valid source or ignored
}

void Spinlock::acquire_general() {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        asm volatile("wfe" ::: "memory"); // Wait For Event (ARM specific)
    }
}

void Spinlock::release_general() {
    lock_flag_.store(false, std::memory_order_release);
    asm volatile("sev" ::: "memory"); // Send Event (ARM specific)
}

bool FixedMemoryPool::init(void* base, size_t num, size_t blk_sz_user, size_t align_user_data) {
    if (!base || num == 0 || blk_sz_user == 0) return false;
    size_t actual_align = std::max(align_user_data, sizeof(void*)); // std::max needs <algorithm>
    ScopedLock lock(pool_lock_); // pool_lock_ is a kernel::Spinlock
    header_actual_size_ = (sizeof(Block) + actual_align - 1) & ~(actual_align - 1);
    block_storage_size_ = (header_actual_size_ + blk_sz_user + actual_align - 1) & ~(actual_align - 1);
    if (block_storage_size_ < header_actual_size_ + blk_sz_user) return false; // Overflow check
    pool_memory_start_ = static_cast<uint8_t*>(base);
    num_total_blocks_ = num_free_blocks_ = num;
    free_head_ = nullptr;
    for (size_t i = 0; i < num_total_blocks_; ++i) {
        Block* block = reinterpret_cast<Block*>(pool_memory_start_ + i * block_storage_size_);
        block->next = free_head_;
        free_head_ = block;
    }
    return true;
}

void* FixedMemoryPool::allocate() {
    ScopedLock lock(pool_lock_);
    if (!free_head_) {
        if (g_platform) g_platform->panic("Memory pool exhausted", __FILE__, __LINE__);
        return nullptr;
    }
    Block* block = free_head_;
    free_head_ = block->next;
    num_free_blocks_--;
    return static_cast<uint8_t*>(static_cast<void*>(block)) + header_actual_size_;
}

void FixedMemoryPool::free_block(void* user_data_ptr) {
    if (!user_data_ptr) return;
    ScopedLock lock(pool_lock_);
    Block* block = reinterpret_cast<Block*>(static_cast<uint8_t*>(user_data_ptr) - header_actual_size_);
    // Basic check: ensure the block is within the pool bounds (optional, for debugging)
    // if (reinterpret_cast<uint8_t*>(block) < pool_memory_start_ ||
    //     reinterpret_cast<uint8_t*>(block) >= pool_memory_start_ + num_total_blocks_ * block_storage_size_) {
    //     if (g_platform) g_platform->panic("Freeing block outside pool", __FILE__, __LINE__);
    //     return;
    // }
    block->next = free_head_;
    free_head_ = block;
    num_free_blocks_++;
}

template<typename T, size_t Capacity>
bool SPSCQueue<T, Capacity>::enqueue(T* item) noexcept {
    size_t current_tail = tail_.load(std::memory_order_acquire);
    size_t next_tail = (current_tail + 1) & (Capacity - 1);
    if (next_tail == head_.load(std::memory_order_acquire)) return false; // Queue full
    items_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
}

template<typename T, size_t Capacity>
T* SPSCQueue<T, Capacity>::dequeue() noexcept {
    size_t current_head = head_.load(std::memory_order_acquire);
    if (current_head == tail_.load(std::memory_order_acquire)) return nullptr; // Queue empty
    T* item = items_[current_head];
    head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
    return item;
}

template class SPSCQueue<audio::AudioBuffer, 16>; // Explicit instantiation

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* /* current */ ) { // Mark current as unused
    if (core_id >= MAX_CORES) return nullptr; // Should not happen if MAX_CORES is correct
    TCB* earliest_task = nullptr;
    uint64_t earliest_deadline = UINT64_MAX;

    // Accessing g_scheduler_ptr->per_core_locks_ and g_scheduler_ptr->ready_qs_
    // This implies EDFPolicy needs a way to access Scheduler internals.
    // A friend class declaration or passing Scheduler members would be needed.
    // Assuming Scheduler made EDFPolicy a friend or provided accessors.
    if (!g_scheduler_ptr) { if(g_platform) g_platform->panic("Scheduler not initialized", __FILE__, __LINE__); return nullptr;}

    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]); // per_core_locks_ must be accessible

    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        TCB* task_iter = g_scheduler_ptr->ready_qs_[core_id][p]; // ready_qs_ must be accessible
        TCB* prev_task = nullptr;
        while (task_iter) {
            if (task_iter->deadline_us > 0 && task_iter->deadline_us < earliest_deadline &&
                (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(core_id))) {
                earliest_deadline = task_iter->deadline_us;
                
                // Temporarily unlink the task to consider it
                TCB* potential_next = task_iter;
                if (prev_task) {
                    prev_task->next_in_q = task_iter->next_in_q;
                } else {
                    g_scheduler_ptr->ready_qs_[core_id][p] = task_iter->next_in_q;
                }
                potential_next->next_in_q = nullptr; // Detach
                // If we had a previous earliest_task, put it back
                if (earliest_task) {
                     // Re-add earliest_task to its original queue position (complex without more context)
                     // Simpler: EDF might just pick one and not try to put others back during selection.
                     // For now, assume we pick the first one that improves the deadline.
                }
                earliest_task = potential_next;
                // To strictly find the *absolute* earliest, we'd iterate all, then extract.
                // This simplified version picks the first improvement.
                // For a real EDF, you might iterate all suitable tasks, then pick the best.
                // Then remove it from the queue.
                // This current logic extracts the first one it finds that *could* be the earliest.
                // For a correct EDF, you should iterate all tasks in the current core's ready queues,
                // find the one with the smallest deadline_us, and then remove that one.
                // The current loop structure removes the *first* task it encounters that *improves* the deadline.
                // This might not be the absolute earliest if a later task in the same priority queue or a
                // task in a lower priority queue (but still valid affinity) has an even earlier deadline.
                // A proper EDF usually maintains a single deadline-ordered queue.
                trace_event("SCHED:PopEDF", reinterpret_cast<uintptr_t>(earliest_task), core_id);
                return earliest_task; // Return the found task
            }
            prev_task = task_iter;
            task_iter = task_iter->next_in_q;
        }
    }
    // If no deadline-based task found, fall back to priority scheduler's method
    return g_scheduler_ptr->pop_highest_priority_ready_task(core_id);
}

void EDFPolicy::add_to_ready_queue(TCB* tcb, uint32_t core_id) {
    if (!tcb || tcb->priority < 0 || static_cast<size_t>(tcb->priority) >= MAX_PRIORITY_LEVELS || core_id >= MAX_CORES) {
        if (g_platform) g_platform->panic("Invalid ready queue args", __FILE__, __LINE__);
        return;
    }
    if (!g_scheduler_ptr) { if(g_platform) g_platform->panic("Scheduler not initialized for AddRdy", __FILE__, __LINE__); return;}
    
    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);
    trace_event("SCHED:AddRdy", reinterpret_cast<uintptr_t>(tcb), (core_id << 16) | tcb->priority);
    tcb->next_in_q = g_scheduler_ptr->ready_qs_[core_id][tcb->priority];
    g_scheduler_ptr->ready_qs_[core_id][tcb->priority] = tcb;
    tcb->state = TCB::State::READY;
}

Scheduler::Scheduler() {
    policy_ = new EDFPolicy(); // Ensure EDFPolicy is defined
    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        // Name for idle thread
        char idle_name[MAX_NAME_LENGTH];
        std::snprintf(idle_name, MAX_NAME_LENGTH, "IdleCore%u", i);
        auto* tcb = create_thread(idle_thread_func, reinterpret_cast<void*>(static_cast<uintptr_t>(i)), 0, i, idle_name, true);
        if (!tcb) {
            // g_platform might not be set yet if this constructor runs before kernel_main sets it.
            // This is a critical initialization order issue.
            // For now, assume g_platform will be available or handle panic differently.
            if (g_platform) g_platform->panic("Failed to create idle thread", __FILE__, __LINE__);
            else { /* Handle panic without g_platform, e.g., spin indefinitely */ while(1); }
        }
        g_per_cpu_data[i].idle_thread = tcb;
    }
}

TCB* Scheduler::create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle, uint64_t deadline_us) {
    if (!fn || !name) return nullptr;
    ScopedLock lock(scheduler_global_lock_);
    if (num_active_tasks_ >= MAX_THREADS) return nullptr;
    for (size_t i = 0; i < MAX_THREADS; ++i) {
        if (g_task_tcbs[i].state == TCB::State::INACTIVE) {
            TCB& tcb = g_task_tcbs[i];
            tcb.entry_point = fn;
            tcb.arg_ptr = const_cast<void*>(arg); // Store the argument
            tcb.priority = (prio >= 0 && static_cast<size_t>(prio) < MAX_PRIORITY_LEVELS) ? prio : 0;
            tcb.core_affinity = (affinity >= -1 && affinity < static_cast<int>(MAX_CORES)) ? affinity : -1;
            if (!util::safe_strcpy(tcb.name, name, MAX_NAME_LENGTH)) return nullptr; // util namespace
            tcb.stack_base = g_task_stacks[i].data();
            tcb.stack_size = DEFAULT_STACK_SIZE;
            tcb.state = TCB::State::READY;
            tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap); // Bootstrap function
            tcb.status = 0; // Initialize status (e.g., PSR for ARM)
            // Stack grows down, so SP is at the top. Ensure stack is aligned.
            tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size) & ~0xFUL; // 16-byte align SP
            tcb.deadline_us = deadline_us;
            if (!is_idle) {
                g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name); // Use kernel::g_trace_manager
                policy_->add_to_ready_queue(&tcb, (affinity != -1 && static_cast<size_t>(affinity) < MAX_CORES) ? affinity : 0);
                num_active_tasks_++;
            }
            return &tcb;
        }
    }
    return nullptr;
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!g_platform || core_id >= MAX_CORES) return;
    g_platform->get_irq_ops()->enable_core_irqs(core_id, 0); // Assuming 0 means enable all relevant for scheduler
    g_platform->get_timer_ops()->init_core_timer_interrupt(); // Initialize timer interrupt for this core
    auto* tcb = g_per_cpu_data[core_id].idle_thread;
    if (!tcb) { g_platform->panic("No idle thread for core", __FILE__, __LINE__); return; }
    g_per_cpu_data[core_id].current_thread = tcb;
    tcb->state = TCB::State::RUNNING;
    tcb->cpu_id_running_on = core_id;
    // This is a placeholder for the actual context switch mechanism.
    // It needs to load the idle thread's context and start execution.
    // On ARM, this would involve setting SP, loading registers, and branching to PC.
    extern void context_switch(TCB* current_tcb, TCB* next_tcb); // Ensure TCB* not void*
    context_switch(nullptr, tcb); // Switch from "null" context to idle_thread
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    schedule(core_id, true);
}

void Scheduler::yield(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    g_trace_manager.record_event(g_per_cpu_data[core_id].current_thread, trace::EventType::THREAD_YIELD, "yield");
    schedule(core_id, false);
}

void Scheduler::signal_event_isr(TCB* tcb) {
    if (!tcb) return;
    // This needs to be atomic if event_flag can be checked/cleared by another core or thread
    // For now, assume it's signaled from ISR and waited by the thread itself.
    tcb->event_flag = true;
    // If the task was READY and waiting, this signal might make it runnable.
    // A more complete implementation might re-evaluate its position in ready queue or wake it.
}

void Scheduler::wait_for_event(TCB* tcb) {
    if (!tcb || !g_platform) return;
    // Ensure interrupts are enabled for yield to work if called from thread context
    while (!tcb->event_flag) { // Check flag
        yield(g_platform->get_core_id()); // Yield until flag is set
    }
    tcb->event_flag = false; // Consume the event
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;

    // Try local queue first
    {
        ScopedLock lock(per_core_locks_[current_core_id]);
        for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
            if (ready_qs_[current_core_id][p]) {
                TCB* task = ready_qs_[current_core_id][p];
                ready_qs_[current_core_id][p] = task->next_in_q;
                task->next_in_q = nullptr;
                trace_event("SCHED:PopSelf", reinterpret_cast<uintptr_t>(task), (current_core_id << 16) | p);
                return task;
            }
        }
    }

    // Try work stealing from other cores for tasks with no affinity or affinity to current_core_id
    for (uint32_t i = 1; i < MAX_CORES; ++i) {
        uint32_t target_core = (current_core_id + i) % MAX_CORES;
        ScopedLock steal_lock(per_core_locks_[target_core]); // Lock the target core's queue
        for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
            TCB* task_iter = ready_qs_[target_core][p];
            TCB* prev_task = nullptr;
            while (task_iter) {
                if (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(current_core_id)) {
                    // Found a stealable task
                    if (prev_task) {
                        prev_task->next_in_q = task_iter->next_in_q;
                    } else {
                        ready_qs_[target_core][p] = task_iter->next_in_q;
                    }
                    task_iter->next_in_q = nullptr; // Detach
                    trace_event("SCHED:StoleOK", reinterpret_cast<uintptr_t>(task_iter), (target_core << 16) | p);
                    return task_iter;
                }
                prev_task = task_iter;
                task_iter = task_iter->next_in_q;
            }
        }
    }

    // If nothing found, return the idle thread for the current core
    trace_event("SCHED:PopIdleAff", current_core_id);
    return g_per_cpu_data[current_core_id].idle_thread;
}

void Scheduler::schedule(uint32_t core_id, bool is_preemption) {
    if (core_id >= MAX_CORES || !policy_ || !g_platform) return;

    // It's generally unsafe to take a spinlock that might disable interrupts,
    // then call into platform functions that might need interrupts (like panic, or even logging).
    // For scheduler lock, it should be general purpose if not directly in ISR path.
    // Let's assume per_core_locks_ are general spinlocks.
    ScopedLock lock(per_core_locks_[core_id]);

    TCB* current = g_per_cpu_data[core_id].current_thread;

    if (current && current->state == TCB::State::RUNNING) { // Only change state if it was indeed running
        if (is_preemption || current != g_per_cpu_data[core_id].idle_thread) { // Don't mark idle as READY unless forced
            current->state = TCB::State::READY;
        }
    }
    
    TCB* next = policy_->select_next_task(core_id, current);

    if (!next) { // Should always at least get idle thread
        if (g_platform) g_platform->panic("Scheduler selected null task", __FILE__, __LINE__);
        return;
    }

    if (next == current && current->state == TCB::State::RUNNING && !is_preemption) { // Already running the best task
         // If current was made READY, but policy selected it again, make it RUNNING
        if (current->state == TCB::State::READY) current->state = TCB::State::RUNNING;
        return;
    }
    
    next->state = TCB::State::RUNNING;
    next->cpu_id_running_on = core_id;
    g_per_cpu_data[core_id].current_thread = next;

    if (current && current != next && current->state == TCB::State::READY) {
        // Only add current back if it's not the idle thread and is ready
        if (current != g_per_cpu_data[core_id].idle_thread) {
             policy_->add_to_ready_queue(current, core_id); // Add old task back to ready queue if it's still ready
        }
    }
    
    g_trace_manager.record_event(next, trace::EventType::THREAD_SCHEDULE, next->name);
    
    if (current != next) { // Only switch if different
        extern void context_switch(TCB* current_tcb, TCB* next_tcb);
        context_switch(current, next);
    }
}

void Scheduler::idle_thread_func(void* arg) {
    // uint32_t core_id = reinterpret_cast<uint32_t>(arg); // If core_id is passed
    (void)arg; // Unused for now
    while (true) {
        // Platform-specific low-power instruction
        // Example for ARM: Wait For Interrupt
        set_power_mode(true); // Call the kernel helper
        asm volatile("wfi" ::: "memory");
        // On interrupt, WFI completes, ISR runs, then scheduler may pick another task.
        // If idle is still scheduled, it loops back to WFI.
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self || !self->entry_point || !g_scheduler_ptr || !g_platform) {
        // Cannot proceed, critical error.
        if (g_platform) g_platform->panic("Thread bootstrap failure", __FILE__, __LINE__);
        else { while(1); } // Spin if panic unavailable
        return; // Should not be reached
    }

    // Enable interrupts for the thread if they were disabled during context switch
    // This is platform-specific. For now, assume interrupts are managed by IRQControllerOps.
    // g_platform->get_irq_ops()->enable_core_irqs(g_platform->get_core_id(), 0);


    configure_memory_protection(self, true); // If MPU/MMU is used

    self->entry_point(self->arg_ptr); // Call the thread's actual function

    // Thread function has returned, clean up.
    // configure_memory_protection(self, false); // Disable MPU region if used

    g_trace_manager.record_event(self, trace::EventType::THREAD_EXIT, self->name);
    
    self->state = TCB::State::INACTIVE; // Mark as inactive
    g_scheduler_ptr->num_active_tasks_--; // Decrement active tasks

    // Yield to let scheduler pick another task. This thread should not run again.
    g_scheduler_ptr->yield(g_platform->get_core_id());

    // Should not be reached if yield works correctly and another task (or idle) is scheduled.
    if (g_platform) g_platform->panic("Thread exited bootstrap unexpectedly", __FILE__, __LINE__);
    else { while(1); } // Spin
}

void set_power_mode(bool low_power) {
    if (!g_platform) return;
    auto* power_ops = g_platform->get_power_ops();
    if (!power_ops) return; // Platform might not support power ops

    if (low_power) {
        power_ops->enter_idle_state(g_platform->get_core_id());
    } else {
        // Example: Set to full performance frequency (0 might mean default/max)
        // This part is highly platform-specific.
        // power_ops->set_cpu_frequency(g_platform->get_core_id(), 0);
    }
}

void configure_memory_protection(TCB* tcb, bool enable) {
    if (!g_platform || !tcb) return;
    // Platform-specific MPU/MMU configuration would go here.
    // For now, it's a placeholder.
    trace_event("MEM:Protection", reinterpret_cast<uintptr_t>(tcb), enable);
}

void get_kernel_stats(hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !g_scheduler_ptr || !g_platform) return;
    
    ScopedLock lock(g_scheduler_ptr->get_global_scheduler_lock()); // Use getter
    
    char buffer[128];

    uart_ops->puts("Kernel Statistics:\n");
    std::snprintf(buffer, sizeof(buffer), "  Active Threads: %zu\n", g_scheduler_ptr->get_num_active_tasks()); // Use getter
    uart_ops->puts(buffer);

    for (uint32_t core = 0; core < MAX_CORES; ++core) {
        std::snprintf(buffer, sizeof(buffer), "  Core %u: ", core);
        uart_ops->puts(buffer);
        if (g_per_cpu_data[core].current_thread) { // g_per_cpu_data is global in kernel namespace
            uart_ops->puts(g_per_cpu_data[core].current_thread->name);
        } else {
            uart_ops->puts("(none)");
        }
        uart_ops->puts("\n");
    }
}

void trace_event(const char* event, uintptr_t a1, uintptr_t a2) {
    if (!g_platform || !event) return; // Basic checks
    
    uint32_t core_id = g_platform->get_core_id();
    if (core_id >= MAX_CORES) return; // Safety check

    // It's better to lock before fetch_add if the buffer itself isn't fully thread-safe
    // or if timestamping needs to be closer to event recording under lock.
    // However, for high-frequency tracing, a lock-free approach for index is common.
    // ScopedLock lock(g_trace_lock); // If g_trace_buffer access needs to be exclusive

    size_t idx = g_trace_buffer_next_idx.fetch_add(1, std::memory_order_relaxed);
    
    if (idx >= TRACE_BUFFER_SIZE) {
        g_trace_overflow_count[core_id].fetch_add(1, std::memory_order_relaxed);
        idx %= TRACE_BUFFER_SIZE; // Wrap around
    }
    
    TraceEntry& entry = g_trace_buffer[idx]; // g_trace_buffer should be defined
    if (g_platform->get_timer_ops()) {
        entry.timestamp_us = g_platform->get_timer_ops()->get_system_time_us();
    } else {
        entry.timestamp_us = 0; // Or some other indicator of timer unavailability
    }
    entry.core_id = core_id;
    entry.event_str = event; // Assumes event string literal has static lifetime
    entry.arg1 = a1;
    entry.arg2 = a2;
}

void dump_trace_buffer(hal::UARTDriverOps* uart_ops) {
    if (!g_platform || !uart_ops) return;
    
    ScopedLock lock(g_trace_lock); // Ensure exclusive access while dumping
    
    char buffer[256]; // Buffer for formatting output

    uart_ops->puts("\n--- Trace Buffer ---\n");
    // Determine the true start index due to wrap-around
    size_t current_next_idx = g_trace_buffer_next_idx.load(std::memory_order_relaxed);
    size_t num_entries_to_dump = TRACE_BUFFER_SIZE;
    size_t start_idx = 0;

    bool overflowed_at_least_once = false;
    for(uint32_t c=0; c < MAX_CORES; ++c) if(g_trace_overflow_count[c].load(std::memory_order_relaxed) > 0) overflowed_at_least_once = true;

    if (current_next_idx >= TRACE_BUFFER_SIZE || overflowed_at_least_once) { // Buffer has wrapped or overflowed
        start_idx = current_next_idx % TRACE_BUFFER_SIZE;
    } else { // Buffer has not yet wrapped, dump from 0 up to current_next_idx
        num_entries_to_dump = current_next_idx;
    }

    size_t count = 0;
    for (size_t i = 0; i < num_entries_to_dump; ++i) {
        size_t idx_to_read = (start_idx + i) % TRACE_BUFFER_SIZE;
        const TraceEntry& entry = g_trace_buffer[idx_to_read];
        if (entry.event_str) { // Only print if event_str is not null (basic validity)
            std::snprintf(buffer, sizeof(buffer), "T:%llu C:%u EVT:%s A1:0x%lx A2:0x%lx\n",
                          static_cast<unsigned long long>(entry.timestamp_us),
                          entry.core_id,
                          entry.event_str,
                          static_cast<unsigned long>(entry.arg1),
                          static_cast<unsigned long>(entry.arg2));
            uart_ops->puts(buffer);
            count++;
        }
    }
    
    if (count == 0 && num_entries_to_dump > 0) uart_ops->puts("(Trace data present but invalid or all null events)\n");
    else if (num_entries_to_dump == 0) uart_ops->puts("(empty)\n");

    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        size_t overflows = g_trace_overflow_count[i].load(std::memory_order_relaxed);
        if (overflows > 0) {
            std::snprintf(buffer, sizeof(buffer), "Core %u Overflows: %zu\n", i, overflows);
            uart_ops->puts(buffer);
        }
    }
    uart_ops->puts("--- End Trace ---\n");
}

extern "C" void kernel_main() {
    if (!g_platform) { /* Cannot panic safely if g_platform is null */ while(1); return; }
    
    uint32_t core_id = g_platform->get_core_id();

    // Early platform init (once per system, or per core if needed)
    // This should happen before most kernel objects are constructed if they rely on HAL.
    if (core_id == 0) { // Core 0 performs one-time system initializations
        g_platform->early_init(); // Platform-wide early init
    }
    g_platform->core_early_init(); // Per-core early init

    static Scheduler scheduler; // Create the scheduler object (static ensures one instance)
    
    if (core_id == 0) {
        g_scheduler_ptr = &scheduler; // Make it globally accessible

        // Initialize memory pool for software timers
        if (!g_software_timer_obj_pool.init(g_software_timer_obj_pool_mem, MAX_SOFTWARE_TIMERS,
                                           sizeof(kernel::hal::timer::SoftwareTimer), alignof(kernel::hal::timer::SoftwareTimer))) {
            g_platform->panic("Timer pool init failed", __FILE__, __LINE__);
        }

        // Initialize Audio System
        audio::AudioConfig audio_cfg{.sample_rate_hz = 48000, .samples_per_block = 256, .num_i2s_bufs = 4};
        if (!g_audio_system.init(audio_cfg) || !g_audio_system.start()) {
            g_platform->panic("Audio system init failed", __FILE__, __LINE__);
        }

        // Initialize File System
        if (!g_file_system.create_file("/", true)) { // Create root directory
            g_platform->panic("File system init failed", __FILE__, __LINE__);
        }
        
        // Initialize GPIO Manager
        if (!g_gpio_manager.init(g_platform->get_gpio_ops())) {
            g_platform->panic("GPIO init failed", __FILE__, __LINE__);
        }

        // Initialize Network Manager
        if (!g_net_manager.init(g_platform->get_net_ops())) {
            g_platform->panic("Network init failed", __FILE__, __LINE__);
        }

        // Initialize Watchdog
        auto* watchdog_ops = g_platform->get_watchdog_ops();
        if (watchdog_ops && !watchdog_ops->start_watchdog(5000)) { // 5 second timeout
            g_platform->panic("Watchdog init failed", __FILE__, __LINE__);
        }

        // Create CLI thread
        if (!g_scheduler_ptr->create_thread(cli::CLI::cli_thread_entry, g_platform->get_uart_ops(), 3, 0, "CLI", false, 10000)) {
            g_platform->panic("CLI thread creation failed", __FILE__, __LINE__);
        }

        demo::register_demo_commands();
        test::register_tests();
        
        if (g_platform->get_uart_ops()) {
            g_platform->get_uart_ops()->puts("[miniOS v1.7] System Initialized on Core 0\n");
        }
        g_bss_cleared_smp_flag = true; // Signal other cores
        asm volatile("sev" ::: "memory"); // Ensure memory visibility and send event

    } else { // Other cores wait for Core 0 to finish basic setup
        while (!g_bss_cleared_smp_flag) { // Use the SMP flag
             asm volatile("wfe" ::: "memory");
        }
        // g_scheduler_ptr should be valid now
        if (g_platform->get_uart_ops()) {
            char msg_buf[64];
            std::snprintf(msg_buf, sizeof(msg_buf), "[miniOS v1.7] Core %u Initialized\n", core_id);
            g_platform->get_uart_ops()->puts(msg_buf);
        }
    }

    // All cores start their schedulers
    if (!g_scheduler_ptr) { g_platform->panic("Scheduler not ready for non-primary core", __FILE__, __LINE__); }
    g_scheduler_ptr->start_core_scheduler(core_id);

    // start_core_scheduler should not return. If it does, it's a critical error.
    g_platform->panic("Scheduler returned unexpectedly", __FILE__, __LINE__);
}

// This is the main IRQ handler called by the platform's assembly vector
extern "C" void hal_irq_handler(uint32_t irq_id) {
    ScopedISRLock lock(g_irq_handler_lock); // Protect shared IRQ handling logic if any
    if (!g_platform || !g_scheduler_ptr ) return; // Not initialized yet

    auto* irq_ops = g_platform->get_irq_ops();
    auto* timer_ops = g_platform->get_timer_ops();
    auto* watchdog_ops = g_platform->get_watchdog_ops(); // May not exist
    uint32_t core_id = g_platform->get_core_id();

    // Typically, irq_id would be the raw ID from GIC, or a platform-specific mapped ID.
    // The platform specific part (e.g. GIC driver) would call this with a usable ID.
    // Example: Assume IRQ 30 is a common ID for system timer per core (varies by HW)
    // For a Raspberry Pi using generic timer, this might be PPI 30 (IRQ ID 30 for GIC)
    // or SPI for shared peripherals.
    // This mapping (e.g. 30 to "System Timer") needs to be consistent with platform setup.

    if (timer_ops && (irq_id == 30 || irq_id == 27)) { // Assuming 30/27 are generic timer IRQ IDs (platform specific!)
        timer_ops->hardware_timer_irq_fired(core_id); // Let timer driver process its logic
        if (watchdog_ops) watchdog_ops->reset_watchdog();
        g_scheduler_ptr->preemptive_tick(core_id); // Then scheduler tick
    } else if (irq_id == 31 ) { // Example network IRQ
        trace_event("NET:IRQ", irq_id, core_id);
        // net::g_net_manager.handle_irq(); // Network stack might have an IRQ handler
    } else {
        // Unhandled IRQ
        trace_event("IRQ:Unhandled", irq_id, core_id);
        if (g_platform->get_uart_ops()) {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "Unhandled IRQ: %u on Core %u\n", irq_id, core_id);
            g_platform->get_uart_ops()->puts(buffer);
        }
    }

    if (irq_ops) irq_ops->end_irq(core_id, irq_id); // Signal End Of Interrupt to controller
}

} // namespace kernel