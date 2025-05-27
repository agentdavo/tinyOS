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

#include "miniOS.hpp" 
#include "util.hpp"   
#include "cli.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "trace.hpp"
#include "fs.hpp"
#include "net.hpp"
#include "gpio.hpp"
#include <cstring>    
#include <cstdio>     
#include <cassert>    
#include <algorithm>  // For std::max

// Declare demo and test registration functions
namespace demo { void register_demo_commands(); }
namespace test { void register_tests(); }

namespace kernel {

hal::Platform* g_platform = nullptr; 
Scheduler* g_scheduler_ptr = nullptr; 
volatile bool g_bss_cleared_smp_flag = false; 
alignas(64) uint8_t g_audio_pool_mem[64 * 1024]; 
FixedMemoryPool g_audio_pool; 
alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(kernel::hal::timer::SoftwareTimer)]; 
FixedMemoryPool g_software_timer_obj_pool; 
audio::AudioSystem g_audio_system; 
trace::TraceManager g_trace_manager; 
fs::FileSystem g_file_system; 
net::NetManager g_net_manager; 
gpio::GPIOManager g_gpio_manager; 
Spinlock g_irq_handler_lock; 
Spinlock g_trace_lock; 
Spinlock g_audio_system_lock; 

std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::atomic<size_t> g_trace_buffer_next_idx{0};
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count{}; 


alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() : lock_id_(next_lock_id_++) {
    // MAX_LOCKS is global, g_platform might not be set here yet.
    // This check should ideally be done after platform is up or MAX_LOCKS handled differently.
    // For now, assuming it's okay or g_platform check will prevent panic if it's null.
    if (next_lock_id_ >= MAX_LOCKS && g_platform) { 
        g_platform->panic("Exceeded maximum number of spinlocks", __FILE__, __LINE__);
    }
}

void Spinlock::acquire_isr_safe() {
    if (!g_platform || !g_platform->get_irq_ops()) {
        while(true) {} 
        return;
    }
    g_platform->get_irq_ops()->disable_core_irqs(g_platform->get_core_id());
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
    }
}

void Spinlock::release_isr_safe() {
    if (!g_platform || !g_platform->get_irq_ops()) {
        while(true) {}
        return;
    }
    lock_flag_.store(false, std::memory_order_release);
    g_platform->get_irq_ops()->enable_core_irqs(g_platform->get_core_id(), 0); 
}

void Spinlock::acquire_general() {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("wfe" ::: "memory");
        #endif
    }
}

void Spinlock::release_general() {
    lock_flag_.store(false, std::memory_order_release);
    #if defined(__aarch64__) || defined(__arm__)
    asm volatile("sev" ::: "memory");
    #endif
}

bool FixedMemoryPool::init(void* base, size_t num_blocks, size_t block_size_user, size_t alignment_user_data) {
    if (!base || num_blocks == 0 || block_size_user == 0) return false;
    
    size_t actual_alignment = std::max(alignment_user_data, sizeof(Block*));
    if (actual_alignment == 0 || (actual_alignment & (actual_alignment - 1)) != 0) {
        actual_alignment = sizeof(Block*);
    }

    ScopedLock lock(pool_lock_); 
    header_actual_size_ = (sizeof(Block) + actual_alignment - 1) & ~(actual_alignment - 1);
    
    // User data starts after header. Ensure user data start is aligned.
    // The total block must accommodate this aligned user data.
    size_t user_data_start_offset = header_actual_size_;
    if (alignment_user_data > 0) { // Only adjust if specific user alignment is requested
        user_data_start_offset = (header_actual_size_ + alignment_user_data - 1) & ~(alignment_user_data - 1);
    }
    
    block_storage_size_ = user_data_start_offset + block_size_user;
    block_storage_size_ = (block_storage_size_ + actual_alignment - 1) & ~(actual_alignment - 1); // Align whole block

    // Update header_actual_size_ to reflect the true offset to user data if it changed due to user_data_start_offset
    header_actual_size_ = user_data_start_offset;


    pool_memory_start_ = static_cast<uint8_t*>(base);
    num_total_blocks_ = num_free_blocks_ = num_blocks;
    free_head_ = nullptr;

    uint8_t* current_block_ptr = pool_memory_start_;
    for (size_t i = 0; i < num_total_blocks_; ++i) {
        // This loop assumes 'base' itself is aligned enough that current_block_ptr will be.
        // If not, each block might need individual alignment from current_block_ptr.
        // However, with block_storage_size_ being aligned, subsequent blocks should be aligned
        // if the first one is.
        Block* block = reinterpret_cast<Block*>(current_block_ptr);
        block->next = free_head_;
        free_head_ = block;
        current_block_ptr += block_storage_size_; 
    }
    return true;
}

void* FixedMemoryPool::allocate() {
    ScopedLock lock(pool_lock_);
    if (!free_head_) {
        if (g_platform) g_platform->panic("Memory pool exhausted", __FILE__, __LINE__);
        return nullptr;
    }
    Block* block_header = free_head_;
    free_head_ = block_header->next;
    num_free_blocks_--;
    return static_cast<uint8_t*>(static_cast<void*>(block_header)) + header_actual_size_;
}

void FixedMemoryPool::free_block(void* user_data_ptr) {
    if (!user_data_ptr) return;
    Block* block_header = reinterpret_cast<Block*>(static_cast<uint8_t*>(user_data_ptr) - header_actual_size_);
    
    ScopedLock lock(pool_lock_);
    block_header->next = free_head_;
    free_head_ = block_header;
    num_free_blocks_++;
}

template<typename T, size_t Capacity>
bool SPSCQueue<T, Capacity>::enqueue(T* item) noexcept {
    size_t current_tail = tail_.load(std::memory_order_acquire);
    size_t next_tail = (current_tail + 1) & (Capacity - 1); 
    if (next_tail == head_.load(std::memory_order_acquire)) {
        return false; 
    }
    items_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
}

template<typename T, size_t Capacity>
T* SPSCQueue<T, Capacity>::dequeue() noexcept {
    size_t current_head = head_.load(std::memory_order_acquire);
    if (current_head == tail_.load(std::memory_order_acquire)) {
        return nullptr; 
    }
    T* item = items_[current_head];
    head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
    return item;
}

template class SPSCQueue<audio::AudioBuffer, 16>; 

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* /*current_task*/) { 
    if (core_id >= MAX_CORES || !g_scheduler_ptr) {
        if(g_platform) g_platform->panic("EDFPolicy: Invalid args or scheduler missing", __FILE__, __LINE__);
        return nullptr;
    }

    TCB* earliest_task = nullptr;
    uint64_t earliest_deadline = UINT64_MAX;
    TCB* chosen_prev_in_list = nullptr;
    int chosen_priority = -1;

    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);

    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        TCB* task_iter = g_scheduler_ptr->ready_qs_[core_id][p];
        TCB* current_prev = nullptr; 
        while (task_iter) {
            if (task_iter->state == TCB::State::READY && 
                task_iter->deadline_us > 0 && task_iter->deadline_us < earliest_deadline &&
                (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(core_id))) {
                
                earliest_deadline = task_iter->deadline_us;
                earliest_task = task_iter;
                chosen_prev_in_list = current_prev; 
                chosen_priority = p; 
            }
            current_prev = task_iter;
            task_iter = task_iter->next_in_q;
        }
    }

    if (earliest_task) {
        if (chosen_prev_in_list) {
            chosen_prev_in_list->next_in_q = earliest_task->next_in_q;
        } else {
            g_scheduler_ptr->ready_qs_[core_id][chosen_priority] = earliest_task->next_in_q;
        }
        earliest_task->next_in_q = nullptr; 
        trace_event("SCHED:PopEDF", reinterpret_cast<uintptr_t>(earliest_task), core_id);
        return earliest_task; 
    }

    return g_scheduler_ptr->pop_highest_priority_ready_task(core_id); 
}

void EDFPolicy::add_to_ready_queue(TCB* tcb, uint32_t core_id) {
    if (!tcb || tcb->priority < 0 || static_cast<size_t>(tcb->priority) >= MAX_PRIORITY_LEVELS || core_id >= MAX_CORES) {
        if (g_platform) g_platform->panic("EDFPolicy: Invalid ready queue args", __FILE__, __LINE__);
        return;
    }
    if (!g_scheduler_ptr) { if(g_platform) g_platform->panic("EDFPolicy: Scheduler not initialized for AddRdy", __FILE__, __LINE__); return;}
    
    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);
    trace_event("SCHED:AddRdy", reinterpret_cast<uintptr_t>(tcb), (core_id << 16) | tcb->priority);
    tcb->next_in_q = g_scheduler_ptr->ready_qs_[core_id][tcb->priority];
    g_scheduler_ptr->ready_qs_[core_id][tcb->priority] = tcb;
    tcb->state = TCB::State::READY;
}

Scheduler::Scheduler() : policy_(nullptr) {
    policy_ = new (std::nothrow) EDFPolicy(); 
    if (!policy_) {
        if (g_platform) g_platform->panic("Failed to allocate scheduler policy", __FILE__, __LINE__);
        else { while(1); } 
    }

    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        char idle_name[MAX_NAME_LENGTH];
        std::snprintf(idle_name, MAX_NAME_LENGTH, "IdleCore%u", i);
        auto* tcb = create_thread(idle_thread_func, reinterpret_cast<void*>(static_cast<uintptr_t>(i)), 
                                  0, static_cast<int>(i), idle_name, true);
        if (!tcb) {
            if (g_platform) g_platform->panic("Failed to create idle thread", __FILE__, __LINE__);
            else { while(1); }
        }
        g_per_cpu_data[i].idle_thread = tcb;
    }
}

Scheduler::~Scheduler() {
    delete policy_; 
}

TCB* Scheduler::create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle, uint64_t deadline_us) {
    if (!fn || !name) return nullptr;
    ScopedLock lock(scheduler_global_lock_); 
    
    if (num_active_tasks_.load(std::memory_order_relaxed) >= MAX_THREADS) return nullptr;

    for (size_t i = 0; i < MAX_THREADS; ++i) {
        if (g_task_tcbs[i].state == TCB::State::INACTIVE || g_task_tcbs[i].state == TCB::State::ZOMBIE) { // Reuse ZOMBIE too
            TCB& tcb = g_task_tcbs[i];
            
            // Manual reset of TCB members
            for (int j=0; j<31; ++j) tcb.regs[j] = 0;
            tcb.sp = 0;
            tcb.pc = 0;
            tcb.pstate = 0; // Corrected from status
            tcb.entry_point = nullptr;
            tcb.arg_ptr = nullptr;
            tcb.stack_base = nullptr;
            tcb.stack_size = 0;
            tcb.priority = 0;
            tcb.core_affinity = -1;
            tcb.cpu_id_running_on = static_cast<uint32_t>(-1);
            tcb.name[0] = '\0';
            tcb.next_in_q = nullptr;
            tcb.event_flag.store(false, std::memory_order_relaxed); 
            tcb.deadline_us = 0;
            // State will be set below
            
            tcb.entry_point = fn;
            tcb.arg_ptr = const_cast<void*>(arg); 
            tcb.priority = (prio >= 0 && static_cast<size_t>(prio) < MAX_PRIORITY_LEVELS) ? prio : 0;
            tcb.core_affinity = (affinity >= -1 && affinity < static_cast<int>(MAX_CORES)) ? affinity : -1;
            
            if (!kernel::util::safe_strcpy(tcb.name, name, MAX_NAME_LENGTH)) {
                return nullptr;
            }
            
            tcb.stack_base = g_task_stacks[i].data();
            tcb.stack_size = DEFAULT_STACK_SIZE;
            
            tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size);
            tcb.sp &= ~0xFUL; 

            tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap); 
            // tcb.pstate = 0; // Initial PSTATE (e.g. EL1h, IRQs unmasked for user threads) - platform specific. 0 might be fine for simple cases.
            
            tcb.deadline_us = deadline_us;
            
            if (!is_idle) {
                tcb.state = TCB::State::READY;
                g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name); 
                uint32_t target_core = (tcb.core_affinity != -1 && static_cast<size_t>(tcb.core_affinity) < MAX_CORES) 
                                     ? static_cast<uint32_t>(tcb.core_affinity) 
                                     : (g_platform ? g_platform->get_core_id() : 0); 
                policy_->add_to_ready_queue(&tcb, target_core);
                num_active_tasks_.fetch_add(1, std::memory_order_relaxed);
            } else {
                tcb.state = TCB::State::READY; 
            }
            return &tcb;
        }
    }
    return nullptr; 
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!g_platform || core_id >= MAX_CORES || !g_platform->get_irq_ops() || !g_platform->get_timer_ops()) {
        while(true) {}
        return;
    }
    g_platform->get_irq_ops()->init_cpu_interface(core_id); 
    g_platform->get_irq_ops()->enable_core_irqs(core_id, 0); 
    g_platform->get_timer_ops()->init_core_timer_interrupt(core_id); // Pass core_id

    auto* idle_tcb = g_per_cpu_data[core_id].idle_thread;
    if (!idle_tcb) { 
        g_platform->panic("No idle thread for core", __FILE__, __LINE__); 
        return; 
    }
    g_per_cpu_data[core_id].current_thread = idle_tcb;
    idle_tcb->state = TCB::State::RUNNING;
    idle_tcb->cpu_id_running_on = core_id;
    
    extern void context_switch_to(TCB* next_tcb); 
    context_switch_to(idle_tcb); 
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    if (g_platform && g_platform->get_timer_ops()) {
        g_platform->get_timer_ops()->ack_core_timer_interrupt(core_id);
    }
    schedule(core_id, true); 
}

void Scheduler::yield(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    if(g_per_cpu_data[core_id].current_thread) { 
         g_trace_manager.record_event(g_per_cpu_data[core_id].current_thread, trace::EventType::THREAD_YIELD, "yield");
    }
    schedule(core_id, false); 
}

void Scheduler::signal_event_isr(TCB* tcb) {
    if (!tcb) return;
    tcb->event_flag.store(true, std::memory_order_release);
}

void Scheduler::wait_for_event(TCB* tcb) {
    if (!tcb || !g_platform) return;
    while (!tcb->event_flag.load(std::memory_order_acquire)) { 
        yield(g_platform->get_core_id()); 
    }
    tcb->event_flag.store(false, std::memory_order_relaxed); 
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;

    // Caller (schedule() or policy) must hold per_core_locks_[current_core_id]
    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        if (ready_qs_[current_core_id][p]) {
            TCB* task = ready_qs_[current_core_id][p];
            ready_qs_[current_core_id][p] = task->next_in_q;
            task->next_in_q = nullptr;
            trace_event("SCHED:PopSelf", reinterpret_cast<uintptr_t>(task), (current_core_id << 16) | p);
            return task;
        }
    }

    // Work stealing attempt - needs careful locking if enabled
    // For now, simplified: EDFPolicy handles complex choices, this is a basic fallback.
    // If work stealing is implemented, it must acquire locks for other cores' queues.
    // Example (conceptual, needs proper locking strategy for production):
    // for (uint32_t i = 1; i < MAX_CORES; ++i) {
    //     uint32_t target_core = (current_core_id + i) % MAX_CORES;
    //     ScopedLock steal_lock(per_core_locks_[target_core]); 
    //     // ... logic to find and steal task ...
    // }

    trace_event("SCHED:PopIdleAff", current_core_id);
    return g_per_cpu_data[current_core_id].idle_thread;
}

void Scheduler::schedule(uint32_t core_id, bool is_preemption) {
    if (core_id >= MAX_CORES || !policy_ || !g_platform) {
        if(g_platform) g_platform->panic("Schedule: Invalid args", __FILE__, __LINE__);
        return;
    }

    ScopedLock core_lock(per_core_locks_[core_id]); 

    TCB* current = g_per_cpu_data[core_id].current_thread;

    if (current && current->state == TCB::State::RUNNING) { 
        if (current != g_per_cpu_data[core_id].idle_thread || is_preemption) {
            current->state = TCB::State::READY;
        }
    }
    
    TCB* next = policy_->select_next_task(core_id, current); 

    if (!next) { 
        if (g_platform) g_platform->panic("Scheduler selected null task", __FILE__, __LINE__);
        return; 
    }

    if (next == current) { 
        if (current) current->state = TCB::State::RUNNING; 
        return; 
    }
    
    next->state = TCB::State::RUNNING;
    next->cpu_id_running_on = core_id;
    g_per_cpu_data[core_id].current_thread = next;

    if (current && current != next && current->state == TCB::State::READY) {
        if (current != g_per_cpu_data[core_id].idle_thread) {
             // If the policy didn't re-queue 'current' (e.g. it was preempted and is still highest EDF),
             // then it needs to be added back. EDFPolicy's select_next_task extracts the chosen task.
             // So, 'current' if still READY, needs to be added back by policy.
             policy_->add_to_ready_queue(current, core_id); 
        }
    }
    
    g_trace_manager.record_event(next, trace::EventType::THREAD_SCHEDULE, next->name);
    
    if (current != next) { 
        extern void context_switch(TCB* current_tcb, TCB* next_tcb); 
        context_switch(current, next);
    }
}

void Scheduler::idle_thread_func(void* arg) {
    (void)arg; 
    while (true) {
        set_power_mode(true); 
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("wfi" ::: "memory");
        #else
        for(volatile int i=0; i<100000; ++i); // Generic placeholder
        #endif
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self || !self->entry_point || !g_scheduler_ptr || !g_platform) {
        if (g_platform) g_platform->panic("Thread bootstrap failure", __FILE__, __LINE__);
        else { while(1); } 
        return; 
    }

    configure_memory_protection(self, true); 

    self->entry_point(self->arg_ptr); 

    {
        ScopedLock lock(g_scheduler_ptr->scheduler_global_lock_);
        g_trace_manager.record_event(self, trace::EventType::THREAD_EXIT, self->name);
        self->state = TCB::State::ZOMBIE; 
        g_scheduler_ptr->num_active_tasks_.fetch_sub(1, std::memory_order_relaxed);
    }
    
    g_scheduler_ptr->schedule(self->cpu_id_running_on, false); 

    if (g_platform) g_platform->panic("Thread exited bootstrap unexpectedly", __FILE__, __LINE__);
    else { while(1); } 
}

void set_power_mode(bool enter_low_power) {
    if (!g_platform) return;
    auto* power_ops = g_platform->get_power_ops();
    if (!power_ops) return; 

    if (enter_low_power) {
        power_ops->enter_idle_state(g_platform->get_core_id());
    } else {
        // power_ops->set_cpu_frequency(g_platform->get_core_id(), 0); // Example: restore normal frequency
    }
}

void configure_memory_protection(TCB* tcb, bool enable_for_task) {
    if (!g_platform || !tcb) return;
    trace_event("MEM:Protection", reinterpret_cast<uintptr_t>(tcb), enable_for_task);
}

void get_kernel_stats(hal::UARTDriverOps* uart_ops) { 
    if (!uart_ops || !g_scheduler_ptr || !g_platform) return;
    
    ScopedLock lock(g_scheduler_ptr->get_global_scheduler_lock()); 
    
    char buffer[128]; 

    uart_ops->puts("Kernel Statistics:\n");
    std::snprintf(buffer, sizeof(buffer), "  Active Threads: %zu\n", g_scheduler_ptr->get_num_active_tasks());
    uart_ops->puts(buffer);

    for (uint32_t core = 0; core < MAX_CORES; ++core) { 
        if (core < g_per_cpu_data.size() && g_per_cpu_data[core].current_thread) {
             std::snprintf(buffer, sizeof(buffer), "  Core %u: Running '%s' (prio %d)\n", 
                           core, 
                           g_per_cpu_data[core].current_thread->name,
                           g_per_cpu_data[core].current_thread->priority);
        } else {
             std::snprintf(buffer, sizeof(buffer), "  Core %u: (idle or no thread info)\n", core);
        }
        uart_ops->puts(buffer);
    }
}

void trace_event(const char* event_str, uintptr_t arg1, uintptr_t arg2) {
    if (!g_platform || !event_str) return; 
    
    uint32_t core_id = g_platform->get_core_id();
    if (core_id >= MAX_CORES) { 
        core_id = 0; 
    }

    size_t idx = g_trace_buffer_next_idx.fetch_add(1, std::memory_order_relaxed);
    
    TraceEntry* entry_ptr;
    if (idx >= TRACE_BUFFER_SIZE) {
        g_trace_overflow_count[core_id].fetch_add(1, std::memory_order_relaxed);
        entry_ptr = &g_trace_buffer[idx % TRACE_BUFFER_SIZE]; 
    } else {
        entry_ptr = &g_trace_buffer[idx];
    }
    
    if (g_platform->get_timer_ops()) {
        entry_ptr->timestamp_us = g_platform->get_timer_ops()->get_system_time_us();
    } else {
        entry_ptr->timestamp_us = 0; 
    }
    entry_ptr->core_id = core_id;
    entry_ptr->event_str = event_str; 
    entry_ptr->arg1 = arg1;
    entry_ptr->arg2 = arg2;
}

void dump_trace_buffer(hal::UARTDriverOps* uart_ops) { 
    if (!g_platform || !uart_ops) return;
    
    ScopedLock lock(g_trace_lock); 
    
    char buffer[256]; 

    uart_ops->puts("\n--- Trace Buffer ---\n");
    size_t current_next_idx = g_trace_buffer_next_idx.load(std::memory_order_relaxed);
    size_t num_entries_to_dump = TRACE_BUFFER_SIZE;
    size_t start_idx_read = 0;

    bool has_overflowed_any_core = false;
    for(uint32_t c=0; c < MAX_CORES; ++c) {
        if(g_trace_overflow_count[c].load(std::memory_order_relaxed) > 0) {
            has_overflowed_any_core = true;
            break;
        }
    }

    if (current_next_idx >= TRACE_BUFFER_SIZE || has_overflowed_any_core) { 
        start_idx_read = current_next_idx % TRACE_BUFFER_SIZE; 
    } else { 
        num_entries_to_dump = current_next_idx; 
    }

    size_t count_valid_entries = 0;
    for (size_t i = 0; i < num_entries_to_dump; ++i) {
        size_t idx_to_read = (start_idx_read + i) % TRACE_BUFFER_SIZE;
        const TraceEntry& entry = g_trace_buffer[idx_to_read];
        if (entry.event_str) { 
            std::snprintf(buffer, sizeof(buffer), "T:%llu C:%u EVT:%s A1:0x%lx A2:0x%lx\n",
                          static_cast<unsigned long long>(entry.timestamp_us),
                          entry.core_id,
                          entry.event_str,
                          static_cast<unsigned long>(entry.arg1),
                          static_cast<unsigned long>(entry.arg2));
            uart_ops->puts(buffer);
            count_valid_entries++;
        }
    }
    
    if (count_valid_entries == 0 && num_entries_to_dump > 0 && !has_overflowed_any_core && current_next_idx == 0) {
        uart_ops->puts("(empty)\n");
    } else if (count_valid_entries == 0 && num_entries_to_dump > 0) {
        uart_ops->puts("(No valid trace entries to display in the current view)\n");
    }


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
    if (!g_platform) { 
        volatile int i = 1; while(i) {} return; 
    }
    
    uint32_t core_id = g_platform->get_core_id();

    if (core_id == 0) { 
        g_platform->early_init_platform(); // Corrected call
    }
    #if defined(__aarch64__) || defined(__arm__)
    asm volatile("dmb sy" ::: "memory");
    #endif

    g_platform->early_init_core(core_id);   // Corrected call

    static Scheduler scheduler; 
    
    if (core_id == 0) {
        g_scheduler_ptr = &scheduler; 

        if (!g_software_timer_obj_pool.init(g_software_timer_obj_pool_mem, MAX_SOFTWARE_TIMERS,
                                           sizeof(kernel::hal::timer::SoftwareTimer), alignof(kernel::hal::timer::SoftwareTimer))) {
            g_platform->panic("Timer obj pool init failed", __FILE__, __LINE__);
        }
        if (!g_audio_pool.init(g_audio_pool_mem, 64, 1024, alignof(float))) { 
             g_platform->panic("Global audio pool init failed", __FILE__, __LINE__);
        }

        audio::AudioConfig audio_cfg{};
        audio_cfg.sample_rate_hz = 48000;
        audio_cfg.samples_per_block = 256;
        audio_cfg.num_channels = 2; 
        audio_cfg.num_i2s_dma_buffers = 4; // Corrected member name
        audio_cfg.num_audio_pool_buffers = 8; 
        audio_cfg.i2s_rx_instance_id = 0;
        audio_cfg.i2s_tx_instance_id = 1;
        
        if (!g_audio_system.init(audio_cfg) || !g_audio_system.start()) {
            g_platform->panic("Audio system init/start failed", __FILE__, __LINE__);
        }

        if (!g_file_system.create_file("/", true)) { 
            g_platform->panic("Root File system init failed", __FILE__, __LINE__);
        }
        
        if (g_platform->get_gpio_ops() && !g_gpio_manager.init(g_platform->get_gpio_ops())) {
            g_platform->panic("GPIO init failed", __FILE__, __LINE__);
        }

        if (g_platform->get_net_ops() && !g_net_manager.init(g_platform->get_net_ops())) {
            g_platform->panic("Network init failed", __FILE__, __LINE__);
        }

        auto* watchdog_ops = g_platform->get_watchdog_ops();
        if (watchdog_ops && !watchdog_ops->start_watchdog(5000)) { 
            g_platform->panic("Watchdog init failed", __FILE__, __LINE__);
        }

        if (!g_scheduler_ptr->create_thread(cli::CLI::cli_thread_entry, g_platform->get_uart_ops(), 3, 0, "CLI", false, 10000)) {
            g_platform->panic("CLI thread creation failed", __FILE__, __LINE__);
        }

        demo::register_demo_commands();
        test::register_tests();
        
        if (g_platform->get_uart_ops()) {
            g_platform->get_uart_ops()->puts("[miniOS v1.7] System Initialized on Core 0\n");
        }
        
        g_bss_cleared_smp_flag = true; 
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("sev" ::: "memory"); 
        asm volatile("dmb sy" ::: "memory"); 
        #endif

    } else { 
        while (!g_bss_cleared_smp_flag) { 
             #if defined(__aarch64__) || defined(__arm__)
             asm volatile("wfe" ::: "memory");
             #endif
        }
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("dmb sy" ::: "memory"); 
        #endif
        if (g_platform->get_uart_ops()) {
            char msg_buf[64];
            std::snprintf(msg_buf, sizeof(msg_buf), "[miniOS v1.7] Core %u Initialized\n", core_id);
            g_platform->get_uart_ops()->puts(msg_buf);
        }
    }

    if (!g_scheduler_ptr) { g_platform->panic("Scheduler not ready for core", __FILE__, __LINE__); }
    g_scheduler_ptr->start_core_scheduler(core_id);

    g_platform->panic("Scheduler returned unexpectedly", __FILE__, __LINE__);
}

extern "C" void hal_irq_handler(uint32_t /*irq_id_from_vector*/) { // Marked unused
    if (!g_platform || !g_platform->get_irq_ops()) {
        while(true) {}
        return;
    }
    uint32_t core_id = g_platform->get_core_id();
    auto* irq_ops = g_platform->get_irq_ops();

    uint32_t actual_irq_id = irq_ops->ack_irq(core_id); 

    ScopedISRLock lock(g_irq_handler_lock); 

    auto* timer_ops = g_platform->get_timer_ops();
    auto* watchdog_ops = g_platform->get_watchdog_ops(); 

    constexpr uint32_t GENERIC_TIMER_IRQ_S = 27; 
    constexpr uint32_t GENERIC_TIMER_IRQ_NS_PHYS = 29; 
    constexpr uint32_t GENERIC_TIMER_IRQ_VIRT = 30;  

    if (timer_ops && (actual_irq_id == GENERIC_TIMER_IRQ_S || actual_irq_id == GENERIC_TIMER_IRQ_NS_PHYS || actual_irq_id == GENERIC_TIMER_IRQ_VIRT)) {
        timer_ops->hardware_timer_irq_fired(core_id); 
        if (watchdog_ops) watchdog_ops->reset_watchdog();
        if (g_scheduler_ptr) g_scheduler_ptr->preemptive_tick(core_id); 
    } else if (actual_irq_id == 31 ) { 
        trace_event("NET:IRQ", actual_irq_id, core_id);
        if (kernel::g_net_manager.is_initialized()) {
            kernel::g_net_manager.tick(); // Or a specific handle_irq() method
        }
    } 
    else if (actual_irq_id < 1022 && actual_irq_id >= 16) { 
        trace_event("IRQ:Unhandled", actual_irq_id, core_id);
        if (g_platform->get_uart_ops()) {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "Unhandled IRQ: %u on Core %u\n", actual_irq_id, core_id);
            g_platform->get_uart_ops()->puts(buffer);
        }
    }

    irq_ops->end_irq(core_id, actual_irq_id); 
}

} // namespace kernel