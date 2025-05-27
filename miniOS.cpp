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

hal::Platform* g_platform = nullptr; 
Scheduler* g_scheduler_ptr = nullptr; 
volatile bool g_bss_cleared_smp_flag = false; // Flag for SMP core synchronization
alignas(64) uint8_t g_audio_pool_mem[64 * 1024]; 
FixedMemoryPool g_audio_pool; 
alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(kernel::hal::timer::SoftwareTimer)]; 
FixedMemoryPool g_software_timer_obj_pool; 
audio::AudioSystem g_audio_system; 
trace::TraceManager g_trace_manager; 
fs::FileSystem g_file_system; 
net::NetManager g_net_manager; 
gpio::GPIOManager g_gpio_manager; 
Spinlock g_irq_handler_lock; // Lock for the main IRQ handler
Spinlock g_trace_lock; // Lock for trace buffer dumping
Spinlock g_audio_system_lock; // Global lock for AudioSystem init/critical sections

// Define global trace buffer and related atomics (these were missing definitions)
std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::atomic<size_t> g_trace_buffer_next_idx{0};
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count{}; // Initialize to zeros


// Task stacks and TCBs
alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() : lock_id_(next_lock_id_++) {
    if (next_lock_id_ >= MAX_LOCKS && g_platform) { // MAX_LOCKS defined in miniOS.hpp
        g_platform->panic("Exceeded maximum number of spinlocks", __FILE__, __LINE__);
    }
}

void Spinlock::acquire_isr_safe() {
    if (!g_platform || !g_platform->get_irq_ops()) {
        // Cannot safely disable IRQs or panic, system is in a bad state
        // Loop forever as a last resort if IRQs can't be disabled
        while(true) {} 
        return;
    }
    g_platform->get_irq_ops()->disable_core_irqs(g_platform->get_core_id());
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        // In an ISR, we should not spin for long. If this happens often, it's a design issue.
        // A short pause or specific CPU instruction (like ARM's YIELD) could be used if absolutely necessary,
        // but generally, spinlocks held by ISRs should be very short-lived.
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
    // Basic spin with CPU pause, good for user-level locks or short critical sections.
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        // On ARM, WFE (Wait For Event) can be used to reduce power while spinning.
        // Requires SEV (Send Event) on release.
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("wfe" ::: "memory");
        #else
        // For other architectures, a simple CPU relax or yield might be appropriate.
        // Or just spin. std::this_thread::yield() if available and suitable.
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
    
    // Ensure alignment is at least sizeof(void*) for the Block header's next pointer.
    size_t actual_alignment = std::max(alignment_user_data, sizeof(Block*));
    if (actual_alignment == 0 || (actual_alignment & (actual_alignment - 1)) != 0) {
        // Alignment must be a power of 2. Default to sizeof(Block*) if invalid.
        actual_alignment = sizeof(Block*);
    }

    ScopedLock lock(pool_lock_); 
    // Calculate the size of the header, aligned.
    header_actual_size_ = (sizeof(Block) + actual_alignment - 1) & ~(actual_alignment - 1);
    
    // Calculate total storage size per block: aligned header + user data size, then align the whole block.
    // The user data itself will start at an aligned offset from the block start.
    // The start of user data will be: block_start_ptr + header_actual_size_
    // This address must be aligned to 'alignment_user_data'.
    // The block_start_ptr itself must be aligned to 'actual_alignment' for the Block struct.
    // The simplest is to ensure block_storage_size_ makes (block_start + header_actual_size_) aligned.
    // This is complex. A common approach is to align the start of the user data section within the block.
    // Total size for one block (header + padding for user data alignment + user data)
    block_storage_size_ = header_actual_size_ + block_size_user;
    // Align the entire block_storage_size_ up to ensure subsequent blocks start aligned.
    block_storage_size_ = (block_storage_size_ + actual_alignment - 1) & ~(actual_alignment - 1);


    // Check if the memory provided for the pool is itself aligned. The 'base' pointer.
    // If 'base' is not aligned to 'actual_alignment', the first block might be misaligned.
    // This init assumes 'base' is suitably aligned by the caller.

    pool_memory_start_ = static_cast<uint8_t*>(base);
    num_total_blocks_ = num_free_blocks_ = num_blocks;
    free_head_ = nullptr;

    uint8_t* current_block_ptr = pool_memory_start_;
    for (size_t i = 0; i < num_total_blocks_; ++i) {
        // Ensure current_block_ptr is aligned for Block struct
        void* aligned_block_base = reinterpret_cast<void*>((reinterpret_cast<uintptr_t>(current_block_ptr) + actual_alignment - 1) & ~(actual_alignment -1));
        
        // Calculate where user data would start if this block was used
        void* user_data_start_check = static_cast<uint8_t*>(aligned_block_base) + header_actual_size_;
        if((reinterpret_cast<uintptr_t>(user_data_start_check) % alignment_user_data) != 0) {
            // This indicates a flaw in block_storage_size calculation or initial base alignment.
            // For simplicity, this example assumes block_storage_size handles this by its own alignment.
            // A more robust way:
            // uintptr_t aligned_user_data_offset = (header_actual_size_ + alignment_user_data -1) & ~(alignment_user_data -1);
            // block_storage_size_ = aligned_user_data_offset + block_size_user;
            // block_storage_size_ = (block_storage_size_ + actual_alignment -1) & ~(actual_alignment-1);
            // And header_actual_size_ would be aligned_user_data_offset.
        }


        Block* block = reinterpret_cast<Block*>(aligned_block_base);
        block->next = free_head_;
        free_head_ = block;
        current_block_ptr += block_storage_size_; // Move to the start of the next potential block
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
    // User data starts after the Block header
    return static_cast<uint8_t*>(static_cast<void*>(block_header)) + header_actual_size_;
}

void FixedMemoryPool::free_block(void* user_data_ptr) {
    if (!user_data_ptr) return;
    // User data pointer is past the header. Calculate header start.
    Block* block_header = reinterpret_cast<Block*>(static_cast<uint8_t*>(user_data_ptr) - header_actual_size_);
    
    ScopedLock lock(pool_lock_);
    // Basic check: ensure the block is within the pool bounds (optional for performance)
    // uintptr_t block_addr = reinterpret_cast<uintptr_t>(block_header);
    // uintptr_t pool_start_addr = reinterpret_cast<uintptr_t>(pool_memory_start_);
    // if (block_addr < pool_start_addr || block_addr >= pool_start_addr + num_total_blocks_ * block_storage_size_) {
    //     if (g_platform) g_platform->panic("Freeing block outside pool bounds", __FILE__, __LINE__);
    //     return;
    // }
    // TODO: Could add check for double free by iterating free_head_, but costly.

    block_header->next = free_head_;
    free_head_ = block_header;
    num_free_blocks_++;
}

template<typename T, size_t Capacity>
bool SPSCQueue<T, Capacity>::enqueue(T* item) noexcept {
    size_t current_tail = tail_.load(std::memory_order_acquire);
    size_t next_tail = (current_tail + 1) & (Capacity - 1); // Capacity is a power of 2
    if (next_tail == head_.load(std::memory_order_acquire)) {
        return false; // Queue full
    }
    items_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
}

template<typename T, size_t Capacity>
T* SPSCQueue<T, Capacity>::dequeue() noexcept {
    size_t current_head = head_.load(std::memory_order_acquire);
    if (current_head == tail_.load(std::memory_order_acquire)) {
        return nullptr; // Queue empty
    }
    T* item = items_[current_head];
    head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
    return item;
}

template class SPSCQueue<audio::AudioBuffer, 16>; 

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* /*current_task*/) { // Marked current_task as unused
    if (core_id >= MAX_CORES || !g_scheduler_ptr) {
        if(g_platform) g_platform->panic("EDFPolicy: Invalid args or scheduler missing", __FILE__, __LINE__);
        return nullptr;
    }

    TCB* earliest_task = nullptr;
    uint64_t earliest_deadline = UINT64_MAX;
    TCB* prev_in_list = nullptr;
    TCB* chosen_prev_in_list = nullptr;
    int chosen_priority = -1;

    // This lock needs to be taken carefully. If g_scheduler_ptr->per_core_locks_ is an array of Spinlock, this is fine.
    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);

    // Iterate through all priority levels for the current core
    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        TCB* task_iter = g_scheduler_ptr->ready_qs_[core_id][p];
        TCB* current_prev = nullptr; 
        while (task_iter) {
            if (task_iter->state == TCB::State::READY && // Ensure task is actually ready
                task_iter->deadline_us > 0 && task_iter->deadline_us < earliest_deadline &&
                (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(core_id))) {
                
                earliest_deadline = task_iter->deadline_us;
                earliest_task = task_iter;
                chosen_prev_in_list = current_prev; // Record previous task in this list
                chosen_priority = p; // Record priority queue of chosen task
            }
            current_prev = task_iter;
            task_iter = task_iter->next_in_q;
        }
    }

    // If an EDF task was found, extract it from its ready queue
    if (earliest_task) {
        if (chosen_prev_in_list) {
            chosen_prev_in_list->next_in_q = earliest_task->next_in_q;
        } else {
            g_scheduler_ptr->ready_qs_[core_id][chosen_priority] = earliest_task->next_in_q;
        }
        earliest_task->next_in_q = nullptr; // Detach
        trace_event("SCHED:PopEDF", reinterpret_cast<uintptr_t>(earliest_task), core_id);
        return earliest_task;
    }

    // If no deadline-based task found, fall back to priority scheduler's method
    return g_scheduler_ptr->pop_highest_priority_ready_task(core_id); // This method needs to be careful about locking
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
    // Policy must be created first
    policy_ = new (std::nothrow) EDFPolicy(); // Use nothrow version
    if (!policy_) {
        if (g_platform) g_platform->panic("Failed to allocate scheduler policy", __FILE__, __LINE__);
        else { while(1); } // Cannot proceed
    }

    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        char idle_name[MAX_NAME_LENGTH];
        // Using snprintf is safer
        std::snprintf(idle_name, MAX_NAME_LENGTH, "IdleCore%u", i);
        // Pass core_id as argument to idle_thread_func
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
    delete policy_; // Clean up allocated policy
}


TCB* Scheduler::create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle, uint64_t deadline_us) {
    if (!fn || !name) return nullptr;
    ScopedLock lock(scheduler_global_lock_); // Protect task_tcbs, task_stacks, num_active_tasks_
    
    if (num_active_tasks_.load(std::memory_order_relaxed) >= MAX_THREADS) return nullptr;

    for (size_t i = 0; i < MAX_THREADS; ++i) {
        if (g_task_tcbs[i].state == TCB::State::INACTIVE) {
            TCB& tcb = g_task_tcbs[i];
            
            // Clear relevant parts of TCB before reuse
            tcb = {}; // Zero-initialize if TCB is simple enough, or memset, or member-wise
            
            tcb.entry_point = fn;
            tcb.arg_ptr = const_cast<void*>(arg); 
            tcb.priority = (prio >= 0 && static_cast<size_t>(prio) < MAX_PRIORITY_LEVELS) ? prio : 0;
            tcb.core_affinity = (affinity >= -1 && affinity < static_cast<int>(MAX_CORES)) ? affinity : -1;
            
            if (!kernel::util::safe_strcpy(tcb.name, name, MAX_NAME_LENGTH)) {
                 // Name too long, or other strcpy error.
                 // Should ideally log or handle this. For now, might proceed with truncated name.
                 // Or return nullptr if strict.
                return nullptr;
            }
            
            tcb.stack_base = g_task_stacks[i].data();
            tcb.stack_size = DEFAULT_STACK_SIZE;
            
            // Initialize stack pointer to the top of the stack (stack grows downwards)
            // Ensure stack pointer is aligned (e.g., 16-byte for AArch64 AAPCS)
            tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size);
            tcb.sp &= ~0xFUL; // Align to 16 bytes (adjust if different ABI)

            tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap); 
            tcb.pstate = 0; // Initialize pstate (e.g., default mode, IRQs enabled for user threads)
            
            tcb.deadline_us = deadline_us;
            tcb.event_flag.store(false, std::memory_order_relaxed);

            // Set to READY and add to queue if not idle. Idle threads are handled specially.
            if (!is_idle) {
                tcb.state = TCB::State::READY;
                g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name); 
                // Determine target core for initial queueing
                uint32_t target_core = (tcb.core_affinity != -1 && static_cast<size_t>(tcb.core_affinity) < MAX_CORES) 
                                     ? static_cast<uint32_t>(tcb.core_affinity) 
                                     : (g_platform ? g_platform->get_core_id() : 0); // Or round-robin if no affinity
                policy_->add_to_ready_queue(&tcb, target_core);
                num_active_tasks_.fetch_add(1, std::memory_order_relaxed);
            } else {
                tcb.state = TCB::State::READY; // Idle threads start as ready but aren't in general queues initially
            }
            return &tcb;
        }
    }
    return nullptr; // No free TCB slots
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!g_platform || core_id >= MAX_CORES || !g_platform->get_irq_ops() || !g_platform->get_timer_ops()) {
         // Cannot panic safely if platform isn't fully up for this core
        while(true) {}
        return;
    }
    g_platform->get_irq_ops()->init_cpu_interface(core_id); // Init GIC CPU interface for this core
    g_platform->get_irq_ops()->enable_core_irqs(core_id, 0); 
    g_platform->get_timer_ops()->init_core_timer_interrupt(core_id); 

    auto* idle_tcb = g_per_cpu_data[core_id].idle_thread;
    if (!idle_tcb) { 
        g_platform->panic("No idle thread for core", __FILE__, __LINE__); 
        return; 
    }
    g_per_cpu_data[core_id].current_thread = idle_tcb;
    idle_tcb->state = TCB::State::RUNNING;
    idle_tcb->cpu_id_running_on = core_id;
    
    extern void context_switch_to(TCB* next_tcb); // Assumes a function that only loads next, no save
    context_switch_to(idle_tcb); 
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    // Acknowledge timer interrupt first
    if (g_platform && g_platform->get_timer_ops()) {
        g_platform->get_timer_ops()->ack_core_timer_interrupt(core_id);
    }
    schedule(core_id, true); // is_preemption = true
}

void Scheduler::yield(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    // A yield is a voluntary reschedule request.
    // The current thread's state will be set to READY by schedule().
    if(g_per_cpu_data[core_id].current_thread) { // Check if current_thread is valid
         g_trace_manager.record_event(g_per_cpu_data[core_id].current_thread, trace::EventType::THREAD_YIELD, "yield");
    }
    schedule(core_id, false); // is_preemption = false
}

void Scheduler::signal_event_isr(TCB* tcb) {
    if (!tcb) return;
    tcb->event_flag.store(true, std::memory_order_release);
    // Optional: If the task was BLOCKED on this event, move it to READY queue.
    // This requires knowing its state and potentially taking scheduler locks.
    // For simplicity, wait_for_event polls and yields.
    // A more advanced version would:
    // ScopedISRLock lock(scheduler_global_lock_ or per_core_lock for tcb's affinity);
    // if (tcb->state == TCB::State::BLOCKED_ON_EVENT) {
    //    tcb->state = TCB::State::READY;
    //    policy_->add_to_ready_queue(tcb, tcb->core_affinity_or_target_core);
    //    // Potentially trigger IPI if task is on another core to force reschedule
    // }
}

void Scheduler::wait_for_event(TCB* tcb) {
    if (!tcb || !g_platform) return;
    // Interrupts should be enabled for yield to work.
    // This is a spin-yield loop.
    while (!tcb->event_flag.load(std::memory_order_acquire)) { 
        yield(g_platform->get_core_id()); 
    }
    tcb->event_flag.store(false, std::memory_order_relaxed); // Consume the event
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;

    // Try local queue first - Lock is already held by schedule() or EDFPolicy
    // ScopedLock lock(per_core_locks_[current_core_id]); // This lock is problematic if called from EDFPolicy which already holds it.
    // Assume caller (schedule or policy) holds the necessary lock for current_core_id's queue.
    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        if (ready_qs_[current_core_id][p]) {
            TCB* task = ready_qs_[current_core_id][p];
            ready_qs_[current_core_id][p] = task->next_in_q;
            task->next_in_q = nullptr;
            trace_event("SCHED:PopSelf", reinterpret_cast<uintptr_t>(task), (current_core_id << 16) | p);
            return task;
        }
    }

    // Try work stealing (only if not using an EDF policy that always picks local)
    // This part is complex with locking. Simplified: try_lock or be careful.
    // For now, EDFPolicy handles its own selection; this is fallback if EDF found nothing.
    // If pop_highest_priority_ready_task is only called as a fallback by EDFPolicy,
    // then EDFPolicy has already determined no local deadline task.
    // This function would then represent the priority-based part of a hybrid scheduler.

    // Return idle thread if nothing else is found.
    trace_event("SCHED:PopIdleAff", current_core_id);
    return g_per_cpu_data[current_core_id].idle_thread;
}

void Scheduler::schedule(uint32_t core_id, bool is_preemption) {
    if (core_id >= MAX_CORES || !policy_ || !g_platform) {
        if(g_platform) g_platform->panic("Schedule: Invalid args", __FILE__, __LINE__);
        return;
    }

    ScopedLock core_lock(per_core_locks_[core_id]); // Lock this core's scheduling data

    TCB* current = g_per_cpu_data[core_id].current_thread;

    if (current && current->state == TCB::State::RUNNING) { 
        // Don't mark idle thread as READY unless it's being preempted by a higher priority task.
        // If current is idle and no other task is ready, it will be selected again.
        if (current != g_per_cpu_data[core_id].idle_thread || is_preemption) {
            current->state = TCB::State::READY;
        }
    }
    
    TCB* next = policy_->select_next_task(core_id, current); // Policy selects, may use pop_highest_priority_ready_task

    if (!next) { 
        if (g_platform) g_platform->panic("Scheduler selected null task", __FILE__, __LINE__);
        return; 
    }

    if (next == current) { // Same task selected
        if (current) current->state = TCB::State::RUNNING; // Ensure it's marked running
        return; // No context switch needed
    }
    
    // A new task is being scheduled
    next->state = TCB::State::RUNNING;
    next->cpu_id_running_on = core_id;
    g_per_cpu_data[core_id].current_thread = next;

    if (current && current != g_per_cpu_data[core_id].idle_thread && current->state == TCB::State::READY) {
        // Add the old (non-idle) task back to the ready queue if it's still ready
        // The policy's select_next_task might have already removed it from a queue.
        // If select_next_task doesn't re-queue 'current', then we must do it here.
        // This depends on policy implementation. For simplicity, assume policy handles it or it's added here.
        // If current was preempted and is still READY, it needs to go back.
        policy_->add_to_ready_queue(current, core_id); 
    }
    
    g_trace_manager.record_event(next, trace::EventType::THREAD_SCHEDULE, next->name);
    
    extern void context_switch(TCB* current_tcb, TCB* next_tcb); // Assumes TCB* args
    context_switch(current, next);
}

void Scheduler::idle_thread_func(void* arg) {
    (void)arg; 
    while (true) {
        set_power_mode(true); 
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("wfi" ::: "memory");
        #else
        // For other arch, busy loop or platform specific idle
        for(volatile int i=0; i<10000; ++i); // Placeholder
        #endif
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self || !self->entry_point || !g_scheduler_ptr || !g_platform) {
        if (g_platform) g_platform->panic("Thread bootstrap failure", __FILE__, __LINE__);
        else { while(1); } 
        return; 
    }

    // Per-thread setup (e.g., enabling interrupts if scheduler disabled them for context switch)
    // This depends on the context_switch implementation.
    // For ARM, if interrupts were globally disabled by scheduler, they'd be re-enabled by eret from exception context.
    // If threads run in user mode, supervisor call might switch context.

    configure_memory_protection(self, true); 

    self->entry_point(self->arg_ptr); 

    // Thread function has returned: clean up.
    // This section runs in the context of the exiting thread.
    // It MUST call yield or schedule and not return, as its stack will be reclaimed.

    // Acquire lock to modify shared scheduler state (num_active_tasks, TCB state)
    // Which lock? Global scheduler lock or per-core lock of the core it ran on?
    // Let's use global lock for num_active_tasks and TCB state change to INACTIVE/ZOMBIE.
    {
        ScopedLock lock(g_scheduler_ptr->scheduler_global_lock_);
        g_trace_manager.record_event(self, trace::EventType::THREAD_EXIT, self->name);
        self->state = TCB::State::ZOMBIE; // Or INACTIVE if no separate ZOMBIE state for reclamation
        g_scheduler_ptr->num_active_tasks_.fetch_sub(1, std::memory_order_relaxed);
        // The TCB and stack can be marked for reclamation by a reaper task or when create_thread looks for INACTIVE.
    }
    
    // Trigger a reschedule. This thread should not run again.
    // The core_id should be the one this thread just ran on.
    g_scheduler_ptr->schedule(self->cpu_id_running_on, false); // Not a preemption tick

    // Should NOT be reached if schedule() context switches correctly.
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
        // Example: Set to full performance frequency (0 might mean default/max)
        // power_ops->set_cpu_frequency(g_platform->get_core_id(), 0);
    }
}

void configure_memory_protection(TCB* tcb, bool enable_for_task) {
    if (!g_platform || !tcb) return;
    trace_event("MEM:Protection", reinterpret_cast<uintptr_t>(tcb), enable_for_task);
    // Actual MPU/MMU configuration would go here, via g_platform->get_mem_ops() or similar
}

void get_kernel_stats(hal::UARTDriverOps* uart_ops) { 
    if (!uart_ops || !g_scheduler_ptr || !g_platform) return;
    
    ScopedLock lock(g_scheduler_ptr->get_global_scheduler_lock()); 
    
    char buffer[128]; 

    uart_ops->puts("Kernel Statistics:\n");
    std::snprintf(buffer, sizeof(buffer), "  Active Threads: %zu\n", g_scheduler_ptr->get_num_active_tasks());
    uart_ops->puts(buffer);

    for (uint32_t core = 0; core < MAX_CORES; ++core) { // Use MAX_CORES
        // Check if per_cpu_data for this core is valid, especially current_thread
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
    if (core_id >= MAX_CORES) { // Safety, should be configured by platform
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
        start_idx_read = current_next_idx % TRACE_BUFFER_SIZE; // Start from the oldest overwritten entry
    } else { 
        num_entries_to_dump = current_next_idx; // Dump only valid entries if no wrap around
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
    // g_platform must be initialized by the platform-specific entry point BEFORE kernel_main is called.
    if (!g_platform) { 
        // Minimal panic if UART isn't even up. Maybe blink LED or halt.
        // This is a catastrophic failure.
        volatile int i = 1; while(i) {} return; 
    }
    
    uint32_t core_id = g_platform->get_core_id();

    if (core_id == 0) { 
        g_platform->early_init_platform(); 
    }
    // Ensure memory barrier if needed for g_bss_cleared_smp_flag visibility
    #if defined(__aarch64__) || defined(__arm__)
    asm volatile("dmb sy" ::: "memory");
    #endif

    g_platform->early_init_core(core_id);   

    static Scheduler scheduler; 
    
    if (core_id == 0) {
        g_scheduler_ptr = &scheduler; 

        if (!g_software_timer_obj_pool.init(g_software_timer_obj_pool_mem, MAX_SOFTWARE_TIMERS,
                                           sizeof(kernel::hal::timer::SoftwareTimer), alignof(kernel::hal::timer::SoftwareTimer))) {
            g_platform->panic("Timer obj pool init failed", __FILE__, __LINE__);
        }
        // Initialize audio pool (g_audio_pool) if it's separate from AudioSystem's internal one
        if (!g_audio_pool.init(g_audio_pool_mem, 64, 1024, alignof(float))) { // Example: 64 buffers of 1KB
             g_platform->panic("Global audio pool init failed", __FILE__, __LINE__);
        }


        audio::AudioConfig audio_cfg{}; // Use default constructor then set members
        audio_cfg.sample_rate_hz = 48000;
        audio_cfg.samples_per_block = 256;
        audio_cfg.num_i2s_dma_buffers = 4; // Corrected member name
        audio_cfg.num_audio_pool_buffers = 8; // Number of AudioBuffer objects AudioSystem will pool
        
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
        asm volatile("dmb sy" ::: "memory"); // Ensure flag change is visible
        #endif

    } else { 
        while (!g_bss_cleared_smp_flag) { 
             #if defined(__aarch64__) || defined(__arm__)
             asm volatile("wfe" ::: "memory");
             #else
             // Busy wait or yield for other archs
             #endif
        }
        #if defined(__aarch64__) || defined(__arm__)
        asm volatile("dmb sy" ::: "memory"); // Ensure we see the updated g_scheduler_ptr
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

extern "C" void hal_irq_handler(uint32_t irq_id_from_vector) {
    // This is the top-level C IRQ handler. It should be brief and efficient.
    // It needs to figure out the actual source (if irq_id_from_vector is just an index)
    // and call the appropriate driver's ISR or a generic handler.

    // It's crucial that this function can run without g_platform fully up if very early IRQs happen.
    // However, most useful IRQs (timers, peripherals) will occur after g_platform is set.
    if (!g_platform || !g_platform->get_irq_ops()) {
        // Cannot do much, maybe a specific panic or loop.
        // This indicates a severe boot sequence or platform driver issue.
        while(true) {}
        return;
    }
    uint32_t core_id = g_platform->get_core_id();
    auto* irq_ops = g_platform->get_irq_ops();

    // Acknowledge the IRQ at the controller level to get the actual IRQ ID.
    // The irq_id_from_vector might just be a generic "IRQ" or "FIQ" signal.
    // The specific IRQ ID is usually read from an GIC_IAR register or similar.
    uint32_t actual_irq_id = irq_ops->ack_irq(core_id); // This should return the true source ID

    // Now handle based on actual_irq_id
    ScopedISRLock lock(g_irq_handler_lock); 

    auto* timer_ops = g_platform->get_timer_ops();
    auto* watchdog_ops = g_platform->get_watchdog_ops(); 

    // Example IRQ IDs (these are platform-specific and need to match hardware)
    // Generic Timer IRQ for ARM is often PPI 27 (secure), 29 (non-secure physical), 30 (virtual)
    // Check your platform's GIC mapping.
    constexpr uint32_t GENERIC_TIMER_IRQ_S = 27; // Example for Secure Physical Timer
    constexpr uint32_t GENERIC_TIMER_IRQ_NS_PHYS = 29; // Example for Non-Secure Physical Timer
    constexpr uint32_t GENERIC_TIMER_IRQ_VIRT = 30;  // Example for Virtual Timer

    if (timer_ops && (actual_irq_id == GENERIC_TIMER_IRQ_S || actual_irq_id == GENERIC_TIMER_IRQ_NS_PHYS || actual_irq_id == GENERIC_TIMER_IRQ_VIRT)) {
        // hardware_timer_irq_fired should ideally call ack_core_timer_interrupt itself or timer_ops handles ack.
        timer_ops->hardware_timer_irq_fired(core_id); 
        if (watchdog_ops) watchdog_ops->reset_watchdog();
        if (g_scheduler_ptr) g_scheduler_ptr->preemptive_tick(core_id); 
    } else if (actual_irq_id == 31 ) { // Example specific network IRQ ID (platform specific)
        trace_event("NET:IRQ", actual_irq_id, core_id);
        if (kernel::g_net_manager.is_initialized()) {
            // kernel::g_net_manager.tick(); // Or a specific handle_irq() method
        }
    } // Add more else if for other peripheral IRQs (UART, I2S, DMA etc.)
    else if (actual_irq_id < 1022 && actual_irq_id >= 16) { // Range for SPIs or unhandled PPIs
        trace_event("IRQ:Unhandled", actual_irq_id, core_id);
        if (g_platform->get_uart_ops()) {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "Unhandled IRQ: %u on Core %u\n", actual_irq_id, core_id);
            g_platform->get_uart_ops()->puts(buffer);
        }
    }
    // Else (ID 1022/1023 spurious) - GIC handles spurious IRQs by returning these IDs.

    irq_ops->end_irq(core_id, actual_irq_id); // Signal End Of Interrupt to controller
}

} // namespace kernel