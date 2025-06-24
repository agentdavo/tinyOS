// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file core.cpp
 * @brief Core kernel implementation for miniOS v1.7.
 */

#include "core.hpp"
#include "hal.hpp"    
#include "util.hpp"
#include "trace.hpp"    
#include "miniOS.hpp"   

#include <cstring>      
#include <algorithm>    
#include <limits>       
#include <atomic> 
#include <cstddef>

extern "C" void early_uart_puts(const char* str);
extern "C" kernel::core::PerCPUData* const kernel_g_per_cpu_data = kernel::core::g_per_cpu_data.data();

namespace kernel {
namespace core {

alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(kernel::hal::timer::SoftwareTimer)];
FixedMemoryPool g_software_timer_obj_pool;
std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count{}; 
alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;
uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() : lock_id_(next_lock_id_++) {}
void Spinlock::acquire_isr_safe() {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {}
    }
}
void Spinlock::release_isr_safe() { lock_flag_.store(false, std::memory_order_release); }
void Spinlock::acquire_general() {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {
            if (kernel::g_platform && kernel::g_platform->get_num_cores() > 1) {
                kernel::hal::sync::barrier_dmb(); 
            }
        }
    }
}
void Spinlock::release_general() { lock_flag_.store(false, std::memory_order_release); }

bool FixedMemoryPool::init(void* base, size_t num_blocks, size_t blk_sz_user, size_t align_user_data) {
    if (!base || num_blocks == 0 || blk_sz_user == 0) return false;
    size_t internal_header_align = sizeof(Block*);
    size_t final_user_data_align = (align_user_data == 0 || (align_user_data & (align_user_data - 1)) != 0) 
                                   ? internal_header_align : align_user_data;
    header_actual_size_ = (sizeof(Block) + internal_header_align - 1) & ~(internal_header_align - 1);
    user_data_offset_ = header_actual_size_;
    if ((user_data_offset_ % final_user_data_align) != 0) {
        user_data_offset_ = (user_data_offset_ + final_user_data_align - 1) & ~(final_user_data_align - 1);
    }
    block_storage_size_ = user_data_offset_ + blk_sz_user;
    pool_memory_start_ = static_cast<uint8_t*>(base);
    num_total_blocks_ = num_free_blocks_ = num_blocks;
    free_head_ = nullptr;
    for (size_t i = 0; i < num_blocks; ++i) {
        uint8_t* current_block_raw_ptr = pool_memory_start_ + i * block_storage_size_;
        Block* block_header = reinterpret_cast<Block*>(current_block_raw_ptr);
        block_header->next = free_head_;
        free_head_ = block_header;
    }
    return true;
}
void* FixedMemoryPool::allocate() {
    ScopedLock lock(pool_lock_);
    if (!free_head_) return nullptr;
    Block* block_header_raw = free_head_;
    free_head_ = block_header_raw->next;
    num_free_blocks_--;
    return static_cast<uint8_t*>(static_cast<void*>(block_header_raw)) + user_data_offset_;
}
void FixedMemoryPool::free_block(void* user_data_ptr) {
    if (!user_data_ptr) return;
    Block* block_header_raw = reinterpret_cast<Block*>(static_cast<uint8_t*>(user_data_ptr) - user_data_offset_);
    ScopedLock lock(pool_lock_);
    block_header_raw->next = free_head_;
    free_head_ = block_header_raw;
    num_free_blocks_++;
}

template<typename T, size_t Capacity>
bool SPSCQueue<T, Capacity>::enqueue(T* item) noexcept {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    size_t next_tail = (current_tail + 1) & (Capacity - 1);
    if (next_tail == head_.load(std::memory_order_acquire)) return false;
    items_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
}
template<typename T, size_t Capacity>
T* SPSCQueue<T, Capacity>::dequeue() noexcept {
    size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire)) return nullptr;
    T* item = items_[current_head];
    head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
    return item;
}

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB*) {
    if (core_id >= MAX_CORES || !kernel::g_scheduler_ptr) return nullptr;
    Scheduler* sched = kernel::g_scheduler_ptr;
    ScopedLock lock(sched->per_core_locks_[core_id]);
    uint64_t earliest_deadline = std::numeric_limits<uint64_t>::max();
    TCB* earliest_task = nullptr; int chosen_priority_idx = -1; TCB* chosen_prev_in_list = nullptr;
    for (int p_idx = MAX_PRIORITY_LEVELS - 1; p_idx >= 0; --p_idx) {
        TCB* task_iter = sched->ready_qs_[core_id][p_idx]; TCB* prev_in_list = nullptr;
        while (task_iter) {
            if (task_iter->state == TCB::State::READY &&
                (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(core_id))) {
                if (task_iter->deadline_us > 0 && task_iter->deadline_us < earliest_deadline) {
                    earliest_deadline = task_iter->deadline_us; earliest_task = task_iter;
                    chosen_priority_idx = p_idx; chosen_prev_in_list = prev_in_list;
                }
            }
            prev_in_list = task_iter; task_iter = task_iter->next_in_q;
        }
    }
    if (earliest_task) {
        if (chosen_prev_in_list) chosen_prev_in_list->next_in_q = earliest_task->next_in_q;
        else sched->ready_qs_[core_id][chosen_priority_idx] = earliest_task->next_in_q;
        earliest_task->next_in_q = nullptr; return earliest_task;
    }
    return sched->pop_highest_priority_ready_task(core_id);
}

void EDFPolicy::add_to_ready_queue(TCB* tcb, uint32_t core_id) {
    if (!tcb || tcb->priority < 0 || static_cast<size_t>(tcb->priority) >= MAX_PRIORITY_LEVELS || core_id >= MAX_CORES || !kernel::g_scheduler_ptr) return;
    Scheduler* sched = kernel::g_scheduler_ptr;
    ScopedLock lock(sched->per_core_locks_[core_id]);
    kernel::trace_event("SCHED:AddRdyEDF", reinterpret_cast<uintptr_t>(tcb), (core_id << 16) | tcb->priority);
    tcb->next_in_q = sched->ready_qs_[core_id][tcb->priority];
    sched->ready_qs_[core_id][tcb->priority] = tcb;
    tcb->state = TCB::State::READY;
}

Scheduler::Scheduler() : policy_(nullptr) {
    for (size_t c = 0; c < MAX_CORES; ++c) {
        for (size_t p = 0; p < MAX_PRIORITY_LEVELS; ++p) { ready_qs_[c][p] = nullptr; }
    }
}
Scheduler::~Scheduler() { delete policy_; }

TCB* Scheduler::create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle, uint64_t deadline_us) {
    ScopedLock lock(scheduler_global_lock_);
    if (num_active_tasks_.load(std::memory_order_relaxed) >= MAX_THREADS) return nullptr;
    size_t tcb_idx = MAX_THREADS;
    for (size_t i = 0; i < MAX_THREADS; ++i) {
        if (g_task_tcbs[i].state == TCB::State::INACTIVE || g_task_tcbs[i].state == TCB::State::ZOMBIE) {
            tcb_idx = i; break;
        }
    }
    if (tcb_idx == MAX_THREADS) return nullptr;
    TCB& tcb = g_task_tcbs[tcb_idx];
    kernel::util::kmemset(&tcb, 0, sizeof(TCB));
    tcb.entry_point = fn; tcb.arg_ptr = const_cast<void*>(arg);
    tcb.priority = (prio >= 0 && static_cast<size_t>(prio) < MAX_PRIORITY_LEVELS) ? prio : 0;
    tcb.core_affinity = (affinity >= -1 && affinity < static_cast<int>(MAX_CORES)) ? affinity : -1;
    kernel::util::safe_strcpy(tcb.name, name, MAX_NAME_LENGTH);
    tcb.stack_base = g_task_stacks[tcb_idx].data(); tcb.stack_size = DEFAULT_STACK_SIZE;
    tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size) & ~0xFUL; 
    tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap); 
    // SPSR_EL1 for EL1h with IRQs enabled: M[4]=0 (AArch64), M[3:0]=0101 (EL1h), DAIF all unmasked (bits 9-6 are 0)
    tcb.pstate = 0x00000005; 
    tcb.deadline_us = deadline_us; tcb.state = TCB::State::READY;
    tcb.cpu_id_running_on = static_cast<uint32_t>(-1); tcb.event_flag.store(false, std::memory_order_relaxed);
    tcb.regs[0] = reinterpret_cast<uint64_t>(&tcb); // thread_bootstrap argument
    trace::g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name);
    uint32_t target_core = (tcb.core_affinity != -1) ? static_cast<uint32_t>(tcb.core_affinity) : 0;
    if (is_idle) {
        g_per_cpu_data[target_core].idle_thread = &tcb;
        tcb.pstate = 0x00000005; // Ensure idle thread can take interrupts
    }
    if (policy_) policy_->add_to_ready_queue(&tcb, target_core);
    else { if (kernel::g_platform) kernel::g_platform->panic("Scheduler policy not set", __FILE__, __LINE__); return nullptr; }
    num_active_tasks_.fetch_add(1, std::memory_order_relaxed);
    return &tcb;
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!kernel::g_platform || core_id >= MAX_CORES || !kernel::g_platform->get_irq_ops() || !kernel::g_platform->get_timer_ops()) {
        if (kernel::g_platform) kernel::g_platform->panic("Invalid core_id or platform components missing", __FILE__, __LINE__); else for(;;); return;
    }
    auto* idle_tcb = g_per_cpu_data[core_id].idle_thread;
    if (!idle_tcb) { kernel::g_platform->panic("Idle thread not created for core", __FILE__, __LINE__); return; }
    g_per_cpu_data[core_id].current_thread = idle_tcb; 
    idle_tcb->state = TCB::State::RUNNING; 
    idle_tcb->cpu_id_running_on = core_id;
    kernel::g_platform->get_irq_ops()->enable_irq_line(kernel::hal::SYSTEM_TIMER_IRQ);
    kernel::g_platform->get_irq_ops()->enable_core_irqs(core_id, 0x1); 
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    kernel::trace_event("SCHED:PreeTick", core_id, reinterpret_cast<uintptr_t>(g_per_cpu_data[core_id].current_thread));
    schedule(core_id, true);
}

void Scheduler::yield(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    TCB* current_task = g_per_cpu_data[core_id].current_thread;
    if (current_task) trace::g_trace_manager.record_event(current_task, trace::EventType::THREAD_YIELD, current_task->name);
    schedule(core_id, false);
}

void Scheduler::signal_event_isr(TCB* tcb) { if (!tcb) return; tcb->event_flag.exchange(true, std::memory_order_release); }
void Scheduler::wait_for_event(TCB* tcb) {
    if (!tcb) return;
    while (!tcb->event_flag.load(std::memory_order_acquire)) kernel::hal::sync::barrier_dmb();
    tcb->event_flag.store(false, std::memory_order_relaxed);
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;
    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        TCB* task_iter = ready_qs_[current_core_id][p]; TCB* prev_task = nullptr;
        while(task_iter) {
            if (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(current_core_id)) {
                if (prev_task) prev_task->next_in_q = task_iter->next_in_q;
                else ready_qs_[current_core_id][p] = task_iter->next_in_q;
                task_iter->next_in_q = nullptr; return task_iter;
            }
            prev_task = task_iter; task_iter = task_iter->next_in_q;
        }
    }
    TCB* idle = g_per_cpu_data[current_core_id].idle_thread;
    if (!idle) { if(kernel::g_platform) kernel::g_platform->panic("Idle thread null in pop", __FILE__, __LINE__); return nullptr; }
    return idle;
}

void Scheduler::schedule(uint32_t core_id, bool is_preemption) {
    if (core_id >= MAX_CORES || !policy_ || !kernel::g_platform) return;
    uint64_t daif_flags; 
    asm volatile("mrs %0, daif; msr daifset, #0xf" : "=r"(daif_flags) :: "memory"); 
    ScopedLock core_lock(per_core_locks_[core_id]);
    TCB* current_task = g_per_cpu_data[core_id].current_thread; TCB* next_task = nullptr;
    if (current_task && current_task->state == TCB::State::RUNNING) {
        if (current_task != g_per_cpu_data[core_id].idle_thread || is_preemption) {
            current_task->state = TCB::State::READY;
            // PC/PSTATE for current_task should be correctly set by IRQ handler or yield mechanism
            // before calling schedule.
            policy_->add_to_ready_queue(current_task, core_id);
        }
    }
    next_task = policy_->select_next_task(core_id, current_task);
    if (!next_task) {
        next_task = g_per_cpu_data[core_id].idle_thread;
        if (!next_task && kernel::g_platform) { 
            kernel::g_platform->panic("Idle thread null in schedule", __FILE__, __LINE__); 
            asm volatile("msr daif, %0" :: "r"(daif_flags) : "memory"); 
            return; 
        }
    }
    if (current_task != next_task) {
        g_per_cpu_data[core_id].current_thread = next_task; 
        next_task->state = TCB::State::RUNNING; 
        next_task->cpu_id_running_on = core_id;
        trace::g_trace_manager.record_event(next_task, trace::EventType::THREAD_SCHEDULE, next_task->name);
        // cpu_context_switch does not return for the old task.
        // Interrupts (DAIF) will be restored by the 'eret' in cpu_context_switch_impl, using new_task->pstate.
        kernel::hal::cpu_context_switch(current_task, next_task);
    } else {
        if(current_task) { current_task->state = TCB::State::RUNNING; current_task->cpu_id_running_on = core_id; }
        asm volatile("msr daif, %0" :: "r"(daif_flags) : "memory"); // Restore DAIF if no switch
    }
}

void Scheduler::idle_thread_func(void* arg) {
    uint32_t core_id = reinterpret_cast<uintptr_t>(arg);
    if (!kernel::g_platform || !kernel::g_platform->get_power_ops()) { for (;;) { asm volatile("nop"); } }
    // Ensure interrupts are enabled for the idle task. This is primarily controlled
    // by the TCB.pstate (SPSR_EL1 value) used when switching to this task.
    asm volatile("msr daifclr, #2"); // Explicitly enable IRQs (clear PSTATE.I bit)
    while (true) { 
        kernel::g_platform->get_power_ops()->enter_idle_state(core_id); 
        kernel::hal::sync::barrier_dmb(); 
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self || !self->entry_point || !kernel::g_scheduler_ptr || !kernel::g_platform) {
        if (kernel::g_platform) kernel::g_platform->panic("Invalid args in thread_bootstrap", __FILE__, __LINE__); else for(;;); return;
    }
    // Ensure interrupts are enabled as per this thread's TCB.pstate
    asm volatile("msr daifclr, #2"); // Assuming tasks run with IRQs generally enabled
    kernel::configure_memory_protection(self, true);
    self->entry_point(self->arg_ptr); 
    kernel::configure_memory_protection(self, false);
    uint64_t daif_flags_on_exit;
    asm volatile("mrs %0, daif; msr daifset, #0xf" : "=r"(daif_flags_on_exit) :: "memory");
    { ScopedLock lock(kernel::g_scheduler_ptr->get_global_scheduler_lock());
        trace::g_trace_manager.record_event(self, trace::EventType::THREAD_EXIT, self->name);
        self->state = TCB::State::ZOMBIE;
        kernel::g_scheduler_ptr->num_active_tasks_.fetch_sub(1, std::memory_order_relaxed);
    }
    uint32_t current_core_id = self->cpu_id_running_on;
    if (current_core_id >= MAX_CORES) { if (kernel::g_platform) kernel::g_platform->panic("Invalid core ID on exit", __FILE__, __LINE__); else for(;;); }
    kernel::g_scheduler_ptr->schedule(current_core_id, false); 
    if (kernel::g_platform) kernel::g_platform->panic("Thread returned from bootstrap after schedule on exit", __FILE__, __LINE__); else for(;;);
}

} // namespace core

void trace_event(const char* event_str, uintptr_t arg1, uintptr_t arg2) {
    if (!g_platform || !g_platform->get_timer_ops() || !trace::g_trace_manager.is_enabled()) return;
    uint32_t core_id = g_platform->get_core_id();
    if (core_id >= core::MAX_CORES) return;
    core::ScopedISRLock lock(g_trace_lock); 
    static std::atomic<size_t> g_legacy_trace_idx{0};
    size_t current_idx = g_legacy_trace_idx.fetch_add(1, std::memory_order_relaxed);
    size_t buffer_idx = current_idx % core::TRACE_BUFFER_SIZE;
    if (current_idx >= core::TRACE_BUFFER_SIZE && (current_idx % core::TRACE_BUFFER_SIZE == 0) ) {
        core::g_trace_overflow_count[core_id].fetch_add(1, std::memory_order_relaxed);
    }
    auto* entry_ptr = &core::g_trace_buffer[buffer_idx];
    entry_ptr->timestamp_us = g_platform->get_timer_ops()->get_system_time_us();
    entry_ptr->core_id = core_id; entry_ptr->event_str = event_str;
    entry_ptr->arg1 = arg1; entry_ptr->arg2 = arg2;
}

void dump_trace_buffer(hal::UARTDriverOps* uart_ops) { 
    if (!uart_ops) return;
    core::ScopedLock lock(g_trace_lock); 
    uart_ops->puts("\n--- Legacy Global Trace Buffer ---\n");
    size_t num_valid_entries = 0;
    for(const auto& entry : core::g_trace_buffer) if(entry.event_str != nullptr) num_valid_entries++;
    if (num_valid_entries == 0) {
        uart_ops->puts("Legacy trace buffer empty or uninitialized.\n--- End Legacy Trace ---\n"); return;
    }
    for (size_t i = 0; i < core::TRACE_BUFFER_SIZE; ++i) {
        const core::TraceEntry& entry = core::g_trace_buffer[i];
        if (entry.event_str) {
            char buf[128];
            kernel::util::k_snprintf(buf, sizeof(buf), "[LTraceC%u] %llu us: %s", entry.core_id, (unsigned long long)entry.timestamp_us, entry.event_str);
            uart_ops->puts(buf);
            if (entry.arg1 != 0 || entry.arg2 != 0) {
                 kernel::util::k_snprintf(buf, sizeof(buf), ", args: 0x%llx, 0x%llx", 
                    static_cast<unsigned long long>(entry.arg1), static_cast<unsigned long long>(entry.arg2));
                 uart_ops->puts(buf);
            }
            uart_ops->puts("\n");
        }
    }
    uart_ops->puts("--- End Legacy Trace ---\n");
    for (uint32_t i = 0; i < core::MAX_CORES; ++i) {
        size_t overflows = core::g_trace_overflow_count[i].load(std::memory_order_relaxed);
        if (overflows > 0) {
            char buf[64]; kernel::util::k_snprintf(buf, sizeof(buf), "Core %u legacy trace overflow count: %zu\n", i, overflows);
            uart_ops->puts(buf);
        }
    }
}
void get_kernel_stats(hal::UARTDriverOps* uart_ops) { 
    if (!uart_ops || !g_scheduler_ptr) return; 
    core::ScopedLock lock(g_scheduler_ptr->get_global_scheduler_lock()); 
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf), "\n--- Kernel Stats ---\nActive tasks: %zu\n", g_scheduler_ptr->get_num_active_tasks()); 
    uart_ops->puts(buf);
    for (uint32_t core_idx = 0; core_idx < core::MAX_CORES; ++core_idx) {
        if (core_idx < core::g_per_cpu_data.size()) { 
            const core::PerCPUData& cpu_data = core::g_per_cpu_data[core_idx];
            if (cpu_data.current_thread) {
                kernel::util::k_snprintf(buf, sizeof(buf), "Core %u: Running Task='%s' (TCB:%p, Prio:%d, Deadline:%lluus)\n",
                              core_idx, cpu_data.current_thread->name, (void*)cpu_data.current_thread,
                              cpu_data.current_thread->priority, (unsigned long long)cpu_data.current_thread->deadline_us);
                uart_ops->puts(buf);
            } else {
                kernel::util::k_snprintf(buf, sizeof(buf), "Core %u: No current task assigned.\n", core_idx); uart_ops->puts(buf);
            }
             if (cpu_data.idle_thread) {
                kernel::util::k_snprintf(buf, sizeof(buf), "Core %u: Idle Task='%s' (TCB:%p)\n",
                              core_idx, cpu_data.idle_thread->name, (void*)cpu_data.idle_thread);
                uart_ops->puts(buf);
            }
        }
    }
    uart_ops->puts("--- End Stats ---\n");
}

extern "C" void kernel_main() {
    early_uart_puts("[DEBUG] Entering kernel_main\n");
    g_platform = hal::get_platform();
    early_uart_puts("[DEBUG] Got platform\n");
    if (!g_platform) {
        early_uart_puts("[DEBUG] Null platform, halting\n");
        for(;;);
        return;
    }
    early_uart_puts("[DEBUG] g_platform address: ");
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "0x%llx\n", (unsigned long long)g_platform);
    early_uart_puts(addr_buf);
    early_uart_puts("[DEBUG] g_platform vtable address: ");
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "0x%llx\n", *(unsigned long long*)g_platform);
    early_uart_puts(addr_buf);
    early_uart_puts("[DEBUG] Calling early_init_platform\n");
    g_platform->early_init_platform();
    early_uart_puts("[DEBUG] early_init_platform done\n");
    static core::Scheduler scheduler_instance;
    scheduler_instance.set_policy(new core::EDFPolicy());
    g_scheduler_ptr = &scheduler_instance;

    if (!core::g_software_timer_obj_pool.init(core::g_software_timer_obj_pool_mem, 
                                              core::MAX_SOFTWARE_TIMERS,
                                              sizeof(hal::timer::SoftwareTimer), 
                                              alignof(hal::timer::SoftwareTimer))) {
        g_platform->panic("Failed to init software timer pool", __FILE__, __LINE__); return;
    }
    trace::g_trace_manager.init();
    trace::g_trace_manager.set_enabled(true);

    for (uint32_t i = 0; i < g_platform->get_num_cores(); ++i) {
        char idle_name[core::MAX_NAME_LENGTH];
        kernel::util::k_snprintf(idle_name, sizeof(idle_name), "idle%u", i);
        if (!g_scheduler_ptr->create_thread(&core::Scheduler::idle_thread_func, 
                reinterpret_cast<void*>(static_cast<uintptr_t>(i)), 0, static_cast<int>(i), idle_name, true, 0)) {
            g_platform->panic("Failed to create idle thread", __FILE__, __LINE__); return;
        }
    }
    
    for (uint32_t i = 0; i < g_platform->get_num_cores(); ++i) {
        g_platform->early_init_core(i); 
        g_scheduler_ptr->start_core_scheduler(i); 
    }
    
    if (g_platform->get_core_id() == 0) { // Only core 0 enables its IRQs here.
         asm volatile("msr daifclr, #2"); // Enable IRQs (clear PSTATE.I bit)
    }
} // namespace core
} // namespace kernel