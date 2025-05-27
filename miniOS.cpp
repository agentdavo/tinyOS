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
#include "demo_cli_dsp.hpp"
#include "test_framework.hpp"
#include <cstring>
#include <cassert>

namespace kernel {

// --- Global Variables ---
Platform* g_platform = nullptr;
Scheduler* g_scheduler_ptr = nullptr;
volatile bool g_bss_cleared_smp_flag = false;
alignas(64) uint8_t g_audio_pool_mem[64 * 1024];
FixedMemoryPool g_audio_pool;
alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(timer::SoftwareTimer)];
FixedMemoryPool g_software_timer_obj_pool;
audio::AudioSystem g_audio_system;
trace::TraceManager g_trace_manager;
fs::FileSystem g_file_system;
net::NetManager g_net_manager;
gpio::GPIOManager g_gpio_manager;
Spinlock g_irq_handler_lock;
std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::atomic<size_t> g_trace_buffer_next_idx{0};
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count = {};
Spinlock g_trace_lock;
alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

// --- Spinlock ---
uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() : lock_id_(next_lock_id_++) {}

void Spinlock::acquire_isr_safe() {
    if (!g_platform) return;
    g_platform->get_irq_ops()->disable_core_irqs(g_platform->get_core_id());
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
    }
}

void Spinlock::release_isr_safe() {
    if (!g_platform) return;
    lock_flag_.store(false, std::memory_order_release);
    g_platform->get_irq_ops()->enable_core_irqs(g_platform->get_core_id(), 0);
}

void Spinlock::acquire_general() {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        expected = false;
        asm volatile("wfe" ::: "memory");
    }
}

void Spinlock::release_general() {
    lock_flag_.store(false, std::memory_order_release);
    asm volatile("sev" ::: "memory");
}

// --- FixedMemoryPool ---
bool FixedMemoryPool::init(void* base, size_t num, size_t blk_sz_user, size_t align_user_data) {
    if (!base || num == 0 || blk_sz_user == 0) return false;
    size_t actual_align = std::max(align_user_data, sizeof(void*));
    ScopedLock lock(pool_lock_);
    header_actual_size_ = (sizeof(Block) + actual_align - 1) & ~(actual_align - 1);
    block_storage_size_ = (header_actual_size_ + blk_sz_user + actual_align - 1) & ~(actual_align - 1);
    if (block_storage_size_ < header_actual_size_ + blk_sz_user) return false;
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
    block->next = free_head_;
    free_head_ = block;
    num_free_blocks_++;
}

// --- SPSCQueue ---
template<typename T, size_t Capacity>
bool SPSCQueue<T, Capacity>::enqueue(T* item) noexcept {
    size_t current_tail = tail_.load(std::memory_order_acquire);
    size_t next_tail = (current_tail + 1) & (Capacity - 1);
    if (next_tail == head_.load(std::memory_order_acquire)) return false;
    items_[current_tail] = item;
    tail_.store(next_tail, std::memory_order_release);
    return true;
}

template<typename T, size_t Capacity>
T* SPSCQueue<T, Capacity>::dequeue() noexcept {
    size_t current_head = head_.load(std::memory_order_acquire);
    if (current_head == tail_.load(std::memory_order_acquire)) return nullptr;
    T* item = items_[current_head];
    head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
    return item;
}

// Explicit template instantiation
template class SPSCQueue<audio::AudioBuffer, 16>;

// --- Scheduler ---
TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* current) {
    if (core_id >= MAX_CORES) return nullptr;
    TCB* earliest_task = nullptr;
    uint64_t earliest_deadline = UINT64_MAX;
    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);
    for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
        TCB* task = g_scheduler_ptr->ready_qs_[core_id][p];
        TCB* prev = nullptr;
        while (task) {
            if (task->deadline_us > 0 && task->deadline_us < earliest_deadline &&
                (task->core_affinity == -1 || task->core_affinity == static_cast<int>(core_id))) {
                earliest_deadline = task->deadline_us;
                earliest_task = task;
                if (prev) {
                    prev->next_in_q = task->next_in_q;
                } else {
                    g_scheduler_ptr->ready_qs_[core_id][p] = task->next_in_q;
                }
                task->next_in_q = nullptr;
                trace_event("SCHED:PopEDF", reinterpret_cast<uintptr_t>(task), core_id);
                return earliest_task;
            }
            prev = task;
            task = task->next_in_q;
        }
    }
    return g_scheduler_ptr->pop_highest_priority_ready_task(core_id);
}

void EDFPolicy::add_to_ready_queue(TCB* tcb, uint32_t core_id) {
    if (!tcb || tcb->priority < 0 || tcb->priority >= MAX_PRIORITY_LEVELS || core_id >= MAX_CORES) {
        if (g_platform) g_platform->panic("Invalid ready queue args", __FILE__, __LINE__);
        return;
    }
    ScopedLock lock(g_scheduler_ptr->per_core_locks_[core_id]);
    trace_event("SCHED:AddRdy", reinterpret_cast<uintptr_t>(tcb), (core_id << 16) | tcb->priority);
    tcb->next_in_q = g_scheduler_ptr->ready_qs_[core_id][tcb->priority];
    g_scheduler_ptr->ready_qs_[core_id][tcb->priority] = tcb;
    tcb->state = TCB::State::READY;
}

Scheduler::Scheduler() {
    policy_ = new EDFPolicy();
    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        auto* tcb = create_thread(idle_thread_func, reinterpret_cast<void*>(static_cast<uintptr_t>(i)), 0, i, "Idle", true);
        if (!tcb) g_platform->panic("Failed to create idle thread", __FILE__, __LINE__);
        g_per_cpu_data[i].idle_thread = tcb;
    }
}

TCB* Scheduler::create_thread(void (*fn)(void*), void* arg, int prio, int affinity, const char* name, bool is_idle, uint64_t deadline_us) {
    if (!fn || !name) return nullptr;
    ScopedLock lock(scheduler_global_lock_);
    if (num_active_tasks_ >= MAX_THREADS) return nullptr;
    for (size_t i = 0; i < MAX_THREADS; ++i) {
        if (g_task_tcbs[i].state == TCB::State::INACTIVE) {
            TCB& tcb = g_task_tcbs[i];
            tcb.entry_point = fn;
            tcb.arg_ptr = arg;
            tcb.priority = (prio >= 0 && prio < MAX_PRIORITY_LEVELS) ? prio : 0;
            tcb.core_affinity = (affinity >= -1 && affinity < static_cast<int>(MAX_CORES)) ? affinity : -1;
            if (!util::safe_strcpy(tcb.name, name, MAX_NAME_LENGTH)) return nullptr;
            tcb.stack_base = g_task_stacks[i].data();
            tcb.stack_size = DEFAULT_STACK_SIZE;
            tcb.state = TCB::State::READY;
            tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap);
            tcb.status = 0;
            tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size);
            tcb.deadline_us = deadline_us;
            if (!is_idle) {
                trace::g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name);
                policy_->add_to_ready_queue(&tcb, affinity != -1 ? affinity : 0);
                num_active_tasks_++;
            }
            return &tcb;
        }
    }
    return nullptr;
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!g_platform || core_id >= MAX_CORES) return;
    g_platform->get_irq_ops()->enable_core_irqs(core_id, 0);
    g_platform->get_timer_ops()->init_core_timer_interrupt();
    auto* tcb = g_per_cpu_data[core_id].idle_thread;
    if (!tcb) g_platform->panic("No idle thread", __FILE__, __LINE__);
    g_per_cpu_data[core_id].current_thread = tcb;
    tcb->state = TCB::State::RUNNING;
    tcb->cpu_id_running_on = core_id;
    extern void context_switch(void* current_tcb, void* next_tcb);
    context_switch(nullptr, tcb);
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    schedule(core_id, true);
}

void Scheduler::yield(uint32_t core_id) {
    schedule(core_id, false);
    trace::g_trace_manager.record_event(g_per_cpu_data[core_id].current_thread, trace::EventType::THREAD_YIELD, "yield");
}

void Scheduler::signal_event_isr(TCB* tcb) {
    if (!tcb) return;
    tcb->event_flag = true;
}

void Scheduler::wait_for_event(TCB* tcb) {
    if (!tcb) return;
    while (!tcb->event_flag) yield(g_platform->get_core_id());
    tcb->event_flag = false;
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;
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
    for (uint32_t i = 1; i < MAX_CORES; ++i) {
        uint32_t target_core = (current_core_id + i) % MAX_CORES;
        ScopedLock steal_lock(per_core_locks_[target_core]);
        for (int p = MAX_PRIORITY_LEVELS - 1; p >= 0; --p) {
            TCB* task = ready_qs_[target_core][p];
            TCB* prev = nullptr;
            while (task) {
                if (task->core_affinity == -1 || task->core_affinity == static_cast<int>(current_core_id)) {
                    if (prev) {
                        prev->next_in_q = task->next_in_q;
                    } else {
                        ready_qs_[target_core][p] = task->next_in_q;
                    }
                    task->next_in_q = nullptr;
                    trace_event("SCHED:StoleOK", reinterpret_cast<uintptr_t>(task), (target_core << 16) | p);
                    return task;
                }
                prev = task;
                task = task->next_in_q;
            }
        }
    }
    trace_event("SCHED:PopIdleAff", current_core_id);
    return g_per_cpu_data[current_core_id].idle_thread;
}

void Scheduler::schedule(uint32_t core_id, bool is_preemption) {
    if (core_id >= MAX_CORES) return;
    ScopedLock lock(per_core_locks_[core_id]);
    TCB* current = g_per_cpu_data[core_id].current_thread;
    if (current && current->state == TCB::State::RUNNING && !is_preemption) {
        current->state = TCB::State::READY;
    }
    TCB* next = policy_->select_next_task(core_id, current);
    if (!next || next == current) return;
    next->state = TCB::State::RUNNING;
    next->cpu_id_running_on = core_id;
    g_per_cpu_data[core_id].current_thread = next;
    if (current && current->state == TCB::State::READY) {
        policy_->add_to_ready_queue(current, core_id);
    }
    trace::g_trace_manager.record_event(next, trace::EventType::THREAD_SCHEDULE, next->name);
    extern void context_switch(void* current_tcb, void* next_tcb);
    context_switch(current, next);
}

void Scheduler::idle_thread_func(void* arg) {
    uint32_t core_id = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg));
    while (true) {
        set_power_mode(true);
        asm volatile("wfi" ::: "memory");
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self) return;
    configure_memory_protection(self, true);
    self->entry_point(self->arg_ptr);
    configure_memory_protection(self, false);
    trace::g_trace_manager.record_event(self, trace::EventType::THREAD_EXIT, self->name);
    self->state = TCB::State::INACTIVE;
    g_scheduler_ptr->yield(g_platform->get_core_id());
}

// --- Power Management ---
void set_power_mode(bool low_power) {
    if (!g_platform) return;
    auto* power_ops = g_platform->get_power_ops();
    if (low_power) {
        power_ops->enter_idle_state(g_platform->get_core_id());
    } else {
        power_ops->set_cpu_frequency(g_platform->get_core_id(), 0);
    }
}

// --- Memory Protection ---
void configure_memory_protection(TCB* tcb, bool enable) {
    if (!g_platform || !tcb) return;
    trace_event("MEM:Protection", reinterpret_cast<uintptr_t>(tcb), enable);
}

// --- Kernel Statistics ---
void get_kernel_stats(hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return;
    ScopedLock lock(g_scheduler_ptr->scheduler_global_lock_);
    uart_ops->puts("Kernel Statistics:\n");
    uart_ops->puts("  Active Threads: "); uart_ops->uart_put_uint64_hex(g_scheduler_ptr->num_active_tasks_); uart_ops->puts("\n");
    for (uint32_t core = 0; core < MAX_CORES; ++core) {
        uart_ops->puts("  Core "); uart_ops->putc('0' + core); uart_ops->puts(": ");
        if (g_per_cpu_data[core].current_thread) {
            uart_ops->puts(g_per_cpu_data[core].current_thread->name);
        } else {
            uart_ops->puts("(none)");
        }
        uart_ops->puts("\n");
    }
}

// --- Diagnostics ---
void trace_event(const char* event, uintptr_t a1, uintptr_t a2) {
    if (!g_platform || !event) return;
    uint32_t core_id = g_platform->get_core_id();
    size_t idx = g_trace_buffer_next_idx.fetch_add(1, std::memory_order_relaxed);
    if (idx >= TRACE_BUFFER_SIZE) {
        g_trace_overflow_count[core_id].fetch_add(1, std::memory_order_relaxed);
        idx %= TRACE_BUFFER_SIZE;
    }
    TraceEntry& entry = g_trace_buffer[idx];
    entry.timestamp_us = g_platform->get_timer_ops()->get_system_time_us();
    entry.core_id = core_id;
    entry.event_str = event;
    entry.arg1 = a1;
    entry.arg2 = a2;
}

void dump_trace_buffer(hal::UARTDriverOps* uart_ops) {
    if (!g_platform || !uart_ops) return;
    ScopedLock lock(g_trace_lock);
    uart_ops->puts("\n--- Trace Buffer ---\n");
    size_t start_idx = g_trace_buffer_next_idx.load(std::memory_order_relaxed) % TRACE_BUFFER_SIZE;
    size_t count = 0;
    for (size_t i = 0; i < TRACE_BUFFER_SIZE; ++i) {
        size_t idx = (start_idx + i) % TRACE_BUFFER_SIZE;
        const TraceEntry& entry = g_trace_buffer[idx];
        if (entry.event_str) {
            uart_ops->puts("T:"); uart_ops->uart_put_uint64_hex(entry.timestamp_us);
            uart_ops->puts(" C:"); uart_ops->putc('0' + entry.core_id);
            uart_ops->puts(" EVT:"); uart_ops->puts(entry.event_str);
            uart_ops->puts(" A1:"); uart_ops->uart_put_uint64_hex(entry.arg1);
            uart_ops->puts(" A2:"); uart_ops->uart_put_uint64_hex(entry.arg2);
            uart_ops->puts("\n");
            count++;
        }
    }
    if (count == 0) uart_ops->puts("(empty)\n");
    for (uint32_t i = 0; i < MAX_CORES; ++i) {
        size_t overflows = g_trace_overflow_count[i].load(std::memory_order_relaxed);
        if (overflows > 0) {
            uart_ops->puts("Core "); uart_ops->putc('0' + i);
            uart_ops->puts(" Overflows: "); uart_ops->uart_put_uint64_hex(overflows);
            uart_ops->puts("\n");
        }
    }
    uart_ops->puts("--- End Trace ---\n");
}

// --- Kernel Entry ---
extern "C" void kernel_main() {
    if (!g_platform) return;
    uint32_t core_id = g_platform->get_core_id();
    g_platform->early_init();
    static Scheduler scheduler;
    if (core_id == 0) {
        g_scheduler_ptr = &scheduler;
        if (!g_software_timer_obj_pool.init(g_software_timer_obj_pool_mem, MAX_SOFTWARE_TIMERS,
                                           sizeof(timer::SoftwareTimer), alignof(timer::SoftwareTimer))) {
            g_platform->panic("Timer pool init failed", __FILE__, __LINE__);
        }
        audio::AudioConfig audio_cfg{.sample_rate_hz = 48000, .samples_per_block = 256, .num_i2s_bufs = 4};
        if (!g_audio_system.init(audio_cfg) || !g_audio_system.start()) {
            g_platform->panic("Audio system init failed", __FILE__, __LINE__);
        }
        if (!g_file_system.create_file("/", true)) {
            g_platform->panic("File system init failed", __FILE__, __LINE__);
        }
        if (!g_gpio_manager.init(g_platform->get_gpio_ops())) {
            g_platform->panic("GPIO init failed", __FILE__, __LINE__);
        }
        if (!g_net_manager.init(g_platform->get_net_ops())) {
            g_platform->panic("Network init failed", __FILE__, __LINE__);
        }
        auto* watchdog_ops = g_platform->get_watchdog_ops();
        if (!watchdog_ops->start_watchdog(5000)) {
            g_platform->panic("Watchdog init failed", __FILE__, __LINE__);
        }
        if (!g_scheduler_ptr->create_thread(cli::CLI::cli_thread_entry, g_platform->get_uart_ops(), 3, 0, "CLI", false, 10000)) {
            g_platform->panic("CLI thread creation failed", __FILE__, __LINE__);
        }
        demo::register_demo_commands();
        test::register_tests();
        if (g_platform->get_uart_ops()) {
            g_platform->get_uart_ops()->puts("[miniOS v1.7] System Initialized\n");
        }
    } else {
        while (!g_scheduler_ptr) asm volatile("wfe" ::: "memory");
        if (g_platform->get_uart_ops()) {
            g_platform->get_uart_ops()->puts("[miniOS v1.7] Core ");
            g_platform->get_uart_ops()->putc('0' + core_id);
            g_platform->get_uart_ops()->puts(" Initialized\n");
        }
    }
    g_platform->core_early_init();
    g_scheduler_ptr->start_core_scheduler(core_id);
    g_platform->panic("Scheduler returned unexpectedly", __FILE__, __LINE__);
}

// --- IRQ Handling ---
void hal_irq_handler(uint32_t irq_id) {
    ScopedISRLock lock(g_irq_handler_lock);
    if (!g_platform) return;
    auto* irq_ops = g_platform->get_irq_ops();
    auto* timer_ops = g_platform->get_timer_ops();
    auto* watchdog_ops = g_platform->get_watchdog_ops();
    uint32_t core_id = g_platform->get_core_id();
    if (irq_id == 30 || irq_id == 7) { // Timer IRQ (ARM64: 30, RISC-V: 7)
        timer_ops->hardware_timer_irq_fired(core_id);
        watchdog_ops->reset_watchdog();
        g_scheduler_ptr->preemptive_tick(core_id);
    } else if (irq_id == 31 || irq_id == 8) { // Virtio-net IRQ (ARM64: 31, RISC-V: 8)
        trace_event("NET:IRQ", irq_id, core_id);
    } else if (irq_id < 1022 && irq_id != 1023) {
        trace_event("IRQ:Unhandled", irq_id, core_id);
        if (g_platform->get_uart_ops()) {
            g_platform->get_uart_ops()->puts("Unhandled IRQ: ");
            g_platform->get_uart_ops()->uart_put_uint64_hex(irq_id);
            g_platform->get_uart_ops()->puts("\n");
        }
    }
    irq_ops->end_irq(core_id, irq_id);
}

} // namespace kernel