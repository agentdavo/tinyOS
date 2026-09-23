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
#include "diag/cpu_load.hpp"

#include <cstring>
#include <algorithm>    
#include <limits>       
#include <atomic> 
#include <cstddef>

extern "C" void early_uart_puts(const char* str);

// Arch-neutral helpers for the code paths below. The core scheduler needs
// three things from the CPU: a relax-hint (cpu_relax), save/disable/restore
// of the interrupt-mask state during a context switch, and a PSCI-style
// "bring up core N" primitive. arm64 uses DAIF + HVC/SMC (PSCI), rv64 uses
// SSTATUS SIE + a spin-table entry slot. Guard each with the compiler's
// builtin arch macro so this file stays a single source.
#if defined(__aarch64__)
#  define MINIOS_CPU_RELAX() asm volatile("yield")
static inline uint64_t minios_irq_save_disable() noexcept {
    uint64_t f;
    asm volatile("mrs %0, daif; msr daifset, #0xf" : "=r"(f) :: "memory");
    return f;
}
static inline void minios_irq_restore(uint64_t f) noexcept {
    asm volatile("msr daif, %0" :: "r"(f) : "memory");
}
static inline void minios_irq_enable() noexcept {
    asm volatile("msr daifclr, #2");
}
#elif defined(__riscv)
#  define MINIOS_CPU_RELAX() asm volatile("nop")
// miniOS rv64 runs in M-mode, so the relevant interrupt-enable bit is
// mstatus.MIE (bit 3), not sstatus.SIE (bit 1, which only gates supervisor
// interrupts). Manipulating sstatus would silently leave M-mode interrupts
// enabled and the scheduler critical sections would race with the timer
// trap.
static inline uint64_t minios_irq_save_disable() noexcept {
    uint64_t f;
    asm volatile("csrrci %0, mstatus, 0x8" : "=r"(f) :: "memory");
    return f;
}
static inline void minios_irq_restore(uint64_t f) noexcept {
    if (f & 0x8) {
        asm volatile("csrsi mstatus, 0x8" ::: "memory");
    } else {
        asm volatile("csrci mstatus, 0x8" ::: "memory");
    }
}
static inline void minios_irq_enable() noexcept {
    asm volatile("csrsi mstatus, 0x8");
}
#else
#  define MINIOS_CPU_RELAX() asm volatile("")
static inline uint64_t minios_irq_save_disable() noexcept { return 0; }
static inline void minios_irq_restore(uint64_t) noexcept {}
static inline void minios_irq_enable() noexcept {}
#endif
extern "C" kernel::core::PerCPUData* const kernel_g_per_cpu_data = kernel::core::g_per_cpu_data.data();

namespace kernel {
namespace core {

alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;
// IRQs are masked BEFORE the CAS spin so that a timer IRQ can't fire while
// this CPU holds the lock and re-enter the same lock from the scheduler tick
// path. The pre-acquire mask state is returned to the caller and round-tripped
// through release_isr_safe so nested critical sections re-enable IRQs in the
// correct stack order.
uint64_t Spinlock::acquire_isr_safe() noexcept {
    const uint64_t saved = minios_irq_save_disable();
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {
            MINIOS_CPU_RELAX();
        }
    }
    return saved;
}
void Spinlock::release_isr_safe(uint64_t saved_irq_state) noexcept {
    lock_flag_.store(false, std::memory_order_release);
    minios_irq_restore(saved_irq_state);
}

void Spinlock::acquire_general() noexcept {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {
            MINIOS_CPU_RELAX();
        }
    }
}

void Spinlock::release_general() noexcept {
    lock_flag_.store(false, std::memory_order_release);
}

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* current_task) {
    if (core_id >= MAX_CORES || !kernel::g_scheduler_ptr) return nullptr;
    Scheduler* sched = kernel::g_scheduler_ptr;
    // ScopedISRLock so a timer IRQ that arrives while another CPU (or the
    // same CPU pre-mask) holds this per-core lock can't recurse into the
    // scheduler from preemptive_tick and self-deadlock on the same lock.
    ScopedISRLock lock(sched->per_core_locks_[core_id]);
    uint64_t earliest_deadline = std::numeric_limits<uint64_t>::max();
    TCB* earliest_task = nullptr; int chosen_priority_idx = -1; TCB* chosen_prev_in_list = nullptr;
    // Pass 1: pick the earliest-deadline task that is NOT the caller. This
    // lets cooperative yield() actually hand CPU off to a peer — otherwise a
    // task that yields from an incomplete-deadline spin-wait (HMI, UI refresh
    // sleeps) is re-picked immediately because its deadline stays lowest.
    // Pass 2 (if pass 1 found nothing) allows re-picking the caller.
    for (int pass = 0; pass < 2; ++pass) {
        for (int p_idx = MAX_PRIORITY_LEVELS - 1; p_idx >= 0; --p_idx) {
            TCB* task_iter = sched->ready_qs_[core_id][p_idx]; TCB* prev_in_list = nullptr;
            while (task_iter) {
                const bool is_caller = (task_iter == current_task);
                if (task_iter->state == TCB::State::READY &&
                    (task_iter->core_affinity == -1 || task_iter->core_affinity == static_cast<int>(core_id)) &&
                    (pass == 1 || !is_caller)) {
                    // deadline_us == 0 means "no explicit deadline" — treat as
                    // effectively-max so the task is selectable but always loses
                    // to any task with a real deadline. Without this, tasks
                    // created with deadline=0 (e.g. cli/uart_io) are silently
                    // invisible to EDF and never run on a core that hosts even
                    // one deadlined task.
                    const uint64_t eff_deadline = task_iter->deadline_us > 0
                        ? task_iter->deadline_us
                        : std::numeric_limits<uint64_t>::max() - 1;
                    // Tie-breaker on equal deadline: prefer the task that was
                    // scheduled longer ago (FIFO across priority levels). Keeps
                    // lower-priority same-deadline peers from being starved by
                    // a higher-priority task that yields and re-enters the
                    // queue immediately on every cycle.
                    const bool wins_outright = eff_deadline < earliest_deadline;
                    const bool wins_by_age   = earliest_task &&
                                               eff_deadline == earliest_deadline &&
                                               task_iter->last_scheduled_seq < earliest_task->last_scheduled_seq;
                    if (wins_outright || wins_by_age) {
                        earliest_deadline = eff_deadline; earliest_task = task_iter;
                        chosen_priority_idx = p_idx; chosen_prev_in_list = prev_in_list;
                    }
                }
                prev_in_list = task_iter; task_iter = task_iter->next_in_q;
            }
        }
        if (earliest_task) break;
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
    // ScopedISRLock for the same reason as select_next_task — preemptive_tick
    // can recurse here from a timer IRQ via schedule().
    ScopedISRLock lock(sched->per_core_locks_[core_id]);
    // FIFO enqueue (push to tail) so same-priority yielders go behind peers.
    // This is what makes cooperative yield() actually hand off CPU to another
    // ready task at the same priority level.
    tcb->next_in_q = nullptr;
    TCB** slot = &sched->ready_qs_[core_id][tcb->priority];
    if (!*slot) {
        *slot = tcb;
    } else {
        TCB* tail = *slot;
        while (tail->next_in_q) tail = tail->next_in_q;
        tail->next_in_q = tcb;
    }
    tcb->state = TCB::State::READY;
}

Scheduler::Scheduler() : policy_(nullptr) {
    for (size_t c = 0; c < MAX_CORES; ++c) {
        for (size_t p = 0; p < MAX_PRIORITY_LEVELS; ++p) { ready_qs_[c][p] = nullptr; }
    }
}
Scheduler::~Scheduler() = default; // Policy is not owned; storage is static.

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
    // Paint the entire stack with a known pattern so the `top` CLI command can
    // estimate high-water-mark usage by counting unbroken pattern bytes from
    // the base up. Compile-time constant — flip STACK_PAINT to false to skip
    // the (DEFAULT_STACK_SIZE * MAX_THREADS = 256 KB on arm64) memset on boot
    // if it ever shows up as a perf regression.
    if constexpr (STACK_PAINT) {
        kernel::util::kmemset(tcb.stack_base, STACK_PAINT_BYTE, tcb.stack_size);
    }
    tcb.sp = reinterpret_cast<uint64_t>(tcb.stack_base + tcb.stack_size) & ~0xFUL;
    tcb.pc = reinterpret_cast<uint64_t>(thread_bootstrap);
#if defined(__aarch64__)
    // SPSR_EL1 for EL1h with IRQs enabled: M[4]=0 (AArch64), M[3:0]=0101 (EL1h), DAIF all unmasked (bits 9-6 are 0)
    tcb.pstate = 0x00000005;
    tcb.regs[0] = reinterpret_cast<uint64_t>(&tcb); // thread_bootstrap argument (x0)
#elif defined(__riscv)
    // mstatus seed for rv64 M-mode: MPP=M (bits 11-12=0b11) so mret stays in
    // M-mode, MPIE=1 (bit 7) so interrupts re-enable on mret, FS=Dirty
    // (bits 13-14=0b11) so floating-point insns don't trap.
    tcb.pstate = (0x3ULL << 11) | (0x3ULL << 13) | (1ULL << 7);
    // rv64 register seeding for cpu_context_switch_rv64's first switch:
    // regs[i] holds x(i+1), so regs[0]=ra, regs[1]=sp, regs[9]=a0. The
    // ret at the end of cpu_context_switch_rv64 jumps to ra (=bootstrap),
    // and the trampoline expects its TCB pointer in a0.
    tcb.regs[0] = reinterpret_cast<uint64_t>(thread_bootstrap); // ra
    tcb.regs[1] = tcb.sp;                                       // sp
    tcb.regs[9] = reinterpret_cast<uint64_t>(&tcb);             // a0
#else
    tcb.pstate = 0;
    tcb.regs[0] = reinterpret_cast<uint64_t>(&tcb);
#endif
    tcb.deadline_us = deadline_us; tcb.state = TCB::State::READY;
    tcb.cpu_id_running_on = static_cast<uint32_t>(-1);
    trace::g_trace_manager.record_event(&tcb, trace::EventType::THREAD_CREATE, tcb.name);
    uint32_t target_core = (tcb.core_affinity != -1) ? static_cast<uint32_t>(tcb.core_affinity) : 0;
    if (is_idle) {
        g_per_cpu_data[target_core].idle_thread = &tcb;
#if defined(__aarch64__)
        // SPSR_EL1 for EL1h with IRQs unmasked. arm64 idle needs this so the
        // first eret into idle keeps IRQs enabled and the scheduler tick can
        // actually fire on a core that's running idle.
        tcb.pstate = 0x00000005;
#elif defined(__riscv)
        // The non-idle seed already set MPP=M, FS=Dirty, MPIE=1 — that's
        // exactly what idle needs too. Don't overwrite it with the arm64
        // value (which on rv64 means MPP=U and MIE off).
#endif
    }
    // Idle threads never live in the ready queue — they're the fallback when
    // the queue is empty (see pop_highest_priority_ready_task).
    if (!is_idle) {
        if (policy_) policy_->add_to_ready_queue(&tcb, target_core);
        else { if (kernel::g_platform) kernel::g_platform->panic("Scheduler policy not set", __FILE__, __LINE__); return nullptr; }
    }
    num_active_tasks_.fetch_add(1, std::memory_order_relaxed);
    return &tcb;
}

void Scheduler::start_core_scheduler(uint32_t core_id) {
    if (!kernel::g_platform || core_id >= MAX_CORES || !kernel::g_platform->get_irq_ops() || !kernel::g_platform->get_timer_ops()) {
        if (kernel::g_platform) kernel::g_platform->panic("Invalid core_id or platform components missing", __FILE__, __LINE__); else for(;;); return;
    }
    auto* idle_tcb = g_per_cpu_data[core_id].idle_thread;
    if (!idle_tcb) { kernel::g_platform->panic("Idle thread not created for core", __FILE__, __LINE__); return; }

    // Install an initial runnable task before the first eret. Shared cores used
    // to enter idle unconditionally and rely on the first timer tick to pull a
    // worker from the ready queue; under QEMU that can delay or suppress first
    // progress during bring-up. Starting with the highest-priority ready task
    // makes thread bring-up deterministic on both shared and dedicated cores.
    //
    // Take the per-core lock around the pop. Without it the secondary hart
    // could race against the primary's add_to_ready_queue (the lock-release
    // pairing is what makes the producing hart's TCB writes — entry_point,
    // name, sp — visible here), grab a partially-initialised TCB, then trap
    // in safe_strcpy when thread_bootstrap dereferences the half-written
    // name pointer.
    TCB* worker = nullptr;
    {
        ScopedISRLock lock(per_core_locks_[core_id]);
        worker = pop_highest_priority_ready_task(core_id);
    }
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        char buf[128];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[sched] core %u initial worker=%s prio=%d aff=%d\n",
                                 core_id,
                                 (worker && worker != idle_tcb) ? worker->name : "idle",
                                 (worker && worker != idle_tcb) ? worker->priority : -1,
                                 (worker && worker != idle_tcb) ? worker->core_affinity : -99);
        uart->puts(buf);
    }
    if (!worker || worker == idle_tcb) {
        g_per_cpu_data[core_id].current_thread = idle_tcb;
        idle_tcb->state = TCB::State::RUNNING;
        idle_tcb->cpu_id_running_on = core_id;
    } else {
        g_per_cpu_data[core_id].current_thread = worker;
        worker->state = TCB::State::RUNNING;
        worker->cpu_id_running_on = core_id;
    }
    // Dedicated (tickless) RT cores get the same enables: their scheduler
    // tick is disabled in TimerDriver::init_core_timer_interrupt instead, and
    // the timer IRQ line stays enabled so TimerDriver::wait_until_ns can use
    // it as a wake source.
    kernel::g_platform->get_irq_ops()->enable_irq_line(kernel::hal::SYSTEM_TIMER_IRQ);
    kernel::g_platform->get_irq_ops()->enable_core_irqs(core_id, 0x1);
}

void Scheduler::preemptive_tick(uint32_t core_id) {
    if (core_id >= MAX_CORES) return;
    // Per-core busy/total tick counters for the `top` CLI command. Counted at
    // tick entry (before the schedule decision) using the *currently running*
    // thread; idle ticks don't increment ticks_busy.
    auto& cc = diag::g_core_counters[core_id];
    cc.ticks_total.fetch_add(1, std::memory_order_relaxed);
    TCB* cur = g_per_cpu_data[core_id].current_thread;
    if (cur && cur != g_per_cpu_data[core_id].idle_thread) {
        cc.ticks_busy.fetch_add(1, std::memory_order_relaxed);
    }
    // Legacy trace_event removed from IRQ-preempt path — see add_to_ready_queue.
    schedule(core_id, true);
}

void Scheduler::yield(uint32_t core_id) {
    // schedule() switches the CALLING CPU's context, so it must be handed the
    // calling core. A mismatched id (e.g. a hard-coded yield(0) from a thread
    // on another core) would requeue the other core's running thread, save
    // this core's registers into its TCB and run its successor here — the
    // other core keeps executing under a stale current_thread and threads
    // later resume from bogus snapshots. Always use the real core id.
    if (kernel::g_platform) core_id = kernel::g_platform->get_core_id();
    if (core_id >= MAX_CORES) return;
    schedule(core_id, false);
}

TCB* Scheduler::pop_highest_priority_ready_task(uint32_t current_core_id) {
    if (current_core_id >= MAX_CORES) return nullptr;
    // Caller is responsible for holding per_core_locks_[current_core_id].
    // The two callers — start_core_scheduler and EDFPolicy::select_next_task
    // — both take the lock themselves; making this function take it too
    // would self-deadlock the select_next_task → pop fallback path because
    // the spinlock isn't recursive.
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
    (void)is_preemption; // Reserved for future use (e.g. distinguishing voluntary yield).
    if (core_id >= MAX_CORES || !policy_ || !kernel::g_platform) return;
    // Mask IRQs for the duration of the switch decision; the policy functions
    // take `per_core_locks_[core_id]` themselves, so this outer path must not
    // hold it (doing so would deadlock against add_to_ready_queue /
    // select_next_task, which re-acquire the same lock).
    const uint64_t daif_flags = minios_irq_save_disable();
    TCB* current_task = g_per_cpu_data[core_id].current_thread; TCB* next_task = nullptr;
    if (current_task && current_task->state == TCB::State::RUNNING) {
        // Idle is never re-added — it's the fallback.
        if (current_task != g_per_cpu_data[core_id].idle_thread) {
            current_task->state = TCB::State::READY;
            policy_->add_to_ready_queue(current_task, core_id);
        }
    }
    next_task = policy_->select_next_task(core_id, current_task);
    if (!next_task) {
        next_task = g_per_cpu_data[core_id].idle_thread;
        if (!next_task && kernel::g_platform) { 
            kernel::g_platform->panic("Idle thread null in schedule", __FILE__, __LINE__); 
            minios_irq_restore(daif_flags);
            return; 
        }
    }
    if (current_task != next_task) {
        g_per_cpu_data[core_id].current_thread = next_task;
        next_task->state = TCB::State::RUNNING;
        next_task->cpu_id_running_on = core_id;
        static std::atomic<uint64_t> s_sched_seq{0};
        next_task->last_scheduled_seq = s_sched_seq.fetch_add(1, std::memory_order_relaxed) + 1;
        // record_event left out of this hot path — it reads the timer and
        // does an atomic fetch_add on every context switch. Re-enable if
        // you're actively instrumenting scheduling.
        // cpu_context_switch does not return for the old task.
        // Interrupts (DAIF) will be restored by the 'eret' in cpu_context_switch_impl, using new_task->pstate.
        kernel::hal::cpu_context_switch(current_task, next_task);
    } else {
        if(current_task) { current_task->state = TCB::State::RUNNING; current_task->cpu_id_running_on = core_id; }
        minios_irq_restore(daif_flags); // Restore IRQ mask if no switch
    }
}

void Scheduler::idle_thread_func(void* arg) {
    uint32_t core_id = reinterpret_cast<uintptr_t>(arg);
    if (!kernel::g_platform || !kernel::g_platform->get_power_ops()) { for (;;) { asm volatile("nop"); } }
    minios_irq_enable(); // Enable IRQs in this thread's pstate.
    while (true) {
        kernel::g_platform->get_power_ops()->enter_idle_state(core_id);
        kernel::hal::sync::barrier_dmb();
        // Cooperatively yield in case work was enqueued onto this core's
        // ready queue but no IRQ could wake us. On dedicated RT cores the
        // scheduler tick is disabled, so without this yield the first RT thread
        // bound to the core after start_core_scheduler() would never be
        // picked up. Cheap on shared cores (already preempted by timer).
        if (kernel::g_scheduler_ptr) kernel::g_scheduler_ptr->yield(core_id);
    }
}

void Scheduler::thread_bootstrap(TCB* self) {
    if (!self || !self->entry_point || !kernel::g_scheduler_ptr || !kernel::g_platform) {
        if (kernel::g_platform) kernel::g_platform->panic("Invalid args in thread_bootstrap", __FILE__, __LINE__); else for(;;); return;
    }
    // Ensure interrupts are enabled as per this thread's TCB.pstate
    minios_irq_enable(); // Assuming tasks run with IRQs generally enabled
    kernel::configure_memory_protection(self, true);
    self->entry_point(self->arg_ptr);
    kernel::configure_memory_protection(self, false);
    minios_irq_save_disable();
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

} // namespace kernel
