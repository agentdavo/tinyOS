// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file core.hpp
 * @brief Core kernel types and constants for miniOS v1.7.
 * @details
 * Defines essential kernel types (TCB, Scheduler, Spinlock) and global constants
 * for the miniOS RTOS. Designed to be dependency-free except for standard headers.
 *
 * @version 1.7
 * @see core.cpp, hal.hpp
 */

#ifndef CORE_HPP
#define CORE_HPP

#include <cstdint>
#include <cstddef>
#include <span>
#include <string_view>
#include <optional>
#include <atomic>
#include <array>
#include <concepts>

namespace kernel {
namespace core {

// Global constants
// A full non-fake-slave boot creates ~18 threads (4 idle + cli/uart_io/ui +
// motion/gcode/macro/ladder/probe/jobs/hmi + ec_a/bus_cfg_a/ec_b/bus_cfg_b),
// which silently overran the old budget of 16 — create_thread returned
// nullptr and the last services (the second EtherCAT master) never started.
// 32 covers the full service set plus headroom for CLI-spawned threads.
// Cost: g_task_stacks grows to 32 * DEFAULT_STACK_SIZE = 512 KB of .bss.
constexpr size_t MAX_THREADS = 32;
constexpr size_t MAX_NAME_LENGTH = 32;
constexpr size_t MAX_CORES = 4;
constexpr size_t MAX_PRIORITY_LEVELS = 16;
// EtherCAT code allocates 1514-byte frame buffers on the stack; 4 KB was too
// tight for the motion + EC threads at -O0. Bump to 16 KB.
constexpr size_t DEFAULT_STACK_SIZE = 16384;

// Stack-painting controls used by Scheduler::create_thread + TCB::stack_used_bytes.
// At thread creation we fill the whole stack region with STACK_PAINT_BYTE; at
// `top` time we count unbroken pattern bytes from the base up to estimate
// high-water-mark usage. Toggle STACK_PAINT to false to disable.
constexpr bool    STACK_PAINT      = true;
constexpr uint8_t STACK_PAINT_BYTE = 0xAA;
// The lowest few bytes of every stack get touched by the initial entry path
// regardless of whether the thread runs (compiler-emitted prologue, our
// thread_bootstrap setup, etc). Skip them so they don't count as "used" and
// give every brand-new thread a misleading non-zero watermark.
constexpr size_t  STACK_PAINT_SKIP_BOTTOM = 16;

// Forward declarations
struct PerCPUData; // Forward declaration
struct TCB;

// Plain test-and-set spinlock. constexpr-constructible so every global
// lock is constant-initialised (no static-init-order hazard). There used to
// be priority inheritance here, but the EDF policy picks by deadline, not
// priority, so a boost could not help the holder run — and the boost/restore
// raced with release, leaving priorities permanently inflated.
class Spinlock {
    std::atomic<bool> lock_flag_{false};
public:
    constexpr Spinlock() noexcept = default;
    // ISR-safe variants return/take the caller's interrupt-mask state so the
    // critical section is entered with IRQs disabled and exited with the
    // pre-acquire mask restored. This closes the window where a timer IRQ
    // fires while a thread holds a per-core scheduler lock and the handler
    // tries to re-acquire the same lock from preemptive_tick → schedule.
    [[nodiscard]] uint64_t acquire_isr_safe() noexcept;
    void release_isr_safe(uint64_t saved_irq_state) noexcept;
    void acquire_general() noexcept;
    void release_general() noexcept;
};

class ScopedLock {
    Spinlock& lock_;
public:
    explicit ScopedLock(Spinlock& l) noexcept : lock_(l) { lock_.acquire_general(); }
    ~ScopedLock() { lock_.release_general(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};

class ScopedISRLock {
    Spinlock& lock_;
    uint64_t saved_irq_state_;
public:
    explicit ScopedISRLock(Spinlock& l) noexcept
        : lock_(l), saved_irq_state_(l.acquire_isr_safe()) {}
    ~ScopedISRLock() { lock_.release_isr_safe(saved_irq_state_); }
    ScopedISRLock(const ScopedISRLock&) = delete;
    ScopedISRLock& operator=(const ScopedISRLock&) = delete;
};

// TCB is laid out to match the byte offsets used by cpu_arm64.S (TCB_REGS_X0_OFFSET
// etc). Do not reorder or insert members without updating the .S. Member order is
// intentionally public to assembly via offsets; C++ access is restricted to the
// scheduler and its policies.
struct TCB {
    uint64_t regs[31];
    uint64_t sp;
    uint64_t pc;
    uint64_t pstate;
    // FP/SIMD state, saved and restored by both the trap path and the
    // voluntary switch on each arch. arm64: q0..q31 (two words each) then
    // fp_ctrl = {FPCR, FPSR}. rv64: f0..f31 in fp_regs[0..31] then
    // fp_ctrl[0] = fcsr. Offsets are pinned below for the .S files.
    alignas(16) uint64_t fp_regs[64];
    uint64_t fp_ctrl[2];
    void (*entry_point)(void*);
    void* arg_ptr;
    uint8_t* stack_base;
    size_t stack_size;
    enum class State { INACTIVE, READY, RUNNING, BLOCKED, ZOMBIE } state = State::INACTIVE;
    int priority;
    int core_affinity;
    uint32_t cpu_id_running_on;
    char name[MAX_NAME_LENGTH];
    TCB* next_in_q = nullptr;
    uint64_t deadline_us = 0;
    // Sequence number of the last context-switch INTO this task (a global
    // monotonic counter — only the order matters). Used as a tie-breaker in
    // EDFPolicy::select_next_task: among tasks with equal earliest deadline,
    // the one scheduled longest ago wins. Without this, same-deadline tasks
    // at different priority levels form a duopoly between the two highest
    // priorities (EDF iterates priority high to low, picks the first
    // deadline match, and the immediate-caller exclusion alone isn't enough
    // to give lower-priority peers a turn). It used to be a timestamp, which
    // cost a timer read + 128-bit divide on every context switch.
    uint64_t last_scheduled_seq = 0;

    // Walk the stack from base up, counting unbroken STACK_PAINT_BYTE bytes
    // (skipping STACK_PAINT_SKIP_BOTTOM). Returns stack_size - unused. If
    // STACK_PAINT is disabled or the stack pointer is null, returns 0.
    size_t stack_used_bytes() const noexcept {
        if (!STACK_PAINT || !stack_base || stack_size == 0) return 0;
        size_t skip = (STACK_PAINT_SKIP_BOTTOM < stack_size) ? STACK_PAINT_SKIP_BOTTOM : stack_size;
        size_t unused = skip;
        for (size_t i = skip; i < stack_size; ++i) {
            if (stack_base[i] != STACK_PAINT_BYTE) break;
            ++unused;
        }
        return (unused < stack_size) ? (stack_size - unused) : 0;
    }
};

// Byte offsets the context-switch assembly hard-codes (cpu_arm64.S TCB_*_OFFSET,
// cpu_rv64.S TCB_*_OFF). Change these together or not at all.
static_assert(offsetof(TCB, regs)    == 0,   "TCB.regs offset is fixed by the .S files");
static_assert(offsetof(TCB, sp)      == 248, "TCB.sp offset is fixed by the .S files");
static_assert(offsetof(TCB, pc)      == 256, "TCB.pc offset is fixed by the .S files");
static_assert(offsetof(TCB, pstate)  == 264, "TCB.pstate offset is fixed by the .S files");
static_assert(offsetof(TCB, fp_regs) == 272, "TCB.fp_regs offset is fixed by the .S files");
static_assert(offsetof(TCB, fp_ctrl) == 784, "TCB.fp_ctrl offset is fixed by the .S files");

struct PerCPUData {
    TCB* current_thread = nullptr; 
    TCB* idle_thread = nullptr;    
};

// Declare g_per_cpu_data after PerCPUData definition
alignas(64) extern std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

// Storage for all thread control blocks. The scheduler hands them out from
// this pool; CLI introspection (e.g. `top`) walks the array.
extern std::array<TCB, MAX_THREADS> g_task_tcbs;

class SchedulerPolicy {
public:
    virtual ~SchedulerPolicy() = default;
    virtual TCB* select_next_task(uint32_t core_id, TCB* current_task) = 0;
    virtual void add_to_ready_queue(TCB* tcb, uint32_t core_id) = 0;
};

class EDFPolicy : public SchedulerPolicy {
public:
    TCB* select_next_task(uint32_t core_id, TCB* current_task) override;
    void add_to_ready_queue(TCB* tcb, uint32_t core_id) override;
};

class Scheduler {
    std::array<std::array<TCB*, MAX_PRIORITY_LEVELS>, MAX_CORES> ready_qs_ = {};
    std::atomic<size_t> num_active_tasks_{0}; 
    Spinlock scheduler_global_lock_;          
    std::array<Spinlock, MAX_CORES> per_core_locks_; 
    SchedulerPolicy* policy_ = nullptr;
public:
    Scheduler();
    ~Scheduler();
    void set_policy(SchedulerPolicy* p) noexcept { policy_ = p; }
    TCB* create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle = false, uint64_t deadline_us = 0);
    void start_core_scheduler(uint32_t core_id); 
    void preemptive_tick(uint32_t core_id);      
    void yield(uint32_t core_id);                
    size_t get_num_active_tasks() const noexcept { return num_active_tasks_.load(std::memory_order_relaxed); }
    Spinlock& get_global_scheduler_lock() noexcept { return scheduler_global_lock_; }
friend class EDFPolicy; 
private:
    TCB* pop_highest_priority_ready_task(uint32_t current_core_id);
    void schedule(uint32_t core_id, bool is_preemption);
public:
    static void idle_thread_func(void* arg); 
private:
    static void thread_bootstrap(TCB* self); 
};

} // namespace core
} // namespace kernel

// Define kernel_g_per_cpu_data to alias g_per_cpu_data for C linkage
extern "C" kernel::core::PerCPUData* const kernel_g_per_cpu_data;

#endif // CORE_HPP
