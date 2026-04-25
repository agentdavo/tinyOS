// SPDX-License-Identifier: MIT OR Apache-2.0
// rv64_sched.cpp — minimal round-robin scheduler for the RISC-V port.
//
// This is a *self-contained* scheduler that doesn't link against core.cpp.
// The arm64 scheduler lives in core.cpp and is deeply entangled with that
// port's arm64-specific inline asm (DAIF manipulation, PSCI SMP bring-up,
// GICv3 operations) plus a stack of subsystems (CLI / EtherCAT / motion /
// devices / diag) that are also arm64-specific.  Rather than surgically
// generalise all of that at once, we stand up a small scheduler here that
// knows just enough to run a few test threads on rv64 with preemptive
// ticks.  Once arm64 is ported to use this same interface we can fold the
// two into a single arch-neutral scheduler.
//
// Interface contract (with cpu_rv64.S):
//   - TCB layout: see struct TCB_Rv64 below; byte offsets must match the
//     TCB_REGS_OFF / TCB_SP_OFF / TCB_PC_OFF / TCB_MSTATUS_OFF constants
//     in cpu_rv64.S.
//   - g_current_tcb[MAX_HARTS]: asm reads this on trap entry/exit and for
//     context switches.  Writing a new value to it IS the context switch
//     from the scheduler's perspective (the asm does the register
//     shuffling transparently).
//   - cpu_context_switch_rv64(old, new): voluntary switch.  old may be
//     null; new must be non-null.
//   - thread_bootstrap_rv64(TCB*): first-entry trampoline.  Calls
//     tcb->entry(tcb->arg).

#include "hal_qemu_rv64.hpp"
#include "../../core.hpp"
#include "../../util.hpp"
#include <cstdint>
#include <cstddef>
#include <atomic>

namespace hal::qemu_virt_rv64 {

// TCB_Rv64 is aliased to kernel::core::TCB in hal_qemu_rv64.hpp. The
// asm-visible prefix (regs[31], sp, pc, pstate) is identical on both arches
// — rv64 reads the `pstate` field as mstatus (same byte offset, 264).
static_assert(offsetof(TCB_Rv64, regs)    == 0,      "TCB regs offset");
static_assert(offsetof(TCB_Rv64, sp)      == 31 * 8, "TCB sp offset");
static_assert(offsetof(TCB_Rv64, pc)      == 32 * 8, "TCB pc offset");
static_assert(offsetof(TCB_Rv64, pstate)  == 33 * 8, "TCB pstate (mstatus) offset");

constexpr uint32_t MAX_THREADS   = 16;
constexpr size_t   STACK_BYTES   = 8192;

// Static TCB pool + static stacks.  No dynamic allocation in the kernel.
static TCB_Rv64 g_tcbs[MAX_THREADS]{};
alignas(16) static uint8_t g_stacks[MAX_THREADS][STACK_BYTES]{};

// Per-hart ready-queue head + currently-running thread.  Asm reads
// g_current_tcb directly; ready_head is C-only.  extern "C" linkage is
// needed so cpu_rv64.S can reference the array by its unmangled name.
extern "C" {
    TCB_Rv64* g_current_tcb[MAX_HARTS] = {nullptr, nullptr, nullptr, nullptr};
}
static TCB_Rv64*     g_ready_head[MAX_HARTS]{};
static TCB_Rv64*     g_idle[MAX_HARTS]{};

// Scheduler spinlock (coarse — one global).  We run preempt-safe by
// clearing mstatus.MIE while holding it.
static std::atomic<bool> g_sched_lock{false};

struct SchedLock {
    uint64_t prev_mstatus;
    SchedLock() {
        uint64_t m;
        // Clear MIE to make this section preempt-safe.
        asm volatile("csrrci %0, mstatus, 0x8" : "=r"(m));
        prev_mstatus = m;
        bool expected = false;
        while (!g_sched_lock.compare_exchange_weak(expected, true,
                   std::memory_order_acquire, std::memory_order_relaxed)) {
            expected = false;
            // No yield — hold the current hart until we win.
        }
    }
    ~SchedLock() {
        g_sched_lock.store(false, std::memory_order_release);
        // Restore prior MIE bit only (bit 3).
        if (prev_mstatus & 0x8) {
            asm volatile("csrsi mstatus, 0x8" ::: "memory");
        }
    }
};

static void ready_enqueue_locked(TCB_Rv64* t) {
    t->next_in_q = nullptr;
    uint32_t h = static_cast<uint32_t>(t->core_affinity < 0 ? 0 : t->core_affinity);
    TCB_Rv64** slot = &g_ready_head[h];
    if (*slot == nullptr) { *slot = t; return; }
    TCB_Rv64* tail = *slot;
    while (tail->next_in_q) tail = tail->next_in_q;
    tail->next_in_q = t;
}

static TCB_Rv64* ready_dequeue_locked(uint32_t hart) {
    TCB_Rv64* h = g_ready_head[hart];
    if (!h) return nullptr;
    g_ready_head[hart] = h->next_in_q;
    h->next_in_q = nullptr;
    return h;
}

// First-entry trampoline. cpu_context_switch_rv64 lands here with a0 =
// TCB pointer (restored via regs[9]=TCB*).  Enable interrupts and call the
// thread's entry.  If the entry returns, mark zombie and yield forever.
extern "C" void thread_bootstrap_rv64(TCB_Rv64* self);
extern "C" void thread_bootstrap_rv64(TCB_Rv64* self) {
    // Enable MIE so the preempt tick can fire.
    asm volatile("csrsi mstatus, 0x8" ::: "memory");
    if (self && self->entry_point) self->entry_point(self->arg_ptr);
    // Entry returned — park the hart.  In a fuller design we'd mark this
    // TCB zombie and request a switch.
    for (;;) asm volatile("wfi");
}

extern "C" void cpu_context_switch_rv64(TCB_Rv64* old_tcb, TCB_Rv64* new_tcb);

// ---- Public API ------------------------------------------------------

TCB_Rv64* sched_create_thread(void (*entry)(void*), void* arg,
                              uint32_t hart_affinity, const char* name,
                              bool is_idle) {
    if (hart_affinity >= MAX_HARTS) return nullptr;
    SchedLock lock;
    TCB_Rv64* t = nullptr;
    for (auto& cand : g_tcbs) {
        if (cand.state == TCB_Rv64::State::INACTIVE) { t = &cand; break; }
    }
    if (!t) return nullptr;
    // Zero-reset the TCB. TCB has a std::atomic<bool> event_flag that
    // defeats copy/move assign, so zero via memset (safe here because no
    // thread has observed this slot yet).
    kernel::util::kmemset(t, 0, sizeof(TCB_Rv64));
    t->state = TCB_Rv64::State::READY;
    t->entry_point = entry;
    t->arg_ptr = arg;
    t->core_affinity = static_cast<int>(hart_affinity);
    t->stack_base = g_stacks[t - g_tcbs];
    t->stack_size = STACK_BYTES;
    // Copy name (truncate).
    size_t i = 0;
    if (name) {
        for (; i < sizeof(t->name) - 1 && name[i]; ++i) t->name[i] = name[i];
    }
    t->name[i] = 0;

    // Seed register frame for first entry via cpu_context_switch_rv64:
    //   regs[0] (ra)  = thread_bootstrap_rv64
    //   regs[1] (sp)  = top-of-stack, 16-byte aligned
    //   regs[9] (a0)  = TCB pointer (bootstrap's argument)
    //   pstate        = rv64 mstatus. MIE=0 (bootstrap turns it on) + MPP=M
    //                   so mret-ish operations work; FS=Dirty so scheduled
    //                   C++ may use float registers without trapping.
    uintptr_t top = reinterpret_cast<uintptr_t>(t->stack_base + t->stack_size);
    top &= ~uintptr_t{15};
    t->regs[0] = reinterpret_cast<uint64_t>(&thread_bootstrap_rv64);
    t->regs[1] = top;
    t->regs[9] = reinterpret_cast<uint64_t>(t);
    t->sp      = top;
    t->pc      = reinterpret_cast<uint64_t>(&thread_bootstrap_rv64);
    t->pstate  = (0x3ULL << 11) | (0x3ULL << 13) | (1ULL << 7);

    if (!is_idle) ready_enqueue_locked(t);
    else          g_idle[hart_affinity] = t;
    return t;
}

// Called from hart_main-style code to transfer execution into the first
// scheduled thread for this hart.  Does not return.
[[noreturn]] void sched_enter_first(uint32_t hart) {
    TCB_Rv64* next = nullptr;
    {
        SchedLock lock;
        next = ready_dequeue_locked(hart);
        if (!next) next = g_idle[hart];
        g_current_tcb[hart] = next;
    }
    if (!next) for (;;) asm volatile("wfi");
    // First switch: no "old" to save.
    cpu_context_switch_rv64(nullptr, next);
    __builtin_unreachable();
}

// Called from the MTI trap path (see hal_trap_dispatch_rv64 in
// hal_qemu_rv64.cpp).  If there's a runnable thread other than the current,
// swap g_current_tcb[hart] to it — the trap-exit assembly will transparently
// restore into the new thread.
extern "C" void rv64_preemptive_tick(uint32_t hart) {
    if (hart >= MAX_HARTS) return;
    SchedLock lock;
    TCB_Rv64* cur = g_current_tcb[hart];
    TCB_Rv64* next = ready_dequeue_locked(hart);
    if (!next) {
        // No other runnable: stay with cur (or idle if cur is null).
        if (!cur) g_current_tcb[hart] = g_idle[hart];
        return;
    }
    // Re-queue the outgoing thread behind the others, unless it's idle
    // (idle is never on the ready queue).
    if (cur && cur != g_idle[hart]) {
        ready_enqueue_locked(cur);
    }
    g_current_tcb[hart] = next;
}

// Cooperative yield from a thread context.  Saves via
// cpu_context_switch_rv64 rather than trap-frame copy.
extern "C" void rv64_yield(uint32_t hart) {
    if (hart >= MAX_HARTS) return;
    TCB_Rv64* cur = nullptr;
    TCB_Rv64* next = nullptr;
    {
        SchedLock lock;
        cur = g_current_tcb[hart];
        next = ready_dequeue_locked(hart);
        if (!next) {
            // Nothing else runnable — just return.
            return;
        }
        if (cur && cur != g_idle[hart]) ready_enqueue_locked(cur);
        g_current_tcb[hart] = next;
    }
    cpu_context_switch_rv64(cur, next);
}

} // namespace hal::qemu_virt_rv64
