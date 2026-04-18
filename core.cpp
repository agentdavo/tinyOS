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
#include "cli.hpp"
#include "ethercat/master.hpp"
#include "ethercat/bus_config.hpp"
#if MINIOS_FAKE_SLAVE
#include "ethercat/fake_slave.hpp"
#endif
#include "motion/motion.hpp"
#include "devices/embedded.hpp"
#include "devices/device_db.hpp"
#include "rt/base_thread.hpp"
#include "ui/splash.hpp"
#include "ui/fb.hpp"
#include "diag/cpu_load.hpp"
#include "render/benchmark.hpp"
#include "render/gles1.hpp"
#include "render/machine_model.hpp"

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
static inline uint64_t minios_irq_save_disable() noexcept {
    uint64_t f;
    asm volatile("csrrci %0, sstatus, 0x2" : "=r"(f) :: "memory");
    return f;
}
static inline void minios_irq_restore(uint64_t f) noexcept {
    asm volatile("csrw sstatus, %0" :: "r"(f) : "memory");
}
static inline void minios_irq_enable() noexcept {
    asm volatile("csrsi sstatus, 0x2");
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

// Benchmark runner for 1000 spinning cubes
void run_benchmark_test() {
    using namespace kernel;
    using namespace render;
    
    auto* uart = g_platform ? g_platform->get_uart_ops() : nullptr;
    if (!uart) return;
    
    auto& fb = ui::framebuffer();
    
    static gles1::Renderer renderer;
    static bool cube_created = false;
    static machine::MeshPart cube_part;
    static uint64_t start_ticks = 0;
    static uint64_t frame_start = 0;
    static bool g_dumped_test_pattern = false;
    
    if (!cube_created) {
        machine::create_cube(cube_part, 1.0f, 1.0f, 1.0f, {255, 128, 64, 255});
        cube_created = true;
        benchmark::init();
        
        auto* timer = g_platform->get_timer_ops();
        start_ticks = timer ? timer->get_system_time_ns() / 1000000ULL : 0;
        frame_start = start_ticks;
        
        char initbuf[48];
        kernel::util::k_snprintf(initbuf, sizeof(initbuf), "[bench] cube benchmark initialized\n");
        uart->puts(initbuf);
    }
    
    auto* timer = g_platform->get_timer_ops();
    uint64_t now = timer ? timer->get_system_time_ns() / 1000000ULL : 0;
    uint32_t elapsed = static_cast<uint32_t>(now - start_ticks);
    
    gles1::FramebufferView fb_view{
        fb.data(),
        fb.width(),
        fb.height(),
        fb.width()
    };
    renderer.bind_framebuffer(fb_view);
    renderer.clear(0xFF000000);  // Black
    
    const uint32_t center_idx = (1920/2) * 1080 + (1080/2);
    fb.data()[center_idx] = 0xFF00FF00;  // Green test pixel
    fb.data()[center_idx + 1] = 0xFF00FF00;
    fb.data()[center_idx + 1080] = 0xFF00FF00;
    fb.data()[center_idx - 1] = 0xFF00FF00;
    fb.data()[center_idx - 1080] = 0xFF00FF00;
    
    gles1::Mat4 proj = gles1::make_perspective(1.2f, 0.5625f, 0.1f, 100.0f);
    gles1::Mat4 view = gles1::make_look_at({0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    renderer.set_projection_matrix(proj);
    renderer.set_view_matrix(view);
    
    gles1::Light light{
        {0.0f, 0.0f, 1.0f},
        {0.3f, 0.3f, 0.3f, 1.0f},
        {0.7f, 0.7f, 0.7f, 1.0f},
        {0.5f, 0.5f, 0.5f, 1.0f}
    };
    renderer.set_light(light);
    
    gles1::Material mat;
    mat.ambient = {0.2f, 0.2f, 0.2f, 1.0f};
    mat.diffuse = {1.0f, 0.0f, 0.0f, 1.0f};
    mat.specular = {0.5f, 0.5f, 0.5f, 1.0f};
    mat.shininess = 32.0f;
    renderer.set_material(mat);
    
    gles1::MeshView cube_mesh{
        cube_part.vertices,
        cube_part.vertex_count,
        cube_part.indices,
        cube_part.index_count
    };
    
    float time = static_cast<float>(elapsed) * 0.01f;
    gles1::Mat4 test_model = gles1::make_rotation_y(time) ;
    test_model = gles1::multiply(test_model, gles1::make_rotation_x(time * 0.7f));
    test_model = gles1::multiply(test_model, gles1::make_translation(0.0f, 0.0f, -3.0f));
    renderer.set_model_matrix(test_model);
    mat.diffuse = {1.0f, 0.3f, 0.0f, 1.0f};
    renderer.set_material(mat);
    bool drawn = renderer.draw_mesh_solid(cube_mesh);
    char dbg[48];
    kernel::util::k_snprintf(dbg, sizeof(dbg), "[bench] gles1 cube drawn=%d\n", drawn ? 1 : 0);
    uart->puts(dbg);
    
    benchmark::run(renderer, cube_mesh, elapsed);
    
    if (cube_created && !g_dumped_test_pattern) {
        constexpr int TEST_W = 32;
        constexpr int TEST_H = 32;
        constexpr int center_x = (1080 - TEST_W) / 2;
        constexpr int center_y = (1920 - TEST_H) / 2;
        
        uart->puts("[bench] test pattern (32x32 center):\n");
        for (int y = 0; y < TEST_H; y += 4) {
            char line[128];
            char* p = line;
            for (int x = 0; x < TEST_W; x += 4) {
                uint32_t px = fb.data()[(center_y + y) * 1080 + (center_x + x)];
                p += kernel::util::k_snprintf(p, 8, "%08X ", px);
            }
            *p++ = '\n';
            *p = '\0';
            uart->puts(line);
        }
        g_dumped_test_pattern = true;
    }
    
    uint64_t frames = benchmark::get_frame_count();
    if (frames > 0 && frames % 60 == 0) {
        uint64_t now2 = timer ? timer->get_system_time_ns() / 1000000ULL : 0;
        uint64_t duration = now2 - frame_start;
        if (duration > 0) {
            uint32_t fps = static_cast<uint32_t>((frames * 1000) / duration);
            char buf[64];
            kernel::util::k_snprintf(buf, sizeof(buf), "[bench] %llu frames, %u fps\n", 
                static_cast<unsigned long long>(frames), fps);
            uart->puts(buf);
            frame_start = now2;
        }
    }
}


alignas(64) uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(kernel::hal::timer::SoftwareTimer)];
FixedMemoryPool g_software_timer_obj_pool;
std::array<TraceEntry, TRACE_BUFFER_SIZE> g_trace_buffer;
std::array<std::atomic<size_t>, MAX_CORES> g_trace_overflow_count{}; 
alignas(16) std::array<std::array<uint8_t, DEFAULT_STACK_SIZE>, MAX_THREADS> g_task_stacks;
std::array<TCB, MAX_THREADS> g_task_tcbs;
alignas(64) std::array<PerCPUData, MAX_CORES> g_per_cpu_data;
uint32_t Spinlock::next_lock_id_ = 0;

Spinlock::Spinlock() noexcept : lock_id_(next_lock_id_++) {}
void Spinlock::acquire_isr_safe() noexcept {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {
            MINIOS_CPU_RELAX();
        }
    }
}
void Spinlock::release_isr_safe() noexcept { lock_flag_.store(false, std::memory_order_release); }
void Spinlock::acquire_general() noexcept {
    bool expected = false;
    while (!lock_flag_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
        expected = false;
        while (lock_flag_.load(std::memory_order_relaxed)) {
            MINIOS_CPU_RELAX();
        }
    }
}
void Spinlock::release_general() noexcept { lock_flag_.store(false, std::memory_order_release); }

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
    // Double-free guard: walk the free list; if this block is already on it, panic
    // rather than corrupt the list.
    for (Block* b = free_head_; b; b = b->next) {
        if (b == block_header_raw) {
            if (kernel::g_platform) kernel::g_platform->panic("FixedMemoryPool: double free", __FILE__, __LINE__);
            for (;;) asm volatile("wfi");
        }
    }
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

TCB* EDFPolicy::select_next_task(uint32_t core_id, TCB* current_task) {
    if (core_id >= MAX_CORES || !kernel::g_scheduler_ptr) return nullptr;
    Scheduler* sched = kernel::g_scheduler_ptr;
    ScopedLock lock(sched->per_core_locks_[core_id]);
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
                    if (task_iter->deadline_us > 0 && task_iter->deadline_us < earliest_deadline) {
                        earliest_deadline = task_iter->deadline_us; earliest_task = task_iter;
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
    ScopedLock lock(sched->per_core_locks_[core_id]);
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
    TCB* worker = pop_highest_priority_ready_task(core_id);
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
    // Dedicated-core tickless mode (cores 2/3 on arm64): the scheduler tick is
    // disabled for this core (see TimerDriver::init_core_timer_interrupt +
    // hal_irq_handler). The GIC IRQ line is still enabled — wait_wfi_until_ns
    // uses it to wake from WFI.
    if (kernel::hal::is_dedicated_rt_core(core_id)) {
        kernel::g_platform->get_irq_ops()->enable_irq_line(kernel::hal::SYSTEM_TIMER_IRQ);
        kernel::g_platform->get_irq_ops()->enable_core_irqs(core_id, 0x1);
        return;
    }
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
    if (core_id >= MAX_CORES) return;
    schedule(core_id, false);
}

void Scheduler::signal_event_isr(TCB* tcb) {
    if (!tcb) return;
    tcb->event_flag.exchange(true, std::memory_order_release);
    // Ensure the flag write is visible to waiters on other cores before we leave ISR.
    kernel::hal::sync::barrier_dsb();
}
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

} // namespace kernel
