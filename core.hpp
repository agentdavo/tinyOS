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
constexpr size_t MAX_THREADS = 16;
constexpr size_t MAX_NAME_LENGTH = 32;
constexpr size_t MAX_CORES = 4;
constexpr size_t MAX_PRIORITY_LEVELS = 16;
constexpr size_t TRACE_BUFFER_SIZE = 1024; 
constexpr size_t DEFAULT_STACK_SIZE = 4096;
constexpr size_t MAX_SOFTWARE_TIMERS = 64;
constexpr size_t MAX_LOCKS = 32; 
constexpr size_t NET_MAX_PACKET_SIZE = 1500; 
constexpr size_t MAX_AUDIO_CHANNELS = 2; 

// Forward declarations
struct PerCPUData; // Forward declaration

class Spinlock {
    std::atomic<bool> lock_flag_{false};
    uint32_t lock_id_;
    static uint32_t next_lock_id_; 
public:
    Spinlock();
    void acquire_isr_safe();
    void release_isr_safe();
    void acquire_general();
    void release_general();
    uint32_t get_id() const noexcept { return lock_id_; }
};

class ScopedLock {
    Spinlock& lock_;
public:
    explicit ScopedLock(Spinlock& l) : lock_(l) { lock_.acquire_general(); }
    ~ScopedLock() { lock_.release_general(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};

class ScopedISRLock {
    Spinlock& lock_;
public:
    explicit ScopedISRLock(Spinlock& l) : lock_(l) { lock_.acquire_isr_safe(); }
    ~ScopedISRLock() { lock_.release_isr_safe(); }
    ScopedISRLock(const ScopedISRLock&) = delete;
    ScopedISRLock& operator=(const ScopedISRLock&) = delete;
};

class FixedMemoryPool {
    struct Block { Block* next; };
    Block* free_head_ = nullptr;
    uint8_t* pool_memory_start_ = nullptr;
    size_t block_storage_size_ = 0; 
    size_t header_actual_size_ = 0; 
    size_t user_data_offset_ = 0;   
    size_t num_total_blocks_ = 0;
    size_t num_free_blocks_ = 0;
    Spinlock pool_lock_;
public:
    FixedMemoryPool() = default;
    bool init(void* base, size_t num_blocks, size_t blk_sz_user, size_t align_user_data);
    void* allocate();
    void free_block(void* ptr);
    size_t get_free_count() const noexcept { ScopedLock lock(const_cast<Spinlock&>(pool_lock_)); return num_free_blocks_; }
    size_t get_total_count() const noexcept { return num_total_blocks_; }
};

template<typename T, size_t Capacity>
class SPSCQueue {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");
    std::array<T*, Capacity> items_;
    alignas(64) std::atomic<size_t> head_{0}; 
    alignas(64) std::atomic<size_t> tail_{0};
public:
    SPSCQueue() = default;
    bool enqueue(T* item) noexcept;
    T* dequeue() noexcept;
    bool is_empty() const noexcept { return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire); }
    bool is_full() const noexcept {
        size_t next_tail = (tail_.load(std::memory_order_acquire) + 1) & (Capacity - 1);
        return next_tail == head_.load(std::memory_order_acquire);
    }
    size_t count() const noexcept {
        return (tail_.load(std::memory_order_relaxed) - head_.load(std::memory_order_relaxed) + Capacity) & (Capacity - 1);
    }
};

struct TCB {
    uint64_t regs[31]; 
    uint64_t sp;       
    uint64_t pc;       
    uint64_t pstate;   
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
    std::atomic<bool> event_flag{false}; 
    uint64_t deadline_us = 0;   
};

struct TraceEntry { 
    uint64_t timestamp_us;
    uint32_t core_id;
    const char* event_str; 
    uintptr_t arg1, arg2;
};

struct PerCPUData {
    TCB* current_thread = nullptr; 
    TCB* idle_thread = nullptr;    
};

// Declare g_per_cpu_data after PerCPUData definition
alignas(64) extern std::array<PerCPUData, MAX_CORES> g_per_cpu_data;

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
    void signal_event_isr(TCB* tcb);             
    void wait_for_event(TCB* tcb);               
    size_t get_num_active_tasks() const { return num_active_tasks_.load(std::memory_order_relaxed); }
    Spinlock& get_global_scheduler_lock() { return scheduler_global_lock_; }
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

// Expose g_per_cpu_data with C linkage for assembly access
extern "C" {
    extern kernel::core::PerCPUData kernel_g_per_cpu_data[4];
}

extern "C" void early_uart_puts(const char* str);

#endif // CORE_HPP