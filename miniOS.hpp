// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file miniOS.hpp
 * @brief Core kernel header for miniOS v1.7, a lightweight, portable RTOS in C++17 with C++20 features.
 * @details
 * Defines the miniOS kernel, including scheduler, HAL interfaces, synchronization primitives, and
 * subsystem integration (CLI, DSP, audio, tracing, RAMFS, networking, GPIO). Supports SMP with EDF
 * scheduling, thread management, and v1.7 enhancements: improved error handling, clearer documentation,
 * and developer-friendly APIs. Renamed from miniOS_v1.6.hpp for simplicity.
 *
 * New in v1.7:
 * - Renamed from miniOS_v1.6.hpp
 * - Enhanced error handling and diagnostics
 * - Improved Doxygen comments and code clarity
 * - Consistent use of std::string_view and std::span
 *
 * @note Hardware-agnostic; platform-specific code in HALs.
 * @version 1.7
 * @see miniOS.cpp, util.hpp, cli.hpp, dsp.hpp, audio.hpp, trace.hpp, fs.hpp, net.hpp, gpio.hpp
 */

#ifndef MINIOS_HPP
#define MINIOS_HPP

// Standard includes (global scope, no project namespaces)
#include <cstdint>
#include <cstddef>
#include <span>
#include <string_view>
#include <optional>
#include <atomic>
#include <array>
#include <concepts>


// Global constants needed by subsystem headers before kernel namespace
constexpr size_t MAX_THREADS = 16;             // Used by miniOS.cpp, potentially others
constexpr size_t MAX_NAME_LENGTH = 32;         // Used by TCB
constexpr size_t MAX_CORES = 4;                // Used by miniOS.cpp, Scheduler
constexpr size_t MAX_PRIORITY_LEVELS = 16;     // Used by Scheduler
constexpr size_t TRACE_BUFFER_SIZE = 1024;     // Used by miniOS.cpp
constexpr size_t DEFAULT_STACK_SIZE = 4096;    // Used by miniOS.cpp
constexpr size_t MAX_SOFTWARE_TIMERS = 64;     // Used by miniOS.cpp
constexpr size_t MAX_LOCKS = 32;               // Not directly used by externals?
constexpr size_t NET_MAX_PACKET_SIZE = 1500;   // Used by net.hpp
constexpr size_t GPIO_BANKS = 4;               // Used by gpio.hpp
constexpr size_t GPIO_PINS_PER_BANK = 64;      // Used by gpio.hpp

// Forward declarations for types used by HAL from other namespaces
namespace audio { class AudioBuffer; } // Used by hal::i2s

namespace kernel {

/**
 * @brief Synchronization spinlock.
 */
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

/**
 * @brief RAII lock for general use.
 */
class ScopedLock {
    Spinlock& lock_;
public:
    explicit ScopedLock(Spinlock& l) : lock_(l) { lock_.acquire_general(); }
    ~ScopedLock() { lock_.release_general(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};

/**
 * @brief RAII lock for ISR use.
 */
class ScopedISRLock {
    Spinlock& lock_;
public:
    explicit ScopedISRLock(Spinlock& l) : lock_(l) { lock_.acquire_isr_safe(); }
    ~ScopedISRLock() { lock_.release_isr_safe(); }
    ScopedISRLock(const ScopedISRLock&) = delete;
    ScopedISRLock& operator=(const ScopedISRLock&) = delete;
};

/**
 * @brief Fixed-size memory pool.
 */
class FixedMemoryPool {
    struct Block { Block* next; };
    Block* free_head_ = nullptr;
    uint8_t* pool_memory_start_ = nullptr;
    size_t block_storage_size_ = 0;
    size_t header_actual_size_ = 0;
    size_t num_total_blocks_ = 0;
    size_t num_free_blocks_ = 0;
    Spinlock pool_lock_;
public:
    bool init(void* base, size_t num, size_t blk_sz_user, size_t align_user_data);
    void* allocate();
    void free_block(void* ptr);
    size_t get_free_count() const noexcept { return num_free_blocks_; }
};

/**
 * @brief Single-producer single-consumer queue.
 */
template<typename T, size_t Capacity>
class SPSCQueue {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");
    std::array<T*, Capacity> items_;
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
public:
    bool enqueue(T* item) noexcept;
    T* dequeue() noexcept;
};

using SPSCQueue_AudioBuffer = SPSCQueue<audio::AudioBuffer, 16>;

/**
 * @brief Thread Control Block (TCB).
 */
struct TCB {
    uint64_t regs[31]; ///< General-purpose registers
    uint64_t sp; ///< Stack pointer
    uint64_t pc; ///< Program counter
    uint64_t status; ///< Thread status
    void (*entry_point)(void*); ///< Thread entry function
    void* arg_ptr; ///< Entry argument
    uint8_t* stack_base; ///< Stack base address
    size_t stack_size; ///< Stack size
    enum class State { INACTIVE, READY, RUNNING, BLOCKED } state = State::INACTIVE;
    int priority; ///< Priority (0 to MAX_PRIORITY_LEVELS-1)
    int core_affinity = -1; ///< Core affinity (-1 for any)
    uint32_t cpu_id_running_on = static_cast<uint32_t>(-1); ///< Current CPU ID
    char name[MAX_NAME_LENGTH]; ///< Thread name
    TCB* next_in_q = nullptr; ///< Next TCB in queue
    bool event_flag = false; ///< Event flag
    uint64_t deadline_us = 0; ///< EDF deadline (microseconds)
};

/**
 * @brief Trace entry for debugging.
 */
struct TraceEntry {
    uint64_t timestamp_us; ///< Event timestamp
    uint32_t core_id; ///< Core ID
    const char* event_str; ///< Event description
    uintptr_t arg1, arg2; ///< Event arguments
};

/**
 * @brief Per-CPU data.
 */
struct PerCPUData {
    TCB* current_thread = nullptr; ///< Current thread
    TCB* idle_thread = nullptr; ///< Idle thread
};

/**
 * @brief Scheduler policy interface.
 */
class SchedulerPolicy {
public:
    virtual TCB* select_next_task(uint32_t core_id, TCB* current) = 0;
    virtual void add_to_ready_queue(TCB* tcb, uint32_t core_id) = 0;
    virtual ~SchedulerPolicy() = default;
};

/**
 * @brief Earliest Deadline First (EDF) scheduler policy.
 */
class EDFPolicy : public SchedulerPolicy {
public:
    TCB* select_next_task(uint32_t core_id, TCB* current) override;
    void add_to_ready_queue(TCB* tcb, uint32_t core_id) override;
};

/**
 * @brief Scheduler class.
 */
class Scheduler {
    std::array<std::array<TCB*, MAX_PRIORITY_LEVELS>, MAX_CORES> ready_qs_ = {};
    size_t num_active_tasks_ = 0;
    Spinlock scheduler_global_lock_;
    std::array<Spinlock, MAX_CORES> per_core_locks_;
    SchedulerPolicy* policy_ = nullptr;
	
public:
    Scheduler();
    void set_policy(SchedulerPolicy* p) noexcept { policy_ = p; }
    TCB* create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle = false, uint64_t deadline_us = 0);
    void start_core_scheduler(uint32_t core_id);
    void preemptive_tick(uint32_t core_id);
    void yield(uint32_t core_id);
    void signal_event_isr(TCB* tcb);
    void wait_for_event(TCB* tcb);
	size_t get_num_active_tasks() const { return num_active_tasks_; }
    Spinlock& get_global_scheduler_lock() { return scheduler_global_lock_; } // Non-const reference
	
// Expose for EDFPolicy, or make EDFPolicy a friend, or pass ready_qs_ by reference
friend class EDFPolicy;

private:
    TCB* pop_highest_priority_ready_task(uint32_t current_core_id);
    void schedule(uint32_t core_id, bool is_preemption);
    static void idle_thread_func(void* arg);
    static void thread_bootstrap(TCB* self);
};

/**
 * @brief HAL synchronization utilities.
 */
namespace hal {
    namespace sync {
        void barrier_dmb();
        void barrier_dsb();
        void barrier_isb();
        template<typename T> T atomic_load_acquire(const std::atomic<T>* addr);
        template<typename T> void atomic_store_release(std::atomic<T>* addr, T val);
        template<typename T> bool atomic_compare_exchange_strong(std::atomic<T>* addr, T& expected, T desired);
        template<typename T> T atomic_fetch_add(std::atomic<T>* addr, T val);
    }

/**
 * @brief Memory operations.
 */
struct MemoryOps {
    virtual void flush_cache_range(const void* addr, size_t size) = 0;
    virtual void invalidate_cache_range(const void* addr, size_t size) = 0;
    virtual ~MemoryOps() = default;
};

/**
 * @brief UART driver operations.
 */
struct UARTDriverOps {
    virtual void putc(char c) = 0;
    virtual void puts(const char* str) = 0;
    virtual void uart_put_uint64_hex(uint64_t value) = 0;
    virtual char getc_blocking() = 0;
    virtual ~UARTDriverOps() = default;
};

/**
 * @brief IRQ controller operations.
 */
struct IRQControllerOps {
    virtual void enable_core_irqs(uint32_t core_id, uint32_t source) = 0;
    virtual void disable_core_irqs(uint32_t core_id) = 0;
    virtual void init_distributor() = 0;
    virtual void init_cpu_interface() = 0;
    virtual uint32_t ack_irq(uint32_t core_id) = 0;
    virtual void end_irq(uint32_t core_id, uint32_t source) = 0;
    virtual void enable_irq_line(uint32_t source) = 0;
    virtual void set_irq_priority(uint32_t source, uint8_t priority) = 0;
    virtual ~IRQControllerOps() = default;
};

/**
 * @brief DMA controller operations.
 */
namespace dma {
    using ChannelID = int32_t;
    constexpr ChannelID INVALID_CHANNEL = -1;
    enum class Direction { MEM_TO_PERIPH, PERIPH_TO_MEM };
    struct TransferConfig {
        uintptr_t src, dst;
        size_t size_bytes;
        Direction dir;
        bool src_inc, dst_inc;
        size_t burst_size_bytes;
    };
    using DMACallback = void (*)(ChannelID, bool success, void* context);
}
struct DMAControllerOps {
    virtual dma::ChannelID request_channel() = 0;
    virtual bool configure_and_start_transfer(dma::ChannelID channel, const dma::TransferConfig& cfg, dma::DMACallback cb, void* context) = 0;
    virtual ~DMAControllerOps() = default;
};

/**
 * @brief I2S driver operations.
 */
namespace i2s {
    enum class Mode { MASTER_TX, MASTER_RX };
    enum class BitDepth { BITS_16, BITS_24_IN_32, BITS_32 };
    struct Format {
        uint32_t sample_rate_hz;
        uint8_t channels;
        BitDepth bit_depth;
        size_t get_bytes_per_frame() const noexcept;
    };
    using I2SCallback = void (*)(uint32_t instance_id, audio::AudioBuffer* buffer, Mode mode, void* user_data);
}
struct I2SDriverOps {
    virtual bool init(uint32_t instance_id, i2s::Mode mode, const i2s::Format& format, size_t samples_per_block, uint8_t num_buffers, i2s::I2SCallback cb, void* user_data) = 0;
    virtual bool start(uint32_t instance_id) = 0;
    virtual audio::AudioBuffer* get_buffer_for_app_tx(uint32_t instance_id) = 0;
    virtual bool submit_filled_buffer_to_hw_tx(uint32_t instance_id, audio::AudioBuffer* buffer) = 0;
    virtual void release_processed_buffer_to_hw_rx(uint32_t instance_id, audio::AudioBuffer* buffer) = 0;
    virtual void convert_hw_format_to_dsp_format(const audio::AudioBuffer* buffer, const i2s::Format& format) = 0;
    virtual void convert_dsp_format_to_hw_format(audio::AudioBuffer* buffer, const i2s::Format& format) = 0;
    virtual ~I2SDriverOps() = default;
};

/**
 * @brief Timer driver operations.
 */
namespace timer {
    struct SoftwareTimer; // Forward declaration
    using software_timer_callback_t = void (*)(SoftwareTimer*, void*);
    struct SoftwareTimer {
        uint64_t expiry_time_us;
        uint64_t period_us;
        software_timer_callback_t callback;
        void* context;
        bool active;
        SoftwareTimer* next;
    };
}
struct TimerDriverOps {
    virtual void init_system_timer_properties(uint64_t freq_hz_override = 0) = 0;
    virtual void init_core_timer_interrupt() = 0;
    virtual void ack_core_timer_interrupt() = 0;
    virtual bool add_software_timer(timer::SoftwareTimer* timer) = 0;
    virtual bool remove_software_timer(timer::SoftwareTimer* timer) = 0;
    virtual uint64_t get_system_time_us() = 0;
    virtual void hardware_timer_irq_fired(uint32_t core_id) = 0;
    virtual ~TimerDriverOps() = default;
};

/**
 * @brief Network driver operations.
 */
namespace net {
    struct NetworkDriverOps; // Forward declaration, actual definition in net.hpp
}

/**
 * @brief GPIO driver operations.
 */
namespace gpio {
    enum class PinMode { INPUT, OUTPUT }; // Defined here for Platform visibility
    enum class PinState { LOW, HIGH };   // Defined here for Platform visibility
    struct GPIODriverOps; // Forward declaration, actual definition in gpio.hpp
}

/**
 * @brief Power operations.
 */
struct PowerOps {
    virtual void enter_idle_state(uint32_t core_id) = 0;
    virtual bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) = 0;
    virtual ~PowerOps() = default;
};

/**
 * @brief Watchdog operations.
 */
struct WatchdogOps {
    virtual bool start_watchdog(uint32_t timeout_ms) = 0;
    virtual bool reset_watchdog() = 0;
    virtual ~WatchdogOps() = default;
};

/**
 * @brief Platform interface.
 */
class Platform {
public:
    virtual uint32_t get_core_id() const = 0;
    virtual UARTDriverOps* get_uart_ops() = 0;
    virtual IRQControllerOps* get_irq_ops() = 0;
    virtual TimerDriverOps* get_timer_ops() = 0;
    virtual DMAControllerOps* get_dma_ops() = 0;
    virtual I2SDriverOps* get_i2s_ops() = 0;
    virtual MemoryOps* get_mem_ops() = 0;
    virtual net::NetworkDriverOps* get_net_ops() = 0;
    virtual PowerOps* get_power_ops() = 0;
    virtual gpio::GPIODriverOps* get_gpio_ops() = 0;
    virtual WatchdogOps* get_watchdog_ops() = 0;
    virtual void early_init() = 0;
    virtual void core_early_init() = 0;
    virtual void panic(const char* msg, const char* file, int line) = 0;
    virtual ~Platform() = default;
};
} // namespace hal

// Global instances
extern hal::Platform* g_platform; // Changed to kernel::hal::Platform
extern Scheduler* g_scheduler_ptr;
extern FixedMemoryPool g_audio_pool;
extern uint8_t g_audio_pool_mem[64 * 1024];
extern FixedMemoryPool g_software_timer_obj_pool;
extern uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(hal::timer::SoftwareTimer)]; // Qualified SoftwareTimer

void trace_event(const char* event, uintptr_t a1, uintptr_t a2 = 0);
void dump_trace_buffer(hal::UARTDriverOps* uart_ops);
void set_power_mode(bool low_power);
void configure_memory_protection(TCB* tcb, bool enable);
void get_kernel_stats(hal::UARTDriverOps* uart_ops);
} // namespace kernel

// Subsystem includes (after kernel namespace and HAL are defined)
#include "util.hpp"
#include "audio.hpp"   // AudioSystem uses dsp::DSPGraph, kernel::hal::i2s, kernel::FixedMemoryPool
#include "cli.hpp"     // CLI uses kernel::hal::UARTDriverOps
#include "dsp.hpp"     // DSPNode uses kernel::hal::UARTDriverOps, net::IPv4Addr
#include "fs.hpp"      // FileSystem uses kernel::hal::UARTDriverOps
#include "gpio.hpp"    // GPIOManager uses kernel::hal::gpio::GPIODriverOps, kernel::Spinlock, kernel::TCB
#include "net.hpp"     // NetManager uses kernel::hal::net::NetworkDriverOps, kernel::hal::UARTDriverOps
#include "trace.hpp"   // TraceManager uses kernel::TCB, kernel::hal::UARTDriverOps

namespace kernel {
    // Extern declarations for subsystem managers, now that their types are defined
    extern audio::AudioSystem g_audio_system;
    extern trace::TraceManager g_trace_manager;
    extern fs::FileSystem g_file_system;
    extern net::NetManager g_net_manager;
    extern gpio::GPIOManager g_gpio_manager;
} // namespace kernel

#endif // MINIOS_HPP