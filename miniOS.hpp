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
constexpr size_t MAX_THREADS = 16;
constexpr size_t MAX_NAME_LENGTH = 32;
constexpr size_t MAX_CORES = 4;
constexpr size_t MAX_PRIORITY_LEVELS = 16;
constexpr size_t TRACE_BUFFER_SIZE = 1024;
constexpr size_t DEFAULT_STACK_SIZE = 4096;
constexpr size_t MAX_SOFTWARE_TIMERS = 64;
constexpr size_t MAX_LOCKS = 32;
constexpr size_t NET_MAX_PACKET_SIZE = 1500;
constexpr size_t GPIO_BANKS = 4;
constexpr size_t GPIO_PINS_PER_BANK = 64;

// Forward declarations for types used by HAL from other namespaces
namespace audio { class AudioBuffer; } // Used by hal::i2s
namespace kernel { namespace dsp { class DSPGraph; } } // Used by audio::AudioSystem

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
    FixedMemoryPool() = default; // Default constructor
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
    SPSCQueue() = default; // Default constructor
    bool enqueue(T* item) noexcept;
    T* dequeue() noexcept;
    bool is_empty() const noexcept { return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire); }
    bool is_full() const noexcept {
        size_t next_tail = (tail_.load(std::memory_order_acquire) + 1) & (Capacity - 1);
        return next_tail == head_.load(std::memory_order_acquire);
    }
};

// Forward declare audio::AudioBuffer if SPSCQueue_AudioBuffer is used widely before audio.hpp
// Already done globally.
using SPSCQueue_AudioBuffer = SPSCQueue<audio::AudioBuffer, 16>;


/**
 * @brief Thread Control Block (TCB).
 */
struct TCB {
    uint64_t regs[31]; ///< General-purpose registers
    uint64_t sp; ///< Stack pointer
    uint64_t pc; ///< Program counter
    uint64_t pstate; ///< Process State Register (e.g., CPSR/PSTATE for ARM)
    void (*entry_point)(void*); ///< Thread entry function
    void* arg_ptr; ///< Entry argument
    uint8_t* stack_base; ///< Stack base address
    size_t stack_size; ///< Stack size
    enum class State { INACTIVE, READY, RUNNING, BLOCKED, ZOMBIE } state = State::INACTIVE;
    int priority; ///< Priority (0 to MAX_PRIORITY_LEVELS-1)
    int core_affinity = -1; ///< Core affinity (-1 for any)
    uint32_t cpu_id_running_on = static_cast<uint32_t>(-1); ///< Current CPU ID
    char name[MAX_NAME_LENGTH]; ///< Thread name
    TCB* next_in_q = nullptr; ///< Next TCB in queue
    std::atomic<bool> event_flag{false}; ///< Event flag (atomic for safe ISR signalling)
    uint64_t deadline_us = 0; ///< EDF deadline (microseconds)
    // Add fields for MPU context if applicable
};

/**
 * @brief Trace entry for debugging.
 */
struct TraceEntry {
    uint64_t timestamp_us; ///< Event timestamp
    uint32_t core_id; ///< Core ID
    const char* event_str; ///< Event description (must be static string literal)
    uintptr_t arg1, arg2; ///< Event arguments
};

/**
 * @brief Per-CPU data.
 */
struct PerCPUData {
    TCB* current_thread = nullptr; ///< Current thread
    TCB* idle_thread = nullptr; ///< Idle thread
    // uint64_t last_schedule_time_us = 0; // Example for preemption logic
};

/**
 * @brief Scheduler policy interface.
 */
class SchedulerPolicy {
public:
    virtual ~SchedulerPolicy() = default;
    virtual TCB* select_next_task(uint32_t core_id, TCB* current_task) = 0;
    virtual void add_to_ready_queue(TCB* tcb, uint32_t core_id) = 0;
    // virtual void remove_from_ready_queue(TCB* tcb, uint32_t core_id) = 0; // Might be useful
};

/**
 * @brief Earliest Deadline First (EDF) scheduler policy.
 */
class EDFPolicy : public SchedulerPolicy {
public:
    TCB* select_next_task(uint32_t core_id, TCB* current_task) override;
    void add_to_ready_queue(TCB* tcb, uint32_t core_id) override;
};

/**
 * @brief Scheduler class.
 */
class Scheduler {
    std::array<std::array<TCB*, MAX_PRIORITY_LEVELS>, MAX_CORES> ready_qs_ = {};
    std::atomic<size_t> num_active_tasks_{0}; // Atomic for safety if read outside lock
    Spinlock scheduler_global_lock_; // For global scheduler operations like thread creation
    std::array<Spinlock, MAX_CORES> per_core_locks_; // For per-core ready queues
    SchedulerPolicy* policy_ = nullptr;
public:
    Scheduler();
    ~Scheduler(); // To delete policy_

    void set_policy(SchedulerPolicy* p) noexcept { policy_ = p; } // Be careful with ownership if policy_ is new'd
    TCB* create_thread(void (*fn)(void*), const void* arg, int prio, int affinity, const char* name, bool is_idle = false, uint64_t deadline_us = 0);
    void start_core_scheduler(uint32_t core_id); // Starts scheduling on a core
    void preemptive_tick(uint32_t core_id); // Called by timer ISR to trigger preemption
    void yield(uint32_t core_id); // Voluntary yield by current thread
    void signal_event_isr(TCB* tcb); // Signal an event to a waiting thread (ISR safe)
    void wait_for_event(TCB* tcb);   // Thread waits for an event

    size_t get_num_active_tasks() const { return num_active_tasks_.load(std::memory_order_relaxed); }
    Spinlock& get_global_scheduler_lock() { return scheduler_global_lock_; }


friend class EDFPolicy; // Allows EDFPolicy to access ready_qs_ and per_core_locks_
// friend void get_kernel_stats(kernel::hal::UARTDriverOps* uart_ops); // If direct access needed

private:
    TCB* pop_highest_priority_ready_task(uint32_t current_core_id); // Internal helper
    void schedule(uint32_t core_id, bool is_preemption); // Core scheduling logic
    static void idle_thread_func(void* arg); // Entry point for idle threads
    static void thread_bootstrap(TCB* self); // Wrapper to start and clean up threads
};

/**
 * @brief HAL (Hardware Abstraction Layer) namespace.
 */
namespace hal {
    /**
     * @brief HAL synchronization utilities.
     */
    namespace sync {
        void barrier_dmb(); ///< Data Memory Barrier
        void barrier_dsb(); ///< Data Synchronization Barrier
        void barrier_isb(); ///< Instruction Synchronization Barrier
        template<typename T> T atomic_load_acquire(const std::atomic<T>* addr);
        template<typename T> void atomic_store_release(std::atomic<T>* addr, T val);
        template<typename T> bool atomic_compare_exchange_strong(std::atomic<T>* addr, T& expected, T desired);
        template<typename T> T atomic_fetch_add(std::atomic<T>* addr, T val);
    }

    /**
     * @brief Memory operations interface for cache management.
     */
    struct MemoryOps {
        virtual ~MemoryOps() = default;
        virtual void flush_cache_range(const void* addr, size_t size) = 0;
        virtual void invalidate_cache_range(const void* addr, size_t size) = 0;
    };

    /**
     * @brief UART driver operations interface.
     */
    struct UARTDriverOps {
        virtual ~UARTDriverOps() = default;
        virtual void putc(char c) = 0;
        virtual void puts(const char* str) = 0;
        virtual void uart_put_uint64_hex(uint64_t value) = 0;
        virtual char getc_blocking() = 0;
        // virtual bool getc_nonblocking(char& c) = 0; // Optional
        // virtual void register_rx_callback(void (*callback)(char c, void* ctx), void* context) = 0; // Optional
    };

    /**
     * @brief IRQ controller operations interface (e.g., GIC for ARM).
     */
    struct IRQControllerOps {
        virtual ~IRQControllerOps() = default;
        virtual void enable_core_irqs(uint32_t core_id, uint32_t irq_source_mask) = 0; // Mask for specific sources or 0 for all
        virtual void disable_core_irqs(uint32_t core_id) = 0; // Disable all on core
        virtual void init_distributor() = 0; // Initialize GIC distributor (once)
        virtual void init_cpu_interface(uint32_t core_id) = 0; // Initialize GIC CPU interface (per core)
        virtual uint32_t ack_irq(uint32_t core_id) = 0; // Acknowledge IRQ, returns IRQ ID
        virtual void end_irq(uint32_t core_id, uint32_t irq_id) = 0; // Signal End of Interrupt
        virtual void enable_irq_line(uint32_t irq_id) = 0; // Enable a specific IRQ line in distributor
        virtual void disable_irq_line(uint32_t irq_id) = 0; // Disable a specific IRQ line
        virtual void set_irq_priority(uint32_t irq_id, uint8_t priority) = 0; // Set priority for an IRQ
        // virtual void set_irq_target_cpus(uint32_t irq_id, uint8_t cpu_mask) = 0; // For SPIs
    };

    /**
     * @brief DMA controller operations namespace and interface.
     */
    namespace dma {
        using ChannelID = int32_t;
        constexpr ChannelID INVALID_CHANNEL = -1;
        enum class Direction { MEM_TO_PERIPH, PERIPH_TO_MEM, MEM_TO_MEM };
        struct TransferConfig {
            uintptr_t src_addr;
            uintptr_t dst_addr;
            size_t size_bytes;
            Direction direction;
            bool src_increment = true;
            bool dst_increment = true;
            size_t burst_size_bytes = 4; // e.g., 4, 8, 16 bytes
            // Add peripheral ID if DMA is peripheral-aware
        };
        using DMACallback = void (*)(ChannelID channel, bool success, void* context);
    } // namespace dma
    struct DMAControllerOps {
        virtual ~DMAControllerOps() = default;
        virtual dma::ChannelID request_channel() = 0;
        virtual void release_channel(dma::ChannelID channel) = 0;
        virtual bool configure_and_start_transfer(dma::ChannelID channel, const dma::TransferConfig& cfg, dma::DMACallback cb, void* context) = 0;
        // virtual bool is_transfer_complete(dma::ChannelID channel) = 0;
    };

    /**
     * @brief I2S driver operations namespace and interface.
     */
    namespace i2s {
        enum class Mode { MASTER_TX, MASTER_RX, SLAVE_TX, SLAVE_RX };
        enum class BitDepth { BITS_16, BITS_24_IN_32, BITS_32 }; // 24-in-32 means 24 valid bits in a 32-bit word
        struct Format {
            uint32_t sample_rate_hz;
            uint8_t num_channels; // e.g., 1 for mono, 2 for stereo
            BitDepth bit_depth;
            // bool is_stereo_TDM_mode; // For multi-channel TDM
            // uint8_t tdm_slots;
            size_t get_bytes_per_sample_per_channel() const noexcept {
                switch(bit_depth) {
                    case BitDepth::BITS_16: return 2;
                    case BitDepth::BITS_24_IN_32: return 4; // Data is in 32-bit container
                    case BitDepth::BITS_32: return 4;
                    default: return 0;
                }
            }
            size_t get_bytes_per_frame() const noexcept {
                return get_bytes_per_sample_per_channel() * num_channels;
            }
        };
        // Callback uses audio::AudioBuffer directly if HAL is designed to work with it.
        // Otherwise, HAL might use raw void* and size, and AudioSystem maps it.
        using I2SCallback = void (*)(uint32_t instance_id, audio::AudioBuffer* buffer, Mode mode, void* user_data);
    } // namespace i2s
    struct I2SDriverOps {
        virtual ~I2SDriverOps() = default;
        virtual bool init(uint32_t instance_id, i2s::Mode mode, const i2s::Format& format, 
                          size_t samples_per_block_per_channel, uint8_t num_dma_buffers, 
                          i2s::I2SCallback cb, void* user_data) = 0;
        virtual bool start(uint32_t instance_id) = 0;
        virtual bool stop(uint32_t instance_id) = 0;
        
        // These methods imply AudioSystem gives its AudioBuffer to HAL for DMA.
        // HAL then calls back with the same AudioBuffer pointer.
        virtual bool submit_filled_buffer_to_hw_tx(uint32_t instance_id, audio::AudioBuffer* buffer_to_send) = 0;
        // For RX, HAL fills a buffer and gives it to AudioSystem via callback.
        // AudioSystem might then need to provide an empty buffer back to HAL.
        // virtual bool submit_empty_buffer_to_hw_rx(uint32_t instance_id, audio::AudioBuffer* empty_buffer) = 0;

        // Conversion utilities (could be part of HAL or AudioSystem)
        // If AudioBuffer contains float, these convert to/from I2S hardware fixed-point format.
        virtual void convert_hw_format_to_dsp_format(const audio::AudioBuffer* hw_buffer_format_info, const i2s::Format& format) = 0;
        virtual void convert_dsp_format_to_hw_format(audio::AudioBuffer* dsp_buffer_to_convert, const i2s::Format& format) = 0;
    };

    /**
     * @brief Timer driver operations namespace and interface.
     */
    namespace timer {
        struct SoftwareTimer; // Forward declaration
        using software_timer_callback_t = void (*)(SoftwareTimer* timer, void* context);
        struct SoftwareTimer {
            uint64_t expiry_time_us;    ///< Absolute time of next expiry (system time).
            uint64_t period_us;         ///< Timer period in microseconds (0 for one-shot).
            software_timer_callback_t callback; ///< Function to call on expiry.
            void* context;              ///< User-defined context for the callback.
            bool active = false;        ///< True if the timer is active.
            SoftwareTimer* next = nullptr; ///< Next timer in a linked list (for driver management).
            uint32_t id = 0;            ///< Optional timer ID.
        };
    } // namespace timer
    struct TimerDriverOps {
        virtual ~TimerDriverOps() = default;
        virtual void init_system_timer_properties(uint64_t freq_hz_override = 0) = 0; // For system-wide tick source
        virtual void init_core_timer_interrupt(uint32_t core_id) = 0; // For per-core scheduler tick
        virtual void ack_core_timer_interrupt(uint32_t core_id) = 0;
        virtual bool add_software_timer(timer::SoftwareTimer* timer) = 0;
        virtual bool remove_software_timer(timer::SoftwareTimer* timer) = 0;
        virtual uint64_t get_system_time_us() = 0;
        virtual void hardware_timer_irq_fired(uint32_t core_id) = 0; // Called by common IRQ handler
    };

    /**
     * @brief Network driver operations namespace and interface.
     */
    namespace net {
        struct NetworkDriverOps; // Forward declaration, actual definition in net.hpp if complex
                                 // Or defined here if simple enough for just Platform.
        struct NetworkDriverOps {
            virtual ~NetworkDriverOps() = default;
            virtual bool init_interface(int if_idx /*, config params */) = 0;
            virtual bool send_packet(int if_idx, const uint8_t* data, size_t len) = 0;
            // Callback for received packets from HAL to NetManager
            using PacketReceivedCallback = void (*)(int if_idx, const uint8_t* data, size_t len, void* context);
            virtual void register_packet_receiver(PacketReceivedCallback cb, void* context) = 0;
            // virtual MacAddr get_mac_address(int if_idx) = 0;
            // virtual IpConfig get_ip_config(int if_idx) = 0;
            // virtual bool set_ip_config(int if_idx, const IpConfig& cfg) = 0;
        };
    } // namespace net

    /**
     * @brief GPIO driver operations namespace and interface.
     */
    namespace gpio {
        enum class PinMode { INPUT, OUTPUT, ALT_FUNC }; // Added ALT_FUNC
        enum class PinState { LOW, HIGH };
        // enum class PinPull { NONE, PULL_UP, PULL_DOWN }; // Optional
        struct GPIODriverOps {
            virtual ~GPIODriverOps() = default;
            virtual bool init_bank(uint32_t bank_id) = 0;
            virtual bool configure_pin(uint32_t bank, uint32_t pin, PinMode mode /*, PinPull pull = PinPull::NONE */) = 0;
            virtual bool set_pin_state(uint32_t bank, uint32_t pin, PinState state) = 0;
            virtual PinState read_pin_state(uint32_t bank, uint32_t pin) = 0;
            // Interrupt related (optional for basic GPIO)
            // enum class InterruptEdge { RISING, FALLING, BOTH };
            // using GPIOIrqCallback = void (*)(uint32_t bank, uint32_t pin, void* context);
            // virtual bool enable_pin_irq(uint32_t bank, uint32_t pin, InterruptEdge edge, GPIOIrqCallback cb, void* context) = 0;
            // virtual void disable_pin_irq(uint32_t bank, uint32_t pin) = 0;
        };
    } // namespace gpio

    /**
     * @brief Power management operations interface.
     */
    struct PowerOps {
        virtual ~PowerOps() = default;
        virtual void enter_idle_state(uint32_t core_id) = 0; // e.g., WFI
        virtual bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) = 0; // 0 for default/max
        // virtual void system_shutdown() = 0;
        // virtual void system_reboot() = 0;
    };

    /**
     * @brief Watchdog timer operations interface.
     */
    struct WatchdogOps {
        virtual ~WatchdogOps() = default;
        virtual bool start_watchdog(uint32_t timeout_ms) = 0;
        virtual bool reset_watchdog() = 0; // "Pet" the watchdog
        virtual bool stop_watchdog() = 0;
    };

    /**
     * @brief Platform interface: provides access to all HAL driver operations.
     */
    class Platform {
    public:
        virtual ~Platform() = default;
        virtual uint32_t get_core_id() const = 0; // Gets current physical core ID
        virtual uint32_t get_num_cores() const = 0; // Gets total number of cores

        virtual UARTDriverOps* get_uart_ops() = 0;
        virtual IRQControllerOps* get_irq_ops() = 0;
        virtual TimerDriverOps* get_timer_ops() = 0;
        virtual DMAControllerOps* get_dma_ops() = 0; // Can be nullptr if no DMA
        virtual I2SDriverOps* get_i2s_ops() = 0;   // Can be nullptr
        virtual MemoryOps* get_mem_ops() = 0;     // Can be nullptr
        virtual net::NetworkDriverOps* get_net_ops() = 0; // Can be nullptr
        virtual PowerOps* get_power_ops() = 0;     // Can be nullptr
        virtual gpio::GPIODriverOps* get_gpio_ops() = 0; // Can be nullptr
        virtual WatchdogOps* get_watchdog_ops() = 0; // Can be nullptr

        virtual void early_init_platform() = 0; // Platform-wide one-time init (by core 0)
        virtual void early_init_core(uint32_t core_id) = 0;   // Per-core early init

        virtual void panic(const char* msg, const char* file, int line) = 0; // System panic
        virtual void reboot_system() = 0; // System reboot
    };
} // namespace hal

// Global instances (declarations)
extern hal::Platform* g_platform; // Pointer to the active platform implementation
extern Scheduler* g_scheduler_ptr; // Pointer to the system scheduler
extern FixedMemoryPool g_audio_pool; // General purpose audio pool (if different from AudioSystem's internal)
extern uint8_t g_audio_pool_mem[64 * 1024]; // Memory for g_audio_pool
extern FixedMemoryPool g_software_timer_obj_pool; // Pool for SoftwareTimer objects
extern uint8_t g_software_timer_obj_pool_mem[MAX_SOFTWARE_TIMERS * sizeof(hal::timer::SoftwareTimer)];

// Kernel-level API functions (declarations)
void trace_event(const char* event_str, uintptr_t arg1, uintptr_t arg2 = 0);
void dump_trace_buffer(hal::UARTDriverOps* uart_ops);
void set_power_mode(bool enter_low_power); // True to enter low power, false to resume normal
void configure_memory_protection(TCB* tcb, bool enable_for_task); // For MPU/MMU setup
void get_kernel_stats(hal::UARTDriverOps* uart_ops);

// Global lock for AudioSystem init/stop if needed (example)
extern Spinlock g_audio_system_lock;


} // namespace kernel

// Subsystem includes (after kernel namespace and HAL are defined)
#include "util.hpp"    // General utilities
#include "dsp.hpp"     // DSPNode uses kernel::hal::UARTDriverOps, net::IPv4Addr (via #include "net.hpp")
#include "audio.hpp"   // AudioSystem uses kernel::dsp::DSPGraph, kernel::hal::i2s, kernel::FixedMemoryPool
#include "cli.hpp"     // CLI uses kernel::hal::UARTDriverOps
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