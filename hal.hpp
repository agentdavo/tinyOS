// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal.hpp
 * @brief Hardware Abstraction Layer (HAL) interfaces for miniOS v1.7.
 * @details
 * Defines HAL interfaces for hardware interactions (UART, IRQ, DMA, I2S, etc.).
 * Depends on core.hpp for shared types, uses forward declarations for subsystems.
 *
 * @version 1.7
 * @see hal.cpp, core.hpp
 */

#ifndef HAL_HPP
#define HAL_HPP

#include "core.hpp" // For kernel::core::MAX_CORES etc.
#include <cstdint>   // For uint32_t, uint64_t etc.
#include <cstddef>   // For size_t

// Forward declarations for subsystems
namespace kernel { namespace audio { class AudioBuffer; } } 
namespace kernel { namespace core { struct TCB; } } // Forward declare TCB for context_switch

namespace kernel {
namespace hal {

// ARM Generic Timer IRQ (Non-Secure EL1 Physical Timer)
// This is a Private Peripheral Interrupt (PPI) for the core.
// GIC IRQ IDs for PPIs are 16-31. CNTPNSIRQ is IRQ 30 (0-indexed from IRQ0).
constexpr uint32_t SYSTEM_TIMER_IRQ = 30; // PPI ID for non-secure physical timer


namespace sync {
    void barrier_dmb();
    void barrier_dsb();
    void barrier_isb();
}

// Architecture specific CPU operations
// This function is called by the scheduler to perform a context switch.
// Its implementation is architecture-specific and typically involves assembly.
void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb);


struct MemoryOps {
    virtual ~MemoryOps() = default;
    virtual void flush_cache_range(const void* addr, size_t size) = 0;
    virtual void invalidate_cache_range(const void* addr, size_t size) = 0;
};

struct UARTDriverOps {
    virtual ~UARTDriverOps() = default;
    virtual void putc(char c) = 0;
    virtual void puts(const char* str) = 0;
    virtual void uart_put_uint64_hex(uint64_t value) = 0;
    virtual char getc_blocking() = 0;
};

struct IRQControllerOps {
    virtual ~IRQControllerOps() = default;
    virtual void enable_core_irqs(uint32_t core_id, uint32_t irq_source_mask) = 0;
    virtual void disable_core_irqs(uint32_t core_id) = 0;
    virtual void init_distributor() = 0;
    virtual void init_cpu_interface(uint32_t core_id) = 0;
    virtual uint32_t ack_irq(uint32_t core_id) = 0;
    virtual void end_irq(uint32_t core_id, uint32_t irq_id) = 0;
    virtual void enable_irq_line(uint32_t irq_id) = 0;
    virtual void disable_irq_line(uint32_t irq_id) = 0;
    virtual void set_irq_priority(uint32_t irq_id, uint8_t priority) = 0;
};

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
        size_t burst_size_bytes = 4;
    };
    using DMACallback = void (*)(ChannelID channel, bool success, void* context);
}
struct DMAControllerOps {
    virtual ~DMAControllerOps() = default;
    virtual dma::ChannelID request_channel() = 0;
    virtual void release_channel(dma::ChannelID channel) = 0;
    virtual bool configure_and_start_transfer(dma::ChannelID channel, const dma::TransferConfig& cfg, dma::DMACallback cb, void* context) = 0;
};

namespace i2s {
    enum class Mode { MASTER_TX, MASTER_RX, SLAVE_TX, SLAVE_RX };
    enum class BitDepth { BITS_16, BITS_24_IN_32, BITS_32 };
    struct Format {
        uint32_t sample_rate_hz;
        uint8_t num_channels;
        BitDepth bit_depth;
        
        size_t get_bytes_per_sample_per_channel() const noexcept {
            switch (bit_depth) {
                case BitDepth::BITS_16: return 2;
                case BitDepth::BITS_24_IN_32: [[fallthrough]];
                case BitDepth::BITS_32: return 4;
                default: return 0;
            }
        }
        size_t get_bytes_per_frame() const noexcept {
            return get_bytes_per_sample_per_channel() * num_channels;
        }
    };
    using I2SCallback = void (*)(uint32_t instance_id, kernel::audio::AudioBuffer* buffer, Mode mode, void* user_data);
}
struct I2SDriverOps {
    virtual ~I2SDriverOps() = default;
    virtual bool init(uint32_t instance_id, i2s::Mode mode, const i2s::Format& format,
                      size_t samples_per_block_per_channel, uint8_t num_dma_buffers,
                      i2s::I2SCallback cb, void* user_data) = 0;
    virtual bool start(uint32_t instance_id) = 0;
    virtual bool stop(uint32_t instance_id) = 0;
    virtual kernel::audio::AudioBuffer* get_buffer_for_app_tx(uint32_t instance_id) = 0;
    virtual bool submit_filled_buffer_to_hw_tx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer_to_send) = 0;
    virtual void release_processed_buffer_to_hw_rx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) = 0;
    virtual void convert_hw_format_to_dsp_format(kernel::audio::AudioBuffer* buffer, const i2s::Format& format) = 0;
    virtual void convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buffer, const i2s::Format& format) = 0;
};

namespace timer {
    struct SoftwareTimer;
    using software_timer_callback_t = void (*)(SoftwareTimer* timer, void* context);
    struct SoftwareTimer {
        uint64_t expiry_time_us;
        uint64_t period_us;
        software_timer_callback_t callback;
        void* context;
        bool active = false;
        SoftwareTimer* next = nullptr;
        uint32_t id = 0;
    };
}
struct TimerDriverOps {
    virtual ~TimerDriverOps() = default;
    virtual void init_system_timer_properties(uint64_t freq_hz_override = 0) = 0;
    virtual void init_core_timer_interrupt(uint32_t core_id) = 0;
    virtual void ack_core_timer_interrupt(uint32_t core_id) = 0;
    virtual bool add_software_timer(timer::SoftwareTimer* timer) = 0;
    virtual bool remove_software_timer(timer::SoftwareTimer* timer) = 0;
    virtual uint64_t get_system_time_us() = 0;
    virtual void hardware_timer_irq_fired(uint32_t core_id) = 0;
};

namespace net {
    using PacketReceivedCallback = void (*)(int if_idx, const uint8_t* data, size_t len, void* context);
    struct NetworkDriverOps {
        virtual ~NetworkDriverOps() = default;
        virtual bool init_interface(int if_idx) = 0;
        virtual bool send_packet(int if_idx, const uint8_t* data, size_t len) = 0;
        virtual void register_packet_receiver(PacketReceivedCallback cb, void* context) = 0;
    };
}

namespace gpio {
    constexpr size_t NUM_BANKS = 4; 
    constexpr size_t PINS_PER_BANK = 32; 
    enum class PinMode { INPUT, OUTPUT, ALT_FUNC };
    enum class PinState { LOW, HIGH };
    struct GPIODriverOps {
        virtual ~GPIODriverOps() = default;
        virtual bool init_bank(uint32_t bank_id) = 0;
        virtual bool configure_pin(uint32_t bank, uint32_t pin, PinMode mode) = 0;
        virtual bool set_pin_state(uint32_t bank, uint32_t pin, PinState state) = 0;
        virtual PinState read_pin_state(uint32_t bank, uint32_t pin) = 0;
        virtual void enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) = 0;
    };
}

struct PowerOps {
    virtual ~PowerOps() = default;
    virtual void enter_idle_state(uint32_t core_id) = 0;
    virtual bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) = 0;
};

struct WatchdogOps {
    virtual ~WatchdogOps() = default;
    virtual bool start_watchdog(uint32_t timeout_ms) = 0;
    virtual bool reset_watchdog() = 0;
    virtual bool stop_watchdog() = 0;
};

class Platform {
public:
    virtual ~Platform() = default;
    virtual uint32_t get_core_id() const = 0;
    virtual uint32_t get_num_cores() const = 0;
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
    virtual void early_init_platform() = 0;
    virtual void early_init_core(uint32_t core_id) = 0;
    [[noreturn]] virtual void panic(const char* msg, const char* file, int line) = 0;
    virtual void reboot_system() = 0;
};

Platform* get_platform();

} // namespace hal
} // namespace kernel

#endif // HAL_HPP