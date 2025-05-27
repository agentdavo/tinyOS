// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_arm64.hpp
 * @brief ARM64 HAL header for QEMU virt platform in miniOS v1.7.
 * @details
 * Defines the ARM64 hardware abstraction layer (HAL) for QEMU virt, implementing platform-specific
 * interfaces for UART, interrupts, timers, DMA, I2S, networking, GPIO, power, and watchdog.
 * Supports NEON SIMD for DSP and SMP with 4 cores. Updated in v1.7 with improved error handling,
 * clearer documentation, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 * - std::string_view for string operations
 *
 * @version 1.7
 * @see hal_qemu_arm64.cpp, miniOS.hpp, dsp.hpp
 */

#ifndef HAL_QEMU_ARM64_HPP
#define HAL_QEMU_ARM64_HPP

#include "miniOS.hpp"
#include <span>
#include <atomic>
#include <array>
#include <string_view>
#include <cstdint>

namespace hal::qemu_virt_arm64 {

constexpr uint64_t UART_BASE = 0x09000000; // PL011 UART
constexpr uint64_t GIC_DISTRIBUTOR_BASE = 0x08000000; // GICv2
constexpr uint64_t GIC_CPU_INTERFACE_BASE = 0x08010000;
constexpr uint64_t RTC_BASE = 0x09010000; // PL031 RTC
constexpr uint64_t VIRTIO_NET_BASE = 0x0A000000;
constexpr uint64_t GPIO_BASE = 0x0C000000;
constexpr uint64_t I2S_BASE = 0x0D000000;
constexpr uint32_t IRQ_UART = 33;
constexpr uint32_t IRQ_RTC = 34;
constexpr uint32_t IRQ_VIRTIO_NET = 35;
constexpr uint32_t IRQ_GPIO = 36;

/**
 * @brief ARM64 UART driver operations.
 */
class UARTDriver : public kernel::hal::UARTDriverOps {
public:
    void putc(char c) override;
    void puts(const char* str) override;
    void uart_put_uint64_hex(uint64_t value) override;
    char getc_blocking() override;
private:
    void write_reg(uint32_t offset, uint32_t value);
    uint32_t read_reg(uint32_t offset);
};

/**
 * @brief ARM64 IRQ controller operations.
 */
class IRQController : public kernel::hal::IRQControllerOps {
public:
    void enable_core_irqs(uint32_t core_id, uint32_t source) override;
    void disable_core_irqs(uint32_t core_id) override;
    void init_distributor() override;
    void init_cpu_interface() override;
    uint32_t ack_irq(uint32_t core_id) override;
    void end_irq(uint32_t core_id, uint32_t source) override;
    void enable_irq_line(uint32_t source) override;
    void set_irq_priority(uint32_t source, uint8_t priority) override;
private:
    void write_gicd(uint32_t offset, uint32_t value);
    uint32_t read_gicd(uint32_t offset);
    void write_gicc(uint32_t offset, uint32_t value);
    uint32_t read_gicc(uint32_t offset);
};

/**
 * @brief ARM64 timer driver operations.
 */
class TimerDriver : public kernel::hal::TimerDriverOps {
public:
    void init_system_timer_properties(uint64_t freq_hz_override = 0) override;
    void init_core_timer_interrupt() override;
    void ack_core_timer_interrupt() override;
    bool add_software_timer(kernel::hal::timer::SoftwareTimer* timer) override;
    bool remove_software_timer(kernel::hal::timer::SoftwareTimer* timer) override;
    uint64_t get_system_time_us() override;
    void hardware_timer_irq_fired(uint32_t core_id) override;
private:
    uint64_t timer_freq_hz_ = 62500000; // Default QEMU virt
    std::array<kernel::hal::timer::SoftwareTimer*, 64> software_timers_;
    size_t timer_count_ = 0;
};

/**
 * @brief ARM64 DMA controller operations.
 */
class DMAController : public kernel::hal::DMAControllerOps {
public:
    kernel::hal::dma::ChannelID request_channel() override;
    bool configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                     const kernel::hal::dma::TransferConfig& cfg,
                                     kernel::hal::dma::DMACallback cb, void* context) override;
private:
    std::array<bool, 8> channels_in_use_;
};

/**
 * @brief ARM64 I2S driver operations.
 */
class I2SDriver : public kernel::hal::I2SDriverOps {
public:
    bool init(uint32_t instance_id, kernel::hal::i2s::Mode mode, const kernel::hal::i2s::Format& format,
              size_t samples_per_block, uint8_t num_buffers, kernel::hal::i2s::I2SCallback cb,
              void* user_data) override;
    bool start(uint32_t instance_id) override;
    kernel::audio::AudioBuffer* get_buffer_for_app_tx(uint32_t instance_id) override;
    bool submit_filled_buffer_to_hw_tx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) override;
    void release_processed_buffer_to_hw_rx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) override;
    void convert_hw_format_to_dsp_format(const kernel::audio::AudioBuffer* buffer,
                                        const kernel::hal::i2s::Format& format) override;
    void convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buffer,
                                        const kernel::hal::i2s::Format& format) override;
private:
    struct I2SInstance {
        kernel::hal::i2s::I2SCallback callback = nullptr;
        void* user_data = nullptr;
        bool active = false;
    };
    std::array<I2SInstance, 2> instances_; // 0=RX, 1=TX
};

/**
 * @brief ARM64 memory operations.
 */
class MemoryOps : public kernel::hal::MemoryOps {
public:
    void flush_cache_range(const void* addr, size_t size) override;
    void invalidate_cache_range(const void* addr, size_t size) override;
};

/**
 * @brief ARM64 network driver operations.
 */
class NetworkDriver : public kernel::hal::net::NetworkDriverOps {
public:
    bool send_packet(std::span<const uint8_t> data) override;
    bool receive_packet(std::span<uint8_t> buffer, size_t& len) override;
};

/**
 * @brief ARM64 power operations.
 */
class PowerOps : public kernel::hal::PowerOps {
public:
    void enter_idle_state(uint32_t core_id) override;
    bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) override;
};

/**
 * @brief ARM64 GPIO driver operations.
 */
class GPIODriver : public kernel::hal::gpio::GPIODriverOps {
public:
    bool configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode) override;
    bool set_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state) override;
    kernel::hal::gpio::PinState read_pin(uint32_t bank, uint32_t pin) override;
    void enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) override;
private:
    void write_gpio_reg(uint32_t offset, uint32_t value);
    uint32_t read_gpio_reg(uint32_t offset);
};

/**
 * @brief ARM64 watchdog operations.
 */
class WatchdogDriver : public kernel::hal::WatchdogOps {
public:
    bool start_watchdog(uint32_t timeout_ms) override;
    bool reset_watchdog() override;
};

/**
 * @brief ARM64 platform implementation for QEMU virt.
 */
class PlatformQEMUVirtARM64 : public kernel::hal::Platform {
public:
    uint32_t get_core_id() const override;
    void early_init() override;
    void core_early_init() override;
    void panic(const char* msg, const char* file, int line) override;
    kernel::hal::UARTDriverOps* get_uart_ops() override { return &uart_driver_; }
    kernel::hal::IRQControllerOps* get_irq_ops() override { return &irq_controller_; }
    kernel::hal::TimerDriverOps* get_timer_ops() override { return &timer_driver_; }
    kernel::hal::DMAControllerOps* get_dma_ops() override { return &dma_controller_; }
    kernel::hal::I2SDriverOps* get_i2s_ops() override { return &i2s_driver_; }
    kernel::hal::MemoryOps* get_mem_ops() override { return &memory_ops_; }
    kernel::hal::net::NetworkDriverOps* get_net_ops() override { return &network_driver_; }
    kernel::hal::PowerOps* get_power_ops() override { return &power_ops_; }
    kernel::hal::gpio::GPIODriverOps* get_gpio_ops() override { return &gpio_driver_; }
    kernel::hal::WatchdogOps* get_watchdog_ops() override { return &watchdog_driver_; }

private:
    UARTDriver uart_driver_;
    IRQController irq_controller_;
    TimerDriver timer_driver_;
    DMAController dma_controller_;
    I2SDriver i2s_driver_;
    MemoryOps memory_ops_;
    NetworkDriver network_driver_;
    PowerOps power_ops_;
    GPIODriver gpio_driver_;
    WatchdogDriver watchdog_driver_;
};

} // namespace hal::qemu_virt_arm64

#endif // HAL_QEMU_ARM64_HPP