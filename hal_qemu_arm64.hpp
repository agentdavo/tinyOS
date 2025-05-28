// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_arm64.hpp
 * @brief ARM64 HAL header for QEMU virt platform in miniOS v1.7.
 */

#ifndef HAL_QEMU_ARM64_HPP
#define HAL_QEMU_ARM64_HPP

#include "hal.hpp"       // Directly include the base HAL interfaces FIRST
#include "core.hpp"      // For TCB definition if used in context switch stubs, and core constants
#include "audio.hpp"     // For kernel::audio::AudioBuffer
#include <string_view>
#include <array>
#include <atomic>
#include <cstdint>

namespace hal::qemu_virt_arm64 { // This is a global namespace, not under kernel::hal

// Base addresses for QEMU 'virt' machine peripherals
constexpr uint64_t UART_BASE = 0x09000000;
constexpr uint64_t GIC_DISTRIBUTOR_BASE = 0x08000000;
constexpr uint64_t GIC_CPU_INTERFACE_BASE = 0x08010000;
constexpr uint64_t RTC_BASE = 0x09010000;
constexpr uint64_t VIRTIO_MMIO_BASE = 0x0A000000; 
constexpr uint64_t VIRTIO_NET_REG_BASE = VIRTIO_MMIO_BASE;

constexpr uint64_t GPIO_CTRL_BASE = 0x0C000000; // Dummy
constexpr uint64_t I2S_CTRL_BASE = 0x0D000000;  // Dummy

// IRQ Numbers (SPIs, ID >= 32)
// SYSTEM_TIMER_IRQ (PPI ID 30) is defined in kernel::hal::SYSTEM_TIMER_IRQ
constexpr uint32_t IRQ_UART0 = 33;
constexpr uint32_t IRQ_RTC_PL031 = 34;
constexpr uint32_t IRQ_VIRTIO_NET = 35;
constexpr uint32_t IRQ_GPIO_EXAMPLE = 36;


class UARTDriver : public kernel::hal::UARTDriverOps {
public:
    void putc(char c) override;
    void puts(const char* str) override;
    void uart_put_uint64_hex(uint64_t value) override;
    char getc_blocking() override;
private:
    void write_uart_reg(uint32_t offset, uint32_t value);
    uint32_t read_uart_reg(uint32_t offset);
};

class IRQController : public kernel::hal::IRQControllerOps {
public:
    void enable_core_irqs(uint32_t core_id, uint32_t irq_source_mask) override;
    void disable_core_irqs(uint32_t core_id) override;
    void init_distributor() override;
    void init_cpu_interface(uint32_t core_id) override;
    uint32_t ack_irq(uint32_t core_id) override;
    void end_irq(uint32_t core_id, uint32_t irq_id) override;
    void enable_irq_line(uint32_t irq_id) override;
    void disable_irq_line(uint32_t irq_id) override;
    void set_irq_priority(uint32_t irq_id, uint8_t priority) override;
private:
    void write_gicd_reg(uint32_t offset, uint32_t value);
    uint32_t read_gicd_reg(uint32_t offset);
    void write_gicc_reg(uint32_t offset, uint32_t value);
    uint32_t read_gicc_reg(uint32_t offset);
};

class TimerDriver : public kernel::hal::TimerDriverOps {
public:
    TimerDriver();
    void init_system_timer_properties(uint64_t freq_hz_override = 0) override;
    void init_core_timer_interrupt(uint32_t core_id) override;
    void ack_core_timer_interrupt(uint32_t core_id) override;
    bool add_software_timer(kernel::hal::timer::SoftwareTimer* timer) override;
    bool remove_software_timer(kernel::hal::timer::SoftwareTimer* timer) override;
    uint64_t get_system_time_us() override;
    void hardware_timer_irq_fired(uint32_t core_id) override;
private:
    uint64_t timer_freq_hz_;
    kernel::hal::timer::SoftwareTimer* active_sw_timers_head_ = nullptr;
    kernel::core::Spinlock sw_timer_lock_;
};

class DMAController : public kernel::hal::DMAControllerOps {
public:
    DMAController();
    kernel::hal::dma::ChannelID request_channel() override;
    bool configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                     const kernel::hal::dma::TransferConfig& cfg,
                                     kernel::hal::dma::DMACallback cb, void* context) override;
    void release_channel(kernel::hal::dma::ChannelID channel) override;
private:
    std::array<bool, 8> channels_in_use_;
    kernel::core::Spinlock dma_lock_;
};

class I2SDriver : public kernel::hal::I2SDriverOps {
public:
    bool init(uint32_t instance_id, kernel::hal::i2s::Mode mode, const kernel::hal::i2s::Format& format,
              size_t samples_per_block, uint8_t num_buffers, kernel::hal::i2s::I2SCallback cb,
              void* user_data) override;
    bool start(uint32_t instance_id) override;
    bool stop(uint32_t instance_id) override;
    kernel::audio::AudioBuffer* get_buffer_for_app_tx(uint32_t instance_id) override;
    bool submit_filled_buffer_to_hw_tx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) override;
    void release_processed_buffer_to_hw_rx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) override;
    void convert_hw_format_to_dsp_format(kernel::audio::AudioBuffer* buffer,
                                        const kernel::hal::i2s::Format& format) override;
    void convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buffer,
                                        const kernel::hal::i2s::Format& format) override;
private:
    struct I2SInstanceData {
        kernel::hal::i2s::I2SCallback callback = nullptr;
        void* user_data = nullptr;
        bool active = false;
        kernel::hal::i2s::Format current_format;
    };
    std::array<I2SInstanceData, 2> instances_;
};

class MemoryOps : public kernel::hal::MemoryOps {
public:
    void flush_cache_range(const void* addr, size_t size) override;
    void invalidate_cache_range(const void* addr, size_t size) override;
};

class NetworkDriver : public kernel::hal::net::NetworkDriverOps {
public:
    // Constructor initializes members
    NetworkDriver() : packet_received_cb_(nullptr), cb_context_(nullptr), initialized_(false) {}
    bool init_interface(int if_idx) override;
    bool send_packet(int if_idx, const uint8_t* data, size_t len) override;
    // Ensure this signature EXACTLY matches the base class version
    void register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb, void* context) override;
private:
    // Ensure this type EXACTLY matches the type used in the function parameter above
    kernel::hal::net::PacketReceivedCallback packet_received_cb_;
    void* cb_context_;
    bool initialized_;
};

class PowerOps : public kernel::hal::PowerOps {
public:
    void enter_idle_state(uint32_t core_id) override;
    bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) override;
};

class GPIODriver : public kernel::hal::gpio::GPIODriverOps {
public:
    bool init_bank(uint32_t bank_id) override;
    bool configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode) override;
    bool set_pin_state(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state) override;
    kernel::hal::gpio::PinState read_pin_state(uint32_t bank, uint32_t pin) override;
    void enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) override;
};

class WatchdogDriver : public kernel::hal::WatchdogOps {
public:
    bool start_watchdog(uint32_t timeout_ms) override;
    bool reset_watchdog() override;
    bool stop_watchdog() override;
};

class PlatformQEMUVirtARM64 : public kernel::hal::Platform {
public:
    PlatformQEMUVirtARM64();
    uint32_t get_core_id() const override;
    uint32_t get_num_cores() const override;
    void early_init_platform() override;
    void early_init_core(uint32_t core_id) override;
    [[noreturn]] void panic(const char* msg, const char* file, int line) override;
    void reboot_system() override;
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
    NetworkDriver network_driver_; // This instantiation was problematic
    PowerOps power_ops_;
    GPIODriver gpio_driver_;
    WatchdogDriver watchdog_driver_;
};

} // namespace hal::qemu_virt_arm64

#endif // HAL_QEMU_ARM64_HPP