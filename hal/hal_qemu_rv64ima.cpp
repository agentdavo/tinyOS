// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_rv64ima.cpp
 * @brief RISC-V HAL implementation for QEMU virt platform in miniOS v1.7.
 * @details
 * Implements the RISC-V HAL for QEMU virt, providing platform-specific operations for UART,
 * interrupts (PLIC/CLINT), timers, DMA, I2S, networking (virtio-net), GPIO, power, and watchdog.
 * Supports RVV SIMD for DSP and SMP with 4 cores. Updated in v1.7 with improved error handling,
 * clearer diagnostics, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 * - std::string_view for string operations
 *
 * @version 1.7
 * @see hal_qemu_rv64ima.hpp, miniOS.hpp, dsp.hpp
 */

#include "hal_qemu_rv64ima.hpp"
#include <cstring>
#include <cassert>

namespace hal::qemu_virt_rv64ima {

namespace {
// MMIO access
inline void write_mmio(uint64_t addr, uint32_t value) {
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

inline uint32_t read_mmio(uint64_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}
}

// --- UARTDriver ---
void UARTDriver::write_reg(uint32_t offset, uint32_t value) {
    write_mmio(UART_BASE + offset, value);
}

uint32_t UARTDriver::read_reg(uint32_t offset) {
    return read_mmio(UART_BASE + offset);
}

void UARTDriver::putc(char c) {
    while (!(read_reg(0x14) & 0x20)); // Wait for TX not full
    write_reg(0x00, static_cast<uint32_t>(c));
}

void UARTDriver::puts(const char* str) {
    if (!str) return;
    while (*str) putc(*str++);
}

void UARTDriver::uart_put_uint64_hex(uint64_t value) {
    char buf[17] = "0123456789ABCDEF";
    char out[17];
    out[16] = '\0';
    for (int i = 15; i >= 0; --i) {
        out[i] = buf[value & 0xF];
        value >>= 4;
    }
    puts(out);
}

char UARTDriver::getc_blocking() {
    while (!(read_reg(0x14) & 0x01)); // Wait for RX not empty
    return static_cast<char>(read_reg(0x00));
}

// --- IRQController ---
void IRQController::write_plic(uint32_t offset, uint32_t value) {
    write_mmio(PLIC_BASE + offset, value);
}

uint32_t IRQController::read_plic(uint32_t offset) {
    return read_mmio(PLIC_BASE + offset);
}

void IRQController::write_clint(uint32_t offset, uint32_t value) {
    write_mmio(CLINT_BASE + offset, value);
}

uint32_t IRQController::read_clint(uint32_t offset) {
    return read_mmio(CLINT_BASE + offset);
}

void IRQController::enable_core_irqs(uint32_t core_id, uint32_t source) {
    write_plic(0x2000 + core_id * 0x80, 1U << source); // Enable context
    enable_irq_line(source);
}

void IRQController::disable_core_irqs(uint32_t core_id) {
    write_plic(0x2000 + core_id * 0x80, 0);
}

void IRQController::init_distributor() {
    write_plic(0x0000, 0); // Disable PLIC
    for (uint32_t i = 1; i < 256; ++i) {
        write_plic(0x1000 + i * 4, 0); // Clear pending
    }
    write_plic(0x0000, 1); // Enable PLIC
}

void IRQController::init_cpu_interface() {
    write_clint(0x4000, 0xFFFFFFFF); // Clear mtimecmp
}

uint32_t IRQController::ack_irq(uint32_t core_id) {
    return read_plic(0x200004 + core_id * 0x2000); // Claim interrupt
}

void IRQController::end_irq(uint32_t core_id, uint32_t source) {
    write_plic(0x200004 + core_id * 0x2000, source); // Complete interrupt
}

void IRQController::enable_irq_line(uint32_t source) {
    write_plic(0x80 + (source / 32) * 4, 1U << (source % 32));
}

void IRQController::set_irq_priority(uint32_t source, uint8_t priority) {
    write_plic(0x0004 + source * 4, priority);
}

// --- TimerDriver ---
void TimerDriver::init_system_timer_properties(uint64_t freq_hz_override) {
    if (freq_hz_override > 0) timer_freq_hz_ = freq_hz_override;
}

void TimerDriver::init_core_timer_interrupt() {
    write_clint(0x4000, 0xFFFFFFFF); // Set mtimecmp
}

void TimerDriver::ack_core_timer_interrupt() {
    write_clint(0xBFF8, 0); // Clear mtime
}

bool TimerDriver::add_software_timer(kernel::hal::timer::SoftwareTimer* timer) {
    if (!timer || timer_count_ >= software_timers_.size()) return false;
    software_timers_[timer_count_++] = timer;
    return true;
}

bool TimerDriver::remove_software_timer(kernel::hal::timer::SoftwareTimer* timer) {
    if (!timer) return false;
    for (size_t i = 0; i < timer_count_; ++i) {
        if (software_timers_[i] == timer) {
            software_timers_[i] = software_timers_[--timer_count_];
            software_timers_[timer_count_] = nullptr;
            return true;
        }
    }
    return false;
}

uint64_t TimerDriver::get_system_time_us() {
    uint64_t mtime = read_clint(0xBFF8);
    return (mtime * 1000000) / timer_freq_hz_;
}

void TimerDriver::hardware_timer_irq_fired(uint32_t core_id) {
    uint64_t now = get_system_time_us();
    for (size_t i = 0; i < timer_count_; ++i) {
        auto* timer = software_timers_[i];
        if (timer && timer->active && now >= timer->expiry_time_us) {
            timer->callback(timer, timer->context);
            if (timer->period_us > 0) {
                timer->expiry_time_us += timer->period_us;
            } else {
                timer->active = false;
            }
        }
    }
    write_clint(0x4000, 0xFFFFFFFF); // Reset mtimecmp
}

// --- DMAController ---
kernel::hal::dma::ChannelID DMAController::request_channel() {
    for (size_t i = 0; i < channels_in_use_.size(); ++i) {
        if (!channels_in_use_[i]) {
            channels_in_use_[i] = true;
            return static_cast<kernel::hal::dma::ChannelID>(i);
        }
    }
    return kernel::hal::dma::INVALID_CHANNEL;
}

bool DMAController::configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                                const kernel::hal::dma::TransferConfig& cfg,
                                                kernel::hal::dma::DMACallback cb, void* context) {
    if (channel < 0 || static_cast<size_t>(channel) >= channels_in_use_.size() || !channels_in_use_[channel]) {
        return false;
    }
    // Simplified: Assume QEMU virt DMA controller
    if (cb) cb(channel, true, context);
    return true;
}

// --- I2SDriver ---
bool I2SDriver::init(uint32_t instance_id, kernel::hal::i2s::Mode mode, const kernel::hal::i2s::Format& format,
                     size_t samples_per_block, uint8_t num_buffers, kernel::hal::i2s::I2SCallback cb, void* user_data) {
    if (instance_id >= instances_.size() || samples_per_block == 0 || num_buffers == 0) return false;
    auto& inst = instances_[instance_id];
    inst.callback = cb;
    inst.user_data = user_data;
    inst.active = true;
    return true;
}

bool I2SDriver::start(uint32_t instance_id) {
    if (instance_id >= instances_.size() || !instances_[instance_id].active) return false;
    write_mmio(I2S_BASE + (instance_id * 0x1000), 1); // Enable I2S
    return true;
}

kernel::audio::AudioBuffer* I2SDriver::get_buffer_for_app_tx(uint32_t instance_id) {
    if (instance_id >= instances_.size() || !instances_[instance_id].active) return nullptr;
    auto* buffer = new kernel::audio::AudioBuffer;
    buffer->data_raw_i2s = new uint8_t[1024]; // Simplified: 1KB
    buffer->size_bytes_raw_buffer = 1024;
    buffer->samples_per_channel = 256;
    return buffer;
}

bool I2SDriver::submit_filled_buffer_to_hw_tx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) {
    if (instance_id >= instances_.size() || !instances_[instance_id].active || !buffer) return false;
    if (instances_[instance_id].callback) {
        instances_[instance_id].callback(instance_id, buffer, kernel::hal::i2s::Mode::MASTER_TX,
                                        instances_[instance_id].user_data);
    }
    return true;
}

void I2SDriver::release_processed_buffer_to_hw_rx(uint32_t instance_id, kernel::audio::AudioBuffer* buffer) {
    if (instance_id >= instances_.size() || !buffer) return;
    delete[] static_cast<uint8_t*>(buffer->data_raw_i2s);
    delete buffer;
}

void I2SDriver::convert_hw_format_to_dsp_format(const kernel::audio::AudioBuffer* buffer,
                                               const kernel::hal::i2s::Format& format) {
    if (!buffer || !buffer->data_raw_i2s || !buffer->data_dsp_canonical) return;
    // Simplified: Assume 24-bit to float conversion
    for (size_t i = 0; i < buffer->samples_per_channel * format.channels; ++i) {
        buffer->data_dsp_canonical[i] = 0.0f; // Placeholder
    }
}

void I2SDriver::convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buffer,
                                               const kernel::hal::i2s::Format& format) {
    if (!buffer || !buffer->data_raw_i2s || !buffer->data_dsp_canonical) return;
    // Simplified: Assume float to 24-bit conversion
    for (size_t i = 0; i < buffer->samples_per_channel * format.channels; ++i) {
        static_cast<uint8_t*>(buffer->data_raw_i2s)[i * 3] = 0; // Placeholder
    }
}

// --- MemoryOps ---
void MemoryOps::flush_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    asm volatile("fence" ::: "memory");
}

void MemoryOps::invalidate_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    asm volatile("fence.i" ::: "memory");
}

// --- NetworkDriver ---
bool NetworkDriver::send_packet(std::span<const uint8_t> data) {
    if (data.empty()) return false;
    for (size_t i = 0; i < data.size(); ++i) {
        write_mmio(VIRTIO_NET_BASE + 0x1000, data[i]);
    }
    return true;
}

bool NetworkDriver::receive_packet(std::span<uint8_t> buffer, size_t& len) {
    if (buffer.empty()) return false;
    len = read_mmio(VIRTIO_NET_BASE + 0x1004); // Simplified: Packet length
    if (len == 0 || len > buffer.size()) return false;
    for (size_t i = 0; i < len; ++i) {
        buffer[i] = read_mmio(VIRTIO_NET_BASE + 0x1008 + i);
    }
    return true;
}

// --- PowerOps ---
void PowerOps::enter_idle_state(uint32_t core_id) {
    asm volatile("wfi");
}

bool PowerOps::set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) {
    // QEMU virt: No frequency scaling
    return true;
}

// --- GPIODriver ---
void GPIODriver::write_gpio_reg(uint32_t offset, uint32_t value) {
    write_mmio(GPIO_BASE + offset, value);
}

uint32_t GPIODriver::read_gpio_reg(uint32_t offset) {
    return read_mmio(GPIO_BASE + offset);
}

bool GPIODriver::configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode) {
    if (bank >= gpio::NUM_BANKS || pin >= gpio::PINS_PER_BANK) return false;
    uint32_t reg = read_gpio_reg(bank * 0x1000);
    reg &= ~(1U << pin);
    if (mode == kernel::hal::gpio::PinMode::OUTPUT) reg |= (1U << pin);
    write_gpio_reg(bank * 0x1000, reg);
    return true;
}

bool GPIODriver::set_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state) {
    if (bank >= gpio::NUM_BANKS || pin >= gpio::PINS_PER_BANK) return false;
    uint32_t reg = read_gpio_reg(bank * 0x1000 + 0x04);
    reg &= ~(1U << pin);
    if (state == kernel::hal::gpio::PinState::HIGH) reg |= (1U << pin);
    write_gpio_reg(bank * 0x1000 + 0x04, reg);
    return true;
}

kernel::hal::gpio::PinState GPIODriver::read_pin(uint32_t bank, uint32_t pin) {
    if (bank >= gpio::NUM_BANKS || pin >= gpio::PINS_PER_BANK) return kernel::hal::gpio::PinState::LOW;
    uint32_t reg = read_gpio_reg(bank * 0x1000 + 0x08);
    return (reg & (1U << pin)) ? kernel::hal::gpio::PinState::HIGH : kernel::hal::gpio::PinState::LOW;
}

void GPIODriver::enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) {
    if (bank >= gpio::NUM_BANKS || pin >= gpio::PINS_PER_BANK) return;
    uint32_t reg = read_gpio_reg(bank * 0x1000 + 0x0C);
    reg |= (1U << pin);
    write_gpio_reg(bank * 0x1000 + 0x0C, reg);
    reg = read_gpio_reg(bank * 0x1000 + 0x10);
    reg &= ~(1U << pin);
    if (rising_edge) reg |= (1U << pin);
    write_gpio_reg(bank * 0x1000 + 0x10, reg);
}

// --- WatchdogDriver ---
bool WatchdogDriver::start_watchdog(uint32_t timeout_ms) {
    write_mmio(CLINT_BASE + 0xBFFC, timeout_ms * 1000); // Timeout in us
    write_mmio(CLINT_BASE + 0xBFF0, 1); // Enable
    return true;
}

bool WatchdogDriver::reset_watchdog() {
    write_mmio(CLINT_BASE + 0xBFF4, 1); // Reset
    return true;
}

// --- PlatformQEMUVirtRV64IMA ---
uint32_t PlatformQEMUVirtRV64IMA::get_core_id() const {
    uint32_t hartid;
    asm volatile("csrr %0, mhartid" : "=r"(hartid));
    return hartid;
}

void PlatformQEMUVirtRV64IMA::early_init() {
    uart_driver_.puts("[miniOS v1.7] RISC-V QEMU Virt Early Init\n");
    irq_controller_.init_distributor();
}

void PlatformQEMUVirtRV64IMA::core_early_init() {
    irq_controller_.init_cpu_interface();
    timer_driver_.init_core_timer_interrupt();
}

void PlatformQEMUVirtRV64IMA::panic(const char* msg, const char* file, int line) {
    uart_driver_.puts("PANIC: ");
    if (msg) uart_driver_.puts(msg);
    uart_driver_.puts(" at ");
    if (file) uart_driver_.puts(file);
    char buf[16];
    std::snprintf(buf, sizeof(buf), ":%d", line);
    uart_driver_.puts(buf);
    uart_driver_.puts("\n");
    while (true) asm volatile("wfi");
}

} // namespace hal::qemu_virt_rv64ima