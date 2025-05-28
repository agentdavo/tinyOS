// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_arm64.cpp
 * @brief ARM64 HAL implementation for QEMU virt platform in miniOS v1.7.
 */

#include "hal_qemu_arm64.hpp"
// miniOS.hpp is not directly included here; kernel globals are accessed via kernel::hal::get_platform()
// or by functions passed into HAL methods if needed.
// However, for panic, it might need a way to call the global panic.
// For simplicity, if panic needs g_platform, it must be obtainable.
#include "miniOS.hpp" // Include for kernel::g_platform access (e.g. in panic)
                      // and other kernel types if needed by stubs.

#include <cstring>    // For std::memcpy, std::memset, strcmp
#include <cstdio>     // For std::snprintf
#include <algorithm>  // For std::min, std::max

// NEON header if used for DSP conversions (example)
#if defined(__aarch64__) && !defined(__ARM_NEON)
#define __ARM_NEON 1
#endif
#include <arm_neon.h>


namespace hal::qemu_virt_arm64 {

namespace { // Anonymous namespace for static helpers
    inline void mmio_write32(uint64_t addr, uint32_t value) {
        *reinterpret_cast<volatile uint32_t*>(addr) = value;
    }
    inline uint32_t mmio_read32(uint64_t addr) {
        return *reinterpret_cast<volatile uint32_t*>(addr);
    }

    // ARM Generic Timer access (using system registers)
    inline void write_sysreg_cntp_tval(uint64_t value) { asm volatile("msr cntp_tval_el0, %0" : : "r"(value)); }
    inline void write_sysreg_cntp_ctl(uint64_t value) { asm volatile("msr cntp_ctl_el0, %0" : : "r"(value)); }
    inline uint64_t read_sysreg_cntpct() { uint64_t v; asm volatile("mrs %0, cntpct_el0" : "=r"(v)); return v; }
    inline uint64_t read_sysreg_cntfrq() { uint64_t v; asm volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }
}

// --- UARTDriver ---
void UARTDriver::write_uart_reg(uint32_t offset, uint32_t value) { mmio_write32(UART_BASE + offset, value); }
uint32_t UARTDriver::read_uart_reg(uint32_t offset) { return mmio_read32(UART_BASE + offset); }
void UARTDriver::putc(char c) {
    while (read_uart_reg(0x18) & (1 << 5)) {} // Wait if TX FIFO full (UARTFR_TXFF)
    write_uart_reg(0x00, static_cast<uint32_t>(c)); // Write to UARTDR
}
void UARTDriver::puts(const char* str) {
    if (!str) return;
    while (*str) putc(*str++);
}
void UARTDriver::uart_put_uint64_hex(uint64_t value) {
    char hex_chars[] = "0123456789ABCDEF";
    char buffer[17] = {0};
    for (int i = 15; i >= 0; --i) {
        buffer[i] = hex_chars[value & 0xF];
        value >>= 4;
    }
    puts(buffer);
}
char UARTDriver::getc_blocking() {
    while (read_uart_reg(0x18) & (1 << 4)) {} // Wait if RX FIFO empty (UARTFR_RXFE)
    return static_cast<char>(read_uart_reg(0x00) & 0xFF); // Read from UARTDR
}

// --- IRQController ---
void IRQController::write_gicd_reg(uint32_t offset, uint32_t value) { mmio_write32(GIC_DISTRIBUTOR_BASE + offset, value); }
uint32_t IRQController::read_gicd_reg(uint32_t offset) { return mmio_read32(GIC_DISTRIBUTOR_BASE + offset); }
void IRQController::write_gicc_reg(uint32_t offset, uint32_t value) { mmio_write32(GIC_CPU_INTERFACE_BASE + offset, value); }
uint32_t IRQController::read_gicc_reg(uint32_t offset) { return mmio_read32(GIC_CPU_INTERFACE_BASE + offset); }

void IRQController::enable_core_irqs(uint32_t /*core_id*/, uint32_t /*irq_source_mask*/) {
    write_gicc_reg(0x0000, read_gicc_reg(0x0000) | 0x1); // GICC_CTLR.EnableGrp0 = 1
}
void IRQController::disable_core_irqs(uint32_t /*core_id*/) {
    write_gicc_reg(0x0000, read_gicc_reg(0x0000) & ~0x3); // GICC_CTLR.EnableGrp0/1 = 0
}
void IRQController::init_distributor() {
    write_gicd_reg(0x000, 0x0); // GICD_CTLR: Disable distributor
    // Set all SPIs (32-1019) to Group 0, default priority, route to core 0, and disable.
    for (uint32_t i = 32; i < 1020; i += 32) { // Iterate over GICD_IGROUPRn registers for SPIs
        write_gicd_reg(0x080 + (i/32 -1)*4, 0x00000000); // All SPIs to Group 0
        write_gicd_reg(0x180 + (i/32 -1)*4, 0xFFFFFFFF); // GICD_ICENABLERn: Disable all SPIs
    }
    for (uint32_t i = 0; i < 1020; i += 4) { // GICD_IPRIORITYRn, 4 IRQs per reg
        write_gicd_reg(0x400 + i, 0xA0A0A0A0); // Default priority A0 for all
    }
    for (uint32_t i = 32; i < 1020; i += 4) { // GICD_ITARGETSRn for SPIs
        write_gicd_reg(0x800 + i, 0x01010101); // Route SPIs to CPU0 (target mask 0x01)
    }
    write_gicd_reg(0x000, 0x1); // GICD_CTLR: Enable Group 0 forwarding
}
void IRQController::init_cpu_interface(uint32_t /*core_id*/) {
    write_gicc_reg(0x004, 0xFF); // GICC_PMR: Allow all priorities
    write_gicc_reg(0x008, 0x03); // GICC_BPR: Group priority can preempt subpriority
    write_gicc_reg(0x000, 0x1);  // GICC_CTLR: Enable Group 0 IRQ signaling
}
uint32_t IRQController::ack_irq(uint32_t /*core_id*/) { return read_gicc_reg(0x00C); /*GICC_IAR*/ }
void IRQController::end_irq(uint32_t /*core_id*/, uint32_t irq_id) { write_gicc_reg(0x010, irq_id); /*GICC_EOIR*/ }
void IRQController::enable_irq_line(uint32_t irq_id) {
    write_gicd_reg(0x100 + (irq_id / 32) * 4, 1U << (irq_id % 32));
}
void IRQController::disable_irq_line(uint32_t irq_id) {
    write_gicd_reg(0x180 + (irq_id / 32) * 4, 1U << (irq_id % 32));
}
void IRQController::set_irq_priority(uint32_t irq_id, uint8_t priority) {
    uint32_t reg_addr = 0x400 + (irq_id / 4) * 4;
    uint32_t shift = (irq_id % 4) * 8;
    uint32_t val = read_gicd_reg(reg_addr);
    val &= ~(0xFFU << shift);
    val |= (static_cast<uint32_t>(priority) << shift);
    write_gicd_reg(reg_addr, val);
}

// --- TimerDriver ---
TimerDriver::TimerDriver() : timer_freq_hz_(0), active_sw_timers_head_(nullptr) {
    timer_freq_hz_ = read_sysreg_cntfrq();
    if (timer_freq_hz_ == 0) timer_freq_hz_ = 62500000; // QEMU default if not set by bootloader
}
void TimerDriver::init_system_timer_properties(uint64_t freq_hz_override) {
    if (freq_hz_override > 0) timer_freq_hz_ = freq_hz_override;
}
void TimerDriver::init_core_timer_interrupt(uint32_t /*core_id*/) {
    uint64_t ticks_for_10ms = (timer_freq_hz_ * 10) / 1000; // Kernel tick e.g. 10ms
    write_sysreg_cntp_tval(ticks_for_10ms);
    write_sysreg_cntp_ctl(1); // Enable timer, unmask interrupt (IMASK=0, ENABLE=1)
}
void TimerDriver::ack_core_timer_interrupt(uint32_t /*core_id*/) {
    uint64_t ticks_for_10ms = (timer_freq_hz_ * 10) / 1000;
    write_sysreg_cntp_tval(ticks_for_10ms); // Writing TVAL implicitly clears IRQ status
}
bool TimerDriver::add_software_timer(kernel::hal::timer::SoftwareTimer* timer) {
    if (!timer) return false;
    kernel::core::ScopedLock lock(sw_timer_lock_);
    timer->next = active_sw_timers_head_;
    active_sw_timers_head_ = timer;
    return true;
}
bool TimerDriver::remove_software_timer(kernel::hal::timer::SoftwareTimer* timer_to_remove) {
    if (!timer_to_remove) return false;
    kernel::core::ScopedLock lock(sw_timer_lock_);
    if (active_sw_timers_head_ == timer_to_remove) {
        active_sw_timers_head_ = timer_to_remove->next;
        return true;
    }
    for (auto curr = active_sw_timers_head_; curr && curr->next; curr = curr->next) {
        if (curr->next == timer_to_remove) {
            curr->next = timer_to_remove->next;
            return true;
        }
    }
    return false;
}
uint64_t TimerDriver::get_system_time_us() {
    if (timer_freq_hz_ == 0) return 0;
    return (read_sysreg_cntpct() * 1000000) / timer_freq_hz_;
}
void TimerDriver::hardware_timer_irq_fired(uint32_t /*core_id*/) {
    kernel::core::ScopedLock lock(sw_timer_lock_);
    uint64_t now_us = get_system_time_us();
    for (auto timer = active_sw_timers_head_; timer != nullptr; timer = timer->next) {
        if (timer->active && now_us >= timer->expiry_time_us) {
            if(timer->callback) timer->callback(timer, timer->context);
            if (timer->period_us > 0) { // Periodic
                timer->expiry_time_us += timer->period_us;
                if (timer->expiry_time_us < now_us) timer->expiry_time_us = now_us + timer->period_us; // Catch up
            } else {
                timer->active = false; // One-shot
            }
        }
    }
}

// --- DMAController (Stub) ---
DMAController::DMAController() { channels_in_use_.fill(false); }
kernel::hal::dma::ChannelID DMAController::request_channel() {
    kernel::core::ScopedLock lock(dma_lock_);
    for (size_t i = 0; i < channels_in_use_.size(); ++i) {
        if (!channels_in_use_[i]) { channels_in_use_[i] = true; return static_cast<kernel::hal::dma::ChannelID>(i); }
    }
    return kernel::hal::dma::INVALID_CHANNEL;
}
bool DMAController::configure_and_start_transfer(kernel::hal::dma::ChannelID ch, const kernel::hal::dma::TransferConfig&, kernel::hal::dma::DMACallback cb, void* ctx) {
    if (ch < 0 || static_cast<size_t>(ch) >= channels_in_use_.size() || !channels_in_use_[static_cast<size_t>(ch)]) return false;
    if (cb) cb(ch, true, ctx); // Simulate immediate success
    return true;
}
void DMAController::release_channel(kernel::hal::dma::ChannelID ch) {
    kernel::core::ScopedLock lock(dma_lock_);
    if (ch >= 0 && static_cast<size_t>(ch) < channels_in_use_.size()) channels_in_use_[static_cast<size_t>(ch)] = false;
}

// --- I2SDriver (Stub) ---
bool I2SDriver::init(uint32_t id, kernel::hal::i2s::Mode, const kernel::hal::i2s::Format& fmt, size_t, uint8_t, kernel::hal::i2s::I2SCallback cb, void* udata) {
    if (id >= instances_.size()) return false;
    instances_[id] = {cb, udata, false, fmt};
    return true;
}
bool I2SDriver::start(uint32_t id) { if (id < instances_.size()) { instances_[id].active = true; return true; } return false; }
bool I2SDriver::stop(uint32_t id) { if (id < instances_.size()) { instances_[id].active = false; return true; } return false; }
kernel::audio::AudioBuffer* I2SDriver::get_buffer_for_app_tx(uint32_t id) {
    if (id >= instances_.size() || !instances_[id].active) return nullptr;
    auto* buf = new (std::nothrow) kernel::audio::AudioBuffer();
    if (!buf) return nullptr;
    buf->samples_per_channel = 256; // Example
    buf->channels = instances_[id].current_format.num_channels;
    size_t frame_size = instances_[id].current_format.get_bytes_per_frame();
    buf->size_bytes_raw_buffer = buf->samples_per_channel * frame_size;
    buf->data_raw_i2s = new (std::nothrow) uint8_t[buf->size_bytes_raw_buffer](); // Zero initialized
    // DSP buffer typically float, deinterleaved.
    buf->data_dsp_canonical = new (std::nothrow) float[buf->samples_per_channel * buf->channels]();
    if (!buf->data_raw_i2s || !buf->data_dsp_canonical) { /* cleanup partial alloc */ delete buf; return nullptr; }
    return buf;
}
bool I2SDriver::submit_filled_buffer_to_hw_tx(uint32_t id, kernel::audio::AudioBuffer* buffer) {
    if (id >= instances_.size() || !instances_[id].active || !buffer) return false;
    // In a real driver, DMA would be started. Here, we simulate by calling back.
    if (instances_[id].callback) {
        instances_[id].callback(id, buffer, kernel::hal::i2s::Mode::MASTER_TX, instances_[id].user_data);
    }
    return true;
}
void I2SDriver::release_processed_buffer_to_hw_rx(uint32_t, kernel::audio::AudioBuffer* buffer) {
    if (buffer) {
        delete[] static_cast<uint8_t*>(buffer->data_raw_i2s);
        delete[] buffer->data_dsp_canonical;
        delete buffer;
    }
}
void I2SDriver::convert_hw_format_to_dsp_format(kernel::audio::AudioBuffer* buf, const kernel::hal::i2s::Format& fmt) {
    if (!buf || !buf->data_raw_i2s || !buf->data_dsp_canonical) return;
    // Example: 16-bit stereo interleaved PCM to float deinterleaved
    // This is a placeholder. Actual conversion depends heavily on format and NEON/DSP usage.
    auto* pcm_data = static_cast<int16_t*>(buf->data_raw_i2s);
    for (size_t i = 0; i < buf->samples_per_channel; ++i) {
        for (uint8_t ch = 0; ch < fmt.num_channels; ++ch) {
            buf->data_dsp_canonical[i * fmt.num_channels + ch] = static_cast<float>(pcm_data[i * fmt.num_channels + ch]) / 32768.0f;
        }
    }
}
void I2SDriver::convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buf, const kernel::hal::i2s::Format& fmt) {
     if (!buf || !buf->data_raw_i2s || !buf->data_dsp_canonical) return;
    // Example: float deinterleaved to 16-bit stereo interleaved PCM
    auto* pcm_data = static_cast<int16_t*>(buf->data_raw_i2s);
    for (size_t i = 0; i < buf->samples_per_channel; ++i) {
        for (uint8_t ch = 0; ch < fmt.num_channels; ++ch) {
            float val = buf->data_dsp_canonical[i * fmt.num_channels + ch];
            val = std::max(-1.0f, std::min(1.0f, val)); // Clamp
            pcm_data[i * fmt.num_channels + ch] = static_cast<int16_t>(val * 32767.0f);
        }
    }
}

// --- MemoryOps ---
void MemoryOps::flush_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    uintptr_t start_addr = reinterpret_cast<uintptr_t>(addr);
    uintptr_t end_addr = start_addr + size;
    // Assuming 64-byte cache line size for Cortex-A53 (typical L1 D-cache)
    const size_t cache_line_size = 64; 
    for (uintptr_t current_addr = start_addr & ~(cache_line_size - 1); current_addr < end_addr; current_addr += cache_line_size) {
        asm volatile("dc cvac, %0" : : "r"(current_addr) : "memory"); // Clean and Invalidate by VA to PoC
    }
    asm volatile("dsb sy" ::: "memory"); // Ensure completion of cache operations
    asm volatile("isb" ::: "memory");    // Ensure instruction synchronization
}
void MemoryOps::invalidate_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    uintptr_t start_addr = reinterpret_cast<uintptr_t>(addr);
    uintptr_t end_addr = start_addr + size;
    const size_t cache_line_size = 64;
    for (uintptr_t current_addr = start_addr & ~(cache_line_size - 1); current_addr < end_addr; current_addr += cache_line_size) {
        asm volatile("dc ivac, %0" : : "r"(current_addr) : "memory"); // Invalidate by VA to PoC
    }
    asm volatile("dsb sy" ::: "memory");
    asm volatile("isb" ::: "memory");
}

// --- NetworkDriver (Stub for VirtIO) ---
bool NetworkDriver::init_interface(int /*if_idx*/) {
    // Stub: QEMU virtio-net setup is complex.
    // A real driver would map MMIO, discover features, set up virtqueues.
    initialized_ = true;
    return true;
}
bool NetworkDriver::send_packet(int /*if_idx*/, const uint8_t* /*data*/, size_t /*len*/) {
    if (!initialized_) return false;
    // Stub: Real driver adds to TX virtqueue, notifies hypervisor.
    return true;
}
void NetworkDriver::register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb, void* context) {
    packet_received_cb_ = cb;
    cb_context_ = context;
}

// --- PowerOps ---
void PowerOps::enter_idle_state(uint32_t /*core_id*/) { asm volatile("wfi"); } // Wait For Interrupt
bool PowerOps::set_cpu_frequency(uint32_t /*core_id*/, uint32_t /*freq_hz*/) { return true; /*NOP on QEMU*/ }

// --- GPIODriver (Stub) ---
bool GPIODriver::init_bank(uint32_t) { return true; }
bool GPIODriver::configure_pin(uint32_t, uint32_t, kernel::hal::gpio::PinMode) { return true; }
bool GPIODriver::set_pin_state(uint32_t, uint32_t, kernel::hal::gpio::PinState) { return true; }
kernel::hal::gpio::PinState GPIODriver::read_pin_state(uint32_t, uint32_t) { return kernel::hal::gpio::PinState::LOW; }
void GPIODriver::enable_interrupt(uint32_t, uint32_t, bool) {}

// --- WatchdogDriver (using PL031 RTC as a simple countdown) ---
bool WatchdogDriver::start_watchdog(uint32_t timeout_ms) {
    // PL031 doesn't have a direct watchdog mode. Simulate by setting a match value.
    // This is a conceptual watchdog using RTC match.
    uint32_t current_time = mmio_read32(RTC_BASE + 0x00); // RTCDR
    uint32_t match_value = current_time + (timeout_ms / 1000); // Rough conversion to seconds
    mmio_write32(RTC_BASE + 0x04, match_value); // RTCMR (Match Register)
    mmio_write32(RTC_BASE + 0x0C, 1); // RTCICR: Enable match interrupt (conceptual enable)
    return true;
}
bool WatchdogDriver::reset_watchdog() {
    // Conceptual: Re-calculate match time based on new current time.
    // This stub doesn't implement full watchdog logic.
    // For a real watchdog, this would "pet" it. Here, maybe just extend match time.
    uint32_t timeout_ms = 5000; // Assume a fixed timeout for re-setting
    uint32_t current_time = mmio_read32(RTC_BASE + 0x00);
    uint32_t match_value = current_time + (timeout_ms / 1000);
    mmio_write32(RTC_BASE + 0x04, match_value);
    return true;
}
bool WatchdogDriver::stop_watchdog() {
    mmio_write32(RTC_BASE + 0x0C, 0); // RTCICR: Disable match interrupt
    return true;
}

// --- PlatformQEMUVirtARM64 ---
PlatformQEMUVirtARM64::PlatformQEMUVirtARM64() {
    // Constructor can be empty if drivers are statically initialized
    // or do minimal setup here.
}
uint32_t PlatformQEMUVirtARM64::get_core_id() const {
    uint64_t mpidr;
    asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    return static_cast<uint32_t>(mpidr & 0xFF); // Affinity Level 0 (Core ID)
}
uint32_t PlatformQEMUVirtARM64::get_num_cores() const { return kernel::core::MAX_CORES; /*Fixed for this QEMU config*/ }

void PlatformQEMUVirtARM64::early_init_platform() {
    // Initialize UART for early messages
    // GIC distributor initialization
    irq_controller_.init_distributor();
    // Timer properties (e.g. frequency)
    timer_driver_.init_system_timer_properties(); 
    // Any other platform-wide setup before cores are fully up.
    uart_driver_.puts("[miniOS HAL] QEMU ARM64 Platform Early Init Done.\n");
}
void PlatformQEMUVirtARM64::early_init_core(uint32_t core_id) {
    // Initialize GIC CPU interface for this core
    irq_controller_.init_cpu_interface(core_id);
    // Initialize core's local timer for preemptive ticks
    timer_driver_.init_core_timer_interrupt(core_id);
    // Other per-core setup (e.g. enable FPU/NEON if not done by bootloader)
    // Enable NEON/FP: clear CPACR_EL1.FPEN (bits 21:20) to 0b00 (trap to EL1),
    // then in EL1, set CPACR_EL1.FPEN to 0b11 (no trap).
    // For EL0 access, ensure EL1 doesn't trap.
    uint64_t cpacr_el1;
    asm volatile("mrs %0, cpacr_el1" : "=r"(cpacr_el1));
    cpacr_el1 |= (3ULL << 20); // Set FPEN bits[21:20] to 0b11 (enable EL0/EL1 access)
    asm volatile("msr cpacr_el1, %0" : : "r"(cpacr_el1));
    asm volatile("isb");

    char buf[64];
    std::snprintf(buf, sizeof(buf), "[miniOS HAL] Core %u Early Init Done.\n", core_id);
    uart_driver_.puts(buf);
}

[[noreturn]] void PlatformQEMUVirtARM64::panic(const char* msg, const char* file, int line) {
    // Attempt to print panic message via UART.
    // Disable interrupts on current core first.
    asm volatile("msr daifset, #0xf" ::: "memory"); // Disable all IRQs/FIQs etc.

    uart_driver_.puts("\n*** KERNEL PANIC ***\n");
    if (msg) { uart_driver_.puts("Message: "); uart_driver_.puts(msg); uart_driver_.puts("\n"); }
    if (file) { uart_driver_.puts("File: "); uart_driver_.puts(file); }
    char line_buf[16];
    std::snprintf(line_buf, sizeof(line_buf), ":%d", line);
    uart_driver_.puts(line_buf);
    uart_driver_.puts("\nCore: ");
    char core_buf[4];
    std::snprintf(core_buf, sizeof(core_buf), "%u", get_core_id());
    uart_driver_.puts(core_buf);
    uart_driver_.puts("\nHalting.\n");

    // Infinite loop to halt the system (or specific core).
    while (true) { asm volatile("wfi"); }
}
void PlatformQEMUVirtARM64::reboot_system() {
    // QEMU virt machine doesn't have a standard software reboot mechanism easily triggerable from guest.
    // For a real board, this would use a reset controller.
    // For QEMU, typically you'd exit QEMU or use monitor commands.
    panic("Reboot requested but not implemented for QEMU virt.", __FILE__, __LINE__);
}

} // namespace hal::qemu_virt_arm64