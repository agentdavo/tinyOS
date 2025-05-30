// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_arm64.cpp
 * @brief ARM64 HAL implementation for QEMU virt platform in miniOS v1.7.
 */

#include "hal_qemu_arm64.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include <cstring>   
#include <algorithm>  

#if defined(__aarch64__) && !defined(__ARM_NEON)
#define __ARM_NEON 1
#endif
#include <arm_neon.h>

extern "C" void early_uart_puts(const char* str);

namespace hal::qemu_virt_arm64 {

PlatformQEMUVirtARM64 g_platform_instance;

inline void mmio_write32(uint64_t addr, uint32_t value) {
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}
inline uint32_t mmio_read32(uint64_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}
inline void write_sysreg_cntp_tval(uint64_t value) { asm volatile("msr cntp_tval_el0, %0" : : "r"(value)); }
inline void write_sysreg_cntp_ctl(uint64_t value) { asm volatile("msr cntp_ctl_el0, %0" : : "r"(value)); }
inline uint64_t read_sysreg_cntpct() { uint64_t v; asm volatile("mrs %0, cntpct_el0" : "=r"(v)); return v; }
inline uint64_t read_sysreg_cntfrq() { uint64_t v; asm volatile("mrs %0, cntfrq_el0" : "=r"(v)); return v; }

// --- UARTDriver ---
void UARTDriver::write_uart_reg(uint32_t offset, uint32_t value) { mmio_write32(UART_BASE + offset, value); }
uint32_t UARTDriver::read_uart_reg(uint32_t offset) { return mmio_read32(UART_BASE + offset); }
void UARTDriver::putc(char c) {
    while (read_uart_reg(0x18) & (1 << 5)) {} 
    write_uart_reg(0x00, static_cast<uint32_t>(c)); 
}
void UARTDriver::puts(const char* str) {
    if (!str) return;
    while (*str) {
        if (*str == '\n') this->putc('\r'); 
        this->putc(*str++);
    }
}
void UARTDriver::uart_put_uint64_hex(uint64_t value) {
    char hex_chars[] = "0123456789ABCDEF";
    char buffer[17] = {0}; 
    buffer[16] = '\0';
    for (int i = 15; i >= 0; --i) {
        buffer[i] = hex_chars[value & 0xF];
        value >>= 4;
    }
    this->puts(buffer); 
}
char UARTDriver::getc_blocking() {
    while (read_uart_reg(0x18) & (1 << 4)) {} 
    return static_cast<char>(read_uart_reg(0x00) & 0xFF); 
}

// --- IRQController ---
void IRQController::write_gicd_reg(uint32_t o, uint32_t v) { mmio_write32(GIC_DISTRIBUTOR_BASE + o, v); }
uint32_t IRQController::read_gicd_reg(uint32_t o) { return mmio_read32(GIC_DISTRIBUTOR_BASE + o); }
void IRQController::write_gicc_reg(uint64_t o, uint32_t v) { mmio_write32(GIC_CPU_INTERFACE_BASE + o, v); }
uint32_t IRQController::read_gicc_reg(uint64_t o) { return mmio_read32(GIC_CPU_INTERFACE_BASE + o); }

void IRQController::enable_core_irqs(uint32_t core_id, uint32_t irq_source_mask) {
    (void)core_id; (void)irq_source_mask;
    write_gicc_reg(0x0000, read_gicc_reg(0x0000) | 0x1); 
}
void IRQController::disable_core_irqs(uint32_t core_id) {
    (void)core_id;
    write_gicc_reg(0x0000, read_gicc_reg(0x0000) & ~0x3); 
}
void IRQController::init_distributor() {
    early_uart_puts("[HAL_DEBUG] IRQController::init_distributor() ENTRY\n"); 
    write_gicd_reg(0x000, 0x0); 
    for (uint32_t i = 32; i < 1020; i += 32) { 
        write_gicd_reg(0x080 + ((i/32) -1)*4, 0x00000000); 
        write_gicd_reg(0x180 + ((i/32) -1)*4, 0xFFFFFFFF); 
    }
    for (uint32_t i = 0; i < 1020; i += 4) { 
        write_gicd_reg(0x400 + i, 0xA0A0A0A0); 
    }
    for (uint32_t i = 32; i < 1020; i += 4) { 
        write_gicd_reg(0x800 + (i-32), 0x01010101); 
    }
    write_gicd_reg(0x000, 0x1); 
    early_uart_puts("[HAL_DEBUG] IRQController::init_distributor() EXIT\n"); 
}
void IRQController::init_cpu_interface(uint32_t core_id) { 
    (void)core_id;
    early_uart_puts("[HAL_DEBUG] IRQController::init_cpu_interface() ENTRY\n");
    write_gicc_reg(0x004, 0xFF); 
    write_gicc_reg(0x008, 0x03); 
    write_gicc_reg(0x000, 0x1);  
    early_uart_puts("[HAL_DEBUG] IRQController::init_cpu_interface() EXIT\n");
}
uint32_t IRQController::ack_irq(uint32_t core_id) { (void)core_id; return read_gicc_reg(0x00C); }
void IRQController::end_irq(uint32_t core_id, uint32_t irq_id) { (void)core_id; write_gicc_reg(0x010, irq_id); }
void IRQController::enable_irq_line(uint32_t irq_id) {
    write_gicd_reg(0x100 + (irq_id / 32) * 4, 1U << (irq_id % 32));
}
void IRQController::disable_irq_line(uint32_t irq_id) {
    write_gicd_reg(0x180 + (irq_id / 32) * 4, 1U << (irq_id % 32));
}
void IRQController::set_irq_priority(uint32_t irq_id, uint8_t priority) {
    uint32_t reg_offset = 0x400 + (irq_id / 4) * 4; 
    uint32_t shift = (irq_id % 4) * 8; 
    uint32_t val = read_gicd_reg(reg_offset);
    val &= ~(0xFFU << shift); 
    val |= (static_cast<uint32_t>(priority) << shift); 
    write_gicd_reg(reg_offset, val);
}

// --- TimerDriver ---
TimerDriver::TimerDriver() : timer_freq_hz_(0), active_sw_timers_head_(nullptr) {
    early_uart_puts("[HAL_DEBUG] TimerDriver::TimerDriver() CONSTRUCTOR ENTRY\n");
    timer_freq_hz_ = read_sysreg_cntfrq();
    if (timer_freq_hz_ == 0) {
        early_uart_puts("[HAL_DEBUG] CNTFRQ_EL0 is 0, using default 62.5MHz for QEMU virt\n");
        timer_freq_hz_ = 62500000; 
    } else {
        early_uart_puts("[HAL_DEBUG] CNTFRQ_EL0 read by TimerDriver constructor.\n"); 
    }
    early_uart_puts("[HAL_DEBUG] TimerDriver::TimerDriver() CONSTRUCTOR EXIT\n");
}
void TimerDriver::init_system_timer_properties(uint64_t freq_hz_override) {
    early_uart_puts("[HAL_DEBUG] TimerDriver::init_system_timer_properties() ENTRY\n");
    if (freq_hz_override > 0) timer_freq_hz_ = freq_hz_override;
    early_uart_puts("[HAL_DEBUG] TimerDriver::init_system_timer_properties() EXIT\n");
}
void TimerDriver::init_core_timer_interrupt(uint32_t core_id) { 
    (void)core_id;
    early_uart_puts("[HAL_DEBUG] TimerDriver::init_core_timer_interrupt() ENTRY\n");
    uint64_t ticks_for_period = (timer_freq_hz_ * 10) / 1000; 
    write_sysreg_cntp_tval(ticks_for_period); 
    write_sysreg_cntp_ctl(1); 
    early_uart_puts("[HAL_DEBUG] TimerDriver::init_core_timer_interrupt() EXIT\n");
}
void TimerDriver::ack_core_timer_interrupt(uint32_t core_id) { 
    (void)core_id;
    uint64_t ticks_for_period = (timer_freq_hz_ * 10) / 1000; 
    write_sysreg_cntp_tval(ticks_for_period);
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
        timer_to_remove->next = nullptr; 
        return true;
    }
    kernel::hal::timer::SoftwareTimer* current = active_sw_timers_head_;
    while (current && current->next) {
        if (current->next == timer_to_remove) {
            current->next = timer_to_remove->next;
            timer_to_remove->next = nullptr; 
            return true;
        }
        current = current->next;
    }
    return false;
}
uint64_t TimerDriver::get_system_time_us() { 
    if (timer_freq_hz_ == 0) return 0; 
    return (read_sysreg_cntpct() * 1000000ULL) / timer_freq_hz_;
}
void TimerDriver::hardware_timer_irq_fired(uint32_t core_id) { 
    (void)core_id;
    kernel::core::ScopedLock lock(sw_timer_lock_);
    uint64_t now_us = get_system_time_us();
    kernel::hal::timer::SoftwareTimer* timer = active_sw_timers_head_;
    while (timer) {
        if (timer->active && now_us >= timer->expiry_time_us) {
            if (timer->callback) {
                timer->callback(timer, timer->context);
            }
            if (timer->period_us > 0) { 
                timer->expiry_time_us += timer->period_us; 
                if (timer->expiry_time_us < now_us) { 
                    timer->expiry_time_us = now_us + timer->period_us;
                }
            } else {
                timer->active = false; 
            }
        }
        timer = timer->next;
    }
}

// --- DMAController ---
DMAController::DMAController() { 
    early_uart_puts("[DEBUG] DMAController CONSTRUCTOR ENTRY\n");
    channels_in_use_.fill(false);
    early_uart_puts("[DEBUG] DMAController CONSTRUCTOR EXIT\n");
}
kernel::hal::dma::ChannelID DMAController::request_channel() {
    kernel::core::ScopedLock lock(dma_lock_);
    for (size_t i = 0; i < channels_in_use_.size(); ++i) {
        if (!channels_in_use_[i]) { channels_in_use_[i] = true; return static_cast<kernel::hal::dma::ChannelID>(i); }
    }
    return kernel::hal::dma::INVALID_CHANNEL;
}
bool DMAController::configure_and_start_transfer(kernel::hal::dma::ChannelID ch, const kernel::hal::dma::TransferConfig&, kernel::hal::dma::DMACallback cb, void* ctx) {
    if (ch < 0 || static_cast<size_t>(ch) >= channels_in_use_.size() || !channels_in_use_[static_cast<size_t>(ch)]) return false;
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
         kernel::g_platform->get_uart_ops()->puts("[DMA STUB] Configure and start transfer.\n");
    }
    if (cb) cb(ch, true, ctx); 
    return true;
}
void DMAController::release_channel(kernel::hal::dma::ChannelID ch) {
    kernel::core::ScopedLock lock(dma_lock_);
    if (ch >= 0 && static_cast<size_t>(ch) < channels_in_use_.size()) channels_in_use_[static_cast<size_t>(ch)] = false;
}

// --- I2SDriver ---
bool I2SDriver::init(uint32_t id, kernel::hal::i2s::Mode, const kernel::hal::i2s::Format& fmt, size_t, uint8_t, kernel::hal::i2s::I2SCallback cb, void* udata) {
    if (id >= instances_.size()) return false;
    instances_[id] = {cb, udata, false, fmt};
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
         kernel::g_platform->get_uart_ops()->puts("[I2S STUB] Initialized.\n");
    }
    return true;
}
bool I2SDriver::start(uint32_t id) { 
    if (id < instances_.size()) { 
        instances_[id].active = true; 
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
            kernel::g_platform->get_uart_ops()->puts("[I2S STUB] Started.\n");
        }
        return true; 
    } 
    return false; 
}
bool I2SDriver::stop(uint32_t id) { 
    if (id < instances_.size()) { 
        instances_[id].active = false; 
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
            kernel::g_platform->get_uart_ops()->puts("[I2S STUB] Stopped.\n");
        }
        return true; 
    } 
    return false; 
}
kernel::audio::AudioBuffer* I2SDriver::get_buffer_for_app_tx(uint32_t id) {
    if (id >= instances_.size() || !instances_[id].active) return nullptr;
    auto* buf = new (std::nothrow) kernel::audio::AudioBuffer();
    if (!buf) return nullptr;
    const auto& format = instances_[id].current_format;
    buf->samples_per_channel = 256; 
    buf->channels = format.num_channels;
    size_t frame_size = format.get_bytes_per_frame();
    buf->size_bytes_raw_buffer = buf->samples_per_channel * frame_size;
    buf->data_raw_i2s = new (std::nothrow) uint8_t[buf->size_bytes_raw_buffer](); 
    buf->data_dsp_canonical = new (std::nothrow) float[buf->samples_per_channel * buf->channels]();
    if (!buf->data_raw_i2s || !buf->data_dsp_canonical) { 
        delete[] static_cast<uint8_t*>(buf->data_raw_i2s); 
        delete[] buf->data_dsp_canonical;
        delete buf; 
        return nullptr; 
    }
    return buf;
}
bool I2SDriver::submit_filled_buffer_to_hw_tx(uint32_t id, kernel::audio::AudioBuffer* buffer) {
    if (id >= instances_.size() || !instances_[id].active || !buffer) return false;
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
    auto* pcm_data = static_cast<int16_t*>(buf->data_raw_i2s); 
    size_t num_samples_total = buf->samples_per_channel * fmt.num_channels;
    if (fmt.bit_depth == kernel::hal::i2s::BitDepth::BITS_16) {
        for (size_t i = 0; i < num_samples_total; ++i) {
            buf->data_dsp_canonical[i] = static_cast<float>(pcm_data[i]) / 32768.0f;
        }
    } 
}
void I2SDriver::convert_dsp_format_to_hw_format(kernel::audio::AudioBuffer* buf, const kernel::hal::i2s::Format& fmt) {
    if (!buf || !buf->data_raw_i2s || !buf->data_dsp_canonical) return;
    auto* pcm_data = static_cast<int16_t*>(buf->data_raw_i2s); 
    size_t num_samples_total = buf->samples_per_channel * fmt.num_channels;
    if (fmt.bit_depth == kernel::hal::i2s::BitDepth::BITS_16) {
        for (size_t i = 0; i < num_samples_total; ++i) {
            float val = buf->data_dsp_canonical[i];
            val = kernel::util::max(-1.0f, kernel::util::min(1.0f, val)); 
            pcm_data[i] = static_cast<int16_t>(val * 32767.0f);
        }
    } 
}

// --- MemoryOps ---
void MemoryOps::flush_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    uintptr_t current_addr = reinterpret_cast<uintptr_t>(addr);
    uintptr_t end_addr = current_addr + size;
    const size_t cache_line_size = 64; 
    current_addr &= ~(cache_line_size - 1); 
    for (; current_addr < end_addr; current_addr += cache_line_size) {
        asm volatile("dc cvac, %0" : : "r"(current_addr) : "memory"); 
    }
    asm volatile("dsb sy" ::: "memory"); 
    asm volatile("isb" ::: "memory");    
}
void MemoryOps::invalidate_cache_range(const void* addr, size_t size) {
    if (!addr || size == 0) return;
    uintptr_t current_addr = reinterpret_cast<uintptr_t>(addr);
    uintptr_t end_addr = current_addr + size;
    const size_t cache_line_size = 64;
    current_addr &= ~(cache_line_size - 1);
    for (; current_addr < end_addr; current_addr += cache_line_size) {
        asm volatile("dc ivac, %0" : : "r"(current_addr) : "memory"); 
    }
    asm volatile("dsb sy" ::: "memory");
    asm volatile("isb" ::: "memory");
}

// --- NetworkDriver ---
bool NetworkDriver::init_interface(int) { initialized_ = true; return true; }
bool NetworkDriver::send_packet(int, const uint8_t*, size_t) { if (!initialized_) return false; return true; }
void NetworkDriver::register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb, void* context) {
    packet_received_cb_ = cb; cb_context_ = context;
}

// --- PowerOps ---
void PowerOps::enter_idle_state(uint32_t) { asm volatile("wfi"); } 
bool PowerOps::set_cpu_frequency(uint32_t, uint32_t) { return true; }

// --- GPIODriver ---
bool GPIODriver::init_bank(uint32_t) { return true; }
bool GPIODriver::configure_pin(uint32_t, uint32_t, kernel::hal::gpio::PinMode) { return true; }
bool GPIODriver::set_pin_state(uint32_t, uint32_t, kernel::hal::gpio::PinState) { return true; }
kernel::hal::gpio::PinState GPIODriver::read_pin_state(uint32_t, uint32_t) { return kernel::hal::gpio::PinState::LOW; }
void GPIODriver::enable_interrupt(uint32_t, uint32_t, bool) {}

// --- WatchdogDriver ---
bool WatchdogDriver::start_watchdog(uint32_t timeout_ms) {
    uint32_t current_time = mmio_read32(RTC_BASE + 0x00); 
    uint32_t match_value = current_time + (timeout_ms / 1000); 
    mmio_write32(RTC_BASE + 0x04, match_value); 
    mmio_write32(RTC_BASE + 0x0C, 1); 
    return true;
}
bool WatchdogDriver::reset_watchdog() {
    uint32_t timeout_ms = 5000; 
    uint32_t current_time = mmio_read32(RTC_BASE + 0x00);
    uint32_t match_value = current_time + (timeout_ms / 1000);
    mmio_write32(RTC_BASE + 0x04, match_value);
    return true;
}
bool WatchdogDriver::stop_watchdog() {
    mmio_write32(RTC_BASE + 0x0C, 0); 
    return true;
}

// --- PlatformQEMUVirtARM64 ---
PlatformQEMUVirtARM64::PlatformQEMUVirtARM64() :
    uart_driver_(),
    irq_controller_(),
    timer_driver_(),
    dma_controller_(),
    i2s_driver_(),
    memory_ops_(),
    network_driver_(),
    power_ops_(),
    gpio_driver_(),
    watchdog_driver_()
{
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR START\n");
    unsigned long long vtable_addr = *(unsigned long long*)this;
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "[HAL_DEBUG] Constructor vtable: 0x%llx\n", vtable_addr);
    early_uart_puts(addr_buf);
    if (vtable_addr == 0) {
        early_uart_puts("[HAL_DEBUG] ERROR: Null vtable in constructor\n");
    }
    early_uart_puts("[HAL_DEBUG] Initializing uart_driver_\n");
    early_uart_puts("[HAL_DEBUG] Initializing irq_controller_\n");
    early_uart_puts("[HAL_DEBUG] Initializing timer_driver_\n");
    early_uart_puts("[HAL_DEBUG] Initializing dma_controller_\n");
    early_uart_puts("[HAL_DEBUG] Initializing i2s_driver_\n");
    early_uart_puts("[HAL_DEBUG] Initializing memory_ops_\n");
    early_uart_puts("[HAL_DEBUG] Initializing network_driver_\n");
    early_uart_puts("[HAL_DEBUG] Initializing power_ops_\n");
    early_uart_puts("[HAL_DEBUG] Initializing gpio_driver_\n");
    early_uart_puts("[HAL_DEBUG] Initializing watchdog_driver_\n");
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR: Members initialized\n");
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR EXIT\n");
}

PlatformQEMUVirtARM64::~PlatformQEMUVirtARM64() {
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64 DESTRUCTOR\n");
}

uint32_t PlatformQEMUVirtARM64::get_core_id() const {
    uint64_t mpidr;
    asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    return static_cast<uint32_t>(mpidr & 0xFF); 
}
uint32_t PlatformQEMUVirtARM64::get_num_cores() const { return kernel::core::MAX_CORES; }

void PlatformQEMUVirtARM64::early_init_platform() {
    unsigned long long vtable_addr = *(unsigned long long*)this;
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "[HAL_DEBUG] early_init_platform vtable: 0x%llx\n", vtable_addr);
    early_uart_puts(addr_buf);
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_platform() ENTRY\n");
    early_uart_puts("[HAL_DEBUG] Preparing to call irq_controller_.init_distributor()\n");
    irq_controller_.init_distributor();
    early_uart_puts("[HAL_DEBUG] Returned from irq_controller_.init_distributor()\n");
    early_uart_puts("[HAL_DEBUG] Calling timer_driver_.init_system_timer_properties()\n");
    timer_driver_.init_system_timer_properties(); 
    early_uart_puts("[HAL_DEBUG] Returned from timer_driver_.init_system_timer_properties()\n");
    if (kernel::g_platform && kernel::g_platform->get_uart_ops() == &uart_driver_) {
        uart_driver_.puts("[miniOS HAL] QEMU ARM64 Platform Early Init Done (via object)\n");
    } else { 
        early_uart_puts("[miniOS HAL] QEMU ARM64 Platform Early Init Done (via early_uart)\n");
    }
    early_uart_puts("[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_platform() EXIT\n");
}

void PlatformQEMUVirtARM64::early_init_core(uint32_t core_id) { 
    char buf[80]; 
    kernel::util::k_snprintf(buf, sizeof(buf), "[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_core(%u) ENTRY\n", core_id);
    early_uart_puts(buf);
    irq_controller_.init_cpu_interface(core_id);
    timer_driver_.init_core_timer_interrupt(core_id);
    uint64_t cpacr_el1;
    asm volatile("mrs %0, cpacr_el1" : "=r"(cpacr_el1));
    cpacr_el1 |= (3ULL << 20); 
    asm volatile("msr cpacr_el1, %0" : : "r"(cpacr_el1));
    asm volatile("isb");
    kernel::util::k_snprintf(buf, sizeof(buf), "[miniOS HAL] Core %u Early Init Done.\n", core_id);
    if (kernel::g_platform && kernel::g_platform->get_uart_ops() == &uart_driver_) {
        uart_driver_.puts(buf);
    } else {
        early_uart_puts(buf);
    }
    kernel::util::k_snprintf(buf, sizeof(buf), "[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_core(%u) EXIT\n", core_id);
    early_uart_puts(buf);
}

[[noreturn]] void PlatformQEMUVirtARM64::panic(const char* msg, const char* file, int line) { 
    asm volatile("msr daifset, #0xf" ::: "memory"); 
    early_uart_puts("\n*** KERNEL PANIC ***\n");
    if (msg) { early_uart_puts("Message: "); early_uart_puts(msg); early_uart_puts("\n"); }
    if (file) { early_uart_puts("File: "); early_uart_puts(file); }
    char line_buf[16];
    kernel::util::k_snprintf(line_buf, sizeof(line_buf), ":%d", line); 
    early_uart_puts(line_buf);
    early_uart_puts("\nCore: ");
    early_uart_puts("\nHalting.\n");
    while (true) { asm volatile("wfi"); }
}
void PlatformQEMUVirtARM64::reboot_system() { 
    panic("Reboot requested but not implemented for QEMU virt.", __FILE__, __LINE__);
}

} // namespace hal::qemu_virt_arm64

namespace hal {
kernel::hal::Platform* get_platform() {
    early_uart_puts("[DEBUG] get_platform ENTRY\n");
    early_uart_puts("[DEBUG] Returning platform instance at 0x");
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "%llx\n", (unsigned long long)&qemu_virt_arm64::g_platform_instance);
    early_uart_puts(addr_buf);
    return &qemu_virt_arm64::g_platform_instance;
}
} // namespace hal