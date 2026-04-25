// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal_qemu_arm64.cpp
 * @brief ARM64 HAL implementation for QEMU virt platform in miniOS v1.7.
 */

#include "hal_qemu_arm64.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include "rt_wait.hpp"
#include "hal/shared/fdt_scan.hpp"
#include <cstring>
#include <algorithm>
#include <new>

#if defined(__aarch64__) && !defined(__ARM_NEON)
#define __ARM_NEON 1
#endif
#include <arm_neon.h>

extern "C" void early_uart_puts(const char* str);
extern "C" void early_uart_lock_acquire();
extern "C" void early_uart_lock_release();
extern "C" uint64_t g_arm64_dtb_ptr;

// Gate the init-path debug noise behind a single flag. The per-call strings
// hold the UART lock for ~500 µs each, and there's ~50 of them — that's
// ~25 ms of UART dominance at boot, which in turn shows up as giant
// max_interval_ns spikes in the RT threads' jitter trackers. Off by default.
#ifndef MINIOS_VERBOSE_BOOT
#  define MINIOS_VERBOSE_BOOT 0
#endif
#if MINIOS_VERBOSE_BOOT
#  define HAL_VDBG(s) early_uart_puts(s)
#else
#  define HAL_VDBG(s) do { } while (0)
#endif

namespace hal::qemu_virt_arm64 {

// Scheduler tick period. Keep this tight: EtherCAT cycles at 1 ms, so a 200 us
// tick gives 5 preemption points per EC cycle with room for jitter.
static constexpr uint64_t TIMER_TICK_US = 200;

alignas(PlatformQEMUVirtARM64) static unsigned char g_platform_storage[sizeof(PlatformQEMUVirtARM64)];
static bool g_platform_constructed = false;

PlatformQEMUVirtARM64& platform_instance() {
    if (!g_platform_constructed) {
        new (g_platform_storage) PlatformQEMUVirtARM64();
        g_platform_constructed = true;
    }
    return *reinterpret_cast<PlatformQEMUVirtARM64*>(g_platform_storage);
}

} // namespace hal::qemu_virt_arm64

// HAL hook implementing kernel::hal::get_platform_instance(). Keeps the generic
// hal.cpp free of platform-specific names.
namespace kernel::hal {
Platform& get_platform_instance() { return ::hal::qemu_virt_arm64::platform_instance(); }
}

namespace hal::qemu_virt_arm64 {

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
    // Share the early_uart_puts lock — both target PL011 @ UART_BASE, and
    // mixing them unlocked shreds any multi-caller log line (boot + [ui] +
    // [sched] + [virtio-gpu] + [hmi] all hit this path from different cores).
    early_uart_lock_acquire();
    while (*str) {
        if (*str == '\n') this->putc('\r');
        this->putc(*str++);
    }
    early_uart_lock_release();
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
    HAL_VDBG("[HAL_DEBUG] IRQController::init_distributor() ENTRY\n"); 
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
    HAL_VDBG("[HAL_DEBUG] IRQController::init_distributor() EXIT\n"); 
}
void IRQController::init_cpu_interface(uint32_t core_id) { 
    (void)core_id;
    HAL_VDBG("[HAL_DEBUG] IRQController::init_cpu_interface() ENTRY\n");
    write_gicc_reg(0x004, 0xFF); 
    write_gicc_reg(0x008, 0x03); 
    write_gicc_reg(0x000, 0x1);  
    HAL_VDBG("[HAL_DEBUG] IRQController::init_cpu_interface() EXIT\n");
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
void IRQController::set_irq_affinity(uint32_t irq_id, uint32_t core_mask) {
    // GICv2 GICD_ITARGETSR<n>: 8 bits per SPI ID, four SPIs per 32-bit word.
    // Targets banked PPIs (id < 32) are read-only — silently skip.
    if (irq_id < 32) return;
    uint32_t reg_offset = 0x800 + (irq_id & ~3u);
    uint32_t shift      = (irq_id & 3u) * 8u;
    uint32_t val = read_gicd_reg(reg_offset);
    val &= ~(0xFFu << shift);
    val |= (core_mask & 0xFFu) << shift;
    write_gicd_reg(reg_offset, val);
}

// --- TimerDriver ---
TimerDriver::TimerDriver() : timer_freq_hz_(0), active_sw_timers_head_(nullptr) {
    HAL_VDBG("[HAL_DEBUG] TimerDriver::TimerDriver() CONSTRUCTOR ENTRY\n");
    timer_freq_hz_ = read_sysreg_cntfrq();
    if (timer_freq_hz_ == 0) {
        HAL_VDBG("[HAL_DEBUG] CNTFRQ_EL0 is 0, using default 62.5MHz for QEMU virt\n");
        timer_freq_hz_ = 62500000; 
    } else {
        HAL_VDBG("[HAL_DEBUG] CNTFRQ_EL0 read by TimerDriver constructor.\n"); 
    }
    HAL_VDBG("[HAL_DEBUG] TimerDriver::TimerDriver() CONSTRUCTOR EXIT\n");
}
void TimerDriver::init_system_timer_properties(uint64_t freq_hz_override) {
    HAL_VDBG("[HAL_DEBUG] TimerDriver::init_system_timer_properties() ENTRY\n");
    if (freq_hz_override > 0) timer_freq_hz_ = freq_hz_override;
    HAL_VDBG("[HAL_DEBUG] TimerDriver::init_system_timer_properties() EXIT\n");
}
void TimerDriver::init_core_timer_interrupt(uint32_t core_id) {
    HAL_VDBG("[HAL_DEBUG] TimerDriver::init_core_timer_interrupt() ENTRY\n");
    if (kernel::hal::is_dedicated_rt_core(core_id)) {
        // Tickless core: leave CNTP disabled. The RT worker on this core will
        // program CNTP_TVAL_EL0 + enable CTL each time it calls
        // wait_wfi_until_ns; the scheduler tick is not used here at all.
        write_sysreg_cntp_ctl(0);
        HAL_VDBG("[HAL_DEBUG] TimerDriver::init_core_timer_interrupt() skipped (tickless)\n");
        return;
    }
    uint64_t ticks_for_period = (timer_freq_hz_ * TIMER_TICK_US) / 1000000ULL;
    write_sysreg_cntp_tval(ticks_for_period);
    write_sysreg_cntp_ctl(1);
    HAL_VDBG("[HAL_DEBUG] TimerDriver::init_core_timer_interrupt() EXIT\n");
}
void TimerDriver::ack_core_timer_interrupt(uint32_t core_id) {
    if (kernel::hal::is_dedicated_rt_core(core_id)) {
        // Tickless core: this IRQ is a WFI wake from wait_wfi_until_ns. Just
        // disable the compare so the line de-asserts; the RT worker will
        // re-arm via CNTP_TVAL on its next wait.
        write_sysreg_cntp_ctl(0);
        return;
    }
    uint64_t ticks_for_period = (timer_freq_hz_ * TIMER_TICK_US) / 1000000ULL;
    write_sysreg_cntp_tval(ticks_for_period);
}

void TimerDriver::wait_until_ns(uint64_t target_ns) {
    // WFI is disabled by default because it is unreliable under WSL2/Hyper-V
    // virtualisation. The CNTP IRQ-driven WFI wake was observed to be delayed,
    // causing masters to accumulate only ~1 cycle per CLI command instead of
    // thousands per second, blocking ESM progression.
    //
    // On real silicon (native hardware), this should work correctly. To enable
    // WFI on real hardware, define MINIOS_USE_WFI at compile time.
    #ifdef MINIOS_USE_WFI
    while (get_system_time_ns() < target_ns) {
        asm volatile("wfi");
    }
    #else
    // On scheduler-driven cores, a pure CPU hint here starves lower-priority
    // threads indefinitely if a higher-priority worker spends most of its life
    // in timed waits (for example the UI thread on core 0). Cooperatively hand
    // the core back to the scheduler until the deadline arrives.
    if (!kernel::hal::is_dedicated_rt_core(kernel::g_platform ? kernel::g_platform->get_core_id() : 0) &&
        kernel::g_scheduler_ptr && kernel::g_platform) {
        while (get_system_time_ns() < target_ns) {
            kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        }
        return;
    }
    while (get_system_time_ns() < target_ns) asm volatile("yield");
    #endif
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
uint64_t TimerDriver::get_system_time_ns() {
    if (timer_freq_hz_ == 0) return 0;
    // ARM generic timer is 62.5 MHz on QEMU virt → ticks * 16 = ns. The
    // general form below covers any freq without overflow (ticks fits in 64b
    // for centuries; × 1e9 before divide only narrows when freq ≥ 1 GHz).
    return (read_sysreg_cntpct() * 1000000000ULL) / timer_freq_hz_;
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
    HAL_VDBG("[DEBUG] DMAController CONSTRUCTOR ENTRY\n");
    channels_in_use_.fill(false);
    HAL_VDBG("[DEBUG] DMAController CONSTRUCTOR EXIT\n");
}
kernel::hal::dma::Capabilities DMAController::get_capabilities() const {
    kernel::hal::dma::Capabilities caps;
    caps.engine_kind = kernel::hal::dma::EngineKind::Software;
    caps.available = true;
    caps.mem_to_mem = true;
    caps.mem_to_periph = true;
    caps.periph_to_mem = true;
    caps.async_completion = false;
    caps.scatter_gather = false;
    caps.cache_coherent = false;
    caps.driver_name = "qemu-arm64-softdma";
    return caps;
}
kernel::hal::dma::ChannelID DMAController::request_channel() {
    kernel::core::ScopedLock lock(dma_lock_);
    for (size_t i = 0; i < channels_in_use_.size(); ++i) {
        if (!channels_in_use_[i]) { channels_in_use_[i] = true; return static_cast<kernel::hal::dma::ChannelID>(i); }
    }
    return kernel::hal::dma::INVALID_CHANNEL;
}
bool DMAController::configure_and_start_transfer(kernel::hal::dma::ChannelID ch, const kernel::hal::dma::TransferConfig& cfg, kernel::hal::dma::DMACallback cb, void* ctx) {
    if (ch < 0 || static_cast<size_t>(ch) >= channels_in_use_.size() || !channels_in_use_[static_cast<size_t>(ch)]) return false;
    if (cfg.size_bytes == 0) {
        if (cb) cb(ch, true, ctx);
        release_channel(ch);
        return true;
    }
    if (cfg.direction == kernel::hal::dma::Direction::MEM_TO_MEM) {
        auto* dst = reinterpret_cast<uint8_t*>(cfg.dst_addr);
        auto* src = reinterpret_cast<const uint8_t*>(cfg.src_addr);
        if (!dst || !src) {
            release_channel(ch);
            return false;
        }
        if (cfg.src_increment && cfg.dst_increment) {
            if (dst < src) {
                for (size_t i = 0; i < cfg.size_bytes; ++i) dst[i] = src[i];
            } else {
                for (size_t i = cfg.size_bytes; i != 0; --i) dst[i - 1] = src[i - 1];
            }
        } else {
            for (size_t i = 0; i < cfg.size_bytes; ++i) {
                const size_t src_i = cfg.src_increment ? i : 0;
                const size_t dst_i = cfg.dst_increment ? i : 0;
                dst[dst_i] = src[src_i];
            }
        }
    }
    if (cb) cb(ch, true, ctx);
    release_channel(ch);
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
void PowerOps::enter_idle_state(uint32_t) {
#ifdef MINIOS_USE_WFI
    asm volatile("wfi");
#else
    // Match TimerDriver::wait_until_ns: under QEMU/WSL2 the scheduler tick is
    // more reliable if the idle thread stays interruptible without parking in
    // WFI. The CPU hint keeps the loop cheap while still allowing timer IRQs
    // to preempt and run the scheduler.
    asm volatile("yield");
#endif
}
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

bool USBHostController::init() {
    return ::hal::shared::xhci::init_from_pci(pci_host_, xhci_, info_);
}

kernel::hal::usb::ControllerInfo USBHostController::get_info() const {
    ::hal::shared::xhci::refresh_info(xhci_, info_);
    return info_;
}

uint32_t USBHostController::get_port_count() const {
    return info_.port_count;
}

bool USBHostController::get_port_status(uint32_t port_index, kernel::hal::usb::PortStatus& out) const {
    return ::hal::shared::xhci::get_port_status(xhci_, port_index, out);
}

bool USBHostController::set_port_power(uint32_t port_index, bool on) {
    return ::hal::shared::xhci::set_port_power(xhci_, port_index, on);
}

bool USBHostController::reset_port(uint32_t port_index) {
    return ::hal::shared::xhci::reset_port(xhci_, port_index);
}

// --- PlatformQEMUVirtARM64 ---
bool InputDriver::init() {
    return ::hal::shared::input::init_virtio_input();
}

void InputDriver::poll() {
    if (auto* driver = ::hal::shared::input::get_input_driver()) driver->poll();
}

bool InputDriver::is_keyboard_connected() {
    auto* driver = ::hal::shared::input::get_input_driver();
    return driver ? driver->is_keyboard_connected() : false;
}

bool InputDriver::is_mouse_connected() {
    auto* driver = ::hal::shared::input::get_input_driver();
    return driver ? driver->is_mouse_connected() : false;
}

bool InputDriver::is_touch_connected() {
    auto* driver = ::hal::shared::input::get_input_driver();
    return driver ? driver->is_touch_connected() : false;
}

bool InputDriver::get_key_state(uint8_t key) {
    auto* driver = ::hal::shared::input::get_input_driver();
    return driver ? driver->get_key(key) : false;
}

void InputDriver::get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) {
    auto* driver = ::hal::shared::input::get_input_driver();
    if (!driver) {
        x = 0; y = 0; buttons = 0;
        return;
    }
    driver->get_mouse_position(x, y, buttons);
}

void InputDriver::get_touch_position(int32_t& x, int32_t& y, bool& pressed) {
    auto* driver = ::hal::shared::input::get_input_driver();
    if (!driver) {
        x = 0; y = 0; pressed = false;
        return;
    }
    driver->get_touch_position(x, y, pressed);
}

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
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR START\n");
#if MINIOS_VERBOSE_BOOT
    unsigned long long vtable_addr = *(unsigned long long*)this;
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "[HAL_DEBUG] Constructor vtable: 0x%llx\n", vtable_addr);
    early_uart_puts(addr_buf);
    if (vtable_addr == 0) {
        HAL_VDBG("[HAL_DEBUG] ERROR: Null vtable in constructor\n");
    }
#endif
    HAL_VDBG("[HAL_DEBUG] Initializing uart_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing irq_controller_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing timer_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing dma_controller_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing i2s_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing memory_ops_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing network_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing power_ops_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing gpio_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing watchdog_driver_\n");
    HAL_VDBG("[HAL_DEBUG] Initializing input_driver_\n");
    gpu_.configure_bus(VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
    ::hal::shared::input::configure_virtio_input_bus(VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR: Members initialized\n");
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64 CONSTRUCTOR EXIT\n");
}

kernel::hal::net::NetworkDriverOps* PlatformQEMUVirtARM64::get_net_ops(int idx) {
    if (idx < 0) return nullptr;
    if (idx < num_e1000_nics_) {
        return &e1000_nics_[idx];
    }
    if (idx < num_e1000_nics_ + num_virtio_nics_) {
        return &virtio_nics_[idx - num_e1000_nics_];
    }
    if (num_e1000_nics_ == 0 && num_virtio_nics_ == 0 && idx == 0) {
        return static_cast<kernel::hal::net::NetworkDriverOps*>(&network_driver_);
    }
    return nullptr;
}

int PlatformQEMUVirtARM64::get_num_nets() const {
    return num_e1000_nics_ + num_virtio_nics_;
}

bool PlatformQEMUVirtARM64::init_e1000_nic(int idx, uintptr_t mmio_base, uint8_t irq) {
    if (idx < 0 || idx >= 3) return false;
    if (e1000_nics_[idx].init_pci(0, 0, mmio_base, irq)) {
        if (idx >= num_e1000_nics_) num_e1000_nics_ = idx + 1;
        return true;
    }
    return false;
}

PlatformQEMUVirtARM64::~PlatformQEMUVirtARM64() {
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64 DESTRUCTOR\n");
}

uint32_t PlatformQEMUVirtARM64::get_core_id() const {
    uint64_t mpidr;
    asm volatile("mrs %0, mpidr_el1" : "=r"(mpidr));
    return static_cast<uint32_t>(mpidr & 0xFF); 
}
uint32_t PlatformQEMUVirtARM64::get_num_cores() const { return kernel::core::MAX_CORES; }

void PlatformQEMUVirtARM64::early_init_platform() {
#if MINIOS_VERBOSE_BOOT
    unsigned long long vtable_addr = *(unsigned long long*)this;
    char addr_buf[20];
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "[HAL_DEBUG] early_init_platform vtable: 0x%llx\n", vtable_addr);
    early_uart_puts(addr_buf);
#endif
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_platform() ENTRY\n");
    irq_controller_.init_distributor();
    timer_driver_.init_system_timer_properties();
    input_driver_.init();
    {
        uint64_t ecam_base = PCIE_ECAM_BASE;
        uint64_t ecam_size = PCIE_ECAM_SIZE;
        uint64_t mmio_base = PCIE_MMIO_BASE;
        uint64_t mmio_size = PCIE_MMIO_SIZE;
        uint32_t bus_start = 0;
        uint32_t bus_end = 0xff;
        if (g_arm64_dtb_ptr != 0 && g_arm64_dtb_ptr != 0xFFFFFFFFFFFFFFFFULL) {
            (void)::hal::shared::fdt::find_compatible_reg(reinterpret_cast<const void*>(g_arm64_dtb_ptr),
                                                          "pci-host-ecam-generic",
                                                          &ecam_base, &ecam_size,
                                                          &mmio_base, &mmio_size,
                                                          &bus_start, &bus_end);
        }
        usb_controller_.set_pci_host({ecam_base,
                                      mmio_base,
                                      mmio_size,
                                      static_cast<uint8_t>(bus_start & 0xffu),
                                      static_cast<uint8_t>(bus_end & 0xffu),
                                      true});
    }

    // Discover virtio-net NICs on the virtio-mmio bus. Up to 3 supported:
    // HMI on eth0 plus two EtherCAT-facing NICs on eth1/eth2. The dumped QEMU
    // arm64 virt DTB advertises 32 `virtio,mmio` slots at 0x0a000000 with a
    // 0x200-byte stride, and slot N uses GIC SPI (32 + 16 + N).
    struct ArmHookCtx { kernel::hal::IRQControllerOps* irq; };
    ArmHookCtx hook_ctx{ &irq_controller_ };
    auto arm_hook = [](size_t slot_idx, size_t nic_idx,
                       ::hal::shared::virtio::VirtioNetDriver& /*drv*/, void* ctx) {
        auto* c = static_cast<ArmHookCtx*>(ctx);
        if (!c) return;
        const uint32_t spi_id = 32u + 16u + static_cast<uint32_t>(slot_idx);
        if (nic_idx < 3) {
            platform_instance().note_virtio_nic_spi(nic_idx, spi_id);
        }
    };
    num_virtio_nics_ = static_cast<int>(
        ::hal::shared::virtio::discover_virtio_net(
            VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM,
            virtio_nics_, sizeof(virtio_nics_) / sizeof(virtio_nics_[0]),
            arm_hook, &hook_ctx));
#if MINIOS_VERBOSE_BOOT
    {
        char b[64];
        kernel::util::k_snprintf(b, sizeof(b), "[HAL] virtio-net discovered: %d\n", num_virtio_nics_);
        early_uart_puts(b);
    }
#endif
    HAL_VDBG("[HAL_DEBUG] PlatformQEMUVirtARM64::early_init_platform() EXIT\n");
}

bool PlatformQEMUVirtARM64::init_block_device() {
    // Scan the same virtio-mmio bus for a block device. If QEMU wasn't
    // started with `-drive ... -device virtio-blk-device`, the scan
    // returns nullptr and we leave fs_ops_ready_ false so `get_fs_ops()`
    // falls back to nullptr (no FS, VFS uses embedded defaults only).
    virtio_blk_ = ::hal::shared::virtio::discover_virtio_blk(
        VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
    if (!virtio_blk_) return false;
    blk_reader_.bind(virtio_blk_);
    if (!fs_ops_.mount()) return false;
    fs_ops_ready_ = true;
    return true;
}

void PlatformQEMUVirtARM64::note_virtio_nic_spi(size_t nic_idx, uint32_t spi_id) {
    if (nic_idx >= (sizeof(virtio_nic_spi_ids_) / sizeof(virtio_nic_spi_ids_[0]))) return;
    virtio_nic_spi_ids_[nic_idx] = spi_id;
}

void PlatformQEMUVirtARM64::route_net_irq(int if_idx, uint32_t core_mask) {
    if (if_idx < 0 || if_idx >= 3) return;
    const uint32_t spi = virtio_nic_spi_ids_[if_idx];
    if (spi == 0 || core_mask == 0) return;
    irq_controller_.set_irq_affinity(spi, core_mask);
}

void PlatformQEMUVirtARM64::early_init_core(uint32_t core_id) {
    (void)core_id;
    irq_controller_.init_cpu_interface(core_id);
    timer_driver_.init_core_timer_interrupt(core_id);
    uint64_t cpacr_el1;
    asm volatile("mrs %0, cpacr_el1" : "=r"(cpacr_el1));
    cpacr_el1 |= (3ULL << 20);
    asm volatile("msr cpacr_el1, %0" : : "r"(cpacr_el1));
    asm volatile("isb");
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
    HAL_VDBG("[DEBUG] get_platform ENTRY\n");
    HAL_VDBG("[DEBUG] Returning platform instance at 0x");
    char addr_buf[20];
    auto& instance = qemu_virt_arm64::platform_instance();
    kernel::util::k_snprintf(addr_buf, sizeof(addr_buf), "%llx\n", (unsigned long long)&instance);
    early_uart_puts(addr_buf);
    return &instance;
}
} // namespace hal
