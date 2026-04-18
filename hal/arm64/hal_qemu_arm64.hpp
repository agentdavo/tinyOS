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
#include "util.hpp"      // For early_uart_puts
#include "hal/shared/virtio_net.hpp" // shared virtio-net driver
#include "hal/shared/virtio_blk.hpp" // shared virtio-block driver
#include "fs/fs_fat32.hpp"           // FAT32 FileSystemOps
#include "hal/shared/virtio_gpu.hpp" // shared virtio-gpu driver
#include "hal/shared/virtio_input.hpp"
#include "hal/shared/e1000.hpp"       // shared e1000 driver for PCI passthrough
#include "hal/shared/pci.hpp"
#include "hal/shared/xhci.hpp"
#include <string_view>
#include <array>
#include <atomic>
#include <cstdint>

// Same verbose-boot gate the .cpp uses. Every inline ctor below has a
// per-driver `CONSTRUCTOR ENTRY/EXIT` print that otherwise fires during
// static init — holding the UART lock for ~1 ms each and collectively
// dominating the boot-time UART. Off by default.
#ifndef MINIOS_VERBOSE_BOOT
#  define MINIOS_VERBOSE_BOOT 0
#endif
#if MINIOS_VERBOSE_BOOT
#  define HAL_CTOR_DBG(s) early_uart_puts(s)
#else
#  define HAL_CTOR_DBG(s) do { } while (0)
#endif

namespace hal::qemu_virt_arm64 { // This is a global namespace, not under kernel::hal

// Base addresses for QEMU 'virt' machine peripherals
constexpr uint64_t UART_BASE = 0x09000000;
constexpr uint64_t GIC_DISTRIBUTOR_BASE = 0x08000000;
constexpr uint64_t GIC_CPU_INTERFACE_BASE = 0x08010000;
constexpr uint64_t RTC_BASE = 0x09010000;
constexpr uint64_t VIRTIO_MMIO_BASE = 0x0A000000; 
constexpr size_t   VIRTIO_MMIO_STRIDE = 0x200;
constexpr size_t   VIRTIO_MMIO_NUM = 32;
constexpr uint64_t VIRTIO_NET_REG_BASE = VIRTIO_MMIO_BASE;
constexpr uint64_t PCIE_ECAM_BASE = 0x4010000000ULL;
constexpr uint64_t PCIE_ECAM_SIZE = 0x10000000ULL;
constexpr uint64_t PCIE_MMIO_BASE = 0x10000000ULL;
constexpr uint64_t PCIE_MMIO_SIZE = 0x2EFF0000ULL;

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
    UARTDriver() {
        HAL_CTOR_DBG("[DEBUG] UARTDriver CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] UARTDriver CONSTRUCTOR EXIT\n");
    }
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
    IRQController() {
        HAL_CTOR_DBG("[DEBUG] IRQController CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] IRQController CONSTRUCTOR EXIT\n");
    }
    void enable_core_irqs(uint32_t core_id, uint32_t irq_source_mask) override;
    void disable_core_irqs(uint32_t core_id) override;
    void init_distributor() override;
    void init_cpu_interface(uint32_t core_id) override;
    uint32_t ack_irq(uint32_t core_id) override;
    void end_irq(uint32_t core_id, uint32_t irq_id) override;
    void enable_irq_line(uint32_t irq_id) override;
    void disable_irq_line(uint32_t irq_id) override;
    void set_irq_priority(uint32_t irq_id, uint8_t priority) override;
    void set_irq_affinity(uint32_t irq_id, uint32_t core_mask) override;
private:
    void write_gicd_reg(uint32_t offset, uint32_t value);
    uint32_t read_gicd_reg(uint32_t offset);
    void write_gicc_reg(uint64_t offset, uint32_t value);
    uint32_t read_gicc_reg(uint64_t offset);
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
    uint64_t get_system_time_ns() override;
    // Dedicated-core tickless path: on cores listed via is_dedicated_rt_core
    // we skip the scheduler's 200 µs tick and let a single RT worker own
    // CNTP_*. wait_until_ns on those cores uses CNTP_TVAL_EL0 + WFI
    // (hal::rt::wait_wfi_until_ns) instead of a yield-spin loop, which stops
    // the VCPU from looking busy to Hyper-V/WSL2 between EC cycles.
    void wait_until_ns(uint64_t target_ns) override;
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
    kernel::hal::dma::Capabilities get_capabilities() const override;
    bool configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                     const kernel::hal::dma::TransferConfig& cfg,
                                     kernel::hal::dma::DMACallback cb, void* context) override;
    void release_channel(kernel::hal::dma::ChannelID channel) override;
private:
    std::array<bool, 8> channels_in_use_;
    kernel::core::Spinlock dma_lock_;
};

class InputDriver : public kernel::hal::InputOps {
public:
    bool init() override;
    void poll() override;
    bool is_keyboard_connected() override;
    bool is_mouse_connected() override;
    bool is_touch_connected() override;
    bool get_key_state(uint8_t key) override;
    void get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) override;
    void get_touch_position(int32_t& x, int32_t& y, bool& pressed) override;
};

class I2SDriver : public kernel::hal::I2SDriverOps {
public:
    I2SDriver() {
        HAL_CTOR_DBG("[DEBUG] I2SDriver CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] I2SDriver CONSTRUCTOR EXIT\n");
    }
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
    MemoryOps() {
        HAL_CTOR_DBG("[DEBUG] MemoryOps CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] MemoryOps CONSTRUCTOR EXIT\n");
    }
    void flush_cache_range(const void* addr, size_t size) override;
    void invalidate_cache_range(const void* addr, size_t size) override;
};

class NetworkDriver : public kernel::hal::net::NetworkDriverOps {
public:
    NetworkDriver() : packet_received_cb_(nullptr), cb_context_(nullptr), initialized_(false) {
        HAL_CTOR_DBG("[DEBUG] NetworkDriver CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] NetworkDriver CONSTRUCTOR EXIT\n");
    }
    bool init_interface(int if_idx) override;
    bool send_packet(int if_idx, const uint8_t* data, size_t len) override;
    void register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb, void* context) override;
    kernel::hal::net::NicStats get_stats() const override { return kernel::hal::net::NicStats{}; }
private:
    kernel::hal::net::PacketReceivedCallback packet_received_cb_ = nullptr;
    void* cb_context_ = nullptr;
    bool initialized_ = false;
};

class PowerOps : public kernel::hal::PowerOps {
public:
    PowerOps() {
        HAL_CTOR_DBG("[DEBUG] PowerOps CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] PowerOps CONSTRUCTOR EXIT\n");
    }
    void enter_idle_state(uint32_t core_id) override;
    bool set_cpu_frequency(uint32_t core_id, uint32_t freq_hz) override;
};

class GPIODriver : public kernel::hal::gpio::GPIODriverOps {
public:
    GPIODriver() {
        HAL_CTOR_DBG("[DEBUG] GPIODriver CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] GPIODriver CONSTRUCTOR EXIT\n");
    }
    bool init_bank(uint32_t bank_id) override;
    bool configure_pin(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinMode mode) override;
    bool set_pin_state(uint32_t bank, uint32_t pin, kernel::hal::gpio::PinState state) override;
    kernel::hal::gpio::PinState read_pin_state(uint32_t bank, uint32_t pin) override;
    void enable_interrupt(uint32_t bank, uint32_t pin, bool rising_edge) override;
};

class WatchdogDriver : public kernel::hal::WatchdogOps {
public:
    WatchdogDriver() {
        HAL_CTOR_DBG("[DEBUG] WatchdogDriver CONSTRUCTOR ENTRY\n");
        HAL_CTOR_DBG("[DEBUG] WatchdogDriver CONSTRUCTOR EXIT\n");
    }
    bool start_watchdog(uint32_t timeout_ms) override;
    bool reset_watchdog() override;
    bool stop_watchdog() override;
};

class USBHostController : public kernel::hal::USBHostControllerOps {
public:
    void set_pci_host(const ::hal::shared::pci::HostBridge& host) { pci_host_ = host; }
    bool init() override;
    kernel::hal::usb::ControllerInfo get_info() const override;
    uint32_t get_port_count() const override;
    bool get_port_status(uint32_t port_index, kernel::hal::usb::PortStatus& out) const override;
    bool set_port_power(uint32_t port_index, bool on) override;
    bool reset_port(uint32_t port_index) override;
private:
    ::hal::shared::pci::HostBridge pci_host_{};
    mutable ::hal::shared::xhci::ControllerState xhci_{};
    mutable kernel::hal::usb::ControllerInfo info_{};
};

class PlatformQEMUVirtARM64 : public kernel::hal::Platform {
public:
    using kernel::hal::Platform::get_net_ops;
    PlatformQEMUVirtARM64();
    virtual ~PlatformQEMUVirtARM64();
    virtual void dummy_vtable_anchor() {} // Force vtable generation
    uint32_t get_core_id() const override;
    uint32_t get_num_cores() const override;
    void early_init_platform() override;
    void early_init_core(uint32_t core_id) override;
    void route_net_irq(int if_idx, uint32_t core_mask) override;
    [[noreturn]] void panic(const char* msg, const char* file, int line) override;
    void reboot_system() override;
    kernel::hal::UARTDriverOps* get_uart_ops() override { return &uart_driver_; }
    kernel::hal::IRQControllerOps* get_irq_ops() override { return &irq_controller_; }
    kernel::hal::TimerDriverOps* get_timer_ops() override { return &timer_driver_; }
    kernel::hal::DMAControllerOps* get_dma_ops() override { return &dma_controller_; }
    kernel::hal::I2SDriverOps* get_i2s_ops() override { return &i2s_driver_; }
    kernel::hal::MemoryOps* get_mem_ops() override { return &memory_ops_; }
    kernel::hal::net::NetworkDriverOps* get_net_ops(int idx) override;
    int get_num_nets() const override;
    bool init_e1000_nic(int idx, uintptr_t mmio_base, uint8_t irq);
    void note_virtio_nic_spi(size_t nic_idx, uint32_t spi_id);
    kernel::hal::PowerOps* get_power_ops() override { return &power_ops_; }
    kernel::hal::gpio::GPIODriverOps* get_gpio_ops() override { return &gpio_driver_; }
    kernel::hal::WatchdogOps* get_watchdog_ops() override { return &watchdog_driver_; }
    kernel::hal::InputOps* get_input_ops() override { return &input_driver_; }
    kernel::hal::StorageOps* get_storage_ops() override { return nullptr; }
    kernel::hal::DisplayOps* get_display_ops() override { return &gpu_; }
    kernel::hal::USBHostControllerOps* get_usb_ops() override { return &usb_controller_; }
    kernel::hal::FileSystemOps* get_fs_ops() override {
        return fs_ops_ready_ ? &fs_ops_ : nullptr;
    }
    // Called from boot to bind the virtio-block device (if any) and set
    // up the FAT32 FS backend. No-op / returns false if no virtio-blk is
    // present on the bus, which keeps QEMU runs that don't pass -drive
    // booting on embedded defaults only.
    bool init_virtio_blk();
private:
    // Adapter so fs::Fat32FileSystem can read sectors via an abstract
    // BlockReader interface without knowing anything about virtio.
    class VirtioBlkReader : public fs::BlockReader {
    public:
        void bind(::hal::shared::virtio::VirtioBlkDriver* d) { drv_ = d; }
        bool read_sectors(uint64_t lba, uint32_t count, void* buf) override {
            return drv_ && drv_->read_sectors(lba, count, buf);
        }
    private:
        ::hal::shared::virtio::VirtioBlkDriver* drv_ = nullptr;
    };
    UARTDriver uart_driver_;
    IRQController irq_controller_;
    TimerDriver timer_driver_;
    DMAController dma_controller_;
    I2SDriver i2s_driver_;
    MemoryOps memory_ops_;
    NetworkDriver network_driver_; // Legacy stub, kept for now.
    ::hal::shared::virtio_gpu::VirtioGpuDriver gpu_;
    ::hal::shared::virtio::VirtioNetDriver virtio_nics_[3]{};
    uint32_t virtio_nic_spi_ids_[3]{};
    ::hal::shared::e1000::E1000Driver e1000_nics_[3]{};
    int num_e1000_nics_ = 0;
    int num_virtio_nics_ = 0;
    PowerOps power_ops_;
    GPIODriver gpio_driver_;
    WatchdogDriver watchdog_driver_;
    InputDriver input_driver_;
    USBHostController usb_controller_;
    ::hal::shared::virtio::VirtioBlkDriver* virtio_blk_ = nullptr;
    VirtioBlkReader blk_reader_;
    fs::Fat32FileSystem fs_ops_{&blk_reader_};
    bool fs_ops_ready_ = false;
};

PlatformQEMUVirtARM64& platform_instance();

} // namespace hal::qemu_virt_arm64

#endif // HAL_QEMU_ARM64_HPP
