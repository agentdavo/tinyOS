// SPDX-License-Identifier: MIT OR Apache-2.0
// RISC-V HAL for miniOS on QEMU virt (rv64, M-mode, -bios none).
//
// Wires up the UART, CLINT-backed timer (with wait_until_ns WFI primitive),
// PLIC, and a trap/IRQ dispatch surface. Scheduler-level pieces (TCBs,
// context switch, preemptive_tick callback) are not yet ported; the timer
// IRQ handler just re-arms mtimecmp and increments a counter for now.

#ifndef HAL_QEMU_RV64_HPP
#define HAL_QEMU_RV64_HPP

#include "hal.hpp"
#include "hal/shared/virtio_gpu.hpp"
#include "hal/shared/virtio_input.hpp"
#include "hal/shared/virtio_net.hpp"
#include "hal/shared/e1000.hpp"
#include "hal/shared/pci.hpp"
#include "hal/shared/xhci.hpp"
#include <cstdint>

namespace hal::qemu_virt_rv64 {

struct TCB_Rv64;
class USBHostController;

// QEMU virt RISC-V peripheral addresses.
constexpr uint64_t UART_BASE   = 0x10000000; // NS16550-compatible
constexpr uint64_t CLINT_BASE  = 0x02000000;
constexpr uint64_t PLIC_BASE   = 0x0C000000;
constexpr uint64_t PCIE_ECAM_BASE = 0x30000000ULL;
constexpr uint64_t PCIE_ECAM_SIZE = 0x10000000ULL;
constexpr uint64_t PCIE_MMIO_BASE = 0x40000000ULL;
constexpr uint64_t PCIE_MMIO_SIZE = 0x40000000ULL;

// virtio-mmio bus on QEMU virt-rv64. Layout differs from arm64: 8 slots of
// 0x1000 bytes starting at 0x10001000 (arm64 uses 32 slots of 0x200 bytes
// at 0x0a000000). The per-slot register map is identical. PLIC IRQ for
// virtio slot N is 1 + N (see QEMU hw/riscv/virt.c, VIRTIO_IRQ = 1).
//
// IMPORTANT: invoke QEMU with `-global virtio-mmio.force-legacy=false` so the
// transport reports version 2. Our shared driver (hal::shared::virtio) refuses
// legacy (v1) transports and discovery returns 0 NICs. The Makefile
// QEMU_ARGS sets this; hand-rolled QEMU invocations must too.
constexpr uint64_t VIRTIO_MMIO_BASE    = 0x10001000;
constexpr size_t   VIRTIO_MMIO_STRIDE  = 0x1000;
constexpr size_t   VIRTIO_MMIO_NUM     = 8;
constexpr uint32_t VIRTIO_IRQ_BASE     = 1; // PLIC source for slot 0

// QEMU virt exposes a 10 MHz timebase on the generic timer (mtime). If a DTB
// is available we could parse /cpus/timebase-frequency to confirm, but the
// value is stable across QEMU versions so we hardcode it.
constexpr uint64_t TIMEBASE_HZ = 10'000'000; // 100 ns / tick
constexpr uint64_t TICK_PERIOD_US = 200;     // 200 µs scheduler tick
constexpr uint32_t MAX_HARTS   = 4;

// CLINT layout (see QEMU hw/intc/sifive_clint.c):
//   +0x0000 ..  msip[hart]     (4 bytes each)
//   +0x4000 ..  mtimecmp[hart] (8 bytes each)
//   +0xBFF8     mtime          (8 bytes)
constexpr uint64_t CLINT_MSIP      = CLINT_BASE + 0x0000;
constexpr uint64_t CLINT_MTIMECMP  = CLINT_BASE + 0x4000;
constexpr uint64_t CLINT_MTIME     = CLINT_BASE + 0xBFF8;

// PLIC layout (QEMU virt: 1 context per hart in M-mode; we ignore S-mode
// contexts because we run bare M-mode without SBI).
//   +0x000000 ..  priority[irq]        (4 bytes each, 1..N)
//   +0x001000 ..  pending bitmap
//   +0x002000 + 0x80*ctx ..  enable bits (bit per IRQ)
//   +0x200000 + 0x1000*ctx .. threshold (+0) / claim-complete (+4)
constexpr uint64_t PLIC_PRIORITY_BASE = PLIC_BASE + 0x000000;
constexpr uint64_t PLIC_ENABLE_BASE   = PLIC_BASE + 0x002000;
constexpr uint64_t PLIC_CONTEXT_BASE  = PLIC_BASE + 0x200000;

class UARTDriver : public kernel::hal::UARTDriverOps {
public:
    void putc(char c) override;
    void puts(const char* str) override;
    void uart_put_uint64_hex(uint64_t value) override;
    char getc_blocking() override;
};

// CLINT-backed timer. Provides wait_until_ns for EtherCAT/motion RT loops.
class TimerDriver : public kernel::hal::TimerDriverOps {
public:
    TimerDriver();
    void init_system_timer_properties(uint64_t freq_hz_override = 0) override;
    void init_core_timer_interrupt(uint32_t core_id) override;
    void ack_core_timer_interrupt(uint32_t core_id) override;
    bool add_software_timer(kernel::hal::timer::SoftwareTimer*) override { return false; }
    bool remove_software_timer(kernel::hal::timer::SoftwareTimer*) override { return false; }
    uint64_t get_system_time_us() override;
    uint64_t get_system_time_ns() override;
    void hardware_timer_irq_fired(uint32_t core_id) override;
    void wait_until_ns(uint64_t target_ns) override;
    // Current timebase in Hz — either the DTB-reported value or the
    // TIMEBASE_HZ fallback, whichever init_system_timer_properties picked.
    uint64_t freq_hz() const { return timer_freq_hz_; }
private:
    uint64_t timer_freq_hz_ = TIMEBASE_HZ;
};

// PLIC driver — external IRQ dispatch. Routing is done by programming the
// per-context enable bitmap for each source, so affinity is a matter of
// selecting which hart contexts have the source bit set.
class PLICDriver : public kernel::hal::IRQControllerOps {
public:
    void enable_core_irqs(uint32_t core_id, uint32_t mask) override;
    void disable_core_irqs(uint32_t core_id) override;
    void init_distributor() override;
    void init_cpu_interface(uint32_t core_id) override;
    uint32_t ack_irq(uint32_t core_id) override;
    void end_irq(uint32_t core_id, uint32_t irq_id) override;
    void enable_irq_line(uint32_t irq_id) override;
    void disable_irq_line(uint32_t irq_id) override;
    void set_irq_priority(uint32_t irq_id, uint8_t prio) override;
    void set_irq_affinity(uint32_t irq_id, uint32_t core_mask) override;

private:
    void apply_irq_routing(uint32_t irq_id) noexcept;
    uint8_t irq_enabled_[128]{};
    uint8_t irq_affinity_mask_[128]{};
};

class DMAController : public kernel::hal::DMAControllerOps {
public:
    DMAController() = default;
    kernel::hal::dma::ChannelID request_channel() override;
    void release_channel(kernel::hal::dma::ChannelID channel) override;
    kernel::hal::dma::Capabilities get_capabilities() const override;
    bool configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                      const kernel::hal::dma::TransferConfig& cfg,
                                      kernel::hal::dma::DMACallback cb,
                                      void* context) override;
private:
    bool channel_in_use_ = false;
};

class MemoryOps : public kernel::hal::MemoryOps {
public:
    void flush_cache_range(const void* addr, size_t size) override;
    void invalidate_cache_range(const void* addr, size_t size) override;
};

class InputDriver : public kernel::hal::InputOps {
public:
    explicit InputDriver(USBHostController* usb) : usb_(usb) {}
    bool init() override;
    void poll() override;
    bool is_keyboard_connected() override;
    bool is_mouse_connected() override;
    bool is_touch_connected() override;
    bool get_key_state(uint8_t key) override;
    void get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) override;
    void get_touch_position(int32_t& x, int32_t& y, bool& pressed) override;
private:
    USBHostController* usb_ = nullptr;
};

class USBHostController : public kernel::hal::USBHostControllerOps {
public:
    void set_pci_host(const ::hal::shared::pci::HostBridge& host) { pci_host_ = host; }
    ::hal::shared::xhci::ControllerState& state() { return xhci_; }
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

class PlatformQEMUVirtRV64 : public kernel::hal::Platform {
public:
    using kernel::hal::Platform::get_net_ops;
    PlatformQEMUVirtRV64();
    virtual ~PlatformQEMUVirtRV64();
    virtual void dummy_vtable_anchor() {}

    uint32_t get_core_id() const override;
    uint32_t get_num_cores() const override { return MAX_HARTS; }
    kernel::hal::UARTDriverOps*           get_uart_ops() override { return &uart_; }
    kernel::hal::IRQControllerOps*        get_irq_ops()  override { return &plic_; }
    kernel::hal::TimerDriverOps*          get_timer_ops() override { return &timer_; }
    kernel::hal::DMAControllerOps*        get_dma_ops()  override { return &dma_; }
    kernel::hal::I2SDriverOps*            get_i2s_ops()  override { return nullptr; }
    kernel::hal::MemoryOps*               get_mem_ops()  override { return &memory_ops_; }
    kernel::hal::net::NetworkDriverOps*   get_net_ops(int idx) override;
    int                                   get_num_nets() const override;
    bool                                  init_e1000_nic(int idx, uintptr_t mmio_base, uint8_t irq);
    // Probe the virtio-mmio bus and bind all virtio-net slots. Safe to call
    // once after PLIC init.
    void discover_virtio_nets();
    void route_net_irq(int if_idx, uint32_t core_mask) override;
    kernel::hal::PowerOps*                get_power_ops() override { return nullptr; }
    kernel::hal::gpio::GPIODriverOps*     get_gpio_ops() override { return nullptr; }
    kernel::hal::WatchdogOps*             get_watchdog_ops() override { return nullptr; }
    kernel::hal::InputOps*                get_input_ops() override { return &input_; }
    kernel::hal::StorageOps*              get_storage_ops() override { return nullptr; }
    kernel::hal::DisplayOps*              get_display_ops() override { return &gpu_; }
    kernel::hal::USBHostControllerOps*    get_usb_ops() override { return &usb_; }

    void early_init_platform() override {}
    void early_init_core(uint32_t) override {}
    [[noreturn]] void panic(const char* msg, const char* file, int line) override;
    void reboot_system() override {}

private:
    UARTDriver  uart_;
    TimerDriver timer_;
    PLICDriver  plic_;
    DMAController dma_;
    MemoryOps memory_ops_;
    USBHostController usb_;
    InputDriver input_{&usb_};
    ::hal::shared::virtio_gpu::VirtioGpuDriver gpu_;
    ::hal::shared::virtio::VirtioNetDriver virtio_nics_[3]{};
    uint32_t virtio_nic_irq_ids_[3]{};
    ::hal::shared::e1000::E1000Driver e1000_nics_[3]{};
    int         num_e1000_nics_ = 0;
    int         num_virtio_nics_ = 0;
};

extern PlatformQEMUVirtRV64 g_platform_instance;

// Secondary-hart spin-table (one u64 per hart). Hart 0 writes an entry
// address here; secondaries poll in cpu_rv64.S and jump when non-zero.
extern "C" uint64_t g_secondary_entry[MAX_HARTS];

// Counters for liveness reporting (incremented by trap dispatcher).
extern "C" uint64_t g_timer_ticks[MAX_HARTS];
extern "C" uint64_t g_wfi_wakes[MAX_HARTS];

} // namespace hal::qemu_virt_rv64

#endif // HAL_QEMU_RV64_HPP
