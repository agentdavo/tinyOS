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

// Dedicated-core tickless mode is placement-driven. A core is considered
// dedicated only when it hosts an EtherCAT master / fake slave and no
// general-purpose runtime threads share it. That keeps the old isolated EC
// behavior, but does not incorrectly force tickless scheduling when the
// placement TSV moves `ec_a` onto a shared core such as `core0`.
bool is_dedicated_rt_core(uint32_t core_id) noexcept;


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
    // Pin an SPI to a set of target cores. `core_mask` is an 8-bit bitmask of
    // CPU interfaces (bit 0 = CPU 0). Used to route NIC IRQs to the core that
    // owns the corresponding EtherCAT master. Default no-op for ports that
    // don't implement per-IRQ affinity.
    virtual void set_irq_affinity(uint32_t irq_id, uint32_t core_mask) {
        (void)irq_id; (void)core_mask;
    }
};

namespace dma {
    using ChannelID = int32_t;
    constexpr ChannelID INVALID_CHANNEL = -1;
    enum class EngineKind { None, Software, Hardware };
    enum class Direction { MEM_TO_PERIPH, PERIPH_TO_MEM, MEM_TO_MEM };
    struct Capabilities {
        EngineKind engine_kind = EngineKind::None;
        bool available = false;
        bool mem_to_mem = false;
        bool mem_to_periph = false;
        bool periph_to_mem = false;
        bool async_completion = false;
        bool scatter_gather = false;
        bool cache_coherent = false;
        const char* driver_name = "none";
    };
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
    virtual dma::Capabilities get_capabilities() const {
        return dma::Capabilities{};
    }
    virtual bool configure_and_start_transfer(dma::ChannelID channel, const dma::TransferConfig& cfg, dma::DMACallback cb, void* context) = 0;
    virtual bool submit_transfer(const dma::TransferConfig& cfg,
                                 dma::DMACallback cb = nullptr,
                                 void* context = nullptr) {
        dma::ChannelID ch = request_channel();
        if (ch == dma::INVALID_CHANNEL) return false;
        const bool ok = configure_and_start_transfer(ch, cfg, cb, context);
        if (!ok) release_channel(ch);
        return ok;
    }
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
    // Nanosecond precision for jitter tracking. Default decomposes to us*1000
    // but platforms should override when the timer resolves finer than 1 µs
    // (e.g. ARM generic timer at 62.5 MHz = 16 ns resolution).
    virtual uint64_t get_system_time_ns() { return get_system_time_us() * 1000ULL; }
    // Block the calling thread until monotonic time reaches `target_ns`.
    // Portable default = busy-spin with yield hints. Platforms that own the
    // per-core timer on dedicated RT cores should override with a WFI-based
    // implementation (arm64: arm CNTP_TVAL + WFI; rv64: CLINT mtimecmp + wfi)
    // so the CPU actually halts between cycles and WSL2/Hyper-V can reclaim.
    virtual void wait_until_ns(uint64_t target_ns) {
        while (get_system_time_ns() < target_ns) {
#if defined(__aarch64__)
            asm volatile("yield");
#else
            asm volatile("nop");
#endif
        }
    }
    virtual void hardware_timer_irq_fired(uint32_t core_id) = 0;
};

namespace net {
    using PacketReceivedCallback = void (*)(int if_idx, const uint8_t* data, size_t len, void* context);
    // Per-NIC accumulators surfaced via `get_stats()` and the `net` CLI.
    struct NicStats {
        uint64_t tx_packets = 0;
        uint64_t tx_bytes   = 0;
        uint64_t tx_drops   = 0;
        uint64_t rx_packets = 0;
        uint64_t rx_bytes   = 0;
        uint64_t rx_drops   = 0;
    };
    struct NetworkDriverOps {
        virtual ~NetworkDriverOps() = default;
        virtual bool init_interface(int if_idx) = 0;
        virtual bool send_packet(int if_idx, const uint8_t* data, size_t len) = 0;
        virtual void register_packet_receiver(PacketReceivedCallback cb, void* context) = 0;
        // Snapshot of current TX/RX counters. Default returns zeros so legacy
        // ops that don't track stats still build.
        virtual NicStats get_stats() const { return NicStats{}; }
        virtual bool get_mac(uint8_t out[6]) const {
            (void)out;
            return false;
        }
        // Drain up to `budget` frames from the NIC RX queue, invoking `cb` for
        // each. Default no-op for ops that don't implement poll-mode RX.
        virtual size_t poll_rx(PacketReceivedCallback cb, void* context,
                               size_t budget = 8) {
            (void)cb; (void)context; (void)budget; return 0;
        }
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

struct InputEvent {
    enum class Type : uint8_t {
        KeyPress,
        KeyRelease,
        MouseMove,
        MouseButton,
        TouchPress,
        TouchRelease,
        TouchMove
    };
    Type type;
    int32_t x;
    int32_t y;
    uint8_t key;
    uint8_t button;
    uint32_t timestamp_ms;
};

struct InputOps {
    virtual ~InputOps() = default;
    virtual bool init() = 0;
    virtual void poll() = 0;
    virtual bool is_keyboard_connected() = 0;
    virtual bool is_mouse_connected() = 0;
    virtual bool is_touch_connected() = 0;
    virtual bool get_key_state(uint8_t key) = 0;
    virtual void get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) = 0;
    virtual void get_touch_position(int32_t& x, int32_t& y, bool& pressed) = 0;
};

struct StorageOps {
    virtual ~StorageOps() = default;
    virtual bool init() = 0;
    virtual bool read_block(uint32_t block_num, void* buffer) = 0;
    virtual bool write_block(uint32_t block_num, const void* buffer) = 0;
    virtual uint32_t get_total_blocks() = 0;
    virtual uint32_t get_block_size() = 0;
    virtual bool is_present() = 0;
};

struct DisplayOps {
    virtual ~DisplayOps() = default;
    virtual bool init(void* framebuffer, uint32_t width, uint32_t height, uint32_t stride) = 0;
    virtual bool present() = 0;
    virtual bool is_connected() = 0;
    virtual void get_resolution(uint32_t& width, uint32_t& height) = 0;
};

namespace usb {
    enum class ControllerKind : uint8_t {
        Unknown,
        UHCI,
        OHCI,
        EHCI,
        XHCI
    };

    enum class PortSpeed : uint8_t {
        Unknown,
        Low,
        Full,
        High,
        Super
    };

    struct ControllerInfo {
        ControllerKind kind = ControllerKind::Unknown;
        const char* driver_name = "none";
        const char* transport = "none";
        const char* enum_state = "none";
        uintptr_t register_base = 0;
        uint32_t irq = 0;
        uint32_t port_count = 0;
        uint32_t active_port = 0;
        uint16_t device_vendor_id = 0;
        uint16_t device_product_id = 0;
        uint8_t device_class = 0;
        uint8_t keyboard_last_report_len = 0;
        uint8_t keyboard_endpoint_dci = 0;
        uint8_t keyboard_last_completion = 0;
        uint32_t keyboard_reports = 0;
        uint32_t transfer_events = 0;
        uint32_t port_change_events = 0;
        bool present = false;
        bool initialized = false;
        bool hotplug_capable = false;
        bool qemu_virtual = false;
        bool device_addressed = false;
        bool device_descriptor_read = false;
        bool keyboard_connected = false;
        bool keyboard_poll_pending = false;
    };

    struct PortStatus {
        bool powered = false;
        bool connected = false;
        bool enabled = false;
        bool over_current = false;
        bool reset_in_progress = false;
        PortSpeed speed = PortSpeed::Unknown;
    };
}

struct USBHostControllerOps {
    virtual ~USBHostControllerOps() = default;
    virtual bool init() = 0;
    virtual usb::ControllerInfo get_info() const = 0;
    virtual uint32_t get_port_count() const = 0;
    virtual bool get_port_status(uint32_t port_index, usb::PortStatus& out) const = 0;
    virtual bool set_port_power(uint32_t port_index, bool on) = 0;
    virtual bool reset_port(uint32_t port_index) = 0;
};

// FileSystemOps — abstract file backend. Implemented on a per-board basis
// so the same callers (kernel::vfs::mount_fs, future runtime hot-reload)
// work against virtio-blk under QEMU and SPI-SD on real hardware without
// touching any higher-level code.
//
// Paths are forward-slash separated; the backend maps them onto whatever
// the underlying FS uses (FAT short names, sector-indexed blobs, etc.).
// The VFS calls these ops once at mount time, caches each file's bytes
// in its static entry pool, then answers `vfs::lookup()` from the cache —
// so the backend doesn't need to be fast, just correct.
struct FileSystemOps {
    struct Entry {
        char name[96];   // full path as seen from the root (e.g. "system/ui/embedded_ui.tsv")
        size_t size;
        bool is_dir;
    };
    using WalkFn = bool(*)(const Entry& entry, void* user);

    virtual ~FileSystemOps() = default;

    // Open the backing storage. Returns false if no media is present or
    // the FS is unreadable. Safe to call more than once (idempotent).
    virtual bool mount() = 0;

    // Release any cached FS state. mount() can be called again afterward.
    virtual void unmount() = 0;

    // Does the backend have a mounted, usable FS right now?
    virtual bool is_mounted() const = 0;

    // Walk files rooted at `prefix` (empty == whole FS). Each entry's full
    // path lives in `entry.name`. Return false from the callback to stop.
    virtual bool list(const char* prefix, WalkFn fn, void* user) = 0;

    // Read at most `buf_size` bytes from `path` into `buf`. On success,
    // `*bytes_read` holds the number copied. Returns false if the file is
    // missing or the read fails.
    virtual bool read(const char* path, void* buf, size_t buf_size, size_t* bytes_read) = 0;

    // Optional helper: file size without a read. Return 0 if unknown.
    virtual size_t file_size(const char* /*path*/) { return 0; }
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
    // Multi-NIC access. Default: single NIC at idx 0. Platforms with two or
    // more NICs override `get_num_nets()` and return each driver from
    // `get_net_ops(idx)`; callers using the old single-NIC API keep working.
    virtual net::NetworkDriverOps* get_net_ops() { return get_net_ops(0); }
    virtual net::NetworkDriverOps* get_net_ops(int idx) = 0;
    virtual int get_num_nets() const { return 1; }
    virtual PowerOps* get_power_ops() = 0;
    virtual gpio::GPIODriverOps* get_gpio_ops() = 0;
    virtual WatchdogOps* get_watchdog_ops() = 0;
    virtual InputOps* get_input_ops() = 0;
    virtual StorageOps* get_storage_ops() = 0;
    virtual DisplayOps* get_display_ops() = 0;
    virtual USBHostControllerOps* get_usb_ops() = 0;
    // File-system backend. Default returns nullptr so existing HAL impls
    // don't have to stub a full FileSystemOps right away — the VFS
    // gracefully falls back to its embedded-defaults-only mode when this
    // returns null.
    virtual FileSystemOps* get_fs_ops() { return nullptr; }
    virtual void early_init_platform() = 0;
    virtual void early_init_core(uint32_t core_id) = 0;
    virtual void route_net_irq(int if_idx, uint32_t core_mask) { (void)if_idx; (void)core_mask; }
    [[noreturn]] virtual void panic(const char* msg, const char* file, int line) = 0;
    virtual void reboot_system() = 0;
    // Dispatch a platform-specific (non-timer) IRQ. The platform decides what the
    // irq_id means; the generic IRQ path in hal.cpp stays free of platform constants.
    virtual void handle_device_irq(uint32_t core_id, uint32_t irq_id) { (void)core_id; (void)irq_id; }
};

Platform* get_platform();

} // namespace hal
} // namespace kernel

#endif // HAL_HPP
