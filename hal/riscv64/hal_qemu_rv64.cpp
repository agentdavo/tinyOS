// SPDX-License-Identifier: MIT OR Apache-2.0
// RISC-V HAL implementation for miniOS on QEMU virt.
//
// Boot path:
//   _start (cpu_rv64.S) -> kernel_main_rv64 (below, hart 0 only)
//                       -> secondaries spin on g_secondary_entry[hartid]
// Trap path:
//   trap_entry (cpu_rv64.S) -> hal_trap_dispatch_rv64 (below)
//     - mcause bit 63 set: async IRQ, dispatch by cause code
//         7  = Machine Timer (MTI)  -> ack CLINT mtimecmp, bump counter
//         11 = Machine External     -> PLIC claim -> dispatch -> complete
//     - otherwise synchronous exception: dump + halt
//
// Timer:
//   get_system_time_ns reads the mtime CSR (rdtime on RV64 reads mtime
//   directly per the Zicntr extension QEMU implements). 10 MHz timebase,
//   ticks * 100 = ns.
//   wait_until_ns programs this hart's mtimecmp for the deadline, sets
//   MTIE + MIE and issues wfi. On return the IRQ handler has masked MTIE
//   for the long-deadline path (handled inside).
//
// IRQs:
//   PLIC context map on QEMU virt -bios none: context N = M-mode view of
//   hart N (because -bios none -> we ARE M-mode, there is no S-mode). That
//   matches QEMU's sifive_plic model with num-contexts-per-hart = 1.

#include "hal_qemu_rv64.hpp"
#include "../../miniOS.hpp"
#include "rt_wait.hpp"
#include "fdt.hpp"
#include "hal/shared/fdt_scan.hpp"
#include "../../cli.hpp"
#include "../../devices/device_db.hpp"
#include "../../devices/embedded.hpp"
#include "../../ethercat/bus_config.hpp"
#include "../../ethercat/fake_slave.hpp"
#include "../../ethercat/master.hpp"
#include "../../motion/motion.hpp"
#include "../../cnc/interpreter.hpp"
#include "../../automation/macro_runtime.hpp"
#include "../../automation/ladder_runtime.hpp"
#include "../../machine/toolpods.hpp"
#include "../../machine/machine_registry.hpp"
#include "../../ui/splash.hpp"
#include "../../ui/display.hpp"
#include "../../ui/fb.hpp"
#include "../../ui/ui_builder_tsv.hpp"
#include "../../automation/probe_runtime.hpp"
#include "../../machine/machine_topology.hpp"
#include "../../machine/runtime_placement.hpp"
#include "../../machine/motion_wiring.hpp"
#include "../../hmi/hmi_service.hpp"
#include "../../kernel/usb/usb.hpp"
#include "../../util.hpp"

#include <atomic>

extern "C" {
extern const char _binary_embedded_ui_tsv_start[];
extern const char _binary_embedded_ui_tsv_end[];
extern const char _binary_embedded_macros_tsv_start[];
extern const char _binary_embedded_macros_tsv_end[];
extern const char _binary_embedded_ladder_tsv_start[];
extern const char _binary_embedded_ladder_tsv_end[];
extern const char _binary_embedded_toolpods_tsv_start[];
extern const char _binary_embedded_toolpods_tsv_end[];
extern const char _binary_embedded_signals_tsv_start[];
extern const char _binary_embedded_signals_tsv_end[];
extern const char _binary_embedded_topology_tsv_start[];
extern const char _binary_embedded_topology_tsv_end[];
extern const char _binary_embedded_placement_tsv_start[];
extern const char _binary_embedded_placement_tsv_end[];
extern const char _binary_embedded_hmi_tsv_start[];
extern const char _binary_embedded_hmi_tsv_end[];
}

namespace hal::qemu_virt_rv64 {

// ----------------------------------------------------------------------------
// UART lock.  The NS16550A at 0x10000000 is shared across all 4 harts, so
// without serialization their putc streams interleave at the byte level
// (e.g. "threthreadad" when two harts both print "thread" at once).  A
// single `std::atomic_flag` acquired around each whole string is enough —
// the UART is not on the steady-state hot path (the kernel only prints on
// bring-up events + the demo threads on hart 0).  We keep putc() lock-free
// so code inside the critical section can still emit single bytes, and
// only lock at the puts()/hex() granularity where smearing is visible.
// ----------------------------------------------------------------------------
static std::atomic_flag g_uart_lock = ATOMIC_FLAG_INIT;

struct UartLockGuard {
    UartLockGuard()  { while (g_uart_lock.test_and_set(std::memory_order_acquire)) { asm volatile("" ::: "memory"); } }
    ~UartLockGuard() { g_uart_lock.clear(std::memory_order_release); }
};

PlatformQEMUVirtRV64 g_platform_instance;

PlatformQEMUVirtRV64::PlatformQEMUVirtRV64() {
    gpu_.configure_bus(VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
    ::hal::shared::input::configure_virtio_input_bus(VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
}

PlatformQEMUVirtRV64::~PlatformQEMUVirtRV64() = default;

// Spin-table + counters (extern "C" so the assembler can see g_secondary_entry).
extern "C" {
uint64_t g_secondary_entry[MAX_HARTS] = {0, 0, 0, 0};
uint64_t g_timer_ticks[MAX_HARTS]     = {0, 0, 0, 0};
uint64_t g_wfi_wakes[MAX_HARTS]       = {0, 0, 0, 0};

// DTB pointer stashed by cpu_rv64.S early in _start before anything can
// clobber a1. Placed in .data (non-zero sentinel) so it survives the BSS
// zero pass — the assembler overwrites it with QEMU's pointer immediately
// after. Sentinel 0xFFFFFFFFFFFFFFFF distinguishes "not written" from
// "bootloader passed NULL".
__attribute__((section(".data")))
uint64_t g_dtb_ptr = 0xFFFFFFFFFFFFFFFFULL;

// Per-hart IRQ stack (4 KiB) + top-of-stack pointers written into mscratch
// at early_init_core(). The trap prologue uses mscratch as "irq stack top"
// (and swaps back to the caller sp on mret).
alignas(16) uint8_t hal_irq_stacks[MAX_HARTS][0x1000];
}

static std::atomic<uint32_t> g_secondary_ready_mask{0};

// Forward declarations for the rv64-local scheduler (see rv64_sched.cpp).
// Intentionally kept at file scope so both the trap dispatcher and
// kernel_main_rv64 can invoke them without pulling in a new header.
struct TCB_Rv64;
TCB_Rv64* sched_create_thread(void (*entry)(void*), void* arg,
                              uint32_t hart_affinity, const char* name,
                              bool is_idle);
[[noreturn]] void sched_enter_first(uint32_t hart);
extern "C" void rv64_preemptive_tick(uint32_t hart);

// ----------------------------------------------------------------------------
// UART (NS16550)
// ----------------------------------------------------------------------------
namespace {
inline volatile uint8_t* uart_reg(uint32_t off) {
    return reinterpret_cast<volatile uint8_t*>(UART_BASE + off);
}
constexpr uint32_t NS16550_RBR = 0;
constexpr uint32_t NS16550_THR = 0;
constexpr uint32_t NS16550_LSR = 5;
constexpr uint8_t  LSR_DR      = 1 << 0;

inline volatile uint64_t* clint_mtimecmp(uint32_t hart) {
    return reinterpret_cast<volatile uint64_t*>(CLINT_MTIMECMP + 8ULL * hart);
}
inline volatile uint64_t* clint_mtime() {
    return reinterpret_cast<volatile uint64_t*>(CLINT_MTIME);
}

inline uint64_t read_mtime() {
    // rdtime reads mtime on QEMU virt (Zicntr). Using the CSR read avoids an
    // MMIO round-trip per sample.
    uint64_t t;
    asm volatile("rdtime %0" : "=r"(t));
    return t;
}

inline void csr_set_mie_mtie() {
    asm volatile("csrsi mstatus, 0x8" ::: "memory"); // MIE
    uint64_t bit = 1ULL << 7; // MTIE
    asm volatile("csrs mie, %0" :: "r"(bit) : "memory");
}
inline void csr_clear_mtie() {
    uint64_t bit = 1ULL << 7;
    asm volatile("csrc mie, %0" :: "r"(bit) : "memory");
}
inline void csr_set_meie() {
    uint64_t bit = 1ULL << 11;
    asm volatile("csrs mie, %0" :: "r"(bit) : "memory");
}

} // anon namespace

void UARTDriver::putc(char c) {
    *uart_reg(NS16550_THR) = static_cast<uint8_t>(c);
}
void UARTDriver::puts(const char* str) {
    if (!str) return;
    // Hold the UART lock for the whole string so cross-hart puts() calls
    // don't interleave character-by-character. Single-byte putc() remains
    // lock-free for callers that have already taken the lock (or genuinely
    // want raw single-character output).
    UartLockGuard g;
    while (*str) {
        if (*str == '\n') this->putc('\r');
        this->putc(*str++);
    }
}
void UARTDriver::uart_put_uint64_hex(uint64_t value) {
    static constexpr char hex[] = "0123456789ABCDEF";
    char buf[17]; buf[16] = '\0';
    for (int i = 15; i >= 0; --i) { buf[i] = hex[value & 0xF]; value >>= 4; }
    this->puts(buf);
}
char UARTDriver::getc_blocking() {
    while ((*uart_reg(NS16550_LSR) & LSR_DR) == 0) { /* spin */ }
    return static_cast<char>(*uart_reg(NS16550_RBR));
}

// ----------------------------------------------------------------------------
// DMA (software fallback for now; future hardware DMA can replace this behind
// the same HAL surface once detected during platform bring-up).
// ----------------------------------------------------------------------------
kernel::hal::dma::ChannelID DMAController::request_channel() {
    if (channel_in_use_) return kernel::hal::dma::INVALID_CHANNEL;
    channel_in_use_ = true;
    return 0;
}

void DMAController::release_channel(kernel::hal::dma::ChannelID channel) {
    if (channel == 0) channel_in_use_ = false;
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
    caps.driver_name = "qemu-rv64-softdma";
    return caps;
}

bool DMAController::configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                                 const kernel::hal::dma::TransferConfig& cfg,
                                                 kernel::hal::dma::DMACallback cb,
                                                 void* context) {
    if (channel != 0 || !channel_in_use_) return false;
    if (cfg.size_bytes == 0) {
        if (cb) cb(channel, true, context);
        release_channel(channel);
        return true;
    }
    if (cfg.direction == kernel::hal::dma::Direction::MEM_TO_MEM) {
        auto* dst = reinterpret_cast<uint8_t*>(cfg.dst_addr);
        auto* src = reinterpret_cast<const uint8_t*>(cfg.src_addr);
        if (!dst || !src) {
            release_channel(channel);
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
    if (cb) cb(channel, true, context);
    release_channel(channel);
    return true;
}

void MemoryOps::flush_cache_range(const void* addr, size_t size) {
    (void)addr;
    (void)size;
    // Generic rv64 on QEMU virt does not expose a standard cache-maintenance
    // instruction set like arm64 DC CVAC/IVAC. The best portable contract we
    // can provide here is a full memory barrier so prior framebuffer/queue
    // writes are globally ordered before MMIO doorbells.
    asm volatile("fence rw, rw" ::: "memory");
}

void MemoryOps::invalidate_cache_range(const void* addr, size_t size) {
    (void)addr;
    (void)size;
    // Same rationale as flush_cache_range(): on this target we provide strong
    // ordering rather than architected cache-line invalidation.
    asm volatile("fence rw, rw" ::: "memory");
}

bool USBHostController::init() {
    return ::hal::shared::xhci::init_from_pci(pci_host_, xhci_, info_);
}

bool InputDriver::init() {
    return ::hal::shared::input::init_virtio_input();
}

void InputDriver::poll() {
    if (auto* driver = ::hal::shared::input::get_input_driver()) driver->poll();
    if (usb_) ::hal::shared::xhci::poll(usb_->state());
}

bool InputDriver::is_keyboard_connected() {
    if (auto* driver = ::hal::shared::input::get_input_driver()) {
        if (driver->is_keyboard_connected()) return true;
    }
    return usb_ ? ::hal::shared::xhci::keyboard_connected(usb_->state()) : false;
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
    if (auto* driver = ::hal::shared::input::get_input_driver()) {
        if (driver->get_key(key)) return true;
    }
    return usb_ ? ::hal::shared::xhci::key_pressed(usb_->state(), key) : false;
}

void InputDriver::get_mouse_position(int32_t& x, int32_t& y, uint8_t& buttons) {
    auto* driver = ::hal::shared::input::get_input_driver();
    if (!driver) {
        x = 0;
        y = 0;
        buttons = 0;
        return;
    }
    driver->get_mouse_position(x, y, buttons);
}

void InputDriver::get_touch_position(int32_t& x, int32_t& y, bool& pressed) {
    auto* driver = ::hal::shared::input::get_input_driver();
    if (!driver) {
        x = 0;
        y = 0;
        pressed = false;
        return;
    }
    driver->get_touch_position(x, y, pressed);
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

// ----------------------------------------------------------------------------
// Timer (CLINT mtime/mtimecmp)
// ----------------------------------------------------------------------------
// Read the timebase frequency from the DTB (QEMU passes the FDT pointer in
// a1 at boot, which cpu_rv64.S stashes into g_dtb_ptr). If the DTB is
// missing or malformed we fall back to the constant 10 MHz QEMU has
// historically exposed — kernel_main_rv64 prints which path was taken.
// Returns (hz, from_dtb).
struct TimebaseResult { uint64_t hz; bool from_dtb; };
static TimebaseResult detect_timebase_hz() {
    const uint64_t dtb = fdt::g_dtb_ptr;
    // Sentinel 0xFF..FF means cpu_rv64.S never overwrote the global (e.g.
    // built without the a1-save); treat as "no DTB".
    if (dtb != 0ULL && dtb != 0xFFFFFFFFFFFFFFFFULL) {
        uint32_t hz = fdt::read_timebase_frequency(reinterpret_cast<const void*>(dtb));
        if (hz) return {hz, true};
    }
    return {TIMEBASE_HZ, false};
}

TimerDriver::TimerDriver() : timer_freq_hz_(TIMEBASE_HZ) {}

void TimerDriver::init_system_timer_properties(uint64_t freq_hz_override) {
    if (freq_hz_override) timer_freq_hz_ = freq_hz_override;
}

void TimerDriver::init_core_timer_interrupt(uint32_t core_id) {
    if (core_id >= MAX_HARTS) return;
    // Program mtimecmp for "now + 200us" using an 8-byte write. Do NOT split
    // into two 4-byte writes: a partial update could momentarily leave
    // mtimecmp at a value <= mtime and trigger a spurious MTI.
    const uint64_t delta = (timer_freq_hz_ * TICK_PERIOD_US) / 1'000'000ULL;
    *clint_mtimecmp(core_id) = read_mtime() + delta;
    // Enable MTIE; caller will flip MIE in mstatus at scheduler go-live.
    uint64_t bit = 1ULL << 7;
    asm volatile("csrs mie, %0" :: "r"(bit) : "memory");
}

void TimerDriver::ack_core_timer_interrupt(uint32_t core_id) {
    if (core_id >= MAX_HARTS) return;
    const uint64_t delta = (timer_freq_hz_ * TICK_PERIOD_US) / 1'000'000ULL;
    *clint_mtimecmp(core_id) = read_mtime() + delta;
}

uint64_t TimerDriver::get_system_time_us() {
    if (timer_freq_hz_ == 0) return 0;
    return (read_mtime() * 1'000'000ULL) / timer_freq_hz_;
}

uint64_t TimerDriver::get_system_time_ns() {
    if (timer_freq_hz_ == 0) return 0;
    // 10 MHz -> ticks * 100 = ns (exact, no overflow for centuries).
    return (read_mtime() * 1'000'000'000ULL) / timer_freq_hz_;
}

void TimerDriver::hardware_timer_irq_fired(uint32_t core_id) {
    // Software timer queue not implemented on rv64 yet.
    (void)core_id;
}

void TimerDriver::wait_until_ns(uint64_t target_ns) {
    // Loop: program mtimecmp at target deadline, enable MTIE+MIE, wfi.
    // WFI can wake spuriously (spec says an implementation may wake even
    // without a pending interrupt), so we re-check the deadline.
    const uint32_t hart = ::hal::qemu_virt_rv64::g_platform_instance.get_core_id();
    for (;;) {
        const uint64_t now_ticks = read_mtime();
        const uint64_t now_ns    = (now_ticks * 1'000'000'000ULL) / timer_freq_hz_;
        if (now_ns >= target_ns) return;
        const uint64_t delta_ns   = target_ns - now_ns;
        const uint64_t delta_tick = (delta_ns * timer_freq_hz_) / 1'000'000'000ULL;
        *clint_mtimecmp(hart) = now_ticks + (delta_tick ? delta_tick : 1);
        csr_set_mie_mtie();
        asm volatile("wfi");
        // Upon IRQ the trap handler acks mtimecmp by pushing it far into the
        // future (below). We loop to cover spurious wakes.
        ::hal::qemu_virt_rv64::g_wfi_wakes[hart]++;
    }
}

// ----------------------------------------------------------------------------
// PLIC
// ----------------------------------------------------------------------------
namespace {
inline volatile uint32_t* plic_priority(uint32_t irq) {
    return reinterpret_cast<volatile uint32_t*>(PLIC_PRIORITY_BASE + 4ULL * irq);
}
// Context index: M-mode on QEMU virt with -bios none uses context 0 for hart 0,
// context 2 for hart 1, context 4 for hart 2, ... QEMU virt allocates 2
// contexts per hart (M + S) even without S-mode firmware. The M-mode one is
// the even index. Reference: QEMU hw/intc/sifive_plic.c and
// hw/riscv/virt.c (VIRT_PLIC_NUM_SOURCES/contexts).
inline uint32_t plic_ctx(uint32_t hart) { return hart * 2; }

inline volatile uint32_t* plic_enable_word(uint32_t hart, uint32_t irq) {
    return reinterpret_cast<volatile uint32_t*>(
        PLIC_ENABLE_BASE + 0x80ULL * plic_ctx(hart) + 4ULL * (irq / 32));
}
inline volatile uint32_t* plic_threshold(uint32_t hart) {
    return reinterpret_cast<volatile uint32_t*>(
        PLIC_CONTEXT_BASE + 0x1000ULL * plic_ctx(hart));
}
inline volatile uint32_t* plic_claim(uint32_t hart) {
    return reinterpret_cast<volatile uint32_t*>(
        PLIC_CONTEXT_BASE + 0x1000ULL * plic_ctx(hart) + 4);
}

constexpr uint32_t plic_valid_hart_mask() {
    return (1u << MAX_HARTS) - 1u;
}
} // anon ns

void PLICDriver::init_distributor() {
    // Zero all priorities. QEMU virt PLIC exposes sources 1..127 (VIRT_IRQCHIP
    // has 127 sources). Priority 0 = disabled, >=1 = eligible.
    for (uint32_t i = 1; i < 128; ++i) {
        *plic_priority(i) = 0;
        irq_enabled_[i] = 0;
        irq_affinity_mask_[i] = static_cast<uint8_t>(plic_valid_hart_mask());
    }
}

void PLICDriver::init_cpu_interface(uint32_t core_id) {
    if (core_id >= MAX_HARTS) return;
    // threshold = 0 means "accept any priority >= 1".
    *plic_threshold(core_id) = 0;
    csr_set_meie();
}

void PLICDriver::enable_core_irqs(uint32_t, uint32_t) {
    uint64_t bits = (1ULL << 7) | (1ULL << 11); // MTIE + MEIE
    asm volatile("csrs mie, %0" :: "r"(bits) : "memory");
    asm volatile("csrsi mstatus, 0x8" ::: "memory"); // MIE
}
void PLICDriver::disable_core_irqs(uint32_t) {
    asm volatile("csrci mstatus, 0x8" ::: "memory");
}

uint32_t PLICDriver::ack_irq(uint32_t core_id) {
    if (core_id >= MAX_HARTS) return 0;
    return *plic_claim(core_id);
}
void PLICDriver::end_irq(uint32_t core_id, uint32_t irq_id) {
    if (core_id >= MAX_HARTS) return;
    *plic_claim(core_id) = irq_id;
}

void PLICDriver::apply_irq_routing(uint32_t irq_id) noexcept {
    if (irq_id == 0 || irq_id >= 128) return;
    const uint32_t bit = 1u << (irq_id & 31u);
    const uint32_t affinity = static_cast<uint32_t>(irq_affinity_mask_[irq_id]) & plic_valid_hart_mask();
    for (uint32_t h = 0; h < MAX_HARTS; ++h) {
        volatile uint32_t* w = plic_enable_word(h, irq_id);
        if (irq_enabled_[irq_id] && (affinity & (1u << h))) {
            *w = *w | bit;
        } else {
            *w = *w & ~bit;
        }
    }
}

void PLICDriver::enable_irq_line(uint32_t irq_id) {
    if (irq_id == 0 || irq_id >= 128) return;
    irq_enabled_[irq_id] = 1;
    apply_irq_routing(irq_id);
}
void PLICDriver::disable_irq_line(uint32_t irq_id) {
    if (irq_id == 0 || irq_id >= 128) return;
    irq_enabled_[irq_id] = 0;
    apply_irq_routing(irq_id);
}
void PLICDriver::set_irq_priority(uint32_t irq_id, uint8_t prio) {
    if (irq_id == 0 || irq_id >= 128) return;
    *plic_priority(irq_id) = prio ? prio : 1; // force non-zero so the line is eligible
}

void PLICDriver::set_irq_affinity(uint32_t irq_id, uint32_t core_mask) {
    if (irq_id == 0 || irq_id >= 128) return;
    const uint32_t sanitized = core_mask & plic_valid_hart_mask();
    irq_affinity_mask_[irq_id] = static_cast<uint8_t>(sanitized ? sanitized : 1u);
    apply_irq_routing(irq_id);
}

// ----------------------------------------------------------------------------
// Platform
// ----------------------------------------------------------------------------
uint32_t PlatformQEMUVirtRV64::get_core_id() const {
    uint64_t h; asm volatile("csrr %0, mhartid" : "=r"(h)); return static_cast<uint32_t>(h);
}

// virtio-net discovery + get_net_ops plumbing. The dumped QEMU virt-rv64 DTB
// advertises 8 `virtio,mmio` slots at 0x10001000 with a 0x1000-byte stride.
// Each slot N raises PLIC IRQ (VIRTIO_IRQ_BASE + N). Routing via PLIC is
// enable-on-every-hart-context (set_irq_affinity is a no-op on PLIC), so the
// hook just enables the line and gives it a non-zero priority.
kernel::hal::net::NetworkDriverOps* PlatformQEMUVirtRV64::get_net_ops(int idx) {
    if (idx < 0) return nullptr;
    if (idx < num_e1000_nics_) {
        return &e1000_nics_[idx];
    }
    if (idx < num_e1000_nics_ + num_virtio_nics_) {
        return &virtio_nics_[idx - num_e1000_nics_];
    }
    return nullptr;
}

int PlatformQEMUVirtRV64::get_num_nets() const {
    return num_e1000_nics_ + num_virtio_nics_;
}

bool PlatformQEMUVirtRV64::init_e1000_nic(int idx, uintptr_t mmio_base, uint8_t irq) {
    if (idx < 0 || idx >= 3) return false;
    if (e1000_nics_[idx].init_pci(0, 0, mmio_base, irq)) {
        if (idx >= num_e1000_nics_) num_e1000_nics_ = idx + 1;
        return true;
    }
    return false;
}

void PlatformQEMUVirtRV64::discover_virtio_nets() {
    // Pre-scan diagnostic: dump magic/version/device_id at every slot so we
    // can tell at a glance whether QEMU populated the bus at all and whether
    // the transport version matches what the shared driver accepts (v2).
    struct HookCtx {
        PLICDriver* plic;
        uint32_t* irq_ids;
    };
    HookCtx ctx{ &plic_, virtio_nic_irq_ids_ };
    auto hook = [](size_t slot_idx, size_t nic_idx,
                   ::hal::shared::virtio::VirtioNetDriver& drv, void* raw) {
        auto* c = static_cast<HookCtx*>(raw);
        if (!c) return;
        const uint32_t irq = VIRTIO_IRQ_BASE + static_cast<uint32_t>(slot_idx);
        if (c->plic) {
            c->plic->set_irq_priority(irq, 1);
            c->plic->enable_irq_line(irq);
        }
        if (c->irq_ids && nic_idx < 3) {
            c->irq_ids[nic_idx] = irq;
        }
        (void)nic_idx;
        (void)drv;
    };
    num_virtio_nics_ = static_cast<int>(
        ::hal::shared::virtio::discover_virtio_net(
            VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM,
            virtio_nics_, sizeof(virtio_nics_) / sizeof(virtio_nics_[0]),
            hook, &ctx));
    uart_.puts("[rv64] virtio-net probed: ");
    uart_.uart_put_uint64_hex(static_cast<uint64_t>(num_virtio_nics_));
    uart_.puts(" nic(s)\n");
}

void PlatformQEMUVirtRV64::route_net_irq(int if_idx, uint32_t core_mask) {
    if (if_idx < 0 || if_idx >= 3) return;
    const uint32_t irq = virtio_nic_irq_ids_[if_idx];
    if (irq == 0 || core_mask == 0) return;
    plic_.set_irq_affinity(irq, core_mask);
    uart_.puts("[rv64] plic route nic");
    uart_.uart_put_uint64_hex(static_cast<uint64_t>(if_idx));
    uart_.puts(" irq=");
    uart_.uart_put_uint64_hex(static_cast<uint64_t>(irq));
    uart_.puts(" hart_mask=");
    uart_.uart_put_uint64_hex(static_cast<uint64_t>(core_mask));
    uart_.puts("\n");
}

[[noreturn]] void PlatformQEMUVirtRV64::panic(const char* msg, const char* file, int line) {
    uart_.puts("\n[PANIC] ");
    if (msg) uart_.puts(msg);
    uart_.puts(" at ");
    if (file) uart_.puts(file);
    uart_.puts(":");
    uart_.uart_put_uint64_hex(static_cast<uint64_t>(line));
    uart_.puts("\n");
    for (;;) asm volatile("wfi");
}

} // namespace hal::qemu_virt_rv64

// HAL glue for generic kernel::hal::get_platform() — used when the scheduler
// is eventually ported to rv64.
namespace kernel::hal {
Platform& get_platform_instance() { return ::hal::qemu_virt_rv64::g_platform_instance; }
Platform* get_platform() { return &::hal::qemu_virt_rv64::g_platform_instance; }
}

// ----------------------------------------------------------------------------
// Trap dispatcher (called from trap_entry in cpu_rv64.S)
// ----------------------------------------------------------------------------
extern "C" void hal_trap_dispatch_rv64(uint64_t hartid, uint64_t mcause) {
    using namespace ::hal::qemu_virt_rv64;
    constexpr uint64_t MCAUSE_INT = 1ULL << 63;

    if (mcause & MCAUSE_INT) {
        uint64_t code = mcause & 0x7FFFFFFFFFFFFFFFULL;
        if (code == 7) {
            // Machine Timer Interrupt. Push mtimecmp far enough out that MTI
            // deasserts. For the "tick" use-case the scheduler would call
            // TimerDriver::ack_core_timer_interrupt here to re-arm at +200us;
            // for wait_until_ns the caller re-arms on each iteration. To
            // serve both: re-arm 200 us out if we're inside the periodic
            // tick path, otherwise the caller's next wait_until_ns will
            // overwrite this anyway.
            if (hartid < MAX_HARTS) {
                // Use whatever the TimerDriver was configured with (DTB
                // value if parsed, else TIMEBASE_HZ fallback).
                const uint64_t hz =
                    g_platform_instance.get_timer_ops()
                        ? static_cast<TimerDriver*>(
                              g_platform_instance.get_timer_ops())->freq_hz()
                        : TIMEBASE_HZ;
                const uint64_t delta = (hz * TICK_PERIOD_US) / 1'000'000ULL;
                uint64_t now;
                asm volatile("rdtime %0" : "=r"(now));
                *reinterpret_cast<volatile uint64_t*>(
                    CLINT_MTIMECMP + 8ULL * hartid) = now + delta;
                g_timer_ticks[hartid]++;
                // Hand off to the scheduler.  If it decides to swap the
                // running thread it updates g_current_tcb[hartid]; the
                // trap-exit assembly in cpu_rv64.S then transparently
                // restores into the new TCB.  Matches the role of
                // preemptive_tick() on the arm64 path in hal.cpp.
                rv64_preemptive_tick(static_cast<uint32_t>(hartid));
            }
            return;
        }
        if (code == 11) {
            // Machine External Interrupt via PLIC.
            if (hartid < MAX_HARTS) {
                auto& plic = *g_platform_instance.get_irq_ops();
                uint32_t irq = plic.ack_irq(static_cast<uint32_t>(hartid));
                if (irq != 0) {
                    g_platform_instance.handle_device_irq(static_cast<uint32_t>(hartid), irq);
                    plic.end_irq(static_cast<uint32_t>(hartid), irq);
                }
            }
            return;
        }
        if (code == 3) {
            // Machine Software Interrupt. Clear MSIP for this hart.
            if (hartid < MAX_HARTS) {
                *reinterpret_cast<volatile uint32_t*>(
                    CLINT_MSIP + 4ULL * hartid) = 0;
            }
            return;
        }
        // Unknown async IRQ.
        auto& uart = *g_platform_instance.get_uart_ops();
        uart.puts("\n[rv64] unknown IRQ cause=");
        uart.uart_put_uint64_hex(code);
        uart.puts("\n");
        for (;;) asm volatile("wfi");
    }

    // Synchronous exception.
    auto& uart = *g_platform_instance.get_uart_ops();
    uint64_t mepc;
    asm volatile("csrr %0, mepc" : "=r"(mepc));
    uart.puts("\n[rv64 TRAP] hart=");
    uart.uart_put_uint64_hex(hartid);
    uart.puts(" mcause=");
    uart.uart_put_uint64_hex(mcause);
    uart.puts(" mepc=");
    uart.uart_put_uint64_hex(mepc);
    uart.puts("\n");
    for (;;) asm volatile("wfi");
}

// ----------------------------------------------------------------------------
// Kernel entry for rv64. Hart 0 boots the platform, releases secondaries via
// the spin-table, demos a wait_until_ns cycle (proof the WFI primitive works)
// and then idles. Secondaries land in hart_main after the spin-table release.
// ----------------------------------------------------------------------------
extern "C" void hart_main(uint32_t hartid);

extern "C" uint8_t hal_irq_stacks[][0x1000]; // forward to global

static void setup_this_hart_trap_stack(uint32_t hartid) {
    // mscratch = top of this hart's IRQ stack. The trap prologue swaps
    // mscratch with sp on entry and swaps back on mret.
    uint64_t top = reinterpret_cast<uint64_t>(&hal_irq_stacks[hartid][0x1000]);
    asm volatile("csrw mscratch, %0" :: "r"(top));
}

// Per-hart idle thread.  Never on the ready queue; the scheduler falls back
// to it when no other thread is runnable on this hart.  Just WFIs until
// preempted.  IRQs are enabled by thread_bootstrap_rv64 before this runs.
static void idle_thread_rv64(void* arg) {
    (void)arg;
    for (;;) asm volatile("wfi");
}

static void ui_thread_entry_rv64(void* arg) {
    (void)arg;
    auto* uart = ::hal::qemu_virt_rv64::g_platform_instance.get_uart_ops();
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    const bool display_ok = kernel::ui::init_display_backend();
    if (display_ok) {
        if (uart) uart->puts("[rv64-ui] display backend initialized\n");
    } else {
        if (uart) uart->puts("[rv64-ui] display backend FAILED - framebuffer only\n");
    }

    kernel::ui::boot_ui_once();

    uint64_t next_ns = timer ? timer->get_system_time_ns() + 750000000ULL : 0;
    if (timer) timer->wait_until_ns(next_ns);
    else for (uint32_t i = 0; i < 1000000; ++i) kernel::util::cpu_relax();

    constexpr uint64_t UI_PERIOD_NS = 100000000ULL;
    for (;;) {
        kernel::ui::render_ui_once();
        if (timer) {
            next_ns += UI_PERIOD_NS;
            const uint64_t now = timer->get_system_time_ns();
            if (now > next_ns) next_ns = now;
            else timer->wait_until_ns(next_ns);
        } else {
            kernel::util::cpu_relax();
        }
    }
}

// Demo work threads — pinned to hart 0, print progress lines through the
// timer.wait_until_ns primitive so preemption + voluntary sleep are both
// exercised.  Once the EtherCAT / motion subsystems are ported across they
// move into these slots.
static void test_thread_a(void* arg) {
    (void)arg;
    auto& plat = ::hal::qemu_virt_rv64::g_platform_instance;
    auto& uart = *plat.get_uart_ops();
    auto& timer = *plat.get_timer_ops();
    uart.puts("[thread A] hello from hart 0\n");
    for (int i = 0; i < 5; ++i) {
        uint64_t t = timer.get_system_time_ns() + 500'000ULL;
        timer.wait_until_ns(t);
        uart.puts("[thread A] tick\n");
    }
    uart.puts("[thread A] exiting\n");
    for (;;) {
        uint64_t t = timer.get_system_time_ns() + 2'000'000ULL;
        timer.wait_until_ns(t);
    }
}

static void test_thread_b(void* arg) {
    (void)arg;
    auto& plat = ::hal::qemu_virt_rv64::g_platform_instance;
    auto& uart = *plat.get_uart_ops();
    auto& timer = *plat.get_timer_ops();
    uart.puts("[thread B] hello\n");
    for (int i = 0; i < 4; ++i) {
        uint64_t t = timer.get_system_time_ns() + 700'000ULL;
        timer.wait_until_ns(t);
        uart.puts("[thread B] tick\n");
    }
    uart.puts("[thread B] exiting\n");
    for (;;) {
        uint64_t t = timer.get_system_time_ns() + 2'000'000ULL;
        timer.wait_until_ns(t);
    }
}

// Secondary hart entry.  Runs on harts 1..N-1 after hart 0 releases them
// via the spin-table.  Sets up this hart's mscratch + timer and enters the
// scheduler.  Mirrors the kernel_secondary_main role on arm64 (see
// core.cpp:kernel_secondary_main).
// Helper: emit "[miniOS rv64] hart N <msg>\n" as a single locked sequence so
// two harts printing the same bring-up phase don't interleave.  Only used by
// the slow bring-up path — not in the steady-state IRQ/thread hot path.
static void hart_phase(hal::qemu_virt_rv64::UARTDriver& uart,
                       uint32_t hartid, const char* msg) {
    ::hal::qemu_virt_rv64::UartLockGuard g;
    // Raw putc() calls inside the guard — UARTDriver::puts() would try to
    // re-acquire the (non-recursive) lock and deadlock.
    const char* p = "[miniOS rv64] hart ";
    while (*p) { if (*p == '\n') uart.putc('\r'); uart.putc(*p++); }
    uart.putc('0' + static_cast<char>(hartid & 0xF));
    uart.putc(' ');
    while (*msg) { if (*msg == '\n') uart.putc('\r'); uart.putc(*msg++); }
    uart.putc('\r'); uart.putc('\n');
}

static uint64_t read_csr_misa() {
    uint64_t v;
    asm volatile("csrr %0, misa" : "=r"(v));
    return v;
}

static uint64_t read_csr_mvendorid() {
    uint64_t v;
    asm volatile("csrr %0, mvendorid" : "=r"(v));
    return v;
}

static uint64_t read_csr_marchid() {
    uint64_t v;
    asm volatile("csrr %0, marchid" : "=r"(v));
    return v;
}

static uint64_t read_csr_mimpid() {
    uint64_t v;
    asm volatile("csrr %0, mimpid" : "=r"(v));
    return v;
}

static uint64_t read_csr_mhartid() {
    uint64_t v;
    asm volatile("csrr %0, mhartid" : "=r"(v));
    return v;
}

static void boot_hw_line(hal::qemu_virt_rv64::UARTDriver& uart,
                         const char* label, uint64_t value) {
    ::hal::qemu_virt_rv64::UartLockGuard g;
    const char* p = "[miniOS rv64] hart 0 hw: ";
    while (*p) { if (*p == '\n') uart.putc('\r'); uart.putc(*p++); }
    while (*label) { if (*label == '\n') uart.putc('\r'); uart.putc(*label++); }
    uart.putc('0');
    uart.putc('x');
    static constexpr char hex[] = "0123456789ABCDEF";
    for (int shift = 60; shift >= 0; shift -= 4) {
        uart.putc(hex[(value >> shift) & 0xF]);
    }
    uart.putc('\r'); uart.putc('\n');
}

static const char* dma_engine_name(kernel::hal::dma::EngineKind kind) {
    switch (kind) {
        case kernel::hal::dma::EngineKind::Software: return "software";
        case kernel::hal::dma::EngineKind::Hardware: return "hardware";
        case kernel::hal::dma::EngineKind::None:     return "none";
    }
    return "unknown";
}

static void boot_dma_selftest(hal::qemu_virt_rv64::UARTDriver& uart,
                              kernel::hal::DMAControllerOps* dma) {
    if (!dma) {
        uart.puts("[boot] dma self-test: no controller\n");
        return;
    }
    const auto caps = dma->get_capabilities();
    if (!caps.available) {
        uart.puts("[boot] dma self-test: unavailable\n");
        return;
    }

    alignas(16) uint8_t src[256];
    alignas(16) uint8_t dst[256];
    for (size_t i = 0; i < sizeof(src); ++i) {
        src[i] = static_cast<uint8_t>((i * 37u + 0x5Au) & 0xFFu);
        dst[i] = 0;
    }
    kernel::hal::dma::TransferConfig cfg{
        reinterpret_cast<uintptr_t>(src),
        reinterpret_cast<uintptr_t>(dst),
        sizeof(src),
        kernel::hal::dma::Direction::MEM_TO_MEM,
        true,
        true,
        16
    };
    const bool submitted = dma->submit_transfer(cfg, nullptr, nullptr);
    bool passed = submitted;
    for (size_t i = 0; i < sizeof(src); ++i) {
        if (src[i] != dst[i]) {
            passed = false;
            break;
        }
    }

    uart.puts("[boot] dma self-test: engine=");
    uart.puts(dma_engine_name(caps.engine_kind));
    uart.puts(" driver=");
    uart.puts(caps.driver_name ? caps.driver_name : "unknown");
    uart.puts(passed ? " PASS\n" : " FAIL\n");
}

extern "C" void hart_main(uint32_t hartid) {
    using namespace ::hal::qemu_virt_rv64;

    auto& plat  = g_platform_instance;
    auto& uart  = *plat.get_uart_ops();
    auto& timer = *plat.get_timer_ops();
    auto& plic  = *plat.get_irq_ops();

    // Phase 1: prove we got here from the spin-table.  First output on this
    // hart — if this never prints, the hart never escaped _start's spin.
    hart_phase(static_cast<UARTDriver&>(uart), hartid,
               "released from spin-table");

    setup_this_hart_trap_stack(hartid);

    plic.init_cpu_interface(hartid);

    // One idle thread per secondary hart.  It's the fallback when nothing
    // else is runnable.  Name encodes the hart id for `top`-style listings.
    char idle_name[12] = "idle_hX";
    idle_name[6] = '0' + static_cast<char>(hartid);
    sched_create_thread(&idle_thread_rv64, nullptr, hartid, idle_name,
                        /*is_idle=*/true);

    // Program this hart's periodic tick.  Because the MTI path calls into
    // rv64_preemptive_tick, the scheduler will run on every tick.
    timer.init_core_timer_interrupt(hartid);

    // Publish that this secondary completed its minimal per-hart bring-up
    // and is ready to accept pinned work.
    g_secondary_ready_mask.fetch_or(1u << hartid, std::memory_order_release);

    // Does not return.  Enters the idle thread (no work threads pinned to
    // secondaries yet — add sched_create_thread calls before this line to
    // park RT work on a non-hart-0 core).
    sched_enter_first(hartid);
}

extern "C" void kernel_main_rv64(uint32_t hartid) {
    using namespace ::hal::qemu_virt_rv64;

    setup_this_hart_trap_stack(hartid);

    auto& plat  = g_platform_instance;
    kernel::g_platform = &plat;
    auto& uart  = *plat.get_uart_ops();
    auto& timer = *plat.get_timer_ops();
    auto& plic  = *plat.get_irq_ops();
    auto* dma   = plat.get_dma_ops();
    auto* usb   = plat.get_usb_ops();

    uart.puts("[miniOS rv64] hart ");
    uart.uart_put_uint64_hex(static_cast<uint64_t>(hartid));
    uart.puts(" up\n");
    boot_hw_line(static_cast<UARTDriver&>(uart), "mhartid=", read_csr_mhartid());
    boot_hw_line(static_cast<UARTDriver&>(uart), "misa=", read_csr_misa());
    boot_hw_line(static_cast<UARTDriver&>(uart), "mvendorid=", read_csr_mvendorid());
    boot_hw_line(static_cast<UARTDriver&>(uart), "marchid=", read_csr_marchid());
    boot_hw_line(static_cast<UARTDriver&>(uart), "mimpid=", read_csr_mimpid());
    boot_dma_selftest(static_cast<UARTDriver&>(uart), dma);

    // Detect CLINT timebase from the DTB before programming mtimecmp.
    const auto tb = detect_timebase_hz();
    timer.init_system_timer_properties(tb.hz);
    uint64_t ecam_base = PCIE_ECAM_BASE;
    uint64_t ecam_size = PCIE_ECAM_SIZE;
    uint64_t mmio_base = PCIE_MMIO_BASE;
    uint64_t mmio_size = PCIE_MMIO_SIZE;
    uint32_t bus_start = 0;
    uint32_t bus_end = 0xff;
    {
        if (usb &&
            fdt::g_dtb_ptr != 0 &&
            fdt::g_dtb_ptr != 0xFFFFFFFFFFFFFFFFULL) {
            (void)::hal::shared::fdt::find_compatible_reg(reinterpret_cast<const void*>(fdt::g_dtb_ptr),
                                                          "pci-host-ecam-generic",
                                                          &ecam_base, &ecam_size,
                                                          &mmio_base, &mmio_size,
                                                          &bus_start, &bus_end);
        }
        if (usb) {
            static_cast<USBHostController*>(usb)->set_pci_host({ecam_base,
                                                                mmio_base,
                                                                mmio_size,
                                                                static_cast<uint8_t>(bus_start & 0xffu),
                                                                static_cast<uint8_t>(bus_end & 0xffu),
                                                                true});
        }
    }
    // Print the real frequency. No libc printf, so emit MHz as an integer
    // (tb.hz / 1e6) followed by a note if we fell back to the hard-coded
    // constant instead of the DTB.
    auto put_dec = [&](uint64_t v) {
        char buf[21]; int i = 20; buf[i--] = '\0';
        if (v == 0) { uart.puts("0"); return; }
        while (v && i >= 0) { buf[i--] = static_cast<char>('0' + (v % 10)); v /= 10; }
        uart.puts(&buf[i + 1]);
    };
    uart.puts("[miniOS rv64] timer init @ ");
    put_dec(tb.hz / 1'000'000ULL);
    uart.puts(" MHz, period ");
    put_dec(TICK_PERIOD_US);
    uart.puts(" us");
    if (!tb.from_dtb) uart.puts(" (DTB unavailable, using 10 MHz fallback)");
    uart.puts("\n");

    if (usb) {
        const bool usb_ok = kernel::usb::init(usb);
        const auto usb_info = kernel::usb::controller_info();
        if (usb_ok && usb_info.present) {
            uart.puts("[boot] usb: driver=");
            uart.puts(usb_info.driver_name ? usb_info.driver_name : "unknown");
            uart.puts(" transport=");
            uart.puts(usb_info.transport ? usb_info.transport : "unknown");
            uart.puts(" ports=");
            put_dec(usb_info.port_count);
            uart.puts(" enum=");
            uart.puts(usb_info.enum_state ? usb_info.enum_state : "unknown");
            uart.puts(" port=");
            put_dec(usb_info.active_port);
            uart.puts(" addressed=");
            put_dec(usb_info.device_addressed ? 1 : 0);
            uart.puts(" desc=");
            put_dec(usb_info.device_descriptor_read ? 1 : 0);
            uart.puts("\n");
        } else {
            uart.puts("[boot] usb: unavailable\n");
        }
    }

    plic.init_distributor();
    uart.puts("[miniOS rv64] plic distributor initialized\n");
    plic.init_cpu_interface(hartid);
    uart.puts("[miniOS rv64] hart 0 plic ctx ready\n");

    // Probe the virtio-mmio bus and bind any virtio-net devices. Default target
    // layout is eth0 = HMI, eth1 = EtherCAT-A, eth2 = EtherCAT-B/fake-slave.
    // IRQ lines are enabled first, then narrowed to the placement-selected hart
    // after the placement TSV is loaded below.
    plat.discover_virtio_nets();

    // Release secondary harts via the spin-table. They'll run hart_main.
    uint64_t entry = reinterpret_cast<uint64_t>(&hart_main);
    for (uint32_t h = 1; h < MAX_HARTS; ++h) {
        g_secondary_entry[h] = entry;
    }
    // Make the store visible to polling secondaries BEFORE kicking them.
    asm volatile("fence w,w" ::: "memory");
    uart.puts("[miniOS rv64] hart 0 releasing secondaries via spin-table\n");
    // Kick secondary harts out of WFI via IPI (software interrupt). Harts
    // poll the spin table and fall through once their slot is non-zero; the
    // WFI in cpu_rv64.S is woken by any pending interrupt, and writing MSIP
    // asserts the MSI pending bit. Note we don't need MSIP enabled to wake
    // from WFI — WFI wakes on any pending interrupt, even if masked.
    for (uint32_t h = 1; h < MAX_HARTS; ++h) {
        *reinterpret_cast<volatile uint32_t*>(CLINT_MSIP + 4ULL * h) = 1;
    }

    // Wait briefly for secondaries to finish their per-hart IRQ/timer setup.
    // This is better than a blind fixed delay: we only wait as long as needed,
    // and we log if a hart misses the bounded window.
    const uint32_t expected_secondary_mask = ((1u << MAX_HARTS) - 1u) & ~1u;
    const uint64_t settle_deadline_ns = timer.get_system_time_ns() + 20'000'000ULL;
    while ((g_secondary_ready_mask.load(std::memory_order_acquire) & expected_secondary_mask) !=
           expected_secondary_mask) {
        if (timer.get_system_time_ns() >= settle_deadline_ns) {
            char buf[96];
            kernel::util::k_snprintf(buf, sizeof(buf),
                                     "[miniOS rv64] secondary ready timeout mask=0x%08lx\n",
                                     static_cast<unsigned long>(
                                         g_secondary_ready_mask.load(std::memory_order_acquire)));
            uart.puts(buf);
            break;
        }
        kernel::util::cpu_relax();
    }

    uart.puts("[boot] loading devices...\n");
    devices::load_all_embedded();
    uart.puts("[boot] devices ok\n");

    const uintptr_t ui_start = reinterpret_cast<uintptr_t>(&_binary_embedded_ui_tsv_start);
    const uintptr_t ui_end = reinterpret_cast<uintptr_t>(&_binary_embedded_ui_tsv_end);
    const size_t ui_len = ui_end - ui_start;
    if (ui_len > 0) {
        if (ui_builder::load_tsv(reinterpret_cast<const char*>(ui_start), ui_len)) {
            uart.puts("[boot] ui template ok\n");
        } else {
            uart.puts("[boot] ui template FAILED\n");
        }
    } else {
        uart.puts("[boot] ui template missing\n");
    }
    const uintptr_t macros_start = reinterpret_cast<uintptr_t>(&_binary_embedded_macros_tsv_start);
    const uintptr_t macros_end = reinterpret_cast<uintptr_t>(&_binary_embedded_macros_tsv_end);
    if (macros_end > macros_start) {
        (void)macros::g_runtime.load_tsv(reinterpret_cast<const char*>(macros_start), macros_end - macros_start);
    }
    const uintptr_t ladder_start = reinterpret_cast<uintptr_t>(&_binary_embedded_ladder_tsv_start);
    const uintptr_t ladder_end = reinterpret_cast<uintptr_t>(&_binary_embedded_ladder_tsv_end);
    if (ladder_end > ladder_start) {
        (void)ladder::g_runtime.load_tsv(reinterpret_cast<const char*>(ladder_start), ladder_end - ladder_start);
    }
    const uintptr_t toolpods_start = reinterpret_cast<uintptr_t>(&_binary_embedded_toolpods_tsv_start);
    const uintptr_t toolpods_end = reinterpret_cast<uintptr_t>(&_binary_embedded_toolpods_tsv_end);
    if (toolpods_end > toolpods_start) {
        (void)machine::toolpods::g_service.load_tsv(reinterpret_cast<const char*>(toolpods_start), toolpods_end - toolpods_start);
    }
    const uintptr_t signals_start = reinterpret_cast<uintptr_t>(&_binary_embedded_signals_tsv_start);
    const uintptr_t signals_end = reinterpret_cast<uintptr_t>(&_binary_embedded_signals_tsv_end);
    if (signals_end > signals_start) {
        (void)machine::g_registry.load_signal_tsv(reinterpret_cast<const char*>(signals_start), signals_end - signals_start);
    }
    const uintptr_t topology_start = reinterpret_cast<uintptr_t>(&_binary_embedded_topology_tsv_start);
    const uintptr_t topology_end = reinterpret_cast<uintptr_t>(&_binary_embedded_topology_tsv_end);
    if (topology_end > topology_start) {
        (void)machine::topology::g_service.load_tsv(reinterpret_cast<const char*>(topology_start), topology_end - topology_start);
    }
    const uintptr_t placement_start = reinterpret_cast<uintptr_t>(&_binary_embedded_placement_tsv_start);
    const uintptr_t placement_end = reinterpret_cast<uintptr_t>(&_binary_embedded_placement_tsv_end);
    if (placement_end > placement_start) {
        (void)machine::placement::g_service.load_tsv(reinterpret_cast<const char*>(placement_start), placement_end - placement_start);
    }
    const uintptr_t hmi_start = reinterpret_cast<uintptr_t>(&_binary_embedded_hmi_tsv_start);
    const uintptr_t hmi_end = reinterpret_cast<uintptr_t>(&_binary_embedded_hmi_tsv_end);
    if (hmi_end > hmi_start) {
        (void)hmi::g_service.load_tsv(reinterpret_cast<const char*>(hmi_start), hmi_end - hmi_start);
    }
    cli::io::init(&uart);

    machine::placement::Config placement{};
    machine::placement::g_service.snapshot(placement);
    const uint32_t num_cores = plat.get_num_cores();
    const uint32_t num_nets = static_cast<uint32_t>(plat.get_num_nets());
    if (num_nets > 0) {
        auto log_net_role = [&](const char* role, uint32_t nic, uint32_t core) {
            char buf[128];
            if (nic < num_nets) {
                kernel::util::k_snprintf(buf, sizeof(buf),
                                         "[boot] net-role %s: eth%u -> core%u\n",
                                         role ? role : "unknown",
                                         static_cast<unsigned>(nic),
                                         static_cast<unsigned>(core));
            } else {
                kernel::util::k_snprintf(buf, sizeof(buf),
                                         "[boot] net-role %s: eth%u unavailable (nics=%u)\n",
                                         role ? role : "unknown",
                                         static_cast<unsigned>(nic),
                                         static_cast<unsigned>(num_nets));
            }
            uart.puts(buf);
        };
        const uint32_t hmi_nic = placement.hmi_nic;
        const uint32_t hmi_core = machine::placement::g_service.sanitize_core(placement.hmi_irq_core, num_cores);
        log_net_role("hmi", hmi_nic, hmi_core);
        if (hmi_nic < num_nets) {
            plat.route_net_irq(static_cast<int>(hmi_nic), 1u << hmi_core);
        }
        const uint32_t ec_a_nic = placement.ec_a_nic;
        log_net_role("ec_a", ec_a_nic,
                     machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores));
        if (ec_a_nic < num_nets) {
            ethercat::g_master_a.set_nic_index(static_cast<int>(ec_a_nic));
            plat.route_net_irq(static_cast<int>(ec_a_nic), 1u << machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores));
        }
        const uint32_t fake_slave_nic = placement.fake_slave_nic;
        log_net_role("fake_slave", fake_slave_nic,
                     machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores));
        if (fake_slave_nic < num_nets) {
            ethercat::g_fake_slave.set_nic_idx(static_cast<int>(fake_slave_nic));
            plat.route_net_irq(static_cast<int>(fake_slave_nic), 1u << machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores));
        }
    }
    const uint32_t cli_core = machine::placement::g_service.sanitize_core(placement.cli_core, num_cores);
    const uint32_t uart_io_core = machine::placement::g_service.sanitize_core(placement.uart_io_core, num_cores);
    const uint32_t ui_core = machine::placement::g_service.sanitize_core(placement.ui_core, num_cores);
    const uint32_t motion_core = machine::placement::g_service.sanitize_core(placement.motion_core, num_cores);
    const uint32_t gcode_core = machine::placement::g_service.sanitize_core(placement.gcode_core, num_cores);
    const uint32_t macro_core = machine::placement::g_service.sanitize_core(placement.macro_core, num_cores);
    const uint32_t ladder_core = machine::placement::g_service.sanitize_core(placement.ladder_core, num_cores);
    const uint32_t probe_core = machine::placement::g_service.sanitize_core(placement.probe_core, num_cores);
    const uint32_t hmi_core = machine::placement::g_service.sanitize_core(placement.hmi_irq_core, num_cores);
    const uint32_t ec_a_core = machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores);
    const uint32_t fake_slave_core = machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores);
    const uint32_t bus_config_core = machine::placement::g_service.sanitize_core(placement.bus_config_core, num_cores);
    const uint32_t hmi_nic = placement.hmi_nic;
    const uint32_t ec_a_nic = placement.ec_a_nic;
    const uint32_t fake_slave_nic = placement.fake_slave_nic;

    sched_create_thread(&idle_thread_rv64, nullptr, 0, "idle_h0",
                        /*is_idle=*/true);
    // On rv64 the scheduler is FIFO round-robin without priorities. Start the
    // shared UI thread before CLI/UART so display bring-up is not gated by a
    // blocking serial reader on hart 0.
    sched_create_thread(&kernel::ui::boot_ui_thread_entry, nullptr, ui_core, "ui", false);
    sched_create_thread(&cli::CLI::thread_entry, nullptr, cli_core, "cli", false);
    sched_create_thread(&cli::io::uart_io_entry, &uart, uart_io_core, "uart_io", false);

    (void)machine::wiring::wire_motion_axes_from_topology(&uart);
    sched_create_thread(&motion::Kernel::thread_entry, nullptr, motion_core, "motion", false);
    sched_create_thread(&cnc::interp::Runtime::thread_entry, nullptr, gcode_core, "gcode", false);
    sched_create_thread(&macros::Runtime::thread_entry, nullptr, macro_core, "macro", false);
    sched_create_thread(&ladder::Runtime::thread_entry, nullptr, ladder_core, "ladder", false);
    sched_create_thread(&probe::Runtime::thread_entry, nullptr, probe_core, "probe", false);
    if (num_nets > hmi_nic) {
        hmi::g_service.configure(static_cast<uint8_t>(hmi_nic));
        uart.puts("[boot] hmi: eth0 service...\n");
        sched_create_thread(&hmi::Service::thread_entry, &hmi::g_service, hmi_core, "hmi", false);
    }

    if (num_nets > ec_a_nic) {
        ethercat::g_master_a.set_nic_index(static_cast<int>(ec_a_nic));
        sched_create_thread(&ethercat::Master::thread_entry, &ethercat::g_master_a, ec_a_core, "ec_a", false);
        sched_create_thread(&ethercat::bus_config_entry, &ethercat::g_master_a, bus_config_core, "bus_cfg_a", false);
#if MINIOS_FAKE_SLAVE
        if (num_nets > fake_slave_nic) {
            ethercat::g_fake_slave.set_nic_idx(static_cast<int>(fake_slave_nic));
            sched_create_thread(&ethercat::FakeSlave::thread_entry, &ethercat::g_fake_slave, fake_slave_core, "fake_slave", false);
        }
#else
        const uint32_t ec_b_core = machine::placement::g_service.sanitize_core(placement.ec_b_core, num_cores);
        const uint32_t ec_b_nic = placement.ec_b_nic;
        if (num_nets > ec_b_nic) {
            ethercat::g_master_b.set_nic_index(static_cast<int>(ec_b_nic));
            sched_create_thread(&ethercat::Master::thread_entry, &ethercat::g_master_b, ec_b_core, "ec_b", false);
            sched_create_thread(&ethercat::bus_config_entry, &ethercat::g_master_b, bus_config_core, "bus_cfg_b", false);
        }
#endif
    } else {
        sched_create_thread(&test_thread_a, nullptr, 0, "thread_a", false);
        sched_create_thread(&test_thread_b, nullptr, 0, "thread_b", false);
    }

    uart.puts("[miniOS rv64] scheduler: services armed on harts 0-2\n");

    // Program hart 0's periodic scheduler tick.
    timer.init_core_timer_interrupt(hartid);

    uart.puts("[miniOS rv64] entering scheduler on hart 0\n");

    // Does not return — jumps into the highest-priority runnable thread.
    sched_enter_first(hartid);
}
