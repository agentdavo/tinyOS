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
#include "../../kernel/main.hpp"
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

// Voluntary context-switch primitive defined in cpu_rv64.S. Re-declared
// inside the namespace so both enter_first_thread (here) and the stubs TU
// can call it without exporting the symbol through the shared HAL surface.
extern "C" void cpu_context_switch_rv64(kernel::core::TCB* old_tcb,
                                        kernel::core::TCB* new_tcb);

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
    // Yield while RX FIFO is empty so the uart_io thread doesn't starve the
    // cli thread sharing its core. Mirrors the arm64 PL011 path.
    while ((*uart_reg(NS16550_LSR) & LSR_DR) == 0) {
        if (kernel::g_scheduler_ptr && kernel::g_platform) {
            kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        }
    }
    return static_cast<char>(*uart_reg(NS16550_RBR));
}
// Non-blocking poll. NS16550 LSR bit 0 (Data Ready) is set when RBR has
// a byte waiting. Mirrors arm64 PL011 try_getc; lets cli's uart_io
// interleave output drains with input polling.
bool UARTDriver::try_getc(char& out) {
    if ((*uart_reg(NS16550_LSR) & LSR_DR) == 0) return false;
    out = static_cast<char>(*uart_reg(NS16550_RBR));
    return true;
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
    // No PCI host bridge discovered. The shared xhci probe would scan a
    // zero-base ECAM window and hang on invalid MMIO; bail fast instead.
    if (pci_host_.ecam_base == 0) return false;
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
    // Enable MTIE + MEIE in mie so the CLINT/PLIC sources are routed to
    // this hart, but do NOT flip mstatus.MIE here. arm64's equivalent
    // only enables the GIC CPU interface — it leaves PSTATE.DAIF masked
    // until eret atomically loads SPSR from the new TCB. rv64 needs the
    // same pattern: leaving MIE off here closes the window between
    // start_core_scheduler returning and cpu_context_switch_rv64's mret,
    // so a timer IRQ can't fire on the boot stack and corrupt the
    // in-flight context switch via the trap-entry TCB-spill path.
    // mret restores MIE atomically from new_tcb.pstate (MPIE=1).
    uint64_t bits = (1ULL << 7) | (1ULL << 11); // MTIE + MEIE
    asm volatile("csrs mie, %0" :: "r"(bits) : "memory");
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
// PowerOps — wfi-based idle for Scheduler::idle_thread_func.
// ----------------------------------------------------------------------------
void PowerOps::enter_idle_state(uint32_t /*core_id*/) {
    // wfi parks the hart until any interrupt is pending. The shared idle loop
    // then yields back to the scheduler so a freshly-enqueued worker can run.
    asm volatile("wfi");
}
bool PowerOps::set_cpu_frequency(uint32_t, uint32_t) { return true; }

// ----------------------------------------------------------------------------
// Platform
// ----------------------------------------------------------------------------
uint32_t PlatformQEMUVirtRV64::get_core_id() const {
    uint64_t h; asm volatile("csrr %0, mhartid" : "=r"(h)); return static_cast<uint32_t>(h);
}

void PlatformQEMUVirtRV64::early_init_core(uint32_t core_id) {
    // Per-hart bring-up shared by hart_main (secondaries) and the boot
    // sequence. Mirrors arm64's PlatformQEMUVirtARM64::early_init_core.
    if (core_id >= MAX_HARTS) return;
    plic_.init_cpu_interface(core_id);
    timer_.init_core_timer_interrupt(core_id);
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

bool PlatformQEMUVirtRV64::init_block_device() {
    // Scan virtio-mmio bus for a virtio-blk device. If QEMU wasn't started
    // with `-drive ... -device virtio-blk-device`, the scan returns nullptr
    // and we leave fs_ops_ready_ false so get_fs_ops() stays nullptr and
    // the VFS falls back to embedded-defaults-only.
    virtio_blk_ = ::hal::shared::virtio::discover_virtio_blk(
        VIRTIO_MMIO_BASE, VIRTIO_MMIO_STRIDE, VIRTIO_MMIO_NUM);
    if (!virtio_blk_) return false;
    blk_reader_.bind(virtio_blk_);
    if (!fs_ops_.mount()) return false;
    fs_ops_ready_ = true;
    return true;
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

// HAL glue for the shared kernel::hal::get_platform(). Both arches expose the
// same surface; rv64's instance lives in the qemu_virt_rv64 namespace.
namespace kernel::hal {
Platform& get_platform_instance() { return ::hal::qemu_virt_rv64::g_platform_instance; }
Platform* get_platform() { return &::hal::qemu_virt_rv64::g_platform_instance; }
}


// ----------------------------------------------------------------------------
// Trap dispatcher (called from trap_entry in cpu_rv64.S)
// ----------------------------------------------------------------------------
// Storage for the rv64 in-trap flag. Two TUs (this one and rv64_stubs.cpp) read
// it; only this TU writes (from the trap dispatcher).
namespace hal::qemu_virt_rv64 {
extern "C" {
volatile uint64_t g_irq_in_progress[MAX_HARTS] = {0, 0, 0, 0};
}
}

extern "C" void hal_trap_dispatch_rv64(uint64_t hartid, uint64_t mcause) {
    using ::hal::qemu_virt_rv64::g_irq_in_progress;
    using namespace ::hal::qemu_virt_rv64;
    constexpr uint64_t MCAUSE_INT = 1ULL << 63;

    // Mark this hart as being inside the trap dispatcher so cpu_context_switch_impl
    // skips its voluntary save/restore — the trap entry path already spilled the
    // outgoing thread's state, and trap exit will reload from
    // g_per_cpu_data[hart].current_thread.
    if (hartid < MAX_HARTS) g_irq_in_progress[hartid] = 1;
    struct IrqGuard {
        uint64_t h;
        ~IrqGuard() { if (h < MAX_HARTS) g_irq_in_progress[h] = 0; }
    } irq_guard{hartid};

    if (mcause & MCAUSE_INT) {
        uint64_t code = mcause & 0x7FFFFFFFFFFFFFFFULL;
        if (code == 7) {
            // Machine Timer Interrupt. Re-arm mtimecmp before consulting the
            // scheduler (so the IRQ deasserts cleanly), then either run the
            // shared preemptive_tick or — on tickless dedicated RT cores —
            // just leave the wake to wait_until_ns's caller.
            if (hartid < MAX_HARTS) {
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
                if (kernel::hal::is_dedicated_rt_core(static_cast<uint32_t>(hartid))) {
                    // Tickless RT core: wake from WFI is the whole point;
                    // skip preemptive_tick to keep ticks_total == 0 for this
                    // core (mirrors arm64's hal_irq_handler behaviour).
                    return;
                }
                if (kernel::g_scheduler_ptr) {
                    kernel::g_scheduler_ptr->preemptive_tick(static_cast<uint32_t>(hartid));
                }
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

    // Synchronous exception. Build the whole header line in one buffer
    // and emit via a single locked puts() so concurrent puts() from
    // another hart don't fragment it. mtval is useful when chasing a
    // load/store fault — it holds the faulting address.
    auto& uart = *g_platform_instance.get_uart_ops();
    uint64_t mepc, mtval;
    asm volatile("csrr %0, mepc"  : "=r"(mepc));
    asm volatile("csrr %0, mtval" : "=r"(mtval));
    uint64_t ra_val = 0, sp_val = 0;
    const char* tcb_name = "?";
    if (hartid < kernel::core::MAX_CORES) {
        if (auto* tcb = kernel::core::g_per_cpu_data[hartid].current_thread) {
            ra_val = tcb->regs[0];
            sp_val = tcb->regs[1];
            tcb_name = tcb->name;
        }
    }
    char buf[256];
    char* p = buf;
    auto put_str = [&](const char* s) { while (*s) *p++ = *s++; };
    auto put_hex = [&](uint64_t v) {
        static const char d[] = "0123456789abcdef";
        for (int i = 0; i < 16; ++i) p[i] = d[(v >> ((15 - i) * 4)) & 0xF];
        p += 16;
    };
    put_str("\n[rv64 TRAP] hart="); put_hex(hartid);
    put_str(" mcause=");            put_hex(mcause);
    put_str(" mepc=");              put_hex(mepc);
    put_str(" mtval=");             put_hex(mtval);
    put_str(" ra=");                put_hex(ra_val);
    put_str(" sp=");                put_hex(sp_val);
    put_str(" name=");
    for (int i = 0; i < 16 && tcb_name[i]; ++i) *p++ = tcb_name[i];
    *p++ = '\n'; *p = '\0';
    uart.puts(buf);
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

// Voluntary context switch defined in cpu_rv64.S.
extern "C" void cpu_context_switch_rv64(kernel::core::TCB* old_tcb,
                                        kernel::core::TCB* new_tcb);

// Enter the highest-priority ready thread for this hart for the first time.
// The shared Scheduler::start_core_scheduler installs the chosen worker into
// g_per_cpu_data[hart].current_thread but does not switch (it expects the
// caller's asm to do the eret/mret). On rv64 we drive that switch from C.
[[noreturn]] static void enter_first_thread(uint32_t hartid) {
    auto* tcb = kernel::core::g_per_cpu_data[hartid].current_thread;
    if (!tcb) tcb = kernel::core::g_per_cpu_data[hartid].idle_thread;
    if (!tcb) for (;;) asm volatile("wfi");
    cpu_context_switch_rv64(nullptr, tcb);
    __builtin_unreachable();
}

extern "C" void hart_main(uint32_t hartid) {
    using namespace ::hal::qemu_virt_rv64;

    auto& plat  = g_platform_instance;
    auto& uart  = *plat.get_uart_ops();

    // Phase 1: prove we got here from the spin-table.  First output on this
    // hart — if this never prints, the hart never escaped _start's spin.
    hart_phase(static_cast<UARTDriver&>(uart), hartid,
               "released from spin-table");

    setup_this_hart_trap_stack(hartid);

    plat.early_init_core(hartid);

    // Publish that this secondary completed its minimal per-hart bring-up
    // and is ready to accept pinned work. Idle threads were created up-front
    // by kernel_main_rv64 (one per hart, via the shared scheduler).
    g_secondary_ready_mask.fetch_or(1u << hartid, std::memory_order_release);

    if (!kernel::g_scheduler_ptr) for (;;) asm volatile("wfi");
    kernel::g_scheduler_ptr->start_core_scheduler(hartid);
    enter_first_thread(hartid);
}

extern "C" void kernel_main_rv64(uint32_t hartid) {
    using namespace ::hal::qemu_virt_rv64;

    setup_this_hart_trap_stack(hartid);

    auto& plat  = g_platform_instance;
    kernel::g_platform = &plat;

    // Stand the shared scheduler up before any thread creation. Mirrors the
    // arm64 initialize_platform_and_scheduler() prelude — same EDFPolicy,
    // same Scheduler instance type, same g_scheduler_ptr global.
    static kernel::core::Scheduler scheduler_instance;
    static kernel::core::EDFPolicy edf_policy;
    scheduler_instance.set_policy(&edf_policy);
    kernel::g_scheduler_ptr = &scheduler_instance;
    if (!kernel::core::g_software_timer_obj_pool.init(
            kernel::core::g_software_timer_obj_pool_mem,
            kernel::core::MAX_SOFTWARE_TIMERS,
            sizeof(kernel::hal::timer::SoftwareTimer),
            alignof(kernel::hal::timer::SoftwareTimer))) {
        plat.panic("Failed to init software timer pool", __FILE__, __LINE__);
    }

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

    // Idle threads are created up-front for every hart so the shared scheduler
    // has a fallback in g_per_cpu_data[hart].idle_thread before any secondary
    // hart starts running its scheduler tick. The shared idle_thread_func
    // wfi-sleeps via PowerOps::enter_idle_state and yields cooperatively after
    // each wake.
    for (uint32_t h = 0; h < MAX_HARTS; ++h) {
        char idle_name[kernel::core::MAX_NAME_LENGTH];
        kernel::util::k_snprintf(idle_name, sizeof(idle_name), "idle%u", h);
        if (!kernel::g_scheduler_ptr->create_thread(
                &kernel::core::Scheduler::idle_thread_func,
                reinterpret_cast<void*>(static_cast<uintptr_t>(h)),
                0, static_cast<int>(h), idle_name, true, 0)) {
            plat.panic("Failed to create idle thread", __FILE__, __LINE__);
        }
    }

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

    // VFS + block-device + devices-TSV load, same sequence as arm64.
    kernel::boot::init_vfs();

    uart.puts("[boot] loading devices...\n");
    devices::load_all_embedded();
    uart.puts("[boot] devices ok\n");

    // Shared boot helpers — identical sequence on arm64. See kernel/main.hpp.
    kernel::boot::load_runtime_tsvs();
    cli::io::init(&uart);
    kernel::boot::log_and_route_net_roles();

    // Bring up display + input on the synchronous boot path so the scanout is
    // live before the scheduler starts. The UI thread created below just runs
    // the refresh loop.
    kernel::ui::init_ui_backends();
    kernel::ui::boot_ui_once();

    auto rv64_create_thread = [](void (*fn)(void*), void* arg, int prio,
                                 int affinity, const char* name, bool is_idle,
                                 uint64_t deadline_us) -> bool {
        return kernel::g_scheduler_ptr->create_thread(fn, arg, prio, affinity,
                                                      name, is_idle, deadline_us) != nullptr;
    };

    kernel::boot::create_boot_services(rv64_create_thread);
    (void)machine::wiring::wire_motion_axes_from_topology(&uart);
    kernel::boot::create_runtime_services(rv64_create_thread);

    uart.puts("[miniOS rv64] scheduler: services armed on harts 0-2\n");

    uart.puts("[miniOS rv64] entering scheduler on hart 0\n");

    // Arm the periodic scheduler tick for the primary hart before handing
    // off. Secondary harts arm theirs in early_init_core via hart_main.
    timer.init_core_timer_interrupt(hartid);

    // start_core_scheduler installs the chosen worker into
    // g_per_cpu_data[hartid].current_thread and enables the timer IRQ;
    // enter_first_thread does the actual mret-equivalent into that worker.
    kernel::g_scheduler_ptr->start_core_scheduler(hartid);
    enter_first_thread(hartid);
}
