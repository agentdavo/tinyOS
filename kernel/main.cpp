// SPDX-License-Identifier: MIT OR Apache-2.0

#include "kernel/main.hpp"

#include "core.hpp"
#include "miniOS.hpp"
#include "ui/ui_builder_tsv.hpp"

namespace kernel { namespace core { void run_benchmark_test(); } }
#include "trace.hpp"
#include "util.hpp"
#include "cli.hpp"
#include "ethercat/master.hpp"
#include "ethercat/bus_config.hpp"
#if MINIOS_FAKE_SLAVE
#include "ethercat/fake_slave.hpp"
#endif
#include "motion/motion.hpp"
#include "cnc/interpreter.hpp"
#include "automation/macro_runtime.hpp"
#include "automation/ladder_runtime.hpp"
#include "automation/probe_runtime.hpp"
#include "machine/toolpods.hpp"
#include "machine/machine_topology.hpp"
#include "machine/runtime_placement.hpp"
#include "machine/motion_wiring.hpp"
#include "devices/embedded.hpp"
#include "devices/device_db.hpp"
#include "rt/base_thread.hpp"
#include "ui/splash.hpp"
#include "diag/cpu_load.hpp"
#include "hmi/hmi_service.hpp"
#include "kernel/usb/usb.hpp"
#if defined(__aarch64__)
#include "hal/arm64/hal_qemu_arm64.hpp"
#endif

#include <atomic>

extern "C" void early_uart_puts(const char* str);
extern "C" void _secondary_start();
#if defined(__aarch64__) || defined(__riscv)
// UI/machine TSV blobs moved to the VFS layer (fs/vfs.cpp). The remaining
// embedded blobs here are subsystem-internal (macros/ladder/signals/
// topology/placement/hmi) and load straight into their owning service —
// they haven't needed the VFS indirection yet and would just add noise.
extern "C" {
extern const char _binary_embedded_macros_tsv_start[];
extern const char _binary_embedded_macros_tsv_end[];
extern const char _binary_embedded_ladder_tsv_start[];
extern const char _binary_embedded_ladder_tsv_end[];
extern const char _binary_embedded_signals_tsv_start[];
extern const char _binary_embedded_signals_tsv_end[];
extern const char _binary_embedded_topology_tsv_start[];
extern const char _binary_embedded_topology_tsv_end[];
extern const char _binary_embedded_placement_tsv_start[];
extern const char _binary_embedded_placement_tsv_end[];
extern const char _binary_embedded_hmi_tsv_start[];
extern const char _binary_embedded_hmi_tsv_end[];
}
#endif

#include "fs/vfs.hpp"

namespace {

bool create_named_thread(void (*fn)(void*), const void* arg, int prio, int affinity,
                         const char* name, bool is_idle, uint64_t deadline_us,
                         const char* log_label) {
    if (kernel::g_scheduler_ptr->create_thread(fn, arg, prio, affinity, name, is_idle, deadline_us)) {
        return true;
    }
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf), "[boot] thread create failed: %s\n",
                             log_label ? log_label : (name ? name : "unknown"));
    early_uart_puts(buf);
    return false;
}

const kernel::BootHooks* g_boot_hooks          = nullptr;
bool g_multicore_supported                      = false;
#if defined(__aarch64__)
std::atomic<uint32_t> g_arm64_secondary_ready_mask{0};
#endif

char g_boot_timestamp_buf[32];
static bool g_boot_time_captured = false;

void put_boot_timestamp() {
    auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!tm) {
        early_uart_puts("0000us");
        return;
    }
    uint64_t ns = tm->get_system_time_ns() / 1000ULL;
    char* p = g_boot_timestamp_buf;
    uint64_t v = ns;
    for (int i = 3; i >= 0; --i) {
        p[i] = '0' + static_cast<char>(v % 10);
        v /= 10;
    }
    p[4] = 'u';
    p[5] = 's';
    p[6] = '\0';
    early_uart_puts(g_boot_timestamp_buf);
}

void capture_boot_start_time() {
    if (g_boot_time_captured) return;
    auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (tm) {
        (void)tm->get_system_time_ns();  // prime the timer
    }
    g_boot_time_captured = true;
}

void log_timestamped(const char* msg) {
    // Route via the platform UART rather than early_uart_puts — the latter
    // hardcodes arm64's PL011 MMIO base (0x09000000) and faults on rv64.
    // Shared boot helpers are called on both arches, so they have to take
    // the platform-abstract path.
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (!uart) {
        // Very early boot (pre-platform): fall back to arm64 early UART.
        // rv64 never hits this branch because kernel_main_rv64 sets
        // g_platform before any log_timestamped call.
        early_uart_puts("[boot] +");
        put_boot_timestamp();
        early_uart_puts(" ");
        early_uart_puts(msg);
        early_uart_puts("\n");
        return;
    }
    char buf[160];
    uart->puts("[boot] +");
    // Reuse put_boot_timestamp which itself uses early_uart_puts — inline a
    // uart-based version.
    auto* tm = kernel::g_platform->get_timer_ops();
    uint64_t us = tm ? tm->get_system_time_ns() / 1000ULL : 0;
    for (int i = 3; i >= 0; --i) { buf[i] = '0' + static_cast<char>(us % 10); us /= 10; }
    buf[4] = 'u'; buf[5] = 's'; buf[6] = '\0';
    uart->puts(buf);
    uart->puts(" ");
    uart->puts(msg);
    uart->puts("\n");
}

void log_net_role(const char* role, uint8_t nic, uint8_t core, uint32_t num_nets) {
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
    // Platform UART — early_uart_puts is PL011-specific and faults on rv64.
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        uart->puts(buf);
    } else {
        early_uart_puts(buf);
    }
}

long psci_cpu_on_impl(uint64_t mpidr, uint64_t entry_pa, uint64_t context);

const char* dma_engine_name(kernel::hal::dma::EngineKind kind) {
    switch (kind) {
        case kernel::hal::dma::EngineKind::Software: return "software";
        case kernel::hal::dma::EngineKind::Hardware: return "hardware";
        case kernel::hal::dma::EngineKind::None:     return "none";
    }
    return "unknown";
}

void run_dma_boot_selftest() {
    if (!kernel::g_platform) return;
    auto* uart = kernel::g_platform->get_uart_ops();
    auto* dma = kernel::g_platform->get_dma_ops();
    if (!uart) return;
    if (!dma) {
        uart->puts("[boot] dma self-test: no controller\n");
        return;
    }

    const auto caps = dma->get_capabilities();
    if (!caps.available) {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[boot] dma self-test: engine=%s unavailable\n",
                                 dma_engine_name(caps.engine_kind));
        uart->puts(buf);
        return;
    }

    alignas(16) uint8_t src[256];
    alignas(16) uint8_t dst[256];
    for (size_t i = 0; i < sizeof(src); ++i) {
        src[i] = static_cast<uint8_t>((i * 37u + 0x5Au) & 0xFFu);
        dst[i] = 0;
    }

    const kernel::hal::dma::TransferConfig cfg{
        reinterpret_cast<uintptr_t>(src),
        reinterpret_cast<uintptr_t>(dst),
        sizeof(src),
        kernel::hal::dma::Direction::MEM_TO_MEM,
        true,
        true,
        16
    };

    const bool submitted = dma->submit_transfer(cfg, nullptr, nullptr);
    const bool passed = submitted && kernel::util::kmemcmp(src, dst, sizeof(src)) == 0;

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "[boot] dma self-test: engine=%s driver=%s %s\n",
                             dma_engine_name(caps.engine_kind),
                             caps.driver_name ? caps.driver_name : "unknown",
                             passed ? "PASS" : "FAIL");
    uart->puts(buf);
}

void run_usb_boot_probe() {
    if (!kernel::g_platform) return;
    auto* uart = kernel::g_platform->get_uart_ops();
    if (!uart) return;

    const bool ok = kernel::usb::init(kernel::g_platform->get_usb_ops());
    const auto info = kernel::usb::controller_info();
    if (!ok || !info.present) {
        uart->puts("[boot] usb: unavailable\n");
        return;
    }

    char buf[224];
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "[boot] usb: driver=%s transport=%s ports=%lu enum=%s port=%lu addressed=%u desc=%u\n",
                             info.driver_name ? info.driver_name : "unknown",
                             info.transport ? info.transport : "unknown",
                             static_cast<unsigned long>(info.port_count),
                             info.enum_state ? info.enum_state : "unknown",
                             static_cast<unsigned long>(info.active_port),
                             info.device_addressed ? 1u : 0u,
                             info.device_descriptor_read ? 1u : 0u);
    uart->puts(buf);
}

bool initialize_platform_and_scheduler() {
    trace::g_trace_manager.init();
#if TRACE_DEFAULT_ENABLED
    trace::g_trace_manager.set_enabled(true);
#else
    trace::g_trace_manager.set_enabled(false);
#endif

    kernel::g_platform = kernel::hal::get_platform();
    if (!kernel::g_platform) {
        early_uart_puts("[PANIC] Null platform, halting\n");
        return false;
    }

    kernel::g_platform->early_init_platform();

    static kernel::core::Scheduler scheduler_instance;
    static kernel::core::EDFPolicy edf_policy;
    scheduler_instance.set_policy(&edf_policy);
    kernel::g_scheduler_ptr = &scheduler_instance;

    if (!kernel::core::g_software_timer_obj_pool.init(kernel::core::g_software_timer_obj_pool_mem,
                                                      kernel::core::MAX_SOFTWARE_TIMERS,
                                                      sizeof(kernel::hal::timer::SoftwareTimer),
                                                      alignof(kernel::hal::timer::SoftwareTimer))) {
        kernel::g_platform->panic("Failed to init software timer pool", __FILE__, __LINE__);
        return false;
    }

    return true;
}

bool create_idle_threads() {
    for (uint32_t i = 0; i < kernel::g_platform->get_num_cores(); ++i) {
        char idle_name[kernel::core::MAX_NAME_LENGTH];
        kernel::util::k_snprintf(idle_name, sizeof(idle_name), "idle%u", i);
        if (!kernel::g_scheduler_ptr->create_thread(&kernel::core::Scheduler::idle_thread_func,
                reinterpret_cast<void*>(static_cast<uintptr_t>(i)), 0, static_cast<int>(i), idle_name, true, 0)) {
            kernel::g_platform->panic("Failed to create idle thread", __FILE__, __LINE__);
            return false;
        }
    }
    return true;
}

void initialize_core0_boot_services() {
    kernel::g_platform->early_init_core(0);
    run_dma_boot_selftest();
    run_usb_boot_probe();

    kernel::boot::init_vfs();

    log_timestamped("loading devices...");
    devices::load_all_embedded();
    log_timestamped("devices ok");

    kernel::boot::load_runtime_tsvs();
    kernel::boot::log_and_route_net_roles();

    kernel::ui::init_ui_backends();
    kernel::ui::boot_ui_once();

    auto arm64_create_thread = [](void (*fn)(void*), void* arg, int prio,
                                  int affinity, const char* name, bool is_idle,
                                  uint64_t deadline_us) -> bool {
        return kernel::g_scheduler_ptr->create_thread(fn, arg, prio, affinity,
                                                      name, is_idle, deadline_us) != nullptr;
    };
    if (auto* uart = kernel::g_platform->get_uart_ops()) {
        cli::io::init(uart);
    }
    kernel::boot::create_boot_services(arm64_create_thread);
}

void wire_motion_axes_from_device_db() {
    (void)machine::wiring::wire_motion_axes_from_topology(
        kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr);
}

void create_runtime_threads() {
    auto arm64_create_thread = [](void (*fn)(void*), void* arg, int prio,
                                  int affinity, const char* name, bool is_idle,
                                  uint64_t deadline_us) -> bool {
        return kernel::g_scheduler_ptr->create_thread(fn, arg, prio, affinity,
                                                      name, is_idle, deadline_us) != nullptr;
    };
    kernel::boot::create_runtime_services(arm64_create_thread);
}

void release_secondary_cores() {
    if (g_boot_hooks && g_boot_hooks->supports_multicore && g_boot_hooks->init_secondary_core) {
        for (uint32_t i = 1; i < kernel::g_platform->get_num_cores(); ++i) {
            g_boot_hooks->init_secondary_core(i);
        }
        return;
    }

#if defined(__aarch64__)
    for (uint32_t i = 1; i < kernel::g_platform->get_num_cores(); ++i) {
        long rc = psci_cpu_on_impl(static_cast<uint64_t>(i),
                                   reinterpret_cast<uint64_t>(&_secondary_start), i);
        if (rc != 0) {
            const uint64_t legacy_target = 0x80000000ULL | static_cast<uint64_t>(i);
            rc = psci_cpu_on_impl(legacy_target,
                                  reinterpret_cast<uint64_t>(&_secondary_start), i);
        }
        if (rc != 0) {
            char buf[80];
            kernel::util::k_snprintf(buf, sizeof(buf), "[SMP] PSCI_CPU_ON core %u rc=%ld\n", i, rc);
            early_uart_puts(buf);
        }
    }
#endif
}

#if defined(__aarch64__)
void wait_for_arm64_secondary_ready() {
    if (!kernel::g_platform || kernel::g_platform->get_num_cores() <= 1) return;
    auto* timer = kernel::g_platform->get_timer_ops();
    if (!timer) return;

    const uint32_t num_cores = kernel::g_platform->get_num_cores();
    const uint32_t expected_mask = (num_cores >= 32)
                                       ? 0xFFFFFFFEu
                                       : (((1u << num_cores) - 1u) & ~1u);
    const uint64_t deadline_ns = timer->get_system_time_ns() + 20'000'000ULL;
    while ((g_arm64_secondary_ready_mask.load(std::memory_order_acquire) & expected_mask) !=
           expected_mask) {
        if (timer->get_system_time_ns() >= deadline_ns) {
            char buf[96];
            kernel::util::k_snprintf(buf, sizeof(buf),
                                     "[miniOS arm64] secondary ready timeout mask=0x%08lx\n",
                                     static_cast<unsigned long>(
                                         g_arm64_secondary_ready_mask.load(std::memory_order_acquire)));
            early_uart_puts(buf);
            break;
        }
        kernel::util::cpu_relax();
    }
}
#endif

#if defined(__aarch64__)
long psci_cpu_on_impl(uint64_t mpidr, uint64_t entry_pa, uint64_t context) {
    register long x0 asm("x0") = 0xC4000003;
    register long x1 asm("x1") = static_cast<long>(mpidr);
    register long x2 asm("x2") = static_cast<long>(entry_pa);
    register long x3 asm("x3") = static_cast<long>(context);
    asm volatile("hvc #0"
                 : "+r"(x0)
                 : "r"(x1), "r"(x2), "r"(x3)
                 : "memory");
    return x0;
}
#elif defined(__riscv)
long psci_cpu_on_impl(uint64_t, uint64_t, uint64_t) { return 0; }
#else
long psci_cpu_on_impl(uint64_t, uint64_t, uint64_t) { return -1; }
#endif

} // namespace

void kernel::register_boot_hooks(const kernel::BootHooks* hooks) {
    if (hooks) {
        g_boot_hooks          = hooks;
        g_multicore_supported = hooks->supports_multicore ? hooks->supports_multicore() : false;
    }
}

bool kernel::has_multicore_support() {
    return g_multicore_supported;
}

extern "C" void kernel_secondary_main(uint32_t core_id) {
    if (!kernel::g_platform || !kernel::g_scheduler_ptr) {
        for (;;) asm volatile("wfi");
    }
    {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[miniOS arm64] core %u scheduler handoff\n",
                                 core_id);
        early_uart_puts(buf);
    }
    kernel::g_platform->early_init_core(core_id);
#if defined(__aarch64__)
    g_arm64_secondary_ready_mask.fetch_or(1u << core_id, std::memory_order_release);
#endif
    {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[miniOS arm64] core %u entering scheduler\n",
                                 core_id);
        early_uart_puts(buf);
    }
    kernel::g_scheduler_ptr->start_core_scheduler(core_id);
}

extern "C" void kernel_main() {
    if (!initialize_platform_and_scheduler()) {
        for (;;);
        return;
    }
    if (!create_idle_threads()) {
        for (;;) asm volatile("wfi");
    }

    capture_boot_start_time();
    log_timestamped("smp...");
    release_secondary_cores();
#if defined(__aarch64__)
    wait_for_arm64_secondary_ready();
#endif

    initialize_core0_boot_services();
    wire_motion_axes_from_device_db();
    create_runtime_threads();

    kernel::g_scheduler_ptr->start_core_scheduler(0);
}

// ----------------------------------------------------------------------------
// Shared boot helpers — callable from arm64 kernel_main() and rv64
// kernel_main_rv64(). Everything that would otherwise drift between the two
// arches lives here. Arch-specific bring-up (PSCI vs spin-table, scheduler
// entry) stays in the per-arch kernel_main entry points.
// ----------------------------------------------------------------------------
namespace kernel {
namespace boot {

void init_vfs() {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (uart) uart->puts("[vfs] register embedded defaults\n");
    kernel::vfs::register_embedded_defaults();
    if (uart) uart->puts("[vfs] probing block device\n");
    const bool blk_ok = kernel::g_platform->init_block_device();
    if (uart) uart->puts(blk_ok ? "[vfs] block device ready\n" : "[vfs] no block device\n");
    (void)kernel::vfs::mount_sd();
}

void load_runtime_tsvs() {
    // UI template lives in VFS so either the embedded default or an SD-card
    // override can provide it.
    {
        const char* ui_data = nullptr; size_t ui_len = 0;
        if (kernel::vfs::lookup("system/ui/embedded_ui.tsv", ui_data, ui_len) && ui_len > 0) {
            ui_builder::load_tsv(ui_data, ui_len);
        }
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
    {
        const char* tp_data = nullptr; size_t tp_len = 0;
        if (kernel::vfs::lookup("system/machine/embedded_toolpods.tsv", tp_data, tp_len) && tp_len > 0) {
            (void)machine::toolpods::g_service.load_tsv(tp_data, tp_len);
        }
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
}

void log_and_route_net_roles() {
    machine::placement::Config placement{};
    machine::placement::g_service.snapshot(placement);
    const uint32_t num_cores = kernel::g_platform->get_num_cores();
    const uint32_t num_nets = static_cast<uint32_t>(kernel::g_platform->get_num_nets());
    if (num_nets == 0) return;

    const uint8_t hmi_nic = placement.hmi_nic;
    const uint8_t hmi_core = machine::placement::g_service.sanitize_core(placement.hmi_irq_core, num_cores);
    log_net_role("hmi", hmi_nic, hmi_core, num_nets);
    if (hmi_nic < num_nets) {
        kernel::g_platform->route_net_irq(hmi_nic, 1u << hmi_core);
    }
    const uint8_t ec_a_nic = placement.ec_a_nic;
    log_net_role("ec_a", ec_a_nic,
                 machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores),
                 num_nets);
    if (ec_a_nic < num_nets) {
        ethercat::g_master_a.set_nic_index(static_cast<int>(ec_a_nic));
        kernel::g_platform->route_net_irq(ec_a_nic, 1u << machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores));
    }
#if MINIOS_FAKE_SLAVE
    const uint8_t fake_slave_nic = placement.fake_slave_nic;
    log_net_role("fake_slave", fake_slave_nic,
                 machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores),
                 num_nets);
    if (fake_slave_nic < num_nets) {
        ethercat::g_fake_slave.set_nic_idx(static_cast<int>(fake_slave_nic));
        kernel::g_platform->route_net_irq(fake_slave_nic, 1u << machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores));
    }
#else
    const uint8_t ec_b_nic = placement.ec_b_nic;
    log_net_role("ec_b", ec_b_nic,
                 machine::placement::g_service.sanitize_core(placement.ec_b_core, num_cores),
                 num_nets);
    if (ec_b_nic < num_nets) {
        ethercat::g_master_b.set_nic_index(static_cast<int>(ec_b_nic));
        kernel::g_platform->route_net_irq(ec_b_nic, 1u << machine::placement::g_service.sanitize_core(placement.ec_b_core, num_cores));
    }
#endif
}

void create_boot_services(CreateThreadFn create_thread) {
    machine::placement::Config placement{};
    machine::placement::g_service.snapshot(placement);
    const uint32_t num_cores = kernel::g_platform->get_num_cores();
    const int cli_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.cli_core, num_cores));
    const int uart_io_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.uart_io_core, num_cores));
    const int ui_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.ui_core, num_cores));

    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        (void)create_thread(&cli::CLI::thread_entry, nullptr, 3, cli_core, "cli", false, 0);
        (void)create_thread(&cli::io::uart_io_entry, uart, 3, uart_io_core, "uart_io", false, 0);
    }

    log_timestamped("cli...");
    // UI must be EDF-eligible with a deadline comparable to the housekeepers
    // (HMI/g-code/macro/ladder/probe all 1000us) so the yield-aware scheduler
    // rotates through UI as well. A much larger UI deadline (e.g. 16000us)
    // made UI lose every EDF comparison and starved it despite priority 10.
    // Actual UI refresh rate is enforced inside run_ui_main_loop via
    // wait_until_ns, not by the scheduler deadline.
    if (!create_thread(&kernel::ui::boot_ui_thread_entry, nullptr, 10, ui_core, "ui", false, 1000)) {
        log_timestamped("ui: FAILED");
    } else {
        log_timestamped("ui: ok");
    }
}

void create_runtime_services(CreateThreadFn create_thread) {
    machine::placement::Config placement{};
    machine::placement::g_service.snapshot(placement);
    const uint32_t num_cores = kernel::g_platform->get_num_cores();
    const uint32_t num_nets = static_cast<uint32_t>(kernel::g_platform->get_num_nets());
    const int motion_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.motion_core, num_cores));
    const int gcode_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.gcode_core, num_cores));
    const int macro_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.macro_core, num_cores));
    const int ladder_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.ladder_core, num_cores));
    const int probe_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.probe_core, num_cores));
    const int bus_config_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.bus_config_core, num_cores));
    const int ec_a_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.ec_a_core, num_cores));
    const int fake_slave_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.fake_slave_core, num_cores));
    const int hmi_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.hmi_irq_core, num_cores));
    const uint8_t hmi_nic = placement.hmi_nic;
    const uint8_t ec_a_nic = placement.ec_a_nic;
    const uint8_t fake_slave_nic = placement.fake_slave_nic;
#if !MINIOS_FAKE_SLAVE
    const uint8_t ec_b_nic = placement.ec_b_nic;
#endif

    (void)create_thread(&motion::Kernel::thread_entry, nullptr, 15, motion_core, "motion", false, 250);
    (void)create_thread(&cnc::interp::Runtime::thread_entry, nullptr, 6, gcode_core, "gcode", false, 1000);
    (void)create_thread(&macros::Runtime::thread_entry, nullptr, 6, macro_core, "macro", false, 1000);
    (void)create_thread(&ladder::Runtime::thread_entry, nullptr, 7, ladder_core, "ladder", false, 1000);
    (void)create_thread(&probe::Runtime::thread_entry, nullptr, 6, probe_core, "probe", false, 1000);
    if (num_nets > hmi_nic) {
        hmi::g_service.configure(hmi_nic);
        log_timestamped("hmi: eth0 service...");
        (void)create_thread(&hmi::Service::thread_entry, &hmi::g_service, 8, hmi_core, "hmi", false, 1000);
    }
    if (num_nets > ec_a_nic) {
        ethercat::g_master_a.set_nic_index(static_cast<int>(ec_a_nic));
        (void)create_thread(&ethercat::Master::thread_entry, &ethercat::g_master_a, 15, ec_a_core, "ec_a", false, 250);
        (void)create_thread(&ethercat::bus_config_entry, &ethercat::g_master_a, 3, bus_config_core, "bus_cfg_a", false, 0);
    }
    if (num_nets > (
#if MINIOS_FAKE_SLAVE
        fake_slave_nic
#else
        ec_b_nic
#endif
        )) {
#if MINIOS_FAKE_SLAVE
        ethercat::g_fake_slave.set_nic_idx(static_cast<int>(fake_slave_nic));
        (void)create_thread(&ethercat::FakeSlave::thread_entry, &ethercat::g_fake_slave, 15, fake_slave_core, "fake_slave", false, 125);
#else
        const int ec_b_core = static_cast<int>(machine::placement::g_service.sanitize_core(placement.ec_b_core, num_cores));
        ethercat::g_master_b.set_nic_index(static_cast<int>(ec_b_nic));
        (void)create_thread(&ethercat::Master::thread_entry, &ethercat::g_master_b, 15, ec_b_core, "ec_b", false, 250);
        (void)create_thread(&ethercat::bus_config_entry, &ethercat::g_master_b, 3, bus_config_core, "bus_cfg_b", false, 0);
#endif
    }
}

} // namespace boot
} // namespace kernel
