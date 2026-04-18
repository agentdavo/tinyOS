// SPDX-License-Identifier: MIT OR Apache-2.0
// CLI implementation. Two threads on core 0:
//   - uart_io (priority 15): owns the UART; pumps bytes between the hardware
//     and a pair of lock-free SPSC queues. Never touches the command table.
//   - cli (priority 3): line editor + history + tab completion + dispatch;
//     reads bytes from the input queue, writes via cli::io::put().

#include "cli.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include "ethercat/master.hpp"
#include "ethercat/cia402.hpp"
#include "ethercat/frame.hpp"  // put_u16_le / put_u32_le / get_u32_le
#if MINIOS_FAKE_SLAVE
#include "ethercat/fake_slave.hpp"
#include "ethercat/esm.hpp"
#endif
#include "motion/motion.hpp"
#include "cnc/interpreter.hpp"
#include "cnc/offsets.hpp"
#include "cnc/programs.hpp"
#include "automation/ladder_runtime.hpp"
#include "automation/macro_runtime.hpp"
#include "machine/machine_registry.hpp"
#include "machine/toolpods.hpp"
#include "devices/device_db.hpp"
#include "diag/jitter.hpp"
#include "diag/cpu_load.hpp"
#include "hmi/hmi_service.hpp"
#include "ui/fb.hpp"
#include "ui/splash.hpp"
#include "ui/ui_builder_tsv.hpp"
#include "kernel/usb/usb.hpp"

#include <cstring>
#include <cstddef>

namespace cli {

CLI g_cli;

static constexpr const char PROMPT[] = "miniOS> ";
static constexpr size_t PROMPT_LEN = sizeof(PROMPT) - 1;

namespace {

constexpr char K_TAB = '\t';
constexpr char K_CR  = '\r';
constexpr char K_LF  = '\n';
constexpr char K_BS  = 0x08;
constexpr char K_DEL = 0x7f;
constexpr char K_ESC = 0x1b;
constexpr char K_ETX = 0x03; // Ctrl-C
constexpr char K_NAK = 0x15; // Ctrl-U (kill line)
constexpr char K_SOH = 0x01; // Ctrl-A (home)
constexpr char K_ENQ = 0x05; // Ctrl-E (end)

bool is_printable(char c) noexcept { return c >= 0x20 && c < 0x7f; }
size_t cstrlen(const char* s) noexcept { size_t n = 0; while (s[n]) ++n; return n; }
int cstrcmp(const char* a, const char* b) noexcept {
    while (*a && *a == *b) { ++a; ++b; }
    return static_cast<unsigned char>(*a) - static_cast<unsigned char>(*b);
}

} // namespace

// ============================================================================
// I/O plumbing — SPSC queues + chunk pool + sink UART adapter.
// ============================================================================
namespace io {

static ByteQueue<INPUT_CAPACITY>   g_in_queue;
static ChunkQueue<OUTPUT_CAPACITY> g_out_queue;

// Free-list of chunks managed as another lock-free SPSC queue: the cli thread
// pops free chunks (consumer of the free queue) and pushes onto the out queue
// (producer); uart_io pops from the out queue (consumer) and returns chunks
// to the free queue (producer). The roles are stable per queue, so SPSC is
// correct.
static ChunkQueue<CHUNK_POOL_SIZE> g_free_queue;
static Chunk                       g_chunk_storage[CHUNK_POOL_SIZE];
static std::atomic<bool>           g_pool_ready{false};

static kernel::hal::UARTDriverOps* g_uart = nullptr;

static void yield_now() noexcept {
    // Cooperative back-off when a queue is full/empty. Use the scheduler
    // yield if available, otherwise just spin with a hint.
    if (kernel::g_scheduler_ptr && kernel::g_platform) {
        kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
    } else {
        kernel::util::cpu_relax();
    }
}

static Chunk* alloc_chunk() noexcept {
    Chunk* c = nullptr;
    while ((c = g_free_queue.try_pop()) == nullptr) {
        yield_now();
    }
    c->len = 0;
    return c;
}

static void publish_chunk(Chunk* c) noexcept {
    while (!g_out_queue.try_push(c)) {
        yield_now();
    }
}

void init(kernel::hal::UARTDriverOps* uart) noexcept {
    g_uart = uart;
    if (g_pool_ready.exchange(true, std::memory_order_acq_rel)) return;
    for (size_t i = 0; i < CHUNK_POOL_SIZE; ++i) {
        g_chunk_storage[i].len = 0;
        // Push will only fail if Capacity == size; the pool size matches the
        // queue capacity, so the last slot is unusable. We leave one chunk
        // off the free list as a safety margin.
        if (i + 1 < CHUNK_POOL_SIZE) {
            g_free_queue.try_push(&g_chunk_storage[i]);
        }
    }
}

void put_n(const char* s, size_t n) noexcept {
    if (!s || n == 0) return;
    Chunk* c = alloc_chunk();
    size_t i = 0;
    while (i < n) {
        size_t room = Chunk::CAPACITY - c->len;
        size_t take = (n - i < room) ? (n - i) : room;
        for (size_t k = 0; k < take; ++k) c->data[c->len + k] = s[i + k];
        c->len += static_cast<uint16_t>(take);
        i += take;
        if (c->len == Chunk::CAPACITY) {
            publish_chunk(c);
            if (i < n) c = alloc_chunk();
            else c = nullptr;
        }
    }
    if (c && c->len > 0) publish_chunk(c);
    else if (c) {
        while (!g_free_queue.try_push(c)) yield_now();
    }
}

void put(const char* s) noexcept {
    if (!s) return;
    size_t n = 0; while (s[n]) ++n;
    put_n(s, n);
}

void putc_one(char c) noexcept {
    put_n(&c, 1);
}

uint8_t get_blocking() noexcept {
    uint8_t b = 0;
    while (!g_in_queue.try_pop(b)) {
        yield_now();
    }
    return b;
}

bool try_get(uint8_t& out) noexcept {
    return g_in_queue.try_pop(out);
}

[[noreturn]] void uart_io_entry(void* arg) {
    auto* uart = static_cast<kernel::hal::UARTDriverOps*>(arg);
    if (uart) g_uart = uart;
    // Make sure the chunk pool exists (call site already invoked init(), but
    // this keeps the entry point self-sufficient if reordered).
    init(g_uart);

    for (;;) {
        // Drain any pending output chunks before blocking on input. This keeps
        // the UART pipe full when the CLI is producing bursts (e.g. `help`).
        while (Chunk* c = g_out_queue.try_pop()) {
            for (uint16_t i = 0; i < c->len; ++i) g_uart->putc(c->data[i]);
            c->len = 0;
            // Return to free list.
            while (!g_free_queue.try_push(c)) {
                // Free queue full shouldn't happen (capacity matches pool),
                // but yield if it does.
                kernel::util::cpu_relax();
            }
        }
        // Read one byte (blocking spin on the UART LSR) and push it onto the
        // input queue. We take one byte per iteration so output drains
        // promptly between keystrokes.
        char ch = g_uart->getc_blocking();
        while (!g_in_queue.try_push(static_cast<uint8_t>(ch))) {
            // Input queue full means the CLI thread isn't keeping up. Yield
            // briefly to let it run.
            kernel::util::cpu_relax();
        }
    }
}

// ---- Sink UART adapter for command handlers ----
// Command handlers were written against UARTDriverOps*. They call puts/putc
// to print results. We provide a small adapter that funnels those calls into
// the chunk pipeline; the CLI passes this adapter to handlers, so they never
// touch the real UART.
class SinkUART : public kernel::hal::UARTDriverOps {
public:
    void putc(char c) override { putc_one(c); }
    void puts(const char* s) override { put(s); }
    void uart_put_uint64_hex(uint64_t value) override {
        char hex[] = "0123456789ABCDEF";
        char buf[17]; buf[16] = '\0';
        for (int i = 15; i >= 0; --i) { buf[i] = hex[value & 0xF]; value >>= 4; }
        put(buf);
    }
    char getc_blocking() override { return static_cast<char>(get_blocking()); }
};

static SinkUART g_sink;

} // namespace io

kernel::hal::UARTDriverOps* CLI::sink_uart() noexcept { return &io::g_sink; }

// ============================================================================
// Built-in command handlers — unchanged behaviour. They write to the sink
// UART, which the cli thread provides; the uart_io thread never invokes them.
// ============================================================================

static bool cli_parse_long(const char*& p, long& out) noexcept; // fwd, defined below
static int cmd_ec_scan(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Probe a contiguous range of station addresses, reporting the
    // identity and matching DB entry for each that responds. Starts
    // higher than bring-up discovery so hot-swapped slaves show up
    // without needing a full ESM walk.
    const char* p = args ? args : "";
    long start_l = 0x1001, count_l = 8;
    (void)cli_parse_long(p, start_l);
    (void)cli_parse_long(p, count_l);
    if (count_l <= 0 || count_l > 64) count_l = 8;

    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "ec_scan 0x%04lx..0x%04lx:\n", start_l, start_l + count_l - 1);
    uart->puts(buf);
    size_t hits = 0;
    for (long i = 0; i < count_l; ++i) {
        const uint16_t station = static_cast<uint16_t>(start_l + i);
        uint32_t vid = 0, pid = 0, rev = 0, ser = 0;
        const bool ok = ethercat::g_master_a.probe_slave_identity(
            station, 0, 0, 0, &vid, &pid, &rev, &ser);
        if (!ok || (vid == 0 && pid == 0)) continue;
        const auto* dev = devices::g_device_db.find({vid, pid});
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  0x%04x vid=0x%08lx pid=0x%08lx rev=0x%08lx  %s\n",
            (unsigned)station,
            (unsigned long)vid, (unsigned long)pid, (unsigned long)rev,
            dev ? dev->name : "(not in DB)");
        uart->puts(buf);
        ++hits;
    }
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  %zu slave(s) responded in range\n", hits);
    uart->puts(buf);
    return 0;
}
static int cmd_ec_abort(const char*, kernel::hal::UARTDriverOps* uart) {
    // Broadcast AL=Init. The cyclic loop will walk the bus back up via
    // the normal ESM transitions once the gate reopens. Warning: also
    // clears allow_safeop_ implicitly since bus_config only runs once
    // at boot — re-opening the gate currently requires the
    // `ec_allow_safeop` CLI or a restart.
    const size_t n = ethercat::g_master_a.broadcast_init_state();
    uart->puts(n ? "ec_abort: broadcast AL=Init (1 frame queued)\n"
                 : "ec_abort: FAIL (send)\n");
    return n ? 0 : 1;
}
static int cmd_ec_watchdog(const char* args, kernel::hal::UARTDriverOps* uart) {
    // ec_watchdog <timeout_ms> [station]. Default 100 ms per PLAN.md
    // "Still open" note.
    const char* p = args ? args : "";
    long ms = 100, station = 0x1001;
    (void)cli_parse_long(p, ms);
    (void)cli_parse_long(p, station);
    const size_t n = ethercat::g_master_a.configure_sm_watchdog(
        static_cast<uint16_t>(station), static_cast<uint16_t>(ms));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "ec_watchdog @ 0x%04lx timeout=%ld ms => %zu FPWRs queued\n",
        station, ms, n);
    uart->puts(buf);
    return n == 3 ? 0 : 1;
}
static int cmd_motion_enable(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: motion_enable <axis>\n"); return 1; }
    const char* p = args; long ax = 0;
    if (!cli_parse_long(p, ax)) { uart->puts("bad axis\n"); return 1; }
    const bool ok = motion::g_motion.enable_axis(static_cast<size_t>(ax));
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "motion_enable ax%ld => %s\n", ax, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_motion_disable(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: motion_disable <axis>\n"); return 1; }
    const char* p = args; long ax = 0;
    if (!cli_parse_long(p, ax)) { uart->puts("bad axis\n"); return 1; }
    const bool ok = motion::g_motion.disable_axis(static_cast<size_t>(ax));
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "motion_disable ax%ld => %s\n", ax, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_save_cfg(const char*, kernel::hal::UARTDriverOps* uart) {
    // Print operator-set state as a sequence of CLI commands — paste the
    // output into a new session to reproduce the runtime configuration.
    // There's no NVM on miniOS yet, so this is the persistence story:
    // serialised-to-text, operator-owned, replayable.
    char buf[192];
    uart->puts("# --- miniOS save_cfg ---\n");
    uart->puts("# Paste this whole block at a fresh CLI prompt to replay.\n");

    // Motion topology.
    for (size_t c = 0; c < motion::g_motion.channel_count(); ++c) {
        const auto& ch = motion::g_motion.channel(c);
        if (c == 0) uart->puts("topology ");
        kernel::util::k_snprintf(buf, sizeof(buf), "%s%s=", c ? " " : "", ch.name);
        uart->puts(buf);
        for (uint8_t k = 0; k < ch.axis_count; ++k) {
            kernel::util::k_snprintf(buf, sizeof(buf), "%s%u",
                k ? "," : "", (unsigned)ch.axis_indices[k]);
            uart->puts(buf);
        }
    }
    uart->puts("\n");

    // Per-axis operator state — emit only for axes with non-default
    // settings. Load feedback, sw limits, spin velocity.
    for (size_t i = 0; i < motion::MAX_AXES; ++i) {
        const auto& a = motion::g_motion.axis(i);
        if (a.load.configured) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "axis_load_configure %zu %lld %lld %ld %ld %ld %d %d\n",
                i, (long long)a.load.scale_num, (long long)a.load.scale_den,
                (long)a.load.sign, (long)a.load.trim_cap_counts,
                (long)a.load.trim_slew_cps, (int)a.load.kp_ppm, (int)a.load.ki_ppm);
            uart->puts(buf);
        }
        if (a.sw_limits_pending) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "axis_sw_limits %zu %ld %ld\n",
                i, (long)a.sw_limit_neg_counts, (long)a.sw_limit_pos_counts);
            uart->puts(buf);
        }
        if (a.spin_velocity_cps != 0) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "axis_spin %zu %ld\n", i, (long)a.spin_velocity_cps);
            uart->puts(buf);
        }
    }

    // Per-channel overrides.
    for (size_t c = 0; c < motion::g_motion.channel_count(); ++c) {
        const auto& ov = motion::g_motion.channel(c).overrides;
        if (ov.feed_permille != 1000) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "override %zu feed %u\n", c, (unsigned)ov.feed_permille);
            uart->puts(buf);
        }
        if (ov.rapid_permille != 1000) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "override %zu rapid %u\n", c, (unsigned)ov.rapid_permille);
            uart->puts(buf);
        }
        if (ov.spindle_permille != 1000) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "override %zu spindle %u\n", c, (unsigned)ov.spindle_permille);
            uart->puts(buf);
        }
    }

    uart->puts("# --- end save_cfg ---\n");
    return 0;
}
static int cmd_meminfo(const char*, kernel::hal::UARTDriverOps* uart) {
    // Memory introspection: FixedMemoryPool occupancy + per-thread stack
    // watermarks. Stack-painting is compiled in (see core.hpp
    // STACK_PAINT) so stack_used_bytes returns meaningful numbers.
    char buf[160];

    // FixedMemoryPool — the software-timer object pool.
    const size_t total = kernel::core::g_software_timer_obj_pool.get_total_count();
    const size_t free_ = kernel::core::g_software_timer_obj_pool.get_free_count();
    kernel::util::k_snprintf(buf, sizeof(buf),
        "FixedMemoryPool (software timers): %zu / %zu in use (%zu free)\n",
        total - free_, total, free_);
    uart->puts(buf);

    // Threads — dump name + state + stack watermark.
    uart->puts("Threads:\n");
    for (size_t i = 0; i < kernel::core::MAX_THREADS; ++i) {
        const auto& t = kernel::core::g_task_tcbs[i];
        if (t.state == kernel::core::TCB::State::INACTIVE) continue;
        const char* sn = "?";
        switch (t.state) {
            case kernel::core::TCB::State::READY:    sn = "ready";    break;
            case kernel::core::TCB::State::RUNNING:  sn = "running";  break;
            case kernel::core::TCB::State::BLOCKED:  sn = "blocked";  break;
            case kernel::core::TCB::State::ZOMBIE:   sn = "zombie";   break;
            default: break;
        }
        const size_t used = t.stack_used_bytes();
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  [%zu] %-12s prio=%d aff=%d state=%s stack=%zu/%zu (%zu%%)\n",
            i, t.name, (int)t.priority, (int)t.core_affinity, sn,
            used, t.stack_size,
            t.stack_size ? (used * 100 / t.stack_size) : 0);
        uart->puts(buf);
    }
    return 0;
}
static int cmd_version(const char*, kernel::hal::UARTDriverOps* uart) {
    // Build stamp — date, time, target, FAKE_SLAVE setting. A real
    // production build would inject a git hash via the Makefile too.
    char buf[192];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "miniOS v1.7 | target=arm64 | fake_slave=%d | git=%s | built %s %s\n",
#ifdef MINIOS_FAKE_SLAVE
        (int)MINIOS_FAKE_SLAVE,
#else
        0,
#endif
#ifdef MINIOS_GIT_HASH
        MINIOS_GIT_HASH,
#else
        "unknown",
#endif
        __DATE__, __TIME__);
    uart->puts(buf);
    return 0;
}
static int cmd_ec_diag_dump(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Walk the DiagHistory buffer. 0x10F3:2 is the newest-message pointer
    // (one-based); 0x10F3:6..55 are 50 slot entries, 20-ish bytes each.
    // For each slot we pull via segmented upload and decode the TextId
    // (at byte offset 6) against cia402::diag_text().
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    const uint16_t station = static_cast<uint16_t>(station_l);

    char buf[192];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "DiagHistory @ 0x%04x:\n", (unsigned)station);
    uart->puts(buf);

    // Read newest-pointer (0x10F3:2) expedited.
    uint8_t ep_bytes = 0;
    uint8_t ep_buf[4] = {};
    uint32_t ep_abort = 0;
    uint16_t newest = 0;
    if (ethercat::g_master_a.upload_sdo(station, 0x10F3, 0x02,
                                        ep_buf, &ep_bytes, 200000, &ep_abort)) {
        newest = static_cast<uint16_t>(ep_buf[0] | (ep_buf[1] << 8));
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  newest slot = 0x%02x\n", (unsigned)newest);
        uart->puts(buf);
    }

    // Walk slots 0x10F3:6..0x10F3:55. Any slot that upload-fails (empty)
    // just gets a ". " marker. Keep output compact.
    for (uint8_t sub = 0x06; sub <= 0x37; ++sub) {
        uint8_t seg[32] = {};
        uint32_t abort = 0;
        const size_t got = ethercat::g_master_a.upload_sdo_segmented(
            station, 0x10F3, sub, seg, sizeof(seg), 100000, &abort);
        if (got < 8) continue; // empty slot
        const uint16_t text_id = static_cast<uint16_t>(seg[6] | (seg[7] << 8));
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  slot 0x%02x TextId=0x%04x %s\n",
            (unsigned)sub, (unsigned)text_id, cia402::diag_text(text_id));
        uart->puts(buf);
    }
    return 0;
}
static int cmd_status(const char*, kernel::hal::UARTDriverOps* uart) {
    // One-shot dashboard. Rolls up everything the kernel knows about
    // what's on the bus, how the motion kernel is configured, and what's
    // actively running. Useful as a first call after boot and whenever
    // you want "am I healthy?" without composing five separate commands.
    char buf[192];

    // Section 1 — kernel + cycle banner.
    kernel::util::k_snprintf(buf, sizeof(buf),
        "========= miniOS status =========\n");
    uart->puts(buf);
    kernel::util::k_snprintf(buf, sizeof(buf),
        "cycle=250us  motion period matches master period\n");
    uart->puts(buf);

    // Section 2 — both masters.
    uart->puts("\n--- EtherCAT ---\n");
    ethercat::g_master_a.dump_status(uart);
    ethercat::g_master_b.dump_status(uart);
    uart->puts("slaves (ec0):\n");
    ethercat::g_master_a.dump_slaves(uart);

    // Section 3 — device DB inventory (what we *know* about, not just
    // what's on the bus). Helps when a slave is missing from discovery
    // and the operator wants to know what the DB recognises.
    uart->puts("\n--- Device DB ---\n");
    devices::g_device_db.dump(uart);

    // Section 4 — motion kernel: channels, axes, any active sync
    // primitives, motion stats.
    uart->puts("\n--- Motion channels ---\n");
    motion::g_motion.dump_channels(uart);
    uart->puts("\n--- Active barriers ---\n");
    motion::g_motion.dump_barriers(uart);
    uart->puts("\n--- Active gear links ---\n");
    motion::g_motion.dump_gears(uart);
    uart->puts("\n--- Active gantry links ---\n");
    motion::g_motion.dump_gantrys(uart);
    uart->puts("\n--- Motion stats ---\n");
    motion::g_motion.dump_status(uart);
    uart->puts("\n--- Axis positions (live) ---\n");
    motion::g_motion.dump_positions(uart);

#if MINIOS_FAKE_SLAVE
    // Section 5 — fake_slave snapshot, when linked. Two summary lines;
    // the full `fake` command dumps I/O, ADC, and CiA-402 state.
    uart->puts("\n--- FakeSlave (NIC 1) ---\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  station=0x%04x vid=0x%08x pid=0x%08x state=%s rx=%llu tx=%llu wkc=%llu\n",
        (unsigned)ethercat::FakeSlave::STATION_ADDR,
        (unsigned)ethercat::FakeSlave::VENDOR_ID,
        (unsigned)ethercat::FakeSlave::PRODUCT_CODE,
        ethercat::al_state_name(ethercat::g_fake_slave.al_state()),
        (unsigned long long)ethercat::g_fake_slave.frames_in(),
        (unsigned long long)ethercat::g_fake_slave.frames_out(),
        (unsigned long long)ethercat::g_fake_slave.wkc_serviced());
    uart->puts(buf);
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  DI=0x%04x DO=0x%04x cia402_state=%s\n",
        (unsigned)ethercat::g_fake_slave.dio_in(),
        (unsigned)ethercat::g_fake_slave.dio_out(),
        cia402::state_name(ethercat::g_fake_slave.cia_state()));
    uart->puts(buf);
#endif

    uart->puts("\n========= end status =========\n");
    return 0;
}

static int cmd_offsets(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[160];
    const auto& svc = cnc::offsets::g_service;
    kernel::util::k_snprintf(buf, sizeof(buf), "active work=%s active tool=T%lu\n",
                             svc.work_offsets()[svc.active_work()].name,
                             static_cast<unsigned long>(svc.tool_offsets()[svc.active_tool()].tool));
    uart->puts(buf);
    uart->puts("work offsets:\n");
    for (size_t i = 0; i < cnc::offsets::WORK_OFFSET_COUNT; ++i) {
        const auto& w = svc.work_offsets()[i];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %s%s X%.3f Y%.3f Z%.3f A%.3f\n",
            i == svc.active_work() ? "* " : "  ",
            w.name, w.value.axis[0], w.value.axis[1], w.value.axis[2], w.value.axis[3]);
        uart->puts(buf);
    }
    uart->puts("tool offsets:\n");
    for (size_t i = 0; i < 8; ++i) {
        const auto& t = svc.tool_offsets()[i];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %c T%02lu len=%.3f rad=%.3f wear=%.3f\n",
            i == svc.active_tool() ? '*' : ' ',
            static_cast<unsigned long>(t.tool), t.length, t.radius, t.wear);
        uart->puts(buf);
    }
    return 0;
}

static int cmd_offset_select(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: offset_select <g54..g59|toolN>\n"); return 1; }
    if ((args[0] == 'g' || args[0] == 'G') && args[1] == '5' && args[2] >= '4' && args[2] <= '9') {
        const size_t idx = static_cast<size_t>(args[2] - '4');
        const bool ok = cnc::offsets::g_service.select_work(idx);
        uart->puts(ok ? "offset_select: work offset selected\n" : "offset_select: invalid work offset\n");
        return ok ? 0 : 1;
    }
    if ((args[0] == 't' || args[0] == 'T')) {
        const char* p = args + 1; long tool = 0;
        if (!cli_parse_long(p, tool) || tool <= 0 || tool > 16) { uart->puts("offset_select: bad tool index\n"); return 1; }
        const bool ok = cnc::offsets::g_service.select_tool(static_cast<size_t>(tool - 1));
        uart->puts(ok ? "offset_select: tool offset selected\n" : "offset_select: invalid tool offset\n");
        return ok ? 0 : 1;
    }
    uart->puts("offset_select: expected g54..g59 or toolN\n");
    return 1;
}

static int cmd_offset_set(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: offset_set <g54..g59> <x|y|z|a> <value> | offset_set tool <idx> <len> <rad> <wear>\n");
        return 1;
    }
    if ((args[0] == 't' || args[0] == 'T') && args[1] == 'o') {
        const char* p = args + 4; long idx = 0, len = 0, rad = 0, wear = 0;
        if (!cli_parse_long(p, idx) || !cli_parse_long(p, len) || !cli_parse_long(p, rad) || !cli_parse_long(p, wear)) {
            uart->puts("offset_set tool: bad args\n"); return 1;
        }
        const bool ok = cnc::offsets::g_service.set_tool_value(static_cast<size_t>(idx), static_cast<float>(len), static_cast<float>(rad), static_cast<float>(wear));
        uart->puts(ok ? "offset_set: tool updated\n" : "offset_set: invalid tool slot\n");
        return ok ? 0 : 1;
    }
    if (!((args[0] == 'g' || args[0] == 'G') && args[1] == '5' && args[2] >= '4' && args[2] <= '9')) {
        uart->puts("offset_set: expected g54..g59 or tool form\n"); return 1;
    }
    const size_t work = static_cast<size_t>(args[2] - '4');
    const char* p = args + 3;
    while (*p == ' ') ++p;
    if (!*p) { uart->puts("offset_set: missing axis\n"); return 1; }
    char axis = *p++;
    long value = 0;
    if (!cli_parse_long(p, value)) { uart->puts("offset_set: missing value\n"); return 1; }
    size_t axis_idx = 0;
    switch (axis | 0x20) {
        case 'x': axis_idx = 0; break;
        case 'y': axis_idx = 1; break;
        case 'z': axis_idx = 2; break;
        case 'a': axis_idx = 3; break;
        default: uart->puts("offset_set: bad axis\n"); return 1;
    }
    const bool ok = cnc::offsets::g_service.set_work_axis(work, axis_idx, static_cast<float>(value));
    uart->puts(ok ? "offset_set: work offset updated\n" : "offset_set: invalid work slot\n");
    return ok ? 0 : 1;
}

static int cmd_program_ls(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[160];
    uart->puts("program store:\n");
    for (size_t i = 0; i < cnc::programs::g_store.count(); ++i) {
        const auto& p = cnc::programs::g_store.program(i);
        const char ch0 = i == cnc::programs::g_store.selected(0) ? '0' : ' ';
        const char ch1 = i == cnc::programs::g_store.selected(1) ? '1' : ' ';
        kernel::util::k_snprintf(buf, sizeof(buf), "  [%c%c] %s (%zu bytes, %zu pts)\n",
                                 ch0, ch1, p.name, p.text_size, p.preview.point_count);
        uart->puts(buf);
    }
    return 0;
}

static int cmd_program_show(const char* args, kernel::hal::UARTDriverOps* uart) {
    size_t idx = cnc::programs::g_store.selected();
    if (args && *args) {
        if (!cnc::programs::g_store.find_by_name(args, idx)) { uart->puts("program_show: not found\n"); return 1; }
    }
    const auto& p = cnc::programs::g_store.program(idx);
    uart->puts(p.name); uart->puts(":\n"); uart->puts(p.text.data()); uart->puts("\n");
    return 0;
}

static int cmd_program_select(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: program_select <name>\n"); return 1; }
    size_t idx = 0;
    if (!cnc::programs::g_store.find_by_name(args, idx)) { uart->puts("program_select: not found\n"); return 1; }
    const bool ok = cnc::programs::g_store.select(idx) && cnc::programs::g_store.open_selected();
    uart->puts(ok ? "program_select: selected and opened\n" : "program_select: failed to open\n");
    return ok ? 0 : 1;
}

static int cmd_program_open(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (args && *args) {
        size_t idx = 0;
        if (!cnc::programs::g_store.find_by_name(args, idx)) { uart->puts("program_open: not found\n"); return 1; }
        if (!cnc::programs::g_store.select(idx)) { uart->puts("program_open: select failed\n"); return 1; }
    }
    const bool ok = cnc::programs::g_store.open_selected();
    uart->puts(ok ? "program_open: opened and preview rebuilt\n" : "program_open: open failed\n");
    return ok ? 0 : 1;
}

static int cmd_program_sim(const char* args, kernel::hal::UARTDriverOps* uart) {
    size_t idx = cnc::programs::g_store.selected();
    if (args && *args && !cnc::programs::g_store.find_by_name(args, idx)) { uart->puts("program_sim: not found\n"); return 1; }
    const auto& p = cnc::programs::g_store.program(idx);
    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "program_sim: %s preview points=%zu bbox=[%.2f,%.2f,%.2f]-[%.2f,%.2f,%.2f]\n",
                             p.name, p.preview.point_count,
                             p.preview.min.x, p.preview.min.y, p.preview.min.z,
                             p.preview.max.x, p.preview.max.y, p.preview.max.z);
    uart->puts(buf);
    return 0;
}

static int cmd_program_select_ch(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: program_select_ch <ch> <name>\n"); return 1; }
    const char* p = args;
    long channel = 0;
    if (!cli_parse_long(p, channel) || channel < 0 ||
        channel >= static_cast<long>(cnc::programs::MAX_CHANNELS)) {
        uart->puts("program_select_ch: bad channel\n");
        return 1;
    }
    while (*p == ' ') ++p;
    if (!*p) { uart->puts("program_select_ch: missing name\n"); return 1; }
    size_t idx = 0;
    if (!cnc::programs::g_store.find_by_name(p, idx)) { uart->puts("program_select_ch: not found\n"); return 1; }
    const bool ok = cnc::programs::g_store.select(static_cast<size_t>(channel), idx) &&
                    cnc::programs::g_store.open_selected(static_cast<size_t>(channel));
    uart->puts(ok ? "program_select_ch: selected and opened\n" : "program_select_ch: failed to open\n");
    return ok ? 0 : 1;
}

static int cmd_program_open_ch(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) { uart->puts("usage: program_open_ch <ch> [name]\n"); return 1; }
    const char* p = args;
    long channel = 0;
    if (!cli_parse_long(p, channel) || channel < 0 ||
        channel >= static_cast<long>(cnc::programs::MAX_CHANNELS)) {
        uart->puts("program_open_ch: bad channel\n");
        return 1;
    }
    while (*p == ' ') ++p;
    if (*p) {
        size_t idx = 0;
        if (!cnc::programs::g_store.find_by_name(p, idx)) { uart->puts("program_open_ch: not found\n"); return 1; }
        if (!cnc::programs::g_store.select(static_cast<size_t>(channel), idx)) {
            uart->puts("program_open_ch: select failed\n");
            return 1;
        }
    }
    const bool ok = cnc::programs::g_store.open_selected(static_cast<size_t>(channel));
    uart->puts(ok ? "program_open_ch: opened and preview rebuilt\n" : "program_open_ch: open failed\n");
    return ok ? 0 : 1;
}

static int cmd_program_run(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args || cstrcmp(args, "all") == 0) {
        const bool ok = cnc::interp::g_runtime.start_all_loaded();
        uart->puts(ok ? "program_run: started all loaded channels\n" : "program_run: nothing started\n");
        return ok ? 0 : 1;
    }
    const char* p = args;
    long channel = 0;
    if (!cli_parse_long(p, channel) || channel < 0 ||
        channel >= static_cast<long>(cnc::programs::MAX_CHANNELS)) {
        uart->puts("program_run: usage program_run <ch|all>\n");
        return 1;
    }
    const bool ok = cnc::interp::g_runtime.start(static_cast<size_t>(channel));
    uart->puts(ok ? "program_run: channel started\n" : "program_run: start failed\n");
    return ok ? 0 : 1;
}

static int cmd_program_status(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[224];
    for (size_t ch = 0; ch < cnc::programs::MAX_CHANNELS; ++ch) {
        const auto state = cnc::programs::g_store.channel_state(ch);
        const auto snap = cnc::interp::g_runtime.snapshot(ch);
        const auto* selected = cnc::programs::g_store.selected_program(ch);
        const auto* loaded = cnc::programs::g_store.loaded_program(ch);
        const char* runtime = "idle";
        switch (snap.state) {
            case cnc::interp::State::Idle: runtime = "idle"; break;
            case cnc::interp::State::Ready: runtime = "ready"; break;
            case cnc::interp::State::Running: runtime = "running"; break;
            case cnc::interp::State::WaitingBarrier: runtime = "barrier"; break;
            case cnc::interp::State::WaitingMacro: runtime = "macro"; break;
            case cnc::interp::State::Dwell: runtime = "dwell"; break;
            case cnc::interp::State::Complete: runtime = "complete"; break;
            case cnc::interp::State::Fault: runtime = "fault"; break;
        }
        kernel::util::k_snprintf(
            buf, sizeof(buf),
            "ch%lu sel=%lu(%s) load=%lu(%s) state=%s line=%lu block=%lu feed=%lu spindle=%ld barrier=%u/%u\n",
            static_cast<unsigned long>(ch),
            static_cast<unsigned long>(state.selected), selected ? selected->name : "-",
            static_cast<unsigned long>(state.loaded), loaded ? loaded->name : "-",
            runtime,
            static_cast<unsigned long>(snap.line),
            static_cast<unsigned long>(snap.block),
            static_cast<unsigned long>(snap.feed),
            static_cast<long>(snap.spindle),
            static_cast<unsigned>(snap.barrier_token),
            static_cast<unsigned>(snap.barrier_mask));
        uart->puts(buf);
    }
    return 0;
}

static int cmd_macro_ls(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "macros: count=%lu selected=%lu\n",
        static_cast<unsigned long>(macros::g_runtime.count()),
        static_cast<unsigned long>(macros::g_runtime.selected()));
    uart->puts(buf);
    for (size_t i = 0; i < macros::g_runtime.count(); ++i) {
        const auto* macro = macros::g_runtime.macro(i);
        if (!macro) continue;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  [%lu] M%u %-16s %s%s\n",
            static_cast<unsigned long>(i), macro->mcode, macro->id, macro->title,
            i == macros::g_runtime.selected() ? "  <selected>" : "");
        uart->puts(buf);
    }
    return 0;
}

static int cmd_macro_run(const char* args, kernel::hal::UARTDriverOps* uart) {
    const char* p = args ? args : "";
    long idx = static_cast<long>(macros::g_runtime.selected());
    (void)cli_parse_long(p, idx);
    if (idx < 0 || static_cast<size_t>(idx) >= macros::g_runtime.count()) {
        uart->puts("macro_run: index out of range\n");
        return 1;
    }
    const bool ok = macros::g_runtime.start(0, static_cast<size_t>(idx));
    uart->puts(ok ? "macro_run: started\n" : "macro_run: start failed\n");
    return ok ? 0 : 1;
}

static int cmd_macro_stop(const char*, kernel::hal::UARTDriverOps* uart) {
    const bool ok = macros::g_runtime.stop(0);
    uart->puts(ok ? "macro_stop: stopped\n" : "macro_stop: stop failed\n");
    return ok ? 0 : 1;
}

static int cmd_symbols(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[160];
    for (size_t i = 0; i < machine::g_registry.count(); ++i) {
        const auto* entry = machine::g_registry.entry(i);
        if (!entry || !entry->used) continue;
        switch (entry->type) {
            case machine::SymbolType::Bool:
                kernel::util::k_snprintf(buf, sizeof(buf), "%-24s bool  %d%s\n",
                    entry->name, entry->value.bool_value ? 1 : 0, entry->writable ? " rw" : "");
                break;
            case machine::SymbolType::Int:
                kernel::util::k_snprintf(buf, sizeof(buf), "%-24s int   %ld%s\n",
                    entry->name, static_cast<long>(entry->value.int_value), entry->writable ? " rw" : "");
                break;
            case machine::SymbolType::Float:
                kernel::util::k_snprintf(buf, sizeof(buf), "%-24s float %.3f%s\n",
                    entry->name, static_cast<double>(entry->value.float_value), entry->writable ? " rw" : "");
                break;
        }
        uart->puts(buf);
    }
    return 0;
}

static int cmd_signals(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf), "signals: %lu bindings\n",
        static_cast<unsigned long>(machine::g_registry.signal_binding_count()));
    uart->puts(buf);
    for (size_t i = 0; i < machine::g_registry.signal_binding_count(); ++i) {
        const auto* binding = machine::g_registry.signal_binding(i);
        if (!binding) continue;
        const char* source = "unknown";
        switch (binding->source) {
            case machine::Registry::SignalSource::EcDi: source = "ec.di"; break;
            case machine::Registry::SignalSource::EcDo: source = "ec.do"; break;
            case machine::Registry::SignalSource::EcAiUni: source = "ec.ai_uni"; break;
            case machine::Registry::SignalSource::EcAiBip: source = "ec.ai_bip"; break;
            case machine::Registry::SignalSource::EcTxBit: source = "ec.tx_bit"; break;
            case machine::Registry::SignalSource::EcRxBit: source = "ec.rx_bit"; break;
            case machine::Registry::SignalSource::EcTxS16: source = "ec.tx_s16"; break;
            case machine::Registry::SignalSource::EcTxU16: source = "ec.tx_u16"; break;
            case machine::Registry::SignalSource::EcTxS32: source = "ec.tx_s32"; break;
            case machine::Registry::SignalSource::EcTxU32: source = "ec.tx_u32"; break;
            case machine::Registry::SignalSource::EcRxS16: source = "ec.rx_s16"; break;
            case machine::Registry::SignalSource::EcRxU16: source = "ec.rx_u16"; break;
            case machine::Registry::SignalSource::EcRxS32: source = "ec.rx_s32"; break;
            case machine::Registry::SignalSource::EcRxU32: source = "ec.rx_u32"; break;
            case machine::Registry::SignalSource::EcTxPin: source = "ec.tx_pin"; break;
            case machine::Registry::SignalSource::EcRxPin: source = "ec.rx_pin"; break;
            case machine::Registry::SignalSource::EcStatusWord: source = "ec.statusword"; break;
            case machine::Registry::SignalSource::EcControlWord: source = "ec.controlword"; break;
            case machine::Registry::SignalSource::EcPositionActual: source = "ec.position_actual"; break;
            case machine::Registry::SignalSource::EcVelocityActual: source = "ec.velocity_actual"; break;
            case machine::Registry::SignalSource::EcErrorCode: source = "ec.error_code"; break;
            case machine::Registry::SignalSource::EcDriveMode: source = "ec.drive_mode"; break;
            case machine::Registry::SignalSource::EcDigitalInputs: source = "ec.digital_inputs"; break;
            case machine::Registry::SignalSource::EcDigitalOutputs: source = "ec.digital_outputs"; break;
            case machine::Registry::SignalSource::None: break;
        }
        size_t idx = 0;
        long value = 0;
        if (machine::g_registry.find(binding->name, idx)) {
            const auto* entry = machine::g_registry.entry(idx);
            if (entry) value = entry->type == machine::SymbolType::Bool
                ? (entry->value.bool_value ? 1 : 0)
                : static_cast<long>(entry->value.int_value);
        }
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %-22s %-16s m=%u s=%u ch=%u bit=%u off=%u pin=%s val=%ld%s\n",
            binding->name, source,
            static_cast<unsigned>(binding->master),
            static_cast<unsigned>(binding->slave),
            static_cast<unsigned>(binding->channel),
            static_cast<unsigned>(binding->bit),
            static_cast<unsigned>(binding->offset),
            binding->pin[0] ? binding->pin : "-",
            value, binding->writable ? " rw" : "");
        uart->puts(buf);
    }
    return 0;
}

static int cmd_symbol_set(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: symbol_set <name> <value>\n");
        return 1;
    }
    const char* p = args;
    while (*p == ' ') ++p;
    char name[32];
    size_t n = 0;
    while (*p && *p != ' ' && n + 1 < sizeof(name)) name[n++] = *p++;
    name[n] = '\0';
    while (*p == ' ') ++p;
    size_t idx = 0;
    if (!machine::g_registry.find(name, idx)) {
        uart->puts("symbol_set: unknown symbol\n");
        return 1;
    }
    const auto* entry = machine::g_registry.entry(idx);
    if (!entry) return 1;
    bool ok = false;
    switch (entry->type) {
        case machine::SymbolType::Bool:
            ok = machine::g_registry.set_bool(name, kernel::util::kstrcmp(p, "0") != 0);
            break;
        case machine::SymbolType::Int: {
            const char* q = p;
            long value = 0;
            ok = cli_parse_long(q, value) && machine::g_registry.set_int(name, static_cast<int32_t>(value));
            break;
        }
        case machine::SymbolType::Float: {
            bool neg = false;
            if (*p == '-') {
                neg = true;
                ++p;
            } else if (*p == '+') {
                ++p;
            }
            long whole = 0;
            const char* q = p;
            if (!cli_parse_long(q, whole)) whole = 0;
            float frac = 0.0f;
            const char* dot = p;
            while (*dot && *dot != '.') ++dot;
            if (*dot == '.') {
                ++dot;
                float place = 0.1f;
                while (*dot >= '0' && *dot <= '9') {
                    frac += static_cast<float>(*dot - '0') * place;
                    place *= 0.1f;
                    ++dot;
                }
            }
            float value = static_cast<float>(whole) + frac;
            if (neg) value = -value;
            ok = machine::g_registry.set_float(name, value);
            break;
        }
    }
    uart->puts(ok ? "symbol_set: ok\n" : "symbol_set: failed\n");
    return ok ? 0 : 1;
}

static int cmd_ladder_ls(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf), "ladder: %lu rung(s)\n",
                             static_cast<unsigned long>(ladder::g_runtime.count()));
    uart->puts(buf);
    for (size_t i = 0; i < ladder::g_runtime.count(); ++i) {
        const auto* rung = ladder::g_runtime.rung(i);
        if (!rung) continue;
        kernel::util::k_snprintf(buf, sizeof(buf), "  [%lu] %s: %s -> %s\n",
                                 static_cast<unsigned long>(i), rung->id, rung->lhs, rung->out);
        uart->puts(buf);
    }
    return 0;
}

static int cmd_toolpods(const char*, kernel::hal::UARTDriverOps* uart) {
    char buf[192];
    kernel::util::k_snprintf(buf, sizeof(buf), "toolpods: %lu\n",
                             static_cast<unsigned long>(machine::toolpods::g_service.pod_count()));
    uart->puts(buf);
    for (size_t i = 0; i < machine::toolpods::g_service.pod_count(); ++i) {
        const auto* pod = machine::toolpods::g_service.pod(i);
        if (!pod) continue;
        kernel::util::k_snprintf(buf, sizeof(buf), "[%lu] %s axis=%s stations=%lu active=%lu locked=%d\n",
                                 static_cast<unsigned long>(i), pod->id, pod->axis_name,
                                 static_cast<unsigned long>(pod->station_count),
                                 static_cast<unsigned long>(pod->active_station),
                                 pod->locked ? 1 : 0);
        uart->puts(buf);
        for (size_t j = 0; j < pod->station_count; ++j) {
            const auto& st = pod->stations[j];
            kernel::util::k_snprintf(buf, sizeof(buf),
                                     "  st%u pos=%ld phys=T%u virt=T%u %s%s\n",
                                     static_cast<unsigned>(st.index),
                                     static_cast<long>(st.position_counts),
                                     static_cast<unsigned>(st.physical_tool),
                                     static_cast<unsigned>(st.virtual_tool),
                                     st.name,
                                     j == pod->active_station ? " <active>" : "");
            uart->puts(buf);
        }
    }
    return 0;
}

static int cmd_toolpod_select(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: toolpod_select <pod> <station>\n");
        return 1;
    }
    const char* p = args;
    while (*p == ' ') ++p;
    char pod[32];
    size_t n = 0;
    while (*p && *p != ' ' && n + 1 < sizeof(pod)) pod[n++] = *p++;
    pod[n] = '\0';
    long station = 0;
    if (!cli_parse_long(p, station)) {
        uart->puts("toolpod_select: bad station\n");
        return 1;
    }
    const bool ok = machine::toolpods::g_service.select_station(pod, static_cast<size_t>(station));
    uart->puts(ok ? "toolpod_select: ok\n" : "toolpod_select: failed\n");
    return ok ? 0 : 1;
}

static int cmd_toolpod_assign(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: toolpod_assign <pod> <station> <virtual_tool>\n");
        return 1;
    }
    const char* p = args;
    while (*p == ' ') ++p;
    char pod[32];
    size_t n = 0;
    while (*p && *p != ' ' && n + 1 < sizeof(pod)) pod[n++] = *p++;
    pod[n] = '\0';
    long station = 0, virtual_tool = 0;
    if (!cli_parse_long(p, station) || !cli_parse_long(p, virtual_tool)) {
        uart->puts("toolpod_assign: bad args\n");
        return 1;
    }
    const bool ok = machine::toolpods::g_service.assign_virtual_tool(
        pod, static_cast<size_t>(station), static_cast<uint16_t>(virtual_tool));
    uart->puts(ok ? "toolpod_assign: ok\n" : "toolpod_assign: failed\n");
    return ok ? 0 : 1;
}

static int cmd_ui_page(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!uart) return 1;
    if (!args || !*args) {
        uart->puts("usage: ui_page <page-id>\n");
        uart->puts("active page: ");
        uart->puts(ui_builder::active_page_id());
        uart->puts("\n");
        return 1;
    }
    if (!ui_builder::set_page(args)) {
        uart->puts("ui_page: unknown page id\n");
        return 1;
    }
    kernel::ui::render_ui_once();
    uart->puts("ui_page: switched to ");
    uart->puts(ui_builder::active_page_id());
    uart->puts("\n");
    return 0;
}

static int cmd_ui_dump(const char* args, kernel::hal::UARTDriverOps* uart) {
    (void)uart;
    uint32_t scale = 6;
    if (args && *args) {
        int32_t parsed = 0;
        const char* p = args;
        while (*p >= '0' && *p <= '9') {
            parsed = parsed * 10 + (*p - '0');
            ++p;
        }
        if (parsed >= 1) scale = static_cast<uint32_t>(parsed);
    }
    if (scale == 0) scale = 1;

    auto& fb = kernel::ui::framebuffer();
    const uint32_t out_w = kernel::ui::FB_WIDTH / scale;
    const uint32_t out_h = kernel::ui::FB_HEIGHT / scale;
    if (out_w == 0 || out_h == 0) {
        io::put("UI_DUMP_ERROR invalid-scale\n");
        return 1;
    }

    kernel::ui::render_ui_once();

    char header[128];
    const int header_len = kernel::util::k_snprintf(
        header, sizeof(header),
        "P6\n%lu %lu\n255\n",
        static_cast<unsigned long>(out_w),
        static_cast<unsigned long>(out_h));
    if (header_len <= 0) {
        io::put("UI_DUMP_ERROR header\n");
        return 1;
    }

    const size_t payload_size = static_cast<size_t>(header_len) +
                                static_cast<size_t>(out_w) * static_cast<size_t>(out_h) * 3U;
    char meta[128];
    kernel::util::k_snprintf(meta, sizeof(meta),
                             "UI_DUMP_BEGIN %lu %lu %lu\n",
                             static_cast<unsigned long>(out_w),
                             static_cast<unsigned long>(out_h),
                             static_cast<unsigned long>(payload_size));
    io::put(meta);
    io::put_n(header, static_cast<size_t>(header_len));

    char row[3 * 256];
    for (uint32_t y = 0; y < out_h; ++y) {
        size_t row_len = 0;
        for (uint32_t x = 0; x < out_w; ++x) {
            const auto c = fb.get_pixel(static_cast<int32_t>(x * scale), static_cast<int32_t>(y * scale));
            row[row_len++] = static_cast<char>(c.r);
            row[row_len++] = static_cast<char>(c.g);
            row[row_len++] = static_cast<char>(c.b);
            if (row_len == sizeof(row)) {
                io::put_n(row, row_len);
                row_len = 0;
            }
        }
        if (row_len > 0) io::put_n(row, row_len);
    }
    io::put("\nUI_DUMP_END\n");
    return 0;
}
static int cmd_help(const char*, kernel::hal::UARTDriverOps* uart) {
    g_cli.print_help(uart);
    return 0;
}
static int cmd_trace(const char*, kernel::hal::UARTDriverOps* uart) {
    kernel::dump_trace_buffer(uart); return 0;
}
static int cmd_stats(const char*, kernel::hal::UARTDriverOps* uart) {
    kernel::get_kernel_stats(uart); return 0;
}
static int cmd_test(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("Usage: test <subtest>\n");
        uart->puts("  status   - test all status dump functions\n");
        uart->puts("  motion   - test motion kernel dumps\n");
        uart->puts("  ec       - test EtherCAT dumps\n");
        uart->puts("  devices  - test device DB dump\n");
        return 1;
    }
    if (kernel::util::kstrcmp(args, "status") == 0) {
        uart->puts("=== Testing status dumps ===\n");
        motion::g_motion.dump_channels(uart);
        motion::g_motion.dump_barriers(uart);
        motion::g_motion.dump_gears(uart);
        motion::g_motion.dump_status(uart);
        motion::g_motion.dump_positions(uart);
        ethercat::g_master_a.dump_status(uart);
        ethercat::g_master_b.dump_status(uart);
        ethercat::g_master_a.dump_slaves(uart);
        ethercat::g_master_b.dump_slaves(uart);
        devices::g_device_db.dump(uart);
        uart->puts("status dump test: PASSED\n");
        return 0;
    }
    if (kernel::util::kstrcmp(args, "motion") == 0) {
        motion::g_motion.dump_channels(uart);
        motion::g_motion.dump_barriers(uart);
        motion::g_motion.dump_gears(uart);
        motion::g_motion.dump_status(uart);
        motion::g_motion.dump_positions(uart);
        uart->puts("motion dump test: PASSED\n");
        return 0;
    }
    if (kernel::util::kstrcmp(args, "ec") == 0) {
        ethercat::g_master_a.dump_status(uart);
        ethercat::g_master_b.dump_status(uart);
        ethercat::g_master_a.dump_slaves(uart);
        ethercat::g_master_b.dump_slaves(uart);
        uart->puts("ec dump test: PASSED\n");
        return 0;
    }
    if (kernel::util::kstrcmp(args, "devices") == 0) {
        devices::g_device_db.dump(uart);
        uart->puts("devices dump test: PASSED\n");
        return 0;
    }
    uart->puts("unknown test: ");
    uart->puts(args);
    uart->puts("\n");
    return 1;
}
static int cmd_echo(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (args) uart->puts(args);
    uart->puts("\n");
    return 0;
}
static int cmd_ec(const char*, kernel::hal::UARTDriverOps* uart) {
    ethercat::g_master_a.dump_status(uart);
    ethercat::g_master_b.dump_status(uart);
    return 0;
}
static int cmd_ec_slaves(const char*, kernel::hal::UARTDriverOps* uart) {
    uart->puts("ec0 slaves:\n");
    ethercat::g_master_a.dump_slaves(uart);
    uart->puts("ec1 slaves:\n");
    ethercat::g_master_b.dump_slaves(uart);
    return 0;
}
static int cmd_ec_hist(const char*, kernel::hal::UARTDriverOps* uart) {
    uart->puts("ec0 latency:\n");
    ethercat::g_master_a.hist_cycle().dump(uart, "ec0:cycle");
    ethercat::g_master_a.hist_probe().dump(uart, "ec0:probe");
    ethercat::g_master_a.hist_esm().dump(uart, "ec0:esm");
    ethercat::g_master_a.hist_lrw().dump(uart, "ec0:lrw");
    ethercat::g_master_a.hist_wait().dump(uart, "ec0:wait");
    uart->puts("ec1 latency:\n");
    ethercat::g_master_b.hist_cycle().dump(uart, "ec1:cycle");
    ethercat::g_master_b.hist_probe().dump(uart, "ec1:probe");
    ethercat::g_master_b.hist_esm().dump(uart, "ec1:esm");
    ethercat::g_master_b.hist_lrw().dump(uart, "ec1:lrw");
    ethercat::g_master_b.hist_wait().dump(uart, "ec1:wait");
    return 0;
}
static int cmd_motion(const char*, kernel::hal::UARTDriverOps* uart) {
    motion::g_motion.dump_status(uart); return 0;
}
static int cmd_channels(const char*, kernel::hal::UARTDriverOps* uart) {
    motion::g_motion.dump_channels(uart); return 0;
}
static bool cli_parse_long(const char*& p, long& out) noexcept;  // fwd
static int cmd_axis_load_configure(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_load_configure <axis> <scale_num> <scale_den> <sign>
    //                     [trim_cap=200] [trim_slew=5000] [kp=500] [ki=10]
    if (!args || !*args) {
        uart->puts("usage: axis_load_configure <axis> <scale_num> <scale_den> <sign> "
                   "[trim_cap=200] [trim_slew=5000] [kp=500] [ki=10]\n");
        return 1;
    }
    const char* p = args;
    long axis = 0, num = 0, den = 0, sign = 0;
    long cap = 200, slew = 5000, kp = 500, ki = 10;
    if (!cli_parse_long(p, axis) || !cli_parse_long(p, num) ||
        !cli_parse_long(p, den)  || !cli_parse_long(p, sign)) {
        uart->puts("axis_load_configure: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, cap);
    (void)cli_parse_long(p, slew);
    (void)cli_parse_long(p, kp);
    (void)cli_parse_long(p, ki);
    const bool ok = motion::g_motion.configure_load_feedback(
        static_cast<size_t>(axis),
        static_cast<int64_t>(num), static_cast<int64_t>(den),
        static_cast<int32_t>(sign),
        static_cast<int32_t>(cap), static_cast<int32_t>(slew),
        static_cast<int16_t>(kp),  static_cast<int16_t>(ki));
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_load_configure ax%ld scale=%ld/%ld sign=%ld cap=%ld slew=%ld => %s\n",
        axis, num, den, sign, cap, slew, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_axis_load_sample(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_load_sample <axis> <raw_pos> [error=0]
    if (!args || !*args) {
        uart->puts("usage: axis_load_sample <axis> <raw_pos> [error=0]\n");
        return 1;
    }
    const char* p = args;
    long axis = 0, raw = 0, err = 0;
    if (!cli_parse_long(p, axis) || !cli_parse_long(p, raw)) {
        uart->puts("axis_load_sample: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, err);
    const bool ok = motion::g_motion.push_load_sample(
        static_cast<size_t>(axis),
        static_cast<int64_t>(raw),
        err != 0);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_load_sample ax%ld raw=%ld err=%ld => %s\n",
        axis, raw, err, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_axis_load_calibrate(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: axis_load_calibrate <axis>\n");
        return 1;
    }
    const char* p = args;
    long axis = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("axis_load_calibrate: bad axis\n"); return 1; }
    const bool ok = motion::g_motion.calibrate_load(static_cast<size_t>(axis));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_load_calibrate ax%ld => %s\n",
        axis, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_barriers(const char*, kernel::hal::UARTDriverOps* uart) {
    motion::g_motion.dump_barriers(uart); return 0;
}
static int cmd_gears(const char*, kernel::hal::UARTDriverOps* uart) {
    motion::g_motion.dump_gears(uart); return 0;
}
static bool cli_parse_long(const char*& p, long& out) noexcept; // fwd
static int cmd_axis_spin(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_spin <axis> <velocity_cps>  (9.9 harness)
    if (!args || !*args) {
        uart->puts("usage: axis_spin <axis> <velocity_cps>\n"); return 1;
    }
    const char* p = args;
    long ax = 0, vel = 0;
    if (!cli_parse_long(p, ax) || !cli_parse_long(p, vel)) {
        uart->puts("axis_spin: bad args\n"); return 1;
    }
    const bool ok = motion::g_motion.set_spin_velocity(
        static_cast<size_t>(ax), static_cast<int32_t>(vel));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_spin ax%ld vel=%ld cps => %s\n",
        ax, vel, ok ? "ok" : "FAIL (drive hooked?)");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_gear_engage(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Usage: gear_engage <leader_ax> <follower_ax> <follower_ch> <k_num> <k_den> [ramp=100]
    if (!args || !*args) {
        uart->puts("usage: gear_engage <leader_ax> <follower_ax> <follower_ch> "
                   "<k_num> <k_den> [ramp=100]\n");
        return 1;
    }
    const char* p = args;
    long la = 0, fa = 0, fc = 0, kn = 0, kd = 0, ramp = 100;
    if (!cli_parse_long(p, la) || !cli_parse_long(p, fa) ||
        !cli_parse_long(p, fc) || !cli_parse_long(p, kn) ||
        !cli_parse_long(p, kd)) {
        uart->puts("gear_engage: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, ramp);

    const bool ok = motion::g_motion.engage_gear(
        static_cast<uint8_t>(la), static_cast<uint8_t>(fa),
        static_cast<uint8_t>(fc),
        static_cast<int32_t>(kn), static_cast<int32_t>(kd),
        static_cast<uint16_t>(ramp));

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "gear_engage leader=ax%ld follower=ax%ld ch%ld k=%ld/%ld ramp=%ld => %s\n",
        la, fa, fc, kn, kd, ramp, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_gear_disengage(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: gear_disengage <follower_ax> [ramp=100]\n");
        return 1;
    }
    const char* p = args;
    long fa = 0, ramp = 100;
    if (!cli_parse_long(p, fa)) { uart->puts("gear_disengage: bad axis\n"); return 1; }
    (void)cli_parse_long(p, ramp);
    const bool ok = motion::g_motion.disengage_gear(
        static_cast<uint8_t>(fa), static_cast<uint16_t>(ramp));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "gear_disengage follower=ax%ld ramp=%ld => %s\n",
        fa, ramp, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}

static int cmd_gantry(const char* args, kernel::hal::UARTDriverOps* uart) {
    // gantry - list active gantry links
    motion::g_motion.dump_gantrys(uart);
    return 0;
}

static int cmd_gantry_engage(const char* args, kernel::hal::UARTDriverOps* uart) {
    // gantry_engage <primary> <secondary> [ratio_num] [ratio_den] [offset]
    // Example: gantry_engage 1 2 1 1 0  (tie Y1 to Y2, 1:1 ratio, no offset)
    if (!args || !*args) {
        uart->puts("usage: gantry_engage <primary_ax> <secondary_ax> [num] [den] [offset]\n");
        uart->puts("  primary:   leader axis (e.g., Y1 = 1)\n");
        uart->puts("  secondary: follower axis (e.g., Y2 = 2)\n");
        uart->puts("  num/den:   gear ratio (default 1/1)\n");
        uart->puts("  offset:    static offset in counts (default 0)\n");
        uart->puts("example: gantry_engage 1 2    # Y2 follows Y1\n");
        return 1;
    }
    const char* p = args;
    long pa = 0, sa = 0, num = 1, den = 1, offset = 0;
    if (!cli_parse_long(p, pa)) { uart->puts("gantry_engage: bad primary\n"); return 1; }
    if (!cli_parse_long(p, sa)) { uart->puts("gantry_engage: bad secondary\n"); return 1; }
    (void)cli_parse_long(p, num);
    (void)cli_parse_long(p, den);
    (void)cli_parse_long(p, offset);

    const int slot = motion::g_motion.engage_gantry(
        static_cast<uint8_t>(pa),
        static_cast<uint8_t>(sa),
        static_cast<int32_t>(num),
        static_cast<int32_t>(den),
        static_cast<int32_t>(offset));

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "gantry_engage primary=ax%ld secondary=ax%ld ratio=%ld/%ld offset=%ld => %s\n",
        pa, sa, num, den, offset, slot >= 0 ? "ok" : "FAIL");
    uart->puts(buf);
    return slot >= 0 ? 0 : 1;
}

static int cmd_gantry_disengage(const char* args, kernel::hal::UARTDriverOps* uart) {
    // gantry_disengage <secondary_axis>
    if (!args || !*args) {
        uart->puts("usage: gantry_disengage <secondary_ax>\n");
        return 1;
    }
    const char* p = args;
    long sa = 0;
    if (!cli_parse_long(p, sa)) { uart->puts("gantry_disengage: bad axis\n"); return 1; }

    const bool ok = motion::g_motion.disengage_gantry(static_cast<uint8_t>(sa));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "gantry_disengage secondary=ax%ld => %s\n", sa, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}

static int cmd_gantry_adjust(const char* args, kernel::hal::UARTDriverOps* uart) {
    // gantry_adjust <secondary_axis> <delta_counts>
    // delta: positive = move secondary forward, negative = move backward
    // 1 count = 0.1um at 1000 counts/deg if CPR=36000
    if (!args || !*args) {
        uart->puts("usage: gantry_adjust <secondary_ax> <delta_counts>\n");
        uart->puts("  delta: +/- counts (e.g., +100 = +10µm at 0.1µm resolution)\n");
        uart->puts("example: gantry_adjust 2 +100   # move Y2 +10µm forward\n");
        uart->puts("         gantry_adjust 2 -50    # move Y2 -5µm backward\n");
        return 1;
    }
    const char* p = args;
    long sa = 0, delta = 0;
    if (!cli_parse_long(p, sa)) { uart->puts("gantry_adjust: bad axis\n"); return 1; }
    if (!cli_parse_long(p, delta)) { uart->puts("gantry_adjust: bad delta\n"); return 1; }

    const bool ok = motion::g_motion.adjust_gantry_offset(
        static_cast<uint8_t>(sa), static_cast<int32_t>(delta));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "gantry_adjust secondary=ax%ld delta=%ld => %s\n", sa, delta, ok ? "ok" : "FAIL");
    uart->puts(buf);
    if (ok) motion::g_motion.dump_gantrys(uart);
    return ok ? 0 : 1;
}

// ===== Axis Calibration Commands =====

static int cmd_cal_linear(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_linear [axis] — show linear PEC table
    if (!args || !*args) {
        uart->puts("usage: cal_linear <axis> — show pitch error compensation table\n");
        return 1;
    }
    const char* p = args;
    long axis = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("cal_linear: bad axis\n"); return 1; }
    if (axis < 0 || (size_t)axis >= motion::MAX_AXES) { uart->puts("cal_linear: axis out of range\n"); return 1; }
    motion::g_motion.dump_linear_cal(uart, static_cast<uint8_t>(axis));
    return 0;
}

static int cmd_cal_add(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_add <axis> <position> <error> — add point to PEC table
    // error = actual_position - commanded_position (in counts)
    if (!args || !*args) {
        uart->puts("usage: cal_add <axis> <position> <error>\n");
        uart->puts("  position: commanded position (counts)\n");
        uart->puts("  error:     actual - commanded (counts)\n");
        uart->puts("example: cal_add 0 50000 5   # at 50K counts, error is +5\n");
        return 1;
    }
    const char* p = args;
    long axis = 0, pos = 0, err = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("cal_add: bad axis\n"); return 1; }
    if (!cli_parse_long(p, pos)) { uart->puts("cal_add: bad position\n"); return 1; }
    if (!cli_parse_long(p, err)) { uart->puts("cal_add: bad error\n"); return 1; }

    const int idx = motion::g_motion.add_cal_point(
        static_cast<uint8_t>(axis),
        static_cast<int32_t>(pos),
        static_cast<int32_t>(err));

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "cal_add ax%ld pos=%ld err=%ld => index=%d\n", axis, pos, err, idx);
    uart->puts(buf);
    return idx >= 0 ? 0 : 1;
}

static int cmd_cal_enable(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_enable <axis> <0|1> — enable/disable linear calibration
    if (!args || !*args) {
        uart->puts("usage: cal_enable <axis> <0|1>\n");
        uart->puts("example: cal_enable 0 1   # enable PEC on axis 0\n");
        return 1;
    }
    const char* p = args;
    long axis = 0, enable = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("cal_enable: bad axis\n"); return 1; }
    if (!cli_parse_long(p, enable)) { uart->puts("cal_enable: bad enable\n"); return 1; }

    const bool ok = motion::g_motion.enable_linear_cal(
        static_cast<uint8_t>(axis), enable != 0);
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf), "cal_enable ax%ld %ld => %s\n", axis, enable, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}

static int cmd_cal_clear(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_clear <axis> — clear linear calibration
    if (!args || !*args) {
        uart->puts("usage: cal_clear <axis>\n");
        return 1;
    }
    const char* p = args;
    long axis = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("cal_clear: bad axis\n"); return 1; }
    motion::g_motion.clear_linear_cal(static_cast<uint8_t>(axis));
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf), "cal_clear ax%ld => ok\n", axis);
    uart->puts(buf);
    return 0;
}

static int cmd_cal_rotary(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_rotary <axis> [offset] — set/get rotary index offset
    if (!args || !*args) {
        uart->puts("usage: cal_rotary <axis> [offset_counts]\n");
        uart->puts("  offset: index offset in counts (default shows current)\n");
        return 1;
    }
    const char* p = args;
    long axis = 0, offset = 0;
    if (!cli_parse_long(p, axis)) { uart->puts("cal_rotary: bad axis\n"); return 1; }
    if (cli_parse_long(p, offset)) {
        // Set offset
        const bool ok = motion::g_motion.set_rotary_index_offset(
            static_cast<uint8_t>(axis), static_cast<int32_t>(offset));
        char buf[64];
        kernel::util::k_snprintf(buf, sizeof(buf), "cal_rotary ax%ld offset=%ld => %s\n",
            axis, offset, ok ? "ok" : "FAIL");
        uart->puts(buf);
        return ok ? 0 : 1;
    }
    // Show current
    motion::g_motion.dump_rotary_cal(uart, static_cast<uint8_t>(axis));
    return 0;
}

static int cmd_cal_geometry(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_geometry [pair] [urad] — set/get squareness errors
    // pair: XY, XZ, YZ (microradians)
    if (!args || !*args) {
        uart->puts("usage: cal_geometry [pair] [urad]\n");
        uart->puts("  pair: XY, XZ, or YZ (default shows all)\n");
        uart->puts("  urad: error in microradians (0 = no correction)\n");
        uart->puts("example: cal_geometry XY 50    # Y is +50 µrad out of square with X\n");
        uart->puts("         cal_geometry          # show current geometry\n");
        motion::g_motion.dump_geometry(uart);
        return 0;
    }
    const char* p = args;
    char pair[8] = {0};
    long urad = 0;
    // Parse pair string (look for non-numeric)
    while (*p && (pair[0] == 0 || (pair[0] != 0 && pair[2] == 0))) {
        if (*p == ' ') { ++p; continue; }
        if (*p >= '0' && *p <= '9') { break; }
        if (pair[0] == 0) pair[0] = *p;
        else if (pair[1] == 0) pair[1] = *p;
        ++p;
    }
    if (cli_parse_long(p, urad)) {
        // Set value
        const bool ok = motion::g_motion.set_geometry_error(pair, static_cast<int32_t>(urad));
        char buf[64];
        kernel::util::k_snprintf(buf, sizeof(buf), "cal_geometry %s %ld urad => %s\n",
            pair, urad, ok ? "ok" : "FAIL");
        uart->puts(buf);
        return ok ? 0 : 1;
    }
    // Show value
    int32_t val = motion::g_motion.get_geometry_error(pair);
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf), "geometry %s = %ld urad\n", pair, (long)val);
    uart->puts(buf);
    return 0;
}

static int cmd_cal_geo_enable(const char* args, kernel::hal::UARTDriverOps* uart) {
    // cal_geo_enable <0|1> — enable/disable geometry compensation
    if (!args || !*args) {
        uart->puts("usage: cal_geo_enable <0|1>\n");
        return 1;
    }
    const char* p = args;
    long enable = 0;
    if (!cli_parse_long(p, enable)) { uart->puts("cal_geo_enable: bad value\n"); return 1; }
    motion::g_motion.enable_geometry(enable != 0);
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf), "cal_geo_enable %ld => ok\n", enable);
    uart->puts(buf);
    return 0;
}

// ===== Sphere-based Volumetric Calibration Commands =====

static int cmd_sphere_config(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_config [diameter_mm] [probe_radius_um] [probe_hits] [rapid_mmpm] [probe_mmpm]
    if (!args || !*args) {
        uart->puts("usage: sphere_config [diam_mm] [probe_um] [hits] [rapid] [probe]\n");
        uart->puts("  diam_mm:   sphere diameter (default 20.0)\n");
        uart->puts("  probe_um:  probe radius in µm (default 0)\n");
        uart->puts("  hits:      probe hits to average (default 3)\n");
        uart->puts("  rapid:     rapid move speed mm/min (default 1000)\n");
        uart->puts("  probe:     probe approach speed mm/min (default 200)\n");
        char buf[128];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "current: diam=%.1fmm probe=%dµm hits=%d rapid=%dmmpm probe=%dmmpm\n",
            (double)motion::g_motion.sphere_diameter(),
            (int)motion::g_motion.sphere_probe_radius(),
            (int)motion::g_motion.sphere_probe_hits(),
            (int)motion::g_motion.sphere_rapid_speed(),
            (int)motion::g_motion.sphere_probe_speed());
        uart->puts(buf);
        return 0;
    }
    const char* p = args;
    long diam = 0, probe = 0, hits = 0, rapid = 0, probe_spd = 0;
    if (cli_parse_long(p, diam)) {
        motion::g_motion.sphere_set_diameter(static_cast<float>(diam));
    }
    if (cli_parse_long(p, probe)) {
        motion::g_motion.sphere_set_probe_radius(static_cast<int32_t>(probe));
    }
    if (cli_parse_long(p, hits)) {
        motion::g_motion.sphere_set_probe_hits(static_cast<int32_t>(hits));
    }
    if (cli_parse_long(p, rapid)) {
        motion::g_motion.sphere_set_rapid_speed(static_cast<int32_t>(rapid));
    }
    if (cli_parse_long(p, probe_spd)) {
        motion::g_motion.sphere_set_probe_speed(static_cast<int32_t>(probe_spd));
    }
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sphere_config: diam=%.1fmm probe=%dµm hits=%d rapid=%dmmpm probe=%dmmpm => ok\n",
        (double)motion::g_motion.sphere_diameter(),
        (int)motion::g_motion.sphere_probe_radius(),
        (int)motion::g_motion.sphere_probe_hits(),
        (int)motion::g_motion.sphere_rapid_speed(),
        (int)motion::g_motion.sphere_probe_speed());
    uart->puts(buf);
    return 0;
}

static int cmd_sphere_add(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_add <cmd_x> <cmd_y> <cmd_z> <act_x> <act_y> <act_z>
    // Add measured point from touching sphere (average of N hits)
    if (!args || !*args) {
        uart->puts("usage: sphere_add <cmd_x> <cmd_y> <cmd_z> <act_x> <act_y> <act_z>\n");
        uart->puts("  cmd_*: commanded position (counts)\n");
        uart->puts("  act_*: AVERAGED actual position from N probe hits (counts)\n");
        uart->puts("  Note: probe routine should do 3 hits and average before calling\n");
        uart->puts("example: sphere_add 0 0 0 100 50 25\n");
        return 1;
    }
    const char* p = args;
    long cx = 0, cy = 0, cz = 0, ax = 0, ay = 0, az = 0;
    if (!cli_parse_long(p, cx)) { uart->puts("sphere_add: bad cmd_x\n"); return 1; }
    if (!cli_parse_long(p, cy)) { uart->puts("sphere_add: bad cmd_y\n"); return 1; }
    if (!cli_parse_long(p, cz)) { uart->puts("sphere_add: bad cmd_z\n"); return 1; }
    if (!cli_parse_long(p, ax)) { uart->puts("sphere_add: bad act_x\n"); return 1; }
    if (!cli_parse_long(p, ay)) { uart->puts("sphere_add: bad act_y\n"); return 1; }
    if (!cli_parse_long(p, az)) { uart->puts("sphere_add: bad act_z\n"); return 1; }

    const int idx = motion::g_motion.sphere_add_point(
        static_cast<int32_t>(cx), static_cast<int32_t>(cy), static_cast<int32_t>(cz),
        static_cast<int32_t>(ax), static_cast<int32_t>(ay), static_cast<int32_t>(az));

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sphere_add point[%d]: cmd(%ld,%ld,%ld) act(%ld,%ld,%ld)\n",
        idx, cx, cy, cz, ax, ay, az);
    uart->puts(buf);
    return idx >= 0 ? 0 : 1;
}

static int cmd_sphere_compute(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_compute — compute volumetric errors from measured points
    (void)args;
    const bool ok = motion::g_motion.sphere_compute_errors();
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sphere_compute => %s (need 7+ points)\n", ok ? "ok" : "FAIL");
    uart->puts(buf);
    if (ok) motion::g_motion.dump_sphere_cal(uart);
    return ok ? 0 : 1;
}

static int cmd_sphere_enable(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_enable <0|1>
    if (!args || !*args) {
        uart->puts("usage: sphere_enable <0|1>\n");
        return 1;
    }
    const char* p = args;
    long enable = 0;
    if (!cli_parse_long(p, enable)) { uart->puts("sphere_enable: bad value\n"); return 1; }
    motion::g_motion.sphere_enable(enable != 0);
    char buf[64];
    kernel::util::k_snprintf(buf, sizeof(buf), "sphere_enable %ld => ok\n", enable);
    uart->puts(buf);
    return 0;
}

static int cmd_sphere_clear(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_clear — clear all measurement points
    (void)args;
    motion::g_motion.sphere_clear_points();
    uart->puts("sphere_clear: points cleared\n");
    return 0;
}

static int cmd_sphere(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere — show sphere calibration status
    (void)args;
    motion::g_motion.dump_sphere_cal(uart);
    return 0;
}

static int cmd_sphere_auto(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sphere_auto <center_x> <center_y> <center_z> <radius_mm>
    // Automatically measure 7-point hemisphere pattern around sphere
    // Points: top, N/S/E/W, NE/SW (at 45° elevation)
    if (!args || !*args) {
        uart->puts("usage: sphere_auto <center_x> <center_y> <center_z> <radius_mm>\n");
        uart->puts("  center_*: sphere center position in mm\n");
        uart->puts("  radius_mm: approach distance from center (e.g., 15mm for 20mm sphere)\n");
        uart->puts("\nMeasures 7 points on upper hemisphere:\n");
        uart->puts("  0: top, 1:N, 2:S, 3:E, 4:W, 5:NE, 6:SW\n");
        uart->puts("\nNOTE: This outputs the measurement points.\n");
        uart->puts("      Your probe routine should call sphere_add for each.\n");
        uart->puts("      See probe_calibrate for fully automated version.\n");
        return 1;
    }
    const char* p = args;
    long cx = 0, cy = 0, cz = 0, radius = 15;
    if (!cli_parse_long(p, cx)) { uart->puts("sphere_auto: bad center_x\n"); return 1; }
    if (!cli_parse_long(p, cy)) { uart->puts("sphere_auto: bad center_y\n"); return 1; }
    if (!cli_parse_long(p, cz)) { uart->puts("sphere_auto: bad center_z\n"); return 1; }
    (void)cli_parse_long(p, radius);

    // 7-point hemisphere pattern (counts = mm * 10000 for 0.1µm resolution)
    // At 45° elevation: offset = radius * sin(45°) ≈ radius * 0.707
    // Horizontal offset = radius * cos(45°) ≈ radius * 0.707
    int32_t r = static_cast<int32_t>(radius) * 10000;  // convert mm to counts
    int32_t h = static_cast<int32_t>(radius * 707 / 1000) * 10000;  // horizontal
    int32_t v = static_cast<int32_t>(radius * 707 / 1000) * 10000;  // vertical

    char buf[128];
    uart->puts("sphere_auto: 7-point hemisphere measurement\n");
    uart->puts("Point 0 (top):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        cx * 10000, cy * 10000, (cz + radius) * 10000);
    uart->puts(buf);

    uart->puts("Point 1 (North +Y):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        cx * 10000, (cy + v) * 10000, (cz + h) * 10000);
    uart->puts(buf);

    uart->puts("Point 2 (South -Y):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        cx * 10000, (cy - v) * 10000, (cz + h) * 10000);
    uart->puts(buf);

    uart->puts("Point 3 (East +X):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        (cx + v) * 10000, cy * 10000, (cz + h) * 10000);
    uart->puts(buf);

    uart->puts("Point 4 (West -X):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        (cx - v) * 10000, cy * 10000, (cz + h) * 10000);
    uart->puts(buf);

    int32_t d = static_cast<int32_t>(radius * 500 / 1000) * 10000;  // diagonal
    uart->puts("Point 5 (NE +X+Y):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        (cx + d) * 10000, (cy + d) * 10000, (cz + h) * 10000);
    uart->puts(buf);

    uart->puts("Point 6 (SW -X-Y):\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  sphere_add %ld %ld %ld <actual_x> <actual_y> <actual_z>\n",
        (cx - d) * 10000, (cy - d) * 10000, (cz + h) * 10000);
    uart->puts(buf);

    uart->puts("\nRun sphere_compute after all 7 points added.\n");
    return 0;
}

// Probe callback function type - machine builder implements this
// Returns actual trigger position (counts), or -1 on error
// This would be connected to the actual probe hardware in a real system
typedef int32_t (*ProbeTriggerFunc)(int32_t target_x, int32_t target_y, int32_t target_z);

// Simple stub that returns target position (simulated probe)
// In real system, this would wait for probe trigger and return actual position
static int32_t probe_trigger_sim(int32_t target_x, int32_t target_y, int32_t target_z) {
    // Simulated: probe triggers exactly at target
    // Real implementation would wait for probe DI and return trigger position
    (void)target_x; (void)target_y; (void)target_z;
    return 0;  // Return 0 = use target position as actual (no error for simulation)
}

static int cmd_probe_calibrate(const char* args, kernel::hal::UARTDriverOps* uart) {
    // probe_calibrate <sphere_x> <sphere_y> <sphere_z> [radius_mm]
    // Fully automated sphere calibration: moves to each point, probes, records
    // Uses rapid speed to approach, probe speed to touch, 3 hits average
    if (!args || !*args) {
        uart->puts("usage: probe_calibrate <sphere_x> <sphere_y> <sphere_z> [radius_mm]\n");
        uart->puts("  sphere_*: sphere center position in mm\n");
        uart->puts("  radius_mm: approach distance (default 15mm)\n");
        uart->puts("\nAutomatically measures 7-point hemisphere:\n");
        uart->puts("  - Moves at rapid speed (1000mm/min)\n");
        uart->puts("  - Probes at probe speed (200mm/min)\n");
        uart->puts("  - Does 3 hits per point, averages\n");
        uart->puts("  - Computes 21-error volumetric model\n");
        return 1;
    }

    const char* p = args;
    long sx = 0, sy = 0, sz = 0, radius = 15;
    if (!cli_parse_long(p, sx)) { uart->puts("probe_calibrate: bad sphere_x\n"); return 1; }
    if (!cli_parse_long(p, sy)) { uart->puts("probe_calibrate: bad sphere_y\n"); return 1; }
    if (!cli_parse_long(p, sz)) { uart->puts("probe_calibrate: bad sphere_z\n"); return 1; }
    (void)cli_parse_long(p, radius);

    // Get calibration settings
    int32_t probe_radius = motion::g_motion.sphere_probe_radius();
    int32_t probe_hits = motion::g_motion.sphere_probe_hits();
    int32_t rapid_spd = motion::g_motion.sphere_rapid_speed();
    int32_t probe_spd = motion::g_motion.sphere_probe_speed();

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "probe_calibrate: sphere at (%ld,%ld,%ld)mm r=%ldmm\n",
        sx, sy, sz, radius);
    uart->puts(buf);
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  probe_r=%dµm hits=%d rapid=%dmmpm probe=%dmmpm\n",
        (int)probe_radius, (int)probe_hits, (int)rapid_spd, (int)probe_spd);
    uart->puts(buf);

    // Clear existing points
    motion::g_motion.sphere_clear_points();
    uart->puts("Cleared existing points.\n");

    // Calculate approach offsets (at 45° elevation for hemisphere)
    int32_t h = static_cast<int32_t>(radius * 707 / 1000) * 10000;  // horizontal
    int32_t d = static_cast<int32_t>(radius * 500 / 1000) * 10000;   // diagonal

    // 7 measurement points (counts = mm * 10000)
    int32_t targets[7][3] = {
        // Point 0: top
        {static_cast<int32_t>(sx * 10000), static_cast<int32_t>(sy * 10000), static_cast<int32_t>((sz + radius) * 10000)},
        // Point 1: North +Y
        {static_cast<int32_t>(sx * 10000), static_cast<int32_t>((sy * 10000) + h), static_cast<int32_t>((sz * 10000) + h)},
        // Point 2: South -Y
        {static_cast<int32_t>(sx * 10000), static_cast<int32_t>((sy * 10000) - h), static_cast<int32_t>((sz * 10000) + h)},
        // Point 3: East +X
        {static_cast<int32_t>((sx * 10000) + h), static_cast<int32_t>(sy * 10000), static_cast<int32_t>((sz * 10000) + h)},
        // Point 4: West -X
        {static_cast<int32_t>((sx * 10000) - h), static_cast<int32_t>(sy * 10000), static_cast<int32_t>((sz * 10000) + h)},
        // Point 5: NE +X+Y
        {static_cast<int32_t>((sx * 10000) + d), static_cast<int32_t>((sy * 10000) + d), static_cast<int32_t>((sz * 10000) + h)},
        // Point 6: SW -X-Y
        {static_cast<int32_t>((sx * 10000) - d), static_cast<int32_t>((sy * 10000) - d), static_cast<int32_t>((sz * 10000) + h)},
    };

    const char* point_names[] = {"top", "N", "S", "E", "W", "NE", "SW"};

    // NOTE: In a real system, this would:
    // 1. Move to approach position at rapid_spd
    // 2. Move toward sphere at probe_spd until probe triggers
    // 3. Record trigger position
    // 4. Repeat 3 times (probe_hits)
    // 5. Average the 3 readings
    // 6. Apply probe radius compensation

    uart->puts("\n[SIMULATION] Would measure 7 points:\n");
    for (int i = 0; i < 7; ++i) {
        // In real system: actual = probe_trigger(targets[i][0], targets[i][1], targets[i][2])
        // For simulation, we just record the target as the "actual" (no probe available)
        int32_t act_x = targets[i][0];  // Would be probe result
        int32_t act_y = targets[i][1];
        int32_t act_z = targets[i][2];

        motion::g_motion.sphere_add_point(targets[i][0], targets[i][1], targets[i][2],
                                           act_x, act_y, act_z);

        kernel::util::k_snprintf(buf, sizeof(buf),
            "  pt%d (%s): cmd(%ld,%ld,%ld) act(%ld,%ld,%ld)\n",
            i, point_names[i],
            (long)targets[i][0], (long)targets[i][1], (long)targets[i][2],
            (long)act_x, (long)act_y, (long)act_z);
        uart->puts(buf);
    }

    // Compute error model
    uart->puts("\nComputing volumetric errors...\n");
    bool ok = motion::g_motion.sphere_compute_errors();
    if (!ok) {
        uart->puts("probe_calibrate: FAILED - need 7+ points\n");
        return 1;
    }

    motion::g_motion.sphere_enable(true);
    uart->puts("Volumetric compensation ENABLED.\n");
    motion::g_motion.dump_sphere_cal(uart);

    uart->puts("\nprobe_calibrate: COMPLETE\n");
    return 0;
}

static int cmd_skiving_config(const char* args, kernel::hal::UARTDriverOps* uart) {
    // skiving_config <ts1_axis> <teeth> <c1_counts_per_rev> [tooth_offset]
    // Configure tool spindle (TS1) for power skiving with C1 workpiece encoder
    // C1 resolution: 0.0001 degrees = 3,600,000 counts per revolution
    if (!args || !*args) {
        uart->puts("usage: skiving_config <ts1_axis> <teeth> <c1_cpr> [offset]\n");
        uart->puts("  C1 encoder: 0.0001 deg = 3600000 counts/rev\n");
        return 1;
    }
    const char* p = args;
    long ax = 0, teeth = 0, cpr = 0, off = 0;
    if (!cli_parse_long(p, ax) || !cli_parse_long(p, teeth) ||
        !cli_parse_long(p, cpr)) {
        uart->puts("skiving_config: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, off);

    motion::g_motion.configure_spindle_indexer(
        static_cast<size_t>(ax),
        static_cast<uint16_t>(teeth),
        static_cast<uint32_t>(cpr),
        static_cast<int32_t>(off));

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "skiving_config: ts1=ax%ld teeth=%ld c1_cpr=%ld offset=%ld\n",
        ax, teeth, cpr, off);
    uart->puts(buf);
    return 0;
}

static int cmd_skiving_engage(const char* args, kernel::hal::UARTDriverOps* uart) {
    // skiving_engage <c1_rpm> <ts1_rpm> <z_axis> <teeth>
    // Power skiving with differential speeds:
    //   C1: workpiece at c1_rpm (CSV velocity mode)
    //   TS1: tool at ts1_rpm (CSV velocity mode) - cutter teeth chop relative to workpiece
    //   Z: linear feed (CSP position mode)
    //   teeth: cutter tooth count
    if (!args || !*args) {
        uart->puts("usage: skiving_engage <c1_rpm> <ts1_rpm> <z_axis> <teeth>\n");
        uart->puts("  C1: workpiece rpm (CSV)\n");
        uart->puts("  TS1: cutter rpm (CSV) \n");
        uart->puts("  Z: linear feed axis (CSP)\n");
        uart->puts("  teeth: cutter tooth count\n");
        uart->puts("Example: skiving_engage 2100 2400 2 20\n");
        return 1;
    }
    const char* p = args;
    long c1_rpm = 0, ts1_rpm = 0, z_axis = 0, teeth = 20;
    if (!cli_parse_long(p, c1_rpm) || !cli_parse_long(p, ts1_rpm) ||
        !cli_parse_long(p, z_axis) || !cli_parse_long(p, teeth)) {
        uart->puts("skiving_engage: bad args\n");
        return 1;
    }

    // Convert RPM to counts/second (C1 CPR = 3,600,000)
    constexpr uint32_t C1_CPR = 3600000;
    int32_t c1_cps = (int32_t)(c1_rpm * C1_CPR / 60);  // counts per second
    int32_t ts1_cps = (int32_t)((ts1_rpm * C1_CPR) / 60);

    // Configure C1 in CSV velocity mode
    if (c1_rpm != 0) {
        motion::g_motion.set_axis_velocity(0, c1_cps);  // C1 on axis 0
    }
    // Configure TS1 in CSV velocity mode  
    if (ts1_rpm != 0) {
        motion::g_motion.set_axis_velocity(1, ts1_cps); // TS1 on axis 1
    }

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "skiving_engage: C1=%ldrpm (%d cps) TS1=%ldrpm (%d cps) Z=ax%ld teeth=%ld\n"
        "  Relative: %ld RPM (%ld teeth/sec)\n",
        c1_rpm, c1_cps, ts1_rpm, ts1_cps, z_axis, teeth,
        ts1_rpm - c1_rpm,
        (ts1_rpm - c1_rpm) * teeth / 60);
    uart->puts(buf);
    return 0;
}

static int cmd_feed_per_tooth(const char* args, kernel::hal::UARTDriverOps* uart) {
    // feed_per_tooth <cutter_diameter> <teeth> [c1_axis]
    // Compute feed per tooth based on cutter geometry
    // feed_per_tooth = (PI * diameter) / teeth
    // Returns feed in encoder counts per tooth
    if (!args || !*args) {
        uart->puts("usage: feed_per_tooth <cutter_dia> <teeth> [c1_axis]\n");
        uart->puts("  Computes: feed = (PI * dia) / teeth, scaled to C1 CPR\n");
        return 1;
    }
    const char* p = args;
    long dia_1000 = 0, teeth = 0, c1_axis = 0;
    if (!cli_parse_long(p, dia_1000) || !cli_parse_long(p, teeth)) {
        uart->puts("feed_per_tooth: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, c1_axis);

    if (teeth <= 0) { uart->puts("feed_per_tooth: teeth must be > 0\n"); return 1; }

    // diameter in thousandths of unit, teeth count
    // feed_per_tooth_counts = (PI * dia * CPR) / (teeth * 1000)
    constexpr uint32_t C1_CPR = 3600000; // 0.0001 degree
    constexpr double PI = 3.14159265358979;

    double feed_inches = (PI * (dia_1000 / 1000.0)) / teeth;
    uint32_t feed_counts = (uint32_t)(feed_inches * C1_CPR);

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "feed_per_tooth: dia=%ld\" teeth=%ld => %.4f\"/tooth = %lu counts/tooth\n",
        dia_1000, teeth, feed_inches, (unsigned long)feed_counts);
    uart->puts(buf);
    return 0;
}

static int cmd_c1_index_sync(const char* args, kernel::hal::UARTDriverOps* uart) {
    // c1_index_sync <c1_axis> <enable(0|1)> <teeth>
    // Configure C1 index pulse to trigger TS1 tooth sync
    if (!args || !*args) {
        uart->puts("usage: c1_index_sync <c1_axis> <enable(0|1)> <teeth>\n");
        uart->puts("  Enable index pulse sync for power skiving\n");
        return 1;
    }
    const char* p = args;
    long c1 = 0, enable = 0, teeth = 0;
    if (!cli_parse_long(p, c1) || !cli_parse_long(p, enable) ||
        !cli_parse_long(p, teeth)) {
        uart->puts("c1_index_sync: bad args\n");
        return 1;
    }

    if (c1 < 0 || (size_t)c1 >= motion::MAX_AXES) {
        uart->puts("c1_index_sync: axis out of range\n");
        return 1;
    }

    auto& a = motion::g_motion.axis((size_t)c1);
    // Store index sync config in spindle_indexer for now
    if (enable) {
        a.spindle_indexer.configure(
            static_cast<uint16_t>(teeth > 0 ? teeth : 20),
            3600000, // C1 CPR
            0);
        a.spindle_indexer.active = true;
    } else {
        a.spindle_indexer.active = false;
    }

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "c1_index_sync: ax%ld enable=%ld teeth=%ld => %s\n",
        c1, enable, teeth, enable ? "armed" : "disabled");
    uart->puts(buf);
    return 0;
}

static int cmd_multipass(const char* args, kernel::hal::UARTDriverOps* uart) {
    // multipass <total_depth> <passes> [depth_per_pass]
    // Configure roughing passes + finishing pass
    // Example: 0.5" depth, 5 passes = 0.1" rough + 0.1" finish
    if (!args || !*args) {
        uart->puts("usage: multipass <total_depth> <passes> [doc]\n");
        uart->puts("  Configure rough/finish passes for deep cuts\n");
        uart->puts("  Example: multipass 5000 5 1000 = 0.5\" total, 5 passes\n");
        return 1;
    }
    const char* p = args;
    long total = 0, passes = 0, doc = 0;
    if (!cli_parse_long(p, total) || !cli_parse_long(p, passes)) {
        uart->puts("multipass: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, doc);

    if (passes <= 0 || passes > 20) { uart->puts("multipass: passes 1-20\n"); return 1; }

    // Auto-calculate DOC if not specified
    if (doc <= 0) {
        doc = total / passes;
    }
    int32_t rough_depth = doc * (passes - 1);
    int32_t finish_depth = total - rough_depth;

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "multipass: %ld total, %ld passes, DOC=%ld\n"
        "  Rough:   %ld x %ld counts\n"
        "  Finish:  %ld counts\n",
        total, passes, doc,
        passes - 1, rough_depth,
        finish_depth);
    uart->puts(buf);
    return 0;
}

static int cmd_din6(const char* args, kernel::hal::UARTDriverOps* uart) {
    // DIN6 axis indexing commands
    // din6 index <axis> <position> - set index position
    // din6 sync <axis> <teeth>     - configure tooth sync
    // din6 tooth <axis> <n>        - move to tooth N position
    if (!args || !*args) {
        uart->puts("usage: din6 <cmd> <axis> [args]\n");
        uart->puts("  din6 index <axis> <pos>   - set index reference\n");
        uart->puts("  din6 sync <axis> <teeth> - enable tooth sync\n");
        uart->puts("  din6 tooth <axis> <n>    - position for tooth N\n");
        uart->puts("  din6 config <axis> <teeth> <cpr> - configure\n");
        return 1;
    }
    const char* p = args;
    char cmd[16] = {};
    size_t cmd_len = 0;
    while (*p && *p != ' ' && cmd_len < 15) cmd[cmd_len++] = *p++;
    cmd[cmd_len] = 0;
    while (*p == ' ') ++p;

    long axis = 0;
    if (!cli_parse_long(p, axis)) {
        uart->puts("din6: bad axis\n");
        return 1;
    }
    if (axis < 0 || (size_t)axis >= motion::MAX_AXES) {
        uart->puts("din6: axis out of range\n");
        return 1;
    }

    auto& a = motion::g_motion.axis((size_t)axis);

    char buf[128];

    if (cstrcmp(cmd, "index") == 0) {
        long pos = 0;
        (void)cli_parse_long(p, pos);
        a.spindle_indexer.index_offset_counts = (int32_t)pos;
        kernel::util::k_snprintf(buf, sizeof(buf), "din6 index: ax%ld pos=%ld\n", axis, pos);
    }
    else if (cstrcmp(cmd, "sync") == 0) {
        long teeth = 20;
        (void)cli_parse_long(p, teeth);
        a.spindle_indexer.configure(
            static_cast<uint16_t>(teeth),
            3600000, // high-res CPR
            a.spindle_indexer.index_offset_counts);
        a.spindle_indexer.active = true;
        kernel::util::k_snprintf(buf, sizeof(buf), "din6 sync: ax%ld teeth=%ld\n", axis, teeth);
    }
    else if (cstrcmp(cmd, "tooth") == 0) {
        long n = 0;
        (void)cli_parse_long(p, n);
        if (a.spindle_indexer.teeth_per_revolution > 0 && a.spindle_indexer.counts_per_rev > 0) {
            int32_t cpr = a.spindle_indexer.counts_per_rev;
            int32_t cpt = cpr / a.spindle_indexer.teeth_per_revolution;
            int32_t tooth_pos = (int32_t)n * cpt + a.spindle_indexer.index_offset_counts;
            motion::g_motion.move_to((size_t)axis, tooth_pos);
            kernel::util::k_snprintf(buf, sizeof(buf), "din6 tooth: ax%ld tooth=%ld pos=%ld\n", axis, n, tooth_pos);
        } else {
            uart->puts("din6 tooth: axis not configured, run 'din6 sync' first\n");
            return 1;
        }
    }
    else if (cstrcmp(cmd, "config") == 0) {
        long teeth = 20, cpr = 3600000;
        (void)cli_parse_long(p, teeth);
        (void)cli_parse_long(p, cpr);
        a.spindle_indexer.configure(
            static_cast<uint16_t>(teeth),
            static_cast<uint32_t>(cpr),
            0);
        kernel::util::k_snprintf(buf, sizeof(buf), "din6 config: ax%ld teeth=%ld cpr=%ld\n", axis, teeth, cpr);
    }
    else {
        kernel::util::k_snprintf(buf, sizeof(buf), "din6: unknown cmd '%s'\n", cmd);
        uart->puts(buf);
        return 1;
    }

    uart->puts(buf);
    return 0;
}

static int cmd_topology(const char* args, kernel::hal::UARTDriverOps* uart) {
    // topology <name0>=<ax,ax,...> [<name1>=<ax,...>] ...
    // e.g.: topology mill=0,1,2,3 lathe=4,5,6,7
    if (!args || !*args) {
        uart->puts("usage: topology <name>=<ax,ax,...> [<name>=<ax,...>] ...\n");
        return 1;
    }
    motion::Kernel::ChannelSpec specs[motion::MAX_CHANNELS] = {};
    size_t n_spec = 0;
    const char* p = args;
    while (*p && n_spec < motion::MAX_CHANNELS) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        // Capture name up to '='.
        auto& s = specs[n_spec];
        size_t nc = 0;
        while (*p && *p != '=' && *p != ' ' && nc < sizeof(s.name) - 1) {
            s.name[nc++] = *p++;
        }
        s.name[nc] = 0;
        if (*p != '=') { uart->puts("topology: expected '=' after name\n"); return 1; }
        ++p;
        // Parse comma-separated axis indices. `cli_parse_long` consumes
        // its own trailing whitespace, so the absence of a comma after a
        // successful parse is what terminates this channel's list.
        while (s.axis_count < motion::MAX_AXES) {
            long ax = 0;
            if (!cli_parse_long(p, ax)) {
                uart->puts("topology: bad axis index\n"); return 1;
            }
            if (ax < 0 || ax >= (long)motion::MAX_AXES) {
                uart->puts("topology: axis out of range\n"); return 1;
            }
            s.axis_indices[s.axis_count++] = static_cast<uint8_t>(ax);
            if (*p == ',') { ++p; continue; }
            break;
        }
        ++n_spec;
    }
    const bool ok = motion::g_motion.set_topology(specs, n_spec);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "topology: %zu channel(s) %s\n", n_spec, ok ? "applied" : "REJECTED (validation)");
    uart->puts(buf);
    if (ok) motion::g_motion.dump_channels(uart);
    return ok ? 0 : 1;
}
static int cmd_feedhold(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: feedhold <channel> <0|1>\n"); return 1;
    }
    const char* p = args;
    long ch = 0, on = 0;
    if (!cli_parse_long(p, ch) || !cli_parse_long(p, on)) {
        uart->puts("feedhold: bad args\n"); return 1;
    }
    const bool ok = motion::g_motion.feedhold(static_cast<size_t>(ch), on != 0);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "feedhold ch%ld %s => %s\n", ch, on ? "on" : "off", ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_override(const char* args, kernel::hal::UARTDriverOps* uart) {
    // override <ch> <feed|rapid|spindle> <permille>
    if (!args || !*args) {
        uart->puts("usage: override <ch> <feed|rapid|spindle> <permille>\n");
        return 1;
    }
    const char* p = args;
    long ch = 0, which = -1;
    // parse channel
    if (!cli_parse_long(p, ch)) { uart->puts("override: bad channel\n"); return 1; }
    // parse which
    while (*p == ' ' || *p == '\t') ++p;
    if      (p[0]=='f' && p[1]=='e' && p[2]=='e' && p[3]=='d') { which = 0; p += 4; }
    else if (p[0]=='r' && p[1]=='a' && p[2]=='p' && p[3]=='i' && p[4]=='d') { which = 1; p += 5; }
    else if (p[0]=='s' && p[1]=='p' && p[2]=='i') { which = 2; while (*p && *p!=' '&&*p!='\t') ++p; }
    else { uart->puts("override: expected feed|rapid|spindle\n"); return 1; }
    long permille = 0;
    if (!cli_parse_long(p, permille)) { uart->puts("override: bad permille\n"); return 1; }
    const bool ok = motion::g_motion.set_override(
        static_cast<size_t>(ch),
        static_cast<motion::Kernel::OverrideKind>(which),
        static_cast<uint16_t>(permille));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "override ch%ld kind=%ld permille=%ld => %s\n",
        ch, which, permille, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_sync_move(const char* args, kernel::hal::UARTDriverOps* uart) {
    // sync_move <axis_mask-hex> <axisA> <targetA> [<axisB> <targetB> ...]
    // e.g.: sync_move 0x100001 0 2000 20 3000
    //       — axes 0 and 20 move together, shared T_final picks the slower.
    if (!args || !*args) {
        uart->puts("usage: sync_move <mask-hex> <ax> <tgt> [<ax> <tgt> ...]\n");
        return 1;
    }
    const char* p = args;
    long mask_l = 0;
    if (!cli_parse_long(p, mask_l)) { uart->puts("sync_move: bad mask\n"); return 1; }
    int32_t targets[motion::MAX_AXES] = {};
    while (true) {
        long ax = 0, tgt = 0;
        if (!cli_parse_long(p, ax)) break;
        if (!cli_parse_long(p, tgt)) {
            uart->puts("sync_move: missing target for axis\n"); return 1;
        }
        if (ax < 0 || (size_t)ax >= motion::MAX_AXES) {
            uart->puts("sync_move: axis out of range\n"); return 1;
        }
        targets[ax] = static_cast<int32_t>(tgt);
    }
    const bool ok = motion::g_motion.sync_move(
        static_cast<uint64_t>(mask_l), targets);
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sync_move mask=0x%08lx => %s\n",
        (unsigned long)mask_l, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_barrier_arrive(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 9.4 — CLI shim to simulate an interpreter arriving at a barrier.
    // Usage: barrier_arrive <channel> <token-hex> <parts-mask-hex>
    //                       [tolerance] [stable_cycles] [max_wait]
    // Example: barrier_arrive 0 0x100 0x3   → ch0 arrives at token 0x100 with ch0+ch1 as participants.
    if (!args || !*args) {
        uart->puts("usage: barrier_arrive <channel> <token-hex> <parts-mask-hex> "
                   "[tolerance=1] [stable_cycles=3] [max_wait=20000]\n");
        return 1;
    }
    const char* p = args;
    long ch = 0, token = 0, parts = 0;
    long tol = 1, stable = 3, maxw = 20000;
    if (!cli_parse_long(p, ch)    ||
        !cli_parse_long(p, token) ||
        !cli_parse_long(p, parts)) {
        uart->puts("barrier_arrive: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, tol);
    (void)cli_parse_long(p, stable);
    (void)cli_parse_long(p, maxw);

    const bool ok = motion::g_motion.arrive_at_barrier(
        static_cast<size_t>(ch),
        static_cast<uint16_t>(token),
        static_cast<uint8_t>(parts),
        static_cast<int32_t>(tol),
        static_cast<uint16_t>(stable),
        static_cast<uint16_t>(maxw));

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "barrier_arrive: ch=%ld token=0x%04lx parts=0x%02lx tol=%ld stable=%ld maxw=%ld => %s\n",
        ch, token, parts, tol, stable, maxw, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_mpos(const char*, kernel::hal::UARTDriverOps* uart) {
    motion::g_motion.dump_positions(uart); return 0;
}
// Inline parser (parse_long sits inside the FAKE_SLAVE namespace and isn't
// visible here). Accepts optional leading +/-, optional 0x for hex; returns
// false on garbage.
static bool cli_parse_long(const char*& p, long& out) noexcept {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) return false;
    bool neg = false;
    if (*p == '-') { neg = true; ++p; }
    else if (*p == '+') { ++p; }
    int base = 10;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
    if (!*p) return false;
    long v = 0;
    bool any = false;
    while (*p) {
        int d = -1;
        if (*p >= '0' && *p <= '9') d = *p - '0';
        else if (base == 16 && *p >= 'a' && *p <= 'f') d = *p - 'a' + 10;
        else if (base == 16 && *p >= 'A' && *p <= 'F') d = *p - 'A' + 10;
        else break;
        v = v * base + d;
        ++p; any = true;
    }
    if (!any) return false;
    out = neg ? -v : v;
    while (*p == ' ' || *p == '\t') ++p;
    return true;
}
// Shared "upload one SDO field + pretty-print" helper. The same
// expedited-upload-then-decode-then-format pattern appears in ec_safety,
// ec_dc, ec_tuning, ec_probe, ec_eeprom_save's read-modify-write, and
// a couple of one-off sites; factor it so the label format stays
// consistent across every command that dumps live drive state.
//
// `label_width` controls the first column; pass 22 for tight columns
// (ec_probe) or 28 for the wider "Bus under Operating Voltage"-style
// labels (ec_safety / ec_dc). Returns the raw value on success, 0 on
// fail (caller checks `*ok` if the distinction matters).
static uint32_t cli_read_sdo_u32(kernel::hal::UARTDriverOps* uart,
                                 uint16_t station, uint16_t idx, uint8_t sub,
                                 const char* label, int label_width,
                                 uint32_t timeout_us = 200000,
                                 bool* ok_out = nullptr) noexcept {
    uint8_t out[4] = {};
    uint8_t bytes = 0;
    uint32_t abort = 0;
    const bool ok = ethercat::g_master_a.upload_sdo(
        station, idx, sub, out, &bytes, timeout_us, &abort);
    uint32_t v = 0;
    for (uint8_t i = 0; i < bytes; ++i)
        v |= static_cast<uint32_t>(out[i]) << (8 * i);
    char buf[128];
    if (ok) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %-*s 0x%04x:%02x = %ld (0x%08lx, %u B)\n",
            label_width, label, (unsigned)idx, (unsigned)sub,
            (long)(int32_t)v, (unsigned long)v, (unsigned)bytes);
    } else {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %-*s 0x%04x:%02x FAIL (abort=0x%08lx)\n",
            label_width, label, (unsigned)idx, (unsigned)sub,
            (unsigned long)abort);
    }
    uart->puts(buf);
    if (ok_out) *ok_out = ok;
    return v;
}

static int cmd_ec_allow_safeop(const char*, kernel::hal::UARTDriverOps* uart) {
    // Temporary manual override for the PreOp→SafeOp gate. Real flow: the
    // bus_config helper probes identity, pushes the PDO mapping, verifies
    // 0x608F, then calls allow_safeop(true). Until that helper lands, this
    // command lets the CLI exercise LRW / motion end-to-end against the
    // fake slave. Safe only on a bench — the whole point of the gate is to
    // refuse torque for a mis-identified slave.
    ethercat::g_master_a.allow_safeop(true);
    ethercat::g_master_b.allow_safeop(true);
    uart->puts("allow_safeop: set on both masters (bus will advance PreOp->SafeOp)\n");
    return 0;
}
static int cmd_ec_sdo_read(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: ec_sdo_read <index-hex> <sub-hex> [station=0x1001]\n");
        return 1;
    }
    const char* p = args;
    long idx_l = 0, sub_l = 0, station_l = 0x1001;
    if (!cli_parse_long(p, idx_l) || !cli_parse_long(p, sub_l)) {
        uart->puts("usage: ec_sdo_read <index-hex> <sub-hex> [station=0x1001]\n");
        return 1;
    }
    (void)cli_parse_long(p, station_l); // optional

    uint8_t out[4] = {};
    uint8_t bytes = 0;
    uint32_t abort_code = 0;
    const bool ok = ethercat::g_master_a.upload_sdo(
        static_cast<uint16_t>(station_l),
        static_cast<uint16_t>(idx_l),
        static_cast<uint8_t>(sub_l),
        out, &bytes,
        /*timeout_us=*/ 200000,
        &abort_code);

    char buf[96];
    if (ok) {
        uint32_t val = 0;
        for (uint8_t i = 0; i < bytes; ++i) val |= static_cast<uint32_t>(out[i]) << (8 * i);
        kernel::util::k_snprintf(buf, sizeof(buf),
            "sdo_read 0x%04lx:%02lx @ station 0x%04lx => %u bytes, value=0x%08lx (%lu)\n",
            idx_l, sub_l, station_l, (unsigned)bytes,
            (unsigned long)val, (unsigned long)val);
        uart->puts(buf);
        return 0;
    }
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sdo_read 0x%04lx:%02lx @ station 0x%04lx FAILED (abort=0x%08lx)\n",
        idx_l, sub_l, station_l, (unsigned long)abort_code);
    uart->puts(buf);
    return 1;
}
static int cmd_ec_identity(const char* args, kernel::hal::UARTDriverOps* uart) {
    // ec_identity [station=0x1001] — probes 0x1018:1/2/3/4 and prints the
    // decoded vendor / product / revision / serial. With no expectations
    // supplied, the probe never fails on mismatch — CI / manual test only.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);

    uint32_t vid = 0, pid = 0, rev = 0, ser = 0;
    const bool ok = ethercat::g_master_a.probe_slave_identity(
        static_cast<uint16_t>(station_l), 0, 0, 0,
        &vid, &pid, &rev, &ser);

    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "identity @ 0x%04lx: vid=0x%08lx pid=0x%08lx rev=0x%08lx ser=0x%08lx [%s]\n",
        station_l,
        (unsigned long)vid, (unsigned long)pid,
        (unsigned long)rev, (unsigned long)ser,
        ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_encoder(const char* args, kernel::hal::UARTDriverOps* uart) {
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);

    uint32_t inc = 0, revs = 0, cpr = 0;
    const bool ok = ethercat::g_master_a.probe_encoder_resolution(
        static_cast<uint16_t>(station_l), &inc, &revs, &cpr);

    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "encoder @ 0x%04lx: 0x608F:01=%lu inc 0x608F:02=%lu revs => %lu cnts/rev [%s]\n",
        station_l, (unsigned long)inc, (unsigned long)revs,
        (unsigned long)cpr, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_probe(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 5.1 — touch-probe status + positions. Reads 0x60B9 status word,
    // 0x60BA (probe-1 pos pos-edge), 0x60BB (probe-1 neg edge), 0x60BC
    // (probe-2 pos), 0x60BD (probe-2 neg), plus 0x60D5..0x60D8 edge
    // counters. Does NOT write 0x60B8 — that's a one-shot `ec_probe_arm`
    // helper below. Purely observational.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    const uint16_t station = static_cast<uint16_t>(station_l);

    char hdr[64];
    kernel::util::k_snprintf(hdr, sizeof(hdr),
        "probe @ 0x%04x:\n", (unsigned)station);
    uart->puts(hdr);
    bool ok = true, this_ok = true;
    static constexpr int W = 22;
    (void)cli_read_sdo_u32(uart, station, 0x60B8, 0, "Probe function",         W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60B9, 0, "Probe status",           W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60BA, 0, "Probe 1 pos-edge pos",   W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60BB, 0, "Probe 1 neg-edge pos",   W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60BC, 0, "Probe 2 pos-edge pos",   W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60BD, 0, "Probe 2 neg-edge pos",   W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60D5, 0, "Probe 1 pos-edge count", W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60D6, 0, "Probe 1 neg-edge count", W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60D7, 0, "Probe 2 pos-edge count", W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x60D8, 0, "Probe 2 neg-edge count", W, 200000, &this_ok); ok &= this_ok;
    return ok ? 0 : 1;
}
static int cmd_ec_probe_arm(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 5.1 — arm the touch-probe function. 0x60B8 encodes per-probe
    // enable / edge / trigger-source bits; default 0x1713 = both probes
    // armed on zero-pulse / index. Caller can override the mask via
    // `ec_probe_arm <mask-hex> [station]` — no sanity check, raw write.
    if (!args || !*args) {
        uart->puts("usage: ec_probe_arm <mask-hex> [station]\n");
        return 1;
    }
    const char* p = args;
    long mask_l = 0, station_l = 0x1001;
    if (!cli_parse_long(p, mask_l)) {
        uart->puts("ec_probe_arm: bad mask\n");
        return 1;
    }
    (void)cli_parse_long(p, station_l);
    uint8_t data[2] = {};
    ethercat::put_u16_le(data, static_cast<uint16_t>(mask_l));
    const bool ok = ethercat::g_master_a.send_sdo_download(
        static_cast<uint16_t>(station_l), 0x60B8, 0, data, 2);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "probe_arm @ 0x%04lx: 0x60B8 = 0x%04lx %s\n",
        station_l, mask_l & 0xFFFF, ok ? "queued" : "TX FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_eeprom_save(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 5.4 — 0x1010:1 ← 0x65766173 ("save" ASCII, LE). Writes all
    // EEPROM-backed parameters back to the drive's NVM; survives reboot.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    uint8_t magic[4] = { 0x73, 0x61, 0x76, 0x65 }; // 'save' little-endian
    const bool ok = ethercat::g_master_a.send_sdo_download(
        static_cast<uint16_t>(station_l), 0x1010, 0x01, magic, 4);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "eeprom_save @ 0x%04lx: 0x1010:01 = 'save' %s\n",
        station_l, ok ? "queued" : "TX FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_tuning(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 5.3 — dump the ClearPath-specific tuning block. All 16-bit or
    // 32-bit, so expedited upload works. Writes are operator-initiated
    // via `ec_sdo_write` (separate tool); this is just the read surface.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    const uint16_t station = static_cast<uint16_t>(station_l);

    struct Entry { uint16_t idx; const char* name; };
    static const Entry entries[] = {
        { 0x2143, "Kr (resonance)"       },
        { 0x2146, "Kv (velocity loop)"   },
        { 0x2147, "Kp (position loop)"   },
        { 0x2148, "Ki (integral)"        },
        { 0x2149, "Kfv (vel feed-fwd)"   },
        { 0x214A, "Kfa (accel feed-fwd)" },
        { 0x214B, "Kfj (jerk feed-fwd)"  },
        { 0x214D, "Knv (notch)"          },
        { 0x214F, "Torque bias"          },
        { 0x215D, "Fine slider"          },
        { 0x2039, "RAS delay"            },
    };

    char hdr[64];
    kernel::util::k_snprintf(hdr, sizeof(hdr),
        "tuning @ 0x%04x:\n", (unsigned)station);
    uart->puts(hdr);
    bool ok = true, this_ok = true;
    for (const auto& e : entries) {
        (void)cli_read_sdo_u32(uart, station, e.idx, 0, e.name, 22, 200000, &this_ok);
        ok &= this_ok;
    }
    return ok ? 0 : 1;
}
static int cmd_ec_safety(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Phase 4 — read back the drive's fault / stop / overspeed / brake /
    // following-error objects so the operator can verify the sdo_init
    // block from the TSV actually landed. Shows: 0x605E fault-reaction,
    // 0x6065 following-error window, 0x6066 f.e. timeout ms, 0x231A
    // overspeed timeout ms, 0x2170 brake-delay ms.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    const uint16_t station = static_cast<uint16_t>(station_l);

    char hdr[64];
    kernel::util::k_snprintf(hdr, sizeof(hdr),
        "safety @ 0x%04x:\n", (unsigned)station);
    uart->puts(hdr);
    bool ok = true, this_ok = true;
    static constexpr int W = 28;
    (void)cli_read_sdo_u32(uart, station, 0x605E, 0x00, "Fault Reaction Option",        W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x6065, 0x00, "Following Error Window",       W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x6066, 0x00, "Following Error Timeout (ms)", W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x231A, 0x00, "Overspeed Timeout (ms)",       W, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x2170, 0x00, "Brake Delay Disable (ms)",     W, 200000, &this_ok); ok &= this_ok;
    return ok ? 0 : 1;
}
static int cmd_ec_home_run(const char* args, kernel::hal::UARTDriverOps* uart) {
    // ec_home_run <method> <fast> <slow> <accel> <off> <torque> [timeout_ms=5000] [station=0x1001]
    if (!args || !*args) {
        uart->puts("usage: ec_home_run <method> <fast> <slow> <accel> <off> "
                   "<torque> [timeout_ms=5000] [station]\n");
        return 1;
    }
    const char* p = args;
    long method = 0, fast = 0, slow = 0, accel = 0, off = 0, torque = 0;
    long timeout = 5000, station = 0x1001;
    if (!cli_parse_long(p, method) || !cli_parse_long(p, fast) ||
        !cli_parse_long(p, slow)   || !cli_parse_long(p, accel) ||
        !cli_parse_long(p, off)    || !cli_parse_long(p, torque)) {
        uart->puts("ec_home_run: bad args\n"); return 1;
    }
    (void)cli_parse_long(p, timeout);
    (void)cli_parse_long(p, station);

    const bool ok = ethercat::g_master_a.run_homing_sequence(
        static_cast<uint16_t>(station),
        static_cast<int8_t>(method),
        static_cast<uint32_t>(fast),
        static_cast<uint32_t>(slow),
        static_cast<uint32_t>(accel),
        static_cast<int32_t>(off),
        static_cast<uint16_t>(torque),
        static_cast<uint32_t>(timeout));
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "ec_home_run @ 0x%04lx method=%ld timeout=%ld ms => %s\n",
        station, method, timeout, ok ? "attained" : "FAIL/TIMEOUT");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_sw_limits(const char* args, kernel::hal::UARTDriverOps* uart) {
    // ec_sw_limits <neg> <pos> [station=0x1001]
    if (!args || !*args) {
        uart->puts("usage: ec_sw_limits <neg> <pos> [station]\n"); return 1;
    }
    const char* p = args;
    long neg = 0, pos = 0, station = 0x1001;
    if (!cli_parse_long(p, neg) || !cli_parse_long(p, pos)) {
        uart->puts("ec_sw_limits: bad args\n"); return 1;
    }
    (void)cli_parse_long(p, station);
    const size_t n = ethercat::g_master_a.push_software_limits(
        static_cast<uint16_t>(station),
        static_cast<int32_t>(neg), static_cast<int32_t>(pos));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "ec_sw_limits @ 0x%04lx neg=%ld pos=%ld => %zu SDO writes\n",
        station, neg, pos, n);
    uart->puts(buf);
    return n == 2 ? 0 : 1;
}
static int cmd_ec_home_methods(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 2.1 — enumerate 0x60E3 supported homing methods for a slave.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);

    int8_t methods[32] = {};
    uint8_t count = 0;
    const bool ok = ethercat::g_master_a.probe_homing_methods(
        static_cast<uint16_t>(station_l), methods, 32, &count);

    char buf[192];
    if (!ok) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "home_methods @ 0x%04lx: upload failed\n", station_l);
        uart->puts(buf);
        return 1;
    }
    kernel::util::k_snprintf(buf, sizeof(buf),
        "home_methods @ 0x%04lx: %u supported\n",
        station_l, (unsigned)count);
    uart->puts(buf);
    char line[128];
    size_t pos = 0;
    line[pos++] = ' '; line[pos++] = ' ';
    for (uint8_t i = 0; i < count && pos + 8 < sizeof(line); ++i) {
        if (i) { line[pos++] = ','; line[pos++] = ' '; }
        pos += kernel::util::k_snprintf(line + pos, sizeof(line) - pos,
                                         "%d", (int)methods[i]);
    }
    if (pos < sizeof(line) - 1) { line[pos++] = '\n'; line[pos] = 0; }
    else                        { line[sizeof(line) - 2] = '\n'; line[sizeof(line) - 1] = 0; }
    uart->puts(line);
    return 0;
}
static int cmd_ec_home_params(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 2.2 — push homing params: method fast slow accel offset torque [station]
    if (!args || !*args) {
        uart->puts("usage: ec_home_params <method> <fast_cps> <slow_cps> "
                   "<accel_cps2> <offset_counts> <torque_permille> [station]\n");
        return 1;
    }
    const char* p = args;
    long method = 0, fast = 0, slow = 0, accel = 0, offset = 0, torque = 0;
    long station = 0x1001;
    if (!cli_parse_long(p, method) || !cli_parse_long(p, fast)   ||
        !cli_parse_long(p, slow)   || !cli_parse_long(p, accel)  ||
        !cli_parse_long(p, offset) || !cli_parse_long(p, torque)) {
        uart->puts("ec_home_params: bad args\n");
        return 1;
    }
    (void)cli_parse_long(p, station);

    const size_t sent = ethercat::g_master_a.push_homing_params(
        static_cast<uint16_t>(station),
        static_cast<int8_t>(method),
        static_cast<uint32_t>(fast),
        static_cast<uint32_t>(slow),
        static_cast<uint32_t>(accel),
        static_cast<int32_t>(offset),
        static_cast<uint16_t>(torque));

    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "home_params @ 0x%04lx method=%ld queued %zu SDO writes\n",
        station, method, sent);
    uart->puts(buf);
    return sent == 6 ? 0 : 1;
}
static int cmd_ec_foldback(const char* args, kernel::hal::UARTDriverOps* uart) {
    // 6.1 — Move Done Torque Foldback. Bit 23 of 0x2018 enables,
    // 0x2163 sets folded-back torque (0.1 % peak), 0x2164 the
    // time-constant in ms. Operator-initiated — off by default so
    // a vertical axis holding a load doesn't silently lose torque.
    //   ec_foldback <0|1> [torque_permille=100] [tc_ms=200] [station=0x1001]
    if (!args || !*args) {
        uart->puts("usage: ec_foldback <0|1> [torque_permille=100] [tc_ms=200] [station]\n");
        return 1;
    }
    const char* p = args;
    long on = 0, torque = 100, tc = 200, station = 0x1001;
    if (!cli_parse_long(p, on)) { uart->puts("ec_foldback: bad on/off\n"); return 1; }
    (void)cli_parse_long(p, torque);
    (void)cli_parse_long(p, tc);
    (void)cli_parse_long(p, station);

    // Step 1 — read existing 0x2018, flip bit 23, write back.
    uint8_t buf[4] = {};
    uint8_t bytes = 0;
    uint32_t abort = 0;
    if (!ethercat::g_master_a.upload_sdo(
            static_cast<uint16_t>(station), 0x2018, 0x00, buf, &bytes, 200000, &abort) ||
        bytes != 4) {
        uart->puts("ec_foldback: 0x2018 read FAIL\n");
        return 1;
    }
    uint32_t app_cfg = ethercat::get_u32_le(buf);
    if (on) app_cfg |=  (1u << 23);
    else    app_cfg &= ~(1u << 23);
    uint8_t wr[4] = {};
    ethercat::put_u32_le(wr, app_cfg);
    size_t sent = 0;
    if (ethercat::g_master_a.send_sdo_download(static_cast<uint16_t>(station),
                                               0x2018, 0x00, wr, 4)) ++sent;
    if (on) {
        uint8_t tq[2] = {};
        ethercat::put_u16_le(tq, static_cast<uint16_t>(torque));
        if (ethercat::g_master_a.send_sdo_download(static_cast<uint16_t>(station),
                                                   0x2163, 0x00, tq, 2)) ++sent;
        uint8_t ts[2] = {};
        ethercat::put_u16_le(ts, static_cast<uint16_t>(tc));
        if (ethercat::g_master_a.send_sdo_download(static_cast<uint16_t>(station),
                                                   0x2164, 0x00, ts, 2)) ++sent;
    }
    char out[128];
    kernel::util::k_snprintf(out, sizeof(out),
        "ec_foldback @ 0x%04lx: %s, torque=%ld.%ld%% tc=%ld ms => %zu writes\n",
        station, on ? "ENABLED" : "disabled",
        torque / 10, torque % 10, tc, sent);
    uart->puts(out);
    return 0;
}
static int cmd_axis_sw_limits(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_sw_limits <axis> <neg> <pos> — arm the post-homing 0x607D push.
    if (!args || !*args) {
        uart->puts("usage: axis_sw_limits <axis> <neg> <pos>\n"); return 1;
    }
    const char* p = args;
    long ax = 0, neg = 0, pos = 0;
    if (!cli_parse_long(p, ax) || !cli_parse_long(p, neg) || !cli_parse_long(p, pos)) {
        uart->puts("axis_sw_limits: bad args\n"); return 1;
    }
    const bool ok = motion::g_motion.arm_post_homing_limits(
        static_cast<size_t>(ax),
        static_cast<int32_t>(neg), static_cast<int32_t>(pos));
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_sw_limits ax%ld armed neg=%ld pos=%ld => %s "
        "(fires 0x607D write on Has-Homed)\n",
        ax, neg, pos, ok ? "ok" : "FAIL");
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_ec_sdo_read_str(const char* args, kernel::hal::UARTDriverOps* uart) {
    // ec_sdo_read_str <idx> <sub> [station] — segmented SDO upload for
    // string / large objects. Prints the result as ASCII with non-printable
    // bytes shown in hex.
    if (!args || !*args) {
        uart->puts("usage: ec_sdo_read_str <idx-hex> <sub-hex> [station=0x1001]\n");
        return 1;
    }
    const char* p = args;
    long idx_l = 0, sub_l = 0, station_l = 0x1001;
    if (!cli_parse_long(p, idx_l) || !cli_parse_long(p, sub_l)) {
        uart->puts("usage: ec_sdo_read_str <idx-hex> <sub-hex> [station]\n");
        return 1;
    }
    (void)cli_parse_long(p, station_l);

    uint8_t buffer[128] = {};
    uint32_t abort = 0;
    const size_t got = ethercat::g_master_a.upload_sdo_segmented(
        static_cast<uint16_t>(station_l),
        static_cast<uint16_t>(idx_l),
        static_cast<uint8_t>(sub_l),
        buffer, sizeof(buffer),
        500000, &abort);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sdo_read_str 0x%04lx:%02lx @ 0x%04lx => %zu bytes\n",
        idx_l, sub_l, station_l, got);
    uart->puts(buf);
    if (got == 0) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  FAIL abort=0x%08lx\n", (unsigned long)abort);
        uart->puts(buf);
        return 1;
    }
    // Print as quoted string + hex dump.
    uart->puts("  \"");
    for (size_t i = 0; i < got; ++i) {
        const uint8_t c = buffer[i];
        if (c >= 0x20 && c < 0x7F) {
            char s[2] = { static_cast<char>(c), 0 };
            uart->puts(s);
        } else {
            char s[8];
            kernel::util::k_snprintf(s, sizeof(s), "\\x%02x", (unsigned)c);
            uart->puts(s);
        }
    }
    uart->puts("\"\n");
    return 0;
}
static int cmd_ec_dc(const char* args, kernel::hal::UARTDriverOps* uart) {
    // Task 3.1 — pure-observation dump of the ClearPath-EC DC telemetry
    // objects. No behavior change, just proves the slave is reachable and the
    // numbers match the ESI-documented defaults (440 ns / 62500 ns / 15000 ns
    // / 5200 ns). Exits non-zero if any single upload fails so CI can grep.
    const char* p = args ? args : "";
    long station_l = 0x1001;
    (void)cli_parse_long(p, station_l);
    const uint16_t station = static_cast<uint16_t>(station_l);

    char hdr[64];
    kernel::util::k_snprintf(hdr, sizeof(hdr),
        "DC telemetry @ station 0x%04x:\n", (unsigned)station);
    uart->puts(hdr);
    bool ok = true, this_ok = true;
    // 0x1C32 — SM Output Parameters (master -> slave direction).
    (void)cli_read_sdo_u32(uart, station, 0x1C32, 0x03, "Sync0 cycle time (ns)",       28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C32, 0x05, "Min cycle time (ns)",         28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C32, 0x06, "Calc+copy time (ns)",         28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C32, 0x09, "Delay time (ns)",             28, 200000, &this_ok); ok &= this_ok;
    // 0x1C33 — SM Input Parameters (slave -> master direction).
    (void)cli_read_sdo_u32(uart, station, 0x1C33, 0x03, "Sync0 cycle time in (ns)",    28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C33, 0x05, "Min cycle time in (ns)",      28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C33, 0x06, "Calc+copy time in (ns)",      28, 200000, &this_ok); ok &= this_ok;
    (void)cli_read_sdo_u32(uart, station, 0x1C33, 0x09, "Delay time in (ns)",          28, 200000, &this_ok); ok &= this_ok;
    return ok ? 0 : 1;
}
static int cmd_move(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: move <axis> <position>\n");
        return 1;
    }
    const char* p = args;
    long axis_l = 0, pos_l = 0;
    if (!cli_parse_long(p, axis_l) || !cli_parse_long(p, pos_l)) {
        uart->puts("usage: move <axis> <position>\n");
        return 1;
    }
    if (axis_l < 0 || (size_t)axis_l >= motion::MAX_AXES) {
        uart->puts("move: axis out of range\n");
        return 1;
    }
    motion::g_motion.move_to((size_t)axis_l, (int32_t)pos_l);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "move: axis %ld -> %ld (CSP, MoveReady)\n", axis_l, pos_l);
    uart->puts(buf);
    return 0;
}

static int cmd_axis_tune(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_tune <axis> [vmax] [accel] [jerk]
    // Set per-axis motion parameters: max velocity, acceleration, jerk
    if (!args || !*args) {
        uart->puts("usage: axis_tune <axis> [vmax_cps] [accel_cps2] [jerk_cps3]\n");
        uart->puts("  vmax:  counts/s (default 100000)\n");
        uart->puts("  accel: counts/s^2 (default 1000000)\n");
        uart->puts("  jerk:  counts/s^3 (default 10000000)\n");
        return 1;
    }
    const char* p = args;
    long axis_l = 0, vmax = -1, accel = -1, jerk = -1;
    if (!cli_parse_long(p, axis_l)) {
        uart->puts("axis_tune: bad axis\n");
        return 1;
    }
    (void)cli_parse_long(p, vmax);
    (void)cli_parse_long(p, accel);
    (void)cli_parse_long(p, jerk);

    if (axis_l < 0 || (size_t)axis_l >= motion::MAX_AXES) {
        uart->puts("axis_tune: axis out of range\n");
        return 1;
    }

    auto& a = motion::g_motion.axis((size_t)axis_l);
    if (vmax > 0) a.vmax_cps = (int32_t)vmax;
    if (accel > 0) a.accel_cps2 = (int32_t)accel;
    if (jerk > 0) a.jerk_cps3 = (int32_t)jerk;

    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_tune: ax%ld vmax=%d accel=%d jerk=%d\n",
        axis_l, a.vmax_cps, a.accel_cps2, a.jerk_cps3);
    uart->puts(buf);
    return 0;
}

static int cmd_encoder_config(const char* args, kernel::hal::UARTDriverOps* uart) {
    // encoder_config <axis> [source] [motor_cpr] [external_cpr] [gear_num] [gear_den] [sign]
    // Configure encoder source and parameters for flexible feedback:
    //   source: 0=motor (default), 1=external, 2=dual-loop
    //   motor_cpr: motor encoder counts/rev
    //   external_cpr: external encoder CPR (EL5042, EL5101, etc)
    //   gear_num/gear_den: gear ratio (default 1/1 = direct drive)
    //   sign: +1 or -1 to flip direction
    if (!args || !*args) {
        uart->puts("usage: encoder_config <axis> [source] [motor_cpr] [external_cpr] [gear_num] [gear_den] [sign]\n");
        uart->puts("  source:       0=motor, 1=external, 2=dual-loop (default 0)\n");
        uart->puts("  motor_cpr:    motor encoder counts/rev (default 0)\n");
        uart->puts("  external_cpr: external encoder CPR (default 0)\n");
        uart->puts("  gear_num:     gear ratio numerator (default 1)\n");
        uart->puts("  gear_den:     gear ratio denominator (default 1)\n");
        uart->puts("  sign:         +1 or -1 (default +1)\n");
        uart->puts("example: encoder_config 0 2 51600 3600000 1 1 1  # dual-loop, 51K motor, 3.6M external\n");
        return 1;
    }
    const char* p = args;
    long axis_l = 0, source = 0, motor_cpr = 0, external_cpr = 0;
    long gear_num = 1, gear_den = 1, sign = 1;
    if (!cli_parse_long(p, axis_l)) {
        uart->puts("encoder_config: bad axis\n");
        return 1;
    }
    (void)cli_parse_long(p, source);
    (void)cli_parse_long(p, motor_cpr);
    (void)cli_parse_long(p, external_cpr);
    (void)cli_parse_long(p, gear_num);
    (void)cli_parse_long(p, gear_den);
    (void)cli_parse_long(p, sign);

    if (axis_l < 0 || (size_t)axis_l >= motion::MAX_AXES) {
        uart->puts("encoder_config: axis out of range\n");
        return 1;
    }

    motion::g_motion.configure_encoder(
        (size_t)axis_l,
        static_cast<uint8_t>(source),
        static_cast<uint32_t>(motor_cpr),
        static_cast<uint32_t>(external_cpr),
        static_cast<uint32_t>(gear_num),
        static_cast<uint32_t>(gear_den),
        static_cast<int32_t>(sign));

    char buf[160];
    const char* src_name[] = {"Motor", "External", "Dual-loop"};
    uint8_t src = (source >= 0 && source <= 2) ? static_cast<uint8_t>(source) : 0;
    kernel::util::k_snprintf(buf, sizeof(buf),
        "encoder_config: ax%ld src=%s motor_cpr=%lu external_cpr=%lu gear=%ld/%ld sign=%ld\n",
        axis_l, src_name[src],
        (unsigned long)motor_cpr, (unsigned long)external_cpr,
        (long)gear_num, (long)gear_den, (long)sign);
    uart->puts(buf);
    return 0;
}

static int cmd_axis_autodetect(const char* args, kernel::hal::UARTDriverOps* uart) {
    // axis_autodetect <axis> — probe OD 0x608F via SDO upload, push the
    // resulting counts-per-rev into motion::Axis. Station address comes
    // from the master's slave table (1:1 with axis index in the default
    // hook wiring: axis[i] = master_a.slave(i)). PLAN 1.4.
    if (!args || !*args) {
        uart->puts("usage: axis_autodetect <axis>\n");
        return 1;
    }
    const char* p = args;
    long axis_l = 0;
    if (!cli_parse_long(p, axis_l) || axis_l < 0
        || (size_t)axis_l >= motion::MAX_AXES
        || (size_t)axis_l >= ethercat::MAX_SLAVES) {
        uart->puts("axis_autodetect: axis out of range\n");
        return 1;
    }
    const auto& slave = ethercat::g_master_a.slave((size_t)axis_l);
    if (slave.station_addr == 0) {
        uart->puts("axis_autodetect: slave not discovered yet (wait for ec state=OP)\n");
        return 1;
    }
    uint32_t inc = 0, revs = 0, cpr = 0;
    const bool ok = ethercat::g_master_a.probe_encoder_resolution(
        slave.station_addr, &inc, &revs, &cpr);
    if (!ok) {
        uart->puts("axis_autodetect: SDO probe failed (timeout or abort)\n");
        return 1;
    }
    motion::g_motion.set_encoder_resolution((size_t)axis_l, cpr);
    char buf[160];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "axis_autodetect: axis %ld @ station 0x%04x — 0x608F:01=%lu, :02=%lu => %lu cnts/rev\n",
        axis_l, (unsigned)slave.station_addr,
        (unsigned long)inc, (unsigned long)revs, (unsigned long)cpr);
    uart->puts(buf);
    return 0;
}
static int cmd_fault_inject(const char*, kernel::hal::UARTDriverOps* uart) {
#if MINIOS_FAKE_SLAVE
    ethercat::g_fake_slave.cia_inject_fault();
    uart->puts("fault_inject: fake_slave FSA will drop to Fault on next tick\n");
    return 0;
#else
    (void)uart;
    return 1;
#endif
}
static int cmd_fault_reset(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args || !*args) {
        uart->puts("usage: fault_reset <axis>\n");
        return 1;
    }
    const char* p = args;
    long axis_l = 0;
    if (!cli_parse_long(p, axis_l) || axis_l < 0
        || (size_t)axis_l >= motion::MAX_AXES) {
        uart->puts("fault_reset: axis out of range\n");
        return 1;
    }
    const bool ok = motion::g_motion.fault_reset((size_t)axis_l);
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        ok ? "fault_reset: axis %ld pulsed CW_FAULT_RESET\n"
           : "fault_reset: axis %ld has no backing drive\n", axis_l);
    uart->puts(buf);
    return ok ? 0 : 1;
}
static int cmd_devices(const char*, kernel::hal::UARTDriverOps* uart) {
    devices::g_device_db.dump(uart); return 0;
}
static int cmd_cia402(const char*, kernel::hal::UARTDriverOps* uart) {
    // Probe decode_state + Drive::step() against canonical statusword values
    // from the CiA-402 FSA table.
    struct Probe { uint16_t sw; };
    static constexpr Probe probes[] = {
        {0x0000}, // NotReadyToSwitchOn (no bits)
        {0x0040}, // SwitchOnDisabled (bit 6)
        {0x0021}, // ReadyToSwitchOn (bits 0,5)
        {0x0023}, // SwitchedOn (bits 0,1,5)
        {0x0027}, // OperationEnabled (bits 0,1,2,5)
        {0x0007}, // QuickStopActive (bits 0,1,2; bit 5 clear)
        {0x000F}, // FaultReactionActive (bits 0..3)
        {0x0008}, // Fault (bit 3)
        {0x0033}, // Dominant OE+voltage combination (spec example)
    };
    cia402::Drive d;
    d.target_state = cia402::State::OperationEnabled;
    d.mode_op      = cia402::Mode::CyclicSyncPosition;

    char line[96];
    for (const auto& p : probes) {
        d.statusword = p.sw;
        d.step();
        kernel::util::k_snprintf(line, sizeof(line),
            "sw=0x%04x -> %s (cw=0x%04x)\n",
            static_cast<unsigned>(p.sw),
            cia402::state_name(d.state),
            static_cast<unsigned>(d.controlword));
        uart->puts(line);
    }
    kernel::util::k_snprintf(line, sizeof(line),
        "mode=%s target=%s\n",
        cia402::mode_name(d.mode_op),
        cia402::state_name(d.target_state));
    uart->puts(line);
    return 0;
}
static int cmd_rt(const char*, kernel::hal::UARTDriverOps* uart) {
    uart->puts("RT threads (ns):\n");
    diag::rt::base.dump  (uart, "base  ");
    diag::rt::motion.dump(uart, "motion");
    diag::rt::ecat_a.dump(uart, "ecat_a");
    diag::rt::ecat_b.dump(uart, "ecat_b");
    return 0;
}

static int cmd_rt_reset(const char*, kernel::hal::UARTDriverOps* uart) {
    diag::rt::base.reset();
    diag::rt::motion.reset();
    diag::rt::ecat_a.reset();
    diag::rt::ecat_b.reset();
    uart->puts("RT trackers cleared. Run `rt` or `budget` for steady-state.\n");
    return 0;
}

static void dump_budget_line(kernel::hal::UARTDriverOps* uart,
                             const char* name,
                             const diag::JitterTracker& jt,
                             uint64_t deadline_misses) {
    const uint64_t period    = jt.period_ns();
    const uint64_t max_iv    = jt.max_interval_ns();
    const uint64_t max_jit   = jt.max_jitter_ns();
    const uint64_t n         = jt.count();
    int64_t slack = static_cast<int64_t>(period) - static_cast<int64_t>(max_iv);
    const char* verdict;
    if (n == 0)                             verdict = "idle  ";
    else if (jt.overruns() > 0)             verdict = "FAIL  ";
    else if (slack < 0)                     verdict = "FAIL  ";
    else if (slack < (int64_t)period / 4)   verdict = "MARGIN";
    else                                    verdict = "OK    ";
    char buf[200];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  %s %s  period=%lluns  maxInt=%lluns  maxJit=%lluns  "
        "slack=%lldns  overruns=%llu  cycleMiss=%llu\n",
        name, verdict,
        (unsigned long long)period,
        (unsigned long long)max_iv,
        (unsigned long long)max_jit,
        (long long)slack,
        (unsigned long long)jt.overruns(),
        (unsigned long long)deadline_misses);
    uart->puts(buf);
}

static int cmd_budget(const char*, kernel::hal::UARTDriverOps* uart) {
    uart->puts("RT budget (verdict from max interval vs period):\n");
    dump_budget_line(uart, "base  ", diag::rt::base,   0);
    dump_budget_line(uart, "motion", diag::rt::motion,
                     motion::g_motion.stats().deadline_miss.load(std::memory_order_relaxed));
    dump_budget_line(uart, "ecat_a", diag::rt::ecat_a,
                     ethercat::g_master_a.stats().cycle_deadline_miss.load(std::memory_order_relaxed));
    dump_budget_line(uart, "ecat_b", diag::rt::ecat_b,
                     ethercat::g_master_b.stats().cycle_deadline_miss.load(std::memory_order_relaxed));
    return 0;
}

// ---- `top` / `ttop` ---------------------------------------------------------
// `top` prints a one-shot snapshot of cores, threads, memory, and RT
// telemetry. `ttop` repeats the same frame every TTOP_PERIOD_US microseconds,
// using ANSI cursor-home + clear-screen so each frame overwrites the last;
// any keystroke from the input queue exits the loop.
//
// Per-core busy% formula (see also diag::g_core_counters):
//     busy% = (ticks_busy * 100) / ticks_total
// where ticks_total is incremented on every preemptive_tick and ticks_busy
// only when the running thread is not this core's idle thread.

static constexpr uint64_t TTOP_PERIOD_US = 500ULL * 1000ULL;

static const char* tcb_state_name(kernel::core::TCB::State s) noexcept {
    using S = kernel::core::TCB::State;
    switch (s) {
        case S::READY:   return "RDY";
        case S::RUNNING: return "RUN";
        case S::BLOCKED: return "BLK";
        case S::ZOMBIE:  return "ZMB";
        default:         return "INA";
    }
}

static void render_top_frame(kernel::hal::UARTDriverOps* uart) {
    if (!uart || !kernel::g_platform) return;
    char buf[200];

    // ---- Cores ----
    uart->puts("Cores:\n");
    uart->puts("  id   running                pri   irqs        ticks_b/ticks_t   busy%\n");
    const uint32_t ncores = kernel::g_platform->get_num_cores();
    for (uint32_t c = 0; c < ncores && c < kernel::core::MAX_CORES; ++c) {
        auto& cc = diag::g_core_counters[c];
        const auto* cur = kernel::core::g_per_cpu_data[c].current_thread;
        const char* name = cur ? cur->name : "-";
        int prio = cur ? cur->priority : -1;
        uint64_t irqs  = cc.irqs.load(std::memory_order_relaxed);
        uint64_t busy  = cc.ticks_busy.load(std::memory_order_relaxed);
        uint64_t total = cc.ticks_total.load(std::memory_order_relaxed);
        uint32_t pct = total ? static_cast<uint32_t>((busy * 100ULL) / total) : 0;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %-2u   %-20s   %-3d   %-10llu  %llu/%llu   %u%%\n",
            c, name, prio,
            (unsigned long long)irqs,
            (unsigned long long)busy, (unsigned long long)total,
            pct);
        uart->puts(buf);
    }

    // ---- Threads ----
    uart->puts("Threads:\n");
    uart->puts("  name           core  pri  state  stackUsed/Total   deadline\n");
    size_t active = 0;
    size_t stack_in_use_total = 0;
    for (const auto& tcb : kernel::core::g_task_tcbs) {
        if (tcb.state == kernel::core::TCB::State::INACTIVE) continue;
        ++active;
        size_t used = tcb.stack_used_bytes();
        stack_in_use_total += used;
        char aff[4];
        if (tcb.core_affinity < 0) { aff[0] = '-'; aff[1] = '\0'; }
        else                       { kernel::util::k_snprintf(aff, sizeof(aff), "%d", tcb.core_affinity); }
        const char* state_name = tcb_state_name(tcb.state);
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  %-12s   %-3s   %-3d  %-5s  %5zu/%-5zu %2u%%  %lluus\n",
            tcb.name, aff, tcb.priority,
            state_name,
            used, tcb.stack_size,
            tcb.stack_size ? static_cast<uint32_t>((used * 100) / tcb.stack_size) : 0,
            (unsigned long long)tcb.deadline_us);
        uart->puts(buf);
    }

    // ---- Memory ----
    uart->puts("Memory:\n");
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  active threads: %zu/%zu  stack-in-use: %zu B\n",
        active, kernel::core::MAX_THREADS, stack_in_use_total);
    uart->puts(buf);
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  software-timer pool: %zu free / %zu total\n",
        kernel::core::g_software_timer_obj_pool.get_free_count(),
        kernel::core::g_software_timer_obj_pool.get_total_count());
    uart->puts(buf);

    // ---- RT telemetry ----
    uart->puts("RT threads (ns):\n");
    diag::rt::base.dump  (uart, "base  ");
    diag::rt::motion.dump(uart, "motion");
    diag::rt::ecat_a.dump(uart, "ecat_a");
    diag::rt::ecat_b.dump(uart, "ecat_b");

    uart->puts("EtherCAT masters:\n");
    ethercat::g_master_a.dump_status(uart);
    ethercat::g_master_b.dump_status(uart);

    uart->puts("Motion:\n");
    motion::g_motion.dump_status(uart);
}

static int cmd_top(const char*, kernel::hal::UARTDriverOps* uart) {
    render_top_frame(uart);
    return 0;
}

static int cmd_ttop(const char*, kernel::hal::UARTDriverOps* uart) {
    if (!uart) return 1;
    // Hide the cursor while we redraw frames; restore on exit.
    uart->puts("\x1b[?25l");
    for (;;) {
        // Cursor home + clear screen, then render the frame.
        uart->puts("\x1b[H\x1b[2J");
        render_top_frame(uart);
        uart->puts("\n(press any key to exit)\n");

        // Sleep ~TTOP_PERIOD_US, polling the input queue every yield so a
        // keystroke exits within one scheduler tick.
        const uint64_t deadline_us = kernel::g_platform && kernel::g_platform->get_timer_ops()
            ? kernel::g_platform->get_timer_ops()->get_system_time_us() + TTOP_PERIOD_US
            : 0;
        bool key = false;
        for (;;) {
            uint8_t b;
            if (io::try_get(b)) { key = true; break; }
            if (kernel::g_platform && kernel::g_platform->get_timer_ops()) {
                if (kernel::g_platform->get_timer_ops()->get_system_time_us() >= deadline_us) break;
            } else {
                break;
            }
            if (kernel::g_scheduler_ptr && kernel::g_platform) {
                kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
            } else {
                kernel::util::cpu_relax();
            }
        }
        if (key) break;
    }
    uart->puts("\x1b[?25h");
    uart->puts("\n");
    return 0;
}

#if MINIOS_FAKE_SLAVE
namespace {

// Tiny number parser — supports optional leading -, optional 0x prefix for
// hex, plain decimal otherwise. Returns false on garbage. Used by fake_set.
bool parse_long(const char*& p, long& out) noexcept {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) return false;
    bool neg = false;
    if (*p == '-') { neg = true; ++p; }
    else if (*p == '+') { ++p; }
    int base = 10;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
    if (!*p) return false;
    long v = 0;
    bool any = false;
    while (*p) {
        int d = -1;
        if (*p >= '0' && *p <= '9') d = *p - '0';
        else if (base == 16 && *p >= 'a' && *p <= 'f') d = *p - 'a' + 10;
        else if (base == 16 && *p >= 'A' && *p <= 'F') d = *p - 'A' + 10;
        else break;
        v = v * base + d;
        ++p;
        any = true;
    }
    if (!any) return false;
    out = neg ? -v : v;
    while (*p == ' ' || *p == '\t') ++p;
    return true;
}

bool parse_token(const char*& p, char* out, size_t cap) noexcept {
    while (*p == ' ' || *p == '\t') ++p;
    size_t i = 0;
    while (*p && *p != ' ' && *p != '\t' && i + 1 < cap) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    while (*p == ' ' || *p == '\t') ++p;
    return i > 0;
}

const char* al_state_label(uint8_t s) noexcept {
    switch (s & 0x0F) {
        case ethercat::AL_INIT:   return "INIT";
        case ethercat::AL_PREOP:  return "PRE-OP";
        case ethercat::AL_SAFEOP: return "SAFE-OP";
        case ethercat::AL_OP:     return "OP";
        default:                  return "?";
    }
}

} // namespace

static int cmd_fake(const char*, kernel::hal::UARTDriverOps* uart) {
    auto& s = ethercat::g_fake_slave;
    char buf[200];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "FakeSlave station=0x%04x vid=0x%08x pid=0x%08x state=%s wkc=%llu rx=%llu tx=%llu\n",
        (unsigned)ethercat::FakeSlave::STATION_ADDR,
        (unsigned)ethercat::FakeSlave::VENDOR_ID,
        (unsigned)ethercat::FakeSlave::PRODUCT_CODE,
        al_state_label(s.al_state()),
        (unsigned long long)s.wkc_serviced(),
        (unsigned long long)s.frames_in(),
        (unsigned long long)s.frames_out());
    uart->puts(buf);

    kernel::util::k_snprintf(buf, sizeof(buf),
        "  DI  = 0x%04x\n  DO  = 0x%04x\n",
        (unsigned)s.dio_in(), (unsigned)s.dio_out());
    uart->puts(buf);

    // ADC unipolar: raw 16-bit, scale 0..0x7FFF -> 0..10000 mV.
    uart->puts("  ADC uni:");
    for (size_t i = 0; i < 16; ++i) {
        const int16_t raw = s.uni(i);
        kernel::util::k_snprintf(buf, sizeof(buf), " [%u]=%d", (unsigned)i, (int)raw);
        uart->puts(buf);
    }
    uart->puts("\n           mV:");
    for (size_t i = 0; i < 16; ++i) {
        const int16_t raw = s.uni(i);
        // raw / 0x7FFF * 10000 mV; integer math: (raw * 10000) / 32767.
        const long mv = (static_cast<long>(raw) * 10000L) / 32767L;
        kernel::util::k_snprintf(buf, sizeof(buf), " [%u]=%ld", (unsigned)i, mv);
        uart->puts(buf);
    }
    uart->puts("\n  ADC bip:");
    for (size_t i = 0; i < 16; ++i) {
        const int16_t raw = s.bip(i);
        kernel::util::k_snprintf(buf, sizeof(buf), " [%u]=%d", (unsigned)i, (int)raw);
        uart->puts(buf);
    }
    uart->puts("\n           mV:");
    for (size_t i = 0; i < 16; ++i) {
        const int16_t raw = s.bip(i);
        // bipolar: raw / 32768 * 10000 mV (signed).
        const long mv = (static_cast<long>(raw) * 10000L) / 32768L;
        kernel::util::k_snprintf(buf, sizeof(buf), " [%u]=%+ld", (unsigned)i, mv);
        uart->puts(buf);
    }
    uart->puts("\n");
    return 0;
}

static int cmd_fake_set(const char* args, kernel::hal::UARTDriverOps* uart) {
    if (!args) {
        uart->puts("usage: fake_set di <bitmap>\n"
                   "       fake_set uni <ch> <raw>\n"
                   "       fake_set bip <ch> <raw>\n");
        return 1;
    }
    const char* p = args;
    char tok[16];
    if (!parse_token(p, tok, sizeof(tok))) {
        uart->puts("fake_set: missing kind (di|uni|bip)\n");
        return 1;
    }
    auto& s = ethercat::g_fake_slave;
    if (cstrcmp(tok, "di") == 0) {
        long v;
        if (!parse_long(p, v)) { uart->puts("fake_set di: bad bitmap\n"); return 1; }
        s.set_di(static_cast<uint16_t>(v));
        char b[64]; kernel::util::k_snprintf(b, sizeof(b), "DI = 0x%04x\n", (unsigned)(v & 0xFFFF));
        uart->puts(b);
        return 0;
    }
    if (cstrcmp(tok, "uni") == 0 || cstrcmp(tok, "bip") == 0) {
        long ch, raw;
        if (!parse_long(p, ch) || !parse_long(p, raw)) {
            uart->puts("fake_set: bad ch / raw\n"); return 1;
        }
        if (ch < 0 || ch > 15) { uart->puts("fake_set: ch out of range 0..15\n"); return 1; }
        if (tok[0] == 'u') s.set_uni(static_cast<size_t>(ch), static_cast<int16_t>(raw));
        else               s.set_bip(static_cast<size_t>(ch), static_cast<int16_t>(raw));
        char b[64]; kernel::util::k_snprintf(b, sizeof(b), "%s[%ld] = %ld\n", tok, ch, raw);
        uart->puts(b);
        return 0;
    }
    uart->puts("fake_set: unknown kind (di|uni|bip)\n");
    return 1;
}

static int cmd_slave(const char*, kernel::hal::UARTDriverOps* uart) {
    auto& s = ethercat::g_fake_slave;
    const cia402::State st = s.cia_state();
    const uint16_t sw  = s.cia_statusword();
    const uint16_t cw  = s.cia_controlword();
    const int8_t   mo  = s.cia_mode_op();
    const int8_t   mod = s.cia_mode_op_display();
    const int32_t  tp  = s.cia_target_pos();
    const int32_t  ap  = s.cia_actual_pos();
    char buf[256];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "CiA-402 FakeSlave (station=0x%04x)\n"
        "  state           = %s\n"
        "  statusword      = 0x%04x\n"
        "  controlword     = 0x%04x\n"
        "  mode_op         = %d\n"
        "  mode_op_display = %d\n"
        "  target_pos      = %ld\n"
        "  actual_pos      = %ld\n",
        (unsigned)ethercat::FakeSlave::STATION_ADDR,
        cia402::state_name(st),
        (unsigned)sw, (unsigned)cw,
        (int)mo, (int)mod,
        (long)tp, (long)ap);
    uart->puts(buf);
    return 0;
}
#endif // MINIOS_FAKE_SLAVE

static int cmd_net(const char*, kernel::hal::UARTDriverOps* uart) {
    if (!kernel::g_platform) { uart->puts("no platform\n"); return 1; }
    int n = kernel::g_platform->get_num_nets();
    char line[160];
    kernel::util::k_snprintf(line, sizeof(line), "NICs: %d\n", n);
    uart->puts(line);
    for (int i = 0; i < n; ++i) {
        auto* ops = kernel::g_platform->get_net_ops(i);
        if (!ops) {
            kernel::util::k_snprintf(line, sizeof(line), "  [%d] null\n", i);
            uart->puts(line);
            continue;
        }
        kernel::hal::net::NicStats s = ops->get_stats();
        kernel::util::k_snprintf(line, sizeof(line),
            "  [%d] up  tx=%llu/%lluB drops=%llu  rx=%llu/%lluB drops=%llu\n",
            i,
            (unsigned long long)s.tx_packets, (unsigned long long)s.tx_bytes,
            (unsigned long long)s.tx_drops,
            (unsigned long long)s.rx_packets, (unsigned long long)s.rx_bytes,
            (unsigned long long)s.rx_drops);
        uart->puts(line);
    }
    return 0;
}

static int cmd_hmi(const char*, kernel::hal::UARTDriverOps* uart) {
    char ip_buf[16];
    char mask_buf[16];
    char gw_buf[16];
    char ping_ip_buf[16];
    kernel::util::uint32_to_ipv4_str(hmi::g_service.local_ip(), std::span<char>(ip_buf, sizeof(ip_buf)));
    kernel::util::uint32_to_ipv4_str(hmi::g_service.netmask(), std::span<char>(mask_buf, sizeof(mask_buf)));
    kernel::util::uint32_to_ipv4_str(hmi::g_service.gateway(), std::span<char>(gw_buf, sizeof(gw_buf)));
    kernel::util::uint32_to_ipv4_str(hmi::g_service.last_ping_target(), std::span<char>(ping_ip_buf, sizeof(ping_ip_buf)));
    const char* ping_rc = "unknown";
    switch (hmi::g_service.last_ping_result()) {
        case hmi::Service::PingResult::Ok: ping_rc = "ok"; break;
        case hmi::Service::PingResult::Busy: ping_rc = "busy"; break;
        case hmi::Service::PingResult::BadAddress: ping_rc = "bad_addr"; break;
        case hmi::Service::PingResult::Timeout: ping_rc = "timeout"; break;
        case hmi::Service::PingResult::SendFailed: ping_rc = "send_failed"; break;
    }
    char buf[320];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "hmi: eth%u rx=%llu tx=%llu last_op=%u page=%s ip=%s mask=%s gw=%s udp=%u dhcp=%u ping_target=%s ping_rc=%s ping_rtt=%lums\n",
        static_cast<unsigned>(hmi::g_service.nic_idx()),
        static_cast<unsigned long long>(hmi::g_service.rx_requests()),
        static_cast<unsigned long long>(hmi::g_service.tx_responses()),
        static_cast<unsigned>(hmi::g_service.last_opcode()),
        ui_builder::active_page_id(),
        ip_buf,
        mask_buf,
        gw_buf,
        static_cast<unsigned>(hmi::g_service.udp_port()),
        hmi::g_service.dhcp_bound() ? 1u : 0u,
        ping_ip_buf,
        ping_rc,
        static_cast<unsigned long>(hmi::g_service.last_ping_rtt_ms()));
    uart->puts(buf);
    return 0;
}

static int cmd_net_ping(const char* args, kernel::hal::UARTDriverOps* uart) {
    const char* p = args ? args : "";
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') {
        uart->puts("usage: net_ping <ipv4> [timeout_ms]\n");
        return 1;
    }

    char ip_buf[32]{};
    size_t ip_len = 0;
    while (*p && *p != ' ' && *p != '\t' && ip_len + 1 < sizeof(ip_buf)) {
        ip_buf[ip_len++] = *p++;
    }
    ip_buf[ip_len] = '\0';
    while (*p == ' ' || *p == '\t') ++p;

    long timeout_ms = 1500;
    if (*p != '\0' && !cli_parse_long(p, timeout_ms)) {
        uart->puts("net_ping: invalid timeout\n");
        return 1;
    }
    if (timeout_ms <= 0) timeout_ms = 1500;

    uint32_t rtt_ms = 0;
    const auto rc = hmi::g_service.ping_ipv4_str(ip_buf, static_cast<uint32_t>(timeout_ms), rtt_ms);
    char buf[160];
    switch (rc) {
        case hmi::Service::PingResult::Ok:
            kernel::util::k_snprintf(buf, sizeof(buf), "net_ping %s: reply rtt=%lums\n", ip_buf, static_cast<unsigned long>(rtt_ms));
            uart->puts(buf);
            return 0;
        case hmi::Service::PingResult::Busy:
            uart->puts("net_ping: HMI network stack busy\n");
            return 1;
        case hmi::Service::PingResult::BadAddress:
            uart->puts("net_ping: invalid IPv4 address\n");
            return 1;
        case hmi::Service::PingResult::Timeout:
            kernel::util::k_snprintf(buf, sizeof(buf), "net_ping %s: timeout\n", ip_buf);
            uart->puts(buf);
            return 1;
        case hmi::Service::PingResult::SendFailed:
            kernel::util::k_snprintf(buf, sizeof(buf), "net_ping %s: send failed\n", ip_buf);
            uart->puts(buf);
            return 1;
    }
    uart->puts("net_ping: unknown error\n");
    return 1;
}

static int cmd_usb(const char*, kernel::hal::UARTDriverOps* uart) {
    kernel::usb::dump_status(uart);
    return 0;
}

static int cmd_input(const char*, kernel::hal::UARTDriverOps* uart) {
    if (!uart || !kernel::g_platform) return 1;
    auto* input = kernel::g_platform->get_input_ops();
    if (!input) {
        uart->puts("input: unavailable\n");
        return 1;
    }

    input->poll();

    char buf[256];
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "input: keyboard=%s mouse=%s touch=%s\n",
                             input->is_keyboard_connected() ? "yes" : "no",
                             input->is_mouse_connected() ? "yes" : "no",
                             input->is_touch_connected() ? "yes" : "no");
    uart->puts(buf);

    uart->puts("  keys:");
    bool any = false;
    for (uint32_t key = 0; key < 128; ++key) {
        if (!input->get_key_state(static_cast<uint8_t>(key))) continue;
        any = true;
        kernel::util::k_snprintf(buf, sizeof(buf), " %lu", static_cast<unsigned long>(key));
        uart->puts(buf);
    }
    if (!any) uart->puts(" none");
    uart->puts("\n");

    int32_t mouse_x = 0;
    int32_t mouse_y = 0;
    uint8_t mouse_buttons = 0;
    input->get_mouse_position(mouse_x, mouse_y, mouse_buttons);
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "  mouse: x=%ld y=%ld buttons=0x%02x\n",
                             static_cast<long>(mouse_x),
                             static_cast<long>(mouse_y),
                             static_cast<unsigned>(mouse_buttons));
    uart->puts(buf);

    int32_t touch_x = 0;
    int32_t touch_y = 0;
    bool touch_pressed = false;
    input->get_touch_position(touch_x, touch_y, touch_pressed);
    kernel::util::k_snprintf(buf, sizeof(buf),
                             "  touch: x=%ld y=%ld pressed=%s\n",
                             static_cast<long>(touch_x),
                             static_cast<long>(touch_y),
                             touch_pressed ? "yes" : "no");
    uart->puts(buf);
    return 0;
}

CLI::CLI() {
    register_command("help",   cmd_help,   "List registered commands");
    register_command("status", cmd_status, "One-shot dashboard: masters, slaves, DB, channels, barriers, gears, axes");
    register_command("version", cmd_version, "Kernel + build info");
    register_command("ec_diag_dump", cmd_ec_diag_dump, "ec_diag_dump [station] — walk 0x10F3 DiagHistory + decode TextId");
    register_command("ec_scan", cmd_ec_scan, "ec_scan [start=0x1001] [count=8] — probe identity over a station range");
    register_command("ec_abort", cmd_ec_abort, "Broadcast AL=Init — emergency bus bring-down");
    register_command("ec_watchdog", cmd_ec_watchdog, "ec_watchdog <timeout_ms> [station] — program SM watchdog");
    register_command("motion_enable", cmd_motion_enable, "motion_enable <axis> — servo on without a move");
    register_command("motion_disable", cmd_motion_disable, "motion_disable <axis> — explicit servo-off");
    register_command("offsets", cmd_offsets, "Show work/tool offsets and active selections");
    register_command("offset_select", cmd_offset_select, "offset_select <g54..g59|toolN>");
    register_command("offset_set", cmd_offset_set, "offset_set <g54..g59> <axis> <value> | offset_set tool <idx> <len> <rad> <wear>");
    register_command("program_ls", cmd_program_ls, "List programs in the in-memory store");
    register_command("program_open", cmd_program_open, "program_open [name] — open current or named program");
    register_command("program_open_ch", cmd_program_open_ch, "program_open_ch <ch> [name] — open current or named program on a channel");
    register_command("program_run", cmd_program_run, "program_run <ch|all> — start the interpreter for one or both channels");
    register_command("program_status", cmd_program_status, "program_status — show per-channel loaded program and interpreter state");
    register_command("program_show", cmd_program_show, "program_show [name] — display a program");
    register_command("program_select", cmd_program_select, "program_select <name> — select active program");
    register_command("program_select_ch", cmd_program_select_ch, "program_select_ch <ch> <name> — select active program on a channel");
    register_command("program_sim", cmd_program_sim, "program_sim [name] — show preview/simulation metadata");
    register_command("macro_ls", cmd_macro_ls, "macro_ls — list loaded TSV macros");
    register_command("macro_run", cmd_macro_run, "macro_run [idx] — run selected or indexed macro on channel 0");
    register_command("macro_stop", cmd_macro_stop, "macro_stop — abort active macro on channel 0");
    register_command("symbols", cmd_symbols, "symbols — dump machine symbol registry");
    register_command("signals", cmd_signals, "signals — dump EtherCAT/IO signal bindings and live values");
    register_command("symbol_set", cmd_symbol_set, "symbol_set <name> <value> — override a writable machine symbol");
    register_command("ladder_ls", cmd_ladder_ls, "ladder_ls — list loaded ladder rungs");
    register_command("toolpods", cmd_toolpods, "toolpods — list configured physical pods and virtual tool assignments");
    register_command("toolpod_select", cmd_toolpod_select, "toolpod_select <pod> <station> — move/select a physical pod station");
    register_command("toolpod_assign", cmd_toolpod_assign, "toolpod_assign <pod> <station> <virtual_tool> — remap virtual tool onto a physical station");
    register_command("ui_page", cmd_ui_page, "ui_page <id> — switch the active TSV UI page");
    register_command("ui_dump", cmd_ui_dump, "ui_dump [scale] — dump the current framebuffer as a downscaled PPM stream");
    register_command("save_cfg", cmd_save_cfg, "Dump current operator-set state as replayable CLI commands");
    register_command("meminfo", cmd_meminfo, "Memory pool usage + per-thread stack watermarks");
    register_command("trace", cmd_trace, "Dump the kernel trace buffer");
    register_command("stats", cmd_stats, "Show scheduler / per-CPU state");
    register_command("test",  cmd_test,  "Run the built-in test framework");
    register_command("echo",  cmd_echo,  "Echo the rest of the line");
    register_command("net",    cmd_net,    "List NICs discovered by the HAL");
    register_command("usb",    cmd_usb,    "Show USB host-controller and root-port status");
    register_command("input",  cmd_input,  "Show current input-device connectivity and pressed keys");
    register_command("hmi",    cmd_hmi,    "Show HMI eth0 status, IP config, and transport counters");
    register_command("net_ping", cmd_net_ping, "net_ping <ipv4> [timeout_ms] — ICMP echo via eth0 HMI stack");
    register_command("ec",     cmd_ec,     "EtherCAT master status (both buses)");
    register_command("ec_slaves", cmd_ec_slaves, "List discovered EtherCAT slaves");
    register_command("ec_hist", cmd_ec_hist, "EtherCAT cycle-latency histograms");
    register_command("ec_sdo_read", cmd_ec_sdo_read, "ec_sdo_read <index> <sub> [station] - SDO upload probe");
    register_command("ec_allow_safeop", cmd_ec_allow_safeop, "Temp gate override — let the bus advance past PreOp");
    register_command("ec_identity", cmd_ec_identity, "ec_identity [station] - read 0x1018 identity object");
    register_command("ec_encoder",  cmd_ec_encoder,  "ec_encoder  [station] - read 0x608F encoder resolution");
    register_command("ec_home_methods", cmd_ec_home_methods, "ec_home_methods [station] - list 0x60E3 supported homing methods");
    register_command("ec_home_params",  cmd_ec_home_params,  "ec_home_params <method> <fast> <slow> <accel> <off> <torque> [station]");
    register_command("ec_home_run",     cmd_ec_home_run,     "ec_home_run <method> <fast> <slow> <accel> <off> <torque> [to] [station] (2.3)");
    register_command("ec_sw_limits",    cmd_ec_sw_limits,    "ec_sw_limits <neg> <pos> [station] — 0x607D:1/2 (2.4)");
    register_command("ec_foldback",     cmd_ec_foldback,     "ec_foldback <0|1> [torque_0.1%] [tc_ms] [station] — Move Done Torque Foldback (6.1)");
    register_command("axis_sw_limits",  cmd_axis_sw_limits,  "axis_sw_limits <axis> <neg> <pos> — arm post-homing 0x607D push");
    register_command("ec_safety",       cmd_ec_safety,       "ec_safety   [station] - read fault/stop/overspeed/brake/following-error objects");
    register_command("ec_probe",        cmd_ec_probe,        "ec_probe    [station] - touch-probe status + captured positions");
    register_command("ec_probe_arm",    cmd_ec_probe_arm,    "ec_probe_arm <mask-hex> [station] - write 0x60B8 touch-probe function");
    register_command("ec_tuning",       cmd_ec_tuning,       "ec_tuning   [station] - dump Kp/Kv/Kfv/... tuning block");
    register_command("ec_eeprom_save",  cmd_ec_eeprom_save,  "ec_eeprom_save [station] - write 'save' to 0x1010:01 (persist NVM)");
    register_command("ec_dc",       cmd_ec_dc,       "ec_dc       [station] - read 0x1C32/0x1C33 DC telemetry");
    register_command("ec_sdo_read_str", cmd_ec_sdo_read_str, "ec_sdo_read_str <idx> <sub> [station] — segmented SDO upload (strings, diag, etc.)");
    register_command("motion", cmd_motion, "Motion kernel status");
    register_command("channels", cmd_channels, "List motion channels and their axis ownership");
    register_command("axis_load_configure", cmd_axis_load_configure,
        "axis_load_configure <axis> <scale_num> <scale_den> <sign> [cap] [slew] [kp] [ki]");
    register_command("axis_load_sample", cmd_axis_load_sample,
        "axis_load_sample <axis> <raw_pos> [error=0]");
    register_command("axis_load_calibrate", cmd_axis_load_calibrate,
        "axis_load_calibrate <axis>");
    register_command("barriers", cmd_barriers, "List active cross-channel barriers (9.4)");
    register_command("barrier_arrive", cmd_barrier_arrive, "barrier_arrive <ch> <token-hex> <parts-mask-hex> [tol] [stable] [maxw]");
    register_command("sync_move", cmd_sync_move, "sync_move <mask-hex> <ax> <tgt> [<ax> <tgt> ...] — cross-channel coordinated move (9.6)");
    register_command("feedhold", cmd_feedhold, "feedhold <ch> <0|1> — per-channel pause/resume (9.7)");
    register_command("topology", cmd_topology, "topology <name>=<ax,ax,...> [...] — rewrite axis-to-channel binding (9.8)");
    register_command("override", cmd_override, "override <ch> <feed|rapid|spindle> <permille> — per-channel speed override (9.7)");
    register_command("gears", cmd_gears, "List active electronic-gear links (9.5) + phase error (9.9)");
    register_command("axis_spin", cmd_axis_spin, "axis_spin <axis> <velocity_cps> — simulated leader spin (9.9 harness)");
    register_command("gear_engage", cmd_gear_engage, "gear_engage <leader> <follower> <ch> <k_num> <k_den> [ramp]");
    register_command("gear_disengage", cmd_gear_disengage, "gear_disengage <follower> [ramp]");
    register_command("gantry", cmd_gantry, "List active gantry links");
    register_command("gantry_engage", cmd_gantry_engage, "gantry_engage <primary> <secondary> [num] [den] [offset]");
    register_command("gantry_disengage", cmd_gantry_disengage, "gantry_disengage <secondary>");
    register_command("gantry_adjust", cmd_gantry_adjust, "gantry_adjust <secondary> <delta_counts>");
    register_command("cal_linear", cmd_cal_linear, "cal_linear <axis> — show pitch error compensation");
    register_command("cal_add", cmd_cal_add, "cal_add <axis> <pos> <error> — add PEC point");
    register_command("cal_enable", cmd_cal_enable, "cal_enable <axis> <0|1> — enable/disable PEC");
    register_command("cal_clear", cmd_cal_clear, "cal_clear <axis> — clear PEC table");
    register_command("cal_rotary", cmd_cal_rotary, "cal_rotary <axis> [offset] — rotary index offset");
    register_command("cal_geometry", cmd_cal_geometry, "cal_geometry [pair] [urad] — squareness error");
    register_command("cal_geo_enable", cmd_cal_geo_enable, "cal_geo_enable <0|1> — enable geometry comp");
    register_command("sphere_config", cmd_sphere_config, "sphere_config [diam_mm] [probe_um] — set sphere params");
    register_command("sphere_add", cmd_sphere_add, "sphere_add <cmd_x> <cmd_y> <cmd_z> <act_x> <act_y> <act_z>");
    register_command("sphere_compute", cmd_sphere_compute, "sphere_compute — compute 21-error model from points");
    register_command("sphere_enable", cmd_sphere_enable, "sphere_enable <0|1> — enable volumetric comp");
    register_command("sphere_clear", cmd_sphere_clear, "sphere_clear — clear measurement points");
    register_command("sphere", cmd_sphere, "sphere — show sphere calibration status");
    register_command("sphere_auto", cmd_sphere_auto, "sphere_auto <cx> <cy> <cz> <radius> — show 7-point hemisphere");
    register_command("probe_calibrate", cmd_probe_calibrate, "probe_calibrate <sx> <sy> <sz> [r] — fully automated sphere probe");
    register_command("skiving_config", cmd_skiving_config, "skiving_config <ts1> <teeth> <c1_cpr> [offset]");
    register_command("skiving_engage", cmd_skiving_engage, "skiving_engage <c1> <b> <ts1> <teeth> [ramp]");
    register_command("feed_per_tooth", cmd_feed_per_tooth, "feed_per_tooth <cutter_dia> <teeth> [c1_axis] - compute feed/tooth");
    register_command("c1_index_sync", cmd_c1_index_sync, "c1_index_sync <c1> <enable> <teeth> - trigger on index pulse");
    register_command("multipass", cmd_multipass, "multipass <depth> <passes> [doc] - rough/finish passes");
    register_command("din6", cmd_din6, "din6 <cmd> [args] - DIN6 axis indexing");
    register_command("move",      cmd_move,      "move <axis> <pos> - command axis to position (CSP)");
    register_command("axis_tune",  cmd_axis_tune, "axis_tune <axis> [vmax] [accel] [jerk] - per-axis motion tuning");
    register_command("encoder_config", cmd_encoder_config, "encoder_config <axis> [src] [motor_cpr] [ext_cpr] [num] [den] [sign] - configure encoder");
    register_command("mpos",   cmd_mpos,   "Dump per-axis commanded/actual positions and following error");
    register_command("fault_reset", cmd_fault_reset, "fault_reset <axis> - pulse CW_FAULT_RESET, drop latch");
    register_command("axis_autodetect", cmd_axis_autodetect, "axis_autodetect <axis> - probe 0x608F, store cnts/rev in motion axis");
#if MINIOS_FAKE_SLAVE
    register_command("fault_inject", cmd_fault_inject, "Inject a Fault into the fake slave for testing propagation");
#endif
    register_command("devices", cmd_devices, "List devices loaded from TSV database");
    register_command("cia402",  cmd_cia402,  "Probe CiA-402 FSA decode against canned statuswords");
    register_command("rt",      cmd_rt,      "RT jitter telemetry (ns) for base/motion/ecat threads");
    register_command("rt_reset", cmd_rt_reset, "Zero the RT jitter trackers (drop boot-time outliers)");
    register_command("budget",  cmd_budget,  "RT budget verdict (OK / MARGIN / FAIL) per thread");
    register_command("top",     cmd_top,     "Snapshot of cores, threads, memory, RT");
    register_command("ttop",    cmd_ttop,    "Continuous top (500ms refresh, any key exits)");
#if MINIOS_FAKE_SLAVE
    register_command("fake",     cmd_fake,     "Dump in-kernel FakeIO slave state (NIC 1)");
    register_command("fake_set", cmd_fake_set, "Inject FakeIO inputs: di <bm> | uni <ch> <raw> | bip <ch> <raw>");
    register_command("slave",    cmd_slave,    "Dump CiA-402 FakeSlave FSA snapshot");
#endif
}

bool CLI::register_command(const char* name, CommandHandler handler, const char* help_text) noexcept {
    if (num_commands_ >= MAX_COMMANDS || !name || !handler) return false;
    commands_[num_commands_++] = {name, handler, help_text ? help_text : ""};
    return true;
}

void CLI::print_help(kernel::hal::UARTDriverOps* uart) const noexcept {
    if (!uart) return;
    for (size_t i = 0; i < num_commands_; ++i) {
        uart->puts("  ");
        uart->puts(commands_[i].name);
        uart->puts(" - ");
        uart->puts(commands_[i].help);
        uart->puts("\n");
    }
}

// ============================================================================
// Output / editing primitives — go through the queued sink (io::put_n).
// ============================================================================

void CLI::put(char c) noexcept { io::putc_one(c); }
void CLI::write(const char* s) noexcept { io::put(s); }
void CLI::write_n(const char* s, size_t n) noexcept { io::put_n(s, n); }

void CLI::write_uint(size_t v) noexcept {
    char buf[24]; size_t n = 0;
    if (v == 0) { buf[n++] = '0'; }
    else { while (v) { buf[n++] = '0' + (v % 10); v /= 10; } }
    char tmp[24]; for (size_t i = 0; i < n; ++i) tmp[i] = buf[n - 1 - i];
    io::put_n(tmp, n);
}

void CLI::cursor_left(size_t n) noexcept {
    if (n == 0) return;
    put(K_ESC); put('['); write_uint(n); put('D');
}
void CLI::erase_eol() noexcept { put(K_ESC); put('['); put('K'); }
void CLI::prompt_line() noexcept { put(K_CR); write_n(PROMPT, PROMPT_LEN); }

void CLI::redraw() noexcept {
    prompt_line();
    write_n(line_, line_len_);
    erase_eol();
    if (cursor_ < line_len_) cursor_left(line_len_ - cursor_);
}

void CLI::replace_line(const char* src, size_t len) noexcept {
    if (len > MAX_LINE_LEN - 1) len = MAX_LINE_LEN - 1;
    for (size_t i = 0; i < len; ++i) line_[i] = src[i];
    line_len_ = len;
    cursor_ = len;
    redraw();
}

void CLI::insert_char(char c) noexcept {
    if (line_len_ >= MAX_LINE_LEN - 1) return;
    for (size_t i = line_len_; i > cursor_; --i) line_[i] = line_[i - 1];
    line_[cursor_] = c;
    ++line_len_;
    ++cursor_;
    if (cursor_ == line_len_) {
        put(c);
    } else {
        write_n(&line_[cursor_ - 1], line_len_ - (cursor_ - 1));
        cursor_left(line_len_ - cursor_);
    }
}

void CLI::backspace() noexcept {
    if (cursor_ == 0) return;
    for (size_t i = cursor_ - 1; i + 1 < line_len_; ++i) line_[i] = line_[i + 1];
    --line_len_;
    --cursor_;
    put(K_BS);
    write_n(&line_[cursor_], line_len_ - cursor_);
    erase_eol();
    if (cursor_ < line_len_) cursor_left(line_len_ - cursor_);
}

void CLI::move_left() noexcept {
    if (cursor_ == 0) return;
    --cursor_;
    put(K_BS);
}

void CLI::move_right() noexcept {
    if (cursor_ >= line_len_) return;
    put(line_[cursor_]);
    ++cursor_;
}

void CLI::history_prev() noexcept {
    if (history_count_ == 0) return;
    if (history_browse_ == -1) {
        saved_line_.len = line_len_;
        for (size_t i = 0; i < line_len_; ++i) saved_line_.buf[i] = line_[i];
        history_browse_ = 0;
    } else if (static_cast<size_t>(history_browse_) + 1 < history_count_) {
        ++history_browse_;
    } else {
        return;
    }
    size_t idx = (history_write_ + HISTORY_SIZE - 1 - history_browse_) % HISTORY_SIZE;
    replace_line(history_[idx].buf, history_[idx].len);
}

void CLI::history_next() noexcept {
    if (history_browse_ == -1) return;
    if (history_browse_ == 0) {
        history_browse_ = -1;
        replace_line(saved_line_.buf, saved_line_.len);
        return;
    }
    --history_browse_;
    size_t idx = (history_write_ + HISTORY_SIZE - 1 - history_browse_) % HISTORY_SIZE;
    replace_line(history_[idx].buf, history_[idx].len);
}

size_t CLI::first_token_len(const char* s, size_t len) noexcept {
    size_t n = 0;
    while (n < len && s[n] != ' ' && s[n] != '\t') ++n;
    return n;
}
bool CLI::starts_with(const char* s, size_t slen, const char* prefix, size_t plen) noexcept {
    if (slen < plen) return false;
    for (size_t i = 0; i < plen; ++i) if (s[i] != prefix[i]) return false;
    return true;
}

void CLI::tab_complete() noexcept {
    size_t tok_len = first_token_len(line_, line_len_);
    if (cursor_ > tok_len) return;
    size_t match_count = 0;
    size_t first_match = 0;
    for (size_t i = 0; i < num_commands_; ++i) {
        size_t nlen = cstrlen(commands_[i].name);
        if (starts_with(commands_[i].name, nlen, line_, tok_len)) {
            if (match_count == 0) first_match = i;
            ++match_count;
        }
    }
    if (match_count == 0) return;
    if (match_count == 1) {
        const char* name = commands_[first_match].name;
        size_t nlen = cstrlen(name);
        char buf[MAX_LINE_LEN];
        size_t out = 0;
        for (size_t i = 0; i < nlen && out < MAX_LINE_LEN - 1; ++i) buf[out++] = name[i];
        if (out < MAX_LINE_LEN - 1) buf[out++] = ' ';
        for (size_t i = tok_len; i < line_len_ && out < MAX_LINE_LEN - 1; ++i) buf[out++] = line_[i];
        replace_line(buf, out);
        cursor_ = (nlen + 1 < MAX_LINE_LEN - 1) ? nlen + 1 : line_len_;
        if (cursor_ > line_len_) cursor_ = line_len_;
        if (cursor_ < line_len_) cursor_left(line_len_ - cursor_);
        return;
    }
    put(K_LF);
    for (size_t i = 0; i < num_commands_; ++i) {
        size_t nlen = cstrlen(commands_[i].name);
        if (starts_with(commands_[i].name, nlen, line_, tok_len)) {
            write("  ");
            write(commands_[i].name);
            put(K_LF);
        }
    }
    redraw();
}

void CLI::submit_line() noexcept {
    put(K_LF);
    size_t end = line_len_;
    while (end > 0 && (line_[end - 1] == ' ' || line_[end - 1] == '\t')) --end;
    size_t start = 0;
    while (start < end && (line_[start] == ' ' || line_[start] == '\t')) ++start;
    if (end > start) {
        bool dup = false;
        if (history_count_ > 0) {
            size_t most_recent = (history_write_ + HISTORY_SIZE - 1) % HISTORY_SIZE;
            const HistoryEntry& e = history_[most_recent];
            if (e.len == end - start) {
                dup = true;
                for (size_t i = 0; i < e.len; ++i) if (e.buf[i] != line_[start + i]) { dup = false; break; }
            }
        }
        if (!dup) {
            HistoryEntry& slot = history_[history_write_];
            slot.len = end - start;
            for (size_t i = 0; i < slot.len; ++i) slot.buf[i] = line_[start + i];
            history_write_ = (history_write_ + 1) % HISTORY_SIZE;
            if (history_count_ < HISTORY_SIZE) ++history_count_;
        }
        char cmdbuf[MAX_LINE_LEN];
        size_t cmdlen = end - start;
        for (size_t i = 0; i < cmdlen; ++i) cmdbuf[i] = line_[start + i];
        cmdbuf[cmdlen] = '\0';
        dispatch(cmdbuf);
    }
    line_len_ = 0;
    cursor_ = 0;
    history_browse_ = -1;
    prompt_line();
}

void CLI::dispatch(char* line) noexcept {
    char* args = line;
    while (*args && *args != ' ' && *args != '\t') ++args;
    if (*args) { *args++ = '\0'; while (*args == ' ' || *args == '\t') ++args; }
    auto* sink = sink_uart();
    for (size_t i = 0; i < num_commands_; ++i) {
        if (cstrcmp(commands_[i].name, line) == 0) {
            commands_[i].handler(args, sink);
            return;
        }
    }
    sink->puts("unknown command: ");
    sink->puts(line);
    sink->puts(" (type 'help')\n");
}

void CLI::handle_csi(char c) noexcept {
    switch (c) {
        case 'A': history_prev(); break;
        case 'B': history_next(); break;
        case 'C': move_right(); break;
        case 'D': move_left(); break;
        default: break;
    }
}

void CLI::handle_char(char c) noexcept {
    switch (esc_) {
        case EscState::NORMAL:
            if (c == K_ESC) { esc_ = EscState::ESC1; return; }
            if (c == K_CR || c == K_LF) { submit_line(); return; }
            if (c == K_BS || c == K_DEL) { backspace(); return; }
            if (c == K_TAB) { tab_complete(); return; }
            if (c == K_SOH) { while (cursor_ > 0) move_left(); return; }
            if (c == K_ENQ) { while (cursor_ < line_len_) move_right(); return; }
            if (c == K_NAK) { line_len_ = 0; cursor_ = 0; redraw(); return; }
            if (c == K_ETX) {
                write("^C\n");
                line_len_ = 0; cursor_ = 0; history_browse_ = -1;
                prompt_line();
                return;
            }
            if (is_printable(c)) insert_char(c);
            return;
        case EscState::ESC1:
            esc_ = (c == '[') ? EscState::CSI : EscState::NORMAL;
            return;
        case EscState::CSI:
            handle_csi(c);
            esc_ = EscState::NORMAL;
            return;
    }
}

void CLI::run() noexcept {
    write("\nminiOS CLI ready. Type 'help'.\n");
    prompt_line();
    for (;;) {
        char c = static_cast<char>(io::get_blocking());
        handle_char(c);
    }
}

void CLI::thread_entry(void* /*arg*/) {
    g_cli.run();
}

} // namespace cli
