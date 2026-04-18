// SPDX-License-Identifier: MIT OR Apache-2.0
// See bus_config.hpp for rationale.

#include "bus_config.hpp"
#include "master.hpp"
#include "pdo.hpp"
#include "../devices/device_db.hpp"
#include "../machine/machine_topology.hpp"
#include "../machine/motion_wiring.hpp"
#include "../motion/motion.hpp"
#include "../miniOS.hpp"
#include "../util.hpp"

namespace ethercat {

namespace {

inline uint64_t now_us() {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return t ? t->get_system_time_us() : 0;
}

// Cooperative sleep by yielding. The master's ESM poll runs at 250 µs; a
// 10 ms poll cadence here is plenty (config only has to happen once).
void sleep_ms(uint32_t ms) noexcept {
    const uint64_t target = now_us() + static_cast<uint64_t>(ms) * 1000ULL;
    while (now_us() < target) {
        if (kernel::g_scheduler_ptr)
            kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        else
            kernel::util::cpu_relax();
    }
}

void log(const char* s) noexcept {
    if (auto* u = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr)
        u->puts(s);
}

void make_master_prefix(char* out, size_t out_len, const Master& m) noexcept {
    if (!out || out_len == 0) return;
    kernel::util::k_snprintf(out, out_len, "[bus_config m%u/eth%u] ",
                             static_cast<unsigned>(m.id()),
                             static_cast<unsigned>(m.nic_idx()));
}

void log_line(const Master& m, const char* msg) noexcept {
    char prefix[32];
    char b[224];
    make_master_prefix(prefix, sizeof(prefix), m);
    kernel::util::k_snprintf(b, sizeof(b), "%s%s\n", prefix, msg);
    log(b);
}

void install_generic_pdo_sizes(const devices::DeviceEntry& dev,
                               SlaveInfo& slave) noexcept {
    uint16_t rx_bits = 0;
    uint16_t tx_bits = 0;
    for (size_t i = 0; i < dev.num_pdo; ++i) {
        const auto& p = dev.pdo[i];
        if (p.dir == 0) rx_bits = static_cast<uint16_t>(rx_bits + p.bits);
        else            tx_bits = static_cast<uint16_t>(tx_bits + p.bits);
    }
    const uint16_t rx_bytes = static_cast<uint16_t>((rx_bits + 7u) / 8u);
    const uint16_t tx_bytes = static_cast<uint16_t>((tx_bits + 7u) / 8u);
    slave.pdo_layout.rx_size_bytes = static_cast<uint8_t>(
        (rx_bytes < ethercat::PDO_SLOT_BYTES) ? rx_bytes : ethercat::PDO_SLOT_BYTES);
    slave.pdo_layout.tx_size_bytes = static_cast<uint8_t>(
        (tx_bytes < ethercat::PDO_SLOT_BYTES) ? tx_bytes : ethercat::PDO_SLOT_BYTES);
}

bool is_fixed_pdo_nomailbox_type(devices::DeviceType t) noexcept {
    return t == devices::DeviceType::Coupler
        || t == devices::DeviceType::DigitalInput
        || t == devices::DeviceType::DigitalOutput;
}

size_t find_device_index(const devices::DeviceEntry* dev) noexcept {
    if (!dev) return devices::MAX_DEVICES;
    for (size_t i = 0; i < devices::g_device_db.device_count(); ++i) {
        if (devices::g_device_db.at(i) == dev) return i;
    }
    return devices::MAX_DEVICES;
}

const devices::DeviceEntry* claim_nomailbox_device(std::array<bool, devices::MAX_DEVICES>& claimed) noexcept {
    for (size_t i = 0; i < devices::g_device_db.device_count(); ++i) {
        if (claimed[i]) continue;
        const auto* dev = devices::g_device_db.at(i);
        if (!dev || !is_fixed_pdo_nomailbox_type(dev->type)) continue;
        claimed[i] = true;
        return dev;
    }
    return nullptr;
}

} // namespace

void bus_config_entry(void* arg) {
    auto& m = arg ? *static_cast<Master*>(arg) : g_master_a;
    std::array<bool, devices::MAX_DEVICES> claimed_db{};
    char prefix[32];
    make_master_prefix(prefix, sizeof(prefix), m);

    // Wait for master to reach PreOp. Guard against never-reaching-PreOp by
    // capping total wait — on a bus with zero slaves the master keeps state
    // at Init forever, which is fine, we just never arm the gate.
    const uint64_t deadline_us = now_us() + 5'000'000ULL;   // 5 s
    while (m.state() != State::PreOp && now_us() < deadline_us) {
        sleep_ms(10);
    }
    if (m.state() != State::PreOp) {
        log_line(m, "master never reached PreOp; gate stays closed");
        return;
    }

    const size_t n = m.slave_count();
    char buf[192];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "%smaster at PreOp, configuring %zu slave(s) (heterogeneous)\n", prefix, n);
    log(buf);

    bool all_ok = true;
    for (size_t i = 0; i < n; ++i) {
        auto& slave = m.slave(i);
        const uint16_t station = slave.station_addr;
        if (station == 0) continue;
        // --- 1.3 identity probe (unconstrained — discover vid/pid) ----------
        // Read 0x1018:1..4 without expectations; match the result against
        // the TSV device DB. This replaces the old hard-coded "everyone is
        // the first DB entry" assumption and is what lets mixed topologies
        // (ClearPath + EK1100 + EL5042) come up cleanly.
        uint32_t vid = 0, pid = 0, rev = 0, ser = 0;
        const bool id_ok = m.probe_slave_identity(
            station, 0, 0, 0, &vid, &pid, &rev, &ser);
        slave.observed_vid = vid;
        slave.observed_pid = pid;
        slave.observed_rev = rev;
        slave.vendor_id    = vid;
        slave.product_code = pid;

        if (!id_ok) {
            const devices::DeviceEntry* guessed = claim_nomailbox_device(claimed_db);
            if (!guessed) {
                slave.identity_mismatch = true;
                all_ok = false;
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "%sslave 0x%04x identity upload FAILED and no "
                    "fixed-PDO no-mailbox DB candidate remained\n",
                    prefix, (unsigned)station);
                log(buf);
                continue;
            }

            slave.identity_mismatch = false;
            slave.observed_vid = guessed->id.vid;
            slave.observed_pid = guessed->id.pid;
            slave.observed_rev = 0;
            slave.vendor_id    = guessed->id.vid;
            slave.product_code = guessed->id.pid;
            install_generic_pdo_sizes(*guessed, slave);

            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: no mailbox identity; assuming %s "
                "(fixed PDO, discovery-order fallback)\n",
                prefix, (unsigned)station, guessed->name);
            log(buf);

            if (guessed->type == devices::DeviceType::Coupler) {
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "%sslave 0x%04x: coupler — no SDO/PDO/DC config needed\n",
                    prefix, (unsigned)station);
                log(buf);
                continue;
            }
            if (guessed->type == devices::DeviceType::DigitalInput ||
                guessed->type == devices::DeviceType::DigitalOutput) {
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "%sslave 0x%04x: %s (no-mailbox I/O) — fixed PDO layout\n",
                    prefix, (unsigned)station, guessed->name);
                log(buf);
                continue;
            }
        }

        // Device-DB lookup by {vid, pid}. An EK1100 entry is legitimate —
        // couplers show up on the segment, walk the ESM, and carry frames
        // for downstream slaves; they just have no SDOs for us to push.
        const devices::DeviceEntry* dev =
            devices::g_device_db.find({ vid, pid });
        if (!dev) {
            slave.identity_mismatch = true;
            all_ok = false;
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x UNKNOWN (vid=0x%08lx pid=0x%08lx) — "
                "not in device DB\n",
                prefix,
                (unsigned)station,
                (unsigned long)vid, (unsigned long)pid);
            log(buf);
            continue;
        }
        slave.identity_mismatch = false;
        install_generic_pdo_sizes(*dev, slave);
        {
            const size_t dbi = find_device_index(dev);
            if (dbi < claimed_db.size()) claimed_db[dbi] = true;
        }

        kernel::util::k_snprintf(buf, sizeof(buf),
            "%sslave 0x%04x = %s (vid=0x%08lx pid=0x%08lx rev=0x%08lx)\n",
            prefix, (unsigned)station, dev->name,
            (unsigned long)vid, (unsigned long)pid, (unsigned long)rev);
        log(buf);

        // Couplers (EK1100/EK1110/...) carry no mailbox so SDO / PDO /
        // DC-sync configuration is all skipped. They just need to walk the
        // ESM; the FSM already handles that via the broadcast AL_CTRL
        // path, no per-slave action required here. (Task 7.1.)
        if (dev->type == devices::DeviceType::Coupler) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: coupler — no SDO/PDO/DC config needed\n",
                prefix, (unsigned)station);
            log(buf);
            continue;
        }

        // Digital I/O terminals (EL1809 / EL2809 and similar) typically
        // have a Fixed PDO layout and NO mailbox — their ESI SyncManager
        // declares only the process-data SM, so CoE SDO is unavailable.
        // bus_config treats them the same as couplers from the SDO
        // standpoint; pack/unpack of the process image goes via
        // PdoLayout / cycle_lrw. DC is irrelevant — these terminals
        // are DC-less in their standard variants.
        if (dev->type == devices::DeviceType::DigitalInput ||
            dev->type == devices::DeviceType::DigitalOutput) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: %s (no-mailbox I/O) — fixed PDO layout\n",
                prefix, (unsigned)station, dev->name);
            log(buf);
            continue;
        }

        // Pin the matching device entry into a local for the tail of the
        // loop; keep `expected` naming for backward diff-readability.
        const devices::DeviceEntry* expected = dev;

        // Servo-only: push encoder resolution into the matching motion axis.
        // EncoderInput terminals (EL5042) don't carry 0x608F — the
        // per-channel bit width is in 0x8000/0x8010 instead, handled by
        // the sdo_init block below.
        if (expected->type == devices::DeviceType::Servo) {
            const int axis_idx = machine::topology::g_service.axis_for_slave(
                static_cast<uint8_t>(m.id()), i);
            if (axis_idx >= 0) {
                (void)machine::wiring::hook_servo_axis(
                    m, i, static_cast<size_t>(axis_idx), expected);
            }
            uint32_t inc = 0, revs = 0, cpr = 0;
            if (m.probe_encoder_resolution(station, &inc, &revs, &cpr) && cpr > 0) {
                if (axis_idx >= 0 && static_cast<size_t>(axis_idx) < motion::MAX_AXES) {
                    motion::g_motion.set_encoder_resolution(static_cast<size_t>(axis_idx), cpr);
                    kernel::util::k_snprintf(buf, sizeof(buf),
                        "%sslave 0x%04x: encoder cpr=%lu → axis %d\n",
                        prefix, (unsigned)station, (unsigned long)cpr, axis_idx);
                } else {
                    kernel::util::k_snprintf(buf, sizeof(buf),
                        "%sslave 0x%04x: encoder cpr=%lu (servo unbound in topology)\n",
                        prefix, (unsigned)station, (unsigned long)cpr);
                }
                log(buf);
            } else if (axis_idx < 0) {
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "%sslave 0x%04x: servo discovered but no topology axis binding\n",
                    prefix, (unsigned)station);
                log(buf);
            }
        }

        // Phase 4 + 7.2: push TSV sdo_init block. For servos this carries
        // 0x605E / 0x6065 / 0x231A / 0x2170 / 0x60C2. For EL5042 it carries
        // 0x8000 / 0x8010 (BiSS-C clock + bit counts) + PDO assignment.
        const size_t inits = configure_from_db(m, station, expected->id);
        kernel::util::k_snprintf(buf, sizeof(buf),
            "%sslave 0x%04x: %zu sdo_init writes queued from DB\n",
            prefix, (unsigned)station, inits);
        log(buf);

        // --- 1.1 PDO map dispatch (servo-only) ------------------------------
        // Default all discovered servo slaves to CSP. EL5042 / EL3162 PDO
        // assignment comes from their own sdo_init block (above) since
        // those TSVs use device-native 0x1A00/0x1A01 selection rather
        // than the CiA-402 per-mode variants. AnalogInput terminals fall
        // into the same "sdo_init did it" bucket.
        const size_t downloads = (expected->type == devices::DeviceType::Servo)
            ? configure_cia402_axis_for_mode(m, station, expected->id,
                                             cia402::Mode::CyclicSyncPosition)
            : 0;
        if (expected->type == devices::DeviceType::Servo) {
            PdoLayout layout;
            if (compute_pdo_layout(expected->id, cia402::Mode::CyclicSyncPosition, layout)) {
                slave.pdo_layout = layout;
            }
        }
        if (downloads == 0) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: no CSP PDO map entries in DB — "
                "skipping PDO configure\n",
                prefix, (unsigned)station);
            log(buf);
        }

        // --- Phase 3.2/3.3/3.4: Distributed Clocks ---------------------------
        // Master cycle is 250 µs (< 1 ms), so DC-Sync is mandatory — the
        // ClearPath PDF explicitly allows sub-ms only with DC configured.
        // Cycle time = master period. Shift = (0x1C32:6 calc+copy, default
        // 15000 ns) + 10 µs margin so the SYNC0 pulse lands after the LRW
        // frame, not inside it. AssignActivate comes from the device DB
        // (0x0300 for ClearPath-EC, the only supported OpMode).
        if (expected->dc_assign_activate != 0) {
            const uint32_t cycle_ns = m.period_us() * 1000u;
            const uint32_t shift_ns = 15000u + 10000u; // 15 µs + 10 µs margin
            const size_t dc_sent = m.configure_dc_sync0(
                station, expected->dc_assign_activate, cycle_ns, shift_ns);
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: DC SYNC0 cycle=%u ns shift=%u ns "
                "act=0x%04x (%zu FPWRs queued)\n",
                prefix,
                (unsigned)station,
                (unsigned)cycle_ns, (unsigned)shift_ns,
                (unsigned)expected->dc_assign_activate,
                dc_sent);
            log(buf);
        } else {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sslave 0x%04x: no dc_assign in DB — "
                "skipping DC-Sync (cycle < 1 ms may degrade)\n",
                prefix, (unsigned)station);
            log(buf);
        }
    }

    // Task 8.1 — E-bus current budget. Sum the TSV-declared current draw
    // for every slave downstream of the first coupler (i.e. every
    // non-coupler slave in discovery order after one has appeared), and
    // refuse to bring the bus up if the draw exceeds 2000 mA. This
    // matches the Beckhoff EK1100 hard limit and is the single most
    // common reason a real multi-terminal bus fails at SafeOp. A bus
    // with no coupler (all servos, like today's fake_slave test) skips
    // the check entirely.
    {
        bool     coupler_seen = false;
        uint32_t downstream_ma = 0;
        for (size_t i = 0; i < n; ++i) {
            const auto& slave = m.slave(i);
            if (slave.station_addr == 0) continue;
            const auto* dev = devices::g_device_db.find({ slave.observed_vid,
                                                          slave.observed_pid });
            if (!dev) continue;
            if (dev->type == devices::DeviceType::Coupler) {
                coupler_seen = true;
                continue;
            }
            if (coupler_seen) downstream_ma += dev->ebus_current_ma;
        }
        if (downstream_ma > 2000) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sE-bus current budget EXCEEDED: %u mA > 2000 mA "
                "(gate stays closed)\n",
                prefix, (unsigned)downstream_ma);
            log(buf);
            all_ok = false;
        } else if (downstream_ma > 0) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "%sE-bus current budget OK: %u / 2000 mA downstream\n",
                prefix, (unsigned)downstream_ma);
            log(buf);
        }
    }

    if (all_ok) {
        m.allow_safeop(true);
        log_line(m, "all slaves OK — gate opened, ESM may proceed to SafeOp");
    } else {
        log_line(m, "one or more slaves failed identity check — gate stays closed");
    }
}

} // namespace ethercat
