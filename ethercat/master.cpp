// SPDX-License-Identifier: MIT OR Apache-2.0
// EtherCAT master — cyclic loop, ESM drive, discovery, LRW process data.
//
// Scope: framing + ESM + CoE-SDO-based PDO mapping. No Distributed Clocks.

#include "master.hpp"
#include "frame.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include "diag/jitter.hpp"

#include <cstring>

extern "C" void early_uart_puts(const char*);

namespace ethercat {

// EtherCAT master ids are logical controller ids; NIC ownership is configured at
// boot. By default the controller reserves eth0 for HMI and places ec_a/ec_b on
// eth1/eth2 respectively.
//
// Cycle period is 250 µs — the ClearPath-EC
// minimum (PDF §Network Timing: cycle must be ≥ 250 µs and an integer multiple
// of 250 µs). Must stay coupled to 0x60C2 in devices/clearpath_ec.tsv
// (currently value=25, index=-5 → 25·10⁻⁵ s = 250 µs). Do not lower without
// updating 0x60C2; the CI gate "period_us % 250 == 0 && >= 250" will reject it.
Master g_master_a(0, 250);
Master g_master_b(1, 250);

namespace {

// Routes through TimerDriverOps::wait_until_ns so the arch-specific HAL
// can substitute a WFI-based halt (arm64 CNTP_TVAL+WFI, rv64 mtimecmp+wfi)
// in dedicated-core mode. Default impl on both HALs is still busy-spin.
inline void wait_until_us(uint64_t target_us) {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!t) return;
    t->wait_until_ns(target_us * 1000ULL);
}

constexpr uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
constexpr uint8_t SRC_MAC[6]       = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

constexpr uint64_t ESM_TIMEOUT_US = 200000; // 200 ms per transition

// poll_rx drain budget. POLL_RX_BATCH small enough that the worst-case
// overshoot is only one batch's worth of handler work past the deadline;
// POLL_RX_SAFETY_MARGIN_US stops the loop while there's still slack to
// finish the iteration without tripping the deadline-fault latch.
constexpr size_t   POLL_RX_BATCH              = 2;
constexpr size_t   POLL_RX_MAX_PER_CYCLE      = 8;
constexpr uint64_t POLL_RX_SAFETY_MARGIN_US   = 30;

template<typename T>
bool upload_scalar(Master& m, uint16_t station, uint16_t index, uint8_t sub,
                   T* out, uint8_t expect_bytes) noexcept {
    if (!out || expect_bytes == 0 || expect_bytes > 4) return false;
    uint8_t buf[4] = {};
    uint8_t bytes  = 0;
    uint32_t abort = 0;
    if (!m.upload_sdo(station, index, sub, buf, &bytes, /*timeout=*/100000, &abort))
        return false;
    if (bytes != expect_bytes) return false;
    uint32_t v = 0;
    for (uint8_t i = 0; i < bytes; ++i) {
        v |= static_cast<uint32_t>(buf[i]) << (8u * i);
    }
    *out = static_cast<T>(v);
    return true;
}

uint8_t next_al_target(uint8_t cur) noexcept {
    switch (cur) {
        case AL_INIT:   return AL_PREOP;
        case AL_PREOP:  return AL_SAFEOP;
        case AL_SAFEOP: return AL_OP;
        default:        return cur;
    }
}

State al_to_master_state(uint8_t s) noexcept {
    switch (s) {
        case AL_INIT:   return State::Init;
        case AL_PREOP:  return State::PreOp;
        case AL_SAFEOP: return State::SafeOp;
        case AL_OP:     return State::Op;
        default:        return State::Fault;
    }
}

} // namespace

uint64_t Master::now_us() const noexcept {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return t ? t->get_system_time_us() : 0;
}

bool Master::send_frame(const uint8_t* buf, size_t len) noexcept {
    if (!net_) return false;
    if (net_->send_packet(nic_idx_, buf, len)) {
        stats_.tx_frames.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

void Master::rx_trampoline(int if_idx, const uint8_t* data, size_t len,
                           void* ctx) noexcept {
    (void)if_idx;
    auto* self = static_cast<Master*>(ctx);
    if (self) self->handle_rx_frame(data, len);
}

void Master::handle_rx_frame(const uint8_t* data, size_t len) noexcept {
    if (!data || len == 0) return;
    stats_.rx_frames.fetch_add(1, std::memory_order_relaxed);

    // Walk every datagram, pulling out three kinds of reply the master cares
    // about today: BRD discovery (WKC → slave_count_), FPRD AL_STATUS
    // (updates slaves_[i].current_state so step_esm can gate on observation),
    // and LRW process-data (parsed back into each slave's cia402::Drive).
    FrameParser fp(data, len);
    if (!fp.valid()) return;
    FrameParser::Datagram dg;
    while (fp.next(dg)) {
        // --- BRD discovery ---
        if (dg.cmd == Cmd::BRD && dg.address == REG_DL_STATUS && dg.wkc > 0) {
            stats_.discovered.store(dg.wkc, std::memory_order_relaxed);
            if (dg.wkc > slave_count_ && dg.wkc <= MAX_SLAVES) {
                const uint64_t epoch_now =
                    snapshot_epoch_.load(std::memory_order_relaxed);
                for (size_t i = slave_count_; i < dg.wkc; ++i) {
                    slaves_[i].station_addr   = static_cast<uint16_t>(0x1001 + i);
                    slaves_[i].current_state  = AL_INIT;
                    slaves_[i].target_state   = AL_INIT;
                    slaves_[i].vendor_id      = 0;
                    slaves_[i].product_code   = 0;
                    // Default drive config: CSP mode, target OperationEnabled.
                    slaves_[i].drive = cia402::Drive{};
                    slaves_[i].drive.mode_op      = cia402::Mode::CyclicSyncPosition;
                    slaves_[i].drive.target_state = cia402::State::OperationEnabled;
                    slaves_[i].pdo_layout         = default_csp_layout();
                    slaves_[i].rx_process_data.fill(0);
                    slaves_[i].tx_process_data.fill(0);
                    slaves_[i].present            = true;
                    slaves_[i].last_seen_cycle    = epoch_now;
                }
                slave_count_ = dg.wkc;
            }
        }

        // --- FPRD AL_STATUS (ESM poll response) ---
        // Address is (station << 16) | REG_AL_STATUS; data[0] low nibble = AL state.
        if (dg.cmd == Cmd::FPRD
            && (dg.address & 0xFFFFu) == REG_AL_STATUS
            && dg.wkc > 0
            && dg.data_len >= 1
            && dg.data != nullptr) {
            const uint16_t station = static_cast<uint16_t>(dg.address >> 16);
            for (size_t i = 0; i < slave_count_; ++i) {
                if (slaves_[i].station_addr == station) {
                    slaves_[i].current_state = static_cast<uint8_t>(dg.data[0] & 0x0F);
                    slaves_[i].last_seen_cycle =
                        snapshot_epoch_.load(std::memory_order_relaxed);
                    break;
                }
            }
        }

        // --- SDO upload response (task 1.2 + segmented follow-up) ---
        // FPRD to (station<<16) | SM1_MAILBOX_IN_OFFS. Dispatch by current
        // upload state — either we're expecting an initiate response
        // (Pending) or a segment response (FetchSegment).
        if (dg.cmd == Cmd::FPRD
            && (dg.address & 0xFFFFu) == 0x1400u
            && dg.wkc > 0
            && dg.data != nullptr) {
            const uint16_t station = static_cast<uint16_t>(dg.address >> 16);
            // Tier 3b: peek at the CoE service / cmd byte to recognise an
            // SDO download confirm/abort response. CoE header is 2 bytes
            // at offset 6 (after the 6-byte mailbox header); cmd byte at
            // offset 8. A download-direction response (master initiated)
            // has cmd nibble 0x60 (confirm) or 0x80 (abort).
            if (dl_in_flight_ && station == dl_station_ && dg.data_len >= 12) {
                const uint8_t  cmd      = dg.data[8];
                const uint16_t resp_idx =
                    static_cast<uint16_t>(dg.data[9]) |
                    (static_cast<uint16_t>(dg.data[10]) << 8);
                const uint8_t  resp_sub = dg.data[11];
                const uint32_t abort_code = (dg.data_len >= 16)
                    ? (static_cast<uint32_t>(dg.data[12]) |
                       (static_cast<uint32_t>(dg.data[13]) << 8) |
                       (static_cast<uint32_t>(dg.data[14]) << 16) |
                       (static_cast<uint32_t>(dg.data[15]) << 24))
                    : 0u;
                if ((cmd & 0xE0) == 0x60 || (cmd & 0xE0) == 0x80) {
                    on_sdo_download_response(station, cmd,
                                             resp_idx, resp_sub, abort_code);
                    if (!dg.more) break;
                    continue;
                }
            }
            if (station != upload_station_) {
                if (!dg.more) break;
                continue;
            }
            const auto st = upload_state_.load(std::memory_order_acquire);
            if (st == UploadState::Pending) {
                extern bool classify_sdo_upload_response(const uint8_t*, size_t,
                                                         uint16_t, uint8_t,
                                                         uint8_t*, uint8_t[4],
                                                         uint8_t*, uint32_t*,
                                                         uint32_t*);
                uint8_t kind = 3;
                uint8_t tmp[4] = {};
                uint8_t bytes = 0;
                uint32_t total = 0;
                uint32_t abort = 0;
                classify_sdo_upload_response(dg.data, dg.data_len,
                    upload_index_, upload_sub_, &kind, tmp, &bytes, &total, &abort);
                if (kind == 0) { // expedited
                    std::memcpy(upload_result_, tmp, bytes);
                    upload_result_bytes_   = bytes;
                    upload_total_expected_ = bytes;
                    upload_abort_code_     = 0;
                    upload_state_.store(UploadState::Done, std::memory_order_release);
                } else if (kind == 1) { // segmented initiate
                    upload_result_bytes_    = 0;
                    upload_total_expected_  = static_cast<uint16_t>(
                        (total > UPLOAD_RESULT_CAP) ? UPLOAD_RESULT_CAP : total);
                    upload_segment_toggle_  = false;
                    upload_state_.store(UploadState::FetchSegment,
                                        std::memory_order_release);
                } else if (kind == 2) { // abort
                    upload_abort_code_ = abort;
                    upload_state_.store(UploadState::Error,
                                        std::memory_order_release);
                }
                // kind==3: keep polling.
            } else if (st == UploadState::FetchSegment) {
                extern bool parse_sdo_upload_segment_response(const uint8_t*, size_t,
                                                              uint8_t[7], uint8_t*,
                                                              bool*, uint32_t*);
                uint8_t segbuf[7] = {};
                uint8_t seglen = 0;
                bool complete = false;
                uint32_t abort = 0;
                if (parse_sdo_upload_segment_response(dg.data, dg.data_len,
                                                     segbuf, &seglen, &complete,
                                                     &abort)) {
                    upload_segment_request_inflight_ = false;
                    const size_t room = (upload_result_bytes_ < UPLOAD_RESULT_CAP)
                        ? (UPLOAD_RESULT_CAP - upload_result_bytes_) : 0;
                    const size_t take = (seglen < room) ? seglen : room;
                    std::memcpy(&upload_result_[upload_result_bytes_], segbuf, take);
                    upload_result_bytes_ = static_cast<uint16_t>(
                        upload_result_bytes_ + take);
                    if (complete ||
                        upload_result_bytes_ >= upload_total_expected_ ||
                        upload_result_bytes_ >= UPLOAD_RESULT_CAP) {
                        upload_state_.store(UploadState::Done,
                                            std::memory_order_release);
                    } else {
                        // Request next segment (service_sdo_upload will send it).
                        upload_segment_toggle_ = !upload_segment_toggle_;
                    }
                } else if (abort != 0) {
                    upload_abort_code_ = abort;
                    upload_state_.store(UploadState::Error,
                                        std::memory_order_release);
                }
            }
        }

        if (dg.cmd == Cmd::FPRD
            && dg.wkc > 0
            && dg.data != nullptr
            && esc_read_state_.load(std::memory_order_acquire) == EscReadState::Pending
            && (dg.address & 0xFFFFu) == esc_read_reg_
            && static_cast<uint16_t>(dg.address >> 16) == esc_read_station_) {
            const uint8_t take = (dg.data_len < esc_read_len_) ? static_cast<uint8_t>(dg.data_len)
                                                               : esc_read_len_;
            std::memcpy(esc_read_buf_, dg.data, take);
            esc_read_state_.store(EscReadState::Done, std::memory_order_release);
        }

        // --- LRW process-data reply ---
        // The master's cycle_lrw() ships one big LRW datagram at logical
        // address 0 carrying the concatenation of every enrolled slave's
        // per-slave slot. Phase C: slot widths are per-slave (set by
        // bus_config from the ESI catalog), so RX walks a running
        // byte_offset accumulator instead of a fixed stride. Slaves with
        // zero tx_size_bytes contribute nothing and are skipped — that
        // matches what cycle_lrw packs.
        if (dg.cmd == Cmd::LRW && dg.wkc > 0 && dg.address == 0
            && dg.data != nullptr) {
            const uint64_t epoch_now =
                snapshot_epoch_.load(std::memory_order_relaxed);
            size_t byte_offset = 0;
            for (size_t i = 0; i < slave_count_; ++i) {
                auto& d = slaves_[i].drive;
                const auto& lay = slaves_[i].pdo_layout;
                const uint8_t per_slave = (lay.rx_size_bytes > lay.tx_size_bytes)
                    ? lay.rx_size_bytes : lay.tx_size_bytes;
                if (per_slave == 0) continue;
                if (byte_offset + per_slave > dg.data_len) break;
                const uint8_t* p = dg.data + byte_offset;
                byte_offset += per_slave;
                slaves_[i].last_seen_cycle = epoch_now;
                const size_t tx_take = (lay.tx_size_bytes <= PDO_SLOT_BYTES)
                    ? lay.tx_size_bytes : PDO_SLOT_BYTES;
                if (tx_take > 0) {
                    std::memcpy(slaves_[i].tx_process_data.data(), p, tx_take);
                }

                // Task 1.8 — unpack each Tx entry at its layout-derived
                // offset. Fields not mapped in this PDO pair are left
                // untouched on the drive (holding their previous value).
                if (lay.tx_statusword_off      != 0xFF)
                    d.statusword      = get_u16_le(&p[lay.tx_statusword_off]);
                if (lay.tx_mode_op_display_off != 0xFF)
                    d.mode_op_display = static_cast<cia402::Mode>(
                        static_cast<int8_t>(p[lay.tx_mode_op_display_off]));
                if (lay.tx_position_actual_off != 0xFF)
                    d.actual_position = static_cast<int32_t>(
                        get_u32_le(&p[lay.tx_position_actual_off]));
                if (lay.tx_velocity_actual_off != 0xFF)
                    d.actual_velocity = static_cast<int32_t>(
                        get_u32_le(&p[lay.tx_velocity_actual_off]));
                if (lay.tx_torque_actual_off   != 0xFF)
                    d.actual_torque   = static_cast<int16_t>(
                        get_u16_le(&p[lay.tx_torque_actual_off]));
                if (lay.tx_error_code_off      != 0xFF)
                    d.error_code      = get_u16_le(&p[lay.tx_error_code_off]);
                if (lay.tx_digital_inputs_off  != 0xFF)
                    d.digital_inputs  = get_u32_le(&p[lay.tx_digital_inputs_off]);
                // Refresh decoded FSA state + next controlword from the just-
                // observed statusword. This is what pumps the handshake.
                d.step();
            }
        }
        if (!dg.more) break;
    }
}

void Master::send_probe() {
    // Legacy probe — a BRD on DL status (0x0000), used as a keep-alive during
    // Init only. Also serves as the discovery datagram.
    uint8_t frame[64];
    FrameBuilder fb(frame, sizeof(frame), BROADCAST_MAC, SRC_MAC);
    uint8_t pad[2] = {0, 0};
    (void)fb.add_datagram(Cmd::BRD, dgram_idx_++, 0x00000000, pad, 2);
    size_t n = fb.finalize();
    send_frame(frame, n);
}

void Master::discover_slaves() {
    // Broadcast-read DL_STATUS (0x0110, 2 B). WKC increments once per slave;
    // without a live bus (QEMU SLIRP) this returns 0. We simply snapshot the
    // most recent WKC into stats; actual WKC walking lives in the RX callback.
    uint8_t frame[64];
    FrameBuilder fb(frame, sizeof(frame), BROADCAST_MAC, SRC_MAC);
    uint8_t pad[2] = {0, 0};
    (void)fb.add_datagram(Cmd::BRD, dgram_idx_++, REG_DL_STATUS, pad, 2);
    size_t n = fb.finalize();
    send_frame(frame, n);
    // With no RX plumbing back into the master yet, we rely on the WKC counter
    // reset below when slaves appear. For now, slave_count_ stays 0 on QEMU.
}

void Master::step_esm() {
    // If no slaves, just mirror cycle count into state for UX.
    if (slave_count_ == 0) {
        // Shadow the legacy walk so `ec` still shows progression on QEMU.
        State s = state_.load(std::memory_order_relaxed);
        uint64_t c = stats_.cycles.load(std::memory_order_relaxed);
        State next = s;
        if (s == State::Init   && c >= 10)  next = State::PreOp;
        else if (s == State::PreOp && c >= 50)  next = State::SafeOp;
        else if (s == State::SafeOp && c >= 100) next = State::Op;
        if (next != s) state_.store(next, std::memory_order_release);
        return;
    }

    switch (esm_phase_) {
    case EsmPhase::Idle: {
        // Pick the first slave that isn't at its target state and request the
        // next transition for the entire bus (we drive them in lock-step).
        uint8_t cur = slaves_[0].current_state;
        uint8_t tgt = next_al_target(cur);
        if (tgt == cur) return; // already Op
        // PLAN 1.1 + 1.3 gate: never leave PreOp until the bus_config helper
        // has verified identity and pushed the per-mode PDO map. This is the
        // single point where a wiring-error or wrong-revision slave gets
        // caught before anyone can command torque.
        if (cur == AL_PREOP && !allow_safeop_.load(std::memory_order_acquire)) {
            return;
        }
        esm_target_ = tgt;
        esm_phase_  = EsmPhase::RequestWrite;
        break;
    }
    case EsmPhase::RequestWrite: {
        // FPWR to AL_CTRL on every known slave. One datagram per slave, one
        // frame. Reuses the Master's pre-allocated tx_scratch_ buffer.
        FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(), BROADCAST_MAC, SRC_MAC);
        uint8_t state_bytes[2] = { esm_target_, 0 };
        for (size_t i = 0; i < slave_count_; ++i) {
            uint32_t addr = (static_cast<uint32_t>(slaves_[i].station_addr) << 16)
                          | REG_AL_CTRL;
            if (!fb.add_datagram(Cmd::FPWR, dgram_idx_++, addr, state_bytes, 2)) break;
        }
        send_frame(tx_scratch_.data(), fb.finalize());
        esm_deadline_us_ = now_us() + ESM_TIMEOUT_US;
        esm_phase_ = EsmPhase::PollStatus;
        break;
    }
    case EsmPhase::PollStatus: {
        // FPRD AL_STATUS on every slave. handle_rx_frame will fold returned
        // AL_STATUS bytes into slaves_[i].current_state; we then check
        // convergence below.
        FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(), BROADCAST_MAC, SRC_MAC);
        uint8_t pad[2] = {0, 0};
        for (size_t i = 0; i < slave_count_; ++i) {
            uint32_t addr = (static_cast<uint32_t>(slaves_[i].station_addr) << 16)
                          | REG_AL_STATUS;
            if (!fb.add_datagram(Cmd::FPRD, dgram_idx_++, addr, pad, 2)) break;
        }
        send_frame(tx_scratch_.data(), fb.finalize());

        // Real transition: fire Settled as soon as every slave observes the
        // requested state. The time-based check below is only a watchdog —
        // it still fires esm_timeouts when the slaves never respond.
        bool all_reached = true;
        for (size_t i = 0; i < slave_count_; ++i) {
            if (slaves_[i].current_state != esm_target_) { all_reached = false; break; }
        }
        if (all_reached) {
            esm_phase_ = EsmPhase::Settled;
        } else if (now_us() >= esm_deadline_us_) {
            stats_.esm_timeouts.fetch_add(1, std::memory_order_relaxed);
            // Watchdog fallback for QEMU backends that don't RX back: assume
            // the transition took so we don't wedge the master forever.
            for (size_t i = 0; i < slave_count_; ++i) {
                slaves_[i].current_state = esm_target_;
            }
            esm_phase_ = EsmPhase::Settled;
        }
        break;
    }
    case EsmPhase::Settled: {
        state_.store(al_to_master_state(esm_target_), std::memory_order_release);
        esm_phase_ = EsmPhase::Idle;
        break;
    }
    case EsmPhase::Error:
        // Stay put — a future pass will read 0x0134 and surface the code.
        break;
    }
}

void Master::cycle_lrw() {
    // Exchange the process-data image with a single LRW datagram at logical
    // address 0. Phase C: per-slave slot width comes from the slave's
    // pdo_layout (rx_size_bytes / tx_size_bytes) populated by bus_config
    // from the ESI catalog. The on-wire slot is sized to max(rx, tx) so
    // both directions fit; bus_config rejects any topology whose total
    // would overflow PDO_BUF_BYTES, so the running offset can't outrun
    // pdo_buf_ here.
    //
    // RX entries are packed at the layout-derived offsets relative to the
    // slot start; NOT_MAPPED = 0xFF skips the field for this PDO pair.
    // Skipping a slave with zero size keeps cycle_lrw idempotent for
    // couplers and digital-IO terminals before their layouts are set up
    // (those don't appear in LRW at all).
    if (slave_count_ == 0) return;

    size_t byte_offset = 0;
    for (size_t i = 0; i < slave_count_; ++i) {
        auto& d = slaves_[i].drive;
        // Before the slave has ever spoken to us the statusword is 0, which
        // decodes to NotReadyToSwitchOn; step() then emits the correct first
        // controlword (CW_CMD_SHUTDOWN) targeting OperationEnabled.
        d.step();

        const auto& lay = slaves_[i].pdo_layout;
        const uint8_t per_slave = (lay.rx_size_bytes > lay.tx_size_bytes)
            ? lay.rx_size_bytes : lay.tx_size_bytes;
        if (per_slave == 0) continue;
        if (byte_offset + per_slave > pdo_buf_.size()) break;

        uint8_t* p = &pdo_buf_[byte_offset];
        // Seed the slot from the per-slave RX mirror (whatever upper layers
        // staged via rx_process_data — e.g. AnalogInput passthroughs).
        const size_t mirror_seed = (per_slave <= PDO_SLOT_BYTES)
            ? per_slave : PDO_SLOT_BYTES;
        std::memcpy(p, slaves_[i].rx_process_data.data(), mirror_seed);
        if (lay.rx_controlword_off     != 0xFF) put_u16_le(&p[lay.rx_controlword_off], d.controlword);
        if (lay.rx_mode_op_off         != 0xFF) p[lay.rx_mode_op_off] =
            static_cast<uint8_t>(static_cast<int8_t>(d.mode_op));
        if (lay.rx_target_position_off != 0xFF) put_u32_le(&p[lay.rx_target_position_off],
                                                           static_cast<uint32_t>(d.target_position));
        if (lay.rx_target_velocity_off != 0xFF) put_u32_le(&p[lay.rx_target_velocity_off],
                                                           static_cast<uint32_t>(d.target_velocity));
        if (lay.rx_target_torque_off   != 0xFF) put_u16_le(&p[lay.rx_target_torque_off],
                                                           static_cast<uint16_t>(d.target_torque));
        if (lay.rx_digital_outputs_off != 0xFF) put_u32_le(&p[lay.rx_digital_outputs_off],
                                                           d.digital_outputs);
        const size_t rx_take = (lay.rx_size_bytes <= PDO_SLOT_BYTES)
            ? lay.rx_size_bytes : PDO_SLOT_BYTES;
        if (rx_take > 0) {
            std::memcpy(slaves_[i].rx_process_data.data(), p, rx_take);
        }
        byte_offset += per_slave;
    }

    if (byte_offset == 0) return; // nothing to ship — every slave is unsized
    const uint16_t len = static_cast<uint16_t>(byte_offset);
    FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(), BROADCAST_MAC, SRC_MAC);
    (void)fb.add_datagram(Cmd::LRW, dgram_idx_++, 0, pdo_buf_.data(), len);
    send_frame(tx_scratch_.data(), fb.finalize());
}

// Task 1.2 — SDO upload support.
//
// SM0 "mailbox-out" (master -> slave) lives at physical offset 0x1000 on the
// slave; SM1 "mailbox-in" (slave -> master) lives at 0x1400. The master's
// FPRD to (station<<16)|0x1400 pulls the response mailbox. See
// devices/clearpath_ec.tsv SyncManager comment (line 24–26) for the
// per-direction offsets; sizes are the ClearPath defaults.
namespace {
constexpr uint16_t SM0_MAILBOX_OUT_OFFS = 0x1000; // master -> slave
constexpr uint16_t SM1_MAILBOX_IN_OFFS  = 0x1400; // slave -> master
constexpr uint16_t SDO_MAILBOX_LEN      = 16;     // [hdr 6][coe 2][sdo 8]
} // namespace

bool Master::upload_sdo(uint16_t station_addr, uint16_t index, uint8_t sub,
                        uint8_t out[4], uint8_t* out_bytes,
                        uint32_t timeout_us,
                        uint32_t* abort_code) noexcept {
    if (!out || !out_bytes) return false;

    // Serialise concurrent callers — one upload in flight per Master.
    bool expected = false;
    if (!upload_busy_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel)) {
        if (abort_code) *abort_code = 0;
        return false;
    }

    // Fire the request via the existing SDO-download-style frame builder —
    // we reuse build_sdo_upload_request which takes the same station/index/sub
    // shape. Fire-and-forget: the slave will write the response into SM1 and
    // the cycle loop's service_sdo_upload() will pull it.
    extern size_t build_sdo_upload_request(uint8_t*, size_t,
                                           const uint8_t[6], const uint8_t[6],
                                           uint16_t, uint16_t, uint8_t,
                                           uint8_t&);
    uint8_t frame[128];
    size_t n = build_sdo_upload_request(frame, sizeof(frame),
                                        BROADCAST_MAC, SRC_MAC,
                                        station_addr, index, sub,
                                        sdo_counter_);
    if (!n || !send_frame(frame, n)) {
        upload_busy_.store(false, std::memory_order_release);
        if (abort_code) *abort_code = 0;
        return false;
    }
    stats_.sdo_tx.fetch_add(1, std::memory_order_relaxed);

    // Publish the slot so the cycle loop starts polling SM1.
    upload_station_ = station_addr;
    upload_index_   = index;
    upload_sub_     = sub;
    upload_result_bytes_ = 0;
    upload_abort_code_   = 0;
    upload_segment_request_inflight_ = false;
    upload_deadline_us_  = now_us() + timeout_us;
    upload_state_.store(UploadState::Pending, std::memory_order_release);

    // Spin-wait for the cycle loop to flip the state. Yield between polls
    // so the scheduler can actually run the ec_a thread; yield to the
    // CALLING core, not core 0 — `yield(0)` always woke core 0's
    // scheduler regardless of who called us, starving same-core callers
    // when this code ran from a non-zero CLI thread or the bus_config
    // helper migrated cores. Also bail when the deadline elapses so a
    // wedged response can't leave the caller looping forever.
    const uint32_t caller_core = kernel::g_platform
        ? kernel::g_platform->get_core_id() : 0;
    for (;;) {
        const auto s = upload_state_.load(std::memory_order_acquire);
        if (s == UploadState::Done) {
            *out_bytes = upload_result_bytes_;
            std::memcpy(out, upload_result_, 4);
            if (abort_code) *abort_code = 0;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Idle, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            return true;
        }
        if (s == UploadState::Error) {
            *out_bytes = 0;
            if (abort_code) *abort_code = upload_abort_code_;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Idle, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            return false;
        }
        if (now_us() > upload_deadline_us_) {
            *out_bytes = 0;
            if (abort_code) *abort_code = 0;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Idle, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            return false;
        }
        if (kernel::g_scheduler_ptr) kernel::g_scheduler_ptr->yield(caller_core);
    }
}

bool Master::upload_sdo_start(uint16_t station_addr, uint16_t index, uint8_t sub,
                              uint32_t timeout_us) noexcept {
    bool expected = false;
    if (!upload_busy_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel)) {
        return false; // another upload in flight
    }
    extern size_t build_sdo_upload_request(uint8_t*, size_t,
                                           const uint8_t[6], const uint8_t[6],
                                           uint16_t, uint16_t, uint8_t,
                                           uint8_t&);
    uint8_t frame[128];
    const size_t n = build_sdo_upload_request(frame, sizeof(frame),
                                              BROADCAST_MAC, SRC_MAC,
                                              station_addr, index, sub,
                                              sdo_counter_);
    if (!n || !send_frame(frame, n)) {
        upload_busy_.store(false, std::memory_order_release);
        return false;
    }
    stats_.sdo_tx.fetch_add(1, std::memory_order_relaxed);
    upload_station_ = station_addr;
    upload_index_   = index;
    upload_sub_     = sub;
    upload_result_bytes_ = 0;
    upload_abort_code_   = 0;
    upload_segment_request_inflight_ = false;
    upload_deadline_us_  = now_us() + timeout_us;
    upload_state_.store(UploadState::Pending, std::memory_order_release);
    return true;
}

bool Master::send_sdo_download_tracked(uint16_t station_addr,
                                       uint16_t index, uint8_t sub,
                                       const uint8_t* data, uint8_t len_bytes,
                                       std::atomic<uint8_t>* completion_state,
                                       uint32_t* abort_code,
                                       uint32_t timeout_us) noexcept {
    if (dl_in_flight_) return false;
    if (completion_state) completion_state->store(0, std::memory_order_release);
    if (abort_code) *abort_code = 0;
    if (!send_sdo_download(station_addr, index, sub, data, len_bytes)) {
        if (completion_state) completion_state->store(2, std::memory_order_release);
        return false;
    }
    dl_in_flight_   = true;
    dl_station_     = station_addr;
    dl_index_       = index;
    dl_sub_         = sub;
    dl_completion_  = completion_state;
    dl_abort_code_  = abort_code;
    dl_deadline_us_ = now_us() + timeout_us;
    return true;
}

void Master::service_sdo_download() noexcept {
    if (!dl_in_flight_) return;
    // Poll SM1 mailbox for the slave's CoE response. Most slaves answer
    // an expedited download in <2 cycles; we issue one FPRD per cycle
    // until something lands or the deadline expires.
    {
        FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(),
                        BROADCAST_MAC, SRC_MAC);
        uint8_t pad[SDO_MAILBOX_LEN] = {};
        const uint32_t addr = (static_cast<uint32_t>(dl_station_) << 16)
                            | SM1_MAILBOX_IN_OFFS;
        if (fb.add_datagram(Cmd::FPRD, dgram_idx_++, addr, pad, SDO_MAILBOX_LEN)) {
            send_frame(tx_scratch_.data(), fb.finalize());
        }
    }
    if (now_us() <= dl_deadline_us_) return;
    if (dl_completion_) dl_completion_->store(2, std::memory_order_release);
    if (dl_abort_code_) *dl_abort_code_ = 0; // 0 = timeout
    dl_in_flight_   = false;
    dl_completion_  = nullptr;
    dl_abort_code_  = nullptr;
}

void Master::on_sdo_download_response(uint16_t station, uint8_t cmd,
                                      uint16_t index, uint8_t sub,
                                      uint32_t abort_code) noexcept {
    if (!dl_in_flight_) return;
    if (station != dl_station_ || index != dl_index_ || sub != dl_sub_) return;
    // CoE: cmd 0x60 = download confirm; cmd 0x80 = abort.
    if ((cmd & 0xE0) == 0x60) {
        if (dl_completion_) dl_completion_->store(1, std::memory_order_release);
    } else {
        if (dl_completion_) dl_completion_->store(2, std::memory_order_release);
        if (dl_abort_code_) *dl_abort_code_ = abort_code;
    }
    dl_in_flight_   = false;
    dl_completion_  = nullptr;
    dl_abort_code_  = nullptr;
}

bool Master::enqueue_sdo_upload(const SdoRequest& req) noexcept {
    if (!req.out_data || !req.out_bytes || !req.completion_state) return false;
    const size_t head = sdo_queue_head_.load(std::memory_order_acquire);
    const size_t tail = sdo_queue_tail_.load(std::memory_order_relaxed);
    if (((tail + 1) % SDO_QUEUE_DEPTH) == head) {
        // Queue full — caller is firing faster than the master can drain.
        return false;
    }
    sdo_queue_[tail] = req;
    req.completion_state->store(0, std::memory_order_release);
    sdo_queue_tail_.store((tail + 1) % SDO_QUEUE_DEPTH,
                          std::memory_order_release);
    return true;
}

size_t Master::sdo_queue_depth() const noexcept {
    const size_t head = sdo_queue_head_.load(std::memory_order_acquire);
    const size_t tail = sdo_queue_tail_.load(std::memory_order_acquire);
    return (tail + SDO_QUEUE_DEPTH - head) % SDO_QUEUE_DEPTH;
}

void Master::service_sdo_queue() noexcept {
    // If a request is in flight, poll it.
    if (sdo_request_in_flight_) {
        uint8_t buf[4]{};
        uint8_t bytes = 0;
        uint32_t abort = 0;
        const int r = upload_sdo_poll(buf, &bytes, &abort);
        if (r == 0) return;  // still pending
        // Copy result into caller's slot and signal completion.
        if (r > 0 && sdo_active_request_.out_data) {
            for (uint8_t i = 0; i < 4; ++i) sdo_active_request_.out_data[i] = buf[i];
        }
        if (sdo_active_request_.out_bytes) *sdo_active_request_.out_bytes = (r > 0 ? bytes : 0);
        if (sdo_active_request_.abort_code) *sdo_active_request_.abort_code = (r > 0 ? 0 : abort);
        if (sdo_active_request_.completion_state) {
            sdo_active_request_.completion_state->store(
                r > 0 ? 1u : 2u, std::memory_order_release);
        }
        sdo_request_in_flight_ = false;
        sdo_active_request_ = SdoRequest{};
    }
    // No in-flight: kick the head of the queue.
    const size_t head = sdo_queue_head_.load(std::memory_order_acquire);
    const size_t tail = sdo_queue_tail_.load(std::memory_order_acquire);
    if (head == tail) return;  // empty
    SdoRequest req = sdo_queue_[head];
    sdo_queue_head_.store((head + 1) % SDO_QUEUE_DEPTH,
                          std::memory_order_release);
    if (!upload_sdo_start(req.station, req.index, req.sub, req.timeout_us)) {
        // Couldn't claim slot (race) — push back? Easiest: signal error.
        if (req.completion_state) {
            req.completion_state->store(2u, std::memory_order_release);
        }
        if (req.out_bytes) *req.out_bytes = 0;
        if (req.abort_code) *req.abort_code = 0;
        return;
    }
    sdo_active_request_ = req;
    sdo_request_in_flight_ = true;
}

int Master::upload_sdo_poll(uint8_t out[4], uint8_t* out_bytes,
                            uint32_t* abort_code) noexcept {
    if (!out || !out_bytes) return -1;
    const auto s = upload_state_.load(std::memory_order_acquire);
    if (s == UploadState::Done) {
        *out_bytes = upload_result_bytes_;
        std::memcpy(out, upload_result_, 4);
        if (abort_code) *abort_code = 0;
        upload_segment_request_inflight_ = false;
        upload_state_.store(UploadState::Idle, std::memory_order_release);
        upload_busy_.store(false, std::memory_order_release);
        return 1;
    }
    if (s == UploadState::Error) {
        *out_bytes = 0;
        if (abort_code) *abort_code = upload_abort_code_;
        upload_segment_request_inflight_ = false;
        upload_state_.store(UploadState::Idle, std::memory_order_release);
        upload_busy_.store(false, std::memory_order_release);
        return -1;
    }
    if (now_us() > upload_deadline_us_) {
        *out_bytes = 0;
        if (abort_code) *abort_code = 0;
        upload_segment_request_inflight_ = false;
        upload_state_.store(UploadState::Idle, std::memory_order_release);
        upload_busy_.store(false, std::memory_order_release);
        return -1;
    }
    return 0; // still pending
}

size_t Master::upload_sdo_segmented(uint16_t station_addr,
                                    uint16_t index, uint8_t sub,
                                    uint8_t* out, size_t cap,
                                    uint32_t timeout_us,
                                    uint32_t* abort_code) noexcept {
    if (!out || cap == 0) return 0;
    if (cap > UPLOAD_RESULT_CAP) cap = UPLOAD_RESULT_CAP;

    bool expected = false;
    if (!upload_busy_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel)) {
        if (abort_code) *abort_code = 0;
        return 0;
    }

    extern size_t build_sdo_upload_request(uint8_t*, size_t,
                                           const uint8_t[6], const uint8_t[6],
                                           uint16_t, uint16_t, uint8_t,
                                           uint8_t&);
    uint8_t frame[128];
    const size_t n = build_sdo_upload_request(frame, sizeof(frame),
        BROADCAST_MAC, SRC_MAC, station_addr, index, sub, sdo_counter_);
    if (!n || !send_frame(frame, n)) {
        upload_busy_.store(false, std::memory_order_release);
        if (abort_code) *abort_code = 0;
        return 0;
    }
    stats_.sdo_tx.fetch_add(1, std::memory_order_relaxed);

    upload_station_          = station_addr;
    upload_index_            = index;
    upload_sub_              = sub;
    upload_result_bytes_     = 0;
    upload_total_expected_   = 0;
    upload_abort_code_       = 0;
    upload_segment_toggle_   = false;
    upload_segment_request_inflight_ = false;
    upload_deadline_us_      = now_us() + timeout_us;
    upload_state_.store(UploadState::Pending, std::memory_order_release);

    for (;;) {
        if (now_us() > upload_deadline_us_) {
            upload_abort_code_ = 0;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Error, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            if (abort_code) *abort_code = 0;
            return 0;
        }
        const auto s = upload_state_.load(std::memory_order_acquire);
        if (s == UploadState::Done) {
            const size_t take = (upload_result_bytes_ < cap)
                ? upload_result_bytes_ : cap;
            std::memcpy(out, upload_result_, take);
            if (abort_code) *abort_code = 0;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Idle, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            return take;
        }
        if (s == UploadState::Error) {
            if (abort_code) *abort_code = upload_abort_code_;
            upload_segment_request_inflight_ = false;
            upload_state_.store(UploadState::Idle, std::memory_order_release);
            upload_busy_.store(false, std::memory_order_release);
            return 0;
        }
        if (kernel::g_scheduler_ptr) kernel::g_scheduler_ptr->yield(0);
    }
}

// Task 1.3 — expedited-upload helper: read one 32-bit field from OD `index`
// sub `sub`. Returns true iff the read completes within the per-call timeout
// and the response carries 4 bytes. Packs little-endian.
static inline bool upload_u8(Master& m, uint16_t station,
                             uint16_t index, uint8_t sub,
                             uint8_t* out) noexcept {
    return upload_scalar<uint8_t>(m, station, index, sub, out, 1);
}
static inline bool upload_u16(Master& m, uint16_t station,
                              uint16_t index, uint8_t sub,
                              uint16_t* out) noexcept {
    return upload_scalar<uint16_t>(m, station, index, sub, out, 2);
}
static inline bool upload_u32(Master& m, uint16_t station,
                              uint16_t index, uint8_t sub,
                              uint32_t* out) noexcept {
    return upload_scalar<uint32_t>(m, station, index, sub, out, 4);
}

bool Master::probe_slave_identity(uint16_t station_addr,
                                  uint32_t expect_vendor,
                                  uint32_t expect_product,
                                  uint32_t expect_revision,
                                  uint32_t* out_vendor,
                                  uint32_t* out_product,
                                  uint32_t* out_revision,
                                  uint32_t* out_serial) noexcept {
    uint32_t vid = 0, pid = 0, rev = 0, ser = 0;
    if (!upload_u32(*this, station_addr, 0x1018, 0x01, &vid)) return false;
    if (!upload_u32(*this, station_addr, 0x1018, 0x02, &pid)) return false;
    if (!upload_u32(*this, station_addr, 0x1018, 0x03, &rev)) return false;
    // Serial is optional — don't fail the probe if the slave doesn't publish it.
    (void)upload_u32(*this, station_addr, 0x1018, 0x04, &ser);

    if (out_vendor)   *out_vendor   = vid;
    if (out_product)  *out_product  = pid;
    if (out_revision) *out_revision = rev;
    if (out_serial)   *out_serial   = ser;

    if (expect_vendor   && vid != expect_vendor)   return false;
    if (expect_product  && pid != expect_product)  return false;
    if (expect_revision && rev != expect_revision) return false;
    return true;
}

bool Master::probe_encoder_resolution(uint16_t station_addr,
                                      uint32_t* out_increments,
                                      uint32_t* out_motor_revs,
                                      uint32_t* out_counts_per_rev) noexcept {
    uint32_t inc = 0, revs = 0;
    if (!upload_u32(*this, station_addr, 0x608F, 0x01, &inc))  return false;
    if (!upload_u32(*this, station_addr, 0x608F, 0x02, &revs)) return false;
    if (revs == 0) return false;
    if (out_increments)     *out_increments     = inc;
    if (out_motor_revs)     *out_motor_revs     = revs;
    if (out_counts_per_rev) *out_counts_per_rev = inc / revs;
    return true;
}

size_t Master::push_software_limits(uint16_t station_addr,
                                    int32_t neg_limit,
                                    int32_t pos_limit) noexcept {
    // 0x607D:1 min, 0x607D:2 max. Both i32. CiA-402 §7.4.4 — these are
    // interpreted relative to the 0x607C home offset, so homing should
    // anchor the axis before these land.
    uint8_t data[4] = {};
    size_t sent = 0;
    put_u32_le(data, static_cast<uint32_t>(neg_limit));
    if (send_sdo_download(station_addr, 0x607D, 0x01, data, 4)) ++sent;
    put_u32_le(data, static_cast<uint32_t>(pos_limit));
    if (send_sdo_download(station_addr, 0x607D, 0x02, data, 4)) ++sent;
    return sent;
}

bool Master::run_homing_sequence(uint16_t station_addr,
                                 int8_t   method,
                                 uint32_t fast_speed_cps,
                                 uint32_t slow_speed_cps,
                                 uint32_t accel_cps2,
                                 int32_t  offset_counts,
                                 uint16_t hardstop_torque_permille,
                                 uint32_t timeout_ms) noexcept {
    // Stage 1 — write mode = Homing (0x06). Drive will accept controlword
    // bit 4 (start) only in this mode.
    uint8_t mode_bytes[1] = { 0x06 };
    if (!send_sdo_download(station_addr, 0x6060, 0x00, mode_bytes, 1)) return false;

    // Stage 2 — push the plan parameters.
    const size_t pushed = push_homing_params(station_addr,
        method, fast_speed_cps, slow_speed_cps, accel_cps2,
        offset_counts, hardstop_torque_permille);
    if (pushed == 0) return false;

    // Stage 3 — pulse controlword bit 4. We write 0x001F (enable-operation
    // + start_homing bit) directly; in a full motion-kernel integration
    // this would be a synchronised pulse next to the CiA-402 state
    // walker. One-shot write for now — the drive latches the rising
    // edge of bit 4 and proceeds.
    uint8_t cw[2] = {};
    put_u16_le(cw, 0x001F);
    if (!send_sdo_download(station_addr, 0x6040, 0x00, cw, 2)) return false;

    // Stage 4 — poll statusword (0x6041) for bit 12 (Homing Attained) or
    // bit 13 (Homing Error). Runs via upload_sdo in 10 ms intervals up
    // to `timeout_ms`. This is a blocking CLI-compatible call; a
    // production interpreter would hook the poll into its cycle tick.
    const uint64_t deadline = now_us() + static_cast<uint64_t>(timeout_ms) * 1000ULL;
    while (now_us() < deadline) {
        uint16_t sw = 0;
        if (upload_u16(*this, station_addr, 0x6041, 0x00, &sw)) {
            if (sw & (1u << 13)) return false; // Homing Error
            if (sw & (1u << 12)) break;        // Homing Attained
        }
        if (kernel::g_scheduler_ptr)
            kernel::g_scheduler_ptr->yield(0);
    }
    if (now_us() >= deadline) return false;

    // Stage 5 — clear controlword bit 4 so a subsequent homing cycle can
    // re-edge. Leave drive in Homing mode; caller restores as needed.
    put_u16_le(cw, 0x000F);
    (void)send_sdo_download(station_addr, 0x6040, 0x00, cw, 2);
    return true;
}

bool Master::probe_homing_methods(uint16_t station_addr,
                                  int8_t* out_methods, uint8_t max,
                                  uint8_t* out_count) noexcept {
    if (!out_methods || !out_count) return false;
    *out_count = 0;

    // Subindex 0 is the count (u8 per spec; uploaded via u32 for simplicity).
    uint8_t count = 0;
    if (!upload_u8(*this, station_addr, 0x60E3, 0x00, &count)) return false;
    const uint8_t n   = count;
    const uint8_t lim = (n < max) ? n : max;

    for (uint8_t i = 0; i < lim; ++i) {
        uint8_t v = 0;
        if (!upload_u8(*this, station_addr, 0x60E3,
                        static_cast<uint8_t>(i + 1), &v)) {
            return false;
        }
        // Methods are i8 per CiA-402. Sign-extend the low byte so the
        // Teknic -1 / -2 hardstop variants decode correctly.
        out_methods[i] = static_cast<int8_t>(v);
    }
    *out_count = lim;
    return true;
}

size_t Master::push_homing_params(uint16_t station_addr,
                                  int8_t   method,
                                  uint32_t fast_speed_cps,
                                  uint32_t slow_speed_cps,
                                  uint32_t accel_cps2,
                                  int32_t  offset_counts,
                                  uint16_t hardstop_torque_permille) noexcept {
    size_t sent = 0;
    uint8_t data[4] = {};

    data[0] = static_cast<uint8_t>(method);                              // 0x6098 (i8)
    if (send_sdo_download(station_addr, 0x6098, 0x00, data, 1)) ++sent;

    put_u32_le(data, fast_speed_cps);                                    // 0x6099:1 (u32)
    if (send_sdo_download(station_addr, 0x6099, 0x01, data, 4)) ++sent;

    put_u32_le(data, slow_speed_cps);                                    // 0x6099:2 (u32)
    if (send_sdo_download(station_addr, 0x6099, 0x02, data, 4)) ++sent;

    put_u32_le(data, accel_cps2);                                        // 0x609A (u32)
    if (send_sdo_download(station_addr, 0x609A, 0x00, data, 4)) ++sent;

    put_u32_le(data, static_cast<uint32_t>(offset_counts));              // 0x607C (i32)
    if (send_sdo_download(station_addr, 0x607C, 0x00, data, 4)) ++sent;

    put_u16_le(data, hardstop_torque_permille);                          // 0x216B (u16)
    if (send_sdo_download(station_addr, 0x216B, 0x00, data, 2)) ++sent;

    return sent;
}

size_t Master::broadcast_init_state() noexcept {
    // One BWR (broadcast write) to AL_CTRL with state byte = AL_INIT.
    FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(),
                    BROADCAST_MAC, SRC_MAC);
    uint8_t state[2] = { AL_INIT, 0 };
    if (!fb.add_datagram(Cmd::BWR, dgram_idx_++, REG_AL_CTRL, state, 2)) return 0;
    return send_frame(tx_scratch_.data(), fb.finalize()) ? fb.finalize() : 0;
}

size_t Master::configure_sm_watchdog(uint16_t station_addr,
                                     uint16_t timeout_ms) noexcept {
    // ESC registers (ETG.1000 §6.5). Divider counts at 40 ns — loading
    // `divider = 2500` gives a 100 µs timebase. `timeout_ms * 10` then
    // lands the timeout directly in those ticks. Hard-cap to fit u16.
    constexpr uint16_t REG_SM_WATCHDOG_DIVIDER = 0x0400;
    constexpr uint16_t REG_SM_WD_TIME_PD       = 0x0410;
    constexpr uint16_t REG_SM_WD_TIME_SM       = 0x0420;

    const uint16_t divider = 2500;                           // 100 µs tick
    uint32_t ticks = static_cast<uint32_t>(timeout_ms) * 10; // at 100 µs
    if (ticks > 0xFFFF) ticks = 0xFFFF;
    const uint16_t t = static_cast<uint16_t>(ticks);

    auto write_u16 = [&](uint16_t reg, uint16_t val) -> bool {
        FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(),
                        BROADCAST_MAC, SRC_MAC);
        uint8_t data[2] = {};
        put_u16_le(data, val);
        const uint32_t addr = (static_cast<uint32_t>(station_addr) << 16) | reg;
        if (!fb.add_datagram(Cmd::FPWR, dgram_idx_++, addr, data, 2)) return false;
        return send_frame(tx_scratch_.data(), fb.finalize());
    };

    size_t sent = 0;
    if (write_u16(REG_SM_WATCHDOG_DIVIDER, divider)) ++sent;
    if (write_u16(REG_SM_WD_TIME_PD,       t))       ++sent;
    if (write_u16(REG_SM_WD_TIME_SM,       t))       ++sent;
    return sent;
}

// Task 3.3 — Distributed Clocks SYNC0 configuration via raw ESC register
// FPWRs. These live in the EtherCAT Slave Controller address space, not the
// mailbox — so we build one FPWR datagram per register and send directly.
// Returns the number of frames successfully queued.
size_t Master::configure_dc_sync0(uint16_t station_addr,
                                  uint16_t assign_activate,
                                  uint32_t sync0_cycle_time_ns,
                                  uint32_t shift_time_ns) noexcept {
    // ESC register offsets (ETG.1000, 6.7). All little-endian.
    constexpr uint16_t REG_DC_SYNC_ACT      = 0x0980; // u16 AssignActivate
    constexpr uint16_t REG_DC_SYNC0_CYCLE   = 0x09A0; // u32 ns
    constexpr uint16_t REG_DC_SYNC0_START   = 0x09A4; // u32 ns (shift offset)
    constexpr uint16_t REG_DC_SYNC1_CYCLE   = 0x09A8; // u32 ns (0 = disable)

    auto write_fpwr = [&](uint16_t reg, const uint8_t* data, uint16_t len) -> bool {
        FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(),
                        BROADCAST_MAC, SRC_MAC);
        const uint32_t addr = (static_cast<uint32_t>(station_addr) << 16) | reg;
        if (!fb.add_datagram(Cmd::FPWR, dgram_idx_++, addr, data, len)) return false;
        return send_frame(tx_scratch_.data(), fb.finalize());
    };

    size_t sent = 0;
    uint8_t buf4[4] = {};

    // SYNC0 cycle time (ns).
    put_u32_le(buf4, sync0_cycle_time_ns);
    if (write_fpwr(REG_DC_SYNC0_CYCLE, buf4, 4)) ++sent;

    // SYNC0 start time must be an absolute DC system time. Read the slave's
    // local DC clock, then round forward to the next cycle boundary and add
    // the requested shift margin.
    uint8_t dc_raw[8] = {};
    if (!read_esc_register(station_addr, 0x0910, dc_raw, sizeof(dc_raw), 100000)) {
        return sent;
    }
    uint64_t dc_now = 0;
    for (uint8_t i = 0; i < sizeof(dc_raw); ++i) {
        dc_now |= static_cast<uint64_t>(dc_raw[i]) << (8u * i);
    }
    const uint64_t cycle = sync0_cycle_time_ns ? sync0_cycle_time_ns : 1u;
    const uint64_t start = ((dc_now / cycle) + 1u) * cycle + shift_time_ns;
    put_u32_le(buf4, static_cast<uint32_t>(start & 0xFFFFFFFFu));
    if (write_fpwr(REG_DC_SYNC0_START, buf4, 4)) ++sent;

    // SYNC1 disabled — cycle time 0.
    put_u32_le(buf4, 0);
    if (write_fpwr(REG_DC_SYNC1_CYCLE, buf4, 4)) ++sent;

    // DC Sync Activate — AssignActivate from ESI (0x0300 for ClearPath-EC
    // DC-Sync OpMode). Must come last so the other registers are already
    // programmed when the sync pulse generator turns on.
    uint8_t buf2[2] = {};
    put_u16_le(buf2, assign_activate);
    if (write_fpwr(REG_DC_SYNC_ACT, buf2, 2)) ++sent;

    // Latch dc_enabled on the matching slave so the cyclic drift sampler
    // knows to round-robin this slave. Done last so a partial config
    // (failed FPWR) doesn't enable feedback against a half-armed slave.
    for (size_t i = 0; i < slave_count_; ++i) {
        if (slaves_[i].station_addr == station_addr) {
            slaves_[i].dc_enabled       = true;
            slaves_[i].dc_have_baseline = false;
            slaves_[i].dc_last_offset_ns = 0;
            break;
        }
    }

    return sent;
}

void Master::service_sdo_upload() noexcept {
    const auto st = upload_state_.load(std::memory_order_acquire);
    if (st != UploadState::Pending && st != UploadState::FetchSegment) return;

    if (now_us() > upload_deadline_us_) {
        upload_abort_code_ = 0;
        upload_segment_request_inflight_ = false;
        upload_state_.store(UploadState::Error, std::memory_order_release);
        return;
    }

    // When in FetchSegment, alternate each cycle between sending a segment
    // request (FPWR SM0) and polling SM1. Cheap: use toggle_ state — bit 0
    // of the cycle counter is fine since we're single-threaded here.
    if (st == UploadState::FetchSegment && !upload_segment_request_inflight_) {
        // First segment request after initiate response.
        extern size_t build_sdo_upload_segment_request(uint8_t*, size_t,
                                                       const uint8_t[6], const uint8_t[6],
                                                       uint16_t, bool, uint8_t&);
        uint8_t frame[128];
        const size_t n = build_sdo_upload_segment_request(frame, sizeof(frame),
            BROADCAST_MAC, SRC_MAC, upload_station_,
            upload_segment_toggle_, sdo_counter_);
        if (n && send_frame(frame, n)) upload_segment_request_inflight_ = true;
        // Fall through to also emit an FPRD so we can catch the response
        // in the same cycle if the slave is fast.
    }

    // Always emit an FPRD SM1 — it's how responses come back.
    FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(), BROADCAST_MAC, SRC_MAC);
    uint8_t pad[SDO_MAILBOX_LEN] = {};
    const uint32_t addr = (static_cast<uint32_t>(upload_station_) << 16)
                        | SM1_MAILBOX_IN_OFFS;
    if (!fb.add_datagram(Cmd::FPRD, dgram_idx_++, addr, pad, SDO_MAILBOX_LEN)) return;
    send_frame(tx_scratch_.data(), fb.finalize());
}

void Master::service_esc_read() noexcept {
    const auto st = esc_read_state_.load(std::memory_order_acquire);
    if (st != EscReadState::Pending) return;
    if (now_us() > esc_read_deadline_us_) {
        esc_read_state_.store(EscReadState::Error,
                              std::memory_order_release);
        return;
    }

    FrameBuilder fb(tx_scratch_.data(), tx_scratch_.size(),
                    BROADCAST_MAC, SRC_MAC);
    uint8_t pad[8] = {};
    const uint32_t addr = (static_cast<uint32_t>(esc_read_station_) << 16)
                        | esc_read_reg_;
    if (!fb.add_datagram(Cmd::FPRD, dgram_idx_++, addr, pad, esc_read_len_)) {
        return;
    }
    send_frame(tx_scratch_.data(), fb.finalize());
}

bool Master::read_esc_register(uint16_t station_addr, uint16_t reg,
                               uint8_t* out, uint8_t len,
                               uint32_t timeout_us) noexcept {
    if (!out || len == 0 || len > sizeof(esc_read_buf_)) return false;
    bool expected = false;
    if (!esc_read_busy_.compare_exchange_strong(expected, true,
                                                std::memory_order_acq_rel)) {
        return false;
    }
    esc_read_station_ = station_addr;
    esc_read_reg_ = reg;
    esc_read_len_ = len;
    std::memset(esc_read_buf_, 0, sizeof(esc_read_buf_));
    esc_read_deadline_us_ = now_us() + timeout_us;
    esc_read_state_.store(EscReadState::Pending, std::memory_order_release);

    for (;;) {
        const auto s = esc_read_state_.load(std::memory_order_acquire);
        if (s == EscReadState::Done) {
            std::memcpy(out, esc_read_buf_, len);
            esc_read_state_.store(EscReadState::Idle, std::memory_order_release);
            esc_read_busy_.store(false, std::memory_order_release);
            return true;
        }
        if (s == EscReadState::Error) {
            esc_read_state_.store(EscReadState::Idle, std::memory_order_release);
            esc_read_busy_.store(false, std::memory_order_release);
            return false;
        }
        if (kernel::g_scheduler_ptr) kernel::g_scheduler_ptr->yield(0);
    }
}

bool Master::send_sdo_download(uint16_t station_addr, uint16_t index,
                               uint8_t sub, const uint8_t* data,
                               uint8_t len_bytes) noexcept {
    // We need the signature from pdo.hpp, but don't want a cyclic include; so
    // we declare the builder here locally.
    extern size_t build_sdo_download(uint8_t*, size_t,
                                     const uint8_t[6], const uint8_t[6],
                                     uint16_t, uint16_t, uint8_t,
                                     const uint8_t*, uint8_t, uint8_t&);
    uint8_t frame[128];
    size_t n = build_sdo_download(frame, sizeof(frame),
                                  BROADCAST_MAC, SRC_MAC,
                                  station_addr, index, sub,
                                  data, len_bytes, sdo_counter_);
    if (!n) return false;
    if (!send_frame(frame, n)) return false;
    stats_.sdo_tx.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void Master::advance_state() {
    // Kept for compatibility — step_esm() now owns all state logic.
    step_esm();
}

void Master::service_dc_drift() noexcept {
    // ESC reg 0x0920: SYNC0 actual time — local DC system time (ns) at
    // which the slave's last SYNC0 pulse fired. Compared against the
    // master's wall clock, the offset (master_ns − slave_0x0920) should
    // stay constant if the two clocks track. Drift between consecutive
    // samples for the SAME slave is the signal we monitor.
    constexpr uint16_t REG_DC_SYNC0_TIME = 0x0920;

    // Don't re-sample faster than the configured cadence. A blocking
    // ESC read costs up to 100 ms wall-clock in the worst case (timeout
    // arg below), so we want this to be rare relative to the cycle.
    if (slave_count_ == 0 || dc_sync_fault_.load(std::memory_order_relaxed)) return;
    const uint64_t now = now_us();
    if (now - last_dc_sample_us_ < dc_drift_check_period_us_) return;
    last_dc_sample_us_ = now;

    // Pick the next DC-enabled slave round-robin. Caps the per-cycle cost
    // at one ESC read regardless of how many slaves are on the bus.
    const size_t start = dc_round_robin_idx_;
    size_t picked = MAX_SLAVES;
    for (size_t step = 0; step < slave_count_; ++step) {
        const size_t idx = (start + step) % slave_count_;
        if (slaves_[idx].dc_enabled && slaves_[idx].present) {
            picked = idx;
            dc_round_robin_idx_ = (idx + 1) % slave_count_;
            break;
        }
    }
    if (picked == MAX_SLAVES) return;

    auto& s = slaves_[picked];

    // 10 ms timeout: well under the 100 ms cadence so a stuck slave
    // can't stall the next sample, and well over the median ESC FPRD
    // round-trip (single-digit µs on a quiet bus).
    uint8_t raw[8] = {};
    if (!read_esc_register(s.station_addr, REG_DC_SYNC0_TIME, raw, sizeof(raw), 10000)) {
        return;
    }
    uint64_t slave_ns = 0;
    for (uint8_t i = 0; i < sizeof(raw); ++i) {
        slave_ns |= static_cast<uint64_t>(raw[i]) << (8u * i);
    }

    const uint64_t master_ns = now_us() * 1000ULL;
    // Offset between master wall clock and slave's last-SYNC0 timestamp.
    // The absolute value is meaningless (start of epoch differs), but
    // its drift across consecutive samples is the DC slope error.
    const uint64_t offset = master_ns - slave_ns;

    stats_.dc_sync_samples.fetch_add(1, std::memory_order_relaxed);

    if (!s.dc_have_baseline) {
        // First sample for this slave is the baseline; drift requires two.
        s.dc_last_offset_ns  = offset;
        s.dc_have_baseline   = true;
        return;
    }

    const int64_t drift_ns = static_cast<int64_t>(offset) - static_cast<int64_t>(s.dc_last_offset_ns);
    s.dc_last_offset_ns = offset;
    last_dc_drift_ns_.store(drift_ns, std::memory_order_release);

    const uint64_t mag = drift_ns < 0 ? static_cast<uint64_t>(-drift_ns)
                                      : static_cast<uint64_t>(drift_ns);

    // Track peak observed drift for diagnostics. Relaxed CAS loop — only
    // one writer (this thread), readers are cosmetic.
    uint64_t prev_max = stats_.dc_drift_max_ns.load(std::memory_order_relaxed);
    while (mag > prev_max
           && !stats_.dc_drift_max_ns.compare_exchange_weak(prev_max, mag,
                                                            std::memory_order_relaxed)) {
    }

    // Single-line periodic log so a CI run can `grep "dc_drift"` and watch
    // the bus settle. Cheap (one early_uart_puts per 100 ms per master).
    {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "[ec%d] dc_drift slave=0x%04x ns=%lld\n",
            id_, (unsigned)s.station_addr, (long long)drift_ns);
        early_uart_puts(buf);
    }

    // Phase-shift compensation. Compute a gentle correction toward the
    // slave's DC clock and stage it for run_loop to apply on the next
    // iteration. Positive drift means master got ahead — correction is
    // positive (lengthen next cycle). Gain is 1/8 of the observed drift
    // so a single outlier doesn't slew the master phase, and the result
    // is hard-capped at ±DC_PHASE_MAX_ADJUST_US (5 µs) — a fraction of
    // the 250 µs cycle, far less than the slave's SM watchdog can
    // tolerate. Sampler runs at ~10 Hz so the correction tracks ~80
    // ns/sample of residual drift before each new sample lands.
    constexpr int32_t kPhaseGainShift  = 3;          // divide by 8
    constexpr int32_t kPhaseMaxAdjustUs = 5;
    int32_t adj_us =
        static_cast<int32_t>((drift_ns >> kPhaseGainShift) / 1000);
    if (adj_us >  kPhaseMaxAdjustUs) adj_us =  kPhaseMaxAdjustUs;
    if (adj_us < -kPhaseMaxAdjustUs) adj_us = -kPhaseMaxAdjustUs;
    if (adj_us != 0) {
        dc_phase_adjust_us_.store(adj_us, std::memory_order_release);
    }

    if (mag > dc_drift_threshold_ns_) {
        if (++consecutive_drift_misses_ >= CONSECUTIVE_DRIFT_THRESHOLD
            && !dc_sync_fault_.load(std::memory_order_relaxed)) {
            dc_sync_fault_.store(true, std::memory_order_release);
            stats_.dc_sync_trips.fetch_add(1, std::memory_order_relaxed);
            if (!dc_sync_fault_logged_) {
                dc_sync_fault_logged_ = true;
                char rbuf[96];
                kernel::util::k_snprintf(rbuf, sizeof(rbuf),
                    "DC sync drift x%u (last=%lld ns slave=0x%04x)",
                    (unsigned)consecutive_drift_misses_,
                    (long long)drift_ns, (unsigned)s.station_addr);
                trip_fault(rbuf);
            }
        }
    } else {
        consecutive_drift_misses_ = 0;
    }
}

void Master::run_loop() {
    auto* t = kernel::g_platform->get_timer_ops();
    uint64_t next_us = t->get_system_time_us();

    // Boot-time banner — CI greps for "ec%d cycle=...us" to enforce the
    // ClearPath-EC 250 µs minimum. Keep the token "cycle=" and the "us"
    // suffix stable; the workflow regex depends on them.
    {
        char banner[64];
        kernel::util::k_snprintf(banner, sizeof(banner),
            "[ec%d] cycle=%uus (min 250, must be multiple of 250)\n",
            id_, period_us_);
        early_uart_puts(banner);
    }

    // Initial discovery pass: one BRD. On a real bus we'd parse WKC here.
    discover_slaves();

    for (;;) {
        // Snapshot epoch fence — bumped before any per-cycle write so
        // operator-side readers can detect a racing cycle. See header.
        const uint64_t epoch_now = snapshot_epoch_.fetch_add(
            1, std::memory_order_release) + 1;

        // Per-slave presence sweep. A slave that hasn't contributed an RX
        // response (FPRD AL_STATUS or LRW PDO) for more than kAbsentCycles
        // is treated as disconnected. At 250 µs cycles the threshold gives
        // ~25 ms of grace before a "lost" event latches — long enough to
        // ride out a single dropped frame, short enough that a cable yank
        // surfaces to the operator before motion drifts further. The first
        // few cycles after boot are skipped so slaves get a chance to
        // respond before we'd flag them.
        constexpr uint64_t kAbsentCycles = 100;
        constexpr uint64_t kBootGrace    = 50;
        if (epoch_now > kBootGrace) {
            for (size_t i = 0; i < slave_count_; ++i) {
                auto& sl = slaves_[i];
                const bool seen_recently =
                    (epoch_now - sl.last_seen_cycle) < kAbsentCycles;
                if (sl.present && !seen_recently) {
                    sl.present = false;
                    sl.presence_lost_cycle = epoch_now;
                    sl.presence_loss_events++;
                    char msg[96];
                    kernel::util::k_snprintf(msg, sizeof(msg),
                        "[ec%d] slave 0x%04x lost contact (cycle %llu)\n",
                        id_,
                        static_cast<unsigned>(sl.station_addr),
                        static_cast<unsigned long long>(epoch_now));
                    early_uart_puts(msg);
                } else if (!sl.present && seen_recently) {
                    sl.present = true;
                    char msg[96];
                    kernel::util::k_snprintf(msg, sizeof(msg),
                        "[ec%d] slave 0x%04x reconnected (cycle %llu)\n",
                        id_,
                        static_cast<unsigned>(sl.station_addr),
                        static_cast<unsigned long long>(epoch_now));
                    early_uart_puts(msg);
                }
            }
        }

        // Jitter sample at cycle top — tracks inter-cycle interval vs period.
        auto& jt = (id_ == 0) ? diag::rt::ecat_a : diag::rt::ecat_b;
        jt.sample(t->get_system_time_ns());

        const uint64_t cycle_start_us = t->get_system_time_us();
        next_us += period_us_;
        // DC phase-shift compensation: apply any pending adjustment from
        // service_dc_drift exactly once per cycle. Positive nudges the next
        // cycle's deadline forward (slow down to let the slave catch up),
        // negative pulls it back (speed up). The sampler caps the magnitude
        // so a one-cycle skew can't blow the master period out of spec.
        const int32_t phase_adj_us =
            dc_phase_adjust_us_.exchange(0, std::memory_order_acquire);
        if (phase_adj_us != 0) {
            const int64_t adj_signed = phase_adj_us;
            // next_us is uint64_t; clamp the shift so a negative adj can't
            // wrap when next_us is small (early boot).
            if (adj_signed < 0 &&
                static_cast<uint64_t>(-adj_signed) > next_us) {
                next_us = 0;
            } else {
                next_us = static_cast<uint64_t>(
                    static_cast<int64_t>(next_us) + adj_signed);
            }
        }

        if (slave_count_ == 0) {
            const uint64_t t0 = t->get_system_time_us();
            send_probe(); // Keep the link lit during Init-on-empty-bus.
            hist_probe_.record(t->get_system_time_us() - t0);
        }

        {
            const uint64_t t0 = t->get_system_time_us();
            step_esm();
            hist_esm_.record(t->get_system_time_us() - t0);
        }

        if (state_.load(std::memory_order_acquire) == State::Op) {
            const uint64_t t0 = t->get_system_time_us();
            cycle_lrw();
            hist_lrw_.record(t->get_system_time_us() - t0);
        }

        // Task 1.2 — if the master has a pending SDO upload, emit one FPRD
        // to SM1 (mailbox-in) this cycle. handle_rx_frame picks up the
        // response and flips upload_state_ to Done / Error. Kept outside
        // the OP gate because SDO mailbox traffic is valid in PREOP and
        // SafeOp too — that's where 1.3/1.4 need it.
        service_sdo_upload();
        service_sdo_download();
        service_esc_read();
        service_sdo_queue();

        // Periodic DC-sync drift sampler. Self-rate-limits to
        // dc_drift_check_period_us_ and only reads one slave per tick, so
        // the cycle budget here is dominated by a single FPRD round-trip
        // when a sample is due (well under POLL_RX_SAFETY_MARGIN_US slack).
        if (state_.load(std::memory_order_acquire) == State::Op) {
            service_dc_drift();
        }

        // Drain whatever the NIC queued, but stop early if we're about to
        // overshoot the cycle deadline — undrained frames stay in the NIC
        // RX queue and get picked up next cycle. A flooded link previously
        // could blow the budget here and trip the deadline-fault latch.
        if (net_) {
            const uint64_t drain_deadline_us =
                next_us > POLL_RX_SAFETY_MARGIN_US ? next_us - POLL_RX_SAFETY_MARGIN_US : 0;
            size_t total = 0;
            while (total < POLL_RX_MAX_PER_CYCLE) {
                if (t->get_system_time_us() >= drain_deadline_us) break;
                const size_t got = net_->poll_rx(&Master::rx_trampoline, this, POLL_RX_BATCH);
                if (got == 0) break;
                total += got;
            }
        }

        stats_.cycles.fetch_add(1, std::memory_order_relaxed);

        // Full-cycle latency: from the top of the iteration to just before
        // wait_until_us. This is the actual work each cycle performs.
        hist_cycle_.record(t->get_system_time_us() - cycle_start_us);

        const uint64_t wait_t0 = t->get_system_time_us();
        if (wait_t0 > next_us) {
            stats_.cycle_deadline_miss.fetch_add(1, std::memory_order_relaxed);
            next_us = wait_t0;
            // Boot grace: the first BOOT_GRACE_CYCLES iterations after
            // the master starts coexist with virtio-gpu scanout setup,
            // hmi DHCP discover, framebuffer clear, ESI blob ingestion,
            // and the rest of the cold-start surge — all of which
            // routinely blow the 250 µs deadline on QEMU TCG. Counting
            // those toward the trip threshold gave every cold boot a
            // "[ec0] FAULT TRIPPED" within seconds, before any operator
            // action. The miss is still recorded in stats so it shows
            // up in `ec` / histograms, but it doesn't accumulate the
            // consecutive-miss counter that latches the fault. After
            // grace, normal CONSECUTIVE_MISS_THRESHOLD applies.
            const uint64_t cycles = stats_.cycles.load(std::memory_order_relaxed);
            if (cycles < BOOT_GRACE_CYCLES) {
                consecutive_misses_ = 0;
            } else if (++consecutive_misses_ >= CONSECUTIVE_MISS_THRESHOLD &&
                       !deadline_fault_.load(std::memory_order_relaxed)) {
                on_deadline_fault();
            }
        } else {
            consecutive_misses_ = 0;
            wait_until_us(next_us);
            hist_wait_.record(t->get_system_time_us() - wait_t0);
        }
    }
}

void Master::on_deadline_fault() noexcept {
    char reason[64];
    kernel::util::k_snprintf(reason, sizeof(reason),
        "deadline miss x%u", (unsigned)consecutive_misses_);
    trip_fault(reason);
}

void Master::trip_fault(const char* reason) noexcept {
    const bool was_already = deadline_fault_.exchange(true, std::memory_order_acq_rel);
    if (!was_already) {
        stats_.deadline_trips.fetch_add(1, std::memory_order_relaxed);
        // Command every CiA-402 servo to QuickStopActive. The next cycle_lrw()
        // pack lifts target_state through Drive::step() into the controlword
        // PDO, so the slaves see CW_CMD_QUICK_STOP without any extra mailbox
        // traffic. Slaves without a CiA-402 PDO layout (rx_controlword_off ==
        // 0xFF — couplers, raw I/O) are left alone.
        for (size_t i = 0; i < slave_count_; ++i) {
            if (slaves_[i].pdo_layout.rx_controlword_off != 0xFF) {
                slaves_[i].drive.target_state = cia402::State::QuickStopActive;
            }
        }
    }
    if (!deadline_fault_logged_) {
        deadline_fault_logged_ = true;
        char buf[128];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "[ec%d] FAULT TRIPPED — %s, QuickStop broadcast\n",
            id_, reason ? reason : "(no reason)");
        early_uart_puts(buf);
    }
}

void Master::thread_entry(void* arg) {
    auto* self = static_cast<Master*>(arg);
    if (!self || !kernel::g_platform) for (;;) asm volatile("wfi");
    self->net_ = kernel::g_platform->get_net_ops(self->nic_idx_);
    self->run_loop();
}

void Master::dump_status(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    const char* sname =
        state_ == State::Init   ? "INIT"   :
        state_ == State::PreOp  ? "PRE-OP" :
        state_ == State::SafeOp ? "SAFE-OP":
        state_ == State::Op     ? "OP"     :
        state_ == State::Fault  ? "FAULT"  : "?";
    char buf[160];
    const char* pname =
        esm_phase_ == EsmPhase::Idle         ? "idle"    :
        esm_phase_ == EsmPhase::RequestWrite ? "req-wr"  :
        esm_phase_ == EsmPhase::PollStatus   ? "poll"    :
        esm_phase_ == EsmPhase::Settled      ? "settled" :
        esm_phase_ == EsmPhase::Error        ? "err"     : "?";
    kernel::util::k_snprintf(buf, sizeof(buf),
        "  ec%d nic=%d state=%s period=%uus cycles=%llu tx=%llu rx=%llu slaves=%u miss=%llu trips=%u dl_fault=%s esm=%s tgt=0x%02x to=%u safeop_gate=%s\n",
        id_, nic_idx_, sname, period_us_,
        (unsigned long long)stats_.cycles.load(std::memory_order_relaxed),
        (unsigned long long)stats_.tx_frames.load(std::memory_order_relaxed),
        (unsigned long long)stats_.rx_frames.load(std::memory_order_relaxed),
        (unsigned)slave_count_,
        (unsigned long long)stats_.cycle_deadline_miss.load(std::memory_order_relaxed),
        (unsigned)stats_.deadline_trips.load(std::memory_order_relaxed),
        deadline_fault_.load(std::memory_order_acquire) ? "LATCHED" : "ok",
        pname,
        (unsigned)esm_target_,
        (unsigned)stats_.esm_timeouts.load(std::memory_order_relaxed),
        allow_safeop_.load(std::memory_order_acquire) ? "open" : "closed");
    uart->puts(buf);
    for (size_t i = 0; i < slave_count_; ++i) {
        const auto& s = slaves_[i];
        if (s.identity_mismatch) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "      al_state[%zu]=0x%02x IDENTITY_MISMATCH vid=0x%08x pid=0x%08x rev=0x%08x\n",
                i, (unsigned)s.current_state,
                (unsigned)s.observed_vid, (unsigned)s.observed_pid, (unsigned)s.observed_rev);
        } else {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "      al_state[%zu]=0x%02x\n", i, (unsigned)s.current_state);
        }
        uart->puts(buf);
    }

    // CiA-402 per-slave status — one line per slave so the operator can see
    // that the drive FSA has tracked the master's requested target_state.
    for (size_t i = 0; i < slave_count_; ++i) {
        const auto& s = slaves_[i];
        const auto& d = s.drive;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "    slave 0x%04x: %s state=%s sw=0x%04x cw=0x%04x tgt=%ld act=%ld\n",
            (unsigned)s.station_addr,
            cia402::mode_name(d.mode_op_display),
            cia402::state_name(d.state),
            (unsigned)d.statusword,
            (unsigned)d.controlword,
            (long)d.target_position,
            (long)d.actual_position);
        uart->puts(buf);
    }
}

void Master::dump_slaves(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    if (slave_count_ == 0) {
        char buf[48];
        kernel::util::k_snprintf(buf, sizeof(buf), "  ec%d nic=%d: no slaves\n", id_, nic_idx_);
        uart->puts(buf);
        return;
    }
    char buf[128];
    for (size_t i = 0; i < slave_count_; ++i) {
        const auto& s = slaves_[i];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  ec%d[%u] addr=0x%04x vid=0x%08x pid=0x%08x state=%s\n",
            id_, (unsigned)i,
            (unsigned)s.station_addr, (unsigned)s.vendor_id,
            (unsigned)s.product_code, al_state_name(s.current_state));
        uart->puts(buf);
    }
}

} // namespace ethercat
