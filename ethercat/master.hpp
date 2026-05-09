// SPDX-License-Identifier: MIT OR Apache-2.0
// EtherCAT master. One `Master` per NIC. Hosts the cyclic thread, the ESM,
// discovery, and the process-data image for LRW.
//
// No DC yet; that comes next phase.

#ifndef ETHERCAT_MASTER_HPP
#define ETHERCAT_MASTER_HPP

#include "esm.hpp"
#include "hal.hpp"
#include "diag/histogram.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace ethercat {

constexpr uint16_t ETHERTYPE_ECAT = 0x88A4;
constexpr size_t   MAX_SLAVES     = 64;
constexpr size_t   PDO_BUF_BYTES  = MAX_SLAVES * PDO_SLOT_BYTES;

enum class State : uint8_t {
    Init = 1,
    PreOp = 2,
    Bootstrap = 3,
    SafeOp = 4,
    Op = 8,
    Fault = 0x10,
};

struct Stats {
    std::atomic<uint64_t> cycles{0};
    std::atomic<uint64_t> tx_frames{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> cycle_deadline_miss{0};
    std::atomic<uint32_t> discovered{0};   // last discovery WKC
    std::atomic<uint32_t> esm_timeouts{0};
    std::atomic<uint32_t> sdo_tx{0};
    std::atomic<uint32_t> deadline_trips{0}; // times the consecutive-miss threshold tripped

    // Distributed Clocks drift telemetry — populated by the periodic ESC
    // reg-0x0920 sampler in run_loop. Peak drift is signed-symmetric
    // (largest |delta| seen between consecutive samples for any slave),
    // tracked as an unsigned magnitude in nanoseconds.
    std::atomic<uint32_t> dc_sync_samples{0};
    std::atomic<uint64_t> dc_drift_max_ns{0};
    std::atomic<uint32_t> dc_sync_trips{0};
};

class Master {
public:
    Master(int id, uint32_t period_us) noexcept
        : id_(id), nic_idx_(id), period_us_(period_us) {}

    int id() const noexcept { return id_; }
    int nic_idx() const noexcept { return nic_idx_; }
    void set_nic_index(int nic_idx) noexcept { nic_idx_ = nic_idx; }
    uint32_t period_us() const noexcept { return period_us_; }
    State state() const noexcept { return state_.load(std::memory_order_acquire); }
    const Stats& stats() const noexcept { return stats_; }

    // Scheduler entry-point — `arg` is `Master*`. Never returns.
    static void thread_entry(void* arg);

    void dump_status(kernel::hal::UARTDriverOps* uart) const;
    void dump_slaves(kernel::hal::UARTDriverOps* uart) const;

    // Public so pdo.cpp can drive SDO writes without circular coupling.
    [[nodiscard]] bool send_sdo_download(uint16_t station_addr, uint16_t index, uint8_t sub,
                                         const uint8_t* data, uint8_t len_bytes) noexcept;

    // Task 1.2 — blocking SDO upload (expedited only, ≤ 4 bytes). Sends an
    // "initiate upload request" into SM0 and polls SM1 via FPRD every cycle
    // until either the response arrives (returns true, fills `out`/`out_bytes`)
    // or `timeout_us` elapses (returns false; `abort_code` contains the slave's
    // SDO abort code when non-zero, 0 on plain timeout).
    //
    // Thread-safety: one upload in flight per Master. Callers on different
    // threads serialise through `upload_busy_`; a concurrent attempt returns
    // false with abort_code=0 immediately.
    [[nodiscard]] bool upload_sdo(uint16_t station_addr, uint16_t index, uint8_t sub,
                                  uint8_t out[4], uint8_t* out_bytes,
                                  uint32_t timeout_us,
                                  uint32_t* abort_code = nullptr) noexcept;

    // Non-blocking upload pair. `upload_sdo_start` returns immediately:
    // true if the request was accepted (one upload in flight per master,
    // same single-slot constraint as `upload_sdo`), false if another
    // transfer is already pending. The caller then polls
    // `upload_sdo_poll` from any thread:
    //   1  = transfer complete (out / out_bytes filled)
    //  -1  = transfer failed (timeout, SDO abort, malformed reply)
    //   0  = still pending — caller is free to do other work
    // Used by callers that want to fire a probe and not block the calling
    // core's scheduler — e.g. boot-time bus_config can issue an SDO read
    // and continue advancing the ESM walker on subsequent cycles instead
    // of stalling the whole bring-up on a single cycle's worth of
    // mailbox round-trip.
    [[nodiscard]] bool upload_sdo_start(uint16_t station_addr, uint16_t index, uint8_t sub,
                                        uint32_t timeout_us) noexcept;
    [[nodiscard]] int  upload_sdo_poll(uint8_t out[4], uint8_t* out_bytes,
                                        uint32_t* abort_code = nullptr) noexcept;

    // Tier 3a: per-slave-parallel-ish queue. Callers submit a complete
    // request (station / index / sub / caller-owned 4-byte result + bytes
    // out + completion_state out) and walk away. The master cycle drains
    // requests one at a time (single mailbox, but no caller-side
    // serialisation), and updates the caller's state atomic when each
    // completes. Boot-time bus_config can fire 8 servos × 5 SDOs upfront
    // and finish in ~5 cycles instead of 40 round-trip-blocking calls.
    // Pending: 0 (queued/in-flight), Done: 1, Error: 2.
    struct SdoRequest {
        uint16_t station        = 0;
        uint16_t index          = 0;
        uint8_t  sub            = 0;
        uint32_t timeout_us     = 0;
        uint8_t* out_data       = nullptr;       // 4 bytes
        uint8_t* out_bytes      = nullptr;
        uint32_t* abort_code    = nullptr;       // nullable
        std::atomic<uint8_t>* completion_state = nullptr; // 0/1/2
    };
    [[nodiscard]] bool enqueue_sdo_upload(const SdoRequest& req) noexcept;
    [[nodiscard]] size_t sdo_queue_depth() const noexcept;

    // Task 1.2 follow-up — segmented SDO upload. Covers responses
    // larger than the 4-byte expedited window: 0x1008 device name,
    // 0x100A firmware version, 0x10F3:6..55 diag-history entries, etc.
    // Returns the number of bytes actually received (clamped to `cap`).
    // `cap` must be >= 16. On SDO abort, returns 0 and fills abort_code.
    // Thread-safety: same single-slot constraint as `upload_sdo`.
    [[nodiscard]] size_t upload_sdo_segmented(uint16_t station_addr,
                                              uint16_t index, uint8_t sub,
                                              uint8_t* out, size_t cap,
                                              uint32_t timeout_us,
                                              uint32_t* abort_code = nullptr) noexcept;

    // Task 1.3 — read the drive's CiA-301 identity object (0x1018:1/2/3).
    // Returns true iff every read succeeded and the values match the caller's
    // expectations (pass 0 to skip a field). On success the 0x1018:4 serial is
    // also uploaded and stored in `out_serial` for logging. Primary purpose:
    // refuse to bring a drive to SafeOp if its {VID, PID, Rev} don't match
    // the TSV fingerprint, catching wiring errors and revision drift.
    [[nodiscard]] bool probe_slave_identity(uint16_t station_addr,
                                            uint32_t expect_vendor,   // 0 = accept any
                                            uint32_t expect_product,  // 0 = accept any
                                            uint32_t expect_revision, // 0 = accept any
                                            uint32_t* out_vendor   = nullptr,
                                            uint32_t* out_product  = nullptr,
                                            uint32_t* out_revision = nullptr,
                                            uint32_t* out_serial   = nullptr) noexcept;

    // Task 1.4 — read the position-encoder resolution object (0x608F:1/2).
    // `out_counts_per_rev = increments / motor_revs`. Returns false iff
    // either sub-index read fails or motor_revs is zero (bogus device).
    [[nodiscard]] bool probe_encoder_resolution(uint16_t station_addr,
                                                uint32_t* out_increments,
                                                uint32_t* out_motor_revs,
                                                uint32_t* out_counts_per_rev) noexcept;

    // Task 2.1 — enumerate the drive's supported homing methods (OD 0x60E3).
    // Sub 0 is the count; subs 1..count each publish one method number.
    // Fills `out_methods[0..min(count, max)-1]` and `*out_count`. Returns
    // true iff subindex 0 and every listed entry read back successfully.
    // Methods can be negative (Teknic's -1/-2 hardstop variants) in
    // addition to the CiA-402 standard 1..35 range.
    [[nodiscard]] bool probe_homing_methods(uint16_t station_addr,
                                            int8_t* out_methods, uint8_t max,
                                            uint8_t* out_count) noexcept;

    // Force every slave on the bus back to AL=Init via one broadcast
    // FPWR on AL_CTRL. Intended for emergency / hot-swap bring-down —
    // the cyclic loop will notice the state mismatch and eventually
    // walk the ESM back up through PreOp/SafeOp. Returns the number of
    // bytes on the wire (1 datagram).
    [[nodiscard]] size_t broadcast_init_state() noexcept;

    // Task: SM watchdog. Write the ESC SM watchdog registers so the
    // slave aborts its state if the master stops sending PDOs for
    // `timeout_ms`. Conservative default (100 ms) per PLAN.md "still
    // open". Four registers:
    //   0x0400 (u16) watchdog divider (base: 40 ns ticks)
    //   0x0410 (u16) watchdog time process data  (in divider units)
    //   0x0420 (u16) watchdog time SM            (in divider units)
    // For a 100 ms timeout at a 100 µs divider (2500 ticks), both
    // counters = 1000.
    [[nodiscard]] size_t configure_sm_watchdog(uint16_t station_addr,
                                               uint16_t timeout_ms) noexcept;

    // Task 3.3 — arm Distributed Clocks SYNC0 on `station_addr`. Writes:
    //   0x09A0 (u32, ns): SYNC0 cycle time (usually = master period)
    //   0x09A4 (u32, ns): SYNC0 start time offset relative to current DC
    //                      system time, includes the shift margin
    //   0x09A8 (u32, ns): SYNC1 cycle time (0 = disabled)
    //   0x0980 (u16):     DC Sync Activate — AssignActivate from ESI
    //                      (0x0300 for ClearPath-EC DC-Sync OpMode)
    // Writes are queued via fire-and-forget FPWR, same contract as
    // `send_sdo_download` — CI should grep the boot banner, not the
    // SDO responses, to confirm completion.
    [[nodiscard]] size_t configure_dc_sync0(uint16_t station_addr,
                                            uint16_t assign_activate,
                                            uint32_t sync0_cycle_time_ns,
                                            uint32_t shift_time_ns) noexcept;

    // Task 2.3 — full homing sequence orchestration. Performs:
    //   1. Save current mode of operation (0x6060) for restore.
    //   2. Write mode = Homing (0x06).
    //   3. `push_homing_params` for the plan.
    //   4. Walk the CiA-402 FSA to OperationEnabled via the normal
    //      servo-on handshake (delegated to motion kernel step).
    //   5. Pulse controlword bit 4 (CW_HOMING_START) for one cycle.
    //   6. Poll 0x6041 for bit 12 (Homing Attained) or bit 13 (Error).
    //   7. Restore prior mode.
    // Returns true on success, false on fault/timeout. `timeout_ms`
    // bounds the whole sequence.
    //
    // Today this helper only issues the SDO writes and controlword
    // pulses — the operator is responsible for ensuring the drive is
    // already in OperationEnabled before calling (same constraint as
    // `push_homing_params`). A full motion-kernel integration (the
    // `motion::homing::run(plan)` wrapper described in the plan) will
    // own the FSA walk and mode restore.
    [[nodiscard]] bool run_homing_sequence(uint16_t station_addr,
                                           int8_t   method,
                                           uint32_t fast_speed_cps,
                                           uint32_t slow_speed_cps,
                                           uint32_t accel_cps2,
                                           int32_t  offset_counts,
                                           uint16_t hardstop_torque_permille,
                                           uint32_t timeout_ms) noexcept;

    // Task 2.4 — apply the software position limits 0x607D:1/2 *after*
    // homing completes. `neg_limit` and `pos_limit` are in axis counts.
    // Intended use: `run_homing_sequence` completes (Homing Attained),
    // operator then calls this to gate the travel envelope. Calling
    // before homing is permitted but the drive will typically apply the
    // limits relative to an un-anchored reference, which can trap the
    // axis at power-on — don't.
    [[nodiscard]] size_t push_software_limits(uint16_t station_addr,
                                              int32_t neg_limit,
                                              int32_t pos_limit) noexcept;

    // Task 2.2 — push the plan's SDO-backed parameters to the drive. Writes
    // 0x6098 method / 0x6099:1 fast / 0x6099:2 slow / 0x609A accel /
    // 0x607C offset / 0x216B hardstop torque max. Fire-and-forget (same
    // contract as `send_sdo_download`); returns the count of frames that
    // were successfully queued.
    [[nodiscard]] size_t push_homing_params(uint16_t station_addr,
                                            int8_t   method,
                                            uint32_t fast_speed_cps,
                                            uint32_t slow_speed_cps,
                                            uint32_t accel_cps2,
                                            int32_t  offset_counts,
                                            uint16_t hardstop_torque_permille) noexcept;

    size_t slave_count() const noexcept { return slave_count_; }

    // Per-slave accessors so upper layers (motion kernel) can pin a
    // `cia402::Drive*` into their per-axis state at wiring time. Index
    // range is [0, MAX_SLAVES); callers should compare against
    // slave_count() when iterating live slaves, but may hook ahead of
    // discovery (the slot exists from construction).
    SlaveInfo&       slave(size_t i) noexcept       { return slaves_[i]; }
    const SlaveInfo& slave(size_t i) const noexcept { return slaves_[i]; }

    // PLAN 1.1 + 1.3 gate: the ESM holds the bus at PreOp until this flag
    // is set. External code (the bus_config helper) probes each slave's
    // identity + pushes the per-mode PDO map, and only then allows the
    // transition to SafeOp. Default is `false` so wiring errors are
    // caught before the drive touches torque.
    void allow_safeop(bool ok) noexcept { allow_safeop_.store(ok, std::memory_order_release); }
    bool allow_safeop_set() const noexcept { return allow_safeop_.load(std::memory_order_acquire); }

    // Deadline-miss safety latch. After CONSECUTIVE_MISS_THRESHOLD cycles
    // overrun back-to-back, the master commands every CiA-402 servo on the
    // bus to QuickStopActive (controlword = CW_CMD_QUICK_STOP). The fault is
    // latched — motion observes it via is_deadline_faulted() and walks the
    // axes through their fault-reaction stop. Operator must explicitly
    // acknowledge via clear_deadline_fault() (e.g. CLI `ec_clear_fault`).
    static constexpr uint32_t CONSECUTIVE_MISS_THRESHOLD = 2;
    // Number of cycles after master start during which deadline misses
    // do NOT count toward the trip threshold. ~50 ms at 250 µs per
    // cycle covers the cold-start surge (virtio-gpu init, hmi DHCP,
    // framebuffer clear, etc) without leaving operators with a
    // pre-faulted machine on every boot.
    static constexpr uint32_t BOOT_GRACE_CYCLES = 200;
    bool is_deadline_faulted() const noexcept {
        return deadline_fault_.load(std::memory_order_acquire);
    }
    void clear_deadline_fault() noexcept {
        deadline_fault_.store(false, std::memory_order_release);
        deadline_fault_logged_ = false;
        consecutive_misses_ = 0;
    }
    // Operator-initiated trip — broadcast QuickStop to every CiA-402 servo
    // and latch the same fault as a deadline miss. Reason string is logged
    // via early_uart_puts. Idempotent: a second call while latched re-logs
    // but does not double-broadcast.
    void trip_fault(const char* reason) noexcept;

    // DC-sync drift fault latch. Mirrors the deadline-fault contract: the
    // cyclic loop polls one DC-enabled slave's ESC reg 0x0920 (SYNC0
    // actual time) every dc_drift_check_period_us_; if the drift between
    // consecutive samples for the SAME slave exceeds dc_drift_threshold_ns_
    // for CONSECUTIVE_DRIFT_THRESHOLD samples in a row, trip_fault fires.
    // 100 ms cadence amortises the blocking ESC read (one per cycle, not
    // per slave) while still detecting drift within ~300 ms. 5 µs threshold
    // is tight enough to catch real DC slope error but wide enough to
    // absorb single-cycle scheduler jitter on the master side.
    static constexpr uint32_t CONSECUTIVE_DRIFT_THRESHOLD = 3;
    bool is_dc_sync_faulted() const noexcept {
        return dc_sync_fault_.load(std::memory_order_acquire);
    }
    void clear_dc_sync_fault() noexcept {
        dc_sync_fault_.store(false, std::memory_order_release);
        consecutive_drift_misses_ = 0;
    }
    int64_t last_dc_drift_ns() const noexcept {
        return last_dc_drift_ns_.load(std::memory_order_acquire);
    }

    // Cycle-latency telemetry. `cycle` measures whole-iteration duration;
    // the sub-histograms split it into the send-probe / ESM / LRW / wait
    // phases so you can see where budget goes.
    const diag::LatencyHistogram& hist_cycle()  const noexcept { return hist_cycle_; }
    const diag::LatencyHistogram& hist_probe()  const noexcept { return hist_probe_; }
    const diag::LatencyHistogram& hist_esm()    const noexcept { return hist_esm_; }
    const diag::LatencyHistogram& hist_lrw()    const noexcept { return hist_lrw_; }
    const diag::LatencyHistogram& hist_wait()   const noexcept { return hist_wait_; }

    // Snapshot epoch — bumped once at the top of every run_loop iteration
    // (master.cpp:1178) before any per-cycle write. Operator-side readers
    // (operator_api::ethercat_snapshot) wrap their multi-field copy in a
    // bounded retry: read epoch, copy fields, re-read epoch. If the epoch
    // moved, the master ticked a cycle mid-copy and the snapshot is torn —
    // retry. With a 250 µs cycle and ~20 atomic loads per snapshot the
    // typical case is a single-shot success; the retry budget is the
    // safety net for the unlucky overlap.
    uint32_t snapshot_epoch() const noexcept {
        return snapshot_epoch_.load(std::memory_order_acquire);
    }

private:
    int id_;
    int nic_idx_;
    uint32_t period_us_;
    std::atomic<State> state_{State::Init};
    std::atomic<uint32_t> snapshot_epoch_{0};
    Stats stats_;
    kernel::hal::net::NetworkDriverOps* net_ = nullptr;

    std::array<SlaveInfo, MAX_SLAVES> slaves_{};
    size_t slave_count_ = 0;

    // Process-data image exchanged via LRW when we're in OP. Cache-line
    // aligned so the per-cycle pack/unpack in cycle_lrw doesn't read
    // across a cache line on the first slave's slot, which would cost a
    // miss at the top of every cycle.
    alignas(64) std::array<uint8_t, PDO_BUF_BYTES> pdo_buf_{};

    // Pre-allocated TX scratch — one Ethernet MTU. Previously each run_loop
    // phase (ESM write, ESM poll, cycle_lrw) did `uint8_t frame[1514]` on the
    // stack. At 200 µs cycles that was 4.5 KB stack churn + three memsets per
    // iteration. Reused in place here, one per Master.
    alignas(64) std::array<uint8_t, 1514> tx_scratch_{};

    // ESM transition tracking.
    EsmPhase esm_phase_ = EsmPhase::Idle;
    uint8_t  esm_target_ = AL_INIT;
    std::atomic<bool> allow_safeop_{false};
    uint64_t esm_deadline_us_ = 0;
    uint8_t  sdo_counter_ = 1;
    uint8_t  dgram_idx_ = 0;

    // Deadline-trip latch state.
    uint32_t consecutive_misses_ = 0;
    std::atomic<bool> deadline_fault_{false};
    bool deadline_fault_logged_ = false;

    // DC-sync drift sampler state. last_sample_us_ gates how often we pay
    // the cost of a blocking ESC read; dc_round_robin_idx_ ensures only one
    // slave per sampling tick gets read, so a 64-slave bus still finishes
    // each read inside the 250 µs cycle budget. Threshold + cadence are
    // tunable here without touching the run_loop hook.
    static constexpr uint32_t DC_DRIFT_CHECK_PERIOD_US = 100000; // 100 ms
    static constexpr uint64_t DC_DRIFT_THRESHOLD_NS    = 5000;   // 5 µs
    uint32_t dc_drift_check_period_us_ = DC_DRIFT_CHECK_PERIOD_US;
    uint64_t dc_drift_threshold_ns_    = DC_DRIFT_THRESHOLD_NS;
    uint32_t consecutive_drift_misses_ = 0;
    std::atomic<bool>    dc_sync_fault_{false};
    bool                 dc_sync_fault_logged_ = false;
    uint64_t             last_dc_sample_us_    = 0;
    std::atomic<int64_t> last_dc_drift_ns_{0};
    size_t               dc_round_robin_idx_   = 0;
    // Phase-shift compensation pending application by run_loop. Positive
    // value means the master cycle should wait LONGER next iteration
    // (master's wall clock leading slave DC); negative means SHORTER.
    // Capped per-application to ±DC_PHASE_MAX_ADJUST_US so a single
    // outlier sample can't slew the bus phase out from under itself.
    // Applied once per cycle then atomically cleared.
    std::atomic<int32_t> dc_phase_adjust_us_{0};

    // --- SDO upload state (task 1.2 + segmented follow-up) ----------------
    // Linear state machine driving a single mailbox exchange at a time.
    // `Pending` waits for the initiate-upload response; the handler can
    // either flip straight to `Done` (expedited) or advance to
    // `FetchSegment` (segmented) which loops sending segment requests
    // + polling until a `c=1` segment response lands. All collected
    // bytes land in `upload_result_` (capped at `UPLOAD_RESULT_CAP`).
    //
    // `alignas(64)` isolates the upload-transfer state from the ESM
    // control flags above — the caller (CLI thread, core 0) polls
    // `upload_state_` while the producer (ec_a, core 2) writes to it
    // from handle_rx_frame. Without this, `allow_safeop_` + the
    // upload atomics share a cache line and each SDO round-trip
    // ping-pongs the line between cores. Measurable on 2+ slaves.
    enum class UploadState : uint8_t {
        Idle = 0, Pending, FetchSegment, Done, Error
    };
    static constexpr size_t UPLOAD_RESULT_CAP = 256;
    alignas(64) std::atomic<UploadState> upload_state_{UploadState::Idle};
    std::atomic<bool>        upload_busy_{false};
    uint16_t upload_station_ = 0;
    uint16_t upload_index_   = 0;
    uint8_t  upload_sub_     = 0;
    uint8_t  upload_result_[UPLOAD_RESULT_CAP]{};
    uint16_t upload_result_bytes_     = 0;
    uint16_t upload_total_expected_   = 0; // from initiate-upload response
    uint32_t upload_abort_code_       = 0;
    uint64_t upload_deadline_us_      = 0;
    bool     upload_segment_toggle_   = false; // toggled 0/1 per segment request
    bool     upload_segment_request_inflight_ = false;

    // Tier 3a — request queue. Single producer (caller), single
    // consumer (master cycle). When upload_state_ idle, the cycle pulls
    // the head request, kicks the existing single-slot machine, and
    // copies the result into the caller's `out_*` slots when done.
    static constexpr size_t SDO_QUEUE_DEPTH = 8;
    std::array<SdoRequest, SDO_QUEUE_DEPTH> sdo_queue_{};
    std::atomic<size_t>     sdo_queue_head_{0};   // next read
    std::atomic<size_t>     sdo_queue_tail_{0};   // next write
    SdoRequest              sdo_active_request_{};
    bool                    sdo_request_in_flight_ = false;
    void service_sdo_queue() noexcept;

    // --- Blocking ESC register read support --------------------------------
    enum class EscReadState : uint8_t {
        Idle = 0, Pending, Done, Error
    };
    std::atomic<EscReadState> esc_read_state_{EscReadState::Idle};
    std::atomic<bool>         esc_read_busy_{false};
    uint16_t esc_read_station_ = 0;
    uint16_t esc_read_reg_     = 0;
    uint8_t  esc_read_len_     = 0;
    uint8_t  esc_read_buf_[8]{};
    uint64_t esc_read_deadline_us_ = 0;

    [[nodiscard]] bool read_esc_register(uint16_t station_addr,
                                         uint16_t reg,
                                         uint8_t* out, uint8_t len,
                                         uint32_t timeout_us) noexcept;

    // Latency histograms (mutable because run_loop records through const APIs).
    mutable diag::LatencyHistogram hist_cycle_;
    mutable diag::LatencyHistogram hist_probe_;
    mutable diag::LatencyHistogram hist_esm_;
    mutable diag::LatencyHistogram hist_lrw_;
    mutable diag::LatencyHistogram hist_wait_;

    void run_loop();
    void send_probe();
    void advance_state();
    void step_esm();
    void discover_slaves();
    void cycle_lrw();
    void on_deadline_fault() noexcept;
    void service_dc_drift() noexcept;
    void service_sdo_upload() noexcept;  // task 1.2 cycle-loop hook
    void service_esc_read() noexcept;
    bool send_frame(const uint8_t* buf, size_t len) noexcept;
    uint64_t now_us() const noexcept;

    // Per-frame RX handler invoked by the NIC poll-RX path.
    void handle_rx_frame(const uint8_t* data, size_t len) noexcept;

    // C-style adapter so NetworkDriverOps::PacketReceivedCallback (`int, const
    // uint8_t*, size_t, void*`) can dispatch into a Master instance.
    static void rx_trampoline(int if_idx, const uint8_t* data, size_t len,
                              void* ctx) noexcept;
};

// Two masters, one per NIC. Declared here, defined in master.cpp.
extern Master g_master_a;
extern Master g_master_b;

} // namespace ethercat

#endif
