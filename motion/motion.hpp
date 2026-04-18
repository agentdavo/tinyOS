// SPDX-License-Identifier: MIT OR Apache-2.0
//
// 32-axis motion kernel for miniOS. This is the "shape" layer: it defines the
// public types that the rest of the codebase (CLI, EtherCAT master, trajectory
// planners) will depend on. The three design lessons baked into this API come
// from the Teknic ClearPath-EC brief (devices/clearpath_ec.md):
//
//   1. Composable homing FSA (approach dir x edge x trigger x backoff) rather
//      than hard-coded CiA-402 method numbers. Torque-threshold is a
//      first-class trigger so "home-on-hard-stop" composes naturally.
//
//   2. Per-mode Controlword masks + a CiA-402 DriveStateMachine that walks
//      Switch-On-Disabled -> Ready-to-Switch-On -> Switched-On ->
//      Operation-Enabled instead of letting callers poke bits directly.
//
//   3. Brake latency and stop matrix as first-class state: five configurable
//      stop actions, an explicit engaging -> servo_disable handshake gated by
//      Delay Disable Time, and ConnectionLost routed through the matrix (CSP
//      abrupt-stops regardless).
//
// Freestanding C++20, no heap, no exceptions, no RTTI.

#ifndef MOTION_MOTION_HPP
#define MOTION_MOTION_HPP

#include "hal.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cia402 { struct Drive; }

namespace motion {

constexpr size_t MAX_AXES = 32;

// -----------------------------------------------------------------------------
// Mode / State enumerations
// -----------------------------------------------------------------------------

enum class Mode : uint8_t {
    None = 0,
    CSP,     // Cyclic Synchronous Position
    CSV,     // Cyclic Synchronous Velocity
    CST,     // Cyclic Synchronous Torque
    PP,      // Profile Position
    PV,      // Profile Velocity
    TQ,      // Profile Torque
    Homing,
};

// CiA-402 drive state as decoded from status_word (0x6041).
enum class DriveState : uint8_t {
    NotReady = 0,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

// Stop-behaviour choices (maps to 0x605A/0x605C/0x605D/0x605E/0x6007 values).
enum class StopKind : uint8_t {
    PositionHold = 0,   // hold closed-loop at current position
    DynamicBrake,       // short motor windings
    Coast,              // disable amplifier, freewheel
    Decel,              // decelerate along a profile
};

// Five independent stop actions the motion kernel models explicitly.
enum class StopAction : uint8_t {
    QuickStop = 0,      // 0x605A
    Disable,            // 0x605C (controlled-shutdown action)
    Halt,               // 0x605D (controlword bit 8)
    FaultReaction,      // 0x605E
    Abort,              // 0x6007 (abort connection option - ignored in CSP)
};

// Source of the homing trigger edge.
enum class TriggerSource : uint8_t {
    LimitSwitch = 0,
    IndexPulse,
    TorqueThreshold,    // home-on-hard-stop (ClearPath methods -1/-2)
    EncoderPosition,
    TouchProbe,
};

// Brake engagement phase. The servo may not be disabled until the physical
// brake has had `engage_delay_us` to latch, hence the explicit intermediate
// `Engaging` state.
enum class BrakePhase : uint8_t {
    Released = 0,       // motor live, brake off
    Releasing,          // waiting release_delay_us after command
    Engaging,           // stop requested, waiting engage_delay_us before servo-off
    Engaged,            // brake latched, servo disabled
};

// -----------------------------------------------------------------------------
// Configuration / plan structures
// -----------------------------------------------------------------------------

struct HomingPlan {
    bool          approach_positive   = true;   // direction of fast-search move
    bool          rising_edge         = true;   // which edge of trigger counts
    TriggerSource trigger             = TriggerSource::LimitSwitch;
    int32_t       backoff_counts      = 0;      // move this far opposite after trigger
    int32_t       creep_speed_cps     = 1000;   // counts/s during confirm phase
    int32_t       fast_speed_cps      = 10000;  // counts/s during initial search
    // TorqueThreshold-specific: ClearPath 0x216B 'Hardstop Torque Maximum', in
    // permille of peak torque (500 = 50.0%).
    int16_t       torque_threshold_permille = 500;
    // Engineering timeout; 0 = none.
    uint32_t      timeout_ms          = 0;
};

struct BrakeConfig {
    uint32_t engage_delay_us  = 10000;  // 0x2170 Delay Disable Time default
    uint32_t release_delay_us = 5000;
};

// Stop matrix: one StopKind per StopAction, configurable per axis. Defaults
// below mirror ClearPath/typical CiA-402 safe defaults.
struct StopMatrix {
    StopKind quick_stop      = StopKind::Decel;
    StopKind disable         = StopKind::PositionHold;
    StopKind halt            = StopKind::Decel;
    StopKind fault_reaction  = StopKind::DynamicBrake;
    StopKind abort_connection = StopKind::DynamicBrake;

    constexpr StopKind for_action(StopAction a) const noexcept {
        switch (a) {
            case StopAction::QuickStop:     return quick_stop;
            case StopAction::Disable:       return disable;
            case StopAction::Halt:          return halt;
            case StopAction::FaultReaction: return fault_reaction;
            case StopAction::Abort:         return abort_connection;
        }
        return StopKind::DynamicBrake;
    }
};

// -----------------------------------------------------------------------------
// CiA-402 Controlword helpers
// -----------------------------------------------------------------------------

// Per-mode controlword masks. Some bits are only legal in certain modes:
//   - Halt (bit 8) is valid in PP/PV/TQ/HM only.
//   - External limits (bits 11/12) are ignored in CST/TQ.
// ControlwordBuilder applies the right mask for the active mode so callers
// never poke bits that the drive will ignore or reject.
struct ControlwordBuilder {
    // CiA-402 standard bits (0-3): SO / EV / QS / EO.
    static constexpr uint16_t SWITCH_ON        = 1u << 0;
    static constexpr uint16_t ENABLE_VOLTAGE   = 1u << 1;
    static constexpr uint16_t QUICK_STOP_N     = 1u << 2;  // 0 = quick stop
    static constexpr uint16_t ENABLE_OPERATION = 1u << 3;
    static constexpr uint16_t FAULT_RESET      = 1u << 7;
    static constexpr uint16_t HALT             = 1u << 8;
    static constexpr uint16_t HOMING_START     = 1u << 4;  // in Homing mode
    static constexpr uint16_t PP_NEW_SETPOINT  = 1u << 4;  // in PP mode
    static constexpr uint16_t EXT_LIMIT_POS    = 1u << 11;
    static constexpr uint16_t EXT_LIMIT_NEG    = 1u << 12;

    // Returns the mask of bits that are legal to set in `mode`. Any bits
    // outside the mask get cleared when the caller asks us to emit a CW.
    static constexpr uint16_t legal_mask(Mode mode) noexcept {
        uint16_t m = 0x008F;  // bits 0..3 and 7 always legal
        switch (mode) {
            case Mode::PP:     return m | HALT | PP_NEW_SETPOINT | EXT_LIMIT_POS | EXT_LIMIT_NEG;
            case Mode::PV:     return m | HALT | EXT_LIMIT_POS | EXT_LIMIT_NEG;
            case Mode::TQ:     return m | HALT;  // CST/TQ ignore ext limits
            case Mode::Homing: return m | HALT | HOMING_START | EXT_LIMIT_POS | EXT_LIMIT_NEG;
            case Mode::CSP:
            case Mode::CSV:    return m | EXT_LIMIT_POS | EXT_LIMIT_NEG;
            case Mode::CST:    return m;  // no halt, no ext limits
            case Mode::None:
            default:           return m;
        }
    }

    // Apply legality rules; clear any bit that is not legal in this mode.
    static constexpr uint16_t sanitize(uint16_t cw, Mode mode) noexcept {
        return cw & legal_mask(mode);
    }
};

// Walks the CiA-402 FSA one step at a time. next_control_word() is meant to be
// called every cycle: it looks at the drive's latest status word plus the
// desired target mode, and emits the controlword that advances the drive one
// state closer to Operation-Enabled (or keeps it there).
class DriveStateMachine {
public:
    DriveState decode(uint16_t status) const noexcept;

    // Returns the next controlword to send to advance toward Operation-Enabled
    // in `mode`. Bits outside the per-mode legal mask are cleared.
    uint16_t next_control_word(uint16_t status, Mode mode) const noexcept;

    // Compose a stop controlword for a given StopAction while in `mode`.
    uint16_t stop_control_word(StopAction action, Mode mode) const noexcept;
};

// -----------------------------------------------------------------------------
// Homing engine
// -----------------------------------------------------------------------------

class HomingEngine {
public:
    enum class Phase : uint8_t {
        Idle = 0,
        Fast,        // seeking trigger at fast_speed_cps
        Creep,       // confirming trigger at creep_speed_cps after back-off
        Triggered,   // trigger latched, about to back off
        BackingOff,  // applying backoff_counts in the opposite direction
        Done,
        Fault,
    };

    void  start(const HomingPlan& plan) noexcept;
    void  abort() noexcept;
    Phase step(uint16_t status, int32_t pos, int16_t torque_permille) noexcept;
    Phase phase() const noexcept { return phase_; }
    bool  done() const noexcept { return phase_ == Phase::Done; }
    const HomingPlan& plan() const noexcept { return plan_; }

    // Name helper for diagnostics.
    static const char* phase_name(Phase p) noexcept;

private:
    HomingPlan plan_{};
    Phase      phase_       = Phase::Idle;
    int32_t    trigger_pos_ = 0;
    bool       prev_level_  = false;
};

// -----------------------------------------------------------------------------
// Per-axis state
// -----------------------------------------------------------------------------

// Trajectory generator state for a "move to position" command.
enum class TrajState : uint8_t {
    Idle = 0,     // No commanded motion in progress.
    MoveReady,    // Target latched; next cycle begins accel.
    Accel,        // S-curve: accelerating (jerk limited)
    ConstantVel,  // S-curve: at max velocity
    Decel,        // S-curve: decelerating (jerk limited)
    Holding,      // At target; commanded_position == target.
};

// Default motion profile limits.
// TODO: follow-up reads these from the device DB (object 0x6083/0x6081/...).
constexpr int32_t DEFAULT_ACCEL_CPS2 = 1'000'000;  // 1e6 counts/s^2
constexpr int32_t DEFAULT_JERK_CPS3  = 10'000'000; // 1e7 counts/s^3 (S-curve jerk)
constexpr int32_t DEFAULT_VMAX_CPS   =   100'000;  //   1e5 counts/s

struct Axis {
    // ---- Hot line (cache-line 0) -------------------------------------------
    // Fields touched every 250 µs motion tick. Grouped at the head of the
    // struct under `alignas(64)` so a single cache-line fetch covers all of
    // them — iterating 32 axes per cycle keeps misses bounded to 32 lines
    // instead of 4×32=128 under the previous scatter-layout. Kept in
    // read-order of `cycle_axis` to minimise false dependencies.
    //
    //   offset 0    drive*          (8 B)  — first read: null-check
    //   offset 8    commanded_position    (4 B)
    //   offset 12   actual_position_feedback (4 B)
    //   offset 16   target          (4 B) — latched setpoint
    //   offset 20   target_velocity (4 B)
    //   offset 24   status_word     (2 B, atomic) — decoded via fsa_
    //   offset 26   control_word    (2 B, atomic) — emitted post-decode
    //   offset 28   actual_torque_permille (2 B, atomic)
    //   offset 30   traj_state      (1 B) — checked on every tick
    //   remaining bytes of the line = scratch for cold fields below.
    alignas(64) cia402::Drive* drive    = nullptr;
    uint16_t  station_addr              = 0;
    int32_t   commanded_position        = 0;
    int32_t   actual_position_feedback  = 0;
    int32_t   target                    = 0;
    int32_t   target_velocity           = 0;         // current cps
    std::atomic<uint16_t> status_word{0};            // 0x6041
    std::atomic<uint16_t> control_word{0};           // 0x6040
    std::atomic<int16_t>  actual_torque_permille{0}; // 0x6077 scaled
    TrajState traj_state                = TrajState::Idle;

    // S-curve state tracking (jerk-limited profiling)
    int32_t   current_accel_cps2       = 0;         // current acceleration
    int32_t   traj_start_pos           = 0;         // position when move started
    int32_t   decel_start_pos          = 0;         // position where decel begins
    int32_t   prev_target_velocity     = 0;        // for smooth velocity transitions

    // ---- Warm line (cache-line 1) ------------------------------------------
    // Cross-core mirrors + frequently-read scalars that aren't strictly on
    // every cycle but are close. `actual_pos` / `target_pos` are diagnostic
    // mirrors of commanded_position / actual_position_feedback published
    // for the CLI `mpos` dump.
    std::atomic<int32_t>  actual_pos{0};             // 0x6064 (CLI mirror)
    std::atomic<int32_t>  target_pos{0};             // 0x607A / 0x60C1 (CLI mirror)
    int32_t   accel_cps2                = DEFAULT_ACCEL_CPS2;
    int32_t   jerk_cps3                 = DEFAULT_JERK_CPS3;
    int32_t   vmax_cps                  = DEFAULT_VMAX_CPS;
    int32_t   max_following_error       = 0;         // |commanded - actual| peak

    // Kernel-owned state (cold — read at state-transition events, not per tick).
    Mode        mode               = Mode::None;
    DriveState  state              = DriveState::NotReady;
    BrakeConfig brake              = {};
    BrakePhase  brake_phase        = BrakePhase::Engaged;
    uint64_t    brake_t_start_us   = 0;
    StopMatrix  stops              = {};
    bool        connection_lost    = false;
    bool        enabled            = false;

    HomingEngine homing            = {};

    // Encoder configuration - supports multiple setups:
    //   1. Motor encoder only (drive's built-in)
    //   2. External encoder only (BiSS at chuck/axis)
    //   3. Dual-loop (motor + external, with outer compensation)
    enum class EncoderSource : uint8_t {
        Motor = 0,     // Use motor encoder (default, simplest)
        External = 1,   // Use external encoder (direct drive)
        Dual = 2       // Dual-loop: external for position, motor for velocity
    };
    struct EncoderConfig {
        EncoderSource source = EncoderSource::Motor;
        uint32_t motor_cpr = 0;        // Motor encoder counts/rev (from drive 0x608F)
        uint32_t external_cpr = 0;     // External encoder CPR (EL5042, etc)
        uint32_t gear_num = 1;         // Gear ratio numerator
        uint32_t gear_den = 1;         // Gear ratio denominator (1:1 default)
        int32_t  sign = 1;            // ±1, flips direction if needed
    } encoder;

    // Legacy: counts_per_rev kept for compatibility, maps to motor_cpr
    uint32_t counts_per_rev        = 0;
    float    counts_per_unit       = 0.f;

    // Task 2.4 — per-axis pending software-position-limit push. The
    // operator arms these via `Kernel::arm_post_homing_limits(axis, neg,
    // pos)`; once `homing` transitions to Done AND statusword bit 8
    // (Has Homed) observes high, `cycle_axis` pushes them to the drive's
    // 0x607D:1 / :2 via SDO-download and clears `sw_limits_pending`.
    // Kept in-Axis rather than in the homing engine because the trigger
    // is a drive-side event (bit 8) and the push is a drive-side write.
    bool      sw_limits_pending         = false;
    int32_t   sw_limit_neg_counts       = 0;
    int32_t   sw_limit_pos_counts       = 0;

    // Task 9.6 — when a cross-channel sync_move scaled `vmax_cps` down so
    // this axis pins its pace to the slowest participant, `restore_vmax_cps`
    // carries the pre-scale value. cycle_axis restores it once the move
    // lands in Holding. Zero means "not currently scaled".
    int32_t   restore_vmax_cps          = 0;

    // Task 9.9 — simulated leader spin rate (counts per second). Non-zero
    // values cause `cycle_axis` to advance `actual_position_feedback` by
    // the per-cycle increment. Ignored when a drive is hooked.
    int32_t   spin_velocity_cps         = 0;

    // Fault propagation from the backing CiA-402 drive. Set when the drive
    // reports Fault / FaultReactionActive; cleared by Kernel::fault_reset
    // (which pulses CW_FAULT_RESET on the controlword PDO).
    bool      fault_latched             = false;

    // Task 1.5 — snapshot of 0x603F at the moment fault_latched flipped true.
    // Decoded via cia402::error_text() in dump_positions. Retained across
    // fault_reset so the operator can still see what happened after clearing.
    uint16_t  last_error_code           = 0;

    // (Drive pointer was hoisted to the hot cache-line at the top of this
    // struct — see the AxisHot block. Null == axis has no backing slave;
    // traj is still computed but not written anywhere, graceful no-op for
    // axes 2..31 on a one-slave bus.)

    // Task 7.4/7.5 — optional load-side feedback (EL5042 BiSS-C or similar
    // absolute encoder on the axis itself, not the motor shaft). When
    // present, a bounded-slew PI loop trims the CSP target to close on
    // load-side truth — catches backlash, ballscrew wind-up, thermal
    // growth, coupling slip.
    struct LoadFeedback {
        bool     configured        = false;   // has the pipeline been set up?
        bool     valid             = false;   // last sample trusted this cycle
        int64_t  raw_position      = 0;       // 64-bit from TxPDO (bits, not units)
        int64_t  scale_num         = 1;       // load_counts_per_unit numerator
        int64_t  scale_den         = 1;       // load_counts_per_unit denominator
        int32_t  sign              = 1;       // ±1, flips if encoder mounted reversed
        int64_t  offset_counts     = 0;       // captured at calibration
        // Outer-PI gains (integer, scaled ×1000 so 1 = 0.001 gain). Keep
        // modest — the inner ClearPath loop is the fast controller.
        int16_t  kp_ppm            = 500;     // 0.5 trim-count per load-count of error
        int16_t  ki_ppm            = 10;      // integral term
        int32_t  trim_cap_counts   = 200;     // hard limit on trim magnitude
        int32_t  trim_slew_cps     = 5000;    // max rate of change of trim
        // Integrator + last trim for slew limit.
        int64_t  integral          = 0;
        int32_t  last_trim         = 0;
        // Watchdog (task 7.6) — `samples_received` is incremented by the
        // publisher (EL5042 PDO unpack or CLI axis_load_sample). Once per
        // motion cycle `cycle_axis` snapshots it into `samples_last_seen`
        // and if the value didn't change, bumps `stale_cycles`. A
        // `max_stale_cycles`-cycle streak invalidates the channel and
        // drives the trim back to zero. Resets on the first fresh sample.
        uint16_t samples_received    = 0;
        uint16_t samples_last_seen   = 0;
        uint8_t  stale_cycles        = 0;
        uint8_t  max_stale_cycles    = 2;

        // Task 7.8 — publisher can set `error_latched` directly (e.g. from
        // the EL5042 status bitfield) to force the outer loop into the
        // stale-decay path and propagate fault up to Axis::fault_latched.
        bool     error_latched       = false;
    } load;

    // Scaled load position in kernel axis counts (after sign/scale/offset).
    // Populated from `load.raw_position` each cycle when `load.configured`.
    int32_t   load_position_counts      = 0;
    int32_t   load_following_error      = 0;   // load_pos - commanded
    int32_t   outer_trim                = 0;   // last PI output, clipped

    // Power skiving support: B-axis (tool spindle) for index tooth positions.
    // When configured, the axis operates in dual-mode: CSP for normal motion,
    // plus indexed tooth positions derived from main spindle position.
    struct SpindleIndexer {
        bool     configured              = false;
        bool     active                 = false;
        uint16_t teeth_per_revolution   = 0;    // e.g. 20 for 20-TPI cutter
        uint32_t counts_per_rev          = 0;    // encoder resolution for this axis
        int32_t  index_offset_counts     = 0;    // tooth 0 position offset
        int32_t  current_tooth          = 0;    // which tooth is currently at cut
        int32_t  last_spindle_pos       = 0;    // for detecting wraparound

        // Configure the indexer with encoder resolution.
        void configure(uint16_t teeth, uint32_t cpr, int32_t offset) noexcept {
            teeth_per_revolution = teeth;
            counts_per_rev = cpr;
            index_offset_counts = offset;
            configured = true;
            active = true;
        }

        // Compute indexed position: given spindle position, compute where
        // the tool should be to cut at the current tooth.
        int32_t compute_indexed_position(int32_t spindle_pos,
                                          int32_t follower_base) const noexcept {
            if (!configured || teeth_per_revolution == 0 || counts_per_rev == 0) {
                return follower_base;
            }
            const int32_t counts_per_tooth = static_cast<int32_t>(counts_per_rev / teeth_per_revolution);
            if (counts_per_tooth == 0) return follower_base;
            const int32_t tooth = (spindle_pos / counts_per_tooth) % teeth_per_revolution;
            const int32_t tooth_pos = tooth * counts_per_tooth + index_offset_counts;
            return follower_base + tooth_pos;
        }
    } spindle_indexer;

    // Hall sensor state decoded from digital_inputs (0x60FD bits 0-2).
    // Standard 3-phase hall pattern: 001(1), 011(3), 010(2), 110(6), 100(4), 101(5)
    struct HallDecoder {
        static constexpr uint32_t HALL_MASK = 0x07;
        uint8_t raw_state = 0;
        uint8_t electrical_angle = 0;
        bool valid = false;

        uint8_t decode(uint32_t digital_inputs) noexcept {
            raw_state = static_cast<uint8_t>(digital_inputs & HALL_MASK);
            if (raw_state == 0 || raw_state == 7) {
                valid = false;
                return 0;
            }
            valid = true;
            electrical_angle = (raw_state - 1) * 60;
            return electrical_angle;
        }
    } hall;

    // Advance the brake state machine by the current time; returns true iff
    // the axis is currently commanded live (brake released, servo on).
    bool tick_brake(uint64_t now_us) noexcept;

    // Request a stop action. Picks the StopKind from the matrix and primes
    // the brake FSA. Does not touch control_word directly.
    void request_stop(StopAction action, uint64_t now_us) noexcept;

    // Signal ConnectionLost. In CSP this abrupt-stops regardless of the
    // abort_connection setting; other modes follow the stop matrix.
    void on_connection_lost(uint64_t now_us) noexcept;

    // Wire the axis to its backing CiA-402 drive. Called once from kernel_main
    // for each configured slave. Nullptr is valid (axis sits idle). The
    // vmax/accel overrides come from the device DB (OD 0x6081/0x6083) when
    // available; pass 0 to keep the compiled-in defaults.
    void hook_drive(cia402::Drive* d, uint16_t station,
                    int32_t vmax_override  = 0,
                    int32_t accel_override = 0) noexcept {
        drive = d;
        station_addr = station;
        if (vmax_override  > 0) vmax_cps   = vmax_override;
        if (accel_override > 0) accel_cps2 = accel_override;
    }

    // Advance the trapezoidal generator by `dt_us` microseconds and update
    // `commanded_position` + `traj_state`. Emits one new setpoint per call.
    void step_trajectory(uint32_t dt_us) noexcept;

    // CSV (Cyclic Synchronous Velocity) trajectory step for spindle control.
    void step_csv_trajectory(uint32_t dt_us) noexcept;

    // Compute spindle gear position from leader spindle position.
    int32_t compute_spindle_gear_position(int32_t leader_actual_pos,
                                           int32_t leader_counts_per_rev,
                                           int32_t follower_base,
                                           int32_t k_num,
                                           int32_t k_den) const noexcept;
};

// -----------------------------------------------------------------------------
// Motion kernel
// -----------------------------------------------------------------------------

struct MotionStats {
    std::atomic<uint64_t> cycles{0};
    std::atomic<uint64_t> deadline_miss{0};
    std::atomic<uint64_t> faults{0};
    std::atomic<uint64_t> homings_started{0};
    std::atomic<uint64_t> homings_done{0};
    std::atomic<uint64_t> connection_losses{0};
};

// -----------------------------------------------------------------------------
// Channel — an independent motion "machine" (Phase 9.1).
// -----------------------------------------------------------------------------
//
// A channel owns a list of axis indices into the global Kernel::axes_[] pool,
// plus its own coarse FSA state, per-channel overrides, and eventually a
// look-ahead queue + interpreter iterator. Axis ownership is exclusive: an
// axis belongs to exactly one channel (see motion/CHANNELS.md "Kernel-level
// correctness invariants").
//
// For Phase 9.1 there is exactly one channel and it owns every axis, so the
// runtime is behaviourally equivalent to the pre-channel flat kernel. 9.3
// adds a second channel; 9.4/9.5 add the sync-primitive state that lives
// next to channels_[] on the Kernel.

constexpr size_t MAX_CHANNELS = 2;          // mill + lathe on a mill-turn
constexpr size_t MAX_BARRIERS = 4;          // how many barriers can be in flight
constexpr size_t MAX_GEARS    = 4;          // how many gear-links can run concurrently
constexpr size_t MAX_GANTRYS  = 2;          // how many gantry pairs can run concurrently
constexpr size_t MAX_CAL_POINTS = 32;       // max calibration points per axis
constexpr size_t MAX_ROTARY_CAL = 4;        // max rotary axes with calibration
constexpr size_t MAX_SPHERE_PTS = 64;       // max sphere measurement points

// Volumetric error compensation — 21-error model for 3-axis machine
// Based on ISO 230-1 / ASME B5.54 machine tool calibration standard
struct VolumetricErrors {
    // Position errors (Xa, Ya, Za) — positioning accuracy
    int32_t pos_x_urad = 0;  // X axis position error (µrad)
    int32_t pos_y_urad = 0;
    int32_t pos_z_urad = 0;

    // Straightness errors (Xy, Xz, Yx, Yz, Zx, Zy) — linear deviation
    int32_t str_x_y_urad = 0;  // X axis straightness in Y direction
    int32_t str_x_z_urad = 0;  // X axis straightness in Z direction
    int32_t str_y_x_urad = 0;
    int32_t str_y_z_urad = 0;
    int32_t str_z_x_urad = 0;
    int32_t str_z_y_urad = 0;

    // Angular errors (pitch, yaw, roll) of each axis
    int32_t pitch_x_urad = 0;  // rotation around Y axis
    int32_t yaw_x_urad = 0;    // rotation around Z axis
    int32_t roll_x_urad = 0;   // rotation around X axis
    int32_t pitch_y_urad = 0;
    int32_t yaw_y_urad = 0;
    int32_t roll_y_urad = 0;
    int32_t pitch_z_urad = 0;
    int32_t yaw_z_urad = 0;
    int32_t roll_z_urad = 0;

    // Squareness errors (perpendicularity)
    int32_t sq_xy_urad = 0;  // X vs Y (Y is tilted relative to X)
    int32_t sq_xz_urad = 0;
    int32_t sq_yz_urad = 0;
};

struct SphereMeasPoint {
    int32_t cmd_x = 0, cmd_y = 0, cmd_z = 0;  // commanded position
    int32_t act_x = 0, act_y = 0, act_z = 0;  // measured position (from probe)
    bool    valid = false;
};

// Sphere-based calibration — measure at multiple positions, compute errors
struct SphereCalibration {
    bool                     enabled = false;
    float                   sphere_diam_mm = 20.0f;   // Renishaw sphere diameter
    int32_t                 probe_radius_um = 0;     // probe radius in micrometers
    int32_t                 probe_hits = 3;          // number of hits per point to average
    int32_t                 rapid_speed_mm_min = 1000;  // rapid move speed
    int32_t                 probe_speed_mm_min = 200;    // probing speed
    std::array<SphereMeasPoint, MAX_SPHERE_PTS> points{};
    size_t                  point_count = 0;
    VolumetricErrors        errors{};                 // computed volumetric errors
    bool                    errors_computed = false;
};

// Pitch error compensation (PEC) — linear axis ball screw mapping.
// Stores measured position error at known points; compensation is applied
// by interpolating between adjacent points at runtime.
struct CalPoint {
    int32_t position_counts = 0;  // commanded position
    int32_t error_counts    = 0;  // measured error (actual - commanded)
};

struct LinearCalibration {
    bool     enabled = false;
    uint8_t  axis    = 0xFF;
    uint16_t point_count = 0;
    std::array<CalPoint, MAX_CAL_POINTS> points{};
};

// Rotary axis calibration — index offset and gear ratio errors.
struct RotaryCalibration {
    bool     enabled = false;
    uint8_t  axis    = 0xFF;
    int32_t  index_offset_counts = 0;  // offset from index pulse to zero position
    int32_t  gear_error_counts    = 0;  // accumulated gear error over full revolution
    uint32_t counts_per_revolution = 36000;  // default 0.01° resolution
};

// Machine geometry — squareness errors between axis pairs.
// Stores angular error (in microradians) between axis pairs: XY, XZ, YZ.
// Example: XY = +100 means Y axis is pitched +100 µrad relative to X (not square).
struct MachineGeometry {
    bool    enabled = false;
    int32_t xy_error_urad = 0;   // X vs Y perpendicularity error (µrad)
    int32_t xz_error_urad = 0;   // X vs Z perpendicularity error (µrad)
    int32_t yz_error_urad = 0;   // Y vs Z perpendicularity error (µrad)
};

// Gantry coupling — synchronizes two axes (e.g., Y1/Y2 on a gantry table).
// The secondary axis follows the primary with an adjustable offset for
// mechanical alignment correction (e.g., ±0.1mm = ±1000 counts at 0.1µm resolution).
struct GantryLink {
    bool     in_use             = false;
    uint8_t  primary_axis       = 0;   // leader axis (Y1)
    uint8_t  secondary_axis     = 0;   // follower axis (Y2)
    int32_t  ratio_num          = 1;   // gear ratio numerator (default 1:1)
    int32_t  ratio_den          = 1;   // gear ratio denominator
    int32_t  offset_counts      = 0;   // static offset correction (counts)
    int32_t  offset_adjust      = 0;   // online adjustment (counts, ±1µm increments)
    int32_t  primary_base       = 0;   // snapshot at engage
    int32_t  secondary_base     = 0;   // snapshot at engage
    int32_t  max_correction     = 10000; // max correction range (default ±10000 counts = ±1mm @ 0.1µm)
};

enum class ChannelState : uint8_t {
    Idle = 0,
    Running,
    FeedHold,
    WaitingBarrier,     // Phase 9.4 — placeholder for now
    Gearing,            // Phase 9.5 — placeholder for now
    Fault,
};

struct ChannelOverrides {
    // Scaled ×1000: 1000 = 100%, 500 = 50%. Integer so the math is exact
    // at 250 µs tick rate. Applied by the interpreter / look-ahead when it
    // lands; today nobody reads these, but they're part of the channel's
    // identity so they live here from the start.
    uint16_t feed_permille    = 1000;
    uint16_t rapid_permille   = 1000;
    uint16_t spindle_permille = 1000;
};

struct Channel {
    // Human-readable tag used by CLI dumps and TSV config. Kept short.
    char         name[8]         = {'c','h','0',0,0,0,0,0};

    // Indices into Kernel::axes_[]. Channel owns each listed index
    // exclusively. axis_count is the live prefix length of axis_indices.
    uint8_t      axis_indices[MAX_AXES] = {};
    uint8_t      axis_count    = 0;

    ChannelState state         = ChannelState::Idle;
    ChannelOverrides overrides = {};

    // Placeholder for the look-ahead queue + interpreter iterator added in
    // 9.3. The flat Kernel currently commands axes directly via move_to(),
    // so there is nothing to buffer yet.

    // Task 9.4 — when state==WaitingBarrier, `barrier_token` is the
    // rendezvous token this channel is stalled on. The arbiter uses it to
    // correlate with the Barrier record in Kernel::barriers_.
    uint16_t     barrier_token = 0;
};

// Task 9.4 — cross-channel rendezvous (G10.1-style).
//
// Channels emit a `barrier_token` via `Kernel::arrive_at_barrier`. The
// arbiter holds each arriving channel (state = WaitingBarrier) at its
// commanded position until every participant's `|commanded - actual| <
// tolerance_counts` for `stable_cycles_required` consecutive cycles; then
// the arbiter atomically flips all participants back to Running in the
// same cycle (motion/CHANNELS.md invariant: partial release breaks
// threading). If a participant never converges, the arbiter raises
// `ChannelState::Fault` with a SyncTimeout on expiry.
//
// Tokens are 16-bit integer IDs at runtime (per the user's durable memory
// note). A future interpreter layer maps human-readable names onto these
// IDs at parse time; the kernel itself never sees a string.
struct Barrier {
    bool     in_use                   = false;   // slot occupancy
    uint16_t token                    = 0;
    uint8_t  participants_mask        = 0;       // bit i set => ch i is a participant
    uint8_t  arrivals_mask            = 0;       // bit i set => ch i has arrived
    uint8_t  released                 = 0;       // post-release: bit i => ch i already released (informational)
    int32_t  tolerance_counts         = 1;       // per-axis |cmd-actual| bound
    uint16_t stable_cycles_required   = 3;       // ride out encoder noise + 1-cycle dead time
    uint16_t max_wait_cycles          = 20000;   // ~5 s at 250 µs
    uint64_t created_cycle            = 0;       // Kernel::stats_.cycles at creation

    // Per-participant state: consecutive cycles the channel has been
    // "within tolerance on every owned axis".
    uint16_t stable_cycles[MAX_CHANNELS] = {};
};

// Task 9.5 — electronic gearing / phase-lock.
//
// A GearLink makes `follower_axis` track `leader_axis` at ratio k=num/den:
//   follower.commanded[t] = follower_base + (k_num * (leader.actual[t] - leader_base)) / k_den
//
// Leader sample and follower command go out in the SAME cycle (motion/
// CHANNELS.md: threading gets ruined by one cycle of latency). Leader and
// follower may live in different channels — typical use is C1 spindle on
// the mill channel driving Z2 on the lathe channel for cross-threading.
//
// Engagement ramps the effective ratio linearly over `ramp_cycles_total`
// so the follower doesn't yank: k_eff = k * (ramp_done / ramp_total).
// Disengage mirrors this back down to zero. While gear is active, the
// follower's channel enters `ChannelState::Gearing` which freezes
// trajectory output (cycle_axis freeze path). The interpreter is
// forbidden from commanding the follower while geared — we treat that as
// a parse-time error in future work; no runtime fault today.
//
// Rational k (num/den int32) keeps the arithmetic integer. For a 1:1
// cross-threading link, k=1/1. For a 5 TPI → ballscrew lead, k could be
// follower_counts_per_rev / leader_counts_per_rev with both > 0.
struct GearLink {
    bool     in_use             = false;
    uint8_t  leader_axis        = 0;   // global axis index
    uint8_t  follower_axis      = 0;   // global axis index
    uint8_t  follower_channel   = 0;   // channel that gets frozen
    int32_t  k_num              = 1;
    int32_t  k_den              = 1;   // must be non-zero
    int32_t  leader_base        = 0;   // snapshot of leader.actual at engage
    int32_t  follower_base      = 0;   // snapshot of follower.commanded at engage
    uint16_t ramp_total         = 0;   // full ramp length in cycles; 0 = step
    uint16_t ramp_done          = 0;   // cycles elapsed since engage (capped at ramp_total)
    bool     disengaging        = false; // when true, ramp counts k down to 0
};

class Kernel {
public:
    // Servo period — must equal the EtherCAT master cycle so every LRW frame
    // carries a freshly computed setpoint. Lifted 200→250 µs in P0-b to meet
    // the ClearPath-EC minimum; stays coupled to ethercat::Master period and
    // devices/clearpath_ec.tsv 0x60C2.
    static constexpr uint32_t DEFAULT_PERIOD_US = 250;

    Kernel() noexcept;

    Axis&       axis(size_t i) noexcept       { return axes_[i]; }
    const Axis& axis(size_t i) const noexcept { return axes_[i]; }

    // Channel accessors. Phase 9.1 ships with channel_count() == 1 and
    // channel(0) owning every axis in axes_[]; 9.3 grows this.
    size_t         channel_count() const noexcept { return channel_count_; }
    Channel&       channel(size_t c) noexcept       { return channels_[c]; }
    const Channel& channel(size_t c) const noexcept { return channels_[c]; }

    const MotionStats& stats() const noexcept { return stats_; }
    const DriveStateMachine& fsa() const noexcept { return fsa_; }

    // Command helpers - higher-level than poking control_word.
    void request_mode(size_t axis_idx, Mode mode) noexcept;
    void start_homing(size_t axis_idx, const HomingPlan& plan) noexcept;
    void request_stop(size_t axis_idx, StopAction action) noexcept;
    void on_connection_lost(size_t axis_idx) noexcept;

    // Commands a "move to position" on axis `axis_idx`. Sets the target,
    // transitions the axis to MoveReady, and the motion kernel begins
    // executing the trajectory next cycle.
    void move_to(size_t axis_idx, int32_t position) noexcept;

    // Set axis velocity directly (for CSV spindle mode). Velocity is in
    // counts per second - axis will rotate at this rate continuously.
    void set_axis_velocity(size_t axis_idx, int32_t velocity_cps) noexcept;

    // Operator-initiated fault clear. Drops the axis's latched fault flag
    // and pulses CW_FAULT_RESET (bit 7) on the drive's controlword so the
    // CiA-402 FSA transitions Fault → SwitchOnDisabled. Caller is then
    // responsible for re-enabling the axis through a normal Shutdown →
    // SwitchOn → EnableOperation sequence (the drive FSA's own step()
    // handles that once statusword is refreshed).
    [[nodiscard]] bool fault_reset(size_t axis_idx) noexcept;

    // Store an encoder resolution read from the drive (OD 0x608F via the
    // master's probe_encoder_resolution helper). Also initialises
    // counts_per_unit to `counts_per_rev` so motion math has a consistent
    // fallback when no explicit unit ratio has been configured. PLAN 1.4.
    void set_encoder_resolution(size_t axis_idx, uint32_t counts_per_rev) noexcept;

    // Configure encoder source and parameters for flexible feedback configurations.
    // source: 0=motor (default), 1=external, 2=dual-loop
    // motor_cpr: motor encoder counts/rev (from drive 0x608F)
    // external_cpr: external encoder CPR (EL5042, EL5101, etc)
    // gear_num/gear_den: gear ratio between motor and external (e.g., 1/1 for direct drive)
    // sign: ±1 to flip direction if encoder mounted reversed
    void configure_encoder(size_t axis_idx, uint8_t source,
                          uint32_t motor_cpr, uint32_t external_cpr,
                          uint32_t gear_num, uint32_t gear_den,
                          int32_t sign) noexcept;

    // Configure spindle indexer for power skiving (B-axis tooth indexing).
    // teeth_per_rev: number of cutter teeth (e.g., 20 for 20-TPI)
    // counts_per_rev: encoder counts per revolution
    // tooth_offset: position offset for tooth 0
    void configure_spindle_indexer(size_t axis_idx, uint16_t teeth_per_rev,
                                    uint32_t counts_per_rev,
                                    int32_t tooth_offset) noexcept;

    // Engage spindle gear: leader spindle drives follower B-axis at ratio.
    // leader_axis: main spindle axis (CSV mode)
    // follower_axis: B-axis to index (CSP mode)
    // k_num/k_den: gear ratio (e.g., 1/1 for 1:1, 20/1 for 20 teeth)
    // ramp_cycles: ramp-up time in cycles
    bool engage_spindle_gear(size_t leader_axis, size_t follower_axis,
                              int32_t k_num, int32_t k_den,
                              uint16_t ramp_cycles) noexcept;

    // CLI dump for the `mpos` command — one line per axis that has a drive
    // hooked or a non-idle trajectory, showing commanded / actual / following
    // error.
    void dump_positions(kernel::hal::UARTDriverOps* uart) const;

    // Thread entry compatible with kernel::Scheduler::create_thread.
    static void thread_entry(void* arg);

    // CLI diagnostic dump (single-shot; safe to call from any thread).
    void dump_status(kernel::hal::UARTDriverOps* uart) const;

    // Names for diagnostics.
    static const char* mode_name(Mode m) noexcept;
    static const char* drive_state_name(DriveState s) noexcept;
    static const char* brake_phase_name(BrakePhase p) noexcept;
    static const char* stop_kind_name(StopKind k) noexcept;
    static const char* trigger_source_name(TriggerSource t) noexcept;

    // CLI `channels` dump — one line per channel + owned axes.
    void dump_channels(kernel::hal::UARTDriverOps* uart) const;

    // Task 7.4 — load-feedback configuration. `scale_num / scale_den` converts
    // raw encoder bits into axis counts (e.g. 1 000 000 Renishaw nm per axis
    // count would be `num=1, den=1_000_000` if axis counts are already in nm;
    // pick whatever unit convention the motion kernel uses). `sign` flips
    // mount polarity. Enables the pipeline — the outer PI will run as soon
    // as the first `push_load_sample` arrives. Returns false if the axis
    // index is out of range.
    [[nodiscard]] bool configure_load_feedback(size_t axis_idx,
                                               int64_t scale_num, int64_t scale_den,
                                               int32_t sign,
                                               int32_t trim_cap_counts = 200,
                                               int32_t trim_slew_cps   = 5000,
                                               int16_t kp_ppm          = 500,
                                               int16_t ki_ppm          = 10,
                                               uint8_t max_stale_cycles = 2) noexcept;

    // Push a load-side raw sample. Called by the EL5042 PDO unpack path
    // once per LRW cycle; also callable from CLI for bench testing. Sets
    // `load.valid=true`, bumps the watchdog counter, and clears the
    // stale-cycles streak. `error` maps to the EL5042 per-channel error
    // bit — when true the arbiter treats the sample as invalid and
    // propagates the fault (task 7.8).
    [[nodiscard]] bool push_load_sample(size_t axis_idx,
                                        int64_t raw_position,
                                        bool error = false) noexcept;

    // Task 7.7 — calibration. Snapshots the current commanded_position as
    // the axis zero and solves for the offset that makes the scaled load
    // reading land on it this cycle. Subsequent cycles will show
    // `load_following_error ≈ 0` until motion or mechanical drift
    // separates them. Returns false if load isn't configured or no sample
    // has been received yet.
    [[nodiscard]] bool calibrate_load(size_t axis_idx) noexcept;

    // Task 2.4 hook — arm the software-limit push. `cycle_axis` watches
    // for statusword bit 8 (Has Homed) and fires the SDO writes once
    // the drive confirms the axis is anchored. Overwrites any previous
    // arming. `neg`/`pos` are axis counts.
    [[nodiscard]] bool arm_post_homing_limits(size_t axis_idx,
                                              int32_t neg_limit,
                                              int32_t pos_limit) noexcept;

    // Explicit servo enable / disable. Sets Axis::enabled; the CiA-402
    // walker in cycle_axis advances toward OperationEnabled when true
    // and toward Disable when false. Separate from `move_to` which
    // always enables as a side effect.
    [[nodiscard]] bool enable_axis(size_t axis_idx)  noexcept;
    [[nodiscard]] bool disable_axis(size_t axis_idx) noexcept;

    // Task 9.4 — barrier API.
    //
    // `arrive_at_barrier` is called by `channel_idx`'s controller (CLI or,
    // eventually, the interpreter) when its program stream hits a barrier
    // statement. If no barrier with that `token` exists yet, a slot is
    // allocated and the `participants_mask` installed; the caller is
    // responsible for supplying the participant list on first arrival.
    // Subsequent arrivals must match. Returns false iff the barrier table
    // is full or the arrivals have already been released.
    [[nodiscard]] bool arrive_at_barrier(size_t channel_idx,
                                         uint16_t token,
                                         uint8_t participants_mask,
                                         int32_t tolerance_counts = 1,
                                         uint16_t stable_cycles_required = 3,
                                         uint16_t max_wait_cycles = 20000) noexcept;

    // CLI helper — dump active barriers (slot, token, participants, arrivals,
    // per-channel stable-cycle counters).
    void dump_barriers(kernel::hal::UARTDriverOps* uart) const;

    // Task 9.6 — synchronous coordinated move across one or more channels.
    // `axis_mask` picks which axes participate (bit i = axis i). `targets[i]`
    // is the destination for axis i (only read for axes with a mask bit
    // set). The scheduler picks T_final = max over participating axes of
    // `|delta| / vmax_cps`, then rescales each axis's effective vmax so
    // the slowest pins the pace and the faster axes arrive together.
    // Barriers are posted automatically on every channel that owns a
    // participating axis — call returns `true` iff the barrier slot was
    // allocated; the CLI / interpreter can pass `barrier_token = 0` to
    // auto-allocate.
    //
    // Vmax is restored on each axis when its trajectory reaches Holding.
    // If `register_barrier` is true (default), the function also registers
    // a cross-channel barrier via `arrive_at_barrier` so motion commits
    // atomically on convergence.
    bool sync_move(uint64_t axis_mask,
                   const int32_t targets[MAX_AXES],
                   uint16_t barrier_token = 0,
                   int32_t  tolerance_counts = 1,
                   uint16_t stable_cycles_required = 3,
                   uint16_t max_wait_cycles = 20000,
                   bool     register_barrier = true) noexcept;

    // Task 9.7 — per-channel feedhold. `on=true` flips the channel into
    // FeedHold state; cycle_channel will freeze its axes' trajectories
    // (same mechanism as WaitingBarrier / Gearing). `on=false` returns
    // the channel to Running. Other channels are untouched — operators
    // can halt the mill channel without stalling the lathe channel.
    bool feedhold(size_t channel_idx, bool on) noexcept;

    // Task 9.8 — topology config. `ChannelSpec` is one channel's binding.
    // `set_topology(specs, count)` validates the full set (no axis in two
    // channels, no channel with zero axes, count ≤ MAX_CHANNELS) and
    // rewrites `channels_[]`. Returns false + leaves the kernel untouched
    // if validation fails. Binding is static — call once at boot.
    struct ChannelSpec {
        char    name[8];
        uint8_t axis_indices[MAX_AXES];
        uint8_t axis_count;
    };
    [[nodiscard]] bool set_topology(const ChannelSpec* specs, size_t count) noexcept;

    // Override selector. Values are permille (1000 = 100 %). Applied by
    // the interpreter / look-ahead when it wraps a move; today the
    // kernel just stores them, they don't yet affect trajectory math.
    enum class OverrideKind : uint8_t { Feed = 0, Rapid, Spindle };
    [[nodiscard]] bool set_override(size_t channel_idx, OverrideKind which, uint16_t permille) noexcept;

    // Task 9.5 — engage an electronic gear link. `leader` and `follower`
    // are global axis indices; `follower_channel` is the channel whose
    // state flips to Gearing (its trajectory output is frozen). k is
    // rational (num/den); `ramp_cycles` cycles linear blend from 0 to k
    // (pass 0 for a step change — usually a bad idea). Returns false
    // iff the gear table is full, k_den==0, or the follower axis is
    // already being geared by another link.
    bool engage_gear(uint8_t leader_axis,
                     uint8_t follower_axis,
                     uint8_t follower_channel,
                     int32_t k_num,
                     int32_t k_den,
                     uint16_t ramp_cycles = 100) noexcept;

    // Ramp the link down to zero over `ramp_cycles`, then free the slot
    // and return the follower channel to Running. Returns false iff no
    // link matches.
    bool disengage_gear(uint8_t follower_axis,
                        uint16_t ramp_cycles = 100) noexcept;

    // CLI helper — one line per active gear link.
    void dump_gears(kernel::hal::UARTDriverOps* uart) const;

    // Gantry coupling — ties two axes together (e.g., Y1/Y2 gantry).
    // primary_axis: leader (Y1), secondary_axis: follower (Y2).
    // ratio: gear ratio (num/den), offset: static offset correction (counts).
    // Returns slot index on success, -1 on failure.
    int engage_gantry(uint8_t primary_axis,
                      uint8_t secondary_axis,
                      int32_t ratio_num = 1,
                      int32_t ratio_den = 1,
                      int32_t offset = 0) noexcept;

    // Disengage a gantry pair by secondary axis.
    bool disengage_gantry(uint8_t secondary_axis) noexcept;

    // Adjust gantry offset online (±1 count increments, capped at max_correction).
    // Use positive to move secondary forward, negative to move backward.
    bool adjust_gantry_offset(uint8_t secondary_axis, int32_t delta_counts) noexcept;

    // Set max correction range for a gantry.
    bool set_gantry_max_correction(uint8_t secondary_axis, int32_t max_correction) noexcept;

    // CLI helper — one line per active gantry link.
    void dump_gantrys(kernel::hal::UARTDriverOps* uart) const;

    // ===== Axis Calibration API =====

    // Linear PEC (pitch error compensation) — add point to calibration table.
    // Returns index on success, -1 if table full.
    int add_cal_point(uint8_t axis, int32_t position, int32_t error);

    // Enable/disable linear calibration for an axis.
    bool enable_linear_cal(uint8_t axis, bool enable);

    // Get linear compensation for a position (interpolates between points).
    int32_t get_linear_correction(uint8_t axis, int32_t position) const;

    // Dump linear calibration table.
    void dump_linear_cal(kernel::hal::UARTDriverOps* uart, uint8_t axis) const;

    // Clear linear calibration for an axis.
    void clear_linear_cal(uint8_t axis);

    // Rotary calibration — set index offset.
    bool set_rotary_index_offset(uint8_t axis, int32_t offset_counts);

    // Enable/disable rotary calibration.
    bool enable_rotary_cal(uint8_t axis, bool enable);

    // Get rotary correction (index offset).
    int32_t get_rotary_correction(uint8_t axis) const;

    // Dump rotary calibration.
    void dump_rotary_cal(kernel::hal::UARTDriverOps* uart, uint8_t axis) const;

    // Machine geometry — squareness errors (in microradians).
    bool set_geometry_error(const char* pair, int32_t urad);
    int32_t get_geometry_error(const char* pair) const;
    bool enable_geometry(bool enable);
    void dump_geometry(kernel::hal::UARTDriverOps* uart) const;

    // ===== Sphere-based Volumetric Calibration =====
    
    // Set sphere diameter (default 20.0mm for Renishaw)
    void sphere_set_diameter(float mm) { sphere_cal_.sphere_diam_mm = mm; }
    float sphere_diameter() const { return sphere_cal_.sphere_diam_mm; }
    
    // Set probe radius in micrometers
    void sphere_set_probe_radius(int32_t um) { sphere_cal_.probe_radius_um = um; }
    int32_t sphere_probe_radius() const { return sphere_cal_.probe_radius_um; }
    
    // Set number of hits to average per point (default 3)
    void sphere_set_probe_hits(int32_t hits) { sphere_cal_.probe_hits = (hits > 0 && hits <= 10) ? hits : 3; }
    int32_t sphere_probe_hits() const { return sphere_cal_.probe_hits; }
    
    // Set rapid/probe speeds in mm/min
    void sphere_set_rapid_speed(int32_t mm_min) { sphere_cal_.rapid_speed_mm_min = mm_min > 0 ? mm_min : 1000; }
    void sphere_set_probe_speed(int32_t mm_min) { sphere_cal_.probe_speed_mm_min = mm_min > 0 ? mm_min : 200; }
    int32_t sphere_rapid_speed() const { return sphere_cal_.rapid_speed_mm_min; }
    int32_t sphere_probe_speed() const { return sphere_cal_.probe_speed_mm_min; }
    
    // Add a sphere measurement point (averaging multiple hits) (use probe to touch sphere at position)
    // Returns point index or -1 if full
    int sphere_add_point(int32_t cmd_x, int32_t cmd_y, int32_t cmd_z,
                         int32_t act_x, int32_t act_y, int32_t act_z);
    
    // Clear all measurement points
    void sphere_clear_points() { sphere_cal_.point_count = 0; sphere_cal_.errors_computed = false; }
    
    // Compute volumetric errors from collected points
    // Uses least-squares fitting to compute 21 error parameters
    bool sphere_compute_errors();
    
    // Enable/disable volumetric compensation
    void sphere_enable(bool en) { sphere_cal_.enabled = en; }
    bool sphere_enabled() const { return sphere_cal_.enabled; }
    
    // Get individual error components
    int32_t sphere_error_pos_x() const { return sphere_cal_.errors.pos_x_urad; }
    int32_t sphere_error_pos_y() const { return sphere_cal_.errors.pos_y_urad; }
    int32_t sphere_error_pos_z() const { return sphere_cal_.errors.pos_z_urad; }
    int32_t sphere_error_sq_xy() const { return sphere_cal_.errors.sq_xy_urad; }
    int32_t sphere_error_sq_xz() const { return sphere_cal_.errors.sq_xz_urad; }
    int32_t sphere_error_sq_yz() const { return sphere_cal_.errors.sq_yz_urad; }
    
    // Apply volumetric correction to position
    void sphere_apply_correction(int32_t& x, int32_t& y, int32_t& z) const;
    
    // Dump sphere calibration status and errors
    void dump_sphere_cal(kernel::hal::UARTDriverOps* uart) const;

    // Apply all compensations to a commanded position (X, Y, Z).
    // Applies linear PEC, rotary offset, and geometry transformation.
    void apply_compensation(int32_t& x, int32_t& y, int32_t& z) const;

    // Task 9.9 — simulated continuous-rotation leader. When non-zero, each
    // cycle adds `velocity_cps * 250 µs / 1e6` counts to the axis's
    // `actual_position_feedback`. Pure simulation — only safe when the
    // axis has no backing drive (`drive == nullptr`); real drives would
    // reject the synthetic feedback. Zero disables. Used to validate
    // gear phase-lock without spinning a physical spindle.
    [[nodiscard]] bool set_spin_velocity(size_t axis_idx, int32_t velocity_cps) noexcept;

private:
    std::array<Axis, MAX_AXES>           axes_{};
    std::array<Channel, MAX_CHANNELS>    channels_{};
    size_t                               channel_count_ = 0;
    std::array<Barrier, MAX_BARRIERS>    barriers_{};
    std::array<GearLink, MAX_GEARS>      gears_{};
    std::array<GantryLink, MAX_GANTRYS>  gantrys_{};
    std::array<LinearCalibration, MAX_AXES> linear_cal_{};
    std::array<RotaryCalibration, MAX_ROTARY_CAL> rotary_cal_{};
    MachineGeometry                       geometry_{};
    SphereCalibration                     sphere_cal_{};
    MotionStats                          stats_;
    DriveStateMachine                    fsa_;

    void run_loop();
    void cycle_channel(Channel& ch, uint64_t now_us) noexcept;
    // `freeze_trajectory` = true holds commanded_position at its current
    // value (no step_trajectory call) while still pumping the CiA-402
    // FSA + drive I/O. Set when the owning channel is stalled at a
    // barrier / gearing primitive (task 9.4+).
    void cycle_axis(Axis& a, uint64_t now_us, bool freeze_trajectory = false) noexcept;

    // Phase 9.2: after every channel has advanced for this cycle, the sync
    // arbiter runs once to apply cross-channel rules (barrier release,
    // gearing follower update, cross-channel fault propagation) before
    // the drive-I/O commit happens inside cycle_axis. Kept empty until
    // 9.4 lands so single-channel behaviour is unchanged.
    void run_sync_arbiter(uint64_t now_us) noexcept;
};

extern Kernel g_motion;

} // namespace motion

#endif // MOTION_MOTION_HPP
