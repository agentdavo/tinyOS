// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Motion kernel implementation. See motion.hpp for the design rationale.

#include "motion.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include "diag/jitter.hpp"
#include "ethercat/cia402.hpp"
#include "ethercat/master.hpp"

#include <cstdlib>

namespace motion {

Kernel g_motion;

// -----------------------------------------------------------------------------
// Kernel ctor — install two default channels split at MAX_AXES/2 (Phase 9.3).
//
// Today's default topology: ch0 "mill" owns axes 0..MAX_AXES/2-1 (16 axes),
// ch1 "lathe" owns axes MAX_AXES/2..MAX_AXES-1 (16 axes). This is the
// mill-turn layout the precision-budget design targets; real deployments
// will override it via the topology config file (task 9.8). Behaviourally
// equivalent to the pre-9.3 single-channel kernel for any workload that
// only touches axes 0..15 — ch1 simply sits idle.
// -----------------------------------------------------------------------------
Kernel::Kernel() noexcept {
    static_assert(MAX_AXES <= 255, "axis_indices is uint8_t");
    static_assert(MAX_AXES % 2 == 0, "split assumes an even MAX_AXES");
    constexpr size_t half = MAX_AXES / 2;

    auto& ch0 = channels_[0];
    ch0.name[0]='m'; ch0.name[1]='i'; ch0.name[2]='l'; ch0.name[3]='l'; ch0.name[4]=0;
    for (size_t i = 0; i < half; ++i) {
        ch0.axis_indices[i] = static_cast<uint8_t>(i);
    }
    ch0.axis_count = static_cast<uint8_t>(half);
    ch0.state      = ChannelState::Running;

    auto& ch1 = channels_[1];
    ch1.name[0]='l'; ch1.name[1]='a'; ch1.name[2]='t'; ch1.name[3]='h'; ch1.name[4]='e'; ch1.name[5]=0;
    for (size_t i = 0; i < half; ++i) {
        ch1.axis_indices[i] = static_cast<uint8_t>(half + i);
    }
    ch1.axis_count = static_cast<uint8_t>(half);
    ch1.state      = ChannelState::Running;

    channel_count_ = 2;
}

namespace {

// Delegates to TimerDriverOps::wait_until_ns — HAL picks busy-spin vs WFI.
inline void wait_until_us(uint64_t target_us) {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!t) return;
    t->wait_until_ns(target_us * 1000ULL);
}

inline uint64_t now_us() {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return t ? t->get_system_time_us() : 0;
}

// Status-word bit masks per CiA-402 (0x6041).
constexpr uint16_t SW_READY_TO_SWITCH_ON = 1u << 0;
constexpr uint16_t SW_SWITCHED_ON        = 1u << 1;
constexpr uint16_t SW_OPERATION_ENABLED  = 1u << 2;
constexpr uint16_t SW_FAULT              = 1u << 3;
constexpr uint16_t SW_VOLTAGE_ENABLED    = 1u << 4;
constexpr uint16_t SW_QUICK_STOP_N       = 1u << 5;   // 0 = quick-stop active
constexpr uint16_t SW_SWITCH_ON_DISABLED = 1u << 6;

} // namespace

// -----------------------------------------------------------------------------
// DriveStateMachine
// -----------------------------------------------------------------------------

DriveState DriveStateMachine::decode(uint16_t s) const noexcept {
    // Unified with `cia402::decode_state` — the motion-side enum is a
    // 1:1 mapping of the ethercat-side enum, we just re-tag. The
    // cia402 implementation handles the full CiA-402 §6.3 state
    // transition table; the older motion-side variant disagreed on
    // FaultReactionActive (`0x004F == 0x000F && !SOD`) vs the
    // cia402-side (`0x000F == 0x000F`). Trusting the ethercat
    // implementation since it's what the drive-side FSA walker
    // (`cia402::controlword_for_transition`) expects.
    switch (cia402::decode_state(s)) {
        case cia402::State::NotReadyToSwitchOn:  return DriveState::NotReady;
        case cia402::State::SwitchOnDisabled:    return DriveState::SwitchOnDisabled;
        case cia402::State::ReadyToSwitchOn:     return DriveState::ReadyToSwitchOn;
        case cia402::State::SwitchedOn:          return DriveState::SwitchedOn;
        case cia402::State::OperationEnabled:    return DriveState::OperationEnabled;
        case cia402::State::QuickStopActive:     return DriveState::QuickStopActive;
        case cia402::State::FaultReactionActive: return DriveState::FaultReactionActive;
        case cia402::State::Fault:               return DriveState::Fault;
    }
    return DriveState::NotReady;
}

uint16_t DriveStateMachine::next_control_word(uint16_t status, Mode mode) const noexcept {
    using CB = ControlwordBuilder;
    uint16_t cw = 0;
    const DriveState cur = decode(status);
    switch (cur) {
        case DriveState::Fault:
            // Pulse Fault-Reset; other bits cleared.
            cw = CB::FAULT_RESET;
            break;
        case DriveState::SwitchOnDisabled:
            // Step 1 of the two-step handshake: bits 1,2 up (shutdown command).
            cw = CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N;
            break;
        case DriveState::ReadyToSwitchOn:
            // Switch On: add bit 0.
            cw = CB::SWITCH_ON | CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N;
            break;
        case DriveState::SwitchedOn:
            // Enable Operation: add bit 3.
            cw = CB::SWITCH_ON | CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N | CB::ENABLE_OPERATION;
            break;
        case DriveState::OperationEnabled:
            // Hold the operation-enabled pattern steady.
            cw = CB::SWITCH_ON | CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N | CB::ENABLE_OPERATION;
            break;
        case DriveState::QuickStopActive:
            // Disable Voltage to exit quick-stop cleanly.
            cw = 0;
            break;
        case DriveState::FaultReactionActive:
        case DriveState::NotReady:
        default:
            cw = 0;
            break;
    }

    // Task 1.6 — servo-on is a two-step handshake. The drive MUST see
    // SwitchedOn before we add ENABLE_OPERATION; raising bits 0..3
    // simultaneously from SwitchOnDisabled skips the Switched-On latch
    // and the ClearPath rejects the transition (see clearpath_ec.md
    // "Servo-on is a two-step handshake"). The switch above never emits
    // all four bits except from the SwitchedOn / OperationEnabled
    // branches, which is exactly where the handshake allows it; if a
    // future edit breaks that, trap here in debug so it's caught at
    // source rather than as a mysterious drive Fault.
    const uint16_t enable_mask = CB::SWITCH_ON | CB::ENABLE_VOLTAGE |
                                  CB::QUICK_STOP_N | CB::ENABLE_OPERATION;
    if ((cw & enable_mask) == enable_mask &&
        cur != DriveState::SwitchedOn &&
        cur != DriveState::OperationEnabled) {
        __builtin_trap();
    }

    return CB::sanitize(cw, mode);
}

uint16_t DriveStateMachine::stop_control_word(StopAction action, Mode mode) const noexcept {
    using CB = ControlwordBuilder;
    uint16_t cw = 0;
    switch (action) {
        case StopAction::QuickStop:
            // Clear bit 2 (QUICK_STOP_N = 0 triggers quick stop).
            cw = CB::ENABLE_VOLTAGE;
            break;
        case StopAction::Halt:
            // Halt requires Operation-Enabled bits plus bit 8.
            cw = CB::SWITCH_ON | CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N |
                 CB::ENABLE_OPERATION | CB::HALT;
            break;
        case StopAction::Disable:
            // Shutdown pattern - bits 1,2 only.
            cw = CB::ENABLE_VOLTAGE | CB::QUICK_STOP_N;
            break;
        case StopAction::FaultReaction:
        case StopAction::Abort:
            cw = 0;  // Disable Voltage
            break;
    }
    return CB::sanitize(cw, mode);
}

// -----------------------------------------------------------------------------
// HomingEngine
// -----------------------------------------------------------------------------

const char* HomingEngine::phase_name(Phase p) noexcept {
    switch (p) {
        case Phase::Idle:       return "idle";
        case Phase::Fast:       return "fast";
        case Phase::Creep:      return "creep";
        case Phase::Triggered:  return "triggered";
        case Phase::BackingOff: return "backoff";
        case Phase::Done:       return "done";
        case Phase::Fault:      return "fault";
    }
    return "?";
}

void HomingEngine::start(const HomingPlan& plan) noexcept {
    plan_        = plan;
    phase_       = Phase::Fast;
    trigger_pos_ = 0;
    prev_level_  = false;
}

void HomingEngine::abort() noexcept {
    phase_ = Phase::Idle;
}

HomingEngine::Phase HomingEngine::step(uint16_t status,
                                       int32_t  pos,
                                       int16_t  torque_permille) noexcept {
    // Extract the trigger level. In a real build this would route to the
    // correct PDO slot per TriggerSource; for the kernel shape we compose the
    // "is the trigger asserted?" boolean.
    bool level = false;
    switch (plan_.trigger) {
        case TriggerSource::LimitSwitch:
            // Use bit 11/12 of statusword as a stand-in for limit inputs.
            level = (status & (plan_.approach_positive ? (1u << 14) : (1u << 15))) != 0;
            break;
        case TriggerSource::IndexPulse:
        case TriggerSource::TouchProbe:
            level = (status & (1u << 13)) != 0;  // Manufacturer-specific probe.
            break;
        case TriggerSource::TorqueThreshold: {
            int16_t t = torque_permille < 0 ? int16_t(-torque_permille) : torque_permille;
            level = t >= plan_.torque_threshold_permille;
            break;
        }
        case TriggerSource::EncoderPosition:
            // Reach a target absolute position with the right sign.
            level = plan_.approach_positive ? (pos >= plan_.backoff_counts)
                                            : (pos <= plan_.backoff_counts);
            break;
    }
    const bool edge = plan_.rising_edge ? (level && !prev_level_)
                                        : (!level && prev_level_);
    prev_level_ = level;

    switch (phase_) {
        case Phase::Idle:
        case Phase::Done:
        case Phase::Fault:
            break;
        case Phase::Fast:
            if (edge) {
                trigger_pos_ = pos;
                phase_       = Phase::Triggered;
            }
            break;
        case Phase::Triggered:
            phase_ = Phase::BackingOff;
            break;
        case Phase::BackingOff: {
            const int32_t delta = pos - trigger_pos_;
            const int32_t need  = plan_.approach_positive ? -plan_.backoff_counts
                                                          :  plan_.backoff_counts;
            const bool reached = plan_.approach_positive ? (delta <= need)
                                                         : (delta >= need);
            if (reached) phase_ = Phase::Creep;
            break;
        }
        case Phase::Creep:
            // Creep until a confirm edge re-asserts the trigger.
            if (edge) phase_ = Phase::Done;
            break;
    }
    return phase_;
}

// -----------------------------------------------------------------------------
// Axis helpers
// -----------------------------------------------------------------------------

bool Axis::tick_brake(uint64_t t_now) noexcept {
    switch (brake_phase) {
        case BrakePhase::Released:
            return true;
        case BrakePhase::Releasing:
            if (t_now - brake_t_start_us >= brake.release_delay_us) {
                brake_phase = BrakePhase::Released;
                return true;
            }
            return false;
        case BrakePhase::Engaging:
            if (t_now - brake_t_start_us >= brake.engage_delay_us) {
                brake_phase = BrakePhase::Engaged;
            }
            return false;
        case BrakePhase::Engaged:
        default:
            return false;
    }
}

void Axis::request_stop(StopAction action, uint64_t t_now) noexcept {
    const StopKind kind = stops.for_action(action);
    // Coast and DynamicBrake disable the amplifier immediately but we still
    // honour Delay Disable Time if an external brake needs to latch first.
    // Decel/PositionHold keep the servo engaged and don't engage the brake.
    switch (kind) {
        case StopKind::PositionHold:
        case StopKind::Decel:
            // Servo stays live; brake stays released.
            break;
        case StopKind::DynamicBrake:
        case StopKind::Coast:
            if (brake_phase == BrakePhase::Released || brake_phase == BrakePhase::Releasing) {
                brake_phase      = BrakePhase::Engaging;
                brake_t_start_us = t_now;
            }
            break;
    }
}

void Axis::on_connection_lost(uint64_t t_now) noexcept {
    connection_lost = true;
    if (mode == Mode::CSP) {
        // CSP abrupt-stops on connection loss regardless of 0x6007.
        brake_phase      = BrakePhase::Engaging;
        brake_t_start_us = t_now;
        return;
    }
    request_stop(StopAction::Abort, t_now);
}

// -----------------------------------------------------------------------------
// Trajectory generator — simple time-optimal trapezoid.
//
// Each 200 µs call advances the commanded position by one step of
//   v = v + a*dt   (clamped to +/- vmax)
//   p = p + v*dt
// with a braking check: if stopping from current v at the current accel would
// overshoot `target`, we decelerate this tick instead of accelerating.
//
// Integer-only; units are counts, counts/s, counts/s^2. dt_us is microseconds.
// -----------------------------------------------------------------------------
// S-curve trajectory with jerk limiting
// Uses jerk_cps3 for smooth acceleration/deceleration transitions
// -----------------------------------------------------------------------------
void Axis::step_trajectory(uint32_t dt_us) noexcept {
    if (traj_state == TrajState::Idle || traj_state == TrajState::Holding) {
        target_velocity = 0;
        current_accel_cps2 = 0;
        return;
    }

    // Power skiving: if spindle indexer is active, compute indexed position
    if (spindle_indexer.active && spindle_indexer.configured) {
        const int32_t indexed_target = spindle_indexer.compute_indexed_position(
            target, commanded_position);
        target = indexed_target;
    }

    if (traj_state == TrajState::MoveReady) {
        target_velocity       = 0;
        current_accel_cps2    = 0;
        max_following_error   = 0;
        traj_start_pos        = commanded_position;
        prev_target_velocity = 0;
        traj_state            = TrajState::Accel;
    }

    const int32_t dist = target - commanded_position;
    if (dist == 0 && target_velocity == 0) {
        traj_state = TrajState::Holding;
        return;
    }

    const int32_t dir         = (dist > 0) ? 1 : -1;
    const int32_t abs_dist   = dir > 0 ? dist : -dist;
    const int32_t abs_v      = target_velocity >= 0 ? target_velocity : -target_velocity;

    // S-curve profiling with jerk limiting
    // Compute jerk-limited acceleration for this tick
    int32_t jerk_limited_accel = accel_cps2;
    if (jerk_cps3 > 0 && dt_us > 0) {
        const int64_t da_max = (int64_t)jerk_cps3 * (int64_t)dt_us / 1'000'000LL;
        const int32_t da = static_cast<int32_t>(da_max);
        
        // Smoothly approach target acceleration
        if (current_accel_cps2 < accel_cps2) {
            current_accel_cps2 += da;
            if (current_accel_cps2 > accel_cps2) current_accel_cps2 = accel_cps2;
        } else if (current_accel_cps2 > accel_cps2) {
            current_accel_cps2 -= da;
            if (current_accel_cps2 < accel_cps2) current_accel_cps2 = accel_cps2;
        }
        jerk_limited_accel = current_accel_cps2;
    }

    // Distance to brake from current velocity using jerk-limited accel
    int64_t stop_dist = 0;
    if (jerk_limited_accel > 0) {
        stop_dist = (int64_t)abs_v * abs_v;
        stop_dist /= (2LL * jerk_limited_accel);
    }

    // Determine phase and acceleration
    bool decel = false;
    if (stop_dist >= abs_dist) {
        decel = true;
        traj_state = TrajState::Decel;
    } else if (abs_v >= vmax_cps) {
        traj_state = TrajState::ConstantVel;
    } else {
        traj_state = TrajState::Accel;
    }

    // Apply acceleration or deceleration
    int32_t a_tick = decel ? jerk_limited_accel : jerk_limited_accel;
    if (decel) a_tick = -a_tick;

    const int64_t dv_num = (int64_t)a_tick * (int64_t)dt_us;
    const int32_t dv = (int32_t)(dv_num / 1'000'000LL);

    target_velocity += dv;

    // Clamp to vmax and handle direction
    if (dir > 0) {
        if (target_velocity > vmax_cps) target_velocity = vmax_cps;
        if (target_velocity < -vmax_cps) target_velocity = -vmax_cps;
    } else {
        if (target_velocity < -vmax_cps) target_velocity = -vmax_cps;
        if (target_velocity > vmax_cps) target_velocity = vmax_cps;
    }

    // Integrate position
    const int64_t dp_num = (int64_t)target_velocity * (int64_t)dt_us;
    const int32_t dp = (int32_t)(dp_num / 1'000'000LL);

    int32_t new_pos = commanded_position + dp;
    if (dir > 0 && new_pos >= target) {
        new_pos = target;
        target_velocity = 0;
    } else if (dir < 0 && new_pos <= target) {
        new_pos = target;
        target_velocity = 0;
    }
    commanded_position = new_pos;

    if (commanded_position == target && target_velocity == 0) {
        traj_state = TrajState::Holding;
        current_accel_cps2 = 0;
    }

    prev_target_velocity = target_velocity;
}

// -----------------------------------------------------------------------------
// CSV (Cyclic Synchronous Velocity) trajectory step
// For spindle velocity control: commanded_position += target_velocity * dt
// -----------------------------------------------------------------------------
void Axis::step_csv_trajectory(uint32_t dt_us) noexcept {
    // CSV mode: target_velocity is the velocity setpoint (counts/s)
    // commanded_position accumulates based on velocity (no position target)

    // Apply velocity command directly to position integration
    // target_velocity is already in counts/s from the gear link or CLI
    const int64_t dp_num = (int64_t)target_velocity * (int64_t)dt_us;
    const int32_t dp = (int32_t)(dp_num / 1'000'000LL);

    commanded_position += dp;

    // Update target_pos mirror for CLI
    target_pos.store(commanded_position, std::memory_order_relaxed);
}

// -----------------------------------------------------------------------------
// Spindle gearing: convert RPM ratio to position ratio
// For power skiving: leader (spindle) RPM → follower (B-axis) position
// k_eff = (leader_rpm * leader_counts_per_rev) / (follower_counts_per_rev)
// -----------------------------------------------------------------------------
int32_t Axis::compute_spindle_gear_position(int32_t leader_actual_pos,
                                             int32_t leader_counts_per_rev,
                                             int32_t follower_base,
                                             int32_t k_num,
                                             int32_t k_den) const noexcept {
    if (k_den == 0) return follower_base;

    // Convert leader position to revolutions
    const int64_t leader_revs = leader_actual_pos / leader_counts_per_rev;

    // Apply ratio: follower = leader_revs * k_num / k_den * follower_counts
    // We compute in leader units then scale to follower
    const int64_t follower_pos_64 = (leader_revs * k_num) / k_den;

    return follower_base + static_cast<int32_t>(follower_pos_64);
}

// -----------------------------------------------------------------------------
// Kernel
// -----------------------------------------------------------------------------

void Kernel::request_mode(size_t i, Mode mode) noexcept {
    if (i >= MAX_AXES) return;
    axes_[i].mode = mode;
}

void Kernel::start_homing(size_t i, const HomingPlan& plan) noexcept {
    if (i >= MAX_AXES) return;
    axes_[i].mode = Mode::Homing;
    axes_[i].homing.start(plan);
    stats_.homings_started.fetch_add(1, std::memory_order_relaxed);
}

void Kernel::request_stop(size_t i, StopAction action) noexcept {
    if (i >= MAX_AXES) return;
    axes_[i].request_stop(action, now_us());
}

void Kernel::on_connection_lost(size_t i) noexcept {
    if (i >= MAX_AXES) return;
    axes_[i].on_connection_lost(now_us());
    stats_.connection_losses.fetch_add(1, std::memory_order_relaxed);
}

void Kernel::move_to(size_t i, int32_t position) noexcept {
    if (i >= MAX_AXES) return;
    auto& a = axes_[i];
    a.target     = position;
    a.mode       = Mode::CSP;
    a.enabled    = true;
    a.traj_state = TrajState::MoveReady;
}

void Kernel::set_axis_velocity(size_t i, int32_t velocity_cps) noexcept {
    if (i >= MAX_AXES) return;
    auto& a = axes_[i];
    a.target_velocity = velocity_cps;
    a.mode = Mode::CSV;
    a.enabled = true;
    a.traj_state = TrajState::Accel;
}

void Kernel::set_encoder_resolution(size_t i, uint32_t cpr) noexcept {
    if (i >= MAX_AXES || cpr == 0) return;
    auto& a = axes_[i];
    a.counts_per_rev  = cpr;
    a.counts_per_unit = static_cast<float>(cpr);
}

void Kernel::configure_encoder(size_t i, uint8_t source,
                              uint32_t motor_cpr, uint32_t external_cpr,
                              uint32_t gear_num, uint32_t gear_den,
                              int32_t sign) noexcept {
    if (i >= MAX_AXES) return;
    auto& a = axes_[i];
    if (source <= 2) {
        a.encoder.source = static_cast<Axis::EncoderSource>(source);
    }
    a.encoder.motor_cpr = motor_cpr;
    a.encoder.external_cpr = external_cpr;
    a.encoder.gear_num = (gear_num > 0) ? gear_num : 1;
    a.encoder.gear_den = (gear_den > 0) ? gear_den : 1;
    a.encoder.sign = (sign >= 0) ? 1 : -1;
    if (motor_cpr > 0) {
        a.counts_per_rev = motor_cpr;
    }
}

void Kernel::configure_spindle_indexer(size_t i, uint16_t teeth, uint32_t cpr, int32_t offset) noexcept {
    if (i >= MAX_AXES) return;
    auto& a = axes_[i];
    a.spindle_indexer.configure(teeth, cpr, offset);
}

bool Kernel::engage_spindle_gear(size_t leader, size_t follower, int32_t k_num, int32_t k_den, uint16_t ramp) noexcept {
    if (leader >= MAX_AXES || follower >= MAX_AXES) return false;
    if (k_den == 0) return false;

    auto& leader_axis = axes_[leader];
    auto& follower_axis = axes_[follower];

    // Use the spindle gearing method for position conversion
    leader_axis.mode = Mode::CSV;
    follower_axis.mode = Mode::CSP;

    // Configure spindle indexer on follower with leader's counts per rev
    follower_axis.spindle_indexer.configure(
        static_cast<uint16_t>(k_num),
        leader_axis.counts_per_rev,
        0);

    // Engage regular gear link but override in cycle_axis for spindle indexing
    return engage_gear(static_cast<uint8_t>(leader), static_cast<uint8_t>(follower),
                        0, k_num, k_den, ramp);
}

bool Kernel::fault_reset(size_t i) noexcept {
    if (i >= MAX_AXES) return false;
    auto& a = axes_[i];
    if (!a.drive) return false;

    // CiA-402 §6.3: Fault → SwitchOnDisabled requires a 0→1 edge on bit 7.
    // The drive's own step() machinery will re-assert the normal controlword
    // on the next cycle once statusword reports SwitchOnDisabled, so we only
    // need to set the bit here — the drive clears it on state transition.
    uint16_t cw = a.drive->controlword;
    cw |= cia402::CW_FAULT_RESET;
    __atomic_store_n(&a.drive->controlword, cw, __ATOMIC_RELAXED);

    // Drop our own latch so the trajectory can run once the drive walks
    // back to OperationEnabled. Don't touch `enabled` — the operator's
    // `move` (or equivalent) latched that, and we preserve intent.
    a.fault_latched = false;
    return true;
}

void Kernel::cycle_axis(Axis& a, uint64_t t_now, bool freeze_trajectory) noexcept {
    a.tick_brake(t_now);
    const uint16_t status = a.status_word.load(std::memory_order_relaxed);
    a.state = fsa_.decode(status);

    if (a.state == DriveState::Fault) {
        stats_.faults.fetch_add(1, std::memory_order_relaxed);
    }

    // Walk the CiA-402 FSA toward Operation-Enabled, but only while enabled
    // and not in an active stop / connection-loss state.
    if (a.enabled && !a.connection_lost && a.brake_phase != BrakePhase::Engaged) {
        const uint16_t cw = fsa_.next_control_word(status, a.mode);
        a.control_word.store(cw, std::memory_order_relaxed);
    } else {
        // Engaged / lost: emit a sanitized Disable-Voltage controlword.
        a.control_word.store(
            fsa_.stop_control_word(StopAction::Disable, a.mode),
            std::memory_order_relaxed);
    }

    // Run homing if active.
    if (a.mode == Mode::Homing && a.homing.phase() != HomingEngine::Phase::Idle) {
        const auto ph = a.homing.step(
            status,
            a.actual_pos.load(std::memory_order_relaxed),
            a.actual_torque_permille.load(std::memory_order_relaxed));
        if (ph == HomingEngine::Phase::Done) {
            stats_.homings_done.fetch_add(1, std::memory_order_relaxed);
            a.mode = Mode::CSP;
        }
    }

    // Task 2.4 — fire the pending 0x607D push when the drive confirms
    // Has-Homed (statusword bit 8) AND the operator armed limits via
    // `arm_post_homing_limits`. Fire-once: clear the pending flag so
    // subsequent cycles don't spam SDO writes. Statusword is the fresh
    // one this cycle (status variable above).
    if (a.sw_limits_pending && (status & (1u << 8)) && a.drive && a.station_addr != 0) {
        const uint16_t station = a.station_addr;
        uint8_t data[4] = {};
        data[0] = static_cast<uint8_t>(a.sw_limit_neg_counts);
        data[1] = static_cast<uint8_t>(a.sw_limit_neg_counts >> 8);
        data[2] = static_cast<uint8_t>(a.sw_limit_neg_counts >> 16);
        data[3] = static_cast<uint8_t>(a.sw_limit_neg_counts >> 24);
        (void)ethercat::g_master_a.send_sdo_download(station, 0x607D, 0x01, data, 4);
        data[0] = static_cast<uint8_t>(a.sw_limit_pos_counts);
        data[1] = static_cast<uint8_t>(a.sw_limit_pos_counts >> 8);
        data[2] = static_cast<uint8_t>(a.sw_limit_pos_counts >> 16);
        data[3] = static_cast<uint8_t>(a.sw_limit_pos_counts >> 24);
        (void)ethercat::g_master_a.send_sdo_download(station, 0x607D, 0x02, data, 4);
        a.sw_limits_pending = false;
    }

    // --- Drive I/O + trajectory gating --------------------------------------
    // Cross-core race note: motion runs on core 1, master on core 2. The
    // cia402::Drive fields are plain int32_t — not std::atomic — because the
    // master already treats them as "I own these for the duration of a
    // cycle". On AArch64 a naturally-aligned 32-bit load/store is
    // single-copy atomic at the ISA level, so torn values are impossible;
    // the worst case is motion reads a value one LRW cycle stale (200 µs),
    // well inside the servo budget. We use __atomic_* built-ins to signal
    // intent without a type change in cia402.hpp.

    if (a.drive) {
        const int32_t fb = __atomic_load_n(&a.drive->actual_position,
                                           __ATOMIC_RELAXED);
        a.actual_position_feedback = fb;
        a.actual_pos.store(fb, std::memory_order_relaxed);

        // Mirror the drive's statusword into the axis so fsa_.decode above
        // sees fresh data on the *next* cycle. (decode already ran earlier
        // in this tick with the previous cycle's value — that's fine; one
        // servo period of latency is the intended design.)
        a.status_word.store(a.drive->statusword, std::memory_order_relaxed);

        // Task 7.4/7.5 — decode hall sensors from digital_inputs (0x60FD).
        // This runs every cycle when the drive is live.
        const uint32_t din = __atomic_load_n(&a.drive->digital_inputs,
                                             __ATOMIC_RELAXED);
        (void)a.hall.decode(din);

        // Fault propagation: if the drive reports Fault or FaultReactionActive,
        // latch the condition and drive the axis through its FaultReaction
        // stop. We stop advancing the trajectory (gate below) and stop
        // publishing target_position. Clear via Kernel::fault_reset which
        // pulses CW_FAULT_RESET on the controlword PDO.
        const bool drive_fault =
            a.drive->state == cia402::State::Fault ||
            a.drive->state == cia402::State::FaultReactionActive;
        if (drive_fault && !a.fault_latched) {
            a.fault_latched = true;
            // Task 1.5 — latch 0x603F at the instant we decide this is a fault
            // so CLI dumps can print a meaningful reason even after
            // fault_reset clears the drive-side state.
            a.last_error_code = a.drive->error_code;
            a.request_stop(StopAction::FaultReaction, t_now);
            stats_.faults.fetch_add(1, std::memory_order_relaxed);
        }
        // NB: we do NOT auto-clear fault_latched on drive recovery. The
        // operator must explicitly acknowledge via Kernel::fault_reset —
        // otherwise a drive that toggles Fault → OperationEnabled inside a
        // single `mpos` cycle would silently resume commanding motion
        // without anyone knowing there was a fault.

        // Safety gate: only run the trapezoidal generator and publish
        // setpoints when the drive reports OperationEnabled *and* we haven't
        // latched a fault. Before enable (ReadyToSwitchOn/SwitchedOn/
        // NotReadyToSwitchOn/...) we mirror commanded_position to
        // actual_position so the first tick after enable doesn't
        // step-command a large delta.
        const bool drive_live =
            a.drive->state == cia402::State::OperationEnabled &&
            !a.fault_latched;

        if (drive_live) {
            // Task 9.4 — freeze commanded_position while the owning channel
            // is stalled at a barrier/gearing primitive. Trajectory state
            // (target, target_velocity, traj_state) is preserved so the
            // move resumes cleanly on release.
            if (!freeze_trajectory) a.step_trajectory(DEFAULT_PERIOD_US);
            const int32_t ferr  = a.commanded_position - fb;
            const int32_t aferr = ferr >= 0 ? ferr : -ferr;
            if (aferr > a.max_following_error) a.max_following_error = aferr;

            // Task 9.6 — once a sync-scaled move lands in Holding, restore
            // the axis's original vmax. `restore_vmax_cps == 0` means no
            // scale is pending, so this is a no-op for normal moves.
            if (a.traj_state == TrajState::Holding && a.restore_vmax_cps > 0) {
                a.vmax_cps         = a.restore_vmax_cps;
                a.restore_vmax_cps = 0;
            }

            // Task 7.6 — cycle-counter watchdog. If the publisher hasn't
            // bumped `samples_received` since the last cycle, the sample
            // is stale this tick. After `max_stale_cycles` consecutive
            // stale cycles (default 2) we invalidate the channel and let
            // the decay-trim path run out below.
            //
            // Task 7.8 — any persistent load-side error (EL5042 status
            // bit, watchdog expiry) raises the Axis's own fault_latched
            // so downstream (motion dump, brake FSA, connection-loss)
            // treats it identically to a drive-side fault.
            if (a.load.configured) {
                const bool fresh_this_cycle =
                    a.load.samples_received != a.load.samples_last_seen;
                if (fresh_this_cycle) {
                    a.load.samples_last_seen = a.load.samples_received;
                    a.load.stale_cycles      = 0;
                } else if (a.load.stale_cycles < 0xFFu) {
                    ++a.load.stale_cycles;
                }
                const bool stale = a.load.stale_cycles >= a.load.max_stale_cycles;
                a.load.valid = fresh_this_cycle && !a.load.error_latched && !stale;
                // Fault fan-out — the load side counts as a drive-equivalent
                // fault so the motion kernel's stop matrix can kick in.
                if ((stale || a.load.error_latched) && !a.fault_latched) {
                    a.fault_latched  = true;
                    a.last_error_code = 0xFFDA; // vendor-internal ~ load-side
                    a.request_stop(StopAction::FaultReaction, t_now);
                    stats_.faults.fetch_add(1, std::memory_order_relaxed);
                }
            }

            // Task 7.4/7.5 — outer PI on load-side feedback. Runs only when
            // `load.configured` (the bus_config helper flips this on once
            // an EL5042 BiSS-C is bound to this axis) AND this cycle's
            // load sample is valid. Output is a small trim added to the
            // CSP target below; the inner ClearPath loop still closes on
            // motor-shaft position, the outer loop just nudges it.
            if (a.load.configured && a.load.valid) {
                // Scaled load position in axis counts.
                const int64_t lp_scaled =
                    (a.load.raw_position * a.load.scale_num) / a.load.scale_den;
                a.load_position_counts =
                    static_cast<int32_t>(a.load.sign * (lp_scaled - a.load.offset_counts));

                // Outer error = load - commanded. If the load trails, the
                // trim is positive (pushes motor forward) until they match.
                const int32_t lerr = a.load_position_counts - a.commanded_position;
                a.load_following_error = lerr;

                // PI. Gains scaled ×1000 so `kp_ppm=500` means +0.5 trim-count
                // per load-count of error. Integrator is anti-wind-up clipped
                // to ±trim_cap_counts * 1000 so it can never dominate.
                a.load.integral += static_cast<int64_t>(lerr);
                const int64_t int_cap = static_cast<int64_t>(a.load.trim_cap_counts) * 1000;
                if (a.load.integral >  int_cap) a.load.integral =  int_cap;
                if (a.load.integral < -int_cap) a.load.integral = -int_cap;
                int64_t raw_trim = (static_cast<int64_t>(a.load.kp_ppm) * lerr
                                   + static_cast<int64_t>(a.load.ki_ppm) * a.load.integral
                                      / 1000) / 1000;
                // Cap magnitude.
                if (raw_trim >  a.load.trim_cap_counts) raw_trim =  a.load.trim_cap_counts;
                if (raw_trim < -a.load.trim_cap_counts) raw_trim = -a.load.trim_cap_counts;
                // Slew-limit per cycle. 5000 cps × 250 µs = 1.25 counts/cycle,
                // rounded up to 1 (integer math). Scale properly:
                //   max_step = trim_slew_cps * dt_us / 1e6
                const int32_t max_step = static_cast<int32_t>(
                    (static_cast<int64_t>(a.load.trim_slew_cps) * DEFAULT_PERIOD_US) / 1'000'000LL);
                const int32_t step_cap = max_step > 0 ? max_step : 1;
                int32_t trim = static_cast<int32_t>(raw_trim);
                if (trim - a.load.last_trim >  step_cap) trim = a.load.last_trim + step_cap;
                if (trim - a.load.last_trim < -step_cap) trim = a.load.last_trim - step_cap;
                a.load.last_trim = trim;
                a.outer_trim     = trim;
            } else if (a.load.configured && !a.load.valid) {
                // Sample invalid — freeze the trim (don't keep integrating
                // a stale error) and decay back toward zero over the slew
                // limit so a recovered encoder doesn't snap.
                const int32_t max_step = static_cast<int32_t>(
                    (static_cast<int64_t>(a.load.trim_slew_cps) * DEFAULT_PERIOD_US) / 1'000'000LL);
                const int32_t step_cap = max_step > 0 ? max_step : 1;
                if (a.load.last_trim > 0) {
                    a.load.last_trim = (a.load.last_trim > step_cap)
                        ? (a.load.last_trim - step_cap) : 0;
                } else if (a.load.last_trim < 0) {
                    a.load.last_trim = (a.load.last_trim < -step_cap)
                        ? (a.load.last_trim + step_cap) : 0;
                }
                a.outer_trim = a.load.last_trim;
            } else {
                a.outer_trim = 0;
            }
        } else {
            // Hold at feedback position. Any pending `move` target stays
            // latched in `a.target`; traj will replay it on re-enable.
            a.commanded_position = fb;
            a.target_velocity    = 0;
        }

        // Task 7.5 — final setpoint = inner-loop commanded + outer-loop trim.
        // `outer_trim` is zero unless the load pipeline is configured AND
        // this cycle's sample was valid; cap + slew limiting already
        // happened so we just add. The ClearPath's inner CSP loop closes
        // on this summed value; the motor shaft chases it, and the load
        // encoder tells the outer loop how well it's keeping up.
        const int32_t final_target = a.commanded_position + a.outer_trim;
        __atomic_store_n(&a.drive->target_position, final_target, __ATOMIC_RELAXED);
        a.target_pos.store(final_target, std::memory_order_relaxed);

        if (a.drive->mode_op != cia402::Mode::CyclicSyncPosition) {
            a.drive->mode_op = cia402::Mode::CyclicSyncPosition;
        }
    } else {
        // No drive hooked — still run the generator so dry-run diagnostics
        // (e.g. trajectory shape inspection) work without a backing slave.
        // Freeze honoured here too so a barrier-stalled channel holds
        // position even in pure-simulation mode.
        if (!freeze_trajectory) a.step_trajectory(DEFAULT_PERIOD_US);

        // Task 9.9 — optional synthetic spin for gear-harness validation.
        // Per-cycle increment: velocity_cps * dt_us / 1e6.
        if (a.spin_velocity_cps != 0) {
            const int64_t dp = (static_cast<int64_t>(a.spin_velocity_cps) *
                                DEFAULT_PERIOD_US) / 1'000'000LL;
            a.actual_position_feedback += static_cast<int32_t>(dp);
        }
    }
}

void Kernel::cycle_channel(Channel& ch, uint64_t t_now) noexcept {
    // Task 9.4 / 9.5 / 9.7 — a channel that's waiting at a barrier, geared
    // to a leader, or in operator-initiated feedhold must hold its axes'
    // commanded positions while the motion FSA and drive I/O keep
    // pumping. Trajectory advancement is the only thing suppressed.
    const bool freeze = (ch.state == ChannelState::WaitingBarrier) ||
                        (ch.state == ChannelState::Gearing)       ||
                        (ch.state == ChannelState::FeedHold);
    // Task 9.7 — per-channel fault domain. If any owned axis has its
    // fault_latched flag set, the channel goes to Fault *but only this
    // channel* — the other channels are untouched unless an active sync
    // primitive ties them in (9.4 timeout / explicit propagation). The
    // Running / Fault flip is one-way until the operator clears the
    // fault_reset on the offending axis.
    bool any_fault = false;
    for (uint8_t k = 0; k < ch.axis_count; ++k) {
        const uint8_t ai = ch.axis_indices[k];
        if (ai < MAX_AXES) cycle_axis(axes_[ai], t_now, freeze);
        if (ai < MAX_AXES && axes_[ai].fault_latched) any_fault = true;
    }
    if (any_fault && ch.state != ChannelState::Fault &&
        ch.state != ChannelState::WaitingBarrier &&
        ch.state != ChannelState::Gearing) {
        // FeedHold + Running both fall to Fault; the other two are
        // sync-primitive-owned and the arbiter decides their fate.
        ch.state = ChannelState::Fault;
    }
}

// Task 9.4 — helper: return true iff every axis owned by `ch` is within
// `tolerance_counts` of its commanded position. The arbiter uses this to
// decide whether a barrier participant has converged for this cycle.
static bool channel_is_stable(const Channel& ch,
                              const std::array<Axis, MAX_AXES>& axes,
                              int32_t tolerance_counts) noexcept {
    for (uint8_t k = 0; k < ch.axis_count; ++k) {
        const uint8_t ai = ch.axis_indices[k];
        if (ai >= MAX_AXES) continue;
        const auto& a = axes[ai];
        const int32_t ferr = a.commanded_position - a.actual_position_feedback;
        const int32_t abs_ferr = ferr >= 0 ? ferr : -ferr;
        if (abs_ferr > tolerance_counts) return false;
    }
    return true;
}

bool Kernel::arrive_at_barrier(size_t channel_idx,
                               uint16_t token,
                               uint8_t participants_mask,
                               int32_t tolerance_counts,
                               uint16_t stable_cycles_required,
                               uint16_t max_wait_cycles) noexcept {
    if (channel_idx >= channel_count_) return false;
    const uint8_t my_bit = static_cast<uint8_t>(1u << channel_idx);
    if ((participants_mask & my_bit) == 0) return false;

    // Find existing slot with this token, or allocate a fresh one.
    Barrier* slot = nullptr;
    for (auto& b : barriers_) {
        if (b.in_use && b.token == token) { slot = &b; break; }
    }
    if (!slot) {
        for (auto& b : barriers_) {
            if (!b.in_use) { slot = &b; break; }
        }
        if (!slot) return false; // barrier table full

        slot->in_use                 = true;
        slot->token                  = token;
        slot->participants_mask      = participants_mask;
        slot->arrivals_mask          = 0;
        slot->released               = 0;
        slot->tolerance_counts       = tolerance_counts;
        slot->stable_cycles_required = stable_cycles_required;
        slot->max_wait_cycles        = max_wait_cycles;
        slot->created_cycle          = stats_.cycles.load(std::memory_order_relaxed);
        for (auto& s : slot->stable_cycles) s = 0;
    } else if (slot->participants_mask != participants_mask) {
        // Late joiner with a different expected participant set — reject
        // rather than silently merge. Prevents two independently-written
        // interpreters from colliding on the same token by accident.
        return false;
    }

    slot->arrivals_mask = static_cast<uint8_t>(slot->arrivals_mask | my_bit);

    // Stall the channel and remember which token it's stalled on. The
    // arbiter flips state back to Running at release time.
    channels_[channel_idx].state         = ChannelState::WaitingBarrier;
    channels_[channel_idx].barrier_token = token;
    return true;
}

bool Kernel::configure_load_feedback(size_t axis_idx,
                                     int64_t scale_num, int64_t scale_den,
                                     int32_t sign,
                                     int32_t trim_cap_counts,
                                     int32_t trim_slew_cps,
                                     int16_t kp_ppm, int16_t ki_ppm,
                                     uint8_t max_stale_cycles) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    if (scale_den == 0) return false;
    if (sign != 1 && sign != -1) return false;
    auto& a = axes_[axis_idx];
    a.load.scale_num         = scale_num;
    a.load.scale_den         = scale_den;
    a.load.sign              = sign;
    a.load.offset_counts     = 0;
    a.load.trim_cap_counts   = trim_cap_counts;
    a.load.trim_slew_cps     = trim_slew_cps;
    a.load.kp_ppm            = kp_ppm;
    a.load.ki_ppm            = ki_ppm;
    a.load.max_stale_cycles  = max_stale_cycles;
    a.load.integral          = 0;
    a.load.last_trim         = 0;
    a.load.samples_received  = 0;
    a.load.samples_last_seen = 0;
    a.load.stale_cycles      = 0;
    a.load.error_latched     = false;
    a.load.raw_position      = 0;
    a.load.valid             = false;
    a.load.configured        = true;
    a.outer_trim             = 0;
    a.load_position_counts   = 0;
    a.load_following_error   = 0;
    return true;
}

bool Kernel::push_load_sample(size_t axis_idx, int64_t raw_position,
                              bool error) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    auto& a = axes_[axis_idx];
    if (!a.load.configured) return false;
    a.load.raw_position = raw_position;
    // Sample is fresh (even if error is flagged — the publisher has spoken
    // this cycle). `error_latched` drives the invalid path downstream.
    a.load.samples_received = static_cast<uint16_t>(a.load.samples_received + 1);
    a.load.error_latched    = error;
    return true;
}

bool Kernel::enable_axis(size_t axis_idx) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    axes_[axis_idx].enabled = true;
    return true;
}

bool Kernel::disable_axis(size_t axis_idx) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    auto& a = axes_[axis_idx];
    a.enabled = false;
    a.request_stop(StopAction::Disable, now_us());
    return true;
}

bool Kernel::arm_post_homing_limits(size_t axis_idx,
                                    int32_t neg_limit,
                                    int32_t pos_limit) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    auto& a = axes_[axis_idx];
    a.sw_limit_neg_counts = neg_limit;
    a.sw_limit_pos_counts = pos_limit;
    a.sw_limits_pending   = true;
    return true;
}

bool Kernel::calibrate_load(size_t axis_idx) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    auto& a = axes_[axis_idx];
    if (!a.load.configured) return false;
    if (a.load.samples_received == 0) return false;
    // Scaled load in axis counts (pre-offset). Solve for offset_counts such
    // that `sign * (scaled - offset) == commanded_position` at this instant.
    const int64_t scaled =
        (a.load.raw_position * a.load.scale_num) / a.load.scale_den;
    a.load.offset_counts = scaled -
        (static_cast<int64_t>(a.commanded_position) * a.load.sign);
    // Zero integrator + trim so the freshly-calibrated axis starts clean.
    a.load.integral  = 0;
    a.load.last_trim = 0;
    a.outer_trim     = 0;
    return true;
}

bool Kernel::set_spin_velocity(size_t axis_idx, int32_t velocity_cps) noexcept {
    if (axis_idx >= MAX_AXES) return false;
    // Safety gate — synthetic feedback is only safe on unhooked axes.
    if (axes_[axis_idx].drive != nullptr && velocity_cps != 0) return false;
    axes_[axis_idx].spin_velocity_cps = velocity_cps;
    return true;
}

bool Kernel::set_topology(const ChannelSpec* specs, size_t count) noexcept {
    if (!specs || count == 0 || count > MAX_CHANNELS) return false;

    // Validator pass — reject before touching live state. Every axis
    // index must appear in at most one channel and at most once per
    // channel; every channel must own at least one axis and every axis
    // index must be in range.
    bool claimed[MAX_AXES] = {};
    for (size_t c = 0; c < count; ++c) {
        const auto& s = specs[c];
        if (s.axis_count == 0) return false;
        if (s.axis_count > MAX_AXES) return false;
        for (uint8_t k = 0; k < s.axis_count; ++k) {
            const uint8_t ai = s.axis_indices[k];
            if (ai >= MAX_AXES) return false;
            if (claimed[ai]) return false; // duplicate ownership
            claimed[ai] = true;
        }
    }

    // Commit.
    for (size_t c = 0; c < count; ++c) {
        auto&       ch = channels_[c];
        const auto& s  = specs[c];
        // Copy name with null-termination guarantee.
        size_t n = 0;
        while (n < sizeof(ch.name) - 1 && s.name[n] != 0) {
            ch.name[n] = s.name[n]; ++n;
        }
        for (; n < sizeof(ch.name); ++n) ch.name[n] = 0;

        for (uint8_t k = 0; k < s.axis_count; ++k) ch.axis_indices[k] = s.axis_indices[k];
        ch.axis_count = s.axis_count;
        ch.state      = ChannelState::Running;
        ch.barrier_token = 0;
        ch.overrides = ChannelOverrides{};
    }
    // Retire any tail channels that were previously configured but are
    // absent from the new spec — they go Idle with zero axes so the tick
    // loop skips them.
    for (size_t c = count; c < MAX_CHANNELS; ++c) {
        channels_[c].axis_count = 0;
        channels_[c].state      = ChannelState::Idle;
        channels_[c].name[0]    = 0;
    }
    channel_count_ = count;
    return true;
}

bool Kernel::feedhold(size_t channel_idx, bool on) noexcept {
    if (channel_idx >= channel_count_) return false;
    auto& ch = channels_[channel_idx];
    if (on) {
        // Only accept FeedHold from Running — barrier/gearing/fault are
        // sync-primitive or safety state, operator can't override those.
        if (ch.state != ChannelState::Running) return false;
        ch.state = ChannelState::FeedHold;
    } else {
        if (ch.state != ChannelState::FeedHold) return false;
        ch.state = ChannelState::Running;
    }
    return true;
}

bool Kernel::set_override(size_t channel_idx, OverrideKind which,
                          uint16_t permille) noexcept {
    if (channel_idx >= channel_count_) return false;
    auto& ov = channels_[channel_idx].overrides;
    switch (which) {
        case OverrideKind::Feed:    ov.feed_permille    = permille; break;
        case OverrideKind::Rapid:   ov.rapid_permille   = permille; break;
        case OverrideKind::Spindle: ov.spindle_permille = permille; break;
        default: return false;
    }
    return true;
}

bool Kernel::sync_move(uint64_t axis_mask,
                       const int32_t targets[MAX_AXES],
                       uint16_t barrier_token,
                       int32_t  tolerance_counts,
                       uint16_t stable_cycles_required,
                       uint16_t max_wait_cycles,
                       bool     register_barrier) noexcept {
    if (axis_mask == 0 || !targets) return false;

    // Pass 1 — compute the slowest axis's nominal T_final in microseconds.
    // `t_final_us = |delta| * 1e6 / vmax_cps`. Accel ramp cancels across
    // axes since they share DEFAULT_ACCEL_CPS2, so this is a good enough
    // pace-picker for the cruise phase.
    uint64_t t_final_us = 0;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        if ((axis_mask & (1ull << i)) == 0) continue;
        auto& a = axes_[i];
        if (a.vmax_cps <= 0) continue;
        const int32_t delta = targets[i] - a.commanded_position;
        const uint32_t abs_delta = static_cast<uint32_t>(delta < 0 ? -delta : delta);
        const uint64_t t_us = (static_cast<uint64_t>(abs_delta) * 1'000'000ULL)
                             / static_cast<uint32_t>(a.vmax_cps);
        if (t_us > t_final_us) t_final_us = t_us;
    }
    if (t_final_us == 0) return false; // zero-length move — nothing to do

    // Pass 2 — commit each participating axis. Scale vmax so everyone
    // reaches the shared T_final. `restore_vmax_cps` gets saved so
    // cycle_axis can revert once the axis lands in Holding.
    uint8_t participants_mask = 0;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        if ((axis_mask & (1ull << i)) == 0) continue;
        auto& a = axes_[i];
        if (a.vmax_cps <= 0) continue;
        const int32_t delta = targets[i] - a.commanded_position;
        const uint32_t abs_delta = static_cast<uint32_t>(delta < 0 ? -delta : delta);

        const uint64_t required_vmax =
            (static_cast<uint64_t>(abs_delta) * 1'000'000ULL) / t_final_us;
        if (required_vmax > 0 && required_vmax < static_cast<uint64_t>(a.vmax_cps)) {
            if (a.restore_vmax_cps == 0) a.restore_vmax_cps = a.vmax_cps;
            a.vmax_cps = static_cast<int32_t>(required_vmax);
        }
        // Fire the move — reuses the existing move_to path so fault / brake
        // interlocks stay intact.
        move_to(i, targets[i]);

        // Track which channels own a participating axis, for the barrier.
        for (size_t c = 0; c < channel_count_; ++c) {
            const auto& ch = channels_[c];
            for (uint8_t k = 0; k < ch.axis_count; ++k) {
                if (ch.axis_indices[k] == i) {
                    participants_mask = static_cast<uint8_t>(
                        participants_mask | (1u << c));
                    break;
                }
            }
        }
    }
    if (participants_mask == 0) return false;

    if (!register_barrier) return true;

    // Allocate a token if the caller didn't supply one.
    if (barrier_token == 0) {
        barrier_token = static_cast<uint16_t>(
            0x9600u + (stats_.cycles.load(std::memory_order_relaxed) & 0xFFu));
    }
    // Every participating channel arrives at the barrier. They're all
    // about to freeze — but only *after* their moves begin, so the
    // setpoints already staged via move_to are the "end-of-move" target
    // against which the arbiter's tolerance check fires.
    bool barrier_ok = true;
    for (size_t c = 0; c < channel_count_; ++c) {
        if ((participants_mask & (1u << c)) == 0) continue;
        if (!arrive_at_barrier(c, barrier_token, participants_mask,
                               tolerance_counts,
                               stable_cycles_required,
                               max_wait_cycles)) {
            barrier_ok = false;
        }
    }
    return barrier_ok;
}

bool Kernel::engage_gear(uint8_t leader_axis,
                         uint8_t follower_axis,
                         uint8_t follower_channel,
                         int32_t k_num,
                         int32_t k_den,
                         uint16_t ramp_cycles) noexcept {
    if (k_den == 0) return false;
    if (leader_axis >= MAX_AXES || follower_axis >= MAX_AXES) return false;
    if (follower_channel >= channel_count_) return false;

    // Reject if the follower axis is already being driven by another link.
    for (const auto& g : gears_) {
        if (g.in_use && g.follower_axis == follower_axis && !g.disengaging)
            return false;
    }

    GearLink* slot = nullptr;
    for (auto& g : gears_) { if (!g.in_use) { slot = &g; break; } }
    if (!slot) return false;

    *slot = GearLink{};
    slot->in_use           = true;
    slot->leader_axis      = leader_axis;
    slot->follower_axis    = follower_axis;
    slot->follower_channel = follower_channel;
    slot->k_num            = k_num;
    slot->k_den            = k_den;
    // Snapshot bases at engagement so the geared expression is
    // zero-referenced to wherever the two axes are right now.
    slot->leader_base      = axes_[leader_axis].actual_position_feedback;
    slot->follower_base    = axes_[follower_axis].commanded_position;
    slot->ramp_total       = ramp_cycles;
    slot->ramp_done        = 0;
    slot->disengaging      = false;

    channels_[follower_channel].state = ChannelState::Gearing;
    return true;
}

bool Kernel::disengage_gear(uint8_t follower_axis, uint16_t ramp_cycles) noexcept {
    for (auto& g : gears_) {
        if (g.in_use && g.follower_axis == follower_axis && !g.disengaging) {
            g.disengaging = true;
            g.ramp_total  = ramp_cycles;
            g.ramp_done   = 0;
            return true;
        }
    }
    return false;
}

void Kernel::run_sync_arbiter(uint64_t /*t_now*/) noexcept {
    // Task 9.4 — walk each active barrier exactly once per cycle. Three
    // possible outcomes per slot:
    //   1. Not everyone arrived yet → keep waiting.
    //   2. Everyone arrived AND every participant has been within
    //      tolerance for >= stable_cycles_required cycles → atomically
    //      release all of them in THIS cycle.
    //   3. max_wait_cycles expired → fault every participant and free
    //      the slot. Partial release is forbidden (breaks threading).
    const uint64_t now_cycle = stats_.cycles.load(std::memory_order_relaxed);

    for (auto& b : barriers_) {
        if (!b.in_use) continue;

        const bool all_arrived = (b.arrivals_mask == b.participants_mask);

        if (all_arrived) {
            // Update per-channel stable-cycle counters for everyone who's
            // here. A cycle spent in-tolerance increments the counter; a
            // cycle spent out of tolerance resets it (convergence must be
            // sustained, not momentary).
            bool all_stable = true;
            for (size_t c = 0; c < channel_count_; ++c) {
                const uint8_t bit = static_cast<uint8_t>(1u << c);
                if ((b.participants_mask & bit) == 0) continue;
                if (channel_is_stable(channels_[c], axes_, b.tolerance_counts)) {
                    if (b.stable_cycles[c] < 0xFFFFu) b.stable_cycles[c]++;
                } else {
                    b.stable_cycles[c] = 0;
                    all_stable = false;
                }
                if (b.stable_cycles[c] < b.stable_cycles_required) all_stable = false;
            }

            if (all_stable) {
                // Atomic release — one pass sets every participant back to
                // Running in the same cycle before anything else runs.
                for (size_t c = 0; c < channel_count_; ++c) {
                    const uint8_t bit = static_cast<uint8_t>(1u << c);
                    if ((b.participants_mask & bit) == 0) continue;
                    channels_[c].state         = ChannelState::Running;
                    channels_[c].barrier_token = 0;
                }
                b.released = b.participants_mask;
                b.in_use   = false;   // free slot
                continue;
            }
        }

        // Timeout check (applies whether or not everyone arrived — a
        // channel that never shows up counts against the budget).
        if (now_cycle - b.created_cycle > b.max_wait_cycles) {
            for (size_t c = 0; c < channel_count_; ++c) {
                const uint8_t bit = static_cast<uint8_t>(1u << c);
                if ((b.participants_mask & bit) == 0) continue;
                channels_[c].state         = ChannelState::Fault;
                channels_[c].barrier_token = 0;
            }
            b.in_use = false;
            stats_.faults.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Task 9.5 — service gear links. For each active link, read the
    // leader's latched actual (from THIS cycle's LRW unpack), compute
    // the ramped ratio, write the follower's commanded position. The
    // follower's channel is in Gearing state so cycle_channel froze its
    // normal trajectory — the arbiter is the only writer of its
    // commanded_position while geared.
    for (auto& g : gears_) {
        if (!g.in_use) continue;
        if (g.leader_axis >= MAX_AXES || g.follower_axis >= MAX_AXES) {
            g.in_use = false; continue;
        }
        if (g.k_den == 0) { g.in_use = false; continue; }

        // Advance ramp. A step engage (ramp_total=0) jumps straight to k.
        if (g.ramp_total > 0 && g.ramp_done < g.ramp_total) {
            ++g.ramp_done;
        }

        // Effective numerator for this cycle.
        //   engaging:    k_eff = k_num * (ramp_done / ramp_total)
        //   disengaging: k_eff = k_num * ((ramp_total - ramp_done) / ramp_total)
        int64_t k_eff = g.k_num;
        if (g.ramp_total > 0) {
            const int64_t phase = g.disengaging
                ? static_cast<int64_t>(g.ramp_total - g.ramp_done)
                : static_cast<int64_t>(g.ramp_done);
            k_eff = (static_cast<int64_t>(g.k_num) * phase) / g.ramp_total;
        }

        const int32_t leader_now = axes_[g.leader_axis].actual_position_feedback;
        const int64_t delta      = static_cast<int64_t>(leader_now) - g.leader_base;
        const int64_t raw_tracked = (delta * k_eff) / g.k_den;
        constexpr int32_t CMD_MIN = -2000000000;
        constexpr int32_t CMD_MAX =  2000000000;
        int64_t tracked = raw_tracked;
        if (tracked > CMD_MAX) tracked = CMD_MAX;
        if (tracked < CMD_MIN) tracked = CMD_MIN;
        const int32_t fcmd = static_cast<int32_t>(g.follower_base + tracked);

        axes_[g.follower_axis].commanded_position = fcmd;
        // Drive::target_position is written at the tail of cycle_axis,
        // but the follower's channel was frozen this cycle — so push the
        // new commanded_position straight to the drive if it's hooked.
        // Without this the drive would see last-cycle's stale value.
        if (auto* d = axes_[g.follower_axis].drive) {
            __atomic_store_n(&d->target_position, fcmd, __ATOMIC_RELAXED);
        }
        axes_[g.follower_axis].target_pos.store(fcmd, std::memory_order_relaxed);

        // Disengage: once the ramp-down completes, release the follower
        // channel back to Running and free the slot. The follower's
        // commanded_position stays where it ended up (tracked value).
        if (g.disengaging && g.ramp_done >= g.ramp_total) {
            if (g.follower_channel < channel_count_) {
                channels_[g.follower_channel].state = ChannelState::Running;
            }
            g.in_use = false;
        }
    }

    // Gantry coupling — synchronize secondary to primary with offset correction.
    for (auto& g : gantrys_) {
        if (!g.in_use) continue;
        if (g.primary_axis >= MAX_AXES || g.secondary_axis >= MAX_AXES) {
            g.in_use = false; continue;
        }

        const int32_t primary_now = axes_[g.primary_axis].actual_position_feedback;
        const int32_t total_offset = g.offset_counts + g.offset_adjust;
        const int64_t delta = static_cast<int64_t>(primary_now) - g.primary_base;
        const int64_t tracked = (delta * g.ratio_num) / g.ratio_den;
        const int32_t gantry_cmd = static_cast<int32_t>(g.secondary_base + tracked + total_offset);

        axes_[g.secondary_axis].commanded_position = gantry_cmd;
        if (auto* d = axes_[g.secondary_axis].drive) {
            __atomic_store_n(&d->target_position, gantry_cmd, __ATOMIC_RELAXED);
        }
        axes_[g.secondary_axis].target_pos.store(gantry_cmd, std::memory_order_relaxed);
    }
}

void Kernel::run_loop() {
    auto* t = kernel::g_platform->get_timer_ops();
    uint64_t next_us = t->get_system_time_us();
    for (;;) {
        diag::rt::motion.sample(t->get_system_time_ns());

        next_us += DEFAULT_PERIOD_US;
        const uint64_t t_now = t->get_system_time_us();

        // Per-cycle pipeline (Phase 9.2 invariant):
        //   1. Advance each channel's axes in fixed order 0..N-1.
        //      Every command computed in step N reads state from step
        //      N-1's feedback; no channel may observe another channel's
        //      same-cycle state (motion/CHANNELS.md "Single cycle, all
        //      channels").
        //   2. Run the sync arbiter once — its decisions affect which
        //      channels progress next cycle; it never rewrites the
        //      commands already staged this cycle.
        //   3. Drive I/O commit is inlined at the tail of cycle_axis
        //      (writes to a.drive->target_position), so by the time
        //      wait_until_us runs every axis has published its setpoint.
        for (size_t c = 0; c < channel_count_; ++c) {
            cycle_channel(channels_[c], t_now);
        }
        run_sync_arbiter(t_now);

        stats_.cycles.fetch_add(1, std::memory_order_relaxed);

        if (t->get_system_time_us() > next_us) {
            stats_.deadline_miss.fetch_add(1, std::memory_order_relaxed);
            next_us = t->get_system_time_us();
        } else {
            wait_until_us(next_us);
        }
    }
}

void Kernel::thread_entry(void* arg) {
    (void)arg;
    if (!kernel::g_platform) for (;;) asm volatile("wfi");
    g_motion.run_loop();
}

// -----------------------------------------------------------------------------
// Name helpers
// -----------------------------------------------------------------------------

const char* Kernel::mode_name(Mode m) noexcept {
    switch (m) {
        case Mode::None:   return "none";
        case Mode::CSP:    return "csp";
        case Mode::CSV:    return "csv";
        case Mode::CST:    return "cst";
        case Mode::PP:     return "pp";
        case Mode::PV:     return "pv";
        case Mode::TQ:     return "tq";
        case Mode::Homing: return "homing";
    }
    return "?";
}

const char* Kernel::drive_state_name(DriveState s) noexcept {
    switch (s) {
        case DriveState::NotReady:             return "not-ready";
        case DriveState::SwitchOnDisabled:     return "sw-on-disabled";
        case DriveState::ReadyToSwitchOn:      return "ready-to-sw-on";
        case DriveState::SwitchedOn:           return "switched-on";
        case DriveState::OperationEnabled:     return "op-enabled";
        case DriveState::QuickStopActive:      return "quick-stop";
        case DriveState::FaultReactionActive:  return "fault-react";
        case DriveState::Fault:                return "fault";
    }
    return "?";
}

const char* Kernel::brake_phase_name(BrakePhase p) noexcept {
    switch (p) {
        case BrakePhase::Released:  return "released";
        case BrakePhase::Releasing: return "releasing";
        case BrakePhase::Engaging:  return "engaging";
        case BrakePhase::Engaged:   return "engaged";
    }
    return "?";
}

const char* Kernel::stop_kind_name(StopKind k) noexcept {
    switch (k) {
        case StopKind::PositionHold: return "hold";
        case StopKind::DynamicBrake: return "dyn-brake";
        case StopKind::Coast:        return "coast";
        case StopKind::Decel:        return "decel";
    }
    return "?";
}

const char* Kernel::trigger_source_name(TriggerSource t) noexcept {
    switch (t) {
        case TriggerSource::LimitSwitch:     return "limit";
        case TriggerSource::IndexPulse:      return "index";
        case TriggerSource::TorqueThreshold: return "torque";
        case TriggerSource::EncoderPosition: return "encpos";
        case TriggerSource::TouchProbe:      return "probe";
    }
    return "?";
}

// -----------------------------------------------------------------------------
// CLI dump
// -----------------------------------------------------------------------------

void Kernel::dump_status(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[160];

    kernel::util::k_snprintf(buf, sizeof(buf),
        "motion cycles=%llu miss=%llu faults=%llu homing(s/d)=%llu/%llu conn-lost=%llu\n",
        (unsigned long long)stats_.cycles.load(std::memory_order_relaxed),
        (unsigned long long)stats_.deadline_miss.load(std::memory_order_relaxed),
        (unsigned long long)stats_.faults.load(std::memory_order_relaxed),
        (unsigned long long)stats_.homings_started.load(std::memory_order_relaxed),
        (unsigned long long)stats_.homings_done.load(std::memory_order_relaxed),
        (unsigned long long)stats_.connection_losses.load(std::memory_order_relaxed));
    uart->puts(buf);

    size_t active = 0;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        const auto& a = axes_[i];
        const bool interesting =
            a.enabled ||
            a.mode != Mode::None ||
            a.state != DriveState::NotReady ||
            a.brake_phase != BrakePhase::Engaged ||
            a.homing.phase() != HomingEngine::Phase::Idle ||
            a.connection_lost;
        if (interesting) {
            ++active;
            const char* hm = a.homing.done() ? "hm=1" : "hm=0";
            uint32_t di = 0;
            if (a.drive) di = a.drive->digital_inputs;
            kernel::util::k_snprintf(buf, sizeof(buf),
                "  ax[%02zu] %s mode=%s fsa=%s brake=%s %s pos=%ld tgt=%ld sw=%04x cw=%04x di=%04x\n",
                i,
                a.enabled ? "en" : "--",
                mode_name(a.mode),
                drive_state_name(a.state),
                brake_phase_name(a.brake_phase),
                hm,
                (long)a.actual_pos.load(std::memory_order_relaxed),
                (long)a.target_pos.load(std::memory_order_relaxed),
                (unsigned)a.status_word.load(std::memory_order_relaxed),
                (unsigned)a.control_word.load(std::memory_order_relaxed),
                (unsigned)di);
            uart->puts(buf);

            if (a.mode == Mode::Homing &&
                a.homing.phase() != HomingEngine::Phase::Idle) {
                const auto& p = a.homing.plan();
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "         homing phase=%s trig=%s dir=%s fast=%ld creep=%ld "
                    "backoff=%ld tq_lim=%d%%\n",
                    HomingEngine::phase_name(a.homing.phase()),
                    trigger_source_name(p.trigger),
                    p.approach_positive ? "+" : "-",
                    (long)p.fast_speed_cps,
                    (long)p.creep_speed_cps,
                    (long)p.backoff_counts,
                    (int)(p.torque_threshold_permille / 10));
                uart->puts(buf);
            }
            if (a.connection_lost) uart->puts("         CONNECTION LOST\n");
            if (a.encoder.source != Axis::EncoderSource::Motor || 
                a.encoder.external_cpr > 0 || a.encoder.gear_num != 1 || a.encoder.gear_den != 1) {
                const char* src_name[] = {"motor", "external", "dual"};
                kernel::util::k_snprintf(buf, sizeof(buf),
                    "         enc=%s m_cpr=%lu x_cpr=%lu gear=%u/%u s=%d\n",
                    src_name[static_cast<uint8_t>(a.encoder.source)],
                    (unsigned long)a.encoder.motor_cpr,
                    (unsigned long)a.encoder.external_cpr,
                    (unsigned)a.encoder.gear_num,
                    (unsigned)a.encoder.gear_den,
                    a.encoder.sign);
                uart->puts(buf);
            }
        }
    }
    if (active == 0) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  all %zu axes idle (mode=none fsa=not-ready brake=engaged)\n",
            (size_t)MAX_AXES);
        uart->puts(buf);
    }
}

void Kernel::dump_channels(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[160];
    auto state_name = [](ChannelState s) -> const char* {
        switch (s) {
            case ChannelState::Idle:           return "idle";
            case ChannelState::Running:        return "running";
            case ChannelState::FeedHold:       return "feedhold";
            case ChannelState::WaitingBarrier: return "barrier";
            case ChannelState::Gearing:        return "gearing";
            case ChannelState::Fault:          return "fault";
        }
        return "?";
    };
    kernel::util::k_snprintf(buf, sizeof(buf),
        "motion channels: %zu (max %zu)\n",
        channel_count_, (size_t)MAX_CHANNELS);
    uart->puts(buf);
    for (size_t c = 0; c < channel_count_; ++c) {
        const auto& ch = channels_[c];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  ch[%zu] name=%s state=%s axes=%u feed=%u%% rapid=%u%% spindle=%u%%\n",
            c,
            ch.name,
            state_name(ch.state),
            (unsigned)ch.axis_count,
            (unsigned)(ch.overrides.feed_permille / 10),
            (unsigned)(ch.overrides.rapid_permille / 10),
            (unsigned)(ch.overrides.spindle_permille / 10));
        uart->puts(buf);
        // One-line list of owned axis indices for sanity.
        char idxbuf[128];
        size_t pos = 0;
        idxbuf[pos++] = ' '; idxbuf[pos++] = ' '; idxbuf[pos++] = ' ';
        idxbuf[pos++] = ' '; idxbuf[pos++] = '[';
        for (uint8_t k = 0; k < ch.axis_count && pos + 6 < sizeof(idxbuf); ++k) {
            if (k) { idxbuf[pos++] = ','; idxbuf[pos++] = ' '; }
            pos += kernel::util::k_snprintf(idxbuf + pos, sizeof(idxbuf) - pos,
                                             "%u", (unsigned)ch.axis_indices[k]);
        }
        if (pos + 2 < sizeof(idxbuf)) {
            idxbuf[pos++] = ']'; idxbuf[pos++] = '\n'; idxbuf[pos] = '\0';
        } else {
            idxbuf[sizeof(idxbuf)-2] = '\n'; idxbuf[sizeof(idxbuf)-1] = '\0';
        }
        uart->puts(idxbuf);
    }
}

void Kernel::dump_barriers(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[160];
    size_t shown = 0;
    for (size_t i = 0; i < MAX_BARRIERS; ++i) {
        const auto& b = barriers_[i];
        if (!b.in_use) continue;
        ++shown;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  brk[%zu] token=0x%04x parts=0x%02x arrived=0x%02x "
            "tol=%ld stable_req=%u max_wait=%u age=%lu\n",
            i,
            (unsigned)b.token,
            (unsigned)b.participants_mask,
            (unsigned)b.arrivals_mask,
            (long)b.tolerance_counts,
            (unsigned)b.stable_cycles_required,
            (unsigned)b.max_wait_cycles,
            (unsigned long)(stats_.cycles.load(std::memory_order_relaxed) - b.created_cycle));
        uart->puts(buf);
        // Per-channel stable counters — one line, readable.
        char line[96];
        size_t pos = 0;
        line[pos++] = ' '; line[pos++] = ' '; line[pos++] = ' '; line[pos++] = ' ';
        line[pos++] = 's'; line[pos++] = 't'; line[pos++] = 'a'; line[pos++] = 'b';
        line[pos++] = 'l'; line[pos++] = 'e'; line[pos++] = ':'; line[pos++] = ' ';
        for (size_t c = 0; c < channel_count_ && pos + 16 < sizeof(line); ++c) {
            if (c) { line[pos++] = ','; line[pos++] = ' '; }
            pos += kernel::util::k_snprintf(line + pos, sizeof(line) - pos,
                                             "ch%zu=%u", c, (unsigned)b.stable_cycles[c]);
        }
        if (pos + 1 < sizeof(line)) { line[pos++] = '\n'; line[pos] = 0; }
        uart->puts(line);
    }
    if (shown == 0) uart->puts("  no active barriers\n");
}

void Kernel::dump_gears(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[192];
    size_t shown = 0;
    for (size_t i = 0; i < MAX_GEARS; ++i) {
        const auto& g = gears_[i];
        if (!g.in_use) continue;
        ++shown;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  gear[%zu] leader=ax%u follower=ax%u ch%u k=%ld/%ld ramp=%u/%u %s\n"
            "      leader_base=%ld follower_base=%ld\n",
            i,
            (unsigned)g.leader_axis, (unsigned)g.follower_axis,
            (unsigned)g.follower_channel,
            (long)g.k_num, (long)g.k_den,
            (unsigned)g.ramp_done, (unsigned)g.ramp_total,
            g.disengaging ? "(disengaging)" : "",
            (long)g.leader_base, (long)g.follower_base);
        uart->puts(buf);

        // Task 9.9 — instantaneous phase error. The arbiter sets the
        // follower's commanded position each cycle from the leader sample;
        // under ideal conditions the follower's actual settles within one
        // following-error cycle of the commanded. The "expected" value
        // assumes the ramp is fully engaged; during ramp-in/out this
        // reports the instantaneous deviation which is expected to be
        // non-zero.
        if (g.leader_axis < MAX_AXES && g.follower_axis < MAX_AXES &&
            g.k_den != 0) {
            const int32_t la = axes_[g.leader_axis].actual_position_feedback;
            const int32_t fa = axes_[g.follower_axis].actual_position_feedback;
            const int64_t delta      = static_cast<int64_t>(la) - g.leader_base;
            const int64_t tracked    = (delta * g.k_num) / g.k_den;
            const int32_t fexpected  = static_cast<int32_t>(
                static_cast<int64_t>(g.follower_base) + tracked);
            const int32_t phase_err  = fa - fexpected;
            kernel::util::k_snprintf(buf, sizeof(buf),
                "      leader_actual=%ld follower_actual=%ld expected=%ld "
                "phase_err=%ld\n",
                (long)la, (long)fa, (long)fexpected, (long)phase_err);
            uart->puts(buf);
        }
    }
    if (shown == 0) uart->puts("  no active gear links\n");
}

int Kernel::engage_gantry(uint8_t primary, uint8_t secondary,
                          int32_t ratio_num, int32_t ratio_den,
                          int32_t offset) noexcept {
    if (primary >= MAX_AXES || secondary >= MAX_AXES || primary == secondary) return -1;
    if (ratio_den == 0) ratio_den = 1;
    for (auto& g : gantrys_) {
        if (!g.in_use) {
            g.in_use = true;
            g.primary_axis = primary;
            g.secondary_axis = secondary;
            g.ratio_num = ratio_num;
            g.ratio_den = ratio_den;
            g.offset_counts = offset;
            g.offset_adjust = 0;
            g.primary_base = axes_[primary].actual_position_feedback;
            g.secondary_base = axes_[secondary].actual_position_feedback;
            return static_cast<int>(&g - gantrys_.data());
        }
    }
    return -1;
}

bool Kernel::disengage_gantry(uint8_t secondary) noexcept {
    for (auto& g : gantrys_) {
        if (g.in_use && g.secondary_axis == secondary) {
            g.in_use = false;
            return true;
        }
    }
    return false;
}

bool Kernel::adjust_gantry_offset(uint8_t secondary, int32_t delta) noexcept {
    for (auto& g : gantrys_) {
        if (g.in_use && g.secondary_axis == secondary) {
            int32_t new_adj = g.offset_adjust + delta;
            if (new_adj > (int32_t)g.max_correction) new_adj = (int32_t)g.max_correction;
            if (new_adj < -(int32_t)g.max_correction) new_adj = -(int32_t)g.max_correction;
            g.offset_adjust = new_adj;
            return true;
        }
    }
    return false;
}

bool Kernel::set_gantry_max_correction(uint8_t secondary, int32_t max_corr) noexcept {
    if (max_corr < 0) max_corr = -max_corr;
    for (auto& g : gantrys_) {
        if (g.in_use && g.secondary_axis == secondary) {
            g.max_correction = max_corr;
            return true;
        }
    }
    return false;
}

void Kernel::dump_gantrys(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[192];
    size_t shown = 0;
    for (size_t i = 0; i < MAX_GANTRYS; ++i) {
        const auto& g = gantrys_[i];
        if (!g.in_use) continue;
        ++shown;
        const int32_t total_offset = g.offset_counts + g.offset_adjust;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  gantry[%zu] primary=ax%u secondary=ax%u ratio=%ld/%ld "
            "offset=%ld adj=%ld total=%ld max=%ld\n",
            i,
            (unsigned)g.primary_axis, (unsigned)g.secondary_axis,
            (long)g.ratio_num, (long)g.ratio_den,
            (long)g.offset_counts, (long)g.offset_adjust,
            (long)total_offset, (long)g.max_correction);
        uart->puts(buf);
        if (g.primary_axis < MAX_AXES && g.secondary_axis < MAX_AXES) {
            const int32_t pa = axes_[g.primary_axis].actual_position_feedback;
            const int32_t sa = axes_[g.secondary_axis].actual_position_feedback;
            const int32_t err = sa - (pa + total_offset);
            kernel::util::k_snprintf(buf, sizeof(buf),
                "      primary=%ld secondary=%ld error=%ld\n",
                (long)pa, (long)sa, (long)err);
            uart->puts(buf);
        }
    }
    if (shown == 0) uart->puts("  no active gantry links\n");
}

// ===== Linear Axis Calibration (Pitch Error Compensation) =====

int Kernel::add_cal_point(uint8_t axis, int32_t position, int32_t error) {
    if (axis >= MAX_AXES) return -1;
    auto& cal = linear_cal_[axis];
    if (cal.point_count >= MAX_CAL_POINTS) return -1;
    cal.points[cal.point_count++] = {position, error};
    return static_cast<int>(cal.point_count - 1);
}

bool Kernel::enable_linear_cal(uint8_t axis, bool enable) {
    if (axis >= MAX_AXES) return false;
    linear_cal_[axis].enabled = enable;
    return true;
}

int32_t Kernel::get_linear_correction(uint8_t axis, int32_t position) const {
    if (axis >= MAX_AXES) return 0;
    const auto& cal = linear_cal_[axis];
    if (!cal.enabled || cal.point_count < 2) return 0;

    // Find surrounding points and interpolate.
    // Points must be sorted by position (user's responsibility).
    int32_t prev_pos = cal.points[0].position_counts;
    int32_t prev_err = cal.points[0].error_counts;
    for (size_t i = 1; i < cal.point_count; ++i) {
        int32_t cur_pos = cal.points[i].position_counts;
        int32_t cur_err = cal.points[i].error_counts;
        if (position <= cur_pos) {
            if (cur_pos == prev_pos) return prev_err;
            // Linear interpolation
            int64_t frac = (static_cast<int64_t>(position - prev_pos) * 1000) / (cur_pos - prev_pos);
            return static_cast<int32_t>(prev_err + (frac * (cur_err - prev_err)) / 1000);
        }
        prev_pos = cur_pos;
        prev_err = cur_err;
    }
    // Beyond last point — return last error
    return prev_err;
}

void Kernel::dump_linear_cal(kernel::hal::UARTDriverOps* uart, uint8_t axis) const {
    if (!uart) return;
    if (axis >= MAX_AXES) { uart->puts("cal_linear: bad axis\n"); return; }
    char buf[128];
    const auto& cal = linear_cal_[axis];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "linear_cal ax%u: enabled=%d points=%u\n",
        (unsigned)axis, (int)cal.enabled, (unsigned)cal.point_count);
    uart->puts(buf);
    for (size_t i = 0; i < cal.point_count; ++i) {
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  [%zu] pos=%ld err=%ld\n",
            i, (long)cal.points[i].position_counts, (long)cal.points[i].error_counts);
        uart->puts(buf);
    }
}

void Kernel::clear_linear_cal(uint8_t axis) {
    if (axis >= MAX_AXES) return;
    linear_cal_[axis] = LinearCalibration{};
    linear_cal_[axis].axis = axis;
}

// ===== Rotary Axis Calibration =====

bool Kernel::set_rotary_index_offset(uint8_t axis, int32_t offset_counts) {
    if (axis >= MAX_AXES) return false;
    for (auto& rc : rotary_cal_) {
        if (!rc.enabled || rc.axis == 0xFF) {
            rc.enabled = true;
            rc.axis = axis;
            rc.index_offset_counts = offset_counts;
            return true;
        }
        if (rc.axis == axis) {
            rc.index_offset_counts = offset_counts;
            return true;
        }
    }
    return false;
}

bool Kernel::enable_rotary_cal(uint8_t axis, bool enable) {
    for (auto& rc : rotary_cal_) {
        if (rc.axis == axis) {
            rc.enabled = enable;
            return true;
        }
    }
    return false;
}

int32_t Kernel::get_rotary_correction(uint8_t axis) const {
    for (const auto& rc : rotary_cal_) {
        if (rc.enabled && rc.axis == axis) {
            return rc.index_offset_counts;
        }
    }
    return 0;
}

void Kernel::dump_rotary_cal(kernel::hal::UARTDriverOps* uart, uint8_t axis) const {
    if (!uart) return;
    char buf[96];
    for (const auto& rc : rotary_cal_) {
        if (rc.axis == axis || axis == 0xFF) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "rotary_cal ax%u: enabled=%d index_offset=%ld\n",
                (unsigned)rc.axis, (int)rc.enabled, (long)rc.index_offset_counts);
            uart->puts(buf);
        }
    }
}

// ===== Machine Geometry (Squareness) =====

bool Kernel::set_geometry_error(const char* pair, int32_t urad) {
    if (kernel::util::kstrcmp(pair, "xy") == 0 || kernel::util::kstrcmp(pair, "XY") == 0) {
        geometry_.xy_error_urad = urad; return true;
    }
    if (kernel::util::kstrcmp(pair, "xz") == 0 || kernel::util::kstrcmp(pair, "XZ") == 0) {
        geometry_.xz_error_urad = urad; return true;
    }
    if (kernel::util::kstrcmp(pair, "yz") == 0 || kernel::util::kstrcmp(pair, "YZ") == 0) {
        geometry_.yz_error_urad = urad; return true;
    }
    return false;
}

int32_t Kernel::get_geometry_error(const char* pair) const {
    if (kernel::util::kstrcmp(pair, "xy") == 0 || kernel::util::kstrcmp(pair, "XY") == 0) {
        return geometry_.xy_error_urad;
    }
    if (kernel::util::kstrcmp(pair, "xz") == 0 || kernel::util::kstrcmp(pair, "XZ") == 0) {
        return geometry_.xz_error_urad;
    }
    if (kernel::util::kstrcmp(pair, "yz") == 0 || kernel::util::kstrcmp(pair, "YZ") == 0) {
        return geometry_.yz_error_urad;
    }
    return 0;
}

bool Kernel::enable_geometry(bool enable) {
    geometry_.enabled = enable;
    return true;
}

void Kernel::dump_geometry(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[96];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "geometry: enabled=%d XY=%ld urad XZ=%ld urad YZ=%ld urad\n",
        (int)geometry_.enabled,
        (long)geometry_.xy_error_urad,
        (long)geometry_.xz_error_urad,
        (long)geometry_.yz_error_urad);
    uart->puts(buf);
}

// ===== Sphere-based Volumetric Calibration =====

int Kernel::sphere_add_point(int32_t cmd_x, int32_t cmd_y, int32_t cmd_z,
                           int32_t act_x, int32_t act_y, int32_t act_z) {
    if (sphere_cal_.point_count >= MAX_SPHERE_PTS) return -1;
    auto& pt = sphere_cal_.points[sphere_cal_.point_count++];
    pt.cmd_x = cmd_x; pt.cmd_y = cmd_y; pt.cmd_z = cmd_z;
    pt.act_x = act_x; pt.act_y = act_y; pt.act_z = act_z;
    pt.valid = true;
    sphere_cal_.errors_computed = false;
    return static_cast<int>(sphere_cal_.point_count - 1);
}

bool Kernel::sphere_compute_errors() {
    // Need minimum points for volumetric calibration
    // For full 21-error model, need at least 7 non-collinear points
    if (sphere_cal_.point_count < 7) return false;

    // Simplified error computation:
    // Compute mean position errors, then use difference to estimate various errors
    int64_t sum_dx = 0, sum_dy = 0, sum_dz = 0;
    for (size_t i = 0; i < sphere_cal_.point_count; ++i) {
        const auto& pt = sphere_cal_.points[i];
        if (!pt.valid) continue;
        sum_dx += (pt.act_x - pt.cmd_x);
        sum_dy += (pt.act_y - pt.cmd_y);
        sum_dz += (pt.act_z - pt.cmd_z);
    }
    int32_t n = static_cast<int32_t>(sphere_cal_.point_count);
    int32_t mean_dx = static_cast<int32_t>(sum_dx / n);
    int32_t mean_dy = static_cast<int32_t>(sum_dy / n);
    int32_t mean_dz = static_cast<int32_t>(sum_dz / n);

    // Convert to µrad assuming 1 count = 1 µm at 1mm = 1000 counts (0.1µm resolution)
    // This is simplified - real implementation would use full matrix inversion
    sphere_cal_.errors.pos_x_urad = mean_dx * 10 / n;  // scale to µrad
    sphere_cal_.errors.pos_y_urad = mean_dy * 10 / n;
    sphere_cal_.errors.pos_z_urad = mean_dz * 10 / n;

    // Compute squareness from position-dependent errors
    // (Simplified: look at how error changes across the workspace)
    int64_t dx_dy = 0, dx_dz = 0, dy_dz = 0;
    int64_t x2 = 0, y2 = 0, z2 = 0;
    for (size_t i = 0; i < sphere_cal_.point_count; ++i) {
        const auto& pt = sphere_cal_.points[i];
        if (!pt.valid) continue;
        int32_t dx = pt.act_x - pt.cmd_x - mean_dx;
        int32_t dy = pt.act_y - pt.cmd_y - mean_dy;
        int32_t dz = pt.act_z - pt.cmd_z - mean_dz;
        dx_dy += static_cast<int64_t>(pt.cmd_x) * dy;
        dx_dz += static_cast<int64_t>(pt.cmd_x) * dz;
        dy_dz += static_cast<int64_t>(pt.cmd_y) * dz;
        x2 += static_cast<int64_t>(pt.cmd_x) * pt.cmd_x;
        y2 += static_cast<int64_t>(pt.cmd_y) * pt.cmd_y;
        z2 += static_cast<int64_t>(pt.cmd_z) * pt.cmd_z;
    }

    // Squareness errors (in µrad) = error_gradient * scale
    // Using small angle approximation
    if (y2 > 0) sphere_cal_.errors.sq_xy_urad = static_cast<int32_t>((dx_dy * 1000) / y2);
    if (z2 > 0) sphere_cal_.errors.sq_xz_urad = static_cast<int32_t>((dx_dz * 1000) / z2);
    if (z2 > 0) sphere_cal_.errors.sq_yz_urad = static_cast<int32_t>((dy_dz * 1000) / z2);

    sphere_cal_.errors_computed = true;
    return true;
}

void Kernel::sphere_apply_correction(int32_t& x, int32_t& y, int32_t& z) const {
    if (!sphere_cal_.enabled || !sphere_cal_.errors_computed) return;

    const auto& e = sphere_cal_.errors;
    // Apply position error compensation (simplified)
    // Real implementation would apply full 21-error model
    if (e.sq_xy_urad != 0) {
        int64_t y_corr = (static_cast<int64_t>(x) * e.sq_xy_urad) / 1000000;
        y += static_cast<int32_t>(y_corr);
    }
    if (e.sq_xz_urad != 0) {
        int64_t z_corr = (static_cast<int64_t>(x) * e.sq_xz_urad) / 1000000;
        z += static_cast<int32_t>(z_corr);
    }
    if (e.sq_yz_urad != 0) {
        int64_t z_corr = (static_cast<int64_t>(y) * e.sq_yz_urad) / 1000000;
        z += static_cast<int32_t>(z_corr);
    }
}

void Kernel::dump_sphere_cal(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[128];
    kernel::util::k_snprintf(buf, sizeof(buf),
        "sphere_cal: enabled=%d diam=%.1fmm probe=%dµm hits=%d rapid=%dmmpm probe=%dmmpm pts=%zu computed=%d\n",
        (int)sphere_cal_.enabled, (double)sphere_cal_.sphere_diam_mm,
        (int)sphere_cal_.probe_radius_um, (int)sphere_cal_.probe_hits,
        (int)sphere_cal_.rapid_speed_mm_min, (int)sphere_cal_.probe_speed_mm_min,
        sphere_cal_.point_count, (int)sphere_cal_.errors_computed);
    uart->puts(buf);

    if (sphere_cal_.errors_computed) {
        const auto& e = sphere_cal_.errors;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  pos_err: X=%ld Y=%ld Z=%ld urad\n",
            (long)e.pos_x_urad, (long)e.pos_y_urad, (long)e.pos_z_urad);
        uart->puts(buf);
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  squareness: XY=%ld XZ=%ld YZ=%ld urad\n",
            (long)e.sq_xy_urad, (long)e.sq_xz_urad, (long)e.sq_yz_urad);
        uart->puts(buf);
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  straightness: Xy=%ld Xz=%ld Yx=%ld Yz=%ld Zx=%ld Zy=%ld urad\n",
            (long)e.str_x_y_urad, (long)e.str_x_z_urad,
            (long)e.str_y_x_urad, (long)e.str_y_z_urad,
            (long)e.str_z_x_urad, (long)e.str_z_y_urad);
        uart->puts(buf);
    }

    // Show measurement points
    for (size_t i = 0; i < sphere_cal_.point_count && i < 8; ++i) {
        const auto& pt = sphere_cal_.points[i];
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  pt[%zu]: cmd(%ld,%ld,%ld) act(%ld,%ld,%ld)\n",
            i,
            (long)pt.cmd_x, (long)pt.cmd_y, (long)pt.cmd_z,
            (long)pt.act_x, (long)pt.act_y, (long)pt.act_z);
        uart->puts(buf);
    }
}

void Kernel::apply_compensation(int32_t& x, int32_t& y, int32_t& z) const {
    // 1. Linear PEC (pitch error compensation)
    // Assumes axis 0=X, 1=Y, 2=Z - adjust based on your setup
    if (linear_cal_[0].enabled) x += get_linear_correction(0, x);
    if (linear_cal_[1].enabled) y += get_linear_correction(1, y);
    if (linear_cal_[2].enabled) z += get_linear_correction(2, z);

    // 2. Rotary index offset (for rotary axes mapped to linear)
    // This is applied separately in cycle_axis for rotary motion

    // 3. Machine geometry (squareness compensation)
    // Apply perpendicularity correction: Y' = Y + X * tan(XY_error)
    // For small angles: tan(urad) ≈ urad / 1e6
    if (geometry_.enabled) {
        if (geometry_.xy_error_urad != 0) {
            // Y correction based on X position
            int64_t y_corr = (static_cast<int64_t>(x) * geometry_.xy_error_urad) / 1000000;
            y += static_cast<int32_t>(y_corr);
        }
        if (geometry_.xz_error_urad != 0) {
            // Z correction based on X position
            int64_t z_corr = (static_cast<int64_t>(x) * geometry_.xz_error_urad) / 1000000;
            z += static_cast<int32_t>(z_corr);
        }
        if (geometry_.yz_error_urad != 0) {
            // Z correction based on Y position
            int64_t z_corr = (static_cast<int64_t>(y) * geometry_.yz_error_urad) / 1000000;
            z += static_cast<int32_t>(z_corr);
        }
    }

    // 4. Sphere-based volumetric compensation
    sphere_apply_correction(x, y, z);
}

void Kernel::dump_positions(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char buf[160];
    auto traj_name = [](TrajState s) -> const char* {
        switch (s) {
            case TrajState::Idle:        return "idle";
            case TrajState::MoveReady:   return "ready";
            case TrajState::Accel:      return "accel";
            case TrajState::ConstantVel: return "cruise";
            case TrajState::Decel:      return "decel";
            case TrajState::Holding:     return "hold";
        }
        return "?";
    };
    size_t shown = 0;
    for (size_t i = 0; i < MAX_AXES; ++i) {
        const auto& a = axes_[i];
        // Show axis if it has a drive hooked or a non-idle trajectory.
        if (!a.drive && a.traj_state == TrajState::Idle) continue;
        ++shown;
        const int32_t ferr = a.commanded_position - a.actual_position_feedback;
        kernel::util::k_snprintf(buf, sizeof(buf),
            "  ax[%02zu] traj=%s hm=%d cmd=%ld act=%ld tgt=%ld v=%ld a=%ld j=%ld ferr=%ld max_ferr=%lu cpr=%lu drive=%s%s\n",
            i,
            traj_name(a.traj_state),
            a.homing.done() ? 1 : 0,
            (long)a.commanded_position,
            (long)a.actual_position_feedback,
            (long)a.target,
            (long)a.target_velocity,
            (long)a.current_accel_cps2,
            (long)a.jerk_cps3,
            (long)ferr,
            (unsigned long)a.max_following_error,
            (unsigned long)a.counts_per_rev,
            a.drive ? "yes" : "no",
            a.fault_latched ? " FAULT" : "");
        uart->puts(buf);
        // Task 1.5 — print the decoded error code when a fault is latched
        // (or was latched and not yet cleared via fault_reset). Keeps the
        // `mpos` dump a single command that shows both motion state and
        // the drive's own "why am I faulted" answer.
        if (a.last_error_code != 0) {
            kernel::util::k_snprintf(buf, sizeof(buf),
                "         0x603F=0x%04x %s\n",
                (unsigned)a.last_error_code,
                cia402::error_text(a.last_error_code));
            uart->puts(buf);
        }
    }
    if (shown == 0) {
        uart->puts("  no axes hooked or moving\n");
    }
}

} // namespace motion
