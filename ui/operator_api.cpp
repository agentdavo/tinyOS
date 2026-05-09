// SPDX-License-Identifier: MIT OR Apache-2.0

#include "operator_api.hpp"

#include "../cnc/offsets.hpp"
#include "../cnc/interpreter.hpp"
#include "../cnc/programs.hpp"
#include "../cnc/setup.hpp"
#include "../core.hpp"
#include "../ethercat/master.hpp"
#include "../automation/macro_runtime.hpp"
#include "../automation/probe_runtime.hpp"
#include "../machine/machine_registry.hpp"
#include "../machine/machine_topology.hpp"
#include "../motion/motion.hpp"
#include "../machine/toolpods.hpp"
#include "../hmi/hmi_service.hpp"
#include "../fs/vfs.hpp"
#include "../util.hpp"

namespace kernel::ui::operator_api {

namespace {

core::Spinlock g_lock;
MachineSnapshot g_machine{};
struct ProgramRunState {
    bool active = false;
    size_t point_index = 0;
};
ProgramRunState g_program_run{};

struct SpindleState {
    int32_t requested_rpm = 0;
    bool running = false;
};
SpindleState g_spindle{};

HomingSnapshot g_homing{};
ProbeWizardSnapshot g_probe_wiz{};

// Calibration page state held in operator_api so a single-tap on either
// COMP page lands on the same axis the operator was inspecting last.
// pending_pos / pending_err buffer the two input fields on the PEC page;
// commit_point flushes them to motion::g_motion.add_cal_point().
struct CalUiState {
    uint8_t  pec_axis    = 0;
    int32_t  pending_pos = 0;
    int32_t  pending_err = 0;
};
CalUiState g_cal_ui{};

// Network setup page operator state. The COMMIT button copies pending_*
// into hmi::Service via set_static_config(); until then the operator can
// edit the inputs without disrupting live networking.
struct NetUiState {
    uint32_t pending_ip = 0;
    uint32_t pending_gateway = 0;
    uint32_t pending_ping_target = 0x08080808u; // 8.8.8.8 default
};
NetUiState g_net_ui{};

bool any_master_deadline_faulted() {
    return ethercat::g_master_a.is_deadline_faulted() ||
           ethercat::g_master_b.is_deadline_faulted();
}

bool name_has_spindle(const char* name) noexcept {
    if (!name) return false;
    for (size_t i = 0; name[i] != '\0' && i + 7 <= 24; ++i) {
        const char* p = name + i;
        const char s[] = "spindle";
        bool match = true;
        for (size_t k = 0; k < 7; ++k) {
            const char c = p[k];
            const char lc = (c >= 'A' && c <= 'Z') ? char(c + 32) : c;
            if (lc != s[k]) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

const ethercat::Master* primary_master() {
    return &ethercat::g_master_a;
}

bool has_primary_channel() {
    return motion::g_motion.channel_count() > 0;
}

const motion::Channel* primary_channel() {
    return has_primary_channel() ? &motion::g_motion.channel(0) : nullptr;
}

const cnc::programs::ProgramEntry* selected_program_entry(size_t channel = 0) {
    return cnc::programs::g_store.loaded_program(channel);
}

bool cycle_start_ready_locked() {
    for (size_t ch = 0; ch < cnc::programs::MAX_CHANNELS; ++ch) {
        const auto* program = selected_program_entry(ch);
        if (program && program->loaded) return true;
    }
    return false;
}

const char* ethercat_state_text(ethercat::State state) {
    switch (state) {
        case ethercat::State::Init: return "INIT";
        case ethercat::State::PreOp: return "PRE-OP";
        case ethercat::State::Bootstrap: return "BOOTSTRAP";
        case ethercat::State::SafeOp: return "SAFE-OP";
        case ethercat::State::Op: return "OPERATIONAL";
        case ethercat::State::Fault: return "FAULT";
        default: return "UNKNOWN";
    }
}

void select_next_axis_locked() {
    g_machine.selected_axis = (g_machine.selected_axis + 1u) & 3u;
}

bool axis_faulted_locked() {
    for (size_t i = 0; i < 4; ++i) {
        if (motion::g_motion.axis(i).fault_latched) return true;
    }
    const auto* ch = primary_channel();
    return ch && ch->state == motion::ChannelState::Fault;
}

bool axis_homing_active_locked() {
    for (size_t i = 0; i < 4; ++i) {
        const auto phase = motion::g_motion.axis(i).homing.phase();
        if (phase != motion::HomingEngine::Phase::Idle &&
            phase != motion::HomingEngine::Phase::Done) {
            return true;
        }
    }
    return false;
}

bool motion_busy_locked() {
    for (size_t i = 0; i < 4; ++i) {
        const auto& axis = motion::g_motion.axis(i);
        if (axis.traj_state != motion::TrajState::Idle &&
            axis.traj_state != motion::TrajState::Holding) {
            return true;
        }
        if (axis.mode == motion::Mode::CSV && axis.target_velocity != 0) return true;
    }
    return false;
}

[[maybe_unused]] bool motion_settled_locked() {
    return !motion_busy_locked();
}

bool interpreter_running_locked() {
    for (size_t ch = 0; ch < cnc::programs::MAX_CHANNELS; ++ch) {
        const auto snap = cnc::interp::g_runtime.snapshot(ch);
        if (snap.state == cnc::interp::State::Running ||
            snap.state == cnc::interp::State::WaitingBarrier ||
            snap.state == cnc::interp::State::WaitingMacro ||
            snap.state == cnc::interp::State::Dwell) {
            return true;
        }
    }
    return false;
}

const char* interpreter_state_text(cnc::interp::State state) {
    switch (state) {
        case cnc::interp::State::Idle: return "idle";
        case cnc::interp::State::Ready: return "ready";
        case cnc::interp::State::Running: return "running";
        case cnc::interp::State::WaitingBarrier: return "barrier";
        case cnc::interp::State::WaitingMacro: return "macro";
        case cnc::interp::State::Dwell: return "dwell";
        case cnc::interp::State::Complete: return "complete";
        case cnc::interp::State::Fault: return "fault";
    }
    return "unknown";
}

Mode current_mode_locked() {
    if (axis_faulted_locked()) return Mode::Alarm;
    if (axis_homing_active_locked()) return Mode::Homing;
    const auto* ch = primary_channel();
    if (ch && ch->state == motion::ChannelState::FeedHold) return Mode::Hold;
    if (interpreter_running_locked() || motion_busy_locked()) return Mode::Running;
    return Mode::Ready;
}

bool axis_homed_locked(size_t axis_idx) {
    const auto& axis = motion::g_motion.axis(axis_idx);
    const uint16_t status = axis.status_word.load(std::memory_order_relaxed);
    return (status & (1u << 8)) != 0 ||
           axis.homing.phase() == motion::HomingEngine::Phase::Done;
}

uint32_t current_feed_locked() {
    const auto* ch = primary_channel();
    if (!ch) return g_machine.feed > 0 ? static_cast<uint32_t>(g_machine.feed) : 100u;
    return static_cast<uint32_t>(ch->overrides.feed_permille / 10u);
}

uint32_t current_spindle_override_locked() {
    const auto* ch = primary_channel();
    if (!ch) return 100u;
    return static_cast<uint32_t>(ch->overrides.spindle_permille / 10u);
}

void stop_preview_cycle_locked(bool completed = false) {
    g_program_run.active = false;
    g_program_run.point_index = 0;
    g_machine.cycle_progress = completed ? 100u : 0u;
}

void refresh_machine_snapshot_locked() {
    const uint32_t selected_axis = g_machine.selected_axis & 3u;
    for (size_t i = 0; i < 4; ++i) {
        const auto& ax = motion::g_motion.axis(i);
        const int32_t act = ax.actual_pos.load(std::memory_order_relaxed);
        const int32_t tgt = ax.target_pos.load(std::memory_order_relaxed);
        g_machine.axis_pos[i] = act;
        g_machine.cmd_pos[i] = tgt;
        g_machine.dtg[i] = tgt - act;
        g_machine.axis_homed[i] = axis_homed_locked(i);
        // Soft-limit proximity: red DRO when within 10% of either edge.
        // Skips when the operator hasn't pushed limits (neg == pos == 0).
        const int32_t neg = ax.sw_limit_neg_counts;
        const int32_t pos = ax.sw_limit_pos_counts;
        bool near = false;
        if (pos > neg) {
            const int64_t span = static_cast<int64_t>(pos) - neg;
            const int64_t margin = span / 10;
            if (margin > 0) {
                const int64_t lo_dist = static_cast<int64_t>(act) - neg;
                const int64_t hi_dist = static_cast<int64_t>(pos) - act;
                near = (lo_dist < margin) || (hi_dist < margin);
            }
        }
        g_machine.axis_near_limit[i] = near;
    }
    g_machine.mode = current_mode_locked();
    g_machine.hold = (g_machine.mode == Mode::Hold);
    g_machine.feed = static_cast<int32_t>(current_feed_locked());
    g_machine.spindle_override = current_spindle_override_locked();
    g_machine.torque = static_cast<uint32_t>(
        motion::g_motion.axis(selected_axis).actual_torque_permille.load(std::memory_order_relaxed));
    const size_t sp_idx = static_cast<size_t>(spindle_axis_index());
    g_machine.spindle = motion::g_motion.axis(sp_idx).target_velocity;
    g_machine.spindle_rpm = motion::g_motion.axis(sp_idx).target_velocity;
    g_machine.spindle_load = motion::g_motion.axis(sp_idx).actual_torque_permille.load(std::memory_order_relaxed);

    const auto interp = cnc::interp::g_runtime.snapshot(0);
    g_machine.wcs_index = static_cast<uint32_t>(interp.active_work);
    g_machine.inch_mode = interp.inch_mode;
    g_machine.block_current = static_cast<uint32_t>(interp.block);
    g_machine.block_next = static_cast<uint32_t>(interp.block + 1);
    g_machine.runtime_ms = g_machine.tick * 100u;  // tick ~= 100ms via step_demo_tick cadence
}

} // namespace

void select_page(PageId page) {
    core::ScopedLock lock(g_lock);
    g_machine.page = page;
}

PageId current_page() {
    core::ScopedLock lock(g_lock);
    return g_machine.page;
}

// ---------------------------------------------------------------------------
// Forward-declared internal state used by the mutators below.
//
// Order matters: jog_selected / jog_axis_step / start_continuous_jog /
// toggle_cycle all reference mode_allows_*_locked() and (for toggle_cycle)
// g_restart_review.pending. Their definitions originally lived further down
// the file alongside set_operator_mode and the restart wizard, which left
// the references unresolved at compile time. Hoisting them here is the
// fix — the *_locked predicates only read g_machine.operator_mode and
// g_restart_review is a plain in-RAM struct, so neither has any
// initialisation-order dependency on the larger surface below.
// ---------------------------------------------------------------------------

struct RestartReviewState {
    bool     pending             = false;
    size_t   target_line         = 0;
    size_t   active_work         = 0;     // 0 = G54
    size_t   active_tool         = 0;
    bool     tool_length_active  = false;
    uint32_t feed                = 0;
    int32_t  spindle             = 0;
    bool     coolant_mist        = false;
    bool     coolant_flood       = false;
    bool     ok                  = false; // dry scan succeeded
};
static RestartReviewState g_restart_review{};

// Mode-gating policy:
//   Auto  → only the autonomous program may run; cycle_start enabled.
//   MDI   → only the operator's manually-typed line is dispatched.
//   Jog   → only continuous / step jog is permitted.
//   Setup → calibration, offsets, homing wizard, jog (operator can move
//           the machine while editing offsets, but cycle-start is locked).
// Caller must hold g_lock.
static bool mode_allows_jog_locked() {
    return g_machine.operator_mode == OperatorMode::Jog ||
           g_machine.operator_mode == OperatorMode::Setup;
}
static bool mode_allows_cycle_locked() {
    return g_machine.operator_mode == OperatorMode::Auto;
}
static bool mode_allows_mdi_locked() {
    return g_machine.operator_mode == OperatorMode::MDI;
}

void jog_selected(int32_t delta) {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    if (!mode_allows_jog_locked()) return;
    if (g_machine.mode == Mode::Alarm) return;
    stop_preview_cycle_locked(false);
    const uint32_t axis_idx = g_machine.selected_axis & 3u;
    const int32_t base = motion::g_motion.axis(axis_idx).actual_pos.load(std::memory_order_relaxed);
    motion::g_motion.move_to(axis_idx, base + delta);
    select_next_axis_locked();
    refresh_machine_snapshot_locked();
}

void jog_axis_step(int32_t sign) {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    if (!mode_allows_jog_locked()) return;
    if (g_machine.mode == Mode::Alarm) return;
    stop_preview_cycle_locked(false);
    const uint32_t axis_idx = g_machine.selected_axis & 3u;
    const int32_t step = (sign >= 0 ? 1 : -1) * g_machine.jog_increment;
    const int32_t base = motion::g_motion.axis(axis_idx).actual_pos.load(std::memory_order_relaxed);
    motion::g_motion.move_to(axis_idx, base + step);
    refresh_machine_snapshot_locked();
}

void set_selected_axis(uint32_t idx) {
    core::ScopedLock lock(g_lock);
    g_machine.selected_axis = idx & 3u;
}

void set_jog_increment(int32_t counts) {
    core::ScopedLock lock(g_lock);
    if (counts < 1) counts = 1;
    if (counts > 1000000) counts = 1000000;
    g_machine.jog_increment = counts;
}

void set_jog_feed_cps(int32_t cps) {
    core::ScopedLock lock(g_lock);
    if (cps < 1) cps = 1;
    if (cps > 10000000) cps = 10000000;
    g_machine.jog_feed_cps = cps;
}

void set_operator_mode(OperatorMode mode) {
    core::ScopedLock lock(g_lock);
    g_machine.operator_mode = mode;
}

bool mode_allows_mdi() {
    core::ScopedLock lock(g_lock);
    return mode_allows_mdi_locked();
}

void start_continuous_jog(uint32_t axis, int32_t sign) {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    // Mode gate: jog is only permitted in Jog or Setup mode. The operator
    // explicitly picks one of those modes before the machine will move under
    // a hold-to-move button — matches the lockout semantics of the
    // dashboard mode buttons.
    if (!mode_allows_jog_locked()) return;
    // Reject motion when any axis fault is latched, the channel is in Fault,
    // or either EtherCAT master is sitting on a deadline trip — hold-to-move
    // must never override an active safety latch.
    if (g_machine.mode == Mode::Alarm) return;
    if (ethercat::g_master_a.is_deadline_faulted() ||
        ethercat::g_master_b.is_deadline_faulted()) return;
    const uint32_t axis_idx = axis & 3u;
    if (motion::g_motion.axis(axis_idx).fault_latched) return;
    stop_preview_cycle_locked(false);
    const int32_t velocity = (sign >= 0 ? 1 : -1) * g_machine.jog_feed_cps;
    motion::g_motion.set_axis_velocity(axis_idx, velocity);
    g_machine.selected_axis = axis_idx;
    refresh_machine_snapshot_locked();
}

void stop_continuous_jog(uint32_t axis) {
    core::ScopedLock lock(g_lock);
    const uint32_t axis_idx = axis & 3u;
    motion::g_motion.set_axis_velocity(axis_idx, 0);
    refresh_machine_snapshot_locked();
}

void toggle_view_toolpath() {
    core::ScopedLock lock(g_lock);
    g_machine.view_toolpath = !g_machine.view_toolpath;
}

void toggle_view_toolpods() {
    core::ScopedLock lock(g_lock);
    g_machine.view_toolpods = !g_machine.view_toolpods;
}

void home_selected() {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    if (g_machine.mode == Mode::Alarm) return;
    stop_preview_cycle_locked(false);
    motion::HomingPlan plan{};
    plan.trigger = motion::TriggerSource::LimitSwitch;
    plan.fast_speed_cps = 10000;
    plan.creep_speed_cps = 1000;
    plan.backoff_counts = 1000;
    plan.timeout_ms = 5000;
    motion::g_motion.start_homing(g_machine.selected_axis & 3u, plan);
    refresh_machine_snapshot_locked();
}

void toggle_cycle() {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    // Mode gate: cycle-start drives the autonomous program channel; only
    // permitted in Auto mode. The dashboard mode buttons are now real
    // safety lockouts rather than indicators.
    if (!mode_allows_cycle_locked()) return;
    if (g_machine.mode == Mode::Alarm || g_machine.mode == Mode::Homing) return;
    if (g_restart_review.pending) return;  // Wizard must be confirmed first.
    if (!cycle_start_ready_locked()) return;
    bool cycle_allowed = true;
    if (!machine::g_registry.get_bool("cycle_allowed", cycle_allowed) || !cycle_allowed) return;

    const auto* ch = primary_channel();
    if (ch && ch->state == motion::ChannelState::FeedHold) {
        (void)motion::g_motion.feedhold(0, false);
        refresh_machine_snapshot_locked();
        return;
    }
    stop_preview_cycle_locked(false);
    g_machine.cycle_progress = 0;
    (void)cnc::interp::g_runtime.start_all_loaded();
    refresh_machine_snapshot_locked();
}

void toggle_hold() {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    const auto* ch = primary_channel();
    if (!ch) return;
    const bool hold = ch->state == motion::ChannelState::FeedHold;
    (void)motion::g_motion.feedhold(0, !hold);
    refresh_machine_snapshot_locked();
}

// Forward decl — defined alongside the alarm helpers further down.
static void copy_text(char* dst, size_t cap, const char* src);

// ===== Operator audit log =====
// 64-entry ring of high-impact operator actions. Captures the safety-relevant
// surface (E-stop, fault clear, alarm ack, restart confirm/cancel, mode
// changes) so post-hoc review can answer "who pressed what, when". Pure
// in-RAM today; persistence to FAT32 belongs alongside save_setup once
// the writer lands.
struct AuditEntry {
    uint64_t tick     = 0;       // g_machine.tick at the time of the event
    char     verb[24] = {};      // "ESTOP", "CLEAR_FAULT", "ACK_ALARM", ...
    char     target[24]= {};     // "ec0", "alarm:101", "tool_change", ...
};

static constexpr size_t kAuditRingSize = 64;
static AuditEntry g_audit[kAuditRingSize];
static size_t g_audit_head = 0;       // next slot to write
static size_t g_audit_count = 0;      // total entries up to ring size

static void audit_locked(const char* verb, const char* target) {
    auto& slot = g_audit[g_audit_head];
    slot.tick = g_machine.tick;
    copy_text(slot.verb, sizeof(slot.verb), verb);
    copy_text(slot.target, sizeof(slot.target), target ? target : "");
    g_audit_head = (g_audit_head + 1) % kAuditRingSize;
    if (g_audit_count < kAuditRingSize) ++g_audit_count;
}

static void audit(const char* verb, const char* target) {
    core::ScopedLock lock(g_lock);
    audit_locked(verb, target);
}

size_t audit_log_count() {
    core::ScopedLock lock(g_lock);
    return g_audit_count;
}

bool audit_log_entry(size_t idx, AuditLogEntry& out) {
    core::ScopedLock lock(g_lock);
    if (idx >= g_audit_count) return false;
    // Walk newest-first: idx 0 is the most recent.
    const size_t slot = (g_audit_head + kAuditRingSize - 1 - idx) % kAuditRingSize;
    const auto& src = g_audit[slot];
    out.tick = src.tick;
    out.verb = src.verb;
    out.target = src.target;
    return true;
}

void request_ec_estop() {
    audit("ESTOP", "ec");
    ethercat::g_master_a.trip_fault("operator E-stop (UI)");
    ethercat::g_master_b.trip_fault("operator E-stop (UI)");
}

void clear_ec_fault() {
    audit("CLEAR_FAULT", "ec");
    ethercat::g_master_a.clear_deadline_fault();
    ethercat::g_master_b.clear_deadline_fault();
    ethercat::g_master_a.clear_dc_sync_fault();
    ethercat::g_master_b.clear_dc_sync_fault();
}

int spindle_axis_index() noexcept {
    const auto& svc = machine::topology::g_service;
    const size_t n = svc.binding_count();
    for (size_t i = 0; i < n; ++i) {
        const auto* b = svc.binding(i);
        if (!b || !b->used || b->role != machine::topology::Role::Servo) continue;
        if (name_has_spindle(b->name) && b->axis_index != 0xFF) {
            return static_cast<int>(b->axis_index);
        }
    }
    return 3;
}

void spindle_set_rpm(int32_t rpm) {
    if (any_master_deadline_faulted()) return;
    core::ScopedLock lock(g_lock);
    g_spindle.requested_rpm = rpm;
    if (g_spindle.running) {
        motion::g_motion.set_axis_velocity(static_cast<size_t>(spindle_axis_index()), rpm);
    }
}

void spindle_start() {
    if (any_master_deadline_faulted()) return;
    core::ScopedLock lock(g_lock);
    if (g_spindle.requested_rpm == 0) return;
    g_spindle.running = true;
    motion::g_motion.set_axis_velocity(static_cast<size_t>(spindle_axis_index()),
                                       g_spindle.requested_rpm);
}

void spindle_stop() {
    if (any_master_deadline_faulted()) return;
    core::ScopedLock lock(g_lock);
    g_spindle.running = false;
    motion::g_motion.set_axis_velocity(static_cast<size_t>(spindle_axis_index()), 0);
}

SpindleStatus spindle_status() {
    SpindleStatus s{};
    s.axis_index = spindle_axis_index();
    s.deadline_faulted = any_master_deadline_faulted();
    {
        core::ScopedLock lock(g_lock);
        s.requested_rpm = g_spindle.requested_rpm;
        s.running = g_spindle.running;
    }
    const auto& ax = motion::g_motion.axis(static_cast<size_t>(s.axis_index));
    s.actual_rpm = ax.target_velocity;
    s.load_permille = ax.actual_torque_permille.load(std::memory_order_relaxed);
    return s;
}

void reset_alarm() {
    core::ScopedLock lock(g_lock);
    for (size_t ch = 0; ch < cnc::programs::MAX_CHANNELS; ++ch) {
        (void)cnc::interp::g_runtime.stop(ch);
        (void)macros::g_runtime.stop(ch);
    }
    for (size_t i = 0; i < 4; ++i) {
        if (motion::g_motion.axis(i).fault_latched) {
            (void)motion::g_motion.fault_reset(i);
        }
    }
    if (const auto* ch = primary_channel();
        ch && ch->state == motion::ChannelState::FeedHold) {
        (void)motion::g_motion.feedhold(0, false);
    }
    stop_preview_cycle_locked(false);
    refresh_machine_snapshot_locked();
}

bool select_work_offset(size_t idx) { return cnc::offsets::g_service.select_work(idx); }
bool select_tool_offset(size_t idx) { return cnc::offsets::g_service.select_tool(idx); }
bool set_work_offset_axis(size_t idx, size_t axis, float value) { return cnc::offsets::g_service.set_work_axis(idx, axis, value); }
bool set_tool_offset(size_t idx, float length, float radius, float wear) { return cnc::offsets::g_service.set_tool_value(idx, length, radius, wear); }
bool select_program(size_t idx) { return select_program(0, idx); }
bool select_program(size_t channel, size_t idx) {
    return cnc::programs::g_store.select(channel, idx) && cnc::programs::g_store.open_selected(channel);
}
bool select_program_by_name(const char* name) {
    return select_program_by_name(0, name);
}
bool select_program_by_name(size_t channel, const char* name) {
    size_t idx = 0;
    return cnc::programs::g_store.find_by_name(name, idx) && select_program(channel, idx);
}
const char* selected_program_name() {
    return selected_program_name(0);
}
const char* selected_program_name(size_t channel) {
    const auto* program = cnc::programs::g_store.loaded_program(channel);
    return program ? program->name : "";
}
bool write_program(const char* name, const char* text) { return cnc::programs::g_store.write_program(name, text); }

bool select_prev_program() {
    return select_prev_program(0);
}

bool select_prev_program(size_t channel) {
    const size_t count = cnc::programs::g_store.count();
    if (count == 0) return false;
    const size_t current = cnc::programs::g_store.selected(channel);
    return select_program(channel, (current + count - 1) % count);
}

bool select_next_program() {
    return select_next_program(0);
}

bool select_next_program(size_t channel) {
    const size_t count = cnc::programs::g_store.count();
    if (count == 0) return false;
    const size_t current = cnc::programs::g_store.selected(channel);
    return select_program(channel, (current + 1) % count);
}

bool request_program_simulation() {
    return request_program_simulation(0);
}

bool request_program_simulation(size_t channel) {
    const size_t current = cnc::programs::g_store.selected(channel);
    return cnc::programs::g_store.rebuild_preview(current);
}

// ===== Restart confirm wizard =====
// The interpreter's restart_at_line() does a motion-free dry scan of all
// G-code lines from the top to target_line, applying modal side effects
// only (G90/91, G20/21, plane, motion mode, F, S, WCS, active_tool,
// tool-length, coolant flags). The machine itself is NOT moved and the
// physical tool/coolant peripherals are NOT toggled — restart_at_line
// reconstructs *intent*, not state.
//
// That's a sharp foot-gun: the operator can punch a line number, hit
// resume, and crash a T1 spindle into a workpiece programmed for T3
// because the interpreter says "active_tool=T3" while the operator
// physically still has T1 mounted.
//
// This wizard adds an explicit confirm step. Flow:
//   1. Operator types target line N, taps Enter on the input.
//   2. request_restart_review(N) runs the dry scan, captures the
//      reconstructed state into g_restart_review, sets pending=true.
//   3. UI shows the captured state (tool, WCS, F, S, coolant) on the
//      restart_confirm page; cycle_start is gated on pending=false.
//   4. Operator either:
//      - taps CONFIRM → confirm_restart() clears pending. Cycle Start
//        re-enables on the dashboard.
//      - taps CANCEL → cancel_restart() stops the channel.
//
// (RestartReviewState + g_restart_review live earlier in the file —
// hoisted above jog_selected so toggle_cycle's pending check resolves.)

bool restart_review_pending() {
    core::ScopedLock lock(g_lock);
    return g_restart_review.pending;
}

RestartReviewSnapshot restart_review_snapshot() {
    core::ScopedLock lock(g_lock);
    RestartReviewSnapshot snap{};
    snap.pending = g_restart_review.pending;
    snap.ok = g_restart_review.ok;
    snap.target_line = g_restart_review.target_line;
    snap.active_work = g_restart_review.active_work;
    snap.active_tool = g_restart_review.active_tool;
    snap.tool_length_active = g_restart_review.tool_length_active;
    snap.feed = g_restart_review.feed;
    snap.spindle = g_restart_review.spindle;
    snap.coolant_mist = g_restart_review.coolant_mist;
    snap.coolant_flood = g_restart_review.coolant_flood;
    return snap;
}

bool request_restart_review(size_t line_no) {
    core::ScopedLock lock(g_lock);
    stop_preview_cycle_locked(false);
    const bool ok = cnc::interp::g_runtime.restart_at_line(0, line_no);
    g_restart_review.pending = true;
    g_restart_review.target_line = line_no;
    g_restart_review.ok = ok;
    if (ok) {
        const auto chsnap = cnc::interp::g_runtime.snapshot(0);
        g_restart_review.active_work = chsnap.active_work;
        g_restart_review.active_tool = chsnap.active_tool;
        g_restart_review.tool_length_active = chsnap.tool_length_active;
        g_restart_review.feed = chsnap.feed;
        g_restart_review.spindle = chsnap.spindle;
        g_restart_review.coolant_mist = chsnap.coolant_mist;
        g_restart_review.coolant_flood = chsnap.coolant_flood;
    }
    refresh_machine_snapshot_locked();
    return ok;
}

void confirm_restart() {
    core::ScopedLock lock(g_lock);
    if (g_restart_review.pending) {
        char tgt[24];
        kernel::util::k_snprintf(tgt, sizeof(tgt), "line=%lu",
                                 static_cast<unsigned long>(g_restart_review.target_line));
        audit_locked("RESTART_CONFIRM", tgt);
    }
    g_restart_review.pending = false;
    refresh_machine_snapshot_locked();
}

void cancel_restart() {
    core::ScopedLock lock(g_lock);
    if (g_restart_review.pending) {
        audit_locked("RESTART_CANCEL", "");
        (void)cnc::interp::g_runtime.stop(0);
    }
    g_restart_review = RestartReviewState{};
    refresh_machine_snapshot_locked();
}

// Legacy single-shot entry point used by the existing program-page input
// (commit:restart:line) and any external callers. Stages the wizard rather
// than going straight to Ready — keeping the semantics of the old API would
// re-introduce the foot-gun this wizard exists to plug.
bool restart_program_at_line(size_t line_no) {
    return request_restart_review(line_no);
}

bool select_prev_macro() {
    const size_t count = macros::g_runtime.count();
    if (count == 0) return false;
    const size_t current = macros::g_runtime.selected();
    return macros::g_runtime.select((current + count - 1) % count);
}

bool select_next_macro() {
    const size_t count = macros::g_runtime.count();
    if (count == 0) return false;
    return macros::g_runtime.select((macros::g_runtime.selected() + 1) % count);
}

bool run_selected_macro() {
    return macros::g_runtime.start(0, macros::g_runtime.selected());
}

bool run_macro_by_name(const char* id) {
    return macros::g_runtime.start_by_id(0, id);
}

bool abort_macro() {
    return macros::g_runtime.stop(0);
}

void set_feed_override(int32_t feed) {
    core::ScopedLock lock(g_lock);
    if (feed < 0) feed = 0;
    if (feed > 150) feed = 150;
    if (has_primary_channel()) {
        (void)motion::g_motion.set_override(
            0, motion::Kernel::OverrideKind::Feed,
            static_cast<uint16_t>(feed * 10));
    }
    refresh_machine_snapshot_locked();
}

// Spindle override mirrors the feed override surface — the motion kernel
// already carries spindle_permille on every channel; this just exposes the
// operator-side knob the way the operator's mental model expects (a slider
// or +/-10% buttons on the dashboard, same shape as feed). Range matches
// the feed-override convention (0..150 %) and gates on no fault.
void set_spindle_override(int32_t pct) {
    core::ScopedLock lock(g_lock);
    if (pct < 0) pct = 0;
    if (pct > 150) pct = 150;
    if (ethercat::g_master_a.is_deadline_faulted() ||
        ethercat::g_master_b.is_deadline_faulted()) return;
    if (has_primary_channel()) {
        (void)motion::g_motion.set_override(
            0, motion::Kernel::OverrideKind::Spindle,
            static_cast<uint16_t>(pct * 10));
    }
    refresh_machine_snapshot_locked();
}

void step_demo_tick() {
    core::ScopedLock lock(g_lock);
    ++g_machine.tick;
    const auto ch0 = cnc::interp::g_runtime.snapshot(0);
    const auto* program = selected_program_entry(0);
    if (program && program->preview.point_count > 1 &&
        ch0.state != cnc::interp::State::Idle && ch0.state != cnc::interp::State::Ready) {
        const size_t denom = program->preview.motion_blocks > 0 ? program->preview.motion_blocks : 1;
        const size_t block = ch0.block < denom ? ch0.block : denom;
        g_machine.cycle_progress = static_cast<uint32_t>((block * 100u) / denom);
    } else if (!interpreter_running_locked()) {
        g_machine.cycle_progress = 0;
    }
    refresh_machine_snapshot_locked();
}

MachineSnapshot machine_snapshot() {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
    return g_machine;
}

OffsetsSnapshot offsets_snapshot() {
    static constexpr const char* kAxes[4] = {"X", "Y", "Z", "A"};
    static constexpr const char* kBackendNote = "work and tool tables are bound to cnc::offsets";
    static constexpr const char* kStatus = "offset and tool data online";
    static char tool_name[16];

    OffsetsSnapshot snap{};
    const auto& work = cnc::offsets::g_service.work_offsets();
    const auto& tool = cnc::offsets::g_service.tool_offsets();
    const size_t active_work = cnc::offsets::g_service.active_work();
    const size_t active_tool = cnc::offsets::g_service.active_tool();

    snap.active_work = active_work;
    snap.active_tool = active_tool;
    snap.workset_name = work[active_work].name;
    if (const auto* station = machine::toolpods::g_service.active_station(static_cast<size_t>(0))) {
        kernel::util::k_snprintf(tool_name, sizeof(tool_name), "T%u @%s:%u",
                                 static_cast<unsigned>(station->virtual_tool),
                                 machine::toolpods::g_service.pod(static_cast<size_t>(0)) ? machine::toolpods::g_service.pod(static_cast<size_t>(0))->id : "pod",
                                 static_cast<unsigned>(station->index));
    } else {
        kernel::util::k_snprintf(tool_name, sizeof(tool_name), "T%lu", static_cast<unsigned long>(tool[active_tool].tool));
    }
    snap.tool_name = tool_name;
    snap.status = kStatus;
    snap.backend_note = kBackendNote;
    snap.row_count = snap.rows.size();
    for (size_t i = 0; i < snap.work.size(); ++i) {
        snap.work[i].name = work[i].name;
        for (size_t axis = 0; axis < 4; ++axis) snap.work[i].axis[axis] = work[i].value.axis[axis];
    }
    for (size_t i = 0; i < snap.tools.size(); ++i) {
        snap.tools[i].tool = tool[i].tool;
        snap.tools[i].length = tool[i].length;
        snap.tools[i].radius = tool[i].radius;
        snap.tools[i].wear = tool[i].wear;
    }
    for (size_t i = 0; i < snap.rows.size(); ++i) {
        snap.rows[i].axis = kAxes[i];
        snap.rows[i].work_label = work[active_work].name;
        snap.rows[i].work_offset = static_cast<int32_t>(work[active_work].value.axis[i] * 1000.0f);
        snap.rows[i].tool_label = tool_name;
        snap.rows[i].tool_offset = static_cast<int32_t>(tool[active_tool].length * 1000.0f);
        snap.rows[i].active = (i == g_machine.selected_axis);
    }
    return snap;
}

const ProgramSnapshot& program_snapshot() {
    static constexpr const char* kStatusReady = "dual-path browser online";
    static constexpr const char* kBackendNote = "two loaded channels, shared preview store, interpreter-backed runtime";
    static constexpr const char* kLoaded = "loaded / preview ready";
    static constexpr const char* kUnloaded = "catalogued / not opened";

    static ProgramSnapshot snap{};
    snap = ProgramSnapshot{};
    snap.browser_available = true;
    snap.parser_available = true;
    snap.simulation_hook_ready = true;
    snap.count = cnc::programs::g_store.count();
    snap.selected = cnc::programs::g_store.selected();
    snap.program_count = cnc::programs::g_store.count();
    snap.selected_index = cnc::programs::g_store.selected();
    snap.status = kStatusReady;
    snap.backend_note = kBackendNote;

    const size_t limit = snap.program_count < snap.programs.size() ? snap.program_count : snap.programs.size();
    const size_t loaded_ch0 = cnc::programs::g_store.loaded(0);
    for (size_t i = 0; i < limit; ++i) {
        const auto& p = cnc::programs::g_store.program(i);
        snap.names[i] = p.name;
        snap.programs[i].name = p.name;
        snap.programs[i].path = p.path;
        snap.programs[i].status = p.loaded ? kLoaded : kUnloaded;
        snap.programs[i].blocks = p.preview.motion_blocks;
        snap.programs[i].bytes = p.text_size;
        snap.programs[i].selected = (i == snap.selected_index);
        // `loaded` here means "this slot is bound to channel 0 right now",
        // which is what the operator wants highlighted in the file browser.
        // The boolean p.loaded (== was-parsed-into-memory) is folded into
        // `status` above for the textual readout.
        snap.programs[i].loaded = (i == loaded_ch0) && p.loaded && p.preview.valid;
    }

    for (size_t ch = 0; ch < snap.channels.size(); ++ch) {
        const auto& channel_state = cnc::programs::g_store.channel_state(ch);
        const auto* selected_ch = cnc::programs::g_store.selected_program(ch);
        const auto* loaded_ch = cnc::programs::g_store.loaded_program(ch);
        const auto runtime = cnc::interp::g_runtime.snapshot(ch);
        snap.channels[ch].selected_index = channel_state.selected;
        snap.channels[ch].loaded_index = channel_state.loaded;
        snap.channels[ch].selected_name = selected_ch ? selected_ch->name : "";
        snap.channels[ch].loaded_name = loaded_ch ? loaded_ch->name : "";
        snap.channels[ch].runtime = interpreter_state_text(runtime.state);
        snap.channels[ch].preview_points = loaded_ch ? loaded_ch->preview.point_count : 0;
        snap.channels[ch].blocks = runtime.block;
        snap.channels[ch].line = runtime.line;
        snap.channels[ch].barrier_token = runtime.barrier_token;
        snap.channels[ch].barrier_mask = runtime.barrier_mask;
        snap.channels[ch].barrier_cycles_remaining =
            motion::g_motion.barrier_cycles_remaining(ch);
    }

    const auto* selected = cnc::programs::g_store.selected_program();
    if (selected) {
        snap.selected_name = selected->name;
        snap.selected_size = selected->text_size;
        snap.preview_points = selected->preview.point_count;
    }
    return snap;
}

MacroSnapshot macro_snapshot() {
    MacroSnapshot snap{};
    snap.available = macros::g_runtime.count() != 0;
    snap.count = macros::g_runtime.count();
    snap.selected = macros::g_runtime.selected();
    if (const auto* selected = macros::g_runtime.macro(snap.selected)) {
        snap.selected_name = selected->title;
    }
    const auto& channel = macros::g_runtime.channel_state(0);
    snap.busy = channel.active;
    snap.fault = channel.fault;
    snap.active_step = channel.step_index;
    snap.message = channel.message;
    snap.status = channel.fault ? "fault" : (channel.active ? "running" : "idle");
    if (const auto* active = macros::g_runtime.macro(channel.macro_index);
        active && (channel.active || channel.fault)) {
        snap.active_name = active->title;
    }
    const size_t limit = snap.count < snap.rows.size() ? snap.count : snap.rows.size();
    for (size_t i = 0; i < limit; ++i) {
        const auto* macro = macros::g_runtime.macro(i);
        if (!macro) continue;
        snap.rows[i].id = macro->id;
        snap.rows[i].title = macro->title;
        snap.rows[i].mcode = macro->mcode;
        snap.rows[i].selected = i == snap.selected;
    }
    return snap;
}

size_t selected_program_preview(const render::gles1::Vec3f*& points_out) {
    return selected_program_preview(0, points_out);
}

size_t selected_program_preview(size_t channel, const render::gles1::Vec3f*& points_out) {
    const auto* p = cnc::programs::g_store.loaded_program(channel);
    if (!p || !p->preview.valid) {
        points_out = nullptr;
        return 0;
    }
    points_out = p->preview.points.data();
    return p->preview.point_count;
}

// ===== Alarm journal =====
// A small ring of persistent records. Replaces the previous derive-on-read
// model where alarms_snapshot rebuilt active+history from the live axis
// fault flags every call — that lost transient faults that auto-recovered,
// dropped 9th-and-later concurrent faults silently, ignored alarm_id on
// ack, and made clear_alarm_history a no-op.
//
// Each record is keyed by alarm id; raising an alarm with the same id
// updates raised_count + last_seen instead of allocating a fresh slot.
// Acknowledge sets the per-record ack flag, but the record stays in the
// journal so the operator can still see the history. clear_alarm_history
// drops every record that is both inactive and acknowledged.
//
// Thread safety: every mutation runs under g_lock (held by the surrounding
// callers — alarms_snapshot, acknowledge_alarm, clear_alarm_history). The
// raise path is invoked from inside alarms_snapshot so it inherits the lock.
struct AlarmRecord {
    uint32_t id            = 0;
    char     message[64]   = {};
    char     axis[6]       = {};
    AlarmsSnapshot::Severity severity = AlarmsSnapshot::Severity::Info;
    uint64_t first_seen_ns = 0;
    uint64_t last_seen_ns  = 0;
    uint32_t raised_count  = 0;
    bool     active        = false;
    bool     acknowledged  = false;
    bool     in_use        = false;
};

static constexpr size_t kAlarmRingSize = 32;
static AlarmRecord g_alarms[kAlarmRingSize];

static AlarmRecord* find_alarm_locked(uint32_t id) {
    for (auto& r : g_alarms) {
        if (r.in_use && r.id == id) return &r;
    }
    return nullptr;
}

static AlarmRecord* allocate_alarm_locked() {
    // Prefer an unused slot.
    for (auto& r : g_alarms) {
        if (!r.in_use) return &r;
    }
    // Else evict the oldest record that is both inactive and acknowledged.
    AlarmRecord* victim = nullptr;
    for (auto& r : g_alarms) {
        if (r.active || !r.acknowledged) continue;
        if (!victim || r.last_seen_ns < victim->last_seen_ns) victim = &r;
    }
    if (victim) return victim;
    // Else evict the oldest inactive (un-acknowledged) record.
    for (auto& r : g_alarms) {
        if (r.active) continue;
        if (!victim || r.last_seen_ns < victim->last_seen_ns) victim = &r;
    }
    if (victim) return victim;
    // Last resort: evict the oldest active. Should only happen if 32+
    // distinct alarms are simultaneously raised — pathological.
    for (auto& r : g_alarms) {
        if (!victim || r.last_seen_ns < victim->last_seen_ns) victim = &r;
    }
    return victim;
}

static void copy_text(char* dst, size_t cap, const char* src) {
    if (!dst || cap == 0) return;
    size_t i = 0;
    if (src) {
        for (; src[i] && i + 1 < cap; ++i) dst[i] = src[i];
    }
    dst[i] = '\0';
}

static void raise_alarm_locked(uint32_t id,
                               const char* msg, const char* axis,
                               AlarmsSnapshot::Severity sev,
                               uint64_t now_ns) {
    if (AlarmRecord* existing = find_alarm_locked(id)) {
        existing->last_seen_ns = now_ns;
        existing->raised_count++;
        existing->active = true;
        // Re-asserting clears acknowledgement so it shows as active again.
        if (existing->acknowledged) existing->acknowledged = false;
        copy_text(existing->message, sizeof(existing->message), msg);
        copy_text(existing->axis, sizeof(existing->axis), axis);
        existing->severity = sev;
        return;
    }
    AlarmRecord* slot = allocate_alarm_locked();
    if (!slot) return;
    slot->id = id;
    slot->first_seen_ns = now_ns;
    slot->last_seen_ns = now_ns;
    slot->raised_count = 1;
    slot->active = true;
    slot->acknowledged = false;
    slot->in_use = true;
    copy_text(slot->message, sizeof(slot->message), msg);
    copy_text(slot->axis, sizeof(slot->axis), axis);
    slot->severity = sev;
}

static void deassert_alarm_locked(uint32_t id) {
    if (AlarmRecord* existing = find_alarm_locked(id))
        existing->active = false;
}

// Walk current state (axis faults, macro_fault registry flag) and either
// raise/refresh or clear the corresponding journal entries. Driven from
// alarms_snapshot so the journal stays in step with live state without
// the master / motion threads needing to know about alarms.
static void refresh_alarm_journal_locked() {
    static constexpr const char* kAxes[4] = {"X", "Y", "Z", "A"};
    const uint64_t now_ns =
        static_cast<uint64_t>(g_machine.tick) * 100000000ULL;
    for (size_t i = 0; i < 4; ++i) {
        const auto& axis = motion::g_motion.axis(i);
        const uint32_t id = 100u + static_cast<uint32_t>(i);
        if (axis.fault_latched) {
            char msg[64];
            kernel::util::k_snprintf(msg, sizeof(msg),
                                     "DRIVE FAULT 0x%04x",
                                     axis.last_error_code);
            raise_alarm_locked(id, msg, kAxes[i],
                               AlarmsSnapshot::Severity::Error, now_ns);
        } else {
            deassert_alarm_locked(id);
        }
    }
    bool macro_fault = false;
    int32_t macro_fault_code = 0;
    if (machine::g_registry.get_bool("macro_fault", macro_fault) && macro_fault &&
        machine::g_registry.get_int("macro_fault_code", macro_fault_code)) {
        // Pull the failed channel's snapshot so the alarm message carries
        // step-level context (macro id, step index, reason). The macro
        // runtime sets .fault=true and leaves .step_index pointing at the
        // failing step + the reason in .message before clearing .active.
        // Without this enrichment the operator only saw "MACRO FAULT" with
        // no way to find the offending step short of reading serial logs.
        const uint32_t id = 500u + static_cast<uint32_t>(macro_fault_code);
        char msg[64] = {};
        const auto& ch = macros::g_runtime.channel_state(0);
        const auto* mac = macros::g_runtime.macro(ch.macro_index);
        if (mac && ch.message[0]) {
            kernel::util::k_snprintf(msg, sizeof(msg),
                                     "MACRO %s step %u: %s",
                                     mac->id,
                                     static_cast<unsigned>(ch.step_index),
                                     ch.message);
        } else if (mac) {
            kernel::util::k_snprintf(msg, sizeof(msg),
                                     "MACRO %s step %u: fault",
                                     mac->id,
                                     static_cast<unsigned>(ch.step_index));
        } else {
            kernel::util::k_snprintf(msg, sizeof(msg), "MACRO FAULT");
        }
        raise_alarm_locked(id, msg, "MAC",
                           AlarmsSnapshot::Severity::Error, now_ns);
    } else {
        // No fault flag → clear any active macro alarm regardless of code.
        for (auto& r : g_alarms) {
            if (r.in_use && r.active && r.id >= 500u && r.id < 1000u) {
                r.active = false;
            }
        }
    }
}

AlarmsSnapshot alarms_snapshot() {
    static constexpr const char* kStatusOnline = "operator alarm board online";
    core::ScopedLock lock(g_lock);
    refresh_alarm_journal_locked();

    AlarmsSnapshot snap{};
    snap.service_online = true;
    snap.status = kStatusOnline;

    // Active = currently asserted, un-acknowledged. Order: severity desc,
    // then last_seen desc — keeps the most recent critical entries on top.
    for (auto& r : g_alarms) {
        if (!r.in_use || !r.active || r.acknowledged) continue;
        if (snap.active_count >= snap.active.size()) break;
        auto& entry = snap.active[snap.active_count++];
        entry.id = r.id;
        entry.message = r.message;
        entry.axis = r.axis;
        entry.severity = r.severity;
        entry.timestamp_ns = r.last_seen_ns;
        entry.acknowledged = r.acknowledged;
    }
    if (snap.active_count != 0) snap.status = "ACTIVE MACHINE ALARM";

    // History = latest N records overall (active or not, ack'd or not),
    // sorted by last_seen_ns descending. Inserted via insertion sort
    // bounded by the array size — fine at 12 slots × 32 ring entries.
    for (auto& r : g_alarms) {
        if (!r.in_use) continue;
        size_t pos = snap.history_count;
        while (pos > 0 &&
               snap.history[pos - 1].timestamp_ns < r.last_seen_ns) {
            if (pos < snap.history.size()) {
                snap.history[pos] = snap.history[pos - 1];
            }
            --pos;
        }
        if (pos < snap.history.size()) {
            auto& entry = snap.history[pos];
            entry.id = r.id;
            entry.message = r.message;
            entry.axis = r.axis;
            entry.severity = r.severity;
            entry.timestamp_ns = r.last_seen_ns;
            entry.acknowledged = r.acknowledged;
            if (snap.history_count < snap.history.size()) ++snap.history_count;
        }
    }
    return snap;
}

SetupSnapshot setup_snapshot() {
    static constexpr const char* kMachineName = "tinyOS CNC Controller";
    static constexpr const char* kKinematics = "3-axis mill with optional rotary A";
    static constexpr const char* kVersion = "tinyOS operator stack";
    static constexpr const char* kBuildTime = __DATE__ " " __TIME__;
    static constexpr const char* kAxisTypes[4] = {"Linear", "Linear", "Linear", "Rotary"};
    static constexpr const char* kAxisNames[4] = {"X", "Y", "Z", "A"};

    SetupSnapshot snap{};
    snap.service_online = true;
    snap.machine_name = kMachineName;
    snap.kinematics = kKinematics;

    const auto* master = primary_master();
    if (master) {
        snap.network.master_name = "Master A";
        snap.network.master_state = ethercat_state_text(master->state());
        snap.network.slave_count = static_cast<uint32_t>(master->slave_count());
        snap.network.cycle_time_us = 250;
        snap.network.dc_enabled = true;
    }

    snap.system.kernel_version = kVersion;
    snap.system.build_time = kBuildTime;
    snap.system.uptime_s = g_machine.tick / 10;
    snap.system.mem_used_kb = 32768;
    snap.system.mem_total_kb = 131072;

    for (size_t i = 0; i < 4; ++i) {
        const auto& axis = motion::g_motion.axis(i);
        snap.axes[i].name = kAxisNames[i];
        snap.axes[i].type = kAxisTypes[i];
        snap.axes[i].position = axis.actual_pos.load(std::memory_order_relaxed);
        snap.axes[i].neg_limit = -100000;
        snap.axes[i].pos_limit = 100000;
        snap.axes[i].home_pos = 0;
        snap.axes[i].enabled = axis.enabled;
        snap.axes[i].homed = axis_homed_locked(i);
    }

    return snap;
}

void acknowledge_alarm(uint32_t alarm_id) {
    core::ScopedLock lock(g_lock);
    if (AlarmRecord* r = find_alarm_locked(alarm_id)) {
        r->acknowledged = true;
        char tgt[24];
        kernel::util::k_snprintf(tgt, sizeof(tgt), "alarm:%lu",
                                 static_cast<unsigned long>(alarm_id));
        audit_locked("ACK_ALARM", tgt);
    }
}

void clear_alarm_history() {
    core::ScopedLock lock(g_lock);
    // Drop every record that is both inactive and acknowledged. Active or
    // un-acknowledged records stay — the operator can't sweep an alarm
    // they haven't seen yet.
    for (auto& r : g_alarms) {
        if (r.in_use && !r.active && r.acknowledged) {
            r = AlarmRecord{};
        }
    }
}

// ===== File browser surface =====
// Backed by kernel::vfs (see fs/vfs.hpp). The header has carried these
// declarations since the service page was sketched, but no .cpp definitions
// existed — the Service page was effectively dead code on every binding.
// All four operations now go through the VFS, which handles the "embedded
// blob vs SD-mounted file vs in-RAM shadow" layering for us.
namespace {
struct FileWalkContext {
    FileSnapshot* snap;
};

bool file_walk_collect(const char* path, const char* /*data*/, size_t size, void* user) {
    auto* ctx = static_cast<FileWalkContext*>(user);
    if (!ctx || !ctx->snap) return false;
    if (ctx->snap->file_count >= ctx->snap->files.size()) return false;
    auto& slot = ctx->snap->files[ctx->snap->file_count++];
    slot.name = path;       // VFS owns the storage; lifetime matches g_vfs.
    slot.path = path;
    slot.size = size;
    slot.is_dir = false;    // VFS is flat — no real dirs.
    ctx->snap->total_size += size;
    return true;
}
}  // namespace

FileSnapshot file_snapshot() {
    FileSnapshot snap{};
    snap.available = kernel::vfs::entry_count() > 0;
    snap.mount_point = "vfs:/";
    snap.free_size = 0;     // VFS doesn't expose a free-space figure today.
    FileWalkContext ctx{&snap};
    kernel::vfs::walk("", &file_walk_collect, &ctx);
    return snap;
}

bool create_file(const char* path) {
    if (!path || !*path) return false;
    static const char empty = 0;
    return kernel::vfs::write_blob(path, &empty, 0, nullptr);
}

bool delete_file(const char* /*path*/) {
    // VFS has no delete primitive today (write_blob can shadow with empty
    // bytes but the entry stays). Returning false until that lands rather
    // than pretending success.
    return false;
}

bool read_file(const char* path, char* buffer, size_t bufsize) {
    if (!path || !buffer || bufsize == 0) return false;
    const char* data = nullptr;
    size_t size = 0;
    if (!kernel::vfs::lookup(path, data, size)) return false;
    const size_t take = (size + 1 <= bufsize) ? size : bufsize - 1;
    for (size_t i = 0; i < take; ++i) buffer[i] = data[i];
    buffer[take] = '\0';
    return true;
}

bool write_file(const char* path, const char* data, size_t len) {
    if (!path || (!data && len > 0)) return false;
    return kernel::vfs::write_blob(path, data, len, nullptr);
}

bool save_setup() {
    // First-cut path: setup.cfg at the FAT32 root. The persistence layer
    // handles real FS write vs in-RAM shadow internally; this entry point
    // just snapshots the live state. Lock-tap is brief — cnc::setup
    // re-acquires per-subsystem locks (offsets, motion) inside its own
    // serializers so we don't hold g_lock across long writes.
    {
        core::ScopedLock lock(g_lock);
        refresh_machine_snapshot_locked();
    }
    return cnc::setup::save_to("setup.cfg");
}

bool load_setup() {
    // Operator-triggered: pulls setup.cfg back into the live offset/cal
    // tables via the same setters the CLI verbs use. Boot-time auto-load
    // is deliberately not wired here — it would race with the TSV-driven
    // init that runs before the operator reaches the LOAD SETUP button.
    return cnc::setup::load_from("setup.cfg");
}

namespace {
const char* homing_state_text(HomingSnapshot::State s) {
    switch (s) {
        case HomingSnapshot::State::Idle: return "Idle";
        case HomingSnapshot::State::Configuring: return "Configuring";
        case HomingSnapshot::State::Searching: return "Searching";
        case HomingSnapshot::State::Approaching: return "Approaching";
        case HomingSnapshot::State::Done: return "Done";
        case HomingSnapshot::State::Faulted: return "Faulted";
    }
    return "?";
}
}  // namespace

HomingSnapshot homing_snapshot() {
    core::ScopedLock lock(g_lock);
    for (size_t i = 0; i < 4; ++i) g_homing.homed_axes[i] = axis_homed_locked(i);
    if (any_master_deadline_faulted()) {
        g_homing.state = HomingSnapshot::State::Faulted;
        g_homing.status_message = "EtherCAT deadline-fault latched";
    } else if (g_homing.state == HomingSnapshot::State::Searching ||
               g_homing.state == HomingSnapshot::State::Approaching) {
        const auto& ax = motion::g_motion.axis(static_cast<size_t>(g_homing.selected_axis & 3));
        const auto phase = ax.homing.phase();
        if (phase == motion::HomingEngine::Phase::Done) {
            g_homing.state = HomingSnapshot::State::Done;
            g_homing.status_message = "Homing attained";
        } else if (phase == motion::HomingEngine::Phase::Idle) {
            g_homing.state = HomingSnapshot::State::Idle;
            g_homing.status_message = "Idle";
        } else {
            g_homing.status_message = homing_state_text(g_homing.state);
        }
    } else {
        g_homing.status_message = homing_state_text(g_homing.state);
    }
    return g_homing;
}

void set_homing_axis(uint32_t axis) {
    core::ScopedLock lock(g_lock);
    g_homing.selected_axis = static_cast<int32_t>(axis & 3u);
    if (g_homing.state == HomingSnapshot::State::Idle ||
        g_homing.state == HomingSnapshot::State::Done) {
        g_homing.state = HomingSnapshot::State::Configuring;
    }
}

void set_homing_method(int32_t method_id, const char* name) {
    core::ScopedLock lock(g_lock);
    g_homing.method_id = method_id;
    g_homing.method_name = name ? name : "(none)";
    g_homing.state = HomingSnapshot::State::Configuring;
}

void set_homing_speeds(int32_t fast_cps, int32_t slow_cps) {
    core::ScopedLock lock(g_lock);
    if (fast_cps > 0) g_homing.fast_cps = fast_cps;
    if (slow_cps > 0) g_homing.slow_cps = slow_cps;
}

void start_homing() {
    if (any_master_deadline_faulted()) return;
    int8_t  method = 0;
    uint16_t station = 0;
    uint32_t fast = 0, slow = 0, accel = 0;
    int32_t  off = 0;
    uint16_t torque = 0;
    {
        core::ScopedLock lock(g_lock);
        if (g_homing.method_id == 0) {
            g_homing.status_message = "Pick a homing strategy first";
            return;
        }
        const size_t axis_idx = static_cast<size_t>(g_homing.selected_axis & 3);
        if (axis_idx >= ethercat::MAX_SLAVES) return;
        const auto& slave = ethercat::g_master_a.slave(axis_idx);
        if (slave.station_addr == 0) {
            g_homing.state = HomingSnapshot::State::Faulted;
            g_homing.status_message = "Slave not discovered";
            return;
        }
        station = slave.station_addr;
        method  = static_cast<int8_t>(g_homing.method_id);
        fast    = static_cast<uint32_t>(g_homing.fast_cps);
        slow    = static_cast<uint32_t>(g_homing.slow_cps);
        accel   = static_cast<uint32_t>(g_homing.accel_cps2);
        off     = g_homing.offset_counts;
        torque  = static_cast<uint16_t>(g_homing.torque_permille);
        g_homing.state = HomingSnapshot::State::Searching;
        g_homing.status_message = "Searching";
    }
    const bool ok = ethercat::g_master_a.run_homing_sequence(
        station, method, fast, slow, accel, off, torque, /*timeout_ms*/5000);
    core::ScopedLock lock(g_lock);
    if (ok) {
        g_homing.state = HomingSnapshot::State::Done;
        g_homing.status_message = "Homing attained";
    } else {
        g_homing.state = HomingSnapshot::State::Faulted;
        g_homing.status_message = "Homing failed or timed out";
    }
}

void abort_homing() {
    core::ScopedLock lock(g_lock);
    const size_t axis_idx = static_cast<size_t>(g_homing.selected_axis & 3);
    motion::g_motion.set_axis_velocity(axis_idx, 0);
    g_homing.state = HomingSnapshot::State::Idle;
    g_homing.status_message = "Aborted";
}

namespace {

struct ProbeCycleSpec {
    ProbeCycleKind kind;
    const char* name;
    const char* macro_id;
    bool uses_probe_runtime;
    int32_t total_steps;
};

constexpr ProbeCycleSpec kProbeCycles[] = {
    {ProbeCycleKind::None,       "(none)",          nullptr,                          false, 0},
    {ProbeCycleKind::Qualify,    "Qualify Stylus",  "probe_qualify",                  false, 4},
    {ProbeCycleKind::ZSurface,   "Z Surface",       "probe_z_surface",                false, 3},
    {ProbeCycleKind::EdgeX,      "X Edge",          "probe_x_edge",                   false, 3},
    {ProbeCycleKind::EdgeY,      "Y Edge",          "probe_y_edge",                   false, 3},
    {ProbeCycleKind::BoreCenter, "Bore Center XY",  "probe_bore_xy",                  false, 4},
    {ProbeCycleKind::Pocket3D,   "3D Pocket",       "probe_calibrate_3d_pocket",      false, 6},
    // Reference sphere is the only cycle with a dedicated probe::Runtime
    // backend; the macro_id is also valid (Macro 311 wraps the same call)
    // but the wizard prefers the direct entry to surface Stage-based
    // progress and message text.
    {ProbeCycleKind::Sphere,     "Reference Sphere", "probe_calibrate_ref_sphere",    true,  7},
};

const ProbeCycleSpec& probe_cycle_spec(ProbeCycleKind kind) {
    const auto idx = static_cast<size_t>(kind);
    if (idx >= sizeof(kProbeCycles) / sizeof(kProbeCycles[0])) return kProbeCycles[0];
    return kProbeCycles[idx];
}

// Re-derive Running / Inspecting / Faulted from the active backend each
// time the snapshot is read, so the wizard stays in sync without a tick
// thread of its own.
void refresh_probe_wizard_locked() {
    if (any_master_deadline_faulted()) {
        g_probe_wiz.state = ProbeWizardState::Faulted;
        g_probe_wiz.status_message = "EtherCAT deadline-fault latched";
        return;
    }
    if (g_probe_wiz.state != ProbeWizardState::Running) return;

    const auto& spec = probe_cycle_spec(g_probe_wiz.cycle);
    if (spec.uses_probe_runtime) {
        if (probe::g_runtime.fault()) {
            g_probe_wiz.state = ProbeWizardState::Faulted;
            const char* m = probe::g_runtime.message();
            g_probe_wiz.status_message = (m && *m) ? m : "Probe fault";
            return;
        }
        if (!probe::g_runtime.active()) {
            const auto& ax_x = motion::g_motion.axis(0);
            const auto& ax_y = motion::g_motion.axis(1);
            const auto& ax_z = motion::g_motion.axis(2);
            g_probe_wiz.result_x = ax_x.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_y = ax_y.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_z = ax_z.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_valid = true;
            g_probe_wiz.state = ProbeWizardState::Inspecting;
            g_probe_wiz.status_message = "Cycle complete";
            return;
        }
        const char* m = probe::g_runtime.message();
        if (m && *m) g_probe_wiz.status_message = m;
    } else if (spec.macro_id) {
        const auto& ch = macros::g_runtime.channel_state(0);
        if (ch.fault) {
            g_probe_wiz.state = ProbeWizardState::Faulted;
            g_probe_wiz.status_message = ch.message[0] ? ch.message : "Macro fault";
            return;
        }
        g_probe_wiz.step = static_cast<int32_t>(ch.step_index);
        if (!ch.active) {
            const auto& ax_x = motion::g_motion.axis(0);
            const auto& ax_y = motion::g_motion.axis(1);
            const auto& ax_z = motion::g_motion.axis(2);
            g_probe_wiz.result_x = ax_x.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_y = ax_y.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_z = ax_z.actual_pos.load(std::memory_order_relaxed);
            g_probe_wiz.result_valid = true;
            g_probe_wiz.state = ProbeWizardState::Inspecting;
            g_probe_wiz.status_message = "Cycle complete";
            return;
        }
        if (ch.message[0]) g_probe_wiz.status_message = ch.message;
    }
}

}  // namespace

ProbeWizardSnapshot probe_wizard_snapshot() {
    core::ScopedLock lock(g_lock);
    refresh_probe_wizard_locked();
    return g_probe_wiz;
}

void probe_wizard_select(ProbeCycleKind kind) {
    core::ScopedLock lock(g_lock);
    g_probe_wiz.cycle = kind;
    const auto& spec = probe_cycle_spec(kind);
    g_probe_wiz.cycle_name = spec.name;
    g_probe_wiz.total_steps = spec.total_steps;
    g_probe_wiz.step = 0;
    if (kind == ProbeCycleKind::None) {
        g_probe_wiz.state = ProbeWizardState::Selecting;
        g_probe_wiz.status_message = "Pick a cycle";
    } else {
        g_probe_wiz.state = ProbeWizardState::Confirming;
        g_probe_wiz.status_message = "Confirm setup, then START";
    }
}

void probe_wizard_start() {
    if (any_master_deadline_faulted()) {
        core::ScopedLock lock(g_lock);
        g_probe_wiz.state = ProbeWizardState::Faulted;
        g_probe_wiz.status_message = "EtherCAT deadline-fault latched";
        return;
    }
    core::ScopedLock lock(g_lock);
    if (g_probe_wiz.state != ProbeWizardState::Confirming &&
        g_probe_wiz.state != ProbeWizardState::Inspecting) {
        g_probe_wiz.status_message = "Pick a cycle first";
        return;
    }
    const auto& spec = probe_cycle_spec(g_probe_wiz.cycle);
    g_probe_wiz.result_valid = false;
    g_probe_wiz.step = 0;
    bool launched = false;
    if (spec.uses_probe_runtime) {
        launched = probe::g_runtime.start_reference_sphere();
    } else if (spec.macro_id) {
        launched = macros::g_runtime.start_by_id(0, spec.macro_id);
    }
    if (!launched) {
        g_probe_wiz.state = ProbeWizardState::Faulted;
        g_probe_wiz.status_message = (spec.kind == ProbeCycleKind::None)
            ? "cycle not implemented"
            : "Backend refused start (busy or missing)";
        return;
    }
    g_probe_wiz.state = ProbeWizardState::Running;
    g_probe_wiz.status_message = "Cycle running";
}

void probe_wizard_abort() {
    core::ScopedLock lock(g_lock);
    const auto& spec = probe_cycle_spec(g_probe_wiz.cycle);
    if (spec.macro_id) (void)macros::g_runtime.stop(0);
    // probe::Runtime has no explicit abort hook; zero the active axes so
    // motion stops. The next start_reference_sphere() resets the runtime.
    motion::g_motion.set_axis_velocity(0, 0);
    motion::g_motion.set_axis_velocity(1, 0);
    motion::g_motion.set_axis_velocity(2, 0);
    g_probe_wiz.state = ProbeWizardState::Idle;
    g_probe_wiz.status_message = "Aborted";
    g_probe_wiz.result_valid = false;
    g_probe_wiz.step = 0;
}

void probe_wizard_accept() {
    // Pull the cycle + captured counts under the lock, then drop it
    // before calling into cnc::offsets (which has its own spinlock).
    ProbeCycleKind cycle = ProbeCycleKind::None;
    int32_t rx = 0, ry = 0, rz = 0;
    bool result_valid = false;
    {
        core::ScopedLock lock(g_lock);
        if (g_probe_wiz.state != ProbeWizardState::Inspecting) return;
        cycle = g_probe_wiz.cycle;
        rx = g_probe_wiz.result_x;
        ry = g_probe_wiz.result_y;
        rz = g_probe_wiz.result_z;
        result_valid = g_probe_wiz.result_valid;
        g_probe_wiz.state = ProbeWizardState::Idle;
        g_probe_wiz.status_message = "Accepted";
    }

    // Counts -> mm using the same 1000 counts/mm convention as the DRO
    // and offsets editor. Qualify and Sphere intentionally fall through
    // without writing anything — they qualify the stylus / build the
    // volumetric model and have their own commit path.
    if (!result_valid) return;
    auto& svc = cnc::offsets::g_service;
    const size_t active = svc.active_work();
    const float xmm = static_cast<float>(rx) / 1000.0f;
    const float ymm = static_cast<float>(ry) / 1000.0f;
    const float zmm = static_cast<float>(rz) / 1000.0f;
    switch (cycle) {
        case ProbeCycleKind::ZSurface:
            svc.set_work_axis(active, 2, zmm);
            break;
        case ProbeCycleKind::EdgeX:
            svc.set_work_axis(active, 0, xmm);
            break;
        case ProbeCycleKind::EdgeY:
            svc.set_work_axis(active, 1, ymm);
            break;
        case ProbeCycleKind::BoreCenter:
            svc.set_work_axis(active, 0, xmm);
            svc.set_work_axis(active, 1, ymm);
            break;
        case ProbeCycleKind::Pocket3D:
            svc.set_work_axis(active, 0, xmm);
            svc.set_work_axis(active, 1, ymm);
            svc.set_work_axis(active, 2, zmm);
            break;
        case ProbeCycleKind::Qualify:
        case ProbeCycleKind::Sphere:
        case ProbeCycleKind::None:
        default:
            break;
    }
}

void probe_wizard_reject() {
    core::ScopedLock lock(g_lock);
    if (g_probe_wiz.state != ProbeWizardState::Inspecting) return;
    g_probe_wiz.result_valid = false;
    g_probe_wiz.state = ProbeWizardState::Idle;
    g_probe_wiz.status_message = "Rejected";
}

EthercatSnapshot ethercat_snapshot() {
    EthercatSnapshot snap{};
    const ethercat::Master* master = primary_master();
    if (!master) return snap;

    // Seqlock retry. Master bumps snapshot_epoch_ at the top of every cycle
    // (master.cpp:1178). If the epoch moves between e1 and e2 the multi-field
    // copy below is potentially torn — try again. At 250 µs cycle and ~20
    // atomic loads per pass the typical case is single-shot; the cap stops a
    // pathological loop on a runaway master.
    constexpr int kMaxRetries = 4;
    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        const uint32_t e1 = master->snapshot_epoch();

        snap.available = true;
        snap.master_state = static_cast<uint8_t>(master->state());
        snap.discovered = master->stats().discovered.load(std::memory_order_relaxed);
        snap.cycles = master->stats().cycles.load(std::memory_order_relaxed);
        snap.tx_frames = master->stats().tx_frames.load(std::memory_order_relaxed);
        snap.rx_frames = master->stats().rx_frames.load(std::memory_order_relaxed);
        snap.deadline_miss = master->stats().cycle_deadline_miss.load(std::memory_order_relaxed);
        snap.esm_timeouts = master->stats().esm_timeouts.load(std::memory_order_relaxed);
        snap.deadline_trips = master->stats().deadline_trips.load(std::memory_order_relaxed);
        snap.deadline_fault = master->is_deadline_faulted();
        snap.last_dc_drift_ns = master->last_dc_drift_ns();
        snap.dc_drift_max_ns  = master->stats().dc_drift_max_ns.load(std::memory_order_relaxed);
        snap.dc_sync_samples  = master->stats().dc_sync_samples.load(std::memory_order_relaxed);
        snap.dc_sync_trips    = master->stats().dc_sync_trips.load(std::memory_order_relaxed);
        snap.dc_sync_faulted  = master->is_dc_sync_faulted();
        snap.cycle_p99_us = master->hist_cycle().percentile(99);
        snap.cycle_max_us = master->hist_cycle().percentile(100);
        snap.period_us = master->period_us();
        snap.slave_count = master->slave_count();

        const size_t limit = snap.slave_count < 6 ? snap.slave_count : 6;
        for (size_t i = 0; i < limit; ++i) {
            const auto& slave = master->slave(i);
            snap.slaves[i].present = slave.present;
            snap.slaves[i].station_addr = slave.station_addr;
            snap.slaves[i].current_state = slave.current_state;
            snap.slaves[i].target_state = slave.target_state;
            snap.slaves[i].identity_mismatch = slave.identity_mismatch;
            snap.slaves[i].vendor_id = slave.vendor_id ? slave.vendor_id : slave.observed_vid;
            snap.slaves[i].product_code = slave.product_code ? slave.product_code : slave.observed_pid;
            snap.slaves[i].presence_loss_events = slave.presence_loss_events;
        }

        const uint32_t e2 = master->snapshot_epoch();
        if (e1 == e2) return snap;
        // else: master ticked mid-copy; loop and resample.
    }
    // Best-effort: return whatever the last attempt produced. The caller
    // sees a (possibly torn) snapshot but never blocks here.
    return snap;
}

// ===== Calibration / compensation surface =====

CalibrationSnapshot calibration_snapshot() {
    CalibrationSnapshot snap{};
    uint8_t axis = 0;
    {
        core::ScopedLock lock(g_lock);
        axis = g_cal_ui.pec_axis & 0x03;
    }
    snap.pec_axis = axis;
    snap.pec_enabled = motion::g_motion.linear_cal_enabled(axis);
    const uint16_t total = motion::g_motion.linear_cal_point_count(axis);
    snap.pec_point_count = total;
    const uint16_t shown = total > CalibrationSnapshot::kMaxPecRows
        ? static_cast<uint16_t>(CalibrationSnapshot::kMaxPecRows) : total;
    for (uint16_t i = 0; i < shown; ++i) {
        const auto p = motion::g_motion.linear_cal_point(axis, i);
        snap.pec_pos[i] = p.position_counts;
        snap.pec_err[i] = p.error_counts;
    }
    snap.rotary_offset_a = motion::g_motion.get_rotary_correction(3);

    snap.geom_enabled = motion::g_motion.geometry_enabled();
    snap.geom_xy_urad = motion::g_motion.get_geometry_error("XY");
    snap.geom_xz_urad = motion::g_motion.get_geometry_error("XZ");
    snap.geom_yz_urad = motion::g_motion.get_geometry_error("YZ");

    snap.sphere_enabled         = motion::g_motion.sphere_enabled();
    snap.sphere_errors_computed = motion::g_motion.sphere_errors_computed();
    snap.sphere_diameter_mm     = motion::g_motion.sphere_diameter();
    snap.sphere_probe_radius_um = motion::g_motion.sphere_probe_radius();
    snap.sphere_point_count     = motion::g_motion.sphere_point_count();
    snap.sphere_err_pos_x_urad  = motion::g_motion.sphere_error_pos_x();
    snap.sphere_err_pos_y_urad  = motion::g_motion.sphere_error_pos_y();
    snap.sphere_err_pos_z_urad  = motion::g_motion.sphere_error_pos_z();
    snap.sphere_err_sq_xy_urad  = motion::g_motion.sphere_error_sq_xy();
    snap.sphere_err_sq_xz_urad  = motion::g_motion.sphere_error_sq_xz();
    snap.sphere_err_sq_yz_urad  = motion::g_motion.sphere_error_sq_yz();
    return snap;
}

void cal_pec_select_axis(uint32_t axis) {
    core::ScopedLock lock(g_lock);
    g_cal_ui.pec_axis = static_cast<uint8_t>(axis & 0x03);
    g_cal_ui.pending_pos = 0;
    g_cal_ui.pending_err = 0;
}

void cal_pec_set_pending_pos(int32_t counts) {
    core::ScopedLock lock(g_lock);
    g_cal_ui.pending_pos = counts;
}

void cal_pec_set_pending_err(int32_t counts) {
    core::ScopedLock lock(g_lock);
    g_cal_ui.pending_err = counts;
}

int32_t cal_pec_pending_pos() {
    core::ScopedLock lock(g_lock);
    return g_cal_ui.pending_pos;
}

int32_t cal_pec_pending_err() {
    core::ScopedLock lock(g_lock);
    return g_cal_ui.pending_err;
}

bool cal_pec_commit_point() {
    if (any_master_deadline_faulted()) return false;
    uint8_t axis = 0;
    int32_t pos = 0, err = 0;
    {
        core::ScopedLock lock(g_lock);
        axis = g_cal_ui.pec_axis & 0x03;
        pos  = g_cal_ui.pending_pos;
        err  = g_cal_ui.pending_err;
    }
    return motion::g_motion.add_cal_point(axis, pos, err) >= 0;
}

void cal_pec_clear() {
    if (any_master_deadline_faulted()) return;
    uint8_t axis = 0;
    {
        core::ScopedLock lock(g_lock);
        axis = g_cal_ui.pec_axis & 0x03;
    }
    motion::g_motion.clear_linear_cal(axis);
}

void cal_pec_set_enabled(bool en) {
    if (any_master_deadline_faulted()) return;
    uint8_t axis = 0;
    {
        core::ScopedLock lock(g_lock);
        axis = g_cal_ui.pec_axis & 0x03;
    }
    (void)motion::g_motion.enable_linear_cal(axis, en);
}

void cal_geom_set(const char* pair, int32_t urad) {
    if (any_master_deadline_faulted()) return;
    if (!pair || !*pair) return;
    (void)motion::g_motion.set_geometry_error(pair, urad);
}

void cal_geom_set_enabled(bool en) {
    if (any_master_deadline_faulted()) return;
    (void)motion::g_motion.enable_geometry(en);
}

void cal_sphere_compute() {
    if (any_master_deadline_faulted()) return;
    (void)motion::g_motion.sphere_compute_errors();
}

void cal_sphere_set_enabled(bool en) {
    if (any_master_deadline_faulted()) return;
    motion::g_motion.sphere_enable(en);
}

void cal_sphere_clear() {
    if (any_master_deadline_faulted()) return;
    motion::g_motion.sphere_clear_points();
}

// ===== Network setup =====
//
// The HMI service owns DHCP, link, and ping; this page is a thin
// projection onto its public state. Commit/ping/dhcp toggles all flow
// into hmi::g_service so the page stays a pure operator surface.

NetworkSnapshot network_snapshot() {
    NetworkSnapshot snap{};
    auto& svc = hmi::g_service;
    snap.service_online = true;
    snap.nic_idx = svc.nic_idx();
    // QEMU virtio-net is always slot 0; render a stable label so the
    // operator has a name even before the NIC enumeration prints anything.
    kernel::util::k_snprintf(snap.nic_name, sizeof(snap.nic_name),
                             "nic%u", static_cast<unsigned>(snap.nic_idx));
    const uint8_t* mac = svc.mac();
    for (int i = 0; i < 6; ++i) snap.mac[i] = mac[i];
    snap.local_ip = svc.local_ip();
    snap.netmask = svc.netmask();
    snap.gateway = svc.gateway();
    snap.dhcp_enabled = svc.dhcp_enabled();
    snap.last_ping_target = svc.last_ping_target();
    snap.last_ping_rtt_ms = svc.last_ping_rtt_ms();
    snap.last_ping_result =
        static_cast<NetworkSnapshot::PingResultKind>(static_cast<uint8_t>(svc.last_ping_result()) + 1);
    if (snap.last_ping_target == 0) snap.last_ping_result = NetworkSnapshot::PingResultKind::None;
    snap.rx_requests = svc.rx_requests();
    snap.tx_responses = svc.tx_responses();

    // Link state: virtio-net never reports link-down post-init in QEMU,
    // so we infer Up once any tx has flowed; Probing while DHCP is mid-
    // discovery; Down before the first DHCP attempt or first packet.
    if (svc.tx_responses() > 0 || snap.local_ip != 0) {
        snap.link_state = NetworkSnapshot::LinkState::Up;
    } else if (svc.dhcp_in_progress()) {
        snap.link_state = NetworkSnapshot::LinkState::Probing;
    } else {
        snap.link_state = NetworkSnapshot::LinkState::Down;
    }

    if (!snap.dhcp_enabled) {
        snap.dhcp_state = NetworkSnapshot::DhcpState::Static;
    } else if (svc.dhcp_bound()) {
        snap.dhcp_state = NetworkSnapshot::DhcpState::Bound;
    } else if (svc.dhcp_in_progress()) {
        snap.dhcp_state = NetworkSnapshot::DhcpState::Discovering;
    } else if (snap.tx_responses > 0) {
        snap.dhcp_state = NetworkSnapshot::DhcpState::Timeout;
    } else {
        snap.dhcp_state = NetworkSnapshot::DhcpState::Idle;
    }

    {
        core::ScopedLock lock(g_lock);
        snap.pending_ip = g_net_ui.pending_ip ? g_net_ui.pending_ip : snap.local_ip;
        snap.pending_gateway = g_net_ui.pending_gateway ? g_net_ui.pending_gateway : snap.gateway;
        snap.pending_ping_target = g_net_ui.pending_ping_target;
        snap.uptime_s = g_machine.tick / 10u;
    }
    return snap;
}

void net_set_dhcp(bool enabled) {
    if (any_master_deadline_faulted()) return;
    hmi::g_service.set_dhcp_enabled(enabled);
}

void net_commit_static() {
    if (any_master_deadline_faulted()) return;
    uint32_t ip = 0, gw = 0;
    {
        core::ScopedLock lock(g_lock);
        ip = g_net_ui.pending_ip;
        gw = g_net_ui.pending_gateway;
    }
    if (ip == 0) return;
    hmi::g_service.set_dhcp_enabled(false);
    hmi::g_service.set_static_config(ip, 0, gw);
}

void net_set_pending_ip(uint32_t ip) {
    core::ScopedLock lock(g_lock);
    g_net_ui.pending_ip = ip;
}

void net_set_pending_gateway(uint32_t gw) {
    core::ScopedLock lock(g_lock);
    g_net_ui.pending_gateway = gw;
}

void net_set_pending_ping_target(uint32_t ip) {
    core::ScopedLock lock(g_lock);
    g_net_ui.pending_ping_target = ip;
}

void net_request_ping() {
    if (any_master_deadline_faulted()) return;
    uint32_t target = 0;
    {
        core::ScopedLock lock(g_lock);
        target = g_net_ui.pending_ping_target;
    }
    if (target == 0) return;
    // Fire-and-forget: hmi::Service runs the exchange on its own worker
    // thread; the result lands in last_ping_result() / last_ping_rtt_ms()
    // and is picked up by the network snapshot. Previously this blocked
    // the UI thread for the full 2 s ICMP timeout — every operator ping
    // froze the dashboard.
    (void)hmi::g_service.ping_ipv4_async(target, 2000u);
}

// ===== Per-axis status sub-page =====
//
// Reads g_machine.selected_axis (set by the dashboard / jog page's axis
// picker) and projects motion::Axis fields. enable / disable / fault_reset
// route through motion::Kernel and check master deadline-fault.

AxisStatusSnapshot axis_status_snapshot() {
    AxisStatusSnapshot snap{};
    uint32_t sel = 0;
    {
        core::ScopedLock lock(g_lock);
        sel = g_machine.selected_axis & 3u;
    }
    snap.selected_axis = sel;
    const auto& ax = motion::g_motion.axis(sel);
    snap.cmd_pos = ax.target_pos.load(std::memory_order_relaxed);
    snap.actual_pos = ax.actual_pos.load(std::memory_order_relaxed);
    snap.following_error = snap.cmd_pos - snap.actual_pos;
    snap.max_following_error = ax.max_following_error;
    snap.vmax_cps = ax.vmax_cps;
    snap.accel_cps2 = ax.accel_cps2;
    snap.jerk_cps3 = ax.jerk_cps3;
    snap.drive_state = static_cast<uint8_t>(ax.state);
    snap.traj_state = static_cast<uint8_t>(ax.traj_state);
    snap.mode = static_cast<uint8_t>(ax.mode);
    snap.enabled = ax.enabled;
    snap.fault_latched = ax.fault_latched;
    snap.last_error_code = ax.last_error_code;
    snap.status_word = ax.status_word.load(std::memory_order_relaxed);
    snap.control_word = ax.control_word.load(std::memory_order_relaxed);
    {
        core::ScopedLock lock(g_lock);
        snap.homed = axis_homed_locked(sel);
    }
    return snap;
}

void axis_detail_enable() {
    if (any_master_deadline_faulted()) return;
    uint32_t sel = 0;
    {
        core::ScopedLock lock(g_lock);
        sel = g_machine.selected_axis & 3u;
    }
    (void)motion::g_motion.enable_axis(sel);
}

void axis_detail_disable() {
    if (any_master_deadline_faulted()) return;
    uint32_t sel = 0;
    {
        core::ScopedLock lock(g_lock);
        sel = g_machine.selected_axis & 3u;
    }
    (void)motion::g_motion.disable_axis(sel);
}

void axis_detail_fault_reset() {
    if (any_master_deadline_faulted()) return;
    uint32_t sel = 0;
    {
        core::ScopedLock lock(g_lock);
        sel = g_machine.selected_axis & 3u;
    }
    (void)motion::g_motion.fault_reset(sel);
}

// ===== Tool change wizard =====
//
// Wraps machine::toolpods::Service's confirmation flow under the master
// deadline-fault gate. set_pending_target buffers the operator-typed tool
// number so START can pick it up; ABORT clears, ACCEPT commits via the
// service's accept_tool_change which calls select_station() inline.

namespace {
struct ToolChangeUiState {
    uint32_t pending_target = 0;
};
ToolChangeUiState g_tc_ui{};
}  // namespace

ToolChangeSnapshot tool_change_snapshot() {
    ToolChangeSnapshot snap{};
    const auto status = machine::toolpods::g_service.tool_change_state();
    snap.state = static_cast<ToolChangeSnapshot::State>(static_cast<uint8_t>(status.state));
    snap.step = status.step;
    snap.total_steps = status.total_steps;
    snap.current_tool = status.current_tool;
    {
        core::ScopedLock lock(g_lock);
        snap.target_tool = status.target_tool ? status.target_tool : g_tc_ui.pending_target;
    }
    snap.target_resolved = status.target_resolved;
    kernel::util::k_snprintf(snap.status_message, sizeof(snap.status_message), "%s", status.message);

    // Resolve current tool → its hosting pod for the CURRENT panel. Walk
    // every pod's active station; first match wins. machine::toolpods::Service
    // does not currently expose a "pod hosting active tool" accessor.
    for (size_t i = 0; i < machine::toolpods::g_service.pod_count(); ++i) {
        const auto* pod = machine::toolpods::g_service.pod(i);
        if (!pod) continue;
        const auto* station = machine::toolpods::g_service.active_station(i);
        if (!station || station->virtual_tool == 0) continue;
        if (station->virtual_tool == snap.current_tool) {
            kernel::util::k_snprintf(snap.current_pod, sizeof(snap.current_pod), "%s", pod->id);
            snap.current_station = station->index;
            kernel::util::k_snprintf(snap.current_label, sizeof(snap.current_label), "%s", station->name);
            break;
        }
    }

    // Resolve target → pod via the same registry walk. Done here (not via
    // status.target_pod_idx) so the UI can render the result even before
    // request_tool_change is fired — handy as the operator types the
    // target number in.
    if (snap.target_tool != 0) {
        size_t pod_idx = 0, station_idx = 0;
        if (machine::toolpods::g_service.lookup_virtual_tool(
                static_cast<uint16_t>(snap.target_tool), pod_idx, station_idx)) {
            const auto* pod = machine::toolpods::g_service.pod(pod_idx);
            if (pod && station_idx < pod->station_count) {
                kernel::util::k_snprintf(snap.target_pod, sizeof(snap.target_pod), "%s", pod->id);
                snap.target_station = pod->stations[station_idx].index;
                kernel::util::k_snprintf(snap.target_label, sizeof(snap.target_label),
                                         "%s", pod->stations[station_idx].name);
                snap.target_resolved = true;
            }
        } else {
            kernel::util::k_snprintf(snap.target_pod, sizeof(snap.target_pod), "%s", "unassigned");
            snap.target_station = 0;
            snap.target_label[0] = '\0';
        }
    }
    return snap;
}

void tool_change_set_pending_target(uint32_t tool) {
    core::ScopedLock lock(g_lock);
    g_tc_ui.pending_target = tool;
}

void tool_change_start() {
    if (any_master_deadline_faulted()) return;
    uint32_t target = 0;
    {
        core::ScopedLock lock(g_lock);
        target = g_tc_ui.pending_target;
    }
    if (target == 0) return;
    (void)machine::toolpods::g_service.request_tool_change(target);
}

void tool_change_abort() {
    (void)machine::toolpods::g_service.abort_tool_change();
}

void tool_change_accept() {
    if (any_master_deadline_faulted()) return;
    (void)machine::toolpods::g_service.accept_tool_change();
}

} // namespace kernel::ui::operator_api
