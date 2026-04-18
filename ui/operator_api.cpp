// SPDX-License-Identifier: MIT OR Apache-2.0

#include "operator_api.hpp"

#include "../cnc/offsets.hpp"
#include "../cnc/interpreter.hpp"
#include "../cnc/programs.hpp"
#include "../core.hpp"
#include "../ethercat/master.hpp"
#include "../automation/macro_runtime.hpp"
#include "../machine/machine_registry.hpp"
#include "../motion/motion.hpp"
#include "../machine/toolpods.hpp"
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

bool motion_settled_locked() {
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
    }
    g_machine.mode = current_mode_locked();
    g_machine.hold = (g_machine.mode == Mode::Hold);
    g_machine.feed = static_cast<int32_t>(current_feed_locked());
    g_machine.torque = static_cast<uint32_t>(
        motion::g_motion.axis(selected_axis).actual_torque_permille.load(std::memory_order_relaxed));
    g_machine.spindle = motion::g_motion.axis(3).target_velocity;
    g_machine.spindle_rpm = motion::g_motion.axis(3).target_velocity;
    g_machine.spindle_load = motion::g_motion.axis(3).actual_torque_permille.load(std::memory_order_relaxed);

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

void jog_selected(int32_t delta) {
    core::ScopedLock lock(g_lock);
    refresh_machine_snapshot_locked();
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
    if (g_machine.mode == Mode::Alarm || g_machine.mode == Mode::Homing) return;
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

bool restart_program_at_line(size_t line_no) {
    core::ScopedLock lock(g_lock);
    stop_preview_cycle_locked(false);
    const bool ok = cnc::interp::g_runtime.restart_at_line(0, line_no);
    refresh_machine_snapshot_locked();
    return ok;
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
    for (size_t i = 0; i < limit; ++i) {
        const auto& p = cnc::programs::g_store.program(i);
        snap.names[i] = p.name;
        snap.programs[i].name = p.name;
        snap.programs[i].path = p.path;
        snap.programs[i].status = p.loaded ? kLoaded : kUnloaded;
        snap.programs[i].blocks = p.preview.motion_blocks;
        snap.programs[i].bytes = p.text_size;
        snap.programs[i].selected = (i == snap.selected_index);
        snap.programs[i].loaded = p.loaded && p.preview.valid;
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

AlarmsSnapshot alarms_snapshot() {
    static constexpr const char* kStatusOnline = "operator alarm board online";
    static constexpr const char* kAxes[4] = {"X", "Y", "Z", "A"};
    static char messages[4][48];

    AlarmsSnapshot snap{};
    snap.service_online = true;
    snap.status = kStatusOnline;
    for (size_t i = 0; i < 4 && i < snap.active.size(); ++i) {
        const auto& axis = motion::g_motion.axis(i);
        if (!axis.fault_latched) continue;
        kernel::util::k_snprintf(messages[snap.active_count], sizeof(messages[0]),
                                 "DRIVE FAULT 0x%04x", axis.last_error_code);
        auto& entry = snap.active[snap.active_count];
        entry.id = 100u + static_cast<uint32_t>(i);
        entry.message = messages[snap.active_count];
        entry.axis = kAxes[i];
        entry.severity = AlarmsSnapshot::Severity::Error;
        entry.timestamp_ns = static_cast<uint64_t>(g_machine.tick) * 100000000ULL;
        entry.acknowledged = false;
        ++snap.active_count;
    }
    snap.history_count = snap.active_count;
    for (size_t i = 0; i < snap.active_count && i < snap.history.size(); ++i) {
        snap.history[i] = snap.active[i];
    }
    bool macro_fault = false;
    int32_t macro_fault_code = 0;
    if (machine::g_registry.get_bool("macro_fault", macro_fault) && macro_fault &&
        machine::g_registry.get_int("macro_fault_code", macro_fault_code) &&
        snap.active_count < snap.active.size()) {
        auto& entry = snap.active[snap.active_count++];
        entry.id = 500u + static_cast<uint32_t>(macro_fault_code);
        entry.message = "MACRO FAULT";
        entry.axis = "MAC";
        entry.severity = AlarmsSnapshot::Severity::Error;
        entry.timestamp_ns = static_cast<uint64_t>(g_machine.tick) * 100000000ULL;
        entry.acknowledged = false;
    }
    if (snap.active_count != 0) snap.status = "ACTIVE MACHINE ALARM";
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
    (void)alarm_id;
    reset_alarm();
}

void clear_alarm_history() {
}

bool save_setup() {
    return true;
}

bool load_setup() {
    return true;
}

EthercatSnapshot ethercat_snapshot() {
    EthercatSnapshot snap{};
    const ethercat::Master* master = primary_master();
    if (!master) return snap;

    snap.available = true;
    snap.master_state = static_cast<uint8_t>(master->state());
    snap.discovered = master->stats().discovered.load(std::memory_order_relaxed);
    snap.cycles = master->stats().cycles.load(std::memory_order_relaxed);
    snap.tx_frames = master->stats().tx_frames.load(std::memory_order_relaxed);
    snap.rx_frames = master->stats().rx_frames.load(std::memory_order_relaxed);
    snap.deadline_miss = master->stats().cycle_deadline_miss.load(std::memory_order_relaxed);
    snap.esm_timeouts = master->stats().esm_timeouts.load(std::memory_order_relaxed);
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
    }
    return snap;
}

} // namespace kernel::ui::operator_api
