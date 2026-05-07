// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace render::gles1 { struct Vec3f; }

namespace kernel::ui::operator_api {

enum class PageId : uint8_t {
    Position = 0, // Dashboard / jog surface
    Program,      // Program browser / preview
    Offsets,      // Work and tool tables
    EtherCAT,     // Bus and diagnostics
    Alarms,       // Alarm history / operator recovery
    Setup,        // Persistence and setup actions
};

enum class Mode : uint8_t {
    Ready = 0,
    Running,
    Hold,
    Homing,
    Alarm,
};

// Operator-declared intent. Distinct from `Mode`, which is the derived
// machine state. Today the buttons set this and the dashboard displays
// it; nothing actually gates behaviour on the value yet — see
// set_operator_mode in operator_api.cpp for the enforcement TODO.
enum class OperatorMode : uint8_t {
    Auto = 0,
    MDI,
    Jog,
    Setup,
};

struct MachineSnapshot {
    PageId page = PageId::Position;
    Mode mode = Mode::Ready;
    bool hold = false;
    uint32_t selected_axis = 0;
    int32_t jog_increment = 100;               // counts applied per JOG± click (default 0.1 mm)
    int32_t jog_feed_cps = 10000;              // counts/s used by continuous (hold-to-move) jog
    int32_t axis_pos[4]{0, 0, 0, 0};           // actual feedback position (compat alias for axis:<n>)
    int32_t cmd_pos[4]{0, 0, 0, 0};            // commanded setpoint
    int32_t dtg[4]{0, 0, 0, 0};                // distance-to-go (target - actual)
    bool axis_homed[4]{false, false, false, false};
    // True when the axis's actual position is within 10% of either soft
    // limit. Drives the DRO digit colour (red) on the dashboard. Stays
    // false until the operator commands software limits via
    // axis_sw_limits — soft limits are not auto-populated.
    bool axis_near_limit[4]{false, false, false, false};
    OperatorMode operator_mode = OperatorMode::Jog;
    uint32_t cycle_progress = 0;
    uint32_t torque = 0;
    int32_t feed = 0;
    int32_t spindle = 0;
    int32_t spindle_rpm = 0;                   // signed spindle command (rpm)
    int32_t spindle_load = 0;                  // permille torque on spindle axis
    uint32_t spindle_override = 100;           // operator-side override percent (0..150)
    uint32_t wcs_index = 0;                    // 0=G54, 1=G55, ...
    bool inch_mode = false;
    uint32_t block_current = 0;
    uint32_t block_next = 0;
    uint32_t runtime_ms = 0;
    uint32_t parts = 0;
    uint32_t tick = 0;
    // --- machine-view page overlay flags ---
    // The `gles1:machine` image widget consults these to decide whether to
    // draw the loaded program's toolpath on top, and whether to draw toolpod
    // markers. Operator buttons toggle them via view:* actions.
    bool view_toolpath = false;
    bool view_toolpods = true;
};

struct EthercatSlaveSnapshot {
    bool present = false;
    uint16_t station_addr = 0;
    uint8_t current_state = 0;
    uint8_t target_state = 0;
    bool identity_mismatch = false;
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    // Per-slave hot-plug telemetry: total times this slave's `present` flag
    // flipped from true to false since boot. Non-zero means the bus dropped
    // and reacquired the slave at least once. See master.cpp run_loop for
    // the threshold (kAbsentCycles).
    uint32_t presence_loss_events = 0;
};

struct EthercatSnapshot {
    bool available = false;
    uint8_t master_state = 0;
    uint32_t discovered = 0;
    uint64_t cycles = 0;
    uint64_t tx_frames = 0;
    uint64_t rx_frames = 0;
    uint32_t deadline_miss = 0;
    uint32_t esm_timeouts = 0;
    uint32_t deadline_trips = 0;
    bool deadline_fault = false;
    uint64_t cycle_p99_us = 0;
    uint64_t cycle_max_us = 0;
    uint32_t period_us = 0;
    size_t slave_count = 0;
    EthercatSlaveSnapshot slaves[6]{};
    // DC-sync drift telemetry. last_dc_drift_ns is a signed delta between
    // consecutive ESC reg 0x0920 samples for one slave; dc_sync_faulted
    // latches when consecutive samples exceed the master's drift threshold.
    int64_t  last_dc_drift_ns = 0;
    uint64_t dc_drift_max_ns  = 0;
    uint32_t dc_sync_samples  = 0;
    uint32_t dc_sync_trips    = 0;
    bool     dc_sync_faulted  = false;
};

struct OffsetsSnapshot {
    size_t active_work = 0;
    size_t active_tool = 0;
    struct WorkRow {
        const char* name = "";
        float axis[4]{0.0f, 0.0f, 0.0f, 0.0f};
    };
    struct ToolRow {
        uint32_t tool = 0;
        float length = 0.0f;
        float radius = 0.0f;
        float wear = 0.0f;
    };
    struct Row {
        const char* axis = "";
        const char* work_label = "";
        int32_t work_offset = 0;
        const char* tool_label = "";
        int32_t tool_offset = 0;
        bool active = false;
    };
    const char* workset_name = "";
    const char* tool_name = "";
    const char* status = "";
    const char* backend_note = "";
    size_t row_count = 0;
    std::array<WorkRow, 6> work{};
    std::array<ToolRow, 8> tools{};
    std::array<Row, 4> rows{};
};

struct ProgramSnapshot {
    struct ProgramRow {
        const char* name = "";
        const char* path = "";
        const char* status = "";
        size_t blocks = 0;
        size_t bytes = 0;
        bool selected = false;
        bool loaded = false;
    };
    struct ChannelState {
        const char* selected_name = "";
        const char* loaded_name = "";
        const char* runtime = "";
        size_t selected_index = 0;
        size_t loaded_index = 0;
        size_t preview_points = 0;
        size_t blocks = 0;
        size_t line = 0;
        uint16_t barrier_token = 0;
        uint8_t barrier_mask = 0;
    };
    bool browser_available = false;
    bool parser_available = false;
    bool simulation_hook_ready = false;
    size_t count = 0;
    size_t selected = 0;
    const char* selected_name = "";
    size_t selected_size = 0;
    size_t preview_points = 0;
    size_t program_count = 0;
    size_t selected_index = 0;
    const char* status = "";
    const char* backend_note = "";
    std::array<const char*, 8> names{};
    std::array<ProgramRow, 8> programs{};
    std::array<ChannelState, 2> channels{};
};

struct MacroSnapshot {
    struct MacroRow {
        const char* id = "";
        const char* title = "";
        uint16_t mcode = 0;
        bool selected = false;
    };
    bool available = false;
    size_t count = 0;
    size_t selected = 0;
    const char* selected_name = "";
    const char* active_name = "";
    const char* status = "";
    const char* message = "";
    size_t active_step = 0;
    bool busy = false;
    bool fault = false;
    std::array<MacroRow, 8> rows{};
};

struct AlarmsSnapshot {
    enum class Severity : uint8_t { Info = 0, Warning, Error, Critical };
    struct AlarmEntry {
        uint32_t id = 0;
        const char* message = "";
        const char* axis = "";
        Severity severity = Severity::Info;
        uint64_t timestamp_ns = 0;
        bool acknowledged = false;
    };
    size_t active_count = 0;
    size_t history_count = 0;
    bool service_online = false;
    const char* status = "";
    std::array<AlarmEntry, 8> active{};
    std::array<AlarmEntry, 12> history{};
};

struct SetupSnapshot {
    struct AxisConfig {
        const char* name = "";
        const char* type = "";
        int32_t position = 0;
        int32_t neg_limit = 0;
        int32_t pos_limit = 0;
        int32_t home_pos = 0;
        bool enabled = false;
        bool homed = false;
    };
    struct NetworkConfig {
        const char* master_name = "";
        const char* master_state = "";
        uint32_t slave_count = 0;
        uint32_t cycle_time_us = 0;
        bool dc_enabled = false;
    };
    struct SystemInfo {
        const char* kernel_version = "";
        const char* build_time = "";
        uint32_t uptime_s = 0;
        uint32_t mem_used_kb = 0;
        uint32_t mem_total_kb = 0;
    };
    bool service_online = false;
    const char* machine_name = "";
    const char* kinematics = "";
    AxisConfig axes[4]{};
    NetworkConfig network{};
    SystemInfo system{};
};

// Operator-driven homing wizard. The dashboard `HOME` button still kicks
// the legacy single-axis sequence; this snapshot powers the dedicated
// wizard page where the operator picks an axis + a strategy from a
// pre-set library before tapping HOME. State is the wizard's own view of
// progress, derived from the cyclic snapshot of motion::HomingEngine and
// the EtherCAT master deadline-fault latch.
struct HomingSnapshot {
    int32_t selected_axis = 0;
    const char* method_name = "(none)";
    int32_t method_id = 0;          // CiA-402 homing method number
    int32_t fast_cps = 50000;       // fast-search velocity (counts/s)
    int32_t slow_cps = 5000;        // slow-approach velocity (counts/s)
    int32_t accel_cps2 = 100000;
    int32_t torque_permille = 0;    // hardstop torque limit
    int32_t offset_counts = 0;
    enum class State : uint8_t { Idle, Configuring, Searching, Approaching, Done, Faulted };
    State state = State::Idle;
    const char* status_message = "Idle";
    bool homed_axes[4]{false, false, false, false};
};

HomingSnapshot homing_snapshot();
void set_homing_axis(uint32_t axis);
void set_homing_method(int32_t method_id, const char* name);
void set_homing_speeds(int32_t fast_cps, int32_t slow_cps);
void start_homing();
void abort_homing();

// Guided probing wizard. Walks the operator through select-cycle ->
// confirm setup -> start -> live progress -> accept / reject. Only the
// reference-sphere cycle has a real probe::Runtime backend; the other
// six cycles execute through the macro runtime (probe_qualify /
// probe_z_surface / probe_x_edge / probe_y_edge / probe_bore_xy /
// probe_calibrate_3d_pocket TSV macros). Selecting None or starting a
// cycle whose backend cannot be reached short-circuits to Faulted with
// a "cycle not implemented" status.
enum class ProbeWizardState : uint8_t {
    Idle = 0,
    Selecting,
    Confirming,
    Running,
    Inspecting,
    Faulted,
};
enum class ProbeCycleKind : uint8_t {
    None = 0,
    Qualify,
    ZSurface,
    EdgeX,
    EdgeY,
    BoreCenter,
    Pocket3D,
    Sphere,
};
struct ProbeWizardSnapshot {
    ProbeWizardState state = ProbeWizardState::Idle;
    ProbeCycleKind cycle = ProbeCycleKind::None;
    const char* cycle_name = "(none)";
    const char* status_message = "Idle";
    int32_t step = 0;
    int32_t total_steps = 0;
    int32_t result_x = 0;
    int32_t result_y = 0;
    int32_t result_z = 0;
    bool result_valid = false;
};

ProbeWizardSnapshot probe_wizard_snapshot();
void probe_wizard_select(ProbeCycleKind kind);
void probe_wizard_start();
void probe_wizard_abort();
// Inspecting -> commits captured result into the active WCS:
//   ZSurface   -> Z;  EdgeX -> X;  EdgeY -> Y;
//   BoreCenter -> XY; Pocket3D -> XYZ
// Qualify and Sphere are calibration cycles and never write a WCS slot.
// Captured counts are converted to mm using the 1000 counts/mm display scale
// shared with the DRO and offsets editor.
void probe_wizard_accept();
void probe_wizard_reject();

// Calibration / compensation operator surface. Surfaces the same backing
// state the cal_* and sphere_* CLI commands mutate (motion::g_motion's
// LinearCalibration / MachineGeometry / SphereCalibration tables) so the
// operator UI is a peer of the CLI rather than a wrapper over a copy.
struct CalibrationSnapshot {
    // PEC — pitch error compensation, axis-scoped. The UI shows one axis
    // at a time (selected by the operator); the backing motion table
    // stores per-axis arrays.
    int32_t  pec_axis        = 0;     // 0..3 selected by the operator
    bool     pec_enabled     = false; // axis-local enable flag
    uint16_t pec_point_count = 0;
    static constexpr size_t kMaxPecRows = 8; // rows displayed on the PEC page
    int32_t  pec_pos[kMaxPecRows]{};
    int32_t  pec_err[kMaxPecRows]{};
    int32_t  rotary_offset_a = 0;     // index offset for the A axis (counts)

    // Geometry — squareness errors between axis pairs (microradians).
    bool     geom_enabled = false;
    int32_t  geom_xy_urad = 0;
    int32_t  geom_xz_urad = 0;
    int32_t  geom_yz_urad = 0;

    // Volumetric (sphere) compensation.
    bool     sphere_enabled         = false;
    bool     sphere_errors_computed = false;
    float    sphere_diameter_mm     = 0.0f;
    int32_t  sphere_probe_radius_um = 0;
    size_t   sphere_point_count     = 0;
    // Last computed error stats (positional + squareness).
    int32_t  sphere_err_pos_x_urad = 0;
    int32_t  sphere_err_pos_y_urad = 0;
    int32_t  sphere_err_pos_z_urad = 0;
    int32_t  sphere_err_sq_xy_urad = 0;
    int32_t  sphere_err_sq_xz_urad = 0;
    int32_t  sphere_err_sq_yz_urad = 0;
};

CalibrationSnapshot calibration_snapshot();

// PEC mutators. The UI's ADD POINT input writes pos/err into a small
// pending struct; cal_pec_commit_point() flushes it through to motion's
// add_cal_point. All mutators are no-ops while either master is in
// deadline-fault — the kernel cannot guarantee correction safety while
// the bus is offline.
void cal_pec_select_axis(uint32_t axis);
void cal_pec_set_pending_pos(int32_t counts);
void cal_pec_set_pending_err(int32_t counts);
int32_t cal_pec_pending_pos();
int32_t cal_pec_pending_err();
bool cal_pec_commit_point();
void cal_pec_clear();
void cal_pec_set_enabled(bool en);

// Geometry / volumetric mutators. set_pair selects "xy"/"xz"/"yz".
void cal_geom_set(const char* pair, int32_t urad);
void cal_geom_set_enabled(bool en);
void cal_sphere_compute();
void cal_sphere_set_enabled(bool en);
void cal_sphere_clear();

struct FileSnapshot {
    struct FileEntry {
        const char* name = "";
        const char* path = "";
        size_t size = 0;
        bool is_dir = false;
    };
    bool available = false;
    const char* mount_point = "";
    size_t file_count = 0;
    size_t total_size = 0;
    size_t free_size = 0;
    std::array<FileEntry, 16> files{};
};

void select_page(PageId page);
PageId current_page();

void jog_selected(int32_t delta);        // raw-counts delta (backward-compat path)
void jog_axis_step(int32_t sign);        // sign in {-1,+1}; uses current axis + jog_increment
void start_continuous_jog(uint32_t axis, int32_t sign);  // hold-to-move; sign in {-1,+1}
void stop_continuous_jog(uint32_t axis);                  // release: drives axis velocity to 0
void set_selected_axis(uint32_t idx);    // explicit axis pick (X=0..A=3), no rotation
void set_jog_increment(int32_t counts);  // clamps to [1, 1_000_000]
void set_jog_feed_cps(int32_t cps);      // clamps to [1, 10_000_000]
void set_operator_mode(OperatorMode mode);
// Mode gate predicate consumed by ui_builder_tsv when the operator
// taps SUBMIT on the MDI page. MDI dispatch is silently dropped when
// the operator is not in MDI mode — the dashboard mode buttons are the
// safety lockout. Other gates (cycle-start, jog) are enforced inside
// the corresponding mutator and don't need an exported predicate.
bool mode_allows_mdi();
void toggle_view_toolpath();             // machine-view overlay: program path
void toggle_view_toolpods();             // machine-view overlay: toolpod markers
void home_selected();
void toggle_cycle();
void toggle_hold();
void reset_alarm();
bool select_work_offset(size_t idx);
bool select_tool_offset(size_t idx);
bool set_work_offset_axis(size_t idx, size_t axis, float value);
bool set_tool_offset(size_t idx, float length, float radius, float wear);
bool select_program(size_t idx);
bool select_program(size_t channel, size_t idx);
bool select_program_by_name(const char* name);
bool select_program_by_name(size_t channel, const char* name);
const char* selected_program_name();
const char* selected_program_name(size_t channel);
bool write_program(const char* name, const char* text);
bool select_prev_program();
bool select_prev_program(size_t channel);
bool select_next_program();
bool select_next_program(size_t channel);
bool request_program_simulation();
bool request_program_simulation(size_t channel);
// Mid-program restart on channel 0: stages the restart-confirm wizard.
// Calls cnc::interp::Runtime::restart_at_line under the hood (dry-scans
// modal state to line_no, leaving the interpreter in Ready) AND captures
// the reconstructed state into a per-process review struct, with
// pending=true.
//
// While pending, toggle_cycle() returns early without driving Cycle
// Start. The operator must explicitly confirm via confirm_restart() —
// matching the safety expectation that "punching a line number then
// hitting Cycle Start" never moves the machine without the operator
// first seeing what the reconstruction implies.
bool restart_program_at_line(size_t line_no);

// Wizard surface used by the new restart_confirm TSV page.
struct RestartReviewSnapshot {
    bool     pending = false;
    bool     ok      = false;            // dry scan reached target_line
    size_t   target_line = 0;
    size_t   active_work = 0;            // 0 = G54
    size_t   active_tool = 0;            // 0 = T1
    bool     tool_length_active = false;
    uint32_t feed    = 0;
    int32_t  spindle = 0;
    bool     coolant_mist = false;
    bool     coolant_flood = false;
};
RestartReviewSnapshot restart_review_snapshot();
bool restart_review_pending();
bool request_restart_review(size_t line_no);
void confirm_restart();
void cancel_restart();
void set_feed_override(int32_t feed);
// Spindle override (0..150 %). Mirrors set_feed_override; gated on
// EtherCAT deadline-fault so a faulted bus can't be sped up by accident.
// Stored on motion::Kernel::Channel::overrides.spindle_permille (×10).
void set_spindle_override(int32_t pct);
void step_demo_tick();
bool select_prev_macro();
bool select_next_macro();
bool run_selected_macro();
bool run_macro_by_name(const char* id);
bool abort_macro();

MachineSnapshot machine_snapshot();
EthercatSnapshot ethercat_snapshot();
OffsetsSnapshot offsets_snapshot();
const ProgramSnapshot& program_snapshot();
MacroSnapshot macro_snapshot();
AlarmsSnapshot alarms_snapshot();
SetupSnapshot setup_snapshot();
size_t selected_program_preview(const render::gles1::Vec3f*& points_out);
size_t selected_program_preview(size_t channel, const render::gles1::Vec3f*& points_out);

void acknowledge_alarm(uint32_t alarm_id);
void clear_alarm_history();

// Operator-initiated EtherCAT actions. Both call into ethercat::Master
// (g_master_a + g_master_b). request_ec_estop broadcasts CW_CMD_QUICK_STOP
// to every CiA-402 servo and latches the deadline-fault flag; clear_ec_fault
// drops both the deadline-fault latch and the DC-sync-drift latch (the
// latter has no other operator-accessible clear path; see master.cpp).
void request_ec_estop();
void clear_ec_fault();
bool save_setup();
bool load_setup();

// Spindle control. Sign of `rpm` selects direction (positive = forward,
// negative = reverse). All three calls are no-ops while either EtherCAT
// master has its deadline-fault latch set.
void spindle_set_rpm(int32_t rpm);
void spindle_start();
void spindle_stop();

struct SpindleStatus {
    int32_t requested_rpm = 0;
    int32_t actual_rpm = 0;
    int32_t load_permille = 0;
    int axis_index = 3;
    bool running = false;
    bool deadline_faulted = false;
};
SpindleStatus spindle_status();

// Resolves the spindle's motion-axis index from the loaded topology
// (binding name containing "spindle"); returns 3 when no such binding
// is registered, matching the legacy snapshot reader.
int spindle_axis_index() noexcept;

FileSnapshot file_snapshot();
bool create_file(const char* path);
bool delete_file(const char* path);
bool read_file(const char* path, char* buffer, size_t bufsize);
bool write_file(const char* path, const char* data, size_t len);

// ===== Network setup operator surface =====
// Surfaces hmi::Service state (DHCP, link, ping) plus a small editor for
// the static-IP fallback fields. Mutators are gated on master deadline-fault
// like every other operator-side write — networking changes that happen
// while the bus is faulted should wait for explicit clear_ec_fault.
struct NetworkSnapshot {
    enum class DhcpState : uint8_t { Idle = 0, Discovering, Bound, Timeout, Static };
    enum class LinkState : uint8_t { Down = 0, Up, Probing };
    enum class PingResultKind : uint8_t {
        None = 0, Ok, Busy, BadAddress, Timeout, SendFailed,
    };
    bool service_online = false;
    uint8_t  nic_idx = 0;
    char     nic_name[16]{};
    uint8_t  mac[6]{};
    uint32_t local_ip = 0;        // big-endian-free host-order, render dotted
    uint32_t netmask = 0;
    uint32_t gateway = 0;
    uint32_t pending_ip = 0;       // operator-edited static IP
    uint32_t pending_gateway = 0;  // operator-edited static gateway
    uint32_t pending_ping_target = 0;
    bool     dhcp_enabled = true;
    DhcpState dhcp_state = DhcpState::Idle;
    LinkState link_state = LinkState::Down;
    PingResultKind last_ping_result = PingResultKind::None;
    uint32_t last_ping_target = 0;
    uint32_t last_ping_rtt_ms = 0;
    uint64_t rx_requests = 0;
    uint64_t tx_responses = 0;
    uint32_t uptime_s = 0;
};

NetworkSnapshot network_snapshot();
void net_set_dhcp(bool enabled);
void net_commit_static();                 // pushes pending_ip/gateway live
void net_set_pending_ip(uint32_t ip);
void net_set_pending_gateway(uint32_t gw);
void net_set_pending_ping_target(uint32_t ip);
void net_request_ping();                  // fires async ping to pending_ping_target

// ===== Per-axis status sub-page operator surface =====
// Reads the SELECTED axis (machine_snapshot().selected_axis) and exposes
// motion::Axis fields directly. ENABLE/DISABLE/FAULT_RESET mutators all go
// through motion::Kernel and are gated on master deadline-fault.
struct AxisStatusSnapshot {
    uint32_t selected_axis = 0;
    int32_t  cmd_pos = 0;
    int32_t  actual_pos = 0;
    int32_t  following_error = 0;
    int32_t  max_following_error = 0;
    int32_t  vmax_cps = 0;
    int32_t  accel_cps2 = 0;
    int32_t  jerk_cps3 = 0;
    uint8_t  drive_state = 0;       // motion::DriveState raw enum
    uint8_t  traj_state = 0;        // motion::TrajState raw enum
    uint8_t  mode = 0;              // motion::Mode raw enum
    bool     enabled = false;
    bool     fault_latched = false;
    bool     homed = false;
    uint16_t last_error_code = 0;
    uint16_t status_word = 0;
    uint16_t control_word = 0;
};

AxisStatusSnapshot axis_status_snapshot();
void axis_detail_enable();
void axis_detail_disable();
void axis_detail_fault_reset();

// ===== Tool change wizard operator surface =====
// MANUAL-SWAP confirmation flow. There is no automatic tool changer in the
// loop — START transitions Idle→AwaitSwap and prints "Manual tool change —
// verify pod, then ACCEPT"; the operator physically swaps the tool by hand,
// then taps ACCEPT which commits the new (pod, station) into
// machine::toolpods::Service via select_station(). The spindle does NOT
// move and motion::g_motion is NOT touched. The internal Releasing /
// Picking / Verifying enum values are dead today (the toolpods service
// only emits Idle / Moving / Done / Faulted) and are mapped to the same
// "AWAIT SWAP" label by the UI to reflect that.
//
// A real ATC controller (drawer release, spindle index, pull-stud unlock,
// pick-and-place, kinematic verify) is a follow-up that would replace
// the simple state machine here without changing the operator surface.
struct ToolChangeSnapshot {
    enum class State : uint8_t {
        Idle = 0, Releasing, Moving, Picking, Verifying, Done, Faulted,
    };
    State state = State::Idle;
    uint32_t step = 0;
    uint32_t total_steps = 0;
    uint32_t current_tool = 0;
    uint32_t target_tool = 0;
    char     current_pod[24]{};
    uint32_t current_station = 0;
    char     current_label[24]{};
    char     target_pod[24]{};
    uint32_t target_station = 0;
    char     target_label[24]{};
    bool     target_resolved = false;     // false → "unassigned"
    char     status_message[64]{};
};

ToolChangeSnapshot tool_change_snapshot();
void tool_change_set_pending_target(uint32_t tool);
void tool_change_start();
void tool_change_abort();
void tool_change_accept();

} // namespace kernel::ui::operator_api
