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

struct MachineSnapshot {
    PageId page = PageId::Position;
    Mode mode = Mode::Ready;
    bool hold = false;
    uint32_t selected_axis = 0;
    int32_t jog_increment = 100;               // counts applied per JOG± click (default 0.1 mm)
    int32_t axis_pos[4]{0, 0, 0, 0};           // actual feedback position (compat alias for axis:<n>)
    int32_t cmd_pos[4]{0, 0, 0, 0};            // commanded setpoint
    int32_t dtg[4]{0, 0, 0, 0};                // distance-to-go (target - actual)
    bool axis_homed[4]{false, false, false, false};
    uint32_t cycle_progress = 0;
    uint32_t torque = 0;
    int32_t feed = 0;
    int32_t spindle = 0;
    int32_t spindle_rpm = 0;                   // signed spindle command (rpm)
    int32_t spindle_load = 0;                  // permille torque on spindle axis
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
    size_t slave_count = 0;
    EthercatSlaveSnapshot slaves[6]{};
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
void set_selected_axis(uint32_t idx);    // explicit axis pick (X=0..A=3), no rotation
void set_jog_increment(int32_t counts);  // clamps to [1, 1_000_000]
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
// Mid-program restart on channel 0: dry-scans modal state to `line_no`,
// leaving the interpreter in Ready at that line. Caller must follow up
// with toggle_cycle() to begin motion from the reconstructed state.
bool restart_program_at_line(size_t line_no);
void set_feed_override(int32_t feed);
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
bool save_setup();
bool load_setup();

FileSnapshot file_snapshot();
bool create_file(const char* path);
bool delete_file(const char* path);
bool read_file(const char* path, char* buffer, size_t bufsize);
bool write_file(const char* path, const char* data, size_t len);

} // namespace kernel::ui::operator_api
