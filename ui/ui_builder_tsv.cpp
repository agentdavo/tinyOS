// SPDX-License-Identifier: MIT OR Apache-2.0

#include "ui_builder_tsv.hpp"

#include "render/gles1.hpp"
#include "render/kinematic_model.hpp"
#include "render/machine_model.hpp"
#include "machine/machine_registry.hpp"
#include "machine/toolpods.hpp"
#include "cnc/programs.hpp"
#include "cnc/mdi.hpp"
#include "motion/motion.hpp"
#include "ui/operator_api.hpp"
#include "ui/ui.hpp"
#include "miniOS.hpp"
#include "util.hpp"

#include <cstring>
#include <new>

#include "fs/vfs.hpp"

#include "core.hpp"

namespace ui_builder {

namespace {

using kernel::ui::Color;
using kernel::ui::Container;
using kernel::ui::Framebuffer;
using kernel::ui::Panel;
using kernel::ui::UIEvent;
using kernel::ui::Widget;
namespace gles1 = render::gles1;
namespace kinematic = render::kinematic;
namespace machine = render::machine;

constexpr uint32_t kTransparent = 0x00000000U;
constexpr uint32_t MAX_LAYOUT_CHILDREN = 32;

enum class RecordType : uint8_t {
    Unknown = 0,
    Page,
    Include,
    Child,
    Action,
    Widget,
};

enum class BuilderEvent : uint8_t {
    None = 0,
    Click,
    Change,
    Timer,
};

struct ActionSpec {
    char widget_id[MAX_FIELD_LEN] = {};
    char target[MAX_FIELD_LEN] = {};
    BuilderEvent event = BuilderEvent::None;
};

struct PageSpec {
    char id[MAX_FIELD_LEN] = {};
    char title[MAX_FIELD_LEN] = {};
    uint32_t first_widget = 0;
    uint32_t widget_count = 0;
    Widget* root = nullptr;
};

struct ChildSpec {
    char parent_id[MAX_FIELD_LEN] = {};
    char widget_id[MAX_FIELD_LEN] = {};
    Align align = Align::Left;
};

struct WidgetNode {
    WidgetSpec spec;
    Widget* widget = nullptr;
    int parent = -1;
    int page = -1;
    Align child_align = Align::Left;
};

struct LayoutBucket {
    int parent = -1;
    int children[MAX_LAYOUT_CHILDREN]{};
    uint32_t count = 0;
};

static WidgetNode g_widgets[MAX_WIDGETS];
static uint32_t g_widget_count = 0;
static PageSpec g_pages[MAX_PAGES];
static uint32_t g_page_count = 0;
static ActionSpec g_actions[MAX_ACTIONS];
static uint32_t g_action_count = 0;
// Sorted index over g_actions, populated by build_action_index() after a
// successful validate_actions(). Lets action_target_for_widget binary-search
// instead of doing an O(action_count) strcmp scan on every button render.
// Empty (g_action_index_count = 0) until parse completes; lookups fall back
// to the linear scan in that window.
static uint16_t g_action_index[MAX_ACTIONS];
static uint32_t g_action_index_count = 0;
static ChildSpec g_child_links[MAX_CHILD_LINKS];
static uint32_t g_child_link_count = 0;
static Widget* g_root_widget = nullptr;
static int g_active_page = 0;
static Widget* g_focusables[MAX_WIDGETS];
static int32_t g_focus_orders[MAX_WIDGETS];
static uint32_t g_focusable_count = 0;
static int g_focus_index = -1;
class BuilderInput;
static BuilderInput* g_inputs[MAX_WIDGETS];
static uint32_t g_input_count = 0;
static constexpr uint32_t kInputFeedbackTicks = 12;
static char g_last_error[160] = {};
static uint32_t g_last_error_line = 0;
static uint32_t g_click_trace_count = 0;
static kernel::core::Spinlock g_state_lock;

void advance_focus(int step);

// Bridges for view:* actions so run_action_target (defined before the
// BuilderImage class) can reach BuilderImage's static camera helpers.
// Implemented below the class definition.
void view_reset_all_cameras();
void view_zoom_all(float factor);

void trace_click_delivery(const char* fmt, const char* a = nullptr, const char* b = nullptr,
                          long c = 0, long d = 0) {
    if (g_click_trace_count >= 12) return;
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (!uart || !fmt) return;
    char buf[192];
    kernel::util::k_snprintf(buf, sizeof(buf), fmt,
                             a ? a : "", b ? b : "", c, d);
    uart->puts(buf);
    ++g_click_trace_count;
}

void register_focusable(Widget* widget, int32_t focus_order) {
    if (!widget || g_focusable_count >= MAX_WIDGETS) return;
    uint32_t insert_at = g_focusable_count;
    while (insert_at > 0 && focus_order < g_focus_orders[insert_at - 1]) {
        g_focusables[insert_at] = g_focusables[insert_at - 1];
        g_focus_orders[insert_at] = g_focus_orders[insert_at - 1];
        --insert_at;
    }
    g_focusables[insert_at] = widget;
    g_focus_orders[insert_at] = focus_order;
    ++g_focusable_count;
    if (g_focus_index < 0) g_focus_index = 0;
}

Color to_color(uint32_t argb, Color fallback = Color::Black()) {
    if (argb == kTransparent) return fallback;
    return Color(static_cast<uint8_t>((argb >> 16) & 0xFF),
                 static_cast<uint8_t>((argb >> 8) & 0xFF),
                 static_cast<uint8_t>(argb & 0xFF),
                 static_cast<uint8_t>((argb >> 24) & 0xFF));
}

void copy_field(char* dst, size_t dst_size, const char* src, size_t src_len) {
    if (!dst || dst_size == 0) return;
    size_t n = src_len;
    if (n >= dst_size) n = dst_size - 1;
    if (src && n > 0) memcpy(dst, src, n);
    dst[n] = '\0';
}

int32_t simple_atoi(const char* s) {
    if (!s) return 0;
    int sign = 1;
    if (*s == '-') {
        sign = -1;
        ++s;
    }
    int32_t n = 0;
    while (*s >= '0' && *s <= '9') {
        n = n * 10 + (*s - '0');
        ++s;
    }
    return n * sign;
}

float simple_atof(const char* s) {
    if (!s) return 0.0f;
    int sign = 1;
    if (*s == '-') {
        sign = -1;
        ++s;
    }
    int32_t whole = 0;
    while (*s >= '0' && *s <= '9') {
        whole = whole * 10 + (*s - '0');
        ++s;
    }
    float value = static_cast<float>(whole);
    if (*s == '.') {
        ++s;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') {
            value += static_cast<float>(*s - '0') * scale;
            scale *= 0.1f;
            ++s;
        }
    }
    return sign < 0 ? -value : value;
}

uint8_t simple_hex(char c) {
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
    if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
    return 0;
}

uint32_t parse_color(const char* s) {
    if (!s || *s == '\0') return kTransparent;
    if (s[0] == '#' && s[1] && s[2] && s[3] && s[4] && s[5] && s[6]) {
        const uint8_t r = static_cast<uint8_t>(simple_hex(s[1]) * 16 + simple_hex(s[2]));
        const uint8_t g = static_cast<uint8_t>(simple_hex(s[3]) * 16 + simple_hex(s[4]));
        const uint8_t b = static_cast<uint8_t>(simple_hex(s[5]) * 16 + simple_hex(s[6]));
        return 0xFF000000U | (static_cast<uint32_t>(r) << 16) |
               (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
    }
    if (strcmp(s, "black") == 0) return 0xFF000000U;
    if (strcmp(s, "white") == 0) return 0xFFFFFFFFU;
    if (strcmp(s, "red") == 0) return 0xFFFF0000U;
    if (strcmp(s, "green") == 0) return 0xFF00FF00U;
    if (strcmp(s, "blue") == 0) return 0xFF0000FFU;
    if (strcmp(s, "yellow") == 0) return 0xFFFFFF00U;
    if (strcmp(s, "cyan") == 0) return 0xFF00FFFFU;
    if (strcmp(s, "magenta") == 0) return 0xFFFF00FFU;
    if (strcmp(s, "gray") == 0) return 0xFF808080U;
    if (strcmp(s, "darkgray") == 0) return 0xFF404040U;
    if (strcmp(s, "lightgray") == 0) return 0xFFC0C0C0U;
    if (strcmp(s, "orange") == 0) return 0xFFFFA500U;
    return kTransparent;
}

uint32_t parse_font_size(const char* s) {
    if (!s || *s == '\0') return 16;
    if (strcmp(s, "small") == 0) return 12;
    if (strcmp(s, "medium") == 0) return 16;
    if (strcmp(s, "large") == 0) return 24;
    if (strcmp(s, "xlarge") == 0) return 32;
    const int32_t n = simple_atoi(s);
    return n > 0 ? static_cast<uint32_t>(n) : 16U;
}

WidgetType parse_type(const char* s) {
    if (!s) return WidgetType::Unknown;
    if (strcmp(s, "label") == 0) return WidgetType::Label;
    if (strcmp(s, "button") == 0) return WidgetType::Button;
    if (strcmp(s, "panel") == 0) return WidgetType::Panel;
    if (strcmp(s, "container") == 0) return WidgetType::Container;
    if (strcmp(s, "slider") == 0) return WidgetType::Slider;
    if (strcmp(s, "input") == 0) return WidgetType::Input;
    if (strcmp(s, "progress") == 0) return WidgetType::Progress;
    if (strcmp(s, "image") == 0) return WidgetType::Image;
    if (strcmp(s, "graph") == 0) return WidgetType::Graph;
    return WidgetType::Unknown;
}

LayoutType parse_layout(const char* s) {
    if (!s) return LayoutType::None;
    if (strcmp(s, "horizontal") == 0) return LayoutType::Horizontal;
    if (strcmp(s, "vertical") == 0) return LayoutType::Vertical;
    if (strcmp(s, "grid") == 0) return LayoutType::Grid;
    return LayoutType::None;
}

Align parse_align(const char* s) {
    if (!s) return Align::Left;
    if (strcmp(s, "center") == 0) return Align::Center;
    if (strcmp(s, "right") == 0) return Align::Right;
    return Align::Left;
}

BuilderEvent parse_event(const char* s) {
    if (!s) return BuilderEvent::None;
    if (strcmp(s, "click") == 0) return BuilderEvent::Click;
    if (strcmp(s, "change") == 0) return BuilderEvent::Change;
    if (strcmp(s, "timer") == 0) return BuilderEvent::Timer;
    return BuilderEvent::None;
}

RecordType parse_record_type(const char* s) {
    if (!s) return RecordType::Unknown;
    if (strcmp(s, "page") == 0) return RecordType::Page;
    if (strcmp(s, "include") == 0) return RecordType::Include;
    if (strcmp(s, "child") == 0) return RecordType::Child;
    if (strcmp(s, "action") == 0) return RecordType::Action;
    return parse_type(s) != WidgetType::Unknown ? RecordType::Widget : RecordType::Unknown;
}

void set_error(uint32_t line, const char* message) {
    g_last_error_line = line;
    copy_field(g_last_error, sizeof(g_last_error), message, message ? strlen(message) : 0);
}

void clear_error() {
    g_last_error[0] = '\0';
    g_last_error_line = 0;
}

struct KeyValueField {
    char key[32] = {};
    char value[MAX_FIELD_LEN] = {};
};

uint32_t parse_fields(const char* record, size_t len, KeyValueField* fields, uint32_t max_fields) {
    if (!record || len == 0 || !fields || max_fields == 0) return 0;
    const char* end = record + len;
    const char* type_end = record;
    while (type_end < end && *type_end != '\t' && *type_end != '\n') ++type_end;

    const char* p = type_end;
    if (p < end && *p == '\t') ++p;

    uint32_t count = 0;
    while (p < end && *p != '\n' && count < max_fields) {
        const char* key_start = p;
        while (p < end && *p != '=' && *p != '\t' && *p != '\n') ++p;
        const size_t key_len = static_cast<size_t>(p - key_start);
        if (key_len == 0 || key_len >= sizeof(fields[count].key)) {
            while (p < end && *p != '\t' && *p != '\n') ++p;
            if (p < end && *p == '\t') ++p;
            continue;
        }
        copy_field(fields[count].key, sizeof(fields[count].key), key_start, key_len);
        if (p < end && *p == '=') {
            ++p;
            const char* value_start = p;
            while (p < end && *p != '\t' && *p != '\n') ++p;
            copy_field(fields[count].value, sizeof(fields[count].value), value_start,
                       static_cast<size_t>(p - value_start));
        }
        ++count;
        if (p < end && *p == '\t') ++p;
    }
    return count;
}

const char* field_value(const KeyValueField* fields, uint32_t count, const char* key) {
    if (!fields || !key) return nullptr;
    for (uint32_t i = 0; i < count; ++i) {
        if (strcmp(fields[i].key, key) == 0) return fields[i].value;
    }
    return nullptr;
}

[[maybe_unused]] int32_t text_x_for_align(int32_t x, uint32_t width, const char* text, Align align) {
    if (!text || align == Align::Left || width == 0) return x;
    const int32_t text_w = static_cast<int32_t>(strlen(text) * 8U);
    if (align == Align::Center) return x + static_cast<int32_t>(width / 2) - text_w / 2;
    return x + static_cast<int32_t>(width) - text_w - 8;
}

// uint16_t — the existing catalogue + three new operator-facing pages
// (network, axis status, tool change) push this enum past 256 entries.
enum class BindKind : uint16_t {
    None = 0,
    Mode,
    Alarm,
    Prompt,
    CycleProgress,
    Torque,
    Feed,
    Spindle,
    AxisX,
    AxisY,
    AxisZ,
    AxisA,
    PageIndex,
    PageName,
    ProgramName,
    ProgramCount,
    ProgramBlocks,
    ProgramBytes,
    PreviewPoints,
    WorkName,
    ToolName,
    WorkOffsetX,
    WorkOffsetY,
    WorkOffsetZ,
    WorkOffsetA,
    ToolLength,
    ToolRadius,
    ToolWear,
    MacroName,
    MacroActive,
    MacroStatus,
    MacroStep,
    MacroCount,
    MacroMessage,
    ProbeX,
    ProbeY,
    ProbeZ,
    ProbeDone,
    ProbeStylus,
    ProbeCenterX,
    ProbeCenterY,
    ProbeSizeX,
    ProbeSizeY,
    ProbeShiftX,
    ProbeShiftY,
    ProbeShiftZ,
    ProbeSphereReady,
    ProbeSpherePoints,
    AxisXCmd, AxisYCmd, AxisZCmd, AxisACmd,
    AxisXAct, AxisYAct, AxisZAct, AxisAAct,
    AxisXDtg, AxisYDtg, AxisZDtg, AxisADtg,
    AxisXHomed, AxisYHomed, AxisZHomed, AxisAHomed,
    Wcs,
    Units,
    SpindleRpm,
    SpindleLoad,
    BlockCurrent,
    BlockNext,
    Runtime,
    Parts,
    MdiInput,
    MdiLast,
    MdiStatus,
    MdiMessage,
    MdiDepth,
    SelectedAxis,
    JogIncrement,
    ActiveTool,
    ViewToolpath,
    ViewToolpods,
    EcState,
    EcSlaves,
    EcMiss,
    EcTrips,
    EcFault,
    EcCycleP99,
    EcCycleMax,
    EcPeriod,
    EcCycles,
    EcTxFrames,
    EcRxFrames,
    EcDcFault,         // bool: DC sync drift latch tripped
    EcDcDriftMaxNs,    // peak |drift| since boot, ns
    EcDcDriftLastNs,   // most recent sample, ns (signed; rendered abs)
    EcDcSamples,       // total DC drift samples taken
    EcDcTrips,         // total DC drift fault latches
    SpindleOverride,   // operator-side spindle override percent (0..150)
    RestartPending,    // bool: restart-confirm wizard staged
    RestartLine,       // target line number
    RestartTool,       // reconstructed active tool (1-based)
    RestartWcs,        // reconstructed WCS (G54..G59)
    RestartFeed,       // reconstructed F word
    RestartSpindle,    // reconstructed S word
    RestartCoolant,    // reconstructed coolant flags rendered as text
    Channel0BarrierMs, // Channel 0 barrier-timeout countdown in ms
    Channel1BarrierMs, // Channel 1 barrier-timeout countdown in ms
    Alarm0Id, Alarm0Severity, Alarm0Axis, Alarm0Message, Alarm0Time,
    Alarm1Id, Alarm1Severity, Alarm1Axis, Alarm1Message, Alarm1Time,
    Alarm2Id, Alarm2Severity, Alarm2Axis, Alarm2Message, Alarm2Time,
    Alarm3Id, Alarm3Severity, Alarm3Axis, Alarm3Message, Alarm3Time,
    AlarmActiveCount,
    AlarmHistoryCount,
    // Per-slot WCS readouts (G54..G59 X/Y/Z/A) — 24 entries laid out as
    // axis-major within each slot to keep the switch arithmetic trivial:
    // (bind - WorkOffset0X) / 4 = slot, ... % 4 = axis.
    WorkOffset0X, WorkOffset0Y, WorkOffset0Z, WorkOffset0A,
    WorkOffset1X, WorkOffset1Y, WorkOffset1Z, WorkOffset1A,
    WorkOffset2X, WorkOffset2Y, WorkOffset2Z, WorkOffset2A,
    WorkOffset3X, WorkOffset3Y, WorkOffset3Z, WorkOffset3A,
    WorkOffset4X, WorkOffset4Y, WorkOffset4Z, WorkOffset4A,
    WorkOffset5X, WorkOffset5Y, WorkOffset5Z, WorkOffset5A,
    // Per-slot tool table (T1..T8) — same layout: slot-major, three
    // fields per slot (length, radius, wear).
    Tool0Length, Tool0Radius, Tool0Wear,
    Tool1Length, Tool1Radius, Tool1Wear,
    Tool2Length, Tool2Radius, Tool2Wear,
    Tool3Length, Tool3Radius, Tool3Wear,
    Tool4Length, Tool4Radius, Tool4Wear,
    Tool5Length, Tool5Radius, Tool5Wear,
    Tool6Length, Tool6Radius, Tool6Wear,
    Tool7Length, Tool7Radius, Tool7Wear,
    AxisXNearLimit, AxisYNearLimit, AxisZNearLimit, AxisANearLimit,
    OperatorMode,
    // Per-row program browser readouts (slot 0..7) — name / size / selected /
    // loaded laid out as field-major within each row so the dispatch in
    // bound_numeric_value / format_bind_value stays a single arithmetic step.
    Program0Name, Program0Size, Program0Selected, Program0Loaded,
    Program1Name, Program1Size, Program1Selected, Program1Loaded,
    Program2Name, Program2Size, Program2Selected, Program2Loaded,
    Program3Name, Program3Size, Program3Selected, Program3Loaded,
    Program4Name, Program4Size, Program4Selected, Program4Loaded,
    Program5Name, Program5Size, Program5Selected, Program5Loaded,
    Program6Name, Program6Size, Program6Selected, Program6Loaded,
    Program7Name, Program7Size, Program7Selected, Program7Loaded,
    // Guided-homing wizard — operator-driven plan state surfaced from
    // ui::operator_api::homing_snapshot().
    HomingAxis, HomingMethod, HomingState, HomingMessage, HomingFastCps, HomingSlowCps,
    // Guided probing wizard — operator-driven cycle plan + live progress
    // surfaced from ui::operator_api::probe_wizard_snapshot().
    ProbeWizardState, ProbeWizardCycle, ProbeWizardStep, ProbeWizardTotal,
    ProbeWizardMessage, ProbeWizardResultX, ProbeWizardResultY, ProbeWizardResultZ,
    ProbeWizardResultValid,
    // Compensation operator surface (cal_*, cal_geometry, sphere_*) backed by
    // ui::operator_api::calibration_snapshot(). PEC table rows are
    // axis-major: the snapshot already filters to the operator-selected
    // axis, so binds 0..7 are positions and errors for that axis.
    CalPecAxis, CalPecEnabled, CalPecCount,
    CalPecPendingPos, CalPecPendingErr, CalRotaryOffset,
    CalPec0Pos, CalPec0Err, CalPec1Pos, CalPec1Err,
    CalPec2Pos, CalPec2Err, CalPec3Pos, CalPec3Err,
    CalPec4Pos, CalPec4Err, CalPec5Pos, CalPec5Err,
    CalPec6Pos, CalPec6Err, CalPec7Pos, CalPec7Err,
    CalGeomXy, CalGeomXz, CalGeomYz, CalGeomEnabled,
    CalSphereEnabled, CalSphereDiameter, CalSphereProbeUm,
    CalSpherePoints, CalSphereComputed,
    CalSphereErrPosX, CalSphereErrPosY, CalSphereErrPosZ,
    CalSphereErrSqXy, CalSphereErrSqXz, CalSphereErrSqYz,
    // Network setup page — projection of hmi::Service state via
    // operator_api::network_snapshot. IP / gateway / pending fields are
    // formatted dotted-quad in format_bind_value; mac is colon-hex.
    NetIp, NetGateway, NetMac, NetDhcpState, NetLinkState,
    NetPendingIp, NetPendingGateway, NetPendingPingTarget,
    NetLastPingTarget, NetLastPingResult, NetLastPingRtt,
    NetRxRequests, NetTxResponses, NetUptime, NetNicName,
    // Per-axis status sub-page — reads the SELECTED axis. All bindings
    // pull from operator_api::axis_status_snapshot() so a single
    // set_selected_axis call refreshes every value on the page.
    AxisDetailDrive, AxisDetailTrajState, AxisDetailMode,
    AxisDetailEnabled, AxisDetailFault, AxisDetailHomed,
    AxisDetailErrorCode, AxisDetailFollowingErr, AxisDetailMaxFollowingErr,
    AxisDetailVmax, AxisDetailAccel, AxisDetailJerk,
    AxisDetailCmd, AxisDetailAct, AxisDetailStatusWord, AxisDetailControlWord,
    // Tool change wizard — confirmation flow over the toolpod registry.
    ToolChangeCurrent, ToolChangeTarget, ToolChangeState, ToolChangeMessage,
    ToolChangeCurrentPod, ToolChangeCurrentStation, ToolChangeCurrentLabel,
    ToolChangeTargetPod, ToolChangeTargetStation, ToolChangeTargetLabel,
    ToolChangeStep, ToolChangeTotalSteps,
};

BindKind parse_bind(const char* s) {
    if (!s || *s == '\0') return BindKind::None;
    if (strcmp(s, "mode") == 0) return BindKind::Mode;
    if (strcmp(s, "alarm") == 0) return BindKind::Alarm;
    if (strcmp(s, "prompt") == 0) return BindKind::Prompt;
    if (strcmp(s, "cycle_progress") == 0) return BindKind::CycleProgress;
    if (strcmp(s, "torque") == 0) return BindKind::Torque;
    if (strcmp(s, "feed") == 0) return BindKind::Feed;
    if (strcmp(s, "spindle") == 0) return BindKind::Spindle;
    if (strcmp(s, "spindle_override") == 0) return BindKind::SpindleOverride;
    if (strcmp(s, "restart:pending") == 0) return BindKind::RestartPending;
    if (strcmp(s, "restart:line")    == 0) return BindKind::RestartLine;
    if (strcmp(s, "restart:tool")    == 0) return BindKind::RestartTool;
    if (strcmp(s, "restart:wcs")     == 0) return BindKind::RestartWcs;
    if (strcmp(s, "restart:feed")    == 0) return BindKind::RestartFeed;
    if (strcmp(s, "restart:spindle") == 0) return BindKind::RestartSpindle;
    if (strcmp(s, "restart:coolant") == 0) return BindKind::RestartCoolant;
    if (strcmp(s, "channel:0:barrier_ms") == 0) return BindKind::Channel0BarrierMs;
    if (strcmp(s, "channel:1:barrier_ms") == 0) return BindKind::Channel1BarrierMs;
    if (strcmp(s, "axis:x") == 0) return BindKind::AxisX;
    if (strcmp(s, "axis:y") == 0) return BindKind::AxisY;
    if (strcmp(s, "axis:z") == 0) return BindKind::AxisZ;
    if (strcmp(s, "axis:a") == 0) return BindKind::AxisA;
    if (strcmp(s, "page") == 0) return BindKind::PageIndex;
    if (strcmp(s, "page_name") == 0) return BindKind::PageName;
    if (strcmp(s, "program_name") == 0) return BindKind::ProgramName;
    if (strcmp(s, "program_count") == 0) return BindKind::ProgramCount;
    if (strcmp(s, "program_blocks") == 0) return BindKind::ProgramBlocks;
    if (strcmp(s, "program_bytes") == 0) return BindKind::ProgramBytes;
    if (strcmp(s, "preview_points") == 0) return BindKind::PreviewPoints;
    if (strcmp(s, "work_name") == 0) return BindKind::WorkName;
    if (strcmp(s, "tool_name") == 0) return BindKind::ToolName;
    if (strcmp(s, "work_offset:x") == 0) return BindKind::WorkOffsetX;
    if (strcmp(s, "work_offset:y") == 0) return BindKind::WorkOffsetY;
    if (strcmp(s, "work_offset:z") == 0) return BindKind::WorkOffsetZ;
    if (strcmp(s, "work_offset:a") == 0) return BindKind::WorkOffsetA;
    if (strcmp(s, "tool_length") == 0) return BindKind::ToolLength;
    if (strcmp(s, "tool_radius") == 0) return BindKind::ToolRadius;
    if (strcmp(s, "tool_wear") == 0) return BindKind::ToolWear;
    if (strcmp(s, "macro_name") == 0) return BindKind::MacroName;
    if (strcmp(s, "macro_active") == 0) return BindKind::MacroActive;
    if (strcmp(s, "macro_status") == 0) return BindKind::MacroStatus;
    if (strcmp(s, "macro_step") == 0) return BindKind::MacroStep;
    if (strcmp(s, "macro_count") == 0) return BindKind::MacroCount;
    if (strcmp(s, "macro_message") == 0) return BindKind::MacroMessage;
    if (strcmp(s, "probe_x") == 0) return BindKind::ProbeX;
    if (strcmp(s, "probe_y") == 0) return BindKind::ProbeY;
    if (strcmp(s, "probe_z") == 0) return BindKind::ProbeZ;
    if (strcmp(s, "probe_done") == 0) return BindKind::ProbeDone;
    if (strcmp(s, "probe_stylus") == 0) return BindKind::ProbeStylus;
    if (strcmp(s, "probe_center_x") == 0) return BindKind::ProbeCenterX;
    if (strcmp(s, "probe_center_y") == 0) return BindKind::ProbeCenterY;
    if (strcmp(s, "probe_size_x") == 0) return BindKind::ProbeSizeX;
    if (strcmp(s, "probe_size_y") == 0) return BindKind::ProbeSizeY;
    if (strcmp(s, "probe_shift_x") == 0) return BindKind::ProbeShiftX;
    if (strcmp(s, "probe_shift_y") == 0) return BindKind::ProbeShiftY;
    if (strcmp(s, "probe_shift_z") == 0) return BindKind::ProbeShiftZ;
    if (strcmp(s, "probe_sphere_ready") == 0) return BindKind::ProbeSphereReady;
    if (strcmp(s, "probe_sphere_points") == 0) return BindKind::ProbeSpherePoints;
    if (strcmp(s, "axis:x:cmd") == 0) return BindKind::AxisXCmd;
    if (strcmp(s, "axis:y:cmd") == 0) return BindKind::AxisYCmd;
    if (strcmp(s, "axis:z:cmd") == 0) return BindKind::AxisZCmd;
    if (strcmp(s, "axis:a:cmd") == 0) return BindKind::AxisACmd;
    if (strcmp(s, "axis:x:act") == 0) return BindKind::AxisXAct;
    if (strcmp(s, "axis:y:act") == 0) return BindKind::AxisYAct;
    if (strcmp(s, "axis:z:act") == 0) return BindKind::AxisZAct;
    if (strcmp(s, "axis:a:act") == 0) return BindKind::AxisAAct;
    if (strcmp(s, "axis:x:dtg") == 0) return BindKind::AxisXDtg;
    if (strcmp(s, "axis:y:dtg") == 0) return BindKind::AxisYDtg;
    if (strcmp(s, "axis:z:dtg") == 0) return BindKind::AxisZDtg;
    if (strcmp(s, "axis:a:dtg") == 0) return BindKind::AxisADtg;
    if (strcmp(s, "axis:x:homed") == 0) return BindKind::AxisXHomed;
    if (strcmp(s, "axis:y:homed") == 0) return BindKind::AxisYHomed;
    if (strcmp(s, "axis:z:homed") == 0) return BindKind::AxisZHomed;
    if (strcmp(s, "axis:a:homed") == 0) return BindKind::AxisAHomed;
    if (strcmp(s, "wcs") == 0) return BindKind::Wcs;
    if (strcmp(s, "units") == 0) return BindKind::Units;
    if (strcmp(s, "spindle_rpm") == 0) return BindKind::SpindleRpm;
    if (strcmp(s, "spindle_load") == 0) return BindKind::SpindleLoad;
    if (strcmp(s, "block_current") == 0) return BindKind::BlockCurrent;
    if (strcmp(s, "block_next") == 0) return BindKind::BlockNext;
    if (strcmp(s, "runtime") == 0) return BindKind::Runtime;
    if (strcmp(s, "parts") == 0) return BindKind::Parts;
    if (strcmp(s, "mdi:input") == 0) return BindKind::MdiInput;
    if (strcmp(s, "mdi:last") == 0) return BindKind::MdiLast;
    if (strcmp(s, "mdi:status") == 0) return BindKind::MdiStatus;
    if (strcmp(s, "mdi:message") == 0) return BindKind::MdiMessage;
    if (strcmp(s, "mdi:depth") == 0) return BindKind::MdiDepth;
    if (strcmp(s, "selected_axis") == 0) return BindKind::SelectedAxis;
    if (strcmp(s, "jog_increment") == 0) return BindKind::JogIncrement;
    if (strcmp(s, "active_tool") == 0) return BindKind::ActiveTool;
    if (strcmp(s, "view_toolpath") == 0) return BindKind::ViewToolpath;
    if (strcmp(s, "view_toolpods") == 0) return BindKind::ViewToolpods;
    if (strcmp(s, "ec:state")  == 0) return BindKind::EcState;
    if (strcmp(s, "ec:slaves") == 0) return BindKind::EcSlaves;
    if (strcmp(s, "ec:miss")   == 0) return BindKind::EcMiss;
    if (strcmp(s, "ec:trips")  == 0) return BindKind::EcTrips;
    if (strcmp(s, "ec:fault")  == 0) return BindKind::EcFault;
    if (strcmp(s, "ec:p99")    == 0) return BindKind::EcCycleP99;
    if (strcmp(s, "ec:max")    == 0) return BindKind::EcCycleMax;
    if (strcmp(s, "ec:period") == 0) return BindKind::EcPeriod;
    if (strcmp(s, "ec:cycles") == 0) return BindKind::EcCycles;
    if (strcmp(s, "ec:tx")     == 0) return BindKind::EcTxFrames;
    if (strcmp(s, "ec:rx")     == 0) return BindKind::EcRxFrames;
    if (strcmp(s, "ec:dc_fault")     == 0) return BindKind::EcDcFault;
    if (strcmp(s, "ec:dc_drift_max") == 0) return BindKind::EcDcDriftMaxNs;
    if (strcmp(s, "ec:dc_drift_last")== 0) return BindKind::EcDcDriftLastNs;
    if (strcmp(s, "ec:dc_samples")   == 0) return BindKind::EcDcSamples;
    if (strcmp(s, "ec:dc_trips")     == 0) return BindKind::EcDcTrips;
    if (strcmp(s, "alarm:active_count")  == 0) return BindKind::AlarmActiveCount;
    if (strcmp(s, "alarm:history_count") == 0) return BindKind::AlarmHistoryCount;
    // Per-slot WCS / tool tokens. Match exact strings; any typo falls
    // through to BindKind::None so the editor's catalogue stays the
    // single source of truth for valid names.
    {
        static const struct { const char* s; BindKind b; } kWcs[] = {
            {"wcs:G54:x", BindKind::WorkOffset0X}, {"wcs:G54:y", BindKind::WorkOffset0Y},
            {"wcs:G54:z", BindKind::WorkOffset0Z}, {"wcs:G54:a", BindKind::WorkOffset0A},
            {"wcs:G55:x", BindKind::WorkOffset1X}, {"wcs:G55:y", BindKind::WorkOffset1Y},
            {"wcs:G55:z", BindKind::WorkOffset1Z}, {"wcs:G55:a", BindKind::WorkOffset1A},
            {"wcs:G56:x", BindKind::WorkOffset2X}, {"wcs:G56:y", BindKind::WorkOffset2Y},
            {"wcs:G56:z", BindKind::WorkOffset2Z}, {"wcs:G56:a", BindKind::WorkOffset2A},
            {"wcs:G57:x", BindKind::WorkOffset3X}, {"wcs:G57:y", BindKind::WorkOffset3Y},
            {"wcs:G57:z", BindKind::WorkOffset3Z}, {"wcs:G57:a", BindKind::WorkOffset3A},
            {"wcs:G58:x", BindKind::WorkOffset4X}, {"wcs:G58:y", BindKind::WorkOffset4Y},
            {"wcs:G58:z", BindKind::WorkOffset4Z}, {"wcs:G58:a", BindKind::WorkOffset4A},
            {"wcs:G59:x", BindKind::WorkOffset5X}, {"wcs:G59:y", BindKind::WorkOffset5Y},
            {"wcs:G59:z", BindKind::WorkOffset5Z}, {"wcs:G59:a", BindKind::WorkOffset5A},
        };
        for (const auto& e : kWcs) if (strcmp(s, e.s) == 0) return e.b;
        static const struct { const char* s; BindKind b; } kTools[] = {
            {"tool:T1:length", BindKind::Tool0Length}, {"tool:T1:radius", BindKind::Tool0Radius}, {"tool:T1:wear", BindKind::Tool0Wear},
            {"tool:T2:length", BindKind::Tool1Length}, {"tool:T2:radius", BindKind::Tool1Radius}, {"tool:T2:wear", BindKind::Tool1Wear},
            {"tool:T3:length", BindKind::Tool2Length}, {"tool:T3:radius", BindKind::Tool2Radius}, {"tool:T3:wear", BindKind::Tool2Wear},
            {"tool:T4:length", BindKind::Tool3Length}, {"tool:T4:radius", BindKind::Tool3Radius}, {"tool:T4:wear", BindKind::Tool3Wear},
            {"tool:T5:length", BindKind::Tool4Length}, {"tool:T5:radius", BindKind::Tool4Radius}, {"tool:T5:wear", BindKind::Tool4Wear},
            {"tool:T6:length", BindKind::Tool5Length}, {"tool:T6:radius", BindKind::Tool5Radius}, {"tool:T6:wear", BindKind::Tool5Wear},
            {"tool:T7:length", BindKind::Tool6Length}, {"tool:T7:radius", BindKind::Tool6Radius}, {"tool:T7:wear", BindKind::Tool6Wear},
            {"tool:T8:length", BindKind::Tool7Length}, {"tool:T8:radius", BindKind::Tool7Radius}, {"tool:T8:wear", BindKind::Tool7Wear},
        };
        for (const auto& e : kTools) if (strcmp(s, e.s) == 0) return e.b;
    }
    if (strcmp(s, "axis:x:near_limit") == 0) return BindKind::AxisXNearLimit;
    if (strcmp(s, "axis:y:near_limit") == 0) return BindKind::AxisYNearLimit;
    if (strcmp(s, "axis:z:near_limit") == 0) return BindKind::AxisZNearLimit;
    if (strcmp(s, "axis:a:near_limit") == 0) return BindKind::AxisANearLimit;
    if (strcmp(s, "operator_mode") == 0) return BindKind::OperatorMode;
    if (strcmp(s, "alarm:0:id")   == 0) return BindKind::Alarm0Id;
    if (strcmp(s, "alarm:0:sev")  == 0) return BindKind::Alarm0Severity;
    if (strcmp(s, "alarm:0:axis") == 0) return BindKind::Alarm0Axis;
    if (strcmp(s, "alarm:0:msg")  == 0) return BindKind::Alarm0Message;
    if (strcmp(s, "alarm:0:time") == 0) return BindKind::Alarm0Time;
    if (strcmp(s, "alarm:1:id")   == 0) return BindKind::Alarm1Id;
    if (strcmp(s, "alarm:1:sev")  == 0) return BindKind::Alarm1Severity;
    if (strcmp(s, "alarm:1:axis") == 0) return BindKind::Alarm1Axis;
    if (strcmp(s, "alarm:1:msg")  == 0) return BindKind::Alarm1Message;
    if (strcmp(s, "alarm:1:time") == 0) return BindKind::Alarm1Time;
    if (strcmp(s, "alarm:2:id")   == 0) return BindKind::Alarm2Id;
    if (strcmp(s, "alarm:2:sev")  == 0) return BindKind::Alarm2Severity;
    if (strcmp(s, "alarm:2:axis") == 0) return BindKind::Alarm2Axis;
    if (strcmp(s, "alarm:2:msg")  == 0) return BindKind::Alarm2Message;
    if (strcmp(s, "alarm:2:time") == 0) return BindKind::Alarm2Time;
    if (strcmp(s, "alarm:3:id")   == 0) return BindKind::Alarm3Id;
    if (strcmp(s, "alarm:3:sev")  == 0) return BindKind::Alarm3Severity;
    if (strcmp(s, "alarm:3:axis") == 0) return BindKind::Alarm3Axis;
    if (strcmp(s, "alarm:3:msg")  == 0) return BindKind::Alarm3Message;
    if (strcmp(s, "alarm:3:time") == 0) return BindKind::Alarm3Time;
    {
        static const struct { const char* s; BindKind b; } kPrograms[] = {
            {"program:0:name", BindKind::Program0Name}, {"program:0:size", BindKind::Program0Size},
            {"program:0:selected", BindKind::Program0Selected}, {"program:0:loaded", BindKind::Program0Loaded},
            {"program:1:name", BindKind::Program1Name}, {"program:1:size", BindKind::Program1Size},
            {"program:1:selected", BindKind::Program1Selected}, {"program:1:loaded", BindKind::Program1Loaded},
            {"program:2:name", BindKind::Program2Name}, {"program:2:size", BindKind::Program2Size},
            {"program:2:selected", BindKind::Program2Selected}, {"program:2:loaded", BindKind::Program2Loaded},
            {"program:3:name", BindKind::Program3Name}, {"program:3:size", BindKind::Program3Size},
            {"program:3:selected", BindKind::Program3Selected}, {"program:3:loaded", BindKind::Program3Loaded},
            {"program:4:name", BindKind::Program4Name}, {"program:4:size", BindKind::Program4Size},
            {"program:4:selected", BindKind::Program4Selected}, {"program:4:loaded", BindKind::Program4Loaded},
            {"program:5:name", BindKind::Program5Name}, {"program:5:size", BindKind::Program5Size},
            {"program:5:selected", BindKind::Program5Selected}, {"program:5:loaded", BindKind::Program5Loaded},
            {"program:6:name", BindKind::Program6Name}, {"program:6:size", BindKind::Program6Size},
            {"program:6:selected", BindKind::Program6Selected}, {"program:6:loaded", BindKind::Program6Loaded},
            {"program:7:name", BindKind::Program7Name}, {"program:7:size", BindKind::Program7Size},
            {"program:7:selected", BindKind::Program7Selected}, {"program:7:loaded", BindKind::Program7Loaded},
        };
        for (const auto& e : kPrograms) if (strcmp(s, e.s) == 0) return e.b;
    }
    if (strcmp(s, "homing:axis")    == 0) return BindKind::HomingAxis;
    if (strcmp(s, "homing:method")  == 0) return BindKind::HomingMethod;
    if (strcmp(s, "homing:state")   == 0) return BindKind::HomingState;
    if (strcmp(s, "homing:message") == 0) return BindKind::HomingMessage;
    if (strcmp(s, "homing:fast")    == 0) return BindKind::HomingFastCps;
    if (strcmp(s, "homing:slow")    == 0) return BindKind::HomingSlowCps;
    if (strcmp(s, "probe:wizard:state")        == 0) return BindKind::ProbeWizardState;
    if (strcmp(s, "probe:wizard:cycle")        == 0) return BindKind::ProbeWizardCycle;
    if (strcmp(s, "probe:wizard:step")         == 0) return BindKind::ProbeWizardStep;
    if (strcmp(s, "probe:wizard:total")        == 0) return BindKind::ProbeWizardTotal;
    if (strcmp(s, "probe:wizard:message")      == 0) return BindKind::ProbeWizardMessage;
    if (strcmp(s, "probe:wizard:result_x")     == 0) return BindKind::ProbeWizardResultX;
    if (strcmp(s, "probe:wizard:result_y")     == 0) return BindKind::ProbeWizardResultY;
    if (strcmp(s, "probe:wizard:result_z")     == 0) return BindKind::ProbeWizardResultZ;
    if (strcmp(s, "probe:wizard:result_valid") == 0) return BindKind::ProbeWizardResultValid;
    if (strcmp(s, "cal:pec:axis")        == 0) return BindKind::CalPecAxis;
    if (strcmp(s, "cal:pec:enabled")     == 0) return BindKind::CalPecEnabled;
    if (strcmp(s, "cal:pec:count")       == 0) return BindKind::CalPecCount;
    if (strcmp(s, "cal:pec:pending_pos") == 0) return BindKind::CalPecPendingPos;
    if (strcmp(s, "cal:pec:pending_err") == 0) return BindKind::CalPecPendingErr;
    if (strcmp(s, "cal:rotary:offset")   == 0) return BindKind::CalRotaryOffset;
    {
        // PEC table rows — keep both columns parsed in the same loop so
        // any typo falls through to BindKind::None and surfaces in the
        // editor catalogue check.
        static const struct { const char* s; BindKind b; } kPec[] = {
            {"cal:pec:0:pos", BindKind::CalPec0Pos}, {"cal:pec:0:err", BindKind::CalPec0Err},
            {"cal:pec:1:pos", BindKind::CalPec1Pos}, {"cal:pec:1:err", BindKind::CalPec1Err},
            {"cal:pec:2:pos", BindKind::CalPec2Pos}, {"cal:pec:2:err", BindKind::CalPec2Err},
            {"cal:pec:3:pos", BindKind::CalPec3Pos}, {"cal:pec:3:err", BindKind::CalPec3Err},
            {"cal:pec:4:pos", BindKind::CalPec4Pos}, {"cal:pec:4:err", BindKind::CalPec4Err},
            {"cal:pec:5:pos", BindKind::CalPec5Pos}, {"cal:pec:5:err", BindKind::CalPec5Err},
            {"cal:pec:6:pos", BindKind::CalPec6Pos}, {"cal:pec:6:err", BindKind::CalPec6Err},
            {"cal:pec:7:pos", BindKind::CalPec7Pos}, {"cal:pec:7:err", BindKind::CalPec7Err},
        };
        for (const auto& e : kPec) if (strcmp(s, e.s) == 0) return e.b;
    }
    if (strcmp(s, "cal:geom:xy")        == 0) return BindKind::CalGeomXy;
    if (strcmp(s, "cal:geom:xz")        == 0) return BindKind::CalGeomXz;
    if (strcmp(s, "cal:geom:yz")        == 0) return BindKind::CalGeomYz;
    if (strcmp(s, "cal:geom:enabled")   == 0) return BindKind::CalGeomEnabled;
    if (strcmp(s, "cal:sphere:enabled")  == 0) return BindKind::CalSphereEnabled;
    if (strcmp(s, "cal:sphere:diameter") == 0) return BindKind::CalSphereDiameter;
    if (strcmp(s, "cal:sphere:probe_um") == 0) return BindKind::CalSphereProbeUm;
    if (strcmp(s, "cal:sphere:points")   == 0) return BindKind::CalSpherePoints;
    if (strcmp(s, "cal:sphere:computed") == 0) return BindKind::CalSphereComputed;
    if (strcmp(s, "cal:sphere:err_pos_x") == 0) return BindKind::CalSphereErrPosX;
    if (strcmp(s, "cal:sphere:err_pos_y") == 0) return BindKind::CalSphereErrPosY;
    if (strcmp(s, "cal:sphere:err_pos_z") == 0) return BindKind::CalSphereErrPosZ;
    if (strcmp(s, "cal:sphere:err_sq_xy") == 0) return BindKind::CalSphereErrSqXy;
    if (strcmp(s, "cal:sphere:err_sq_xz") == 0) return BindKind::CalSphereErrSqXz;
    if (strcmp(s, "cal:sphere:err_sq_yz") == 0) return BindKind::CalSphereErrSqYz;
    if (strcmp(s, "net:ip")             == 0) return BindKind::NetIp;
    if (strcmp(s, "net:gateway")        == 0) return BindKind::NetGateway;
    if (strcmp(s, "net:mac")            == 0) return BindKind::NetMac;
    if (strcmp(s, "net:dhcp")           == 0) return BindKind::NetDhcpState;
    if (strcmp(s, "net:link")           == 0) return BindKind::NetLinkState;
    if (strcmp(s, "net:pending_ip")     == 0) return BindKind::NetPendingIp;
    if (strcmp(s, "net:pending_gateway")== 0) return BindKind::NetPendingGateway;
    if (strcmp(s, "net:pending_ping")   == 0) return BindKind::NetPendingPingTarget;
    if (strcmp(s, "net:ping_target")    == 0) return BindKind::NetLastPingTarget;
    if (strcmp(s, "net:ping_result")    == 0) return BindKind::NetLastPingResult;
    if (strcmp(s, "net:ping_rtt")       == 0) return BindKind::NetLastPingRtt;
    if (strcmp(s, "net:rx_requests")    == 0) return BindKind::NetRxRequests;
    if (strcmp(s, "net:tx_responses")   == 0) return BindKind::NetTxResponses;
    if (strcmp(s, "net:uptime")         == 0) return BindKind::NetUptime;
    if (strcmp(s, "net:nic")            == 0) return BindKind::NetNicName;
    if (strcmp(s, "axis:detail:state")    == 0) return BindKind::AxisDetailDrive;
    if (strcmp(s, "axis:detail:traj")     == 0) return BindKind::AxisDetailTrajState;
    if (strcmp(s, "axis:detail:mode")     == 0) return BindKind::AxisDetailMode;
    if (strcmp(s, "axis:detail:enabled")  == 0) return BindKind::AxisDetailEnabled;
    if (strcmp(s, "axis:detail:fault")    == 0) return BindKind::AxisDetailFault;
    if (strcmp(s, "axis:detail:homed")    == 0) return BindKind::AxisDetailHomed;
    if (strcmp(s, "axis:detail:err_code") == 0) return BindKind::AxisDetailErrorCode;
    if (strcmp(s, "axis:detail:fe")       == 0) return BindKind::AxisDetailFollowingErr;
    if (strcmp(s, "axis:detail:fe_max")   == 0) return BindKind::AxisDetailMaxFollowingErr;
    if (strcmp(s, "axis:detail:vmax")     == 0) return BindKind::AxisDetailVmax;
    if (strcmp(s, "axis:detail:accel")    == 0) return BindKind::AxisDetailAccel;
    if (strcmp(s, "axis:detail:jerk")     == 0) return BindKind::AxisDetailJerk;
    if (strcmp(s, "axis:detail:cmd")      == 0) return BindKind::AxisDetailCmd;
    if (strcmp(s, "axis:detail:act")      == 0) return BindKind::AxisDetailAct;
    if (strcmp(s, "axis:detail:sw")       == 0) return BindKind::AxisDetailStatusWord;
    if (strcmp(s, "axis:detail:cw")       == 0) return BindKind::AxisDetailControlWord;
    if (strcmp(s, "tc:current")     == 0) return BindKind::ToolChangeCurrent;
    if (strcmp(s, "tc:target")      == 0) return BindKind::ToolChangeTarget;
    if (strcmp(s, "tc:state")       == 0) return BindKind::ToolChangeState;
    if (strcmp(s, "tc:msg")         == 0) return BindKind::ToolChangeMessage;
    if (strcmp(s, "tc:current_pod") == 0) return BindKind::ToolChangeCurrentPod;
    if (strcmp(s, "tc:current_st")  == 0) return BindKind::ToolChangeCurrentStation;
    if (strcmp(s, "tc:current_lbl") == 0) return BindKind::ToolChangeCurrentLabel;
    if (strcmp(s, "tc:target_pod")  == 0) return BindKind::ToolChangeTargetPod;
    if (strcmp(s, "tc:target_st")   == 0) return BindKind::ToolChangeTargetStation;
    if (strcmp(s, "tc:target_lbl")  == 0) return BindKind::ToolChangeTargetLabel;
    if (strcmp(s, "tc:step")        == 0) return BindKind::ToolChangeStep;
    if (strcmp(s, "tc:total")       == 0) return BindKind::ToolChangeTotalSteps;
    return BindKind::None;
}

int alarm_row_index(BindKind bind) {
    switch (bind) {
        case BindKind::Alarm0Id: case BindKind::Alarm0Severity: case BindKind::Alarm0Axis:
        case BindKind::Alarm0Message: case BindKind::Alarm0Time: return 0;
        case BindKind::Alarm1Id: case BindKind::Alarm1Severity: case BindKind::Alarm1Axis:
        case BindKind::Alarm1Message: case BindKind::Alarm1Time: return 1;
        case BindKind::Alarm2Id: case BindKind::Alarm2Severity: case BindKind::Alarm2Axis:
        case BindKind::Alarm2Message: case BindKind::Alarm2Time: return 2;
        case BindKind::Alarm3Id: case BindKind::Alarm3Severity: case BindKind::Alarm3Axis:
        case BindKind::Alarm3Message: case BindKind::Alarm3Time: return 3;
        default: return -1;
    }
}

const char* alarm_severity_text(kernel::ui::operator_api::AlarmsSnapshot::Severity sev) {
    using S = kernel::ui::operator_api::AlarmsSnapshot::Severity;
    switch (sev) {
        case S::Info:     return "INFO";
        case S::Warning:  return "WARN";
        case S::Error:    return "ERR";
        case S::Critical: return "CRIT";
    }
    return "?";
}

int32_t bound_numeric_value(BindKind bind) {
    switch (bind) {
        case BindKind::CycleProgress: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.cycle_progress);
        }
        case BindKind::Torque: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.torque);
        }
        case BindKind::Feed: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.feed;
        }
        case BindKind::SpindleOverride: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.spindle_override);
        }
        case BindKind::Spindle: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.spindle;
        }
        case BindKind::AxisX: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.axis_pos[0];
        }
        case BindKind::AxisY: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.axis_pos[1];
        }
        case BindKind::AxisZ: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.axis_pos[2];
        }
        case BindKind::AxisA: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.axis_pos[3];
        }
        case BindKind::PageIndex: return g_active_page;
        case BindKind::ProgramCount: {
            const auto& program = kernel::ui::operator_api::program_snapshot();
            return static_cast<int32_t>(program.program_count);
        }
        case BindKind::ProgramBlocks: {
            const auto& program = kernel::ui::operator_api::program_snapshot();
            return static_cast<int32_t>(program.programs[program.selected_index].blocks);
        }
        case BindKind::ProgramBytes: {
            const auto& program = kernel::ui::operator_api::program_snapshot();
            return static_cast<int32_t>(program.selected_size);
        }
        case BindKind::PreviewPoints: {
            const auto& program = kernel::ui::operator_api::program_snapshot();
            return static_cast<int32_t>(program.preview_points);
        }
        case BindKind::WorkOffsetX: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.rows[0].work_offset;
        }
        case BindKind::WorkOffsetY: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.rows[1].work_offset;
        }
        case BindKind::WorkOffsetZ: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.rows[2].work_offset;
        }
        case BindKind::WorkOffsetA: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.rows[3].work_offset;
        }
        case BindKind::ToolLength: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.active_tool < offsets.tools.size()
                ? static_cast<int32_t>(offsets.tools[offsets.active_tool].length * 1000.0f) : 0;
        }
        case BindKind::ToolRadius: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.active_tool < offsets.tools.size()
                ? static_cast<int32_t>(offsets.tools[offsets.active_tool].radius * 1000.0f) : 0;
        }
        case BindKind::ToolWear: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return offsets.active_tool < offsets.tools.size()
                ? static_cast<int32_t>(offsets.tools[offsets.active_tool].wear * 1000.0f) : 0;
        }
        case BindKind::MacroStep: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            return static_cast<int32_t>(macro.active_step);
        }
        case BindKind::MacroCount: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            return static_cast<int32_t>(macro.count);
        }
        case BindKind::ProbeX: {
            int32_t value = 0;
            return ::machine::g_registry.get_int("probe_result_x", value) ? value : 0;
        }
        case BindKind::ProbeY: {
            int32_t value = 0;
            return ::machine::g_registry.get_int("probe_result_y", value) ? value : 0;
        }
        case BindKind::ProbeZ: {
            int32_t value = 0;
            return ::machine::g_registry.get_int("probe_result_z", value) ? value : 0;
        }
        case BindKind::ProbeDone: {
            bool value = false;
            return ::machine::g_registry.get_bool("probe_cycle_done", value) && value ? 1 : 0;
        }
        case BindKind::ProbeStylus: {
            bool value = false;
            return ::machine::g_registry.get_bool("probe_stylus_ok", value) && value ? 1 : 0;
        }
        case BindKind::ProbeCenterX: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_center_x_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeCenterY: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_center_y_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeSizeX: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_pocket_size_x_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeSizeY: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_pocket_size_y_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeShiftX: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_shift_x_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeShiftY: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_shift_y_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeShiftZ: {
            float value = 0.0f;
            return ::machine::g_registry.get_float("probe_shift_z_mm", value)
                ? static_cast<int32_t>(value * 1000.0f) : 0;
        }
        case BindKind::ProbeSphereReady: {
            bool value = false;
            return ::machine::g_registry.get_bool("probe_sphere_ready", value) && value ? 1 : 0;
        }
        case BindKind::ProbeSpherePoints: {
            int32_t value = 0;
            return ::machine::g_registry.get_int("probe_sphere_point_count", value) ? value : 0;
        }
        case BindKind::AxisXCmd: case BindKind::AxisYCmd:
        case BindKind::AxisZCmd: case BindKind::AxisACmd: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            const int idx = static_cast<int>(bind) - static_cast<int>(BindKind::AxisXCmd);
            return snap.cmd_pos[idx];
        }
        case BindKind::AxisXAct: case BindKind::AxisYAct:
        case BindKind::AxisZAct: case BindKind::AxisAAct: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            const int idx = static_cast<int>(bind) - static_cast<int>(BindKind::AxisXAct);
            return snap.axis_pos[idx];
        }
        case BindKind::AxisXDtg: case BindKind::AxisYDtg:
        case BindKind::AxisZDtg: case BindKind::AxisADtg: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            const int idx = static_cast<int>(bind) - static_cast<int>(BindKind::AxisXDtg);
            return snap.dtg[idx];
        }
        case BindKind::AxisXHomed: case BindKind::AxisYHomed:
        case BindKind::AxisZHomed: case BindKind::AxisAHomed: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            const int idx = static_cast<int>(bind) - static_cast<int>(BindKind::AxisXHomed);
            return snap.axis_homed[idx] ? 1 : 0;
        }
        case BindKind::Wcs: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.wcs_index);
        }
        case BindKind::Units: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.inch_mode ? 1 : 0;
        }
        case BindKind::SpindleRpm: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.spindle_rpm;
        }
        case BindKind::SpindleLoad: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.spindle_load;
        }
        case BindKind::BlockCurrent: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.block_current);
        }
        case BindKind::BlockNext: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.block_next);
        }
        case BindKind::Runtime: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.runtime_ms);
        }
        case BindKind::Parts: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.parts);
        }
        case BindKind::MdiDepth: {
            const auto s = cnc::mdi::g_service.snapshot();
            return static_cast<int32_t>(s.depth);
        }
        case BindKind::SelectedAxis: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.selected_axis);
        }
        case BindKind::JogIncrement: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.jog_increment;
        }
        case BindKind::ActiveTool: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            return static_cast<int32_t>(offsets.active_tool);
        }
        case BindKind::ViewToolpath: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.view_toolpath ? 1 : 0;
        }
        case BindKind::ViewToolpods: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return snap.view_toolpods ? 1 : 0;
        }
        case BindKind::AlarmActiveCount: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            return static_cast<int32_t>(snap.active_count);
        }
        case BindKind::AlarmHistoryCount: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            return static_cast<int32_t>(snap.history_count);
        }
        case BindKind::Alarm0Id: case BindKind::Alarm1Id:
        case BindKind::Alarm2Id: case BindKind::Alarm3Id: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            if (row < 0 || static_cast<size_t>(row) >= snap.active_count) return 0;
            return static_cast<int32_t>(snap.active[row].id);
        }
        case BindKind::Alarm0Time: case BindKind::Alarm1Time:
        case BindKind::Alarm2Time: case BindKind::Alarm3Time: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            if (row < 0 || static_cast<size_t>(row) >= snap.active_count) return 0;
            return static_cast<int32_t>(snap.active[row].timestamp_ns / 1000000000ULL);
        }
        case BindKind::WorkOffset0X: case BindKind::WorkOffset0Y:
        case BindKind::WorkOffset0Z: case BindKind::WorkOffset0A:
        case BindKind::WorkOffset1X: case BindKind::WorkOffset1Y:
        case BindKind::WorkOffset1Z: case BindKind::WorkOffset1A:
        case BindKind::WorkOffset2X: case BindKind::WorkOffset2Y:
        case BindKind::WorkOffset2Z: case BindKind::WorkOffset2A:
        case BindKind::WorkOffset3X: case BindKind::WorkOffset3Y:
        case BindKind::WorkOffset3Z: case BindKind::WorkOffset3A:
        case BindKind::WorkOffset4X: case BindKind::WorkOffset4Y:
        case BindKind::WorkOffset4Z: case BindKind::WorkOffset4A:
        case BindKind::WorkOffset5X: case BindKind::WorkOffset5Y:
        case BindKind::WorkOffset5Z: case BindKind::WorkOffset5A: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            const int delta = static_cast<int>(bind) - static_cast<int>(BindKind::WorkOffset0X);
            const size_t slot = static_cast<size_t>(delta / 4);
            const size_t axis = static_cast<size_t>(delta % 4);
            if (slot >= offsets.work.size()) return 0;
            return static_cast<int32_t>(offsets.work[slot].axis[axis] * 1000.0f);
        }
        case BindKind::Tool0Length: case BindKind::Tool0Radius: case BindKind::Tool0Wear:
        case BindKind::Tool1Length: case BindKind::Tool1Radius: case BindKind::Tool1Wear:
        case BindKind::Tool2Length: case BindKind::Tool2Radius: case BindKind::Tool2Wear:
        case BindKind::Tool3Length: case BindKind::Tool3Radius: case BindKind::Tool3Wear:
        case BindKind::Tool4Length: case BindKind::Tool4Radius: case BindKind::Tool4Wear:
        case BindKind::Tool5Length: case BindKind::Tool5Radius: case BindKind::Tool5Wear:
        case BindKind::Tool6Length: case BindKind::Tool6Radius: case BindKind::Tool6Wear:
        case BindKind::Tool7Length: case BindKind::Tool7Radius: case BindKind::Tool7Wear: {
            const auto offsets = kernel::ui::operator_api::offsets_snapshot();
            const int delta = static_cast<int>(bind) - static_cast<int>(BindKind::Tool0Length);
            const size_t slot = static_cast<size_t>(delta / 3);
            const int field = delta % 3;
            if (slot >= offsets.tools.size()) return 0;
            const float v = field == 0 ? offsets.tools[slot].length :
                            field == 1 ? offsets.tools[slot].radius :
                                         offsets.tools[slot].wear;
            return static_cast<int32_t>(v * 1000.0f);
        }
        case BindKind::AxisXNearLimit: case BindKind::AxisYNearLimit:
        case BindKind::AxisZNearLimit: case BindKind::AxisANearLimit: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            const int idx = static_cast<int>(bind) - static_cast<int>(BindKind::AxisXNearLimit);
            return snap.axis_near_limit[idx] ? 1 : 0;
        }
        case BindKind::OperatorMode: {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            return static_cast<int32_t>(snap.operator_mode);
        }
        case BindKind::Program0Name: case BindKind::Program0Size: case BindKind::Program0Selected: case BindKind::Program0Loaded:
        case BindKind::Program1Name: case BindKind::Program1Size: case BindKind::Program1Selected: case BindKind::Program1Loaded:
        case BindKind::Program2Name: case BindKind::Program2Size: case BindKind::Program2Selected: case BindKind::Program2Loaded:
        case BindKind::Program3Name: case BindKind::Program3Size: case BindKind::Program3Selected: case BindKind::Program3Loaded:
        case BindKind::Program4Name: case BindKind::Program4Size: case BindKind::Program4Selected: case BindKind::Program4Loaded:
        case BindKind::Program5Name: case BindKind::Program5Size: case BindKind::Program5Selected: case BindKind::Program5Loaded:
        case BindKind::Program6Name: case BindKind::Program6Size: case BindKind::Program6Selected: case BindKind::Program6Loaded:
        case BindKind::Program7Name: case BindKind::Program7Size: case BindKind::Program7Selected: case BindKind::Program7Loaded: {
            const auto& ps = kernel::ui::operator_api::program_snapshot();
            const int delta = static_cast<int>(bind) - static_cast<int>(BindKind::Program0Name);
            const size_t slot = static_cast<size_t>(delta / 4);
            const int field = delta % 4;
            if (slot >= ps.programs.size() || slot >= ps.program_count) return 0;
            switch (field) {
                case 0: return 0;  // name surfaced via format_bind_value
                case 1: return static_cast<int32_t>(ps.programs[slot].bytes);
                case 2: return ps.programs[slot].selected ? 1 : 0;
                case 3: return ps.programs[slot].loaded ? 1 : 0;
            }
            return 0;
        }
        case BindKind::HomingAxis: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            return h.selected_axis;
        }
        case BindKind::HomingMethod: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            return h.method_id;
        }
        case BindKind::HomingState: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            return static_cast<int32_t>(h.state);
        }
        case BindKind::HomingFastCps: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            return h.fast_cps;
        }
        case BindKind::HomingSlowCps: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            return h.slow_cps;
        }
        case BindKind::ProbeWizardState: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return static_cast<int32_t>(w.state);
        }
        case BindKind::ProbeWizardCycle: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return static_cast<int32_t>(w.cycle);
        }
        case BindKind::ProbeWizardStep: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.step;
        }
        case BindKind::ProbeWizardTotal: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.total_steps;
        }
        case BindKind::ProbeWizardResultX: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.result_x;
        }
        case BindKind::ProbeWizardResultY: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.result_y;
        }
        case BindKind::ProbeWizardResultZ: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.result_z;
        }
        case BindKind::ProbeWizardResultValid: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            return w.result_valid ? 1 : 0;
        }
        case BindKind::CalPecAxis: {
            return kernel::ui::operator_api::calibration_snapshot().pec_axis;
        }
        case BindKind::CalPecEnabled: {
            return kernel::ui::operator_api::calibration_snapshot().pec_enabled ? 1 : 0;
        }
        case BindKind::CalPecCount: {
            return static_cast<int32_t>(
                kernel::ui::operator_api::calibration_snapshot().pec_point_count);
        }
        case BindKind::CalPecPendingPos:
            return kernel::ui::operator_api::cal_pec_pending_pos();
        case BindKind::CalPecPendingErr:
            return kernel::ui::operator_api::cal_pec_pending_err();
        case BindKind::CalRotaryOffset:
            return kernel::ui::operator_api::calibration_snapshot().rotary_offset_a;
        case BindKind::CalPec0Pos: case BindKind::CalPec1Pos:
        case BindKind::CalPec2Pos: case BindKind::CalPec3Pos:
        case BindKind::CalPec4Pos: case BindKind::CalPec5Pos:
        case BindKind::CalPec6Pos: case BindKind::CalPec7Pos: {
            const auto snap = kernel::ui::operator_api::calibration_snapshot();
            const size_t row = static_cast<size_t>(
                static_cast<int>(bind) - static_cast<int>(BindKind::CalPec0Pos)) / 2;
            return row < snap.kMaxPecRows ? snap.pec_pos[row] : 0;
        }
        case BindKind::CalPec0Err: case BindKind::CalPec1Err:
        case BindKind::CalPec2Err: case BindKind::CalPec3Err:
        case BindKind::CalPec4Err: case BindKind::CalPec5Err:
        case BindKind::CalPec6Err: case BindKind::CalPec7Err: {
            const auto snap = kernel::ui::operator_api::calibration_snapshot();
            const size_t row = static_cast<size_t>(
                static_cast<int>(bind) - static_cast<int>(BindKind::CalPec0Err)) / 2;
            return row < snap.kMaxPecRows ? snap.pec_err[row] : 0;
        }
        case BindKind::CalGeomXy:
            return kernel::ui::operator_api::calibration_snapshot().geom_xy_urad;
        case BindKind::CalGeomXz:
            return kernel::ui::operator_api::calibration_snapshot().geom_xz_urad;
        case BindKind::CalGeomYz:
            return kernel::ui::operator_api::calibration_snapshot().geom_yz_urad;
        case BindKind::CalGeomEnabled:
            return kernel::ui::operator_api::calibration_snapshot().geom_enabled ? 1 : 0;
        case BindKind::CalSphereEnabled:
            return kernel::ui::operator_api::calibration_snapshot().sphere_enabled ? 1 : 0;
        case BindKind::CalSphereDiameter:
            // 1000-scaled mm so format_bind_value's /1000.0f path renders it.
            return static_cast<int32_t>(
                kernel::ui::operator_api::calibration_snapshot().sphere_diameter_mm * 1000.0f);
        case BindKind::CalSphereProbeUm:
            return kernel::ui::operator_api::calibration_snapshot().sphere_probe_radius_um;
        case BindKind::CalSpherePoints:
            return static_cast<int32_t>(
                kernel::ui::operator_api::calibration_snapshot().sphere_point_count);
        case BindKind::CalSphereComputed:
            return kernel::ui::operator_api::calibration_snapshot().sphere_errors_computed ? 1 : 0;
        case BindKind::CalSphereErrPosX:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_pos_x_urad;
        case BindKind::CalSphereErrPosY:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_pos_y_urad;
        case BindKind::CalSphereErrPosZ:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_pos_z_urad;
        case BindKind::CalSphereErrSqXy:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_sq_xy_urad;
        case BindKind::CalSphereErrSqXz:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_sq_xz_urad;
        case BindKind::CalSphereErrSqYz:
            return kernel::ui::operator_api::calibration_snapshot().sphere_err_sq_yz_urad;
        case BindKind::NetIp:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().local_ip);
        case BindKind::NetGateway:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().gateway);
        case BindKind::NetPendingIp:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().pending_ip);
        case BindKind::NetPendingGateway:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().pending_gateway);
        case BindKind::NetPendingPingTarget:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().pending_ping_target);
        case BindKind::NetLastPingTarget:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().last_ping_target);
        case BindKind::NetLastPingRtt:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().last_ping_rtt_ms);
        case BindKind::NetDhcpState:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().dhcp_state);
        case BindKind::NetLinkState:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().link_state);
        case BindKind::NetLastPingResult:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().last_ping_result);
        case BindKind::NetUptime:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().uptime_s);
        case BindKind::NetRxRequests:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().rx_requests);
        case BindKind::NetTxResponses:
            return static_cast<int32_t>(kernel::ui::operator_api::network_snapshot().tx_responses);
        // Per-axis status sub-page — selected_axis is the implicit input.
        case BindKind::AxisDetailDrive:
            return kernel::ui::operator_api::axis_status_snapshot().drive_state;
        case BindKind::AxisDetailTrajState:
            return kernel::ui::operator_api::axis_status_snapshot().traj_state;
        case BindKind::AxisDetailMode:
            return kernel::ui::operator_api::axis_status_snapshot().mode;
        case BindKind::AxisDetailEnabled:
            return kernel::ui::operator_api::axis_status_snapshot().enabled ? 1 : 0;
        case BindKind::AxisDetailFault:
            return kernel::ui::operator_api::axis_status_snapshot().fault_latched ? 1 : 0;
        case BindKind::AxisDetailHomed:
            return kernel::ui::operator_api::axis_status_snapshot().homed ? 1 : 0;
        case BindKind::AxisDetailErrorCode:
            return kernel::ui::operator_api::axis_status_snapshot().last_error_code;
        case BindKind::AxisDetailFollowingErr:
            return kernel::ui::operator_api::axis_status_snapshot().following_error;
        case BindKind::AxisDetailMaxFollowingErr:
            return kernel::ui::operator_api::axis_status_snapshot().max_following_error;
        case BindKind::AxisDetailVmax:
            return kernel::ui::operator_api::axis_status_snapshot().vmax_cps;
        case BindKind::AxisDetailAccel:
            return kernel::ui::operator_api::axis_status_snapshot().accel_cps2;
        case BindKind::AxisDetailJerk:
            return kernel::ui::operator_api::axis_status_snapshot().jerk_cps3;
        case BindKind::AxisDetailCmd:
            return kernel::ui::operator_api::axis_status_snapshot().cmd_pos;
        case BindKind::AxisDetailAct:
            return kernel::ui::operator_api::axis_status_snapshot().actual_pos;
        case BindKind::AxisDetailStatusWord:
            return kernel::ui::operator_api::axis_status_snapshot().status_word;
        case BindKind::AxisDetailControlWord:
            return kernel::ui::operator_api::axis_status_snapshot().control_word;
        // Tool change wizard.
        case BindKind::ToolChangeCurrent:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().current_tool);
        case BindKind::ToolChangeTarget:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().target_tool);
        case BindKind::ToolChangeState:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().state);
        case BindKind::ToolChangeCurrentStation:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().current_station);
        case BindKind::ToolChangeTargetStation:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().target_station);
        case BindKind::ToolChangeStep:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().step);
        case BindKind::ToolChangeTotalSteps:
            return static_cast<int32_t>(kernel::ui::operator_api::tool_change_snapshot().total_steps);
        default: return 0;
    }
}

void format_bind_value(BindKind bind, char* buf, size_t buf_size, const char* prefix) {
    if (!buf || buf_size == 0) return;
    const auto offsets = kernel::ui::operator_api::offsets_snapshot();
    switch (bind) {
        case BindKind::ProgramName:
            copy_field(buf, buf_size, kernel::ui::operator_api::selected_program_name(),
                       strlen(kernel::ui::operator_api::selected_program_name()));
            return;
        case BindKind::WorkName:
            copy_field(buf, buf_size, offsets.workset_name, strlen(offsets.workset_name));
            return;
        case BindKind::ToolName:
            copy_field(buf, buf_size, offsets.tool_name, strlen(offsets.tool_name));
            return;
        case BindKind::MacroName: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            copy_field(buf, buf_size, macro.selected_name, strlen(macro.selected_name));
            return;
        }
        case BindKind::MacroActive: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            copy_field(buf, buf_size, macro.active_name, strlen(macro.active_name));
            return;
        }
        case BindKind::MacroStatus: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            copy_field(buf, buf_size, macro.status, strlen(macro.status));
            return;
        }
        case BindKind::MacroMessage: {
            const auto macro = kernel::ui::operator_api::macro_snapshot();
            copy_field(buf, buf_size, macro.message, strlen(macro.message));
            return;
        }
        case BindKind::ProbeDone: {
            const char* text = bound_numeric_value(bind) != 0 ? "DONE" : "IDLE";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ProbeStylus: {
            const char* text = bound_numeric_value(bind) != 0 ? "QUALIFIED" : "UNQUALIFIED";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ProbeSphereReady: {
            const char* text = bound_numeric_value(bind) != 0 ? "READY" : "IDLE";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::WorkOffsetX:
        case BindKind::WorkOffsetY:
        case BindKind::WorkOffsetZ:
        case BindKind::WorkOffsetA:
        case BindKind::ToolLength:
        case BindKind::ToolRadius:
        case BindKind::ToolWear:
        case BindKind::ProbeCenterX:
        case BindKind::ProbeCenterY:
        case BindKind::ProbeSizeX:
        case BindKind::ProbeSizeY:
        case BindKind::ProbeShiftX:
        case BindKind::ProbeShiftY:
        case BindKind::ProbeShiftZ:
        case BindKind::WorkOffset0X: case BindKind::WorkOffset0Y:
        case BindKind::WorkOffset0Z: case BindKind::WorkOffset0A:
        case BindKind::WorkOffset1X: case BindKind::WorkOffset1Y:
        case BindKind::WorkOffset1Z: case BindKind::WorkOffset1A:
        case BindKind::WorkOffset2X: case BindKind::WorkOffset2Y:
        case BindKind::WorkOffset2Z: case BindKind::WorkOffset2A:
        case BindKind::WorkOffset3X: case BindKind::WorkOffset3Y:
        case BindKind::WorkOffset3Z: case BindKind::WorkOffset3A:
        case BindKind::WorkOffset4X: case BindKind::WorkOffset4Y:
        case BindKind::WorkOffset4Z: case BindKind::WorkOffset4A:
        case BindKind::WorkOffset5X: case BindKind::WorkOffset5Y:
        case BindKind::WorkOffset5Z: case BindKind::WorkOffset5A:
        case BindKind::Tool0Length: case BindKind::Tool0Radius: case BindKind::Tool0Wear:
        case BindKind::Tool1Length: case BindKind::Tool1Radius: case BindKind::Tool1Wear:
        case BindKind::Tool2Length: case BindKind::Tool2Radius: case BindKind::Tool2Wear:
        case BindKind::Tool3Length: case BindKind::Tool3Radius: case BindKind::Tool3Wear:
        case BindKind::Tool4Length: case BindKind::Tool4Radius: case BindKind::Tool4Wear:
        case BindKind::Tool5Length: case BindKind::Tool5Radius: case BindKind::Tool5Wear:
        case BindKind::Tool6Length: case BindKind::Tool6Radius: case BindKind::Tool6Wear:
        case BindKind::Tool7Length: case BindKind::Tool7Radius: case BindKind::Tool7Wear: {
            const float value = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%.3f", prefix ? prefix : "", static_cast<double>(value));
            return;
        }
        case BindKind::OperatorMode: {
            const int32_t v = bound_numeric_value(bind);
            const char* text = v == 0 ? "AUTO" : v == 1 ? "MDI" : v == 2 ? "JOG" : v == 3 ? "SETUP" : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisX: case BindKind::AxisY: case BindKind::AxisZ: case BindKind::AxisA:
        case BindKind::AxisXCmd: case BindKind::AxisYCmd:
        case BindKind::AxisZCmd: case BindKind::AxisACmd:
        case BindKind::AxisXAct: case BindKind::AxisYAct:
        case BindKind::AxisZAct: case BindKind::AxisAAct:
        case BindKind::AxisXDtg: case BindKind::AxisYDtg:
        case BindKind::AxisZDtg: case BindKind::AxisADtg: {
            // Axes are counts; treat 1000 counts = 1 mm for display.
            const float value = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%+9.3f", prefix ? prefix : "",
                                     static_cast<double>(value));
            return;
        }
        case BindKind::AxisXHomed: case BindKind::AxisYHomed:
        case BindKind::AxisZHomed: case BindKind::AxisAHomed: {
            const char* text = bound_numeric_value(bind) ? "HOMED" : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Wcs: {
            const char* names[6] = {"G54", "G55", "G56", "G57", "G58", "G59"};
            const int32_t idx = bound_numeric_value(bind);
            const char* name = (idx >= 0 && idx < 6) ? names[idx] : "G54";
            copy_field(buf, buf_size, name, strlen(name));
            return;
        }
        case BindKind::Units: {
            const char* text = bound_numeric_value(bind) ? "inch" : "mm";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Runtime: {
            const uint32_t total_s = static_cast<uint32_t>(bound_numeric_value(bind)) / 1000u;
            kernel::util::k_snprintf(buf, buf_size, "%s%02u:%02u:%02u", prefix ? prefix : "",
                                     (total_s / 3600u) % 100u, (total_s / 60u) % 60u, total_s % 60u);
            return;
        }
        case BindKind::MdiInput: {
            const auto s = cnc::mdi::g_service.snapshot();
            copy_field(buf, buf_size, s.input, strlen(s.input));
            return;
        }
        case BindKind::MdiLast: {
            const auto s = cnc::mdi::g_service.snapshot();
            copy_field(buf, buf_size, s.last, strlen(s.last));
            return;
        }
        case BindKind::MdiStatus: {
            const auto s = cnc::mdi::g_service.snapshot();
            const char* text = "IDLE";
            switch (s.status) {
                case cnc::mdi::Status::Idle: text = "IDLE"; break;
                case cnc::mdi::Status::Queued: text = "QUEUED"; break;
                case cnc::mdi::Status::Running: text = "RUN"; break;
                case cnc::mdi::Status::Error: text = "ERROR"; break;
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::MdiMessage: {
            const auto s = cnc::mdi::g_service.snapshot();
            copy_field(buf, buf_size, s.message, strlen(s.message));
            return;
        }
        case BindKind::SelectedAxis: {
            static const char* kNames[4] = {"X", "Y", "Z", "A"};
            const int32_t v = bound_numeric_value(bind);
            const char* name = (v >= 0 && v < 4) ? kNames[v] : "?";
            copy_field(buf, buf_size, name, strlen(name));
            return;
        }
        case BindKind::JogIncrement: {
            const float mm = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%.3f mm", prefix ? prefix : "",
                                     static_cast<double>(mm));
            return;
        }
        case BindKind::EcState: {
            const auto snap = kernel::ui::operator_api::ethercat_snapshot();
            const char* text =
                !snap.available                              ? "OFFLINE" :
                snap.master_state == 1                       ? "INIT"    :
                snap.master_state == 2                       ? "PRE-OP"  :
                snap.master_state == 4                       ? "SAFE-OP" :
                snap.master_state == 8                       ? "OP"      :
                snap.master_state == 0x10                    ? "FAULT"   : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::EcFault: {
            const auto snap = kernel::ui::operator_api::ethercat_snapshot();
            const char* text = snap.deadline_fault ? "LATCHED" : "ok";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::EcDcFault: {
            const auto snap = kernel::ui::operator_api::ethercat_snapshot();
            const char* text = snap.dc_sync_faulted ? "DC FAULT" : "ok";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::RestartPending: {
            const bool p = kernel::ui::operator_api::restart_review_pending();
            const char* text = p ? "STAGED" : "ok";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::RestartLine: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            kernel::util::k_snprintf(buf, buf_size, "%s%lu", prefix ? prefix : "",
                                     static_cast<unsigned long>(r.target_line));
            return;
        }
        case BindKind::RestartTool: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            kernel::util::k_snprintf(buf, buf_size, "%sT%lu%s", prefix ? prefix : "",
                                     static_cast<unsigned long>(r.active_tool + 1),
                                     r.tool_length_active ? " (H)" : "");
            return;
        }
        case BindKind::RestartWcs: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            const char* names[] = {"G54", "G55", "G56", "G57", "G58", "G59"};
            const char* text = r.active_work < 6 ? names[r.active_work] : "G54";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::RestartFeed: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            kernel::util::k_snprintf(buf, buf_size, "%sF%lu", prefix ? prefix : "",
                                     static_cast<unsigned long>(r.feed));
            return;
        }
        case BindKind::RestartSpindle: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            kernel::util::k_snprintf(buf, buf_size, "%sS%ld", prefix ? prefix : "",
                                     static_cast<long>(r.spindle));
            return;
        }
        case BindKind::RestartCoolant: {
            const auto r = kernel::ui::operator_api::restart_review_snapshot();
            const char* text = r.coolant_flood ? (r.coolant_mist ? "FLOOD+MIST" : "FLOOD")
                              : (r.coolant_mist ? "MIST" : "OFF");
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Channel0BarrierMs:
        case BindKind::Channel1BarrierMs: {
            const auto& prog = kernel::ui::operator_api::program_snapshot();
            const auto ec = kernel::ui::operator_api::ethercat_snapshot();
            const size_t ch = (bind == BindKind::Channel0BarrierMs) ? 0u : 1u;
            const uint32_t cycles = (ch < prog.channels.size())
                ? prog.channels[ch].barrier_cycles_remaining : 0u;
            const uint32_t period_us = ec.period_us ? ec.period_us : 250u;
            const uint64_t ms = (static_cast<uint64_t>(cycles) * period_us) / 1000u;
            if (cycles == 0) {
                copy_field(buf, buf_size, "-", 1);
                return;
            }
            kernel::util::k_snprintf(buf, buf_size, "%s%llu ms",
                                     prefix ? prefix : "",
                                     static_cast<unsigned long long>(ms));
            return;
        }
        case BindKind::Alarm0Severity: case BindKind::Alarm1Severity:
        case BindKind::Alarm2Severity: case BindKind::Alarm3Severity: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            const char* text = "---";
            if (row >= 0 && static_cast<size_t>(row) < snap.active_count) {
                text = alarm_severity_text(snap.active[row].severity);
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Alarm0Axis: case BindKind::Alarm1Axis:
        case BindKind::Alarm2Axis: case BindKind::Alarm3Axis: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            const char* text = "-";
            if (row >= 0 && static_cast<size_t>(row) < snap.active_count && snap.active[row].axis) {
                text = snap.active[row].axis;
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Alarm0Message: case BindKind::Alarm1Message:
        case BindKind::Alarm2Message: case BindKind::Alarm3Message: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            const char* text = "";
            if (row >= 0 && static_cast<size_t>(row) < snap.active_count && snap.active[row].message) {
                text = snap.active[row].message;
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Alarm0Id: case BindKind::Alarm1Id:
        case BindKind::Alarm2Id: case BindKind::Alarm3Id: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            if (row < 0 || static_cast<size_t>(row) >= snap.active_count) {
                copy_field(buf, buf_size, "-", 1);
                return;
            }
            kernel::util::k_snprintf(buf, buf_size, "%s%lu", prefix ? prefix : "",
                                     static_cast<unsigned long>(snap.active[row].id));
            return;
        }
        case BindKind::Alarm0Time: case BindKind::Alarm1Time:
        case BindKind::Alarm2Time: case BindKind::Alarm3Time: {
            const auto snap = kernel::ui::operator_api::alarms_snapshot();
            const int row = alarm_row_index(bind);
            if (row < 0 || static_cast<size_t>(row) >= snap.active_count) {
                copy_field(buf, buf_size, "-", 1);
                return;
            }
            // Best-effort: render as "Xs" using seconds since epoch from
            // the snapshot. No wall clock available for true relative.
            const uint32_t s = static_cast<uint32_t>(snap.active[row].timestamp_ns / 1000000000ULL);
            kernel::util::k_snprintf(buf, buf_size, "%s%us", prefix ? prefix : "", s);
            return;
        }
        case BindKind::EcSlaves:
        case BindKind::EcMiss:
        case BindKind::EcTrips:
        case BindKind::EcCycleP99:
        case BindKind::EcCycleMax:
        case BindKind::EcPeriod:
        case BindKind::EcCycles:
        case BindKind::EcTxFrames:
        case BindKind::EcRxFrames:
        case BindKind::EcDcDriftMaxNs:
        case BindKind::EcDcDriftLastNs:
        case BindKind::EcDcSamples:
        case BindKind::EcDcTrips: {
            const auto snap = kernel::ui::operator_api::ethercat_snapshot();
            unsigned long long v = 0;
            switch (bind) {
                case BindKind::EcSlaves:    v = (unsigned long long)snap.slave_count;    break;
                case BindKind::EcMiss:      v = snap.deadline_miss;                       break;
                case BindKind::EcTrips:     v = snap.deadline_trips;                      break;
                case BindKind::EcCycleP99:  v = snap.cycle_p99_us;                        break;
                case BindKind::EcCycleMax:  v = snap.cycle_max_us;                        break;
                case BindKind::EcPeriod:    v = snap.period_us;                           break;
                case BindKind::EcCycles:    v = snap.cycles;                              break;
                case BindKind::EcTxFrames:  v = snap.tx_frames;                           break;
                case BindKind::EcRxFrames:  v = snap.rx_frames;                           break;
                case BindKind::EcDcDriftMaxNs: v = snap.dc_drift_max_ns;                  break;
                case BindKind::EcDcDriftLastNs: {
                    const int64_t s = snap.last_dc_drift_ns;
                    v = (unsigned long long)(s < 0 ? -s : s);
                    break;
                }
                case BindKind::EcDcSamples: v = snap.dc_sync_samples;                     break;
                case BindKind::EcDcTrips:   v = snap.dc_sync_trips;                       break;
                default: break;
            }
            kernel::util::k_snprintf(buf, buf_size, "%s%llu", prefix ? prefix : "", v);
            return;
        }
        case BindKind::Program0Name: case BindKind::Program1Name:
        case BindKind::Program2Name: case BindKind::Program3Name:
        case BindKind::Program4Name: case BindKind::Program5Name:
        case BindKind::Program6Name: case BindKind::Program7Name: {
            const auto& ps = kernel::ui::operator_api::program_snapshot();
            const size_t slot = static_cast<size_t>(
                (static_cast<int>(bind) - static_cast<int>(BindKind::Program0Name)) / 4);
            const char* text = (slot < ps.programs.size() && slot < ps.program_count &&
                                ps.programs[slot].name) ? ps.programs[slot].name : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::Program0Size: case BindKind::Program1Size:
        case BindKind::Program2Size: case BindKind::Program3Size:
        case BindKind::Program4Size: case BindKind::Program5Size:
        case BindKind::Program6Size: case BindKind::Program7Size: {
            const int32_t bytes = bound_numeric_value(bind);
            if (bytes <= 0) {
                copy_field(buf, buf_size, "---", 3);
                return;
            }
            if (bytes < 1024) {
                kernel::util::k_snprintf(buf, buf_size, "%s%ld B", prefix ? prefix : "",
                                         static_cast<long>(bytes));
            } else {
                const float kb = static_cast<float>(bytes) / 1024.0f;
                kernel::util::k_snprintf(buf, buf_size, "%s%.1f kB", prefix ? prefix : "",
                                         static_cast<double>(kb));
            }
            return;
        }
        case BindKind::Program0Selected: case BindKind::Program1Selected:
        case BindKind::Program2Selected: case BindKind::Program3Selected:
        case BindKind::Program4Selected: case BindKind::Program5Selected:
        case BindKind::Program6Selected: case BindKind::Program7Selected:
        case BindKind::Program0Loaded: case BindKind::Program1Loaded:
        case BindKind::Program2Loaded: case BindKind::Program3Loaded:
        case BindKind::Program4Loaded: case BindKind::Program5Loaded:
        case BindKind::Program6Loaded: case BindKind::Program7Loaded: {
            const char* text = bound_numeric_value(bind) ? "*" : "";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::HomingMessage: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            copy_field(buf, buf_size, h.status_message ? h.status_message : "",
                       h.status_message ? strlen(h.status_message) : 0);
            return;
        }
        case BindKind::HomingAxis: {
            static const char* kNames[4] = {"X", "Y", "Z", "A"};
            const int32_t v = bound_numeric_value(bind);
            const char* name = (v >= 0 && v < 4) ? kNames[v] : "?";
            copy_field(buf, buf_size, name, strlen(name));
            return;
        }
        case BindKind::HomingState: {
            const int32_t v = bound_numeric_value(bind);
            const char* names[6] = {"IDLE", "READY", "SEARCH", "APPROACH", "DONE", "FAULT"};
            const char* text = (v >= 0 && v < 6) ? names[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::HomingMethod: {
            const auto h = kernel::ui::operator_api::homing_snapshot();
            copy_field(buf, buf_size, h.method_name ? h.method_name : "(none)",
                       h.method_name ? strlen(h.method_name) : 6);
            return;
        }
        case BindKind::HomingFastCps:
        case BindKind::HomingSlowCps: {
            kernel::util::k_snprintf(buf, buf_size, "%s%ld cps", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        case BindKind::ProbeWizardState: {
            const int32_t v = bound_numeric_value(bind);
            const char* names[6] = {"IDLE", "SELECT", "CONFIRM", "RUNNING", "INSPECT", "FAULT"};
            const char* text = (v >= 0 && v < 6) ? names[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ProbeWizardCycle: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            copy_field(buf, buf_size, w.cycle_name ? w.cycle_name : "(none)",
                       w.cycle_name ? strlen(w.cycle_name) : 6);
            return;
        }
        case BindKind::ProbeWizardMessage: {
            const auto w = kernel::ui::operator_api::probe_wizard_snapshot();
            copy_field(buf, buf_size, w.status_message ? w.status_message : "",
                       w.status_message ? strlen(w.status_message) : 0);
            return;
        }
        case BindKind::ProbeWizardResultValid: {
            const int32_t v = bound_numeric_value(bind);
            const char* text = v ? "VALID" : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ProbeWizardStep:
        case BindKind::ProbeWizardTotal:
        case BindKind::ProbeWizardResultX:
        case BindKind::ProbeWizardResultY:
        case BindKind::ProbeWizardResultZ:
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        case BindKind::CalPecAxis: {
            static const char* kNames[4] = {"X", "Y", "Z", "A"};
            const int32_t v = bound_numeric_value(bind);
            const char* name = (v >= 0 && v < 4) ? kNames[v] : "?";
            copy_field(buf, buf_size, name, strlen(name));
            return;
        }
        case BindKind::CalPecEnabled:
        case BindKind::CalGeomEnabled:
        case BindKind::CalSphereEnabled: {
            const char* text = bound_numeric_value(bind) ? "ON" : "OFF";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::CalSphereComputed: {
            const char* text = bound_numeric_value(bind) ? "READY" : "PENDING";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::CalSphereDiameter: {
            const float mm = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%.2f mm", prefix ? prefix : "",
                                     static_cast<double>(mm));
            return;
        }
        case BindKind::CalSphereProbeUm: {
            kernel::util::k_snprintf(buf, buf_size, "%s%ld um", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        case BindKind::CalGeomXy: case BindKind::CalGeomXz: case BindKind::CalGeomYz:
        case BindKind::CalSphereErrPosX: case BindKind::CalSphereErrPosY: case BindKind::CalSphereErrPosZ:
        case BindKind::CalSphereErrSqXy: case BindKind::CalSphereErrSqXz: case BindKind::CalSphereErrSqYz: {
            kernel::util::k_snprintf(buf, buf_size, "%s%+ld urad", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        case BindKind::CalRotaryOffset: {
            kernel::util::k_snprintf(buf, buf_size, "%s%+ld cnt", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        case BindKind::CalPec0Pos: case BindKind::CalPec1Pos:
        case BindKind::CalPec2Pos: case BindKind::CalPec3Pos:
        case BindKind::CalPec4Pos: case BindKind::CalPec5Pos:
        case BindKind::CalPec6Pos: case BindKind::CalPec7Pos:
        case BindKind::CalPec0Err: case BindKind::CalPec1Err:
        case BindKind::CalPec2Err: case BindKind::CalPec3Err:
        case BindKind::CalPec4Err: case BindKind::CalPec5Err:
        case BindKind::CalPec6Err: case BindKind::CalPec7Err:
        case BindKind::CalPecPendingPos:
        case BindKind::CalPecPendingErr:
        case BindKind::CalPecCount: {
            // Hide rows that haven't been populated by surfacing "---" so
            // the table doesn't render eight zero rows on a clear axis.
            const auto snap = kernel::ui::operator_api::calibration_snapshot();
            const bool is_row =
                bind >= BindKind::CalPec0Pos && bind <= BindKind::CalPec7Err;
            if (is_row) {
                const size_t row = (bind >= BindKind::CalPec0Err)
                    ? static_cast<size_t>(static_cast<int>(bind) - static_cast<int>(BindKind::CalPec0Err)) / 2
                    : static_cast<size_t>(static_cast<int>(bind) - static_cast<int>(BindKind::CalPec0Pos)) / 2;
                if (row >= snap.pec_point_count) {
                    copy_field(buf, buf_size, "---", 3);
                    return;
                }
            }
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        case BindKind::CalSpherePoints: {
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        // Network setup page formatters. IPs are stored host-order in the
        // snapshot; format big-endian-style for the operator (a.b.c.d). The
        // hmi service stores IPs in host order so byte 0 = MSB.
        case BindKind::NetIp:
        case BindKind::NetGateway:
        case BindKind::NetPendingIp:
        case BindKind::NetPendingGateway:
        case BindKind::NetPendingPingTarget:
        case BindKind::NetLastPingTarget: {
            const uint32_t v = static_cast<uint32_t>(bound_numeric_value(bind));
            if (v == 0) { copy_field(buf, buf_size, "---", 3); return; }
            kernel::util::k_snprintf(buf, buf_size, "%s%u.%u.%u.%u", prefix ? prefix : "",
                                     (v >> 24) & 0xFFu, (v >> 16) & 0xFFu,
                                     (v >> 8) & 0xFFu, v & 0xFFu);
            return;
        }
        case BindKind::NetMac: {
            const auto snap = kernel::ui::operator_api::network_snapshot();
            kernel::util::k_snprintf(buf, buf_size, "%s%02X:%02X:%02X:%02X:%02X:%02X",
                                     prefix ? prefix : "",
                                     snap.mac[0], snap.mac[1], snap.mac[2],
                                     snap.mac[3], snap.mac[4], snap.mac[5]);
            return;
        }
        case BindKind::NetDhcpState: {
            using DS = kernel::ui::operator_api::NetworkSnapshot::DhcpState;
            const auto snap = kernel::ui::operator_api::network_snapshot();
            const char* text = "?";
            switch (snap.dhcp_state) {
                case DS::Idle: text = "IDLE"; break;
                case DS::Discovering: text = "DISCOVERING"; break;
                case DS::Bound: text = "BOUND"; break;
                case DS::Timeout: text = "TIMEOUT"; break;
                case DS::Static: text = "STATIC"; break;
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::NetLinkState: {
            using LS = kernel::ui::operator_api::NetworkSnapshot::LinkState;
            const auto snap = kernel::ui::operator_api::network_snapshot();
            const char* text = snap.link_state == LS::Up ? "UP" :
                               snap.link_state == LS::Probing ? "PROBING" : "DOWN";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::NetLastPingResult: {
            using PR = kernel::ui::operator_api::NetworkSnapshot::PingResultKind;
            const auto snap = kernel::ui::operator_api::network_snapshot();
            const char* text = "---";
            switch (snap.last_ping_result) {
                case PR::None: text = "---"; break;
                case PR::Ok: text = "OK"; break;
                case PR::Busy: text = "BUSY"; break;
                case PR::BadAddress: text = "BAD ADDR"; break;
                case PR::Timeout: text = "TIMEOUT"; break;
                case PR::SendFailed: text = "SEND FAIL"; break;
            }
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::NetLastPingRtt: {
            const int32_t v = bound_numeric_value(bind);
            if (v <= 0) { copy_field(buf, buf_size, "---", 3); return; }
            kernel::util::k_snprintf(buf, buf_size, "%s%ld ms", prefix ? prefix : "", static_cast<long>(v));
            return;
        }
        case BindKind::NetUptime: {
            const uint32_t s = static_cast<uint32_t>(bound_numeric_value(bind));
            kernel::util::k_snprintf(buf, buf_size, "%s%02u:%02u:%02u", prefix ? prefix : "",
                                     (s / 3600u) % 100u, (s / 60u) % 60u, s % 60u);
            return;
        }
        case BindKind::NetNicName: {
            const auto snap = kernel::ui::operator_api::network_snapshot();
            copy_field(buf, buf_size, snap.nic_name, strlen(snap.nic_name));
            return;
        }
        case BindKind::NetRxRequests:
        case BindKind::NetTxResponses: {
            kernel::util::k_snprintf(buf, buf_size, "%s%lld", prefix ? prefix : "",
                                     static_cast<long long>(static_cast<uint32_t>(bound_numeric_value(bind))));
            return;
        }
        // Per-axis status sub-page formatters.
        case BindKind::AxisDetailDrive: {
            // Names follow motion::DriveState ordering (NotReady..Fault).
            static const char* kNames[] = {"NOT READY", "DISABLED", "READY", "ON",
                                           "ENABLED", "QSTOP", "FAULT REACT", "FAULT"};
            const int32_t v = bound_numeric_value(bind);
            const char* text = (v >= 0 && v < 8) ? kNames[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailTrajState: {
            // motion::TrajState ordering: Idle, MoveReady, Accel, ConstantVel, Decel, Holding.
            static const char* kNames[] = {"IDLE", "READY", "ACCEL", "CRUISE", "DECEL", "HOLDING"};
            const int32_t v = bound_numeric_value(bind);
            const char* text = (v >= 0 && v < 6) ? kNames[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailMode: {
            // motion::Mode ordering: None, CSP, CSV, CST, PP, PV, TQ, Homing.
            static const char* kNames[] = {"---", "CSP", "CSV", "CST", "PP", "PV", "TQ", "HM"};
            const int32_t v = bound_numeric_value(bind);
            const char* text = (v >= 0 && v < 8) ? kNames[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailEnabled: {
            const char* text = bound_numeric_value(bind) ? "ENABLED" : "DISABLED";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailFault: {
            const char* text = bound_numeric_value(bind) ? "LATCHED" : "ok";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailHomed: {
            const char* text = bound_numeric_value(bind) ? "HOMED" : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::AxisDetailErrorCode:
        case BindKind::AxisDetailStatusWord:
        case BindKind::AxisDetailControlWord: {
            kernel::util::k_snprintf(buf, buf_size, "%s0x%04X", prefix ? prefix : "",
                                     static_cast<unsigned>(bound_numeric_value(bind)) & 0xFFFFu);
            return;
        }
        case BindKind::AxisDetailFollowingErr:
        case BindKind::AxisDetailMaxFollowingErr:
        case BindKind::AxisDetailCmd:
        case BindKind::AxisDetailAct: {
            // Counts → mm via the same 1000 counts/mm convention as the DRO.
            const float value = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%+9.3f", prefix ? prefix : "",
                                     static_cast<double>(value));
            return;
        }
        case BindKind::AxisDetailVmax:
        case BindKind::AxisDetailAccel:
        case BindKind::AxisDetailJerk: {
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        // Tool change wizard formatters. The state labels are deliberately
        // *not* "RELEASING / PICKING / VERIFYING" — the wizard does not
        // drive a physical changer (see operator_api.hpp:ToolChangeSnapshot
        // and machine::toolpods::Service). It's a confirmation flow over a
        // manual swap: operator triggers, swaps tool by hand, accepts.
        // Naming the states after physical motions misleads operators into
        // thinking the spindle moved when it did not.
        case BindKind::ToolChangeState: {
            using TS = kernel::ui::operator_api::ToolChangeSnapshot::State;
            const int32_t v = bound_numeric_value(bind);
            const char* names[] = {"IDLE", "REQUESTED", "AWAIT SWAP", "AWAIT SWAP",
                                   "AWAIT SWAP", "DONE", "FAULTED"};
            const char* text = (v >= 0 && v <= static_cast<int32_t>(TS::Faulted)) ? names[v] : "?";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ToolChangeMessage: {
            const auto snap = kernel::ui::operator_api::tool_change_snapshot();
            copy_field(buf, buf_size, snap.status_message, strlen(snap.status_message));
            return;
        }
        case BindKind::ToolChangeCurrent:
        case BindKind::ToolChangeTarget: {
            const int32_t v = bound_numeric_value(bind);
            if (v <= 0) { copy_field(buf, buf_size, "---", 3); return; }
            kernel::util::k_snprintf(buf, buf_size, "%sT%ld", prefix ? prefix : "",
                                     static_cast<long>(v));
            return;
        }
        case BindKind::ToolChangeCurrentPod: {
            const auto snap = kernel::ui::operator_api::tool_change_snapshot();
            const char* text = snap.current_pod[0] ? snap.current_pod : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ToolChangeCurrentLabel: {
            const auto snap = kernel::ui::operator_api::tool_change_snapshot();
            const char* text = snap.current_label[0] ? snap.current_label : "";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ToolChangeTargetPod: {
            const auto snap = kernel::ui::operator_api::tool_change_snapshot();
            const char* text = snap.target_pod[0] ? snap.target_pod : "---";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ToolChangeTargetLabel: {
            const auto snap = kernel::ui::operator_api::tool_change_snapshot();
            const char* text = snap.target_label[0] ? snap.target_label : "";
            copy_field(buf, buf_size, text, strlen(text));
            return;
        }
        case BindKind::ToolChangeCurrentStation:
        case BindKind::ToolChangeTargetStation:
        case BindKind::ToolChangeStep:
        case BindKind::ToolChangeTotalSteps: {
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
        }
        default:
            kernel::util::k_snprintf(buf, buf_size, "%s%ld", prefix ? prefix : "",
                                     static_cast<long>(bound_numeric_value(bind)));
            return;
    }
}

const char* mode_text(kernel::ui::operator_api::Mode mode) {
    switch (mode) {
        case kernel::ui::operator_api::Mode::Running: return "MODE RUNNING";
        case kernel::ui::operator_api::Mode::Hold: return "MODE HOLD";
        case kernel::ui::operator_api::Mode::Homing: return "MODE HOMING";
        case kernel::ui::operator_api::Mode::Alarm: return "MODE ALARM";
        case kernel::ui::operator_api::Mode::Ready:
        default: return "MODE READY";
    }
}

const char* active_page_title() {
    if (g_active_page < 0 || static_cast<uint32_t>(g_active_page) >= g_page_count) return "";
    return g_pages[g_active_page].title[0] ? g_pages[g_active_page].title : g_pages[g_active_page].id;
}

int find_widget_index_by_id(const char* id) {
    if (!id || *id == '\0') return -1;
    for (uint32_t i = 0; i < g_widget_count; ++i) {
        if (strcmp(g_widgets[i].spec.id, id) == 0) return static_cast<int>(i);
    }
    return -1;
}

int find_page_index_by_id(const char* id) {
    if (!id || *id == '\0') return -1;
    for (uint32_t i = 0; i < g_page_count; ++i) {
        if (strcmp(g_pages[i].id, id) == 0) return static_cast<int>(i);
    }
    return -1;
}

bool page_has_duplicate_id(const char* id) {
    return id && *id && find_page_index_by_id(id) >= 0;
}

bool widget_has_duplicate_id(const char* id) {
    return id && *id && find_widget_index_by_id(id) >= 0;
}

bool page_accepts_widget(const WidgetSpec& spec) {
    return spec.type != WidgetType::Unknown;
}

bool add_widget_to_state(const WidgetSpec& spec, int page, uint32_t line_no) {
    if (!page_accepts_widget(spec)) {
        set_error(line_no, "widget record has unknown type");
        return false;
    }
    if (g_widget_count >= MAX_WIDGETS) {
        set_error(line_no, "widget limit exceeded");
        return false;
    }
    if (spec.id[0] != '\0' && widget_has_duplicate_id(spec.id)) {
        set_error(line_no, "duplicate widget id");
        return false;
    }
    g_widgets[g_widget_count].spec = spec;
    g_widgets[g_widget_count].page = page;
    g_widgets[g_widget_count].parent = -1;
    g_widgets[g_widget_count].child_align = Align::Left;
    if (page >= 0) ++g_pages[page].widget_count;
    ++g_widget_count;
    return true;
}

bool add_child_link(const char* parent_id, const char* widget_id, Align align, uint32_t line_no) {
    if (!parent_id || !*parent_id || !widget_id || !*widget_id) {
        set_error(line_no, "child record requires parent and widget");
        return false;
    }
    if (g_child_link_count >= MAX_CHILD_LINKS) {
        set_error(line_no, "child link limit exceeded");
        return false;
    }
    copy_field(g_child_links[g_child_link_count].parent_id, sizeof(g_child_links[g_child_link_count].parent_id),
               parent_id, strlen(parent_id));
    copy_field(g_child_links[g_child_link_count].widget_id, sizeof(g_child_links[g_child_link_count].widget_id),
               widget_id, strlen(widget_id));
    g_child_links[g_child_link_count].align = align;
    ++g_child_link_count;
    return true;
}

void format_prefixed_id(char* dst, size_t dst_size, const char* prefix, const char* suffix) {
    if (!dst || dst_size == 0) return;
    dst[0] = '\0';
    const char* safe_prefix = prefix ? prefix : "";
    const char* safe_suffix = suffix ? suffix : "";
    kernel::util::k_snprintf(dst, dst_size, "%s__%s", safe_prefix, safe_suffix);
}

bool is_widget_in_page_range(uint32_t idx, int page_idx) {
    if (page_idx < 0 || static_cast<uint32_t>(page_idx) >= g_page_count) return false;
    const uint32_t start = g_pages[page_idx].first_widget;
    const uint32_t end = start + g_pages[page_idx].widget_count;
    return idx >= start && idx < end;
}

bool remap_action_for_include(const char* source_page_id, const char* current_page_id, const ActionSpec& source, uint32_t line_no) {
    if (g_action_count >= MAX_ACTIONS) {
        set_error(line_no, "action limit exceeded");
        return false;
    }
    ActionSpec& action = g_actions[g_action_count];
    format_prefixed_id(action.widget_id, sizeof(action.widget_id), current_page_id, source.widget_id);
    copy_field(action.target, sizeof(action.target), source.target, strlen(source.target));
    if (strncmp(action.target, "goto:", 5) == 0 && strcmp(action.target + 5, source_page_id) == 0) {
        kernel::util::k_snprintf(action.target, sizeof(action.target), "goto:%s", current_page_id);
    }
    action.event = source.event;
    ++g_action_count;
    return true;
}

bool include_page_into_current(const char* source_id, int current_page, uint32_t line_no) {
    if (!source_id || !*source_id) {
        set_error(line_no, "include record requires page");
        return false;
    }
    if (current_page < 0) {
        set_error(line_no, "include record requires an active page");
        return false;
    }
    const int source_page = find_page_index_by_id(source_id);
    if (source_page < 0) {
        set_error(line_no, "include references unknown page");
        return false;
    }
    const char* current_page_id = g_pages[current_page].id;
    const uint32_t start = g_pages[source_page].first_widget;
    const uint32_t end = start + g_pages[source_page].widget_count;
    for (uint32_t i = start; i < end; ++i) {
        WidgetSpec cloned = g_widgets[i].spec;
        if (cloned.id[0] != '\0') {
            char remapped[MAX_FIELD_LEN] = {};
            format_prefixed_id(remapped, sizeof(remapped), current_page_id, cloned.id);
            copy_field(cloned.id, sizeof(cloned.id), remapped, strlen(remapped));
        }
        if (cloned.parent_id[0] != '\0') {
            char remapped_parent[MAX_FIELD_LEN] = {};
            format_prefixed_id(remapped_parent, sizeof(remapped_parent), current_page_id, cloned.parent_id);
            copy_field(cloned.parent_id, sizeof(cloned.parent_id), remapped_parent, strlen(remapped_parent));
        }
        if (!add_widget_to_state(cloned, current_page, line_no)) return false;
    }
    const uint32_t action_count_before_include = g_action_count;
    for (uint32_t i = 0; i < action_count_before_include; ++i) {
        const int widget_idx = find_widget_index_by_id(g_actions[i].widget_id);
        if (widget_idx >= 0 && is_widget_in_page_range(static_cast<uint32_t>(widget_idx), source_page)) {
            if (!remap_action_for_include(source_id, current_page_id, g_actions[i], line_no)) return false;
        }
    }
    const uint32_t child_count_before_include = g_child_link_count;
    for (uint32_t i = 0; i < child_count_before_include; ++i) {
        const int widget_idx = find_widget_index_by_id(g_child_links[i].widget_id);
        if (widget_idx >= 0 && is_widget_in_page_range(static_cast<uint32_t>(widget_idx), source_page)) {
            char parent_id[MAX_FIELD_LEN] = {};
            char widget_id[MAX_FIELD_LEN] = {};
            format_prefixed_id(parent_id, sizeof(parent_id), current_page_id, g_child_links[i].parent_id);
            format_prefixed_id(widget_id, sizeof(widget_id), current_page_id, g_child_links[i].widget_id);
            if (!add_child_link(parent_id, widget_id, g_child_links[i].align, line_no)) return false;
        }
    }
    return true;
}

bool resolve_relationships(uint32_t line_no) {
    for (uint32_t i = 0; i < g_widget_count; ++i) {
        g_widgets[i].parent = -1;
        g_widgets[i].child_align = Align::Left;
        if (g_widgets[i].spec.parent_id[0] != '\0') {
            const int parent = find_widget_index_by_id(g_widgets[i].spec.parent_id);
            if (parent < 0) {
                set_error(line_no, "widget parent references unknown id");
                return false;
            }
            g_widgets[i].parent = parent;
        } else if (g_widgets[i].page >= 0 && g_pages[g_widgets[i].page].root != nullptr) {
            g_widgets[i].parent = find_widget_index_by_id(g_pages[g_widgets[i].page].id);
        }
    }
    for (uint32_t i = 0; i < g_child_link_count; ++i) {
        const int child_idx = find_widget_index_by_id(g_child_links[i].widget_id);
        const int parent_idx = find_widget_index_by_id(g_child_links[i].parent_id);
        if (child_idx < 0 || parent_idx < 0) {
            set_error(line_no, "child record references unknown widget");
            return false;
        }
        g_widgets[child_idx].parent = parent_idx;
        g_widgets[child_idx].child_align = g_child_links[i].align;
    }
    return true;
}

bool validate_actions(uint32_t line_no) {
    for (uint32_t i = 0; i < g_action_count; ++i) {
        if (find_widget_index_by_id(g_actions[i].widget_id) < 0) {
            set_error(line_no, "action references unknown widget");
            return false;
        }
        if (strncmp(g_actions[i].target, "goto:", 5) == 0 &&
            find_page_index_by_id(g_actions[i].target + 5) < 0) {
            set_error(line_no, "action target references unknown page");
            return false;
        }
    }
    return true;
}

void set_active_page(int idx) {
    if (idx < 0 || static_cast<uint32_t>(idx) >= g_page_count) return;
    kernel::core::ScopedLock guard(g_state_lock);
    g_active_page = idx;
    for (uint32_t i = 0; i < g_page_count; ++i) {
        if (!g_pages[i].root) continue;
        if (static_cast<int>(i) == g_active_page) g_pages[i].root->show();
        else g_pages[i].root->hide();
    }
    if (g_focus_index >= 0 &&
        static_cast<uint32_t>(g_focus_index) < g_focusable_count &&
        !g_focusables[g_focus_index]->visible()) {
        g_focus_index = -1;
        advance_focus(1);
    }
    if (g_root_widget) g_root_widget->mark_subtree_dirty();
}

void set_focus_widget(Widget* widget) {
    if (!widget) return;
    for (uint32_t i = 0; i < g_focusable_count; ++i) {
        if (g_focusables[i] == widget) {
            g_focus_index = static_cast<int>(i);
            for (uint32_t j = 0; j < g_focusable_count; ++j) g_focusables[j]->mark_dirty();
            return;
        }
    }
}

bool is_focused(const Widget* widget) {
    return g_focus_index >= 0 &&
           static_cast<uint32_t>(g_focus_index) < g_focusable_count &&
           g_focusables[g_focus_index] == widget;
}

void advance_focus(int step) {
    if (g_focusable_count == 0) return;
    int next = g_focus_index;
    if (next < 0 || static_cast<uint32_t>(next) >= g_focusable_count) next = 0;
    for (uint32_t i = 0; i < g_focusable_count; ++i) {
        if (g_focus_index < 0 && i == 0) {
            next = 0;
        } else {
            next = (next + step + static_cast<int>(g_focusable_count)) %
                   static_cast<int>(g_focusable_count);
        }
        if (g_focusables[next] && g_focusables[next]->visible()) {
            g_focus_index = next;
            break;
        }
    }
    for (uint32_t i = 0; i < g_focusable_count; ++i) g_focusables[i]->mark_dirty();
}

void run_action_target(const char* target) {
    using namespace kernel::ui::operator_api;
    if (!target || *target == '\0') return;
    trace_click_delivery("[ui-click] action target=%s\n", target);
    if (strncmp(target, "goto:", 5) == 0) {
        set_active_page(find_page_index_by_id(target + 5));
        return;
    }
    if (strncmp(target, "page:", 5) == 0) {
        const char* page = target + 5;
        if (strcmp(page, "position") == 0) select_page(PageId::Position);
        else if (strcmp(page, "program") == 0) select_page(PageId::Program);
        else if (strcmp(page, "offsets") == 0) select_page(PageId::Offsets);
        else if (strcmp(page, "ethercat") == 0) select_page(PageId::EtherCAT);
        else if (strcmp(page, "alarms") == 0) select_page(PageId::Alarms);
        else if (strcmp(page, "setup") == 0) select_page(PageId::Setup);
        return;
    }
    if (strncmp(target, "program:", 8) == 0) {
        const char* command = target + 8;
        if (strcmp(command, "prev") == 0) select_prev_program();
        else if (strcmp(command, "next") == 0) select_next_program();
        else if (strcmp(command, "simulate") == 0) request_program_simulation();
        else if (strncmp(command, "select:", 7) == 0) {
            const int row = simple_atoi(command + 7);
            if (row >= 0) select_program(0, static_cast<size_t>(row));
        }
        return;
    }
    if (strncmp(target, "homing:", 7) == 0) {
        const char* command = target + 7;
        if (strncmp(command, "axis:", 5) == 0) {
            set_homing_axis(static_cast<uint32_t>(simple_atoi(command + 5)));
        } else if (strncmp(command, "method:", 7) == 0) {
            // homing:method:<id>:<name?>  — name is rendered in the wizard,
            // id is the CiA-402 method number passed to run_homing_sequence.
            const char* p = command + 7;
            const int32_t id = simple_atoi(p);
            const char* colon = nullptr;
            for (const char* q = p; *q; ++q) if (*q == ':') { colon = q; break; }
            const char* name = colon ? (colon + 1) : "Method";
            set_homing_method(id, name);
        } else if (strcmp(command, "start") == 0) {
            start_homing();
        } else if (strcmp(command, "abort") == 0) {
            abort_homing();
        }
        return;
    }
    if (strncmp(target, "probe:wizard:", 13) == 0) {
        // Guided probing wizard. Cycle selection tokens map to the
        // operator-facing ProbeCycleKind enum; transport tokens drive
        // the state machine in operator_api. probe_wizard_start checks
        // the EtherCAT deadline-fault latch internally.
        const char* command = target + 13;
        if (strncmp(command, "select:", 7) == 0) {
            const char* k = command + 7;
            ProbeCycleKind kind = ProbeCycleKind::None;
            if      (strcmp(k, "qualify") == 0) kind = ProbeCycleKind::Qualify;
            else if (strcmp(k, "z")       == 0) kind = ProbeCycleKind::ZSurface;
            else if (strcmp(k, "edgex")   == 0) kind = ProbeCycleKind::EdgeX;
            else if (strcmp(k, "edgey")   == 0) kind = ProbeCycleKind::EdgeY;
            else if (strcmp(k, "bore")    == 0) kind = ProbeCycleKind::BoreCenter;
            else if (strcmp(k, "pocket")  == 0) kind = ProbeCycleKind::Pocket3D;
            else if (strcmp(k, "sphere")  == 0) kind = ProbeCycleKind::Sphere;
            probe_wizard_select(kind);
        } else if (strcmp(command, "start")  == 0) probe_wizard_start();
        else if (strcmp(command, "abort")  == 0) probe_wizard_abort();
        else if (strcmp(command, "accept") == 0) probe_wizard_accept();
        else if (strcmp(command, "reject") == 0) probe_wizard_reject();
        return;
    }
    if (strncmp(target, "macro:", 6) == 0) {
        const char* command = target + 6;
        if (strcmp(command, "prev") == 0) select_prev_macro();
        else if (strcmp(command, "next") == 0) select_next_macro();
        else if (strcmp(command, "run") == 0) run_selected_macro();
        else if (strcmp(command, "abort") == 0) abort_macro();
        else if (strncmp(command, "run:", 4) == 0) run_macro_by_name(command + 4);
        return;
    }
    if (strncmp(target, "setup:", 6) == 0) {
        const char* command = target + 6;
        if (strcmp(command, "save") == 0) save_setup();
        else if (strcmp(command, "load") == 0) load_setup();
        return;
    }
    if (strncmp(target, "offset:work:", 12) == 0) {
        select_work_offset(static_cast<size_t>(simple_atoi(target + 12)));
        return;
    }
    if (strncmp(target, "offset:tool:", 12) == 0) {
        select_tool_offset(static_cast<size_t>(simple_atoi(target + 12)));
        return;
    }
    if (strncmp(target, "offset:nudge:", 13) == 0) {
        const auto offsets = offsets_snapshot();
        const size_t active = offsets.active_work;
        const char axis_name = target[13];
        const char* delta_str = target + 15;
        const float delta = simple_atof(delta_str) / 1000.0f;
        size_t axis = 0;
        if (axis_name == 'x' || axis_name == 'X') axis = 0;
        else if (axis_name == 'y' || axis_name == 'Y') axis = 1;
        else if (axis_name == 'z' || axis_name == 'Z') axis = 2;
        else if (axis_name == 'a' || axis_name == 'A') axis = 3;
        else return;
        if (axis < offsets.row_count) {
            const float current = static_cast<float>(offsets.rows[axis].work_offset) / 1000.0f;
            set_work_offset_axis(active, axis, current + delta);
        }
        return;
    }
    if (strcmp(target, "demo:cycle") == 0) toggle_cycle();
    else if (strcmp(target, "demo:hold") == 0) toggle_hold();
    else if (strcmp(target, "demo:reset") == 0) reset_alarm();
    else if (strcmp(target, "demo:home") == 0) home_selected();
    else if (strcmp(target, "demo:jog+") == 0) jog_axis_step(+1);
    else if (strcmp(target, "demo:jog-") == 0) jog_axis_step(-1);
    else if (strncmp(target, "jog:axis:", 9) == 0) {
        set_selected_axis(static_cast<uint32_t>(simple_atoi(target + 9)));
    }
    else if (strncmp(target, "jog:inc:", 8) == 0) {
        set_jog_increment(simple_atoi(target + 8));
    }
    else if (strncmp(target, "jog:hold:", 9) == 0) {
        // Press-to-toggle continuous jog: button widgets only fire on
        // TouchUp, so a true hold event is not available. First tap
        // starts motion in the requested direction; tapping the same
        // axis (any direction) stops it. Formats:
        //   jog:hold:<axis>:<sign>   explicit axis (0..3)
        //   jog:hold:sel:<sign>      use machine_snapshot().selected_axis
        // A 4-bit bitfield tracks which axes the UI started so a second
        // tap reliably stops them without depending on motion telemetry.
        static uint8_t jogging_axes = 0;
        const char* p = target + 9;
        uint32_t a = 0;
        if (strncmp(p, "sel:", 4) == 0) {
            a = machine_snapshot().selected_axis & 3u;
            p += 4;
        } else {
            a = static_cast<uint32_t>(simple_atoi(p) & 3);
            while (*p && *p != ':') ++p;
            if (*p == ':') ++p;
        }
        const int32_t sign = simple_atoi(p);
        const uint8_t mask = static_cast<uint8_t>(1u << a);
        if (jogging_axes & mask) {
            stop_continuous_jog(a);
            jogging_axes = static_cast<uint8_t>(jogging_axes & ~mask);
        } else {
            start_continuous_jog(a, sign);
            jogging_axes = static_cast<uint8_t>(jogging_axes | mask);
        }
    }
    else if (strncmp(target, "jog:stop:", 9) == 0) {
        stop_continuous_jog(static_cast<uint32_t>(simple_atoi(target + 9) & 3));
    }
    else if (strcmp(target, "spindle:start") == 0) spindle_start();
    else if (strcmp(target, "spindle:stop") == 0) spindle_stop();
    else if (strcmp(target, "spindle:rev") == 0) {
        const auto s = spindle_status();
        spindle_set_rpm(-s.requested_rpm);
        spindle_start();
    }
    else if (strcmp(target, "mode:auto") == 0) set_operator_mode(OperatorMode::Auto);
    else if (strcmp(target, "mode:mdi") == 0) set_operator_mode(OperatorMode::MDI);
    else if (strcmp(target, "mode:jog") == 0) set_operator_mode(OperatorMode::Jog);
    else if (strcmp(target, "mode:setup") == 0) set_operator_mode(OperatorMode::Setup);
    else if (strcmp(target, "view:toolpath:toggle") == 0) toggle_view_toolpath();
    else if (strcmp(target, "view:toolpods:toggle") == 0) toggle_view_toolpods();
    else if (strcmp(target, "view:reset") == 0) view_reset_all_cameras();
    else if (strcmp(target, "view:zoom:in") == 0) view_zoom_all(0.85f);
    else if (strcmp(target, "view:zoom:out") == 0) view_zoom_all(1.18f);
    else if (strcmp(target, "ec:estop") == 0) request_ec_estop();
    else if (strcmp(target, "ec:clear_fault") == 0) clear_ec_fault();
    else if (strcmp(target, "restart:confirm") == 0) confirm_restart();
    else if (strcmp(target, "restart:cancel") == 0) cancel_restart();
    else if (strcmp(target, "alarm:clear_history") == 0) clear_alarm_history();
    else if (strncmp(target, "alarm:ack:", 10) == 0) {
        const int row = simple_atoi(target + 10);
        if (row >= 0 && row < 4) {
            const auto snap = alarms_snapshot();
            if (static_cast<size_t>(row) < snap.active_count) {
                acknowledge_alarm(snap.active[row].id);
            }
        }
    }
    else if (strcmp(target, "demo:estop") == 0) {
        // Stop all interpreter channels + latch alarm. Full drive-level quickstop
        // belongs in the motion layer; this mirrors the reset path plus an
        // explicit hold so the operator sees immediate motion suspension.
        reset_alarm();
        toggle_hold();
    }
    else if (strncmp(target, "cal:", 4) == 0) {
        // Compensation page actions. Routes to operator_api wrappers
        // which apply the master deadline-fault gate before calling
        // motion::g_motion. PEC ADD POINT reads the buffered pending
        // pos/err set by the two input fields above the button.
        const char* command = target + 4;
        if (strncmp(command, "pec:axis:", 9) == 0) {
            cal_pec_select_axis(static_cast<uint32_t>(simple_atoi(command + 9)));
        } else if (strcmp(command, "pec:add") == 0) {
            cal_pec_commit_point();
        } else if (strcmp(command, "pec:clear") == 0) {
            cal_pec_clear();
        } else if (strcmp(command, "pec:enable:toggle") == 0) {
            cal_pec_set_enabled(!calibration_snapshot().pec_enabled);
        } else if (strcmp(command, "geom:enable:toggle") == 0) {
            cal_geom_set_enabled(!calibration_snapshot().geom_enabled);
        } else if (strcmp(command, "sphere:enable:toggle") == 0) {
            cal_sphere_set_enabled(!calibration_snapshot().sphere_enabled);
        } else if (strcmp(command, "sphere:compute") == 0) {
            cal_sphere_compute();
        } else if (strcmp(command, "sphere:clear") == 0) {
            cal_sphere_clear();
        }
    }
    else if (strcmp(target, "mdi:submit") == 0) {
        if (!kernel::ui::operator_api::mode_allows_mdi()) return;
        const auto s = cnc::mdi::g_service.snapshot();
        cnc::mdi::g_service.submit(s.input);
    }
    else if (strcmp(target, "mdi:clear") == 0) cnc::mdi::g_service.clear();
    else if (strcmp(target, "mdi:abort") == 0) cnc::mdi::g_service.abort();
    // Network setup actions. dhcp:on/off flips hmi config; commit_static
    // pushes the operator's pending IP/gateway live; ping fires async at
    // the pending target. All gated on master deadline-fault inside
    // operator_api.
    else if (strcmp(target, "net:dhcp:on")  == 0) net_set_dhcp(true);
    else if (strcmp(target, "net:dhcp:off") == 0) net_set_dhcp(false);
    else if (strcmp(target, "net:commit_static") == 0) net_commit_static();
    else if (strcmp(target, "net:ping") == 0) net_request_ping();
    // Per-axis status sub-page mutators — operate on the SELECTED axis.
    else if (strcmp(target, "axis:detail:enable") == 0) axis_detail_enable();
    else if (strcmp(target, "axis:detail:disable") == 0) axis_detail_disable();
    else if (strcmp(target, "axis:detail:fault_reset") == 0) axis_detail_fault_reset();
    // Tool change wizard transport. start/abort/accept gate on
    // master-fault inside operator_api.
    else if (strcmp(target, "tc:start") == 0) tool_change_start();
    else if (strcmp(target, "tc:abort") == 0) tool_change_abort();
    else if (strcmp(target, "tc:accept") == 0) tool_change_accept();
}

bool commit_input_target(const char* target, const char* value_text, BindKind bind_hint) {
    using namespace kernel::ui::operator_api;
    if (!value_text) return false;
    if (target && *target) {
        if (strncmp(target, "commit:offset:", 14) == 0) {
            const auto offsets = offsets_snapshot();
            const size_t active = offsets.active_work;
            size_t axis = 0;
            const char axis_name = target[14];
            if (axis_name == 'x' || axis_name == 'X') axis = 0;
            else if (axis_name == 'y' || axis_name == 'Y') axis = 1;
            else if (axis_name == 'z' || axis_name == 'Z') axis = 2;
            else if (axis_name == 'a' || axis_name == 'A') axis = 3;
            else return false;
            float value = simple_atof(value_text);
            if (axis == 3) {
                if (value < -360.0f) value = -360.0f;
                if (value > 360.0f) value = 360.0f;
            } else {
                if (value < -999.999f) value = -999.999f;
                if (value > 999.999f) value = 999.999f;
            }
            return set_work_offset_axis(active, axis, value);
        }
        if (strcmp(target, "commit:cal:pec:pos") == 0) {
            cal_pec_set_pending_pos(simple_atoi(value_text));
            return true;
        }
        if (strcmp(target, "commit:cal:pec:err") == 0) {
            cal_pec_set_pending_err(simple_atoi(value_text));
            return true;
        }
        if (strcmp(target, "commit:cal:geom:xy") == 0) {
            cal_geom_set("XY", simple_atoi(value_text));
            return true;
        }
        if (strcmp(target, "commit:cal:geom:xz") == 0) {
            cal_geom_set("XZ", simple_atoi(value_text));
            return true;
        }
        if (strcmp(target, "commit:cal:geom:yz") == 0) {
            cal_geom_set("YZ", simple_atoi(value_text));
            return true;
        }
        if (strcmp(target, "commit:tool:length") == 0 ||
            strcmp(target, "commit:tool:radius") == 0 ||
            strcmp(target, "commit:tool:wear") == 0) {
            const auto offsets = offsets_snapshot();
            const size_t active = offsets.active_tool;
            if (active >= offsets.tools.size()) return false;
            float length = offsets.tools[active].length;
            float radius = offsets.tools[active].radius;
            float wear = offsets.tools[active].wear;
            float value = simple_atof(value_text);
            if (strcmp(target, "commit:tool:length") == 0) {
                if (value < 0.0f) value = 0.0f;
                if (value > 999.999f) value = 999.999f;
                length = value;
            } else if (strcmp(target, "commit:tool:radius") == 0) {
                if (value < 0.0f) value = 0.0f;
                if (value > 250.000f) value = 250.000f;
                radius = value;
            } else {
                if (value < -25.000f) value = -25.000f;
                if (value > 25.000f) value = 25.000f;
                wear = value;
            }
            return set_tool_offset(active, length, radius, wear);
        }
    }

    if (bind_hint == BindKind::ProgramName) return select_program_by_name(value_text);
    if (bind_hint == BindKind::MdiInput) {
        cnc::mdi::g_service.set_input(value_text);
        if (!kernel::ui::operator_api::mode_allows_mdi()) return false;
        return cnc::mdi::g_service.submit(value_text);
    }
    if (target && *target && strcmp(target, "commit:restart:line") == 0) {
        const int32_t line_no = simple_atoi(value_text);
        if (line_no < 0) return false;
        const bool ok = restart_program_at_line(static_cast<size_t>(line_no));
        // Hop the operator straight to the confirm wizard so they see the
        // reconstructed state without manually navigating. Cycle Start on
        // the dashboard is gated until they tap CONFIRM.
        if (ok) (void)set_page("restart_confirm");
        return ok;
    }
    if (target && *target && strcmp(target, "commit:spindle:rpm") == 0) {
        // Bare-number entry only — sign carries direction. Operator must
        // tap START explicitly after committing the value to begin motion.
        spindle_set_rpm(simple_atoi(value_text));
        return true;
    }
    // Network setup: dotted-quad IP entry. simple_atoi can't parse "a.b.c.d"
    // so do it inline. Reject any text that doesn't end with three dots and
    // four numeric octets in 0..255.
    if (target && *target &&
        (strcmp(target, "commit:net:ip") == 0 ||
         strcmp(target, "commit:net:gateway") == 0 ||
         strcmp(target, "commit:net:ping_target") == 0)) {
        uint32_t parts[4] = {0, 0, 0, 0};
        size_t pi = 0;
        const char* p = value_text;
        while (*p && pi < 4) {
            uint32_t v = 0;
            bool any = false;
            while (*p >= '0' && *p <= '9') { v = v * 10 + static_cast<uint32_t>(*p - '0'); ++p; any = true; }
            if (!any || v > 255u) return false;
            parts[pi++] = v;
            if (*p == '.') ++p; else if (*p != '\0') return false;
        }
        if (pi != 4) return false;
        const uint32_t ip = (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
        if (strcmp(target, "commit:net:ip") == 0) net_set_pending_ip(ip);
        else if (strcmp(target, "commit:net:gateway") == 0) net_set_pending_gateway(ip);
        else net_set_pending_ping_target(ip);
        return true;
    }
    if (target && *target && strcmp(target, "commit:tc:target") == 0) {
        const int32_t v = simple_atoi(value_text);
        if (v <= 0) return false;
        tool_change_set_pending_target(static_cast<uint32_t>(v));
        return true;
    }
    return false;
}

bool is_numeric_bind(BindKind bind) {
    switch (bind) {
        case BindKind::WorkOffsetX:
        case BindKind::WorkOffsetY:
        case BindKind::WorkOffsetZ:
        case BindKind::WorkOffsetA:
        case BindKind::ToolLength:
        case BindKind::ToolRadius:
        case BindKind::ToolWear:
        case BindKind::CalPecPendingPos:
        case BindKind::CalPecPendingErr:
        case BindKind::CalGeomXy:
        case BindKind::CalGeomXz:
        case BindKind::CalGeomYz:
            return true;
        default:
            return false;
    }
}

bool contains_char(const char* s, char ch) {
    if (!s) return false;
    while (*s) {
        if (*s == ch) return true;
        ++s;
    }
    return false;
}

const char* helper_text_for_bind(BindKind bind) {
    switch (bind) {
        case BindKind::ProgramName: return "Enter program name, then press Enter";
        case BindKind::WorkOffsetX:
        case BindKind::WorkOffsetY:
        case BindKind::WorkOffsetZ: return "Range: -999.999 to 999.999 mm";
        case BindKind::WorkOffsetA: return "Range: -360.000 to 360.000 deg";
        case BindKind::ToolLength: return "Range: 0.000 to 999.999 mm";
        case BindKind::ToolRadius: return "Range: 0.000 to 250.000 mm";
        case BindKind::ToolWear: return "Range: -25.000 to 25.000 mm";
        case BindKind::CalPecPendingPos: return "Position in counts; Enter to buffer";
        case BindKind::CalPecPendingErr: return "Error in counts (actual - cmd)";
        case BindKind::CalGeomXy:
        case BindKind::CalGeomXz:
        case BindKind::CalGeomYz: return "Squareness in microradians";
        default: return "";
    }
}

// Sort the action index by widget_id (then by event). Insertion sort is fine
// at the scale we run (MAX_ACTIONS is small, parse-time-only).
static void build_action_index() {
    g_action_index_count = g_action_count;
    for (uint32_t i = 0; i < g_action_count; ++i) {
        g_action_index[i] = static_cast<uint16_t>(i);
    }
    for (uint32_t i = 1; i < g_action_index_count; ++i) {
        const uint16_t key = g_action_index[i];
        const ActionSpec& key_act = g_actions[key];
        uint32_t j = i;
        while (j > 0) {
            const ActionSpec& prev = g_actions[g_action_index[j - 1]];
            const int c = strcmp(prev.widget_id, key_act.widget_id);
            if (c < 0) break;
            if (c == 0 && static_cast<uint8_t>(prev.event)
                          <= static_cast<uint8_t>(key_act.event)) break;
            g_action_index[j] = g_action_index[j - 1];
            --j;
        }
        g_action_index[j] = key;
    }
}

const char* action_target_for_widget(const char* id, BuilderEvent event, const char* inline_action) {
    if (inline_action && *inline_action) return inline_action;
    // Pre-parse window: index isn't built yet. Fall back to linear scan so
    // anything that resolves actions during build_ui() (before
    // build_action_index() runs) still works.
    if (g_action_index_count == 0) {
        for (uint32_t i = 0; i < g_action_count; ++i) {
            if (g_actions[i].event == event && strcmp(g_actions[i].widget_id, id) == 0) {
                return g_actions[i].target;
            }
        }
        return nullptr;
    }
    // Binary search by widget_id, then linear walk over the (small) run of
    // actions sharing that widget_id to match the event.
    int32_t lo = 0;
    int32_t hi = static_cast<int32_t>(g_action_index_count) - 1;
    while (lo <= hi) {
        const int32_t mid = lo + (hi - lo) / 2;
        const ActionSpec& a = g_actions[g_action_index[mid]];
        const int c = strcmp(a.widget_id, id);
        if (c == 0) {
            // Walk left to the first match, then forward over the run.
            int32_t k = mid;
            while (k > 0 && strcmp(g_actions[g_action_index[k - 1]].widget_id, id) == 0) --k;
            for (; k < static_cast<int32_t>(g_action_index_count); ++k) {
                const ActionSpec& cand = g_actions[g_action_index[k]];
                if (strcmp(cand.widget_id, id) != 0) break;
                if (cand.event == event) return cand.target;
            }
            return nullptr;
        }
        if (c < 0) lo = mid + 1; else hi = mid - 1;
    }
    return nullptr;
}

class BuilderLabel final : public Widget {
public:
    explicit BuilderLabel(const WidgetSpec& spec)
        : Widget(spec.x, spec.y, spec.w > 0 ? static_cast<uint32_t>(spec.w) : 100U,
                 spec.h > 0 ? static_cast<uint32_t>(spec.h) : 24U),
          spec_(spec),
          fg_(to_color(spec.color, Color::White())),
          bg_(to_color(spec.bg_color, Color::Black())),
          bind_(parse_bind(spec.bind)) {
        // Labels honour active_if by swapping foreground to the design
        // system's fault red. Used by DRO digits to flag soft-limit
        // proximity. Same parser shape as BuilderButton — split on the
        // LAST colon so multi-segment bind tokens are preserved.
        if (spec.active_if[0] != '\0') {
            const char* colon = nullptr;
            for (const char* p = spec.active_if; *p; ++p) if (*p == ':') colon = p;
            if (colon) {
                char bind_buf[MAX_FIELD_LEN] = {};
                copy_field(bind_buf, sizeof(bind_buf), spec.active_if,
                           static_cast<size_t>(colon - spec.active_if));
                active_bind_ = parse_bind(bind_buf);
                active_value_ = simple_atoi(colon + 1);
            }
        }
    }

    void render(Framebuffer& fb) override {
        if (!visible_) return;
        const char* text = spec_.text;
        char buf[MAX_FIELD_LEN] = {};
        Color fg = fg_;
        if (active_bind_ != BindKind::None &&
            bound_numeric_value(active_bind_) == active_value_) {
            fg = Color(0xEF, 0x44, 0x44);
        }
        if (bind_ != BindKind::None) {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            switch (bind_) {
                case BindKind::Mode: text = mode_text(snap.mode); break;
                case BindKind::Alarm:
                    text = snap.mode == kernel::ui::operator_api::Mode::Alarm ? "ALARM ACTIVE" : "ALARMS CLEAR";
                    break;
                case BindKind::Prompt:
                    text = snap.mode == kernel::ui::operator_api::Mode::Running ? "CYCLE RUN IN PROGRESS" : "READY FOR COMMAND";
                    break;
                case BindKind::PageName:
                    text = active_page_title();
                    break;
                case BindKind::ProgramName:
                    text = kernel::ui::operator_api::selected_program_name();
                    break;
                case BindKind::WorkName:
                    text = kernel::ui::operator_api::offsets_snapshot().workset_name;
                    break;
                case BindKind::ToolName:
                    text = kernel::ui::operator_api::offsets_snapshot().tool_name;
                    break;
                default:
                    format_bind_value(bind_, buf, sizeof(buf), spec_.text[0] ? spec_.text : "");
                    text = buf;
                    break;
            }
        }
        fb.fill_rect(x_, y_, width_, height_, bg_);
        const uint32_t scale = spec_.text_scale > 0 ? spec_.text_scale : 1u;
        int32_t draw_x = x_;
        if (spec_.align != Align::Left) {
            const int32_t text_w = static_cast<int32_t>(strlen(text) * 8U * scale);
            if (spec_.align == Align::Center) draw_x = x_ + static_cast<int32_t>(width_ / 2) - text_w / 2;
            else draw_x = x_ + static_cast<int32_t>(width_) - text_w - 8;
        }
        if (scale > 1) {
            fb.draw_text_scaled(draw_x, y_, text, fg, bg_, scale);
        } else {
            fb.draw_text(draw_x, y_, text, fg, bg_);
        }
    }

private:
    WidgetSpec spec_;
    Color fg_;
    Color bg_;
    BindKind bind_;
    BindKind active_bind_ = BindKind::None;
    int32_t  active_value_ = 0;
};

class BuilderButton final : public kernel::ui::Button {
public:
    explicit BuilderButton(const WidgetSpec& spec)
        : kernel::ui::Button(spec.x, spec.y,
                             spec.w > 0 ? static_cast<uint32_t>(spec.w) : 120U,
                             spec.h > 0 ? static_cast<uint32_t>(spec.h) : 48U,
                             spec.text),
          spec_(spec),
          base_bg_(to_color(spec.bg_color != kTransparent ? spec.bg_color : spec.color, Color(64, 64, 64))),
          fg_(to_color(spec.color, Color::White())),
          bind_(parse_bind(spec.bind)) {
        // Dimmed variant (for goto: tabs on the current page — keeps the
        // "you are here" hint quiet).
        dim_bg_ = Color(base_bg_.r > 40 ? base_bg_.r - 40 : 0,
                        base_bg_.g > 40 ? base_bg_.g - 40 : 0,
                        base_bg_.b > 40 ? base_bg_.b - 40 : 0);
        // Brightened variant for active_if matches (operator "this is the
        // currently selected WCS / tool" hint — must be unmistakable).
        bright_bg_ = Color(static_cast<uint8_t>(base_bg_.r < 215 ? base_bg_.r + 40 : 255),
                           static_cast<uint8_t>(base_bg_.g < 215 ? base_bg_.g + 40 : 255),
                           static_cast<uint8_t>(base_bg_.b < 215 ? base_bg_.b + 40 : 255));
        set_colors(base_bg_, base_bg_, Color(base_bg_.r / 2, base_bg_.g / 2, base_bg_.b / 2), fg_);

        // Parse active_if = "<bind>:<int>". Split on the LAST colon so
        // multi-segment bind tokens (program:0:loaded, homing:method, etc.)
        // are preserved. A bind name with no colon-suffixed integer is rejected.
        if (spec.active_if[0] != '\0') {
            const char* colon = nullptr;
            for (const char* p = spec.active_if; *p; ++p) if (*p == ':') colon = p;
            if (colon) {
                char bind_buf[MAX_FIELD_LEN] = {};
                copy_field(bind_buf, sizeof(bind_buf), spec.active_if,
                           static_cast<size_t>(colon - spec.active_if));
                active_bind_ = parse_bind(bind_buf);
                active_value_ = simple_atoi(colon + 1);
            }
        }
    }

    bool active_if_matches() const {
        if (active_bind_ == BindKind::None) return false;
        return bound_numeric_value(active_bind_) == active_value_;
    }

    void render(Framebuffer& fb) override {
        Color bg = base_bg_;
        bool highlight_border = false;
        // "You are here" highlight on any button whose goto: target points to
        // the current page. Same visual treatment as active_if (brightened bg
        // + white border) so the bottom-nav tab for the active page is
        // unmistakable. Applies uniformly to every page that includes
        // bottom_nav, which keeps the nav consistent across the whole UI.
        const char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action);
        const bool is_goto_self = target && strncmp(target, "goto:", 5) == 0 &&
                                  find_page_index_by_id(target + 5) == g_active_page;
        if (is_goto_self || active_if_matches()) {
            bg = bright_bg_;
            highlight_border = true;
        }
        set_colors(bg, bg, Color(bg.r / 2, bg.g / 2, bg.b / 2), fg_);

        // Buttons may bind their label to a string snapshot field (e.g. the
        // file-browser row buttons read program:N:name). Resolve once into a
        // local buffer and treat it as the label below.
        char label_buf[MAX_FIELD_LEN] = {};
        const char* label = spec_.text;
        if (bind_ != BindKind::None) {
            format_bind_value(bind_, label_buf, sizeof(label_buf), "");
            if (label_buf[0] != '\0') label = label_buf;
        }
        // If the spec asked for scaled label text, bypass the base Button
        // renderer's draw_text call by painting the box ourselves and then
        // drawing label with draw_text_scaled.
        const uint32_t scale = spec_.text_scale > 0 ? spec_.text_scale : 1u;
        if (scale > 1) {
            fb.fill_rect(x_, y_, width_, height_, bg);
            fb.draw_rect(x_, y_, width_, height_, fg_, 3);
            fb.fill_rect(x_ + 8, y_ + 8, width_ - 16, height_ - 16, bg);
            const int32_t text_w = static_cast<int32_t>(strlen(label) * 8U * scale);
            const int32_t text_h = static_cast<int32_t>(16U * scale);
            const int32_t lx = spec_.align == Align::Left
                ? x_ + 16
                : (spec_.align == Align::Right
                    ? x_ + static_cast<int32_t>(width_) - text_w - 16
                    : x_ + static_cast<int32_t>(width_) / 2 - text_w / 2);
            const int32_t ly = y_ + static_cast<int32_t>(height_) / 2 - text_h / 2;
            fb.draw_text_scaled(lx, ly, label, fg_, bg, scale);
        } else if (bind_ != BindKind::None) {
            // Honour the resolved label even at scale 1; the base Button
            // renderer would re-draw the static spec_.text otherwise.
            fb.fill_rect(x_, y_, width_, height_, bg);
            fb.draw_rect(x_, y_, width_, height_, fg_, 2);
            const int32_t text_w = static_cast<int32_t>(strlen(label) * 8U);
            const int32_t lx = spec_.align == Align::Left
                ? x_ + 8
                : (spec_.align == Align::Right
                    ? x_ + static_cast<int32_t>(width_) - text_w - 8
                    : x_ + static_cast<int32_t>(width_) / 2 - text_w / 2);
            const int32_t ly = y_ + static_cast<int32_t>(height_) / 2 - 8;
            fb.draw_text(lx, ly, label, fg_, bg);
        } else {
            kernel::ui::Button::render(fb);
        }

        if (highlight_border) {
            fb.draw_rect(x_ - 2, y_ - 2, width_ + 4, height_ + 4, Color(255, 255, 255), 3);
        }
        if (is_focused(this)) {
            fb.draw_rect(x_ - 3, y_ - 3, width_ + 6, height_ + 6, Color(255, 255, 255), 2);
        }
    }

    bool on_event(const UIEvent& event) override {
        if (event.type == kernel::ui::EventType::TouchDown && contains(event.touch.x, event.touch.y)) {
            set_focus_widget(this);
        }
        const bool handled = kernel::ui::Button::on_event(event);
        if (handled && event.type == kernel::ui::EventType::TouchUp && contains(event.touch.x, event.touch.y)) {
            trace_click_delivery("[ui-click] widget=%s touch=(%ld,%ld)\n",
                                 spec_.id, nullptr,
                                 static_cast<long>(event.touch.x),
                                 static_cast<long>(event.touch.y));
            if (const char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action)) {
                run_action_target(target);
            }
        } else if (event.type == kernel::ui::EventType::KeyDown && is_focused(this) &&
                   (event.key.keycode == '\r' || event.key.keycode == ' ')) {
            if (const char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action)) {
                run_action_target(target);
            }
            return true;
        }
        return handled;
    }

private:
    WidgetSpec spec_;
    Color base_bg_;
    Color dim_bg_;
    Color bright_bg_;
    Color fg_;
    BindKind active_bind_ = BindKind::None;
    int32_t active_value_ = 0;
    BindKind bind_ = BindKind::None;
};

class BuilderPanel final : public Panel {
public:
    explicit BuilderPanel(const WidgetSpec& spec)
        : Panel(spec.x, spec.y,
                spec.w > 0 ? static_cast<uint32_t>(spec.w) : 320U,
                spec.h > 0 ? static_cast<uint32_t>(spec.h) : 240U,
                spec.text[0] ? spec.text : nullptr,
                to_color(spec.bg_color, Color::White()),
                to_color(spec.border_color, Color::Black()),
                to_color(spec.color, Color::Black())) {}
};

class BuilderContainer final : public Container {
public:
    explicit BuilderContainer(const WidgetSpec& spec)
        : Container(spec.x, spec.y,
                    spec.w > 0 ? static_cast<uint32_t>(spec.w) : 240U,
                    spec.h > 0 ? static_cast<uint32_t>(spec.h) : 180U),
          bg_(to_color(spec.bg_color, Color::Black())),
          border_(to_color(spec.border_color, Color::Black())),
          has_bg_(spec.bg_color != kTransparent),
          has_border_(spec.border_color != kTransparent) {}

    void render(Framebuffer& fb) override {
        if (!visible_) return;
        if (has_bg_) fb.fill_rect(x_, y_, width_, height_, bg_);
        if (has_border_) fb.draw_rect(x_, y_, width_, height_, border_, 2);
        Container::render(fb);
    }

private:
    Color bg_;
    Color border_;
    bool has_bg_;
    bool has_border_;
};

class BuilderProgress final : public kernel::ui::ProgressBar {
public:
    explicit BuilderProgress(const WidgetSpec& spec)
        : kernel::ui::ProgressBar(spec.x, spec.y,
                                  spec.w > 0 ? static_cast<uint32_t>(spec.w) : 100U,
                                  spec.h > 0 ? static_cast<uint32_t>(spec.h) : 20U),
          bind_(parse_bind(spec.bind)) {
        set_max(spec.max_val > 0 ? static_cast<uint32_t>(spec.max_val) : 100U);
        set_value(spec.value > 0 ? static_cast<uint32_t>(spec.value) : 0U);
        set_colors(to_color(spec.color, Color::Green()), to_color(spec.bg_color, Color::DarkGray()));
    }

    void render(Framebuffer& fb) override {
        if (bind_ != BindKind::None) {
            int32_t v = bound_numeric_value(bind_);
            if (v < 0) v = 0;
            set_value(static_cast<uint32_t>(v));
        }
        kernel::ui::ProgressBar::render(fb);
    }

private:
    BindKind bind_;
};

class BuilderGraph final : public kernel::ui::BarGraph {
public:
    explicit BuilderGraph(const WidgetSpec& spec)
        : kernel::ui::BarGraph(spec.x, spec.y,
                               spec.w > 0 ? static_cast<uint32_t>(spec.w) : 240U,
                               spec.h > 0 ? static_cast<uint32_t>(spec.h) : 120U,
                               4U),
          bind_(parse_bind(spec.bind)) {
        set_max(spec.max_val > 0 ? static_cast<uint32_t>(spec.max_val) : 10000U);
    }

    void render(Framebuffer& fb) override {
        if (bind_ == BindKind::None || bind_ == BindKind::AxisX || bind_ == BindKind::AxisY ||
            bind_ == BindKind::AxisZ || bind_ == BindKind::AxisA) {
            const auto snap = kernel::ui::operator_api::machine_snapshot();
            set_value(0, snap.axis_pos[0] < 0 ? -snap.axis_pos[0] : snap.axis_pos[0]);
            set_value(1, snap.axis_pos[1] < 0 ? -snap.axis_pos[1] : snap.axis_pos[1]);
            set_value(2, snap.axis_pos[2] < 0 ? -snap.axis_pos[2] : snap.axis_pos[2]);
            set_value(3, snap.axis_pos[3] < 0 ? -snap.axis_pos[3] : snap.axis_pos[3]);
        } else {
            const int32_t v = bound_numeric_value(bind_);
            set_value(0, v < 0 ? -v : v);
            set_value(1, 0);
            set_value(2, 0);
            set_value(3, 0);
        }
        kernel::ui::BarGraph::render(fb);
    }

private:
    BindKind bind_;
};

class BuilderSlider final : public Widget {
public:
    explicit BuilderSlider(const WidgetSpec& spec)
        : Widget(spec.x, spec.y,
                 spec.w > 0 ? static_cast<uint32_t>(spec.w) : 180U,
                 spec.h > 0 ? static_cast<uint32_t>(spec.h) : 32U),
          spec_(spec),
          min_(spec.min_val),
          max_(spec.max_val > spec.min_val ? spec.max_val : spec.min_val + 100.0f),
          bind_(parse_bind(spec.bind)),
          value_(static_cast<int32_t>(spec.value)),
          hovered_(false),
          dragging_(false) {}

    void render(Framebuffer& fb) override {
        const Color bg = to_color(spec_.bg_color, Color(80, 88, 102));
        const Color fg = to_color(spec_.color, Color(38, 139, 210));
        const Color ink = Color::White();
        if (!dragging_ && bind_ != BindKind::None) value_ = bound_numeric_value(bind_);
        const int32_t track_y = y_ + static_cast<int32_t>(height_ / 2) - 4;
        fb.fill_rect(x_, track_y, width_, 8, bg);
        const float ratio = (value_ - min_) / (max_ - min_);
        int32_t knob_x = x_ + static_cast<int32_t>(ratio * static_cast<float>(width_ > 16 ? width_ - 16 : 0));
        if (knob_x < x_) knob_x = x_;
        const uint32_t fill_w = knob_x > x_ ? static_cast<uint32_t>(knob_x - x_ + 8) : 0U;
        if (fill_w > 0) fb.fill_rect(x_, track_y, fill_w, 8, fg);
        const Color knob = hovered_ && !dragging_
            ? Color(static_cast<uint8_t>(fg.r > 24 ? fg.r - 24 : 0),
                    static_cast<uint8_t>(fg.g > 24 ? fg.g - 24 : 0),
                    static_cast<uint8_t>(fg.b > 24 ? fg.b - 24 : 0))
            : fg;
        fb.fill_rect(knob_x, y_, 16, height_, knob);
        fb.draw_rect(knob_x, y_, 16, height_, Color::Black(), 2);
        char value_buf[24];
        kernel::util::k_snprintf(value_buf, sizeof(value_buf), "%ld", static_cast<long>(value_));
        fb.draw_text(x_ + static_cast<int32_t>(width_) - 48, y_ - 18, value_buf, ink, Color::Black());
        if (is_focused(this)) {
            fb.draw_rect(x_ - 3, y_ - 3, width_ + 6, height_ + 6, Color(255, 255, 255), 2);
        }
    }

    bool on_event(const UIEvent& event) override {
        if (event.type == kernel::ui::EventType::TouchDown && contains(event.touch.x, event.touch.y)) {
            set_focus_widget(this);
            dragging_ = true;
            hovered_ = true;
            update_from_x(event.touch.x);
            return true;
        }
        if (event.type == kernel::ui::EventType::TouchMove) {
            const bool next_hovered = contains(event.touch.x, event.touch.y);
            if (hovered_ != next_hovered) {
                hovered_ = next_hovered;
                mark_dirty();
            }
        }
        if (event.type == kernel::ui::EventType::TouchMove && dragging_) {
            update_from_x(event.touch.x);
            return true;
        }
        if (event.type == kernel::ui::EventType::TouchUp && dragging_) {
            dragging_ = false;
            hovered_ = contains(event.touch.x, event.touch.y);
            update_from_x(event.touch.x);
            return true;
        }
        if (event.type == kernel::ui::EventType::KeyDown && is_focused(this)) {
            if (event.key.keycode == 'a' || event.key.keycode == '[' || event.key.keycode == 0x1002U) {
                value_ -= 1;
                if (value_ < min_) value_ = static_cast<int32_t>(min_);
                mark_dirty();
                return true;
            }
            if (event.key.keycode == 'd' || event.key.keycode == ']' || event.key.keycode == 0x1003U) {
                value_ += 1;
                if (value_ > max_) value_ = static_cast<int32_t>(max_);
                mark_dirty();
                return true;
            }
        }
        return false;
    }

private:
    void update_from_x(int32_t x) {
        float ratio = 0.0f;
        if (width_ > 0) ratio = static_cast<float>(x - x_) / static_cast<float>(width_);
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        value_ = static_cast<int32_t>(min_ + (max_ - min_) * ratio);
        if (bind_ == BindKind::Feed) kernel::ui::operator_api::set_feed_override(value_);
        if (bind_ == BindKind::SpindleOverride) kernel::ui::operator_api::set_spindle_override(value_);
        mark_dirty();
        if (const char* target = action_target_for_widget(spec_.id, BuilderEvent::Change, spec_.action)) {
            run_action_target(target);
        }
    }

    WidgetSpec spec_;
    float min_;
    float max_;
    BindKind bind_;
    int32_t value_;
    bool hovered_;
    bool dragging_;
};

class BuilderInput final : public Widget {
public:
    explicit BuilderInput(const WidgetSpec& spec)
        : Widget(spec.x, spec.y,
                 spec.w > 0 ? static_cast<uint32_t>(spec.w) : 220U,
                 spec.h > 0 ? static_cast<uint32_t>(spec.h) : 44U),
          spec_(spec),
          bind_(parse_bind(spec.bind)),
          focused_(false) {
        copy_field(buffer_, sizeof(buffer_), spec.text, strlen(spec.text));
    }

    void tick_feedback() {
        if (feedback_ticks_ > 0) {
            --feedback_ticks_;
            mark_dirty();
        }
    }

    void render(Framebuffer& fb) override {
        if (!focused_ && bind_ != BindKind::None) {
            format_bind_value(bind_, buffer_, sizeof(buffer_), "");
        }
        const Color bg = focused_ ? to_color(spec_.bg_color, Color::White()) : to_color(spec_.bg_color, Color(240, 240, 240));
        Color border = focused_ ? Color(38, 139, 210)
                                : (hovered_ ? Color(96, 165, 250) : to_color(spec_.border_color, Color(128, 128, 128)));
        if (feedback_ticks_ > 0) {
            border = feedback_ok_ ? Color(34, 197, 94) : Color(239, 68, 68);
        }
        const Color fg = to_color(spec_.color, Color::Black());
        fb.fill_rect(x_, y_, width_, height_, bg);
        fb.draw_rect(x_, y_, width_, height_, border, 2);
        fb.draw_text(x_ + 8, y_ + 12, buffer_[0] ? buffer_ : spec_.text, fg, bg);
        if (focused_) {
            const char* helper = helper_text_for_bind(bind_);
            if (helper && *helper) {
                fb.draw_text(x_ + 8, y_ + static_cast<int32_t>(height_) - 14, helper, Color(71, 85, 105), bg);
            }
        }
        if (is_focused(this)) {
            fb.draw_rect(x_ - 3, y_ - 3, width_ + 6, height_ + 6, Color(255, 255, 255), 2);
        }
    }

    bool on_event(const UIEvent& event) override {
        if (event.type == kernel::ui::EventType::TouchDown) {
            focused_ = contains(event.touch.x, event.touch.y);
            hovered_ = focused_;
            if (focused_) set_focus_widget(this);
            mark_dirty();
            return focused_;
        }
        if (event.type == kernel::ui::EventType::TouchMove) {
            const bool next_hovered = contains(event.touch.x, event.touch.y);
            if (hovered_ != next_hovered) {
                hovered_ = next_hovered;
                mark_dirty();
            }
        }
        if (event.type == kernel::ui::EventType::KeyDown && is_focused(this)) {
            const uint32_t key = event.key.keycode;
            size_t len = strlen(buffer_);
            if ((key == 8 || key == 127) && len > 0) {
                buffer_[len - 1] = '\0';
                if (bind_ == BindKind::MdiInput) cnc::mdi::g_service.set_input(buffer_);
                mark_dirty();
                return true;
            }
            if (key == '\r') {
                bool ok = false;
                if (const char* target = action_target_for_widget(spec_.id, BuilderEvent::Change, spec_.action)) {
                    ok = commit_input_target(target, buffer_, bind_);
                } else {
                    ok = commit_input_target(nullptr, buffer_, bind_);
                }
                feedback_ok_ = ok;
                feedback_ticks_ = kInputFeedbackTicks;
                if (ok && bind_ != BindKind::None) {
                    format_bind_value(bind_, buffer_, sizeof(buffer_), "");
                }
                focused_ = false;
                mark_dirty();
                return true;
            }
            const bool numeric_by_action = spec_.action[0] != '\0' &&
                (strncmp(spec_.action, "commit:restart", 14) == 0 ||
                 strncmp(spec_.action, "commit:cal:", 11) == 0);
            if (is_numeric_bind(bind_) || numeric_by_action) {
                const char ch = static_cast<char>(key);
                const bool has_dot = contains_char(buffer_, '.');
                const bool has_sign = buffer_[0] == '-' || buffer_[0] == '+';
                if (key >= '0' && key <= '9' && len + 1 < sizeof(buffer_)) {
                    buffer_[len] = ch;
                    buffer_[len + 1] = '\0';
                    mark_dirty();
                    return true;
                }
                if (ch == '.' && !has_dot && len + 1 < sizeof(buffer_)) {
                    buffer_[len] = ch;
                    buffer_[len + 1] = '\0';
                    mark_dirty();
                    return true;
                }
                if ((ch == '-' || ch == '+') && len == 0 && !has_sign && len + 1 < sizeof(buffer_)) {
                    buffer_[len] = ch;
                    buffer_[len + 1] = '\0';
                    mark_dirty();
                    return true;
                }
                feedback_ok_ = false;
                feedback_ticks_ = kInputFeedbackTicks / 2;
                mark_dirty();
                return true;
            }
            if ((bind_ == BindKind::ProgramName || bind_ == BindKind::MdiInput) &&
                key >= 32 && key <= 126 && len + 1 < sizeof(buffer_)) {
                buffer_[len] = static_cast<char>(key);
                buffer_[len + 1] = '\0';
                if (bind_ == BindKind::MdiInput) cnc::mdi::g_service.set_input(buffer_);
                mark_dirty();
                return true;
            }
        }
        return false;
    }

private:
    WidgetSpec spec_;
    BindKind bind_;
    char buffer_[MAX_FIELD_LEN]{};
    bool focused_;
    bool hovered_ = false;
    bool feedback_ok_ = false;
    uint32_t feedback_ticks_ = 0;
};

class BuilderImage final : public Widget {
public:
    // Orbit camera state per widget. Each GLES1 image widget has its own
    // view so operators can frame the Program page's toolpath preview and
    // the Machine View page's live scene differently. Defaults picked to
    // match the legacy hardcoded eye/center in render_live_machine.
    struct Camera {
        float yaw = 0.0f;       // radians around world-Y
        float pitch = 0.5f;     // radians from horizontal (+ looks down)
        float dist = 4.4f;      // units from center
        bool dual_channel_default = false;  // remembers which default we applied
    };

    explicit BuilderImage(const WidgetSpec& spec)
        : Widget(spec.x, spec.y,
                 spec.w > 0 ? static_cast<uint32_t>(spec.w) : 160U,
                 spec.h > 0 ? static_cast<uint32_t>(spec.h) : 120U),
          spec_(spec) {
        // Register for view:reset action targeting. Tiny static fan-out;
        // the builder resets g_gles1_widgets[] on every TSV reload via
        // reset_state, so no lifetime concern.
        if (g_gles1_widget_count < kMaxGles1Widgets) {
            g_gles1_widgets[g_gles1_widget_count++] = this;
        }
    }

    static void reset_all_cameras() {
        for (uint32_t i = 0; i < g_gles1_widget_count; ++i) {
            if (g_gles1_widgets[i]) g_gles1_widgets[i]->reset_camera();
        }
    }

    // Called from reset_state() on TSV reload — prior widget instances are
    // discarded along with the widget tree, so the static pointer table must
    // be cleared to avoid dangling refs.
    static void clear_registry() {
        for (uint32_t i = 0; i < kMaxGles1Widgets; ++i) g_gles1_widgets[i] = nullptr;
        g_gles1_widget_count = 0;
    }

    void reset_camera() {
        camera_.yaw = 0.0f;
        camera_.pitch = 0.5f;
        camera_.dist = camera_.dual_channel_default ? 6.2f : 4.4f;
        mark_dirty();
    }

    void render(Framebuffer& fb) override {
        if (strncmp(spec_.text, "gles1:", 6) == 0) {
            render_gles_preview(fb);
            return;
        }

        const Color bg = to_color(spec_.bg_color, Color(40, 40, 40));
        const Color fg = to_color(spec_.color, Color::White());
        const Color border = hovered_ ? Color(255, 255, 255) : to_color(spec_.border_color, Color::LightGray());
        fb.fill_rect(x_, y_, width_, height_, bg);
        fb.draw_rect(x_, y_, width_, height_, border, 2);
        fb.draw_line(x_, y_, x_ + static_cast<int32_t>(width_), y_ + static_cast<int32_t>(height_), border);
        fb.draw_line(x_ + static_cast<int32_t>(width_), y_, x_, y_ + static_cast<int32_t>(height_), border);
        fb.draw_text(x_ + 8, y_ + static_cast<int32_t>(height_ / 2) - 8,
                     spec_.text[0] ? spec_.text : "image", fg, bg);
    }

    bool on_event(const UIEvent& event) override {
        const bool is_gles = strncmp(spec_.text, "gles1:", 6) == 0;
        if (event.type == kernel::ui::EventType::TouchDown &&
            contains(event.touch.x, event.touch.y)) {
            if (is_gles) {
                dragging_ = true;
                drag_x_ = event.touch.x;
                drag_y_ = event.touch.y;
                return true;
            }
        }
        if (event.type == kernel::ui::EventType::TouchUp && dragging_) {
            dragging_ = false;
            return true;
        }
        if (event.type == kernel::ui::EventType::TouchMove) {
            if (dragging_ && is_gles) {
                // Drag maps pixels to radians. ~200 px across the widget ~= 2π,
                // so 0.01 rad/px gives a full orbit in a single screen-wide
                // swipe — tuned for the 1040-wide machine_view viewport.
                const float dx = static_cast<float>(event.touch.x - drag_x_) * 0.008f;
                const float dy = static_cast<float>(event.touch.y - drag_y_) * 0.008f;
                camera_.yaw += dx;
                camera_.pitch += dy;
                // Clamp pitch just shy of poles to avoid gimbal flip in the
                // look-at matrix.
                const float kLimit = 1.553f;  // ~89°
                if (camera_.pitch > kLimit) camera_.pitch = kLimit;
                if (camera_.pitch < -kLimit) camera_.pitch = -kLimit;
                drag_x_ = event.touch.x;
                drag_y_ = event.touch.y;
                mark_dirty();
                return true;
            }
            const bool next_hovered = contains(event.touch.x, event.touch.y);
            if (hovered_ != next_hovered) {
                hovered_ = next_hovered;
                mark_dirty();
            }
        }
        // Keyboard zoom only when focused elsewhere isn't meaningful here;
        // handle +/- globally via the BuilderRoot for simplicity — see the
        // root key dispatcher.
        return false;
    }

    // Exposed so BuilderRoot (which owns keyboard focus) can nudge zoom on
    // the most-recently-interacted widget. Simpler than plumbing focus into
    // the GLES1 image widget itself.
    static void zoom_all(float factor) {
        for (uint32_t i = 0; i < g_gles1_widget_count; ++i) {
            if (!g_gles1_widgets[i]) continue;
            auto& c = g_gles1_widgets[i]->camera_;
            c.dist *= factor;
            if (c.dist < 0.5f) c.dist = 0.5f;
            if (c.dist > 40.0f) c.dist = 40.0f;
            g_gles1_widgets[i]->mark_dirty();
        }
    }

private:
    static constexpr uint32_t kMaxGles1Widgets = 8;
    static BuilderImage* g_gles1_widgets[kMaxGles1Widgets];
    static uint32_t g_gles1_widget_count;

    static machine::MachineModel& machine_model() {
        alignas(machine::MachineModel) static unsigned char storage[sizeof(machine::MachineModel)];
        static bool constructed = false;
        if (!constructed) {
            auto* model = new (storage) machine::MachineModel{};
            machine::create_machine_model(*model);
            constructed = true;
        }
        return *reinterpret_cast<machine::MachineModel*>(storage);
    }

    struct SimState {
        kinematic::KinematicChain chain{};
        kinematic::MachineType type = kinematic::MachineType::Custom;
        bool loaded = false;
    };

    static SimState& sim_state() {
        alignas(SimState) static unsigned char storage[sizeof(SimState)];
        static bool constructed = false;
        if (!constructed) {
            new (storage) SimState{};
            constructed = true;
        }
        return *reinterpret_cast<SimState*>(storage);
    }

    static gles1::Renderer& preview_renderer() {
        alignas(gles1::Renderer) static unsigned char storage[sizeof(gles1::Renderer)];
        static bool constructed = false;
        if (!constructed) {
            new (storage) gles1::Renderer{};
            constructed = true;
        }
        return *reinterpret_cast<gles1::Renderer*>(storage);
    }

    // Each machine_view widget gets its own depth buffer sized to width × height
    // so overlapping geometry resolves correctly without paying for a full
    // FB-sized (8 MiB) depth allocation. Lazy alloc so widgets that never
    // render 3D don't pay anything; the buffer is reused across redraws.
    float* ensure_depth_buffer() const {
        const size_t needed = static_cast<size_t>(width_) * static_cast<size_t>(height_);
        if (needed == 0) return nullptr;
        if (depth_buffer_ && depth_capacity_ >= needed) return depth_buffer_;
        if (depth_buffer_) {
            ::operator delete(depth_buffer_);
            depth_buffer_ = nullptr;
            depth_capacity_ = 0;
        }
        depth_buffer_ = static_cast<float*>(::operator new(needed * sizeof(float)));
        depth_capacity_ = needed;
        return depth_buffer_;
    }

    gles1::FramebufferView bind_view(Framebuffer& fb) const {
        gles1::FramebufferView v{};
        v.pixels = fb.data() + static_cast<uint32_t>(y_) * kernel::ui::FB_WIDTH + static_cast<uint32_t>(x_);
        v.width = width_;
        v.height = height_;
        v.stride_pixels = kernel::ui::FB_WIDTH;
        v.depth = ensure_depth_buffer();
        v.depth_stride_pixels = width_;
        return v;
    }

    static bool axis_is_live(size_t axis_idx) {
        if (axis_idx >= motion::MAX_AXES) return false;
        const auto& axis = motion::g_motion.axis(axis_idx);
        return axis.drive != nullptr ||
               axis.enabled ||
               axis.mode != motion::Mode::None ||
               axis.target_velocity != 0 ||
               axis.actual_pos.load(std::memory_order_relaxed) != 0;
    }

    static bool channel_has_live_axes(size_t channel_idx) {
        if (channel_idx >= motion::g_motion.channel_count()) return false;
        const auto& channel = motion::g_motion.channel(channel_idx);
        for (uint8_t i = 0; i < channel.axis_count; ++i) {
            if (axis_is_live(channel.axis_indices[i])) return true;
        }
        return false;
    }

    static kinematic::MachineType choose_machine_type() {
        if (channel_has_live_axes(1)) return kinematic::MachineType::MillTurn2Channel;
        if (axis_is_live(3)) return kinematic::MachineType::Mill5Axis;
        return kinematic::MachineType::Mill3Axis;
    }

    static bool load_embedded_chain(kinematic::KinematicChain& chain, kinematic::MachineType type) {
        const char* start = nullptr;
        size_t len = 0;
        // Operator override: if an authored MX850 chain is present in the
        // VFS (typically via sdcard.img system/machine/kinematic_mx850.tsv),
        // prefer it over the built-in Mill3/MillTurn templates regardless
        // of the live motion-axis count. This lets a customer ship a 5-axis
        // machine just by dropping kinematic_mx850.tsv + its STLs onto the
        // card — no kernel rebuild required.
        if (kernel::vfs::lookup("system/machine/kinematic_mx850.tsv", start, len)
            && start && len > 0) {
            return kinematic::load_chain_from_tsv(chain, start, len);
        }
        const char* path = nullptr;
        switch (type) {
            case kinematic::MachineType::MillTurn2Channel:
                path = "system/machine/kinematic_millturn.tsv"; break;
            case kinematic::MachineType::Mill3Axis:
                path = "system/machine/kinematic_mill3.tsv"; break;
            default:
                return false;
        }
        if (!kernel::vfs::lookup(path, start, len) || !start || len == 0) return false;
        return kinematic::load_chain_from_tsv(chain, start, len);
    }

    static void ensure_sim_machine() {
        auto& sim = sim_state();
        const auto desired = choose_machine_type();
        if (sim.type == desired && sim.loaded && sim.chain.axis_count != 0) return;
        sim.chain = kinematic::KinematicChain{};
        sim.loaded = load_embedded_chain(sim.chain, desired);
        if (!sim.loaded) {
            kinematic::create_standard_machine(sim.chain, desired);
            sim.loaded = sim.chain.axis_count != 0;
        }
        sim.type = desired;
        // Apply OBJ-file meshes from the TSV's obj_file column. No-op unless
        // the machine editor has shipped OBJs embedded via the registry.
        (void)machine::apply_axis_obj_meshes(machine_model(), sim.chain);
    }

    static float motion_to_scene_units(const kinematic::AxisConfig& axis_cfg, int32_t raw_pos) {
        const float scale = 0.01f;
        float pos = static_cast<float>(raw_pos) * scale;
        if (pos < axis_cfg.travel_min) pos = axis_cfg.travel_min;
        if (pos > axis_cfg.travel_max) pos = axis_cfg.travel_max;
        return pos;
    }

    static void sync_live_axes() {
        ensure_sim_machine();
        auto& sim = sim_state();
        for (size_t i = 0; i < sim.chain.axis_count; ++i) {
            sim.chain.axes[i].position = 0.0f;
        }
        for (size_t i = 0; i < sim.chain.axis_count; ++i) {
            auto& axis_cfg = sim.chain.axes[i];
            if (axis_cfg.motion_axis < 0 ||
                static_cast<size_t>(axis_cfg.motion_axis) >= motion::MAX_AXES) {
                continue;
            }
            const int32_t raw_pos =
                motion::g_motion.axis(static_cast<size_t>(axis_cfg.motion_axis))
                    .actual_pos.load(std::memory_order_relaxed);
            axis_cfg.position = motion_to_scene_units(axis_cfg, raw_pos);
        }
        kinematic::compute_forward_kinematics(sim.chain);
    }

    static const machine::MeshPart* mesh_for_axis(const machine::MachineModel& model,
                                                  const kinematic::AxisConfig& axis,
                                                  size_t axis_idx) {
        // Imported OBJ (machine-editor-authored) takes precedence over the
        // legacy programmatic slot whenever apply_axis_obj_meshes populated a
        // mesh for this axis index.
        if (axis_idx < kinematic::MAX_AXES &&
            model.per_axis[axis_idx].vertex_count > 0 &&
            model.per_axis[axis_idx].vertices != nullptr) {
            return &model.per_axis[axis_idx];
        }
        if (strcmp(axis.name, "base") == 0) return &model.base;
        if (strcmp(axis.name, "X") == 0) return &model.x_axis;
        if (strcmp(axis.name, "Y") == 0) return &model.y_axis;
        if (strcmp(axis.name, "Z") == 0) return &model.z_axis;
        if (strcmp(axis.name, "A") == 0 || strcmp(axis.name, "B") == 0) return &model.pivot;
        if (strcmp(axis.name, "C") == 0) return &model.table;
        if (strcmp(axis.name, "spindle") == 0) return &model.spindle;
        return nullptr;
    }

    static gles1::Color4u8 wire_color_for_axis(const kinematic::AxisConfig& axis) {
        if (axis.channel == 1) return {251, 146, 60, 255};
        if (axis.type == kinematic::AxisType::Rotary) return {250, 204, 21, 255};
        if (strcmp(axis.name, "Z") == 0) return {196, 181, 253, 255};
        return {96, 165, 250, 255};
    }

    static gles1::Mat4 toolpod_station_transform(const ::machine::toolpods::Pod& pod,
                                                 const ::machine::toolpods::Station& station) {
        // Station machine_* are expressed relative to the pod's machine_*
        // anchor. With the Z-up / SolidWorks world, machine X/Y/Z map
        // straight to world X/Y/Z; no axis swap. 0.35 is a small Z lift
        // so markers sit above the table rather than coplanar with it.
        const float scale = 0.01f;
        return gles1::make_translation((pod.machine_x + station.machine_x) * scale,
                                       (pod.machine_y + station.machine_y) * scale,
                                       0.35f + (pod.machine_z + station.machine_z) * scale);
    }

    void render_toolpods(Framebuffer& fb, gles1::Renderer& renderer) {
        auto& model = machine_model();
        const auto* marker = &model.pivot;
        for (size_t pod_idx = 0; pod_idx < ::machine::toolpods::g_service.pod_count(); ++pod_idx) {
            const auto* pod = ::machine::toolpods::g_service.pod(pod_idx);
            if (!pod) continue;
            for (size_t st_idx = 0; st_idx < pod->station_count; ++st_idx) {
                const auto& station = pod->stations[st_idx];
                const bool active = st_idx == pod->active_station;
                renderer.set_model_matrix(toolpod_station_transform(*pod, station));
                renderer.set_flat_color(active ? gles1::Color4u8{74, 222, 128, 255}
                                               : gles1::Color4u8{148, 163, 184, 255});
                renderer.draw_mesh_wireframe({marker->vertices, marker->vertex_count, marker->indices, marker->index_count});
                if (active) {
                    char label[96];
                    kernel::util::k_snprintf(label, sizeof(label), "pod=%s st=%u virt=T%u",
                                             pod->id,
                                             static_cast<unsigned>(station.index),
                                             static_cast<unsigned>(station.virtual_tool));
                    fb.draw_text(x_ + 12, y_ + 78 + static_cast<int32_t>(pod_idx) * 18, label,
                                 Color(134, 239, 172), Color(17, 24, 39));
                }
            }
        }
    }

    void render_probe_calibration_overlay(Framebuffer& fb, gles1::Renderer& renderer) {
        float pocket_size = 0.0f;
        float pocket_depth = 0.0f;
        float center_x = 0.0f;
        float center_y = 0.0f;
        float floor_z = 0.0f;
        if (!::machine::g_registry.get_float("probe_pocket_nominal_mm", pocket_size) || pocket_size <= 0.0f) return;
        (void)::machine::g_registry.get_float("probe_pocket_depth_mm", pocket_depth);
        (void)::machine::g_registry.get_float("probe_center_x_mm", center_x);
        (void)::machine::g_registry.get_float("probe_center_y_mm", center_y);
        (void)::machine::g_registry.get_float("probe_floor_z_mm", floor_z);

        const float half = pocket_size * 0.5f;
        const float depth = pocket_depth > 0.0f ? -pocket_depth : floor_z;
        const float scale = 0.01f;
        // Z-up world: pocket corners sit on the X-Y plane, depth is along Z.
        gles1::Vec3f pocket[5] = {
            {(-half) * scale, (-half) * scale, depth * scale},
            {( half) * scale, (-half) * scale, depth * scale},
            {( half) * scale, ( half) * scale, depth * scale},
            {(-half) * scale, ( half) * scale, depth * scale},
            {(-half) * scale, (-half) * scale, depth * scale},
        };
        gles1::Vec3f measured[5] = {
            {(center_x - half) * scale, (center_y - half) * scale, floor_z * scale},
            {(center_x + half) * scale, (center_y - half) * scale, floor_z * scale},
            {(center_x + half) * scale, (center_y + half) * scale, floor_z * scale},
            {(center_x - half) * scale, (center_y + half) * scale, floor_z * scale},
            {(center_x - half) * scale, (center_y - half) * scale, floor_z * scale},
        };
        renderer.set_model_matrix(gles1::Mat4::identity());
        renderer.draw_polyline(pocket, 5, gles1::Color4u8{45, 212, 191, 255});
        renderer.draw_polyline(measured, 5, gles1::Color4u8{251, 191, 36, 255});
        fb.draw_text(x_ + 12, y_ + height_ - 42, "teal=nominal pocket  amber=probed datum", Color(148, 163, 184), Color(17, 24, 39));
    }

    void render_live_machine(Framebuffer& fb, gles1::Renderer& renderer, bool overlay_program) {
        sync_live_axes();
        auto& model = machine_model();
        auto& sim = sim_state();
        const bool dual_channel = sim.type == kinematic::MachineType::MillTurn2Channel;
        // Z-up / SolidWorks world. The centre-of-interest is slightly
        // above the XY ground plane to frame the bulk of the machine.
        const gles1::Vec3f center = dual_channel ? gles1::Vec3f{0.0f, 0.6f, 0.7f}
                                                 : gles1::Vec3f{0.0f, 0.0f, 0.5f};
        // First-touch defaulting: pick a distance based on machine type so
        // mill-turn frames get a wider shot than a plain mill.
        if (camera_.dual_channel_default != dual_channel) {
            camera_.dual_channel_default = dual_channel;
            if (camera_.yaw == 0.0f && camera_.pitch == 0.5f) {
                camera_.dist = dual_channel ? 6.2f : 4.4f;
            }
        }
        // Polar -> cartesian for the orbit camera. yaw rotates around the
        // Z axis (world up), pitch tilts above/below the XY plane. Eye
        // sits in the +X/-Y/+Z octant by default — SolidWorks iso.
        auto ssin = [](float x) {
            constexpr float kPi = 3.14159265358979f;
            while (x > kPi) x -= 2.0f * kPi;
            while (x < -kPi) x += 2.0f * kPi;
            const float x2 = x * x;
            return x * (1.0f - x2 / 6.0f + (x2 * x2) / 120.0f);
        };
        auto scos = [&](float x) { return ssin(x + 1.57079632679f); };
        const float horiz = scos(camera_.pitch) * camera_.dist;
        const float ez = center.z + ssin(camera_.pitch) * camera_.dist;
        // Yaw 0 puts the camera on -Y (SolidWorks Front direction).
        const gles1::Vec3f eye = {
            center.x + ssin(camera_.yaw) * horiz,
            center.y - scos(camera_.yaw) * horiz,
            ez,
        };
        renderer.set_projection_matrix(gles1::make_perspective(
            1.05f, static_cast<float>(width_) / static_cast<float>(height_ ? height_ : 1U), 0.1f, 100.0f));
        renderer.set_view_matrix(gles1::make_look_at(eye, center, {0.0f, 0.0f, 1.0f}));

        for (size_t i = 0; i < sim.chain.axis_count; ++i) {
            const auto& axis = sim.chain.axes[i];
            const auto* part = mesh_for_axis(model, axis, i);
            if (!part) continue;
            const gles1::Mat4 local_part = gles1::make_translation(part->offset_x, part->offset_y, part->offset_z);
            renderer.set_model_matrix(gles1::multiply(kinematic::get_mesh_world_transform(sim.chain, i), local_part));
            renderer.set_flat_color(wire_color_for_axis(axis));
            renderer.draw_mesh_wireframe({part->vertices, part->vertex_count, part->indices, part->index_count});
        }

        // Honour the operator's view flags. `overlay_program` is the
        // compile-time hint for the source tag (`gles1:program` always, else
        // consults the live toggle); the toolpods toggle hides the station
        // markers when the operator wants a clean chain view.
        const auto view_snap = kernel::ui::operator_api::machine_snapshot();
        const bool draw_path = overlay_program || view_snap.view_toolpath;
        if (view_snap.view_toolpods) render_toolpods(fb, renderer);
        render_probe_calibration_overlay(fb, renderer);

        if (draw_path) {
            renderer.set_model_matrix(gles1::Mat4::identity());
            for (size_t channel = 0; channel < cnc::programs::MAX_CHANNELS; ++channel) {
                const gles1::Vec3f* points = nullptr;
                const size_t count = kernel::ui::operator_api::selected_program_preview(channel, points);
                if (!points || count <= 1) continue;
                const gles1::Color4u8 color = channel == 0
                    ? gles1::Color4u8{56, 189, 248, 255}
                    : gles1::Color4u8{251, 146, 60, 255};
                renderer.draw_polyline(points, count, color);
            }
        }

        fb.draw_text(x_ + 12, y_ + 12,
                     draw_path ? "LIVE MACHINE + TOOLPATH" : "LIVE MACHINE SIM",
                     Color(226, 232, 240), Color(17, 24, 39));
        if (dual_channel) {
            fb.draw_text(x_ + 12, y_ + 34, "ch0 mill / ch1 lathe", Color(251, 191, 36), Color(17, 24, 39));
            if (draw_path) {
                fb.draw_text(x_ + 12, y_ + 56, "blue=ch0 path  orange=ch1 path", Color(148, 163, 184), Color(17, 24, 39));
            }
        }
    }

    void render_program_preview(Framebuffer& fb, gles1::Renderer& renderer) {
        render_live_machine(fb, renderer, true);
    }

    void render_gles_preview(Framebuffer& fb) {
        const Color bg = to_color(spec_.bg_color, Color(17, 24, 39));
        const Color border = hovered_ ? Color(226, 232, 240) : to_color(spec_.border_color, Color(148, 163, 184));
        fb.fill_rect(x_, y_, width_, height_, bg);
        fb.draw_rect(x_, y_, width_, height_, border, 2);

        auto& renderer = preview_renderer();
        renderer.bind_framebuffer(bind_view(fb));
        renderer.clear(0xFF111827U);

        if (strcmp(spec_.text, "gles1:machine") == 0) {
            render_live_machine(fb, renderer, false);
        } else if (strcmp(spec_.text, "gles1:program") == 0) {
            render_program_preview(fb, renderer);
        } else {
            fb.draw_text(x_ + 12, y_ + 12, "UNKNOWN GLES1 PREVIEW", Color::White(), bg);
        }
    }

    WidgetSpec spec_;
    bool hovered_ = false;
    bool dragging_ = false;
    int32_t drag_x_ = 0;
    int32_t drag_y_ = 0;
    Camera camera_{};
    // Per-widget depth buffer for the GLES1 Z-test path. Lazy-allocated by
    // ensure_depth_buffer() the first time bind_view runs; mutable so the
    // const bind_view can populate it without losing the const-call ergonomics.
    mutable float* depth_buffer_ = nullptr;
    mutable size_t depth_capacity_ = 0;
};

BuilderImage* BuilderImage::g_gles1_widgets[BuilderImage::kMaxGles1Widgets] = {};
uint32_t BuilderImage::g_gles1_widget_count = 0;

void view_reset_all_cameras() { BuilderImage::reset_all_cameras(); }
void view_zoom_all(float factor) { BuilderImage::zoom_all(factor); }

class BuilderRoot final : public Container {
public:
    BuilderRoot() : Container(0, 0, kernel::ui::FB_WIDTH, kernel::ui::FB_HEIGHT) {}

    bool on_event(const UIEvent& event) override {
        if (event.type == kernel::ui::EventType::KeyDown) {
            if (event.key.keycode == '\t' || event.key.keycode == 0x1004U) {
                advance_focus(1);
                return true;
            }
            if (event.key.keycode == 'Q' || event.key.keycode == 0x1001U) {
                advance_focus(-1);
                return true;
            }
            // Global GLES1 viewport zoom — applies to all image widgets so
            // the active view frames appropriately whether the operator is
            // on machine_view, program, or probe.
            if (event.key.keycode == '+' || event.key.keycode == '=') {
                view_zoom_all(0.85f);
                return true;
            }
            if (event.key.keycode == '-' || event.key.keycode == '_') {
                view_zoom_all(1.18f);
                return true;
            }
            if (event.key.keycode == '0') {
                view_reset_all_cameras();
                return true;
            }
        }
        return Container::on_event(event);
    }
};

Widget* create_widget(const WidgetSpec& spec) {
    Widget* widget = nullptr;
    switch (spec.type) {
        case WidgetType::Label: widget = new BuilderLabel(spec); break;
        case WidgetType::Button: widget = new BuilderButton(spec); break;
        case WidgetType::Container: widget = new BuilderContainer(spec); break;
        case WidgetType::Panel: widget = new BuilderPanel(spec); break;
        case WidgetType::Slider: widget = new BuilderSlider(spec); break;
        case WidgetType::Input: widget = new BuilderInput(spec); break;
        case WidgetType::Progress: widget = new BuilderProgress(spec); break;
        case WidgetType::Image: widget = new BuilderImage(spec); break;
        case WidgetType::Graph: widget = new BuilderGraph(spec); break;
        default: break;
    }
    if (widget && (spec.type == WidgetType::Button || spec.type == WidgetType::Slider || spec.type == WidgetType::Input)) {
        register_focusable(widget, spec.focus_order);
    }
    if (widget && spec.type == WidgetType::Input && g_input_count < MAX_WIDGETS) {
        g_inputs[g_input_count++] = static_cast<BuilderInput*>(widget);
    }
    return widget;
}

void apply_layout(int parent_idx) {
    if (parent_idx < 0 || static_cast<uint32_t>(parent_idx) >= g_widget_count) return;
    WidgetNode& parent = g_widgets[parent_idx];
    if (parent.spec.layout == LayoutType::None) return;

    LayoutBucket bucket{};
    bucket.parent = parent_idx;
    for (uint32_t i = 0; i < g_widget_count && bucket.count < MAX_LAYOUT_CHILDREN; ++i) {
        if (g_widgets[i].parent == parent_idx) {
            bucket.children[bucket.count++] = static_cast<int>(i);
        }
    }
    if (bucket.count == 0) return;

    const int32_t pad = 16;
    const int32_t gap = 12;
    int32_t cursor_x = parent.spec.x + pad;
    int32_t cursor_y = parent.spec.y + pad + (parent.spec.type == WidgetType::Panel ? 24 : 0);
    const int32_t inner_w = parent.spec.w - pad * 2;
    const int32_t inner_h = parent.spec.h - pad * 2 - (parent.spec.type == WidgetType::Panel ? 24 : 0);

    if (parent.spec.layout == LayoutType::Vertical) {
        for (uint32_t i = 0; i < bucket.count; ++i) {
            WidgetNode& child_node = g_widgets[bucket.children[i]];
            WidgetSpec& child = child_node.spec;
            child.x = cursor_x;
            if (child.w <= 0) child.w = inner_w;
            if (child.h <= 0) child.h = (child.type == WidgetType::Button) ? 56 : 28;
            if (child_node.child_align == Align::Center) {
                child.x = cursor_x + (inner_w - child.w) / 2;
            } else if (child_node.child_align == Align::Right) {
                child.x = cursor_x + inner_w - child.w;
            }
            child.y = cursor_y;
            cursor_y += child.h + gap;
        }
    } else if (parent.spec.layout == LayoutType::Horizontal) {
        const int32_t slot_w = bucket.count > 0 ? (inner_w - static_cast<int32_t>((bucket.count - 1) * gap)) / static_cast<int32_t>(bucket.count) : inner_w;
        for (uint32_t i = 0; i < bucket.count; ++i) {
            WidgetSpec& child = g_widgets[bucket.children[i]].spec;
            child.x = cursor_x;
            child.y = cursor_y;
            if (child.w <= 0) child.w = slot_w;
            if (child.h <= 0) child.h = inner_h;
            cursor_x += child.w + gap;
        }
    } else if (parent.spec.layout == LayoutType::Grid) {
        const int32_t cols = bucket.count > 1 ? 2 : 1;
        const int32_t rows = static_cast<int32_t>((bucket.count + cols - 1) / cols);
        const int32_t cell_w = (inner_w - (cols - 1) * gap) / cols;
        const int32_t cell_h = rows > 0 ? (inner_h - (rows - 1) * gap) / rows : inner_h;
        for (uint32_t i = 0; i < bucket.count; ++i) {
            WidgetSpec& child = g_widgets[bucket.children[i]].spec;
            const int32_t row = static_cast<int32_t>(i / cols);
            const int32_t col = static_cast<int32_t>(i % cols);
            child.x = cursor_x + col * (cell_w + gap);
            child.y = cursor_y + row * (cell_h + gap);
            if (child.w <= 0) child.w = cell_w;
            if (child.h <= 0) child.h = cell_h;
        }
    }
}

void reset_state() {
    g_widget_count = 0;
    g_page_count = 0;
    g_action_count = 0;
    g_action_index_count = 0;
    g_child_link_count = 0;
    g_root_widget = nullptr;
    g_active_page = 0;
    g_focusable_count = 0;
    g_focus_index = -1;
    g_input_count = 0;
    BuilderImage::clear_registry();
    clear_error();
    memset(g_widgets, 0, sizeof(g_widgets));
    memset(g_pages, 0, sizeof(g_pages));
    memset(g_actions, 0, sizeof(g_actions));
    memset(g_child_links, 0, sizeof(g_child_links));
    memset(g_focus_orders, 0, sizeof(g_focus_orders));
    memset(g_inputs, 0, sizeof(g_inputs));
}

} // namespace

WidgetSpec parse_widget(const char* record, size_t len) {
    WidgetSpec spec{};
    if (!record || len == 0 || *record == '#') return spec;

    const char* end = record + len;
    const char* type_end = record;
    while (type_end < end && *type_end != '\t' && *type_end != '\n') ++type_end;

    char type_buf[16] = {};
    const size_t type_len = static_cast<size_t>(type_end - record);
    if (type_len >= sizeof(type_buf)) return spec;
    memcpy(type_buf, record, type_len);
    type_buf[type_len] = '\0';
    if (parse_record_type(type_buf) != RecordType::Widget) return spec;
    spec.type = parse_type(type_buf);

    KeyValueField fields[24]{};
    const uint32_t field_count = parse_fields(record, len, fields, 24);
    for (uint32_t i = 0; i < field_count; ++i) {
        const char* key_buf = fields[i].key;
        const char* val_buf = fields[i].value;
        if (strcmp(key_buf, "id") == 0) copy_field(spec.id, sizeof(spec.id), val_buf, strlen(val_buf));
        else if (strcmp(key_buf, "text") == 0 || strcmp(key_buf, "title") == 0 || strcmp(key_buf, "src") == 0) {
            copy_field(spec.text, sizeof(spec.text), val_buf, strlen(val_buf));
        } else if (strcmp(key_buf, "parent") == 0) {
            copy_field(spec.parent_id, sizeof(spec.parent_id), val_buf, strlen(val_buf));
        } else if (strcmp(key_buf, "action") == 0) {
            copy_field(spec.action, sizeof(spec.action), val_buf, strlen(val_buf));
        } else if (strcmp(key_buf, "bind") == 0) {
            copy_field(spec.bind, sizeof(spec.bind), val_buf, strlen(val_buf));
        } else if (strcmp(key_buf, "focus") == 0) spec.focus_order = simple_atoi(val_buf);
        else if (strcmp(key_buf, "x") == 0) spec.x = simple_atoi(val_buf);
        else if (strcmp(key_buf, "y") == 0) spec.y = simple_atoi(val_buf);
        else if (strcmp(key_buf, "w") == 0) spec.w = simple_atoi(val_buf);
        else if (strcmp(key_buf, "h") == 0) spec.h = simple_atoi(val_buf);
        else if (strcmp(key_buf, "min") == 0) spec.min_val = simple_atof(val_buf);
        else if (strcmp(key_buf, "max") == 0) spec.max_val = simple_atof(val_buf);
        else if (strcmp(key_buf, "value") == 0) spec.value = simple_atof(val_buf);
        else if (strcmp(key_buf, "font") == 0 || strcmp(key_buf, "size") == 0) spec.font_size = parse_font_size(val_buf);
        else if (strcmp(key_buf, "color") == 0) spec.color = parse_color(val_buf);
        else if (strcmp(key_buf, "bg") == 0) spec.bg_color = parse_color(val_buf);
        else if (strcmp(key_buf, "border") == 0) spec.border_color = parse_color(val_buf);
        else if (strcmp(key_buf, "layout") == 0) spec.layout = parse_layout(val_buf);
        else if (strcmp(key_buf, "align") == 0) spec.align = parse_align(val_buf);
        else if (strcmp(key_buf, "scale") == 0) {
            const int32_t n = simple_atoi(val_buf);
            spec.text_scale = n > 0 ? static_cast<uint32_t>(n) : 1U;
        }
        else if (strcmp(key_buf, "active_if") == 0) {
            copy_field(spec.active_if, sizeof(spec.active_if), val_buf, strlen(val_buf));
        }
    }
    return spec;
}

bool build_ui(const WidgetSpec* specs, uint32_t count) {
    reset_state();
    if (!specs || count == 0 || count > MAX_WIDGETS) return false;
    for (uint32_t i = 0; i < count; ++i) {
        if (specs[i].type == WidgetType::Unknown) continue;
        if (!add_widget_to_state(specs[i], -1, 0)) return false;
    }
    if (!resolve_relationships(0)) return false;
    if (!validate_actions(0)) return false;
    build_action_index();
    for (uint32_t i = 0; i < g_widget_count; ++i) apply_layout(static_cast<int>(i));
    for (uint32_t i = 0; i < g_widget_count; ++i) g_widgets[i].widget = create_widget(g_widgets[i].spec);
    static BuilderRoot root;
    root.clear_children();
    g_root_widget = &root;
    for (uint32_t i = 0; i < g_widget_count; ++i) {
        if (!g_widgets[i].widget) continue;
        if (g_widgets[i].parent >= 0) {
            auto* parent_widget = g_widgets[g_widgets[i].parent].widget;
            if (parent_widget) static_cast<Container*>(parent_widget)->add_child(g_widgets[i].widget);
        } else {
            root.add_child(g_widgets[i].widget);
        }
    }
    return true;
}

bool load_tsv(const char* buf, size_t len) {
    reset_state();
    if (!buf || len == 0) return false;

    int current_page = -1;
    uint32_t line_no = 0;
    for (size_t offset = 0; offset < len;) {
        const char* line = buf + offset;
        const char* end = line;
        ++line_no;
        while (offset < len && *end != '\n') {
            ++end;
            ++offset;
        }
        if (offset < len && buf[offset] == '\n') ++offset;
        const size_t line_len = static_cast<size_t>(end - line);
        if (line_len == 0 || *line == '#') continue;

        char type_buf[16] = {};
        const char* type_end = line;
        while (type_end < end && *type_end != '\t' && *type_end != '\n') ++type_end;
        const size_t type_len = static_cast<size_t>(type_end - line);
        if (type_len == 0 || type_len >= sizeof(type_buf)) {
            set_error(line_no, "invalid record type");
            return false;
        }
        memcpy(type_buf, line, type_len);
        type_buf[type_len] = '\0';

        const RecordType record_type = parse_record_type(type_buf);
        if (record_type == RecordType::Unknown) {
            set_error(line_no, "unknown record type");
            return false;
        }
        if (record_type == RecordType::Page) {
            if (g_page_count >= MAX_PAGES) {
                set_error(line_no, "page limit exceeded");
                return false;
            }
            PageSpec& page = g_pages[g_page_count];
            page.first_widget = g_widget_count;
            page.widget_count = 0;
            KeyValueField fields[8]{};
            const uint32_t field_count = parse_fields(line, line_len, fields, 8);
            if (const char* id = field_value(fields, field_count, "id")) {
                copy_field(page.id, sizeof(page.id), id, strlen(id));
            }
            if (const char* title = field_value(fields, field_count, "title")) {
                copy_field(page.title, sizeof(page.title), title, strlen(title));
            }
            if (page.id[0] == '\0') {
                set_error(line_no, "page record requires id");
                return false;
            }
            if (page_has_duplicate_id(page.id)) {
                set_error(line_no, "duplicate page id");
                return false;
            }
            current_page = static_cast<int>(g_page_count++);
            continue;
        }
        if (record_type == RecordType::Include) {
            KeyValueField fields[4]{};
            const uint32_t field_count = parse_fields(line, line_len, fields, 4);
            if (!include_page_into_current(field_value(fields, field_count, "page"), current_page, line_no)) {
                return false;
            }
            continue;
        }
        if (record_type == RecordType::Child) {
            KeyValueField fields[6]{};
            const uint32_t field_count = parse_fields(line, line_len, fields, 6);
            const char* parent = field_value(fields, field_count, "parent");
            const char* widget = field_value(fields, field_count, "widget");
            Align align = parse_align(field_value(fields, field_count, "align"));
            if (!add_child_link(parent, widget, align, line_no)) return false;
            continue;
        }
        if (record_type == RecordType::Action) {
            if (g_action_count >= MAX_ACTIONS) {
                set_error(line_no, "action limit exceeded");
                return false;
            }
            ActionSpec& action = g_actions[g_action_count];
            KeyValueField fields[8]{};
            const uint32_t field_count = parse_fields(line, line_len, fields, 8);
            if (const char* widget = field_value(fields, field_count, "widget")) {
                copy_field(action.widget_id, sizeof(action.widget_id), widget, strlen(widget));
            }
            if (const char* target = field_value(fields, field_count, "target")) {
                copy_field(action.target, sizeof(action.target), target, strlen(target));
            }
            action.event = parse_event(field_value(fields, field_count, "event"));
            if (action.widget_id[0] == '\0' || action.target[0] == '\0' || action.event == BuilderEvent::None) {
                set_error(line_no, "action record requires widget, event, and target");
                return false;
            }
            ++g_action_count;
            continue;
        }
        if (record_type == RecordType::Widget) {
            WidgetSpec spec = parse_widget(line, line_len);
            if (!add_widget_to_state(spec, current_page, line_no)) return false;
        }
    }

    if (g_widget_count == 0) {
        set_error(line_no, "tsv did not define any widgets");
        return false;
    }
    if (!resolve_relationships(line_no)) return false;
    if (!validate_actions(line_no)) return false;
    build_action_index();

    for (uint32_t i = 0; i < g_widget_count; ++i) apply_layout(static_cast<int>(i));

    for (uint32_t i = 0; i < g_widget_count; ++i) {
        g_widgets[i].widget = create_widget(g_widgets[i].spec);
        if (g_widgets[i].page >= 0 && g_widgets[i].parent < 0 &&
            (g_widgets[i].spec.type == WidgetType::Panel || g_widgets[i].spec.type == WidgetType::Container) &&
            g_pages[g_widgets[i].page].root == nullptr) {
            g_pages[g_widgets[i].page].root = g_widgets[i].widget;
            if (g_pages[g_widgets[i].page].id[0] == '\0') {
                copy_field(g_pages[g_widgets[i].page].id, sizeof(g_pages[g_widgets[i].page].id),
                           g_widgets[i].spec.id, strlen(g_widgets[i].spec.id));
            }
        }
    }

    for (uint32_t i = 0; i < g_widget_count; ++i) {
        if (g_widgets[i].page >= 0 && g_widgets[i].parent < 0 &&
            g_pages[g_widgets[i].page].root != nullptr &&
            g_pages[g_widgets[i].page].root != g_widgets[i].widget) {
            for (uint32_t j = 0; j < g_widget_count; ++j) {
                if (g_widgets[j].widget == g_pages[g_widgets[i].page].root) {
                    g_widgets[i].parent = static_cast<int>(j);
                    break;
                }
            }
        }
    }

    static BuilderRoot root;
    root.clear_children();
    g_root_widget = &root;

    for (uint32_t i = 0; i < g_widget_count; ++i) {
        if (!g_widgets[i].widget) continue;
        if (g_widgets[i].parent >= 0) {
            auto* parent_widget = g_widgets[g_widgets[i].parent].widget;
            if (parent_widget) static_cast<Container*>(parent_widget)->add_child(g_widgets[i].widget);
        } else if (g_widgets[i].page < 0) {
            root.add_child(g_widgets[i].widget);
        }
    }

    for (uint32_t i = 0; i < g_page_count; ++i) {
        if (!g_pages[i].root) {
            for (uint32_t j = 0; j < g_widget_count; ++j) {
                if (g_widgets[j].page == static_cast<int>(i) && g_widgets[j].parent < 0) {
                    g_pages[i].root = g_widgets[j].widget;
                    break;
                }
            }
        }
        if (g_pages[i].root) root.add_child(g_pages[i].root);
    }

    if (g_page_count > 0) set_active_page(0);
    return g_root_widget != nullptr;
}

Widget* root_widget() {
    return g_root_widget;
}

bool set_page(const char* page_id) {
    const int idx = find_page_index_by_id(page_id);
    if (idx < 0) return false;
    set_active_page(idx);
    return true;
}

const char* active_page_id() {
    if (g_active_page < 0 || static_cast<uint32_t>(g_active_page) >= g_page_count) return "";
    return g_pages[g_active_page].id;
}

const char* last_error() {
    return g_last_error;
}

uint32_t last_error_line() {
    return g_last_error_line;
}

void lock_state() noexcept   { g_state_lock.acquire_general(); }
void unlock_state() noexcept { g_state_lock.release_general(); }

void tick() {
    for (uint32_t i = 0; i < g_input_count; ++i) {
        if (g_inputs[i]) g_inputs[i]->tick_feedback();
    }
    cnc::mdi::g_service.tick();
    if (g_root_widget) g_root_widget->mark_subtree_dirty();
}

} // namespace ui_builder
