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

int32_t text_x_for_align(int32_t x, uint32_t width, const char* text, Align align) {
    if (!text || align == Align::Left || width == 0) return x;
    const int32_t text_w = static_cast<int32_t>(strlen(text) * 8U);
    if (align == Align::Center) return x + static_cast<int32_t>(width / 2) - text_w / 2;
    return x + static_cast<int32_t>(width) - text_w - 8;
}

enum class BindKind : uint8_t {
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
    return BindKind::None;
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
        case BindKind::ProbeShiftZ: {
            const float value = static_cast<float>(bound_numeric_value(bind)) / 1000.0f;
            kernel::util::k_snprintf(buf, buf_size, "%s%.3f", prefix ? prefix : "", static_cast<double>(value));
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
    else if (strcmp(target, "view:toolpath:toggle") == 0) toggle_view_toolpath();
    else if (strcmp(target, "view:toolpods:toggle") == 0) toggle_view_toolpods();
    else if (strcmp(target, "view:reset") == 0) view_reset_all_cameras();
    else if (strcmp(target, "view:zoom:in") == 0) view_zoom_all(0.85f);
    else if (strcmp(target, "view:zoom:out") == 0) view_zoom_all(1.18f);
    else if (strcmp(target, "demo:estop") == 0) {
        // Stop all interpreter channels + latch alarm. Full drive-level quickstop
        // belongs in the motion layer; this mirrors the reset path plus an
        // explicit hold so the operator sees immediate motion suspension.
        reset_alarm();
        toggle_hold();
    }
    else if (strcmp(target, "mdi:submit") == 0) {
        const auto s = cnc::mdi::g_service.snapshot();
        cnc::mdi::g_service.submit(s.input);
    }
    else if (strcmp(target, "mdi:clear") == 0) cnc::mdi::g_service.clear();
    else if (strcmp(target, "mdi:abort") == 0) cnc::mdi::g_service.abort();
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
        return cnc::mdi::g_service.submit(value_text);
    }
    if (target && *target && strcmp(target, "commit:restart:line") == 0) {
        const int32_t line_no = simple_atoi(value_text);
        if (line_no < 0) return false;
        return restart_program_at_line(static_cast<size_t>(line_no));
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
        default: return "";
    }
}

char* action_target_for_widget(const char* id, BuilderEvent event, const char* inline_action) {
    if (inline_action && *inline_action) return const_cast<char*>(inline_action);
    for (uint32_t i = 0; i < g_action_count; ++i) {
        if (g_actions[i].event == event && strcmp(g_actions[i].widget_id, id) == 0) {
            return g_actions[i].target;
        }
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
          bind_(parse_bind(spec.bind)) {}

    void render(Framebuffer& fb) override {
        if (!visible_) return;
        const char* text = spec_.text;
        char buf[MAX_FIELD_LEN] = {};
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
            fb.draw_text_scaled(draw_x, y_, text, fg_, bg_, scale);
        } else {
            fb.draw_text(draw_x, y_, text, fg_, bg_);
        }
    }

private:
    WidgetSpec spec_;
    Color fg_;
    Color bg_;
    BindKind bind_;
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
          fg_(to_color(spec.color, Color::White())) {
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

        // Parse active_if = "<bind>:<int>".
        if (spec.active_if[0] != '\0') {
            const char* colon = nullptr;
            for (const char* p = spec.active_if; *p; ++p) if (*p == ':') { colon = p; break; }
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
        const char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action);
        if (target && strncmp(target, "goto:", 5) == 0) {
            const bool is_active = find_page_index_by_id(target + 5) == g_active_page;
            if (is_active) bg = dim_bg_;
        }
        if (active_if_matches()) {
            bg = bright_bg_;
            highlight_border = true;
        }
        set_colors(bg, bg, Color(bg.r / 2, bg.g / 2, bg.b / 2), fg_);

        // If the spec asked for scaled label text, bypass the base Button
        // renderer's draw_text call by painting the box ourselves and then
        // drawing label with draw_text_scaled.
        const uint32_t scale = spec_.text_scale > 0 ? spec_.text_scale : 1u;
        if (scale > 1) {
            fb.fill_rect(x_, y_, width_, height_, bg);
            fb.draw_rect(x_, y_, width_, height_, fg_, 3);
            fb.fill_rect(x_ + 8, y_ + 8, width_ - 16, height_ - 16, bg);
            const char* label = spec_.text;
            const int32_t text_w = static_cast<int32_t>(strlen(label) * 8U * scale);
            const int32_t text_h = static_cast<int32_t>(16U * scale);
            const int32_t lx = x_ + static_cast<int32_t>(width_) / 2 - text_w / 2;
            const int32_t ly = y_ + static_cast<int32_t>(height_) / 2 - text_h / 2;
            fb.draw_text_scaled(lx, ly, label, fg_, bg, scale);
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
            if (char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action)) {
                run_action_target(target);
            }
        } else if (event.type == kernel::ui::EventType::KeyDown && is_focused(this) &&
                   (event.key.keycode == '\r' || event.key.keycode == ' ')) {
            if (char* target = action_target_for_widget(spec_.id, BuilderEvent::Click, spec_.action)) {
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
        mark_dirty();
        if (char* target = action_target_for_widget(spec_.id, BuilderEvent::Change, spec_.action)) {
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
                if (char* target = action_target_for_widget(spec_.id, BuilderEvent::Change, spec_.action)) {
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
                strncmp(spec_.action, "commit:restart", 14) == 0;
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

    gles1::FramebufferView bind_view(Framebuffer& fb) const {
        return gles1::FramebufferView{
            fb.data() + static_cast<uint32_t>(y_) * kernel::ui::FB_WIDTH + static_cast<uint32_t>(x_),
            width_,
            height_,
            kernel::ui::FB_WIDTH
        };
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
            renderer.set_model_matrix(gles1::multiply(kinematic::get_link_transform(sim.chain, i), local_part));
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

void tick() {
    for (uint32_t i = 0; i < g_input_count; ++i) {
        if (g_inputs[i]) g_inputs[i]->tick_feedback();
    }
    cnc::mdi::g_service.tick();
    if (g_root_widget) g_root_widget->mark_subtree_dirty();
}

} // namespace ui_builder
