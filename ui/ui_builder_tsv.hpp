// SPDX-License-Identifier: MIT OR Apache-2.0
// UI Builder TSV specification. Web tool exports this format,
// kernel parses at boot to construct widget tree.
//
// Format: one record per line, tab-separated
//   type    key=value    key=value ...
//
// Widget types:
//   label      text=    x= y= w= h= color= font= size=
//   button     text=    x= y= w= h= color= bg= action=
//   container  id=     x= y= w= h= bg=
//   panel      id=     x= y= w= h= bg= border= title=
//   slider     min=    max= value= x= y= w= h= bind= focus=
//   input      x=     y= w= h= maxlen= bind= focus=
//   progress   x=     y= w= h= max= color= bg= bind=
//   image      src=    x= y= w= h=
//   graph      x=     y= w= h= type= bind= max=
//
// Layout:
//   panel/container ... layout=horizontal|vertical|grid
//   child     parent=    widget=   align=left|center|right
//
// Events:
//   action    widget=    event=click|timer|change  target=page
//
// Pages:
//   page      id=     title=
//   include   page=template_page
//
// Useful extra fields:
//   align=left|center|right
//   bind=mode|alarm|prompt|cycle_progress|torque|feed|spindle|program_name|program_count|program_blocks|program_bytes|preview_points|work_name|tool_name|work_offset:x|work_offset:y|work_offset:z|work_offset:a|tool_length|tool_radius|tool_wear|axis:x|axis:y|axis:z|axis:a|macro_name|macro_active|macro_status|macro_step|macro_count|macro_message|probe_x|probe_y|probe_z|probe_done|probe_stylus|probe_center_x|probe_center_y|probe_size_x|probe_size_y|probe_shift_x|probe_shift_y|probe_shift_z|probe_sphere_ready|probe_sphere_points
//   action=page:position|page:program|page:offsets|page:ethercat|page:alarms|page:setup
//   action=demo:cycle|demo:hold|demo:reset|demo:home|demo:jog+|demo:jog-
//
// Colors: #RRGGBB or name (red, green, blue, black, white, etc)
// Font sizes: small=12, medium=16, large=24, xlarge=32
//
// Example:
// page	id=main	title=Main
// panel	id=main	x=0	y=0	w=540	h=1920	layout=vertical
//   button	text=Start	x=20	y=100	w=500	h=80	color=#2563eb	action=goto:run
//   button	text=Setup	x=20	y=200	w=500	h=80	color=#059669	action=goto:setup
//   label	text=miniOS	x=20	y=50	w=500	h=40	color=#6b7280	font=large
//   slider	min=0	max=100	value=50	x=20	y=400	w=500
//   progress	max=100	value=75	x=20	y=500	w=500
//
// Embedded via devices/embedded_ui.tsv - loaded at boot

#ifndef UI_BUILDER_TSV_HPP
#define UI_BUILDER_TSV_HPP

#include <cstdint>
#include <cstddef>

namespace kernel::ui {
class Widget;
}

namespace ui_builder {

constexpr uint32_t MAX_WIDGETS = 256;
constexpr uint32_t MAX_PAGES = 32;
constexpr uint32_t MAX_ACTIONS = 128;
constexpr uint32_t MAX_CHILD_LINKS = 256;
constexpr uint32_t MAX_FIELD_LEN = 96;

// Widget types
enum class WidgetType : uint8_t {
    Unknown = 0,
    Label,
    Button,
    Container,
    Panel,
    Slider,
    Input,
    Progress,
    Image,
    Graph,
};

// Layout types
enum class LayoutType : uint8_t {
    None = 0,
    Horizontal,
    Vertical,
    Grid,
};

enum class Align : uint8_t {
    Left = 0,
    Center,
    Right,
};

// Event types
enum class EventType : uint8_t {
    None = 0,
    Click,
    Timer,
    Change,
    TouchDown,
    TouchUp,
};

// Parse state - built from TSV
struct WidgetSpec {
    WidgetType type = WidgetType::Unknown;
    int32_t x = 0;
    int32_t y = 0;
    int32_t w = 0;
    int32_t h = 0;
    LayoutType layout = LayoutType::None;
    Align align = Align::Left;
    uint32_t color = 0xFF000000;  // ARGB
    uint32_t bg_color = 0x00000000; // 0 => use widget default
    uint32_t border_color = 0x00000000;
    uint32_t font_size = 16;
    char text[MAX_FIELD_LEN] = {};
    char id[MAX_FIELD_LEN] = {};
    char action[MAX_FIELD_LEN] = {};
    char bind[MAX_FIELD_LEN] = {};
    char parent_id[MAX_FIELD_LEN] = {};
    char active_if[MAX_FIELD_LEN] = {};  // "<bind>:<int>" -> brighten bg when binder value matches
    int32_t focus_order = 0;
    float min_val = 0;
    float max_val = 100;
    float value = 0;
    uint32_t text_scale = 1;  // Label + Button: integer pixel scale for draw_text_scaled
};

// Parse TSV record into WidgetSpec
WidgetSpec parse_widget(const char* record, size_t len);

// Build widget tree from specs - called at boot
bool build_ui(const WidgetSpec* specs, uint32_t count);
bool load_tsv(const char* buf, size_t len);
kernel::ui::Widget* root_widget();
bool set_page(const char* page_id);
const char* active_page_id();
const char* last_error();
uint32_t last_error_line();
void tick();

}

#endif // UI_BUILDER_TSV_HPP
