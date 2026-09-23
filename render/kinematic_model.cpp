// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/kinematic_model.hpp"

#include "miniOS.hpp"
#include "hal.hpp"
#include "util.hpp"
#include "util_math.hpp"

namespace render::kinematic {

namespace {

void copy_token(char* dst, size_t dst_size, const char* src) {
    if (!dst || dst_size == 0) return;
    size_t i = 0;
    while (src && src[i] && i + 1 < dst_size) {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = '\0';
}

int kstrcmp(const char* a, const char* b) {
    if (!a || !b) return a ? 1 : (b ? -1 : 0);
    while (*a && *b) {
        if (*a != *b) return static_cast<int>(*a) - static_cast<int>(*b);
        ++a; ++b;
    }
    return static_cast<int>(*a) - static_cast<int>(*b);
}

int32_t simple_atoi(const char* s) {
    if (!s) return 0;
    int sign = 1;
    if (*s == '-') { sign = -1; ++s; }
    int32_t out = 0;
    while (*s >= '0' && *s <= '9') {
        out = out * 10 + (*s - '0');
        ++s;
    }
    return sign * out;
}

float simple_atof(const char* s) {
    return kernel::util::parse_float(s, 0.0f);
}

AxisType parse_axis_type(const char* s) {
    if (kstrcmp(s, "Linear") == 0) return AxisType::Linear;
    if (kstrcmp(s, "Rotary") == 0) return AxisType::Rotary;
    return AxisType::Fixed;
}

void set_axis(AxisConfig& ax, const char* name, AxisType type, const char* parent_name,
              uint8_t ch, float dx, float dy, float dz,
              float ox, float oy, float oz, float tmin, float tmax,
              const char* mesh, const char* obj, int8_t motion_axis,
              float mox = 0.0f, float moy = 0.0f, float moz = 0.0f,
              float mrx = 0.0f, float mry = 0.0f, float mrz = 0.0f,
              float mscale = 1.0f) {
    copy_token(ax.name, sizeof(ax.name), name);
    ax.type = type;
    copy_token(ax.parent_name, sizeof(ax.parent_name), parent_name);
    ax.parent_index = -1;
    ax.channel = ch;
    ax.motion_axis = motion_axis;
    ax.axis_direction.x = dx;
    ax.axis_direction.y = dy;
    ax.axis_direction.z = dz;
    ax.origin_offset.x = ox;
    ax.origin_offset.y = oy;
    ax.origin_offset.z = oz;
    ax.travel_min = tmin;
    ax.travel_max = tmax;
    ax.position = 0.0f;
    copy_token(ax.mesh, sizeof(ax.mesh), mesh ? mesh : "box");
    copy_token(ax.obj_file, sizeof(ax.obj_file), obj ? obj : "");
    ax.mesh_offset = {mox, moy, moz};
    ax.mesh_rotation_deg = {mrx, mry, mrz};
    ax.mesh_scale = mscale;
}

size_t split_csv_fields(char* line, char* fields[], size_t max_fields) {
    size_t count = 0;
    char* p = line;
    while (*p && count < max_fields) {
        fields[count++] = p;
        while (*p && *p != ',') ++p;
        if (*p == ',') {
            *p = '\0';
            ++p;
        }
    }
    return count;
}

void clear_axis(AxisConfig& ax) {
    ax = AxisConfig{};
}

void initialize_transforms(KinematicChain& chain) {
    for (size_t i = 0; i < chain.axis_count; ++i) {
        chain.transforms[i].parent_index = chain.axes[i].parent_index;
    }
}

void warn_kinematic_tsv(const char* msg) {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (uart) uart->puts(msg);
}

// Recognised header versions. Returns the expected column count, or 0 if the
// line isn't a recognised header. Each version is a strict superset of the
// previous, but the column count is the contract — any data row shorter than
// `expected_cols` would silently default-fill the missing fields, which has
// historically been the parse footgun. Catch it instead of letting it slide.
size_t recognize_header(const char* line) {
    struct HeaderSpec { const char* text; size_t cols; };
    static constexpr HeaderSpec kHeaders[] = {
        {"name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis", 14},
        {"name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis,obj_file", 15},
        {"name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis,obj_file,mesh_off_x,mesh_off_y,mesh_off_z,mesh_rot_x,mesh_rot_y,mesh_rot_z", 21},
        {"name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis,obj_file,mesh_off_x,mesh_off_y,mesh_off_z,mesh_rot_x,mesh_rot_y,mesh_rot_z,mesh_scale", 22},
    };
    for (const auto& h : kHeaders) {
        if (kstrcmp(line, h.text) == 0) return h.cols;
    }
    return 0;
}

} // namespace

// Built-in templates, used only when no chain TSV resolves in the VFS.
// Units match the TSVs: millimetres and degrees, Z up.
void create_standard_machine(KinematicChain& chain, MachineType type) {
    chain = KinematicChain{};
    chain.axis_count = 0;
    chain.base_transform = gles1::Mat4::identity();
    chain.num_channels = 1;

    switch (type) {
        case MachineType::Mill3Axis:
            set_axis(chain.axes[0], "base",   AxisType::Fixed,  "-1", 0, 0,0,0, 0,0,0, 0,0, "none", nullptr, -1);
            set_axis(chain.axes[1], "X",      AxisType::Linear, "base", 0, 1,0,0, 0,0,0, 0,1000, "box", "axis_x.obj", 0);
            set_axis(chain.axes[2], "Y",      AxisType::Linear, "X", 0, 0,1,0, 0,0,0, 0,800, "box", "axis_y.obj", 1);
            set_axis(chain.axes[3], "Z",      AxisType::Linear, "Y", 0, 0,0,1, 0,0,0, 0,500, "box", "axis_z.obj", 2);
            set_axis(chain.axes[4], "spindle",AxisType::Fixed,  "Z", 0, 0,0,0, 0,0,150, 0,0, "spindle", "spindle.obj", 3);
            chain.axis_count = 5;
            chain.num_channels = 1;
            break;

        case MachineType::MillTurn2Channel:
            set_axis(chain.axes[0], "base",   AxisType::Fixed,  "-1", 0, 0,0,0, 0,0,0, 0,0, "none", nullptr, -1);
            set_axis(chain.axes[1], "X",      AxisType::Linear, "base", 0, 1,0,0, 0,0,0, 0,500, "box", "x_axis.obj", 0);
            set_axis(chain.axes[2], "Y",      AxisType::Linear, "X", 0, 0,1,0, 0,0,0, 0,400, "box", "y_axis.obj", 1);
            set_axis(chain.axes[3], "Z",      AxisType::Linear, "Y", 0, 0,0,1, 0,0,0, 0,300, "box", "z_axis.obj", 2);
            set_axis(chain.axes[4], "C",      AxisType::Rotary, "Z", 1, 0,0,1, 0,0,100, 0,360, "table", "c_table.obj", 16);
            set_axis(chain.axes[5], "B",      AxisType::Rotary, "C", 1, 0,1,0, 0,0,150, -120,120, "pivot", "b_pivot.obj", 17);
            set_axis(chain.axes[6], "spindle",AxisType::Fixed,  "B", 1, 0,0,0, 0,0,250, 0,0, "spindle", "spindle.obj", 18);
            chain.axis_count = 7;
            chain.num_channels = 2;
            break;

        case MachineType::Mill5Axis:
            set_axis(chain.axes[0], "base",   AxisType::Fixed, "-1", 0, 0,0,0, 0,0,0, 0,0, "none", nullptr, -1);
            set_axis(chain.axes[1], "X",      AxisType::Linear, "base", 0, 1,0,0, 0,0,0, 0,1000, "box", "x_axis.obj", 0);
            set_axis(chain.axes[2], "Y",      AxisType::Linear, "X", 0, 0,1,0, 0,0,0, 0,800, "box", "y_axis.obj", 1);
            set_axis(chain.axes[3], "Z",      AxisType::Linear, "Y", 0, 0,0,1, 0,0,0, 0,500, "box", "z_axis.obj", 2);
            set_axis(chain.axes[4], "A",      AxisType::Rotary, "Z", 0, 1,0,0, 0,0,100, -120,120, "pivot", "a_pivot.obj", 3);
            set_axis(chain.axes[5], "C",      AxisType::Rotary, "A", 0, 0,0,1, 0,0,150, 0,360, "table", "c_table.obj", 4);
            set_axis(chain.axes[6], "spindle",AxisType::Fixed,  "C", 0, 0,0,0, 0,0,200, 0,0, "spindle", "spindle.obj", 5);
            chain.axis_count = 7;
            chain.num_channels = 1;
            break;

        default:
            break;
    }
    initialize_transforms(chain);
}

bool load_chain_from_tsv(KinematicChain& chain, const char* buf, size_t len) {
    if (!buf || len == 0) return false;
    chain = KinematicChain{};
    chain.base_transform = gles1::Mat4::identity();

    static constexpr size_t kLineBuf = 256;
    char line[kLineBuf];
    size_t pos = 0;
    // expected_cols is the column count promised by the most recently seen
    // header. Data rows must satisfy that promise, otherwise missing fields
    // silently default-fill (the historical footgun). 0 = no header seen
    // yet, in which case rows fall back to the legacy "13+ fields" rule.
    size_t expected_cols = 0;
    size_t line_no = 0;
    auto reject = [&](const char* why, const char* detail) {
        char msg[192];
        kernel::util::k_snprintf(msg, sizeof(msg), "[kinematic] tsv line %u: %s%s%s%s\n",
                                 static_cast<unsigned>(line_no), why,
                                 detail ? " '" : "", detail ? detail : "", detail ? "'" : "");
        warn_kinematic_tsv(msg);
        chain.axis_count = 0;
        return false;
    };
    while (pos < len) {
        size_t line_len = 0;
        while (pos < len && buf[pos] != '\n' && buf[pos] != '\r' && line_len + 1 < kLineBuf) {
            line[line_len++] = buf[pos++];
        }
        line[line_len] = '\0';
        ++line_no;
        // A line that didn't end at the buffer limit used to be split into
        // two bogus rows; refuse it instead.
        if (pos < len && buf[pos] != '\n' && buf[pos] != '\r') {
            return reject("line longer than 255 chars", nullptr);
        }
        while (pos < len && (buf[pos] == '\n' || buf[pos] == '\r')) ++pos;
        if (line_len == 0 || line[0] == '#') continue;
        // Recognise the 14/15/21/22-column headers and remember their column
        // count for the row-shape contract below.
        const size_t header_cols = recognize_header(line);
        if (header_cols != 0) {
            expected_cols = header_cols;
            continue;
        }
        if (chain.axis_count >= MAX_AXES) return reject("more than MAX_AXES links", nullptr);

        char* fields[24]{};
        const size_t field_count = split_csv_fields(line, fields, 24);
        if (field_count < 13) return reject("fewer than 13 columns", nullptr);
        if (expected_cols != 0 && field_count < expected_cols) {
            return reject("fewer columns than the header promises (column drift)", nullptr);
        }

        const char* type_s = fields[1];
        if (kstrcmp(type_s, "Linear") != 0 && kstrcmp(type_s, "Rotary") != 0 &&
            kstrcmp(type_s, "Fixed") != 0) {
            return reject("unknown axis type", type_s);
        }
        const int32_t channel = simple_atoi(fields[12]);
        if (channel < 0 || channel >= static_cast<int32_t>(MAX_CHANNELS)) {
            return reject("channel out of range", fields[12]);
        }
        const int32_t motion_axis = field_count > 13 ? simple_atoi(fields[13]) : -1;
        if (motion_axis < -1 || motion_axis > 127) {
            return reject("motion_axis out of range", fields[13]);
        }

        // mesh_scale defaults to 1.0 (not 0.0) when the column is absent —
        // that's the difference between "no scaling applied" and "render
        // collapses to a point".
        AxisConfig& axis = chain.axes[chain.axis_count];
        clear_axis(axis);
        set_axis(axis,
                 fields[0],
                 parse_axis_type(type_s),
                 fields[2],
                 static_cast<uint8_t>(channel),
                 simple_atof(fields[3]), simple_atof(fields[4]), simple_atof(fields[5]),
                 simple_atof(fields[6]), simple_atof(fields[7]), simple_atof(fields[8]),
                 simple_atof(fields[9]), simple_atof(fields[10]),
                 fields[11],
                 field_count > 14 ? fields[14] : "",
                 static_cast<int8_t>(motion_axis),
                 field_count > 15 ? simple_atof(fields[15]) : 0.0f,
                 field_count > 16 ? simple_atof(fields[16]) : 0.0f,
                 field_count > 17 ? simple_atof(fields[17]) : 0.0f,
                 field_count > 18 ? simple_atof(fields[18]) : 0.0f,
                 field_count > 19 ? simple_atof(fields[19]) : 0.0f,
                 field_count > 20 ? simple_atof(fields[20]) : 0.0f,
                 field_count > 21 ? simple_atof(fields[21]) : 1.0f);

        if (axis.name[0] == '\0') return reject("empty link name", nullptr);
        for (size_t j = 0; j < chain.axis_count; ++j) {
            if (kstrcmp(chain.axes[j].name, axis.name) == 0) {
                return reject("duplicate link name", axis.name);
            }
        }
        if (axis.travel_min > axis.travel_max) return reject("min > max for", axis.name);
        if (!(axis.mesh_scale > 0.0f)) return reject("mesh_scale must be > 0 for", axis.name);
        if (axis.type != AxisType::Fixed) {
            // FK scales the linear step by |dir| and IK assumes unit axes, so
            // normalise here; a zero direction would be a joint that can't move.
            auto& d = axis.axis_direction;
            const float len2 = d.x * d.x + d.y * d.y + d.z * d.z;
            if (len2 < 1e-8f) return reject("zero axis direction for", axis.name);
            const float inv = 1.0f / kernel::util::math::sqrt_approx(len2);
            d.x *= inv; d.y *= inv; d.z *= inv;
        }
        // Parents must appear on an EARLIER row: compute_forward_kinematics is
        // a single forward pass that reads the parent's world transform, so a
        // later parent (or a cycle) would silently compose last frame's value.
        axis.parent_index = -1;
        const bool is_root = axis.parent_name[0] == '\0' ||
            (axis.parent_name[0] == '-' && axis.parent_name[1] == '1' && axis.parent_name[2] == '\0');
        if (!is_root) {
            for (size_t j = 0; j < chain.axis_count; ++j) {
                if (kstrcmp(axis.parent_name, chain.axes[j].name) == 0) {
                    axis.parent_index = static_cast<int8_t>(j);
                    break;
                }
            }
            if (axis.parent_index < 0) {
                return reject("parent not defined on an earlier row:", axis.parent_name);
            }
        }
        if (axis.channel + 1 > chain.num_channels) chain.num_channels = static_cast<uint8_t>(axis.channel + 1);
        ++chain.axis_count;
    }

    initialize_transforms(chain);
    return chain.axis_count != 0;
}

size_t find_axis_by_name(const KinematicChain& chain, const char* name) {
    for (size_t i = 0; i < chain.axis_count; ++i) {
        if (name && kstrcmp(chain.axes[i].name, name) == 0) {
            return i;
        }
    }
    return chain.axis_count;
}

void compute_forward_kinematics(KinematicChain& chain) {
    for (size_t i = 0; i < chain.axis_count; ++i) {
        const AxisConfig& axis = chain.axes[i];
        LinkTransform& transform = chain.transforms[i];
        const gles1::Mat4 origin = gles1::make_translation(
            axis.origin_offset.x,
            axis.origin_offset.y,
            axis.origin_offset.z
        );

        if (axis.type == AxisType::Fixed) {
            transform.local_transform = origin;
        } else if (axis.type == AxisType::Linear) {
            const float pos = axis.position;
            const gles1::Mat4 motion = gles1::make_translation(
                axis.axis_direction.x * pos,
                axis.axis_direction.y * pos,
                axis.axis_direction.z * pos
            );
            transform.local_transform = gles1::multiply(origin, motion);
        } else if (axis.type == AxisType::Rotary) {
            const float angle = axis.position * 0.01745329252f;
            const gles1::Mat4 rotation = gles1::make_rotation_axis_angle(axis.axis_direction, angle);
            transform.local_transform = gles1::multiply(origin, rotation);
        }

        if (axis.parent_index >= 0) {
            transform.world_transform = gles1::multiply(
                chain.transforms[axis.parent_index].world_transform,
                transform.local_transform
            );
        } else {
            transform.world_transform = gles1::multiply(chain.base_transform,
                                                        transform.local_transform);
        }

        const gles1::Mat4 mesh_t = gles1::make_translation(
            axis.mesh_offset.x, axis.mesh_offset.y, axis.mesh_offset.z);
        const gles1::Mat4 mesh_r = gles1::make_rotation_xyz_intrinsic_deg(
            axis.mesh_rotation_deg.x, axis.mesh_rotation_deg.y, axis.mesh_rotation_deg.z);
        // Composition is T * R * S so the picked pivot point (which lives at
        // mesh-local origin after Pick Pivot writes mesh_off = -hit) maps to
        // mesh_offset regardless of scale; the body grows / shrinks around
        // the picked feature instead of drifting off the rotation axis.
        const gles1::Mat4 mesh_s = gles1::make_scale(axis.mesh_scale);
        transform.mesh_local_transform =
            gles1::multiply(mesh_t, gles1::multiply(mesh_r, mesh_s));
    }
}

const gles1::Mat4& get_link_transform(const KinematicChain& chain, size_t link_idx) {
    static const gles1::Mat4 identity = gles1::Mat4::identity();
    if (link_idx >= chain.axis_count) return identity;
    return chain.transforms[link_idx].world_transform;
}

gles1::Mat4 get_mesh_world_transform(const KinematicChain& chain, size_t link_idx) {
    if (link_idx >= chain.axis_count) return gles1::Mat4::identity();
    return gles1::multiply(chain.transforms[link_idx].world_transform,
                           chain.transforms[link_idx].mesh_local_transform);
}

// ---- Tool pose + inverse kinematics --------------------------------------

namespace {

using gles1::Vec3f;

Vec3f v_sub(const Vec3f& a, const Vec3f& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3f v_scale(const Vec3f& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float v_dot(const Vec3f& a, const Vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float v_len(const Vec3f& a) { return kernel::util::math::sqrt_approx(v_dot(a, a)); }
Vec3f v_cross(const Vec3f& a, const Vec3f& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
// Column c of the upper 3x3 (c = 3 is the translation).
Vec3f m_col(const gles1::Mat4& m, int c) { return {m.m[c * 4], m.m[c * 4 + 1], m.m[c * 4 + 2]}; }
// R * v for the rotation part (links are rigid; base_transform is identity).
Vec3f m_rot(const gles1::Mat4& m, const Vec3f& v) {
    return {m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z,
            m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z,
            m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z};
}
// Rᵀ * v — the inverse rotation of a rigid transform.
Vec3f m_rot_t(const gles1::Mat4& m, const Vec3f& v) {
    return {v_dot(m_col(m, 0), v), v_dot(m_col(m, 1), v), v_dot(m_col(m, 2), v)};
}

bool is_ancestor_or_self(const KinematicChain& chain, int anc, int link) {
    for (int guard = 0; link >= 0 && guard <= static_cast<int>(MAX_AXES); ++guard) {
        if (link == anc) return true;
        link = chain.axes[link].parent_index;
    }
    return false;
}

int depth_of(const KinematicChain& chain, int link) {
    int d = 0;
    while (link >= 0 && d <= static_cast<int>(MAX_AXES)) {
        link = chain.axes[link].parent_index;
        ++d;
    }
    return d;
}

float clamp_joint(const AxisConfig& ax, float q) {
    if (ax.type == AxisType::Rotary && ax.travel_max - ax.travel_min >= 359.999f) {
        while (q < ax.travel_min) q += 360.0f;
        while (q >= ax.travel_min + 360.0f) q -= 360.0f;
        return q;
    }
    if (q < ax.travel_min) return ax.travel_min;
    if (q > ax.travel_max) return ax.travel_max;
    return q;
}

// Orientation rows are weighted by this length so a unit-vector error
// competes with millimetres of position error.
constexpr float kAxisWeightMm = 100.0f;

float pose_cost(const ToolPose& p, const ToolPose& target, float& pos_err, float& axis_err) {
    pos_err = v_len(v_sub(target.position, p.position));
    axis_err = v_len(v_sub(target.axis, p.axis));
    const float wa = axis_err * kAxisWeightMm;
    return pos_err * pos_err + wa * wa;
}

// Solve the n×n system A x = b in place (Gaussian elimination, partial
// pivoting). A is row-major with stride MAX_AXES. Returns false if singular.
bool solve_dense(float (&A)[MAX_AXES][MAX_AXES], float (&b)[MAX_AXES], size_t n) {
    for (size_t col = 0; col < n; ++col) {
        size_t piv = col;
        float best = A[col][col] < 0 ? -A[col][col] : A[col][col];
        for (size_t r = col + 1; r < n; ++r) {
            const float v = A[r][col] < 0 ? -A[r][col] : A[r][col];
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-20f) return false;
        if (piv != col) {
            for (size_t c = 0; c < n; ++c) { const float t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
            const float t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (size_t r = col + 1; r < n; ++r) {
            const float f = A[r][col] / A[col][col];
            if (f == 0.0f) continue;
            for (size_t c = col; c < n; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    for (size_t i = n; i-- > 0;) {
        float s = b[i];
        for (size_t c = i + 1; c < n; ++c) s -= A[i][c] * b[c];
        b[i] = s / A[i][i];
    }
    return true;
}

} // namespace

ToolFrames find_tool_frames(const KinematicChain& chain) {
    ToolFrames f{};
    if (chain.axis_count == 0) return f;
    size_t tool = find_axis_by_name(chain, "spindle");
    if (tool >= chain.axis_count) tool = find_axis_by_name(chain, "Z");
    if (tool >= chain.axis_count) tool = chain.axis_count - 1;
    f.tool_link = static_cast<int8_t>(tool);

    int work = -1;
    int work_depth = -1;
    for (size_t i = 0; i < chain.axis_count; ++i) {
        if (is_ancestor_or_self(chain, static_cast<int>(i), static_cast<int>(tool))) continue;
        const int d = depth_of(chain, static_cast<int>(i));
        if (d > work_depth) { work_depth = d; work = static_cast<int>(i); }
    }
    if (work < 0) {
        // Every link is on the tool path: the work is the root of that path.
        work = static_cast<int>(tool);
        while (chain.axes[work].parent_index >= 0) work = chain.axes[work].parent_index;
    }
    f.work_link = static_cast<int8_t>(work);
    return f;
}

ToolPose compute_tool_pose(const KinematicChain& chain, const ToolFrames& frames) {
    ToolPose p{};
    if (frames.tool_link < 0 || frames.work_link < 0 ||
        static_cast<size_t>(frames.tool_link) >= chain.axis_count ||
        static_cast<size_t>(frames.work_link) >= chain.axis_count) {
        return p;
    }
    const gles1::Mat4& T = chain.transforms[frames.tool_link].world_transform;
    const gles1::Mat4& W = chain.transforms[frames.work_link].world_transform;
    p.position = m_rot_t(W, v_sub(m_col(T, 3), m_col(W, 3)));
    p.axis = m_rot_t(W, m_col(T, 2));
    const float len = v_len(p.axis);
    if (len > 1e-6f) p.axis = v_scale(p.axis, 1.0f / len);
    return p;
}

IkResult solve_ik(KinematicChain& chain, const ToolFrames& frames, const ToolPose& target,
                  int max_iterations) {
    IkResult res{};
    if (frames.tool_link < 0 || frames.work_link < 0 ||
        static_cast<size_t>(frames.tool_link) >= chain.axis_count ||
        static_cast<size_t>(frames.work_link) >= chain.axis_count) {
        return res;
    }

    // Joints that move the tool relative to the work: +1 on the tool path,
    // -1 on the work path (moving the work is the inverse motion), 0 when
    // shared by both.
    size_t joint[MAX_AXES];
    float sign[MAX_AXES];
    size_t n = 0;
    for (size_t i = 0; i < chain.axis_count; ++i) {
        chain.axes[i].position = clamp_joint(chain.axes[i], chain.axes[i].position);
        if (chain.axes[i].type == AxisType::Fixed) continue;
        const int s = (is_ancestor_or_self(chain, static_cast<int>(i), frames.tool_link) ? 1 : 0) -
                      (is_ancestor_or_self(chain, static_cast<int>(i), frames.work_link) ? 1 : 0);
        if (s == 0) continue;
        joint[n] = i;
        sign[n] = static_cast<float>(s);
        ++n;
    }

    compute_forward_kinematics(chain);
    float cost = pose_cost(compute_tool_pose(chain, frames), target,
                           res.position_error, res.axis_error);
    float lambda = 1e-3f;
    constexpr float kDegToRad = 0.01745329252f;

    for (int it = 0; it < max_iterations; ++it) {
        res.iterations = it;
        if (res.position_error < 2e-3f && res.axis_error < 2e-5f) {
            res.converged = true;
            return res;
        }
        if (n == 0) break;

        // Analytic Jacobian in the work frame; rows 0-2 position (mm per
        // unit), rows 3-5 tool axis weighted by kAxisWeightMm.
        const gles1::Mat4& T = chain.transforms[frames.tool_link].world_transform;
        const gles1::Mat4& W = chain.transforms[frames.work_link].world_transform;
        const Vec3f P = m_col(T, 3);
        const Vec3f Z = m_col(T, 2);
        float J[6][MAX_AXES];
        for (size_t k = 0; k < n; ++k) {
            const AxisConfig& ax = chain.axes[joint[k]];
            const gles1::Mat4& J_world = chain.transforms[joint[k]].world_transform;
            const Vec3f a = m_rot(J_world, ax.axis_direction);
            Vec3f dp{}, dz{};
            if (ax.type == AxisType::Linear) {
                dp = a;
            } else {
                const Vec3f w = v_scale(a, kDegToRad);
                dp = v_cross(w, v_sub(P, m_col(J_world, 3)));
                dz = v_cross(w, Z);
            }
            dp = m_rot_t(W, v_scale(dp, sign[k]));
            dz = m_rot_t(W, v_scale(dz, sign[k] * kAxisWeightMm));
            J[0][k] = dp.x; J[1][k] = dp.y; J[2][k] = dp.z;
            J[3][k] = dz.x; J[4][k] = dz.y; J[5][k] = dz.z;
        }
        const ToolPose cur = compute_tool_pose(chain, frames);
        const Vec3f ep = v_sub(target.position, cur.position);
        const Vec3f ez = v_scale(v_sub(target.axis, cur.axis), kAxisWeightMm);
        const float e[6] = {ep.x, ep.y, ep.z, ez.x, ez.y, ez.z};

        float JtJ[MAX_AXES][MAX_AXES];
        float Jte[MAX_AXES];
        float diag_max = 0.0f;
        for (size_t r = 0; r < n; ++r) {
            for (size_t c = 0; c < n; ++c) {
                float s = 0.0f;
                for (int k = 0; k < 6; ++k) s += J[k][r] * J[k][c];
                JtJ[r][c] = s;
            }
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += J[k][r] * e[k];
            Jte[r] = s;
            if (JtJ[r][r] > diag_max) diag_max = JtJ[r][r];
        }

        // Levenberg-Marquardt: shrink the damping after a step that lowers
        // the cost, grow it and retry after one that doesn't.
        float saved[MAX_AXES];
        for (size_t k = 0; k < n; ++k) saved[k] = chain.axes[joint[k]].position;
        bool improved = false;
        for (int attempt = 0; attempt < 8 && !improved; ++attempt) {
            float A[MAX_AXES][MAX_AXES];
            float dq[MAX_AXES];
            for (size_t r = 0; r < n; ++r) {
                for (size_t c = 0; c < n; ++c) A[r][c] = JtJ[r][c];
                A[r][r] += lambda * (diag_max + 1e-6f);
                dq[r] = Jte[r];
            }
            if (!solve_dense(A, dq, n)) { lambda *= 10.0f; continue; }
            // Cap the step so one iteration can't swing a rotary by more
            // than 30 deg; scale the whole step to keep its direction.
            float scale = 1.0f;
            for (size_t k = 0; k < n; ++k) {
                if (chain.axes[joint[k]].type != AxisType::Rotary) continue;
                const float mag = dq[k] < 0 ? -dq[k] : dq[k];
                if (mag * scale > 30.0f) scale = 30.0f / mag;
            }
            for (size_t k = 0; k < n; ++k) {
                AxisConfig& ax = chain.axes[joint[k]];
                ax.position = clamp_joint(ax, saved[k] + dq[k] * scale);
            }
            compute_forward_kinematics(chain);
            float pe = 0.0f, ae = 0.0f;
            const float new_cost = pose_cost(compute_tool_pose(chain, frames), target, pe, ae);
            if (new_cost < cost) {
                cost = new_cost;
                res.position_error = pe;
                res.axis_error = ae;
                lambda = lambda * 0.3f < 1e-7f ? 1e-7f : lambda * 0.3f;
                improved = true;
            } else {
                for (size_t k = 0; k < n; ++k) chain.axes[joint[k]].position = saved[k];
                lambda *= 10.0f;
            }
        }
        if (!improved) {
            compute_forward_kinematics(chain);
            break;   // stuck (limit or local minimum)
        }
    }
    res.converged = res.position_error < 2e-3f && res.axis_error < 2e-5f;
    return res;
}

} // namespace render::kinematic
