// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/kinematic_model.hpp"

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
    if (!s) return 0.0f;
    int sign = 1;
    if (*s == '-') { sign = -1; ++s; }
    int32_t whole = 0;
    while (*s >= '0' && *s <= '9') {
        whole = whole * 10 + (*s - '0');
        ++s;
    }
    float out = static_cast<float>(whole);
    if (*s == '.') {
        ++s;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') {
            out += static_cast<float>(*s - '0') * scale;
            scale *= 0.1f;
            ++s;
        }
    }
    return sign < 0 ? -out : out;
}

AxisType parse_axis_type(const char* s) {
    if (kstrcmp(s, "Linear") == 0) return AxisType::Linear;
    if (kstrcmp(s, "Rotary") == 0) return AxisType::Rotary;
    return AxisType::Fixed;
}

void set_axis(AxisConfig& ax, const char* name, AxisType type, const char* parent_name,
              uint8_t ch, float dx, float dy, float dz,
              float ox, float oy, float oz, float tmin, float tmax,
              const char* mesh, const char* obj, int8_t motion_axis) {
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

} // namespace

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
            set_axis(chain.axes[4], "spindle",AxisType::Fixed,  "Z", 0, 0,0,0, 0,0,0.15f, 0,0, "spindle", "spindle.obj", 3);
            chain.axis_count = 5;
            chain.num_channels = 1;
            break;

        case MachineType::MillTurn2Channel:
            set_axis(chain.axes[0], "base",   AxisType::Fixed,  "-1", 0, 0,0,0, 0,0,0, 0,0, "none", nullptr, -1);
            set_axis(chain.axes[1], "X",      AxisType::Linear, "base", 0, 1,0,0, 0,0,0, 0,500, "box", "x_axis.obj", 0);
            set_axis(chain.axes[2], "Y",      AxisType::Linear, "X", 0, 0,1,0, 0,0,0, 0,400, "box", "y_axis.obj", 1);
            set_axis(chain.axes[3], "Z",      AxisType::Linear, "Y", 0, 0,0,1, 0,0,0, 0,300, "box", "z_axis.obj", 2);
            set_axis(chain.axes[4], "C",      AxisType::Rotary, "Z", 1, 0,0,1, 0,0,0.1f, 0,360, "table", "c_table.obj", 16);
            set_axis(chain.axes[5], "B",      AxisType::Rotary, "C", 1, 0,1,0, 0,0,0.15f, -120,120, "pivot", "b_pivot.obj", 17);
            set_axis(chain.axes[6], "spindle",AxisType::Fixed,  "B", 1, 0,0,0, 0,0,0.25f, 0,0, "spindle", "spindle.obj", 18);
            chain.axis_count = 7;
            chain.num_channels = 2;
            break;

        case MachineType::Mill5Axis:
            set_axis(chain.axes[0], "base",   AxisType::Fixed, "-1", 0, 0,0,0, 0,0,0, 0,0, "none", nullptr, -1);
            set_axis(chain.axes[1], "X",      AxisType::Linear, "base", 0, 1,0,0, 0,0,0, 0,1000, "box", "x_axis.obj", 0);
            set_axis(chain.axes[2], "Y",      AxisType::Linear, "X", 0, 0,1,0, 0,0,0, 0,800, "box", "y_axis.obj", 1);
            set_axis(chain.axes[3], "Z",      AxisType::Linear, "Y", 0, 0,0,1, 0,0,0, 0,500, "box", "z_axis.obj", 2);
            set_axis(chain.axes[4], "A",      AxisType::Rotary, "Z", 0, 1,0,0, 0,0,0.1f, -120,120, "pivot", "a_pivot.obj", 3);
            set_axis(chain.axes[5], "C",      AxisType::Rotary, "A", 0, 0,0,1, 0,0,0.15f, 0,360, "table", "c_table.obj", 4);
            set_axis(chain.axes[6], "spindle",AxisType::Fixed,  "C", 0, 0,0,0, 0,0,0.2f, 0,0, "spindle", "spindle.obj", 5);
            chain.axis_count = 7;
            chain.num_channels = 1;
            break;

        default:
            break;
    }
    initialize_transforms(chain);
}

void destroy_kinematic_chain(KinematicChain& chain) {
    chain.axis_count = 0;
    chain.num_channels = 1;
}

bool load_chain_from_tsv(KinematicChain& chain, const char* buf, size_t len) {
    if (!buf || len == 0) return false;
    chain = KinematicChain{};
    chain.base_transform = gles1::Mat4::identity();

    static constexpr size_t kLineBuf = 256;
    char line[kLineBuf];
    size_t pos = 0;
    while (pos < len) {
        size_t line_len = 0;
        while (pos < len && buf[pos] != '\n' && buf[pos] != '\r' && line_len + 1 < kLineBuf) {
            line[line_len++] = buf[pos++];
        }
        while (pos < len && (buf[pos] == '\n' || buf[pos] == '\r')) ++pos;
        line[line_len] = '\0';
        if (line_len == 0 || line[0] == '#') continue;
        // Accept either the legacy 14-column header (obj_file implicit/empty)
        // or the extended 15-column header the machine editor writes.
        if (kstrcmp(line, "name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis") == 0 ||
            kstrcmp(line, "name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel,motion_axis,obj_file") == 0) {
            continue;
        }
        if (chain.axis_count >= MAX_AXES) return false;

        char* fields[16]{};
        const size_t field_count = split_csv_fields(line, fields, 16);
        if (field_count < 13) return false;

        AxisConfig& axis = chain.axes[chain.axis_count];
        clear_axis(axis);
        set_axis(axis,
                 fields[0],
                 parse_axis_type(fields[1]),
                 fields[2],
                 static_cast<uint8_t>(simple_atoi(fields[12])),
                 simple_atof(fields[3]), simple_atof(fields[4]), simple_atof(fields[5]),
                 simple_atof(fields[6]), simple_atof(fields[7]), simple_atof(fields[8]),
                 simple_atof(fields[9]), simple_atof(fields[10]),
                 fields[11],
                 field_count > 14 ? fields[14] : "",
                 field_count > 13 ? static_cast<int8_t>(simple_atoi(fields[13])) : -1);
        if (axis.channel + 1 > chain.num_channels) chain.num_channels = static_cast<uint8_t>(axis.channel + 1);
        ++chain.axis_count;
    }

    for (size_t i = 0; i < chain.axis_count; ++i) {
        AxisConfig& axis = chain.axes[i];
        if (axis.parent_name[0] == '\0' ||
            (axis.parent_name[0] == '-' && axis.parent_name[1] == '1' && axis.parent_name[2] == '\0')) {
            axis.parent_index = -1;
            continue;
        }
        axis.parent_index = -1;
        for (size_t j = 0; j < chain.axis_count; ++j) {
            if (kstrcmp(axis.parent_name, chain.axes[j].name) == 0) {
                axis.parent_index = static_cast<int8_t>(j);
                break;
            }
        }
        if (axis.parent_index < 0) return false;
    }

    initialize_transforms(chain);
    return chain.axis_count != 0;
}

void update_axis_position(KinematicChain& chain, size_t axis_idx, float position) {
    if (axis_idx >= chain.axis_count) return;
    chain.axes[axis_idx].position = position;
}

void update_axis_by_name(KinematicChain& chain, const char* name, float position) {
    size_t idx = find_axis_by_name(chain, name);
    if (idx < chain.axis_count) {
        chain.axes[idx].position = position;
    }
}

size_t find_axis_by_name(const KinematicChain& chain, const char* name) {
    for (size_t i = 0; i < chain.axis_count; ++i) {
        if (name && kstrcmp(chain.axes[i].name, name) == 0) {
            return i;
        }
    }
    return chain.axis_count;
}

size_t get_channel_axis_count(const KinematicChain& chain, uint8_t channel) {
    size_t count = 0;
    for (size_t i = 0; i < chain.axis_count; ++i) {
        if (chain.axes[i].channel == channel) ++count;
    }
    return count;
}

size_t get_channel_axes(const KinematicChain& chain, uint8_t channel, size_t* axis_indices_out, size_t max_out) {
    size_t written = 0;
    for (size_t i = 0; i < chain.axis_count && written < max_out; ++i) {
        if (chain.axes[i].channel == channel) {
            axis_indices_out[written++] = i;
        }
    }
    return written;
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
            gles1::Mat4 rotation = gles1::Mat4::identity();
            if (axis.axis_direction.x != 0.0f) {
                rotation = gles1::make_rotation_x(angle);
            } else if (axis.axis_direction.y != 0.0f) {
                rotation = gles1::make_rotation_y(angle);
            } else {
                rotation = gles1::make_rotation_z(angle);
            }
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
    }
}

const gles1::Mat4& get_link_transform(const KinematicChain& chain, size_t link_idx) {
    static const gles1::Mat4 identity = gles1::Mat4::identity();
    if (link_idx >= chain.axis_count) return identity;
    return chain.transforms[link_idx].world_transform;
}

} // namespace render::kinematic
