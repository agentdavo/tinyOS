// SPDX-License-Identifier: MIT OR Apache-2.0
// Kinematic model for machine tool visualization.
// Supports configurable axis types, parent-child relationships, and multi-channel machines.
// Each axis can reference an OBJ mesh file for 3D rendering.

#ifndef RENDER_KINEMATIC_MODEL_HPP
#define RENDER_KINEMATIC_MODEL_HPP

#include "render/gles1.hpp"
#include <cstdint>
#include <cstddef>

namespace render::kinematic {

constexpr size_t MAX_CHANNELS = 2;
constexpr size_t MAX_AXES = 12;

enum class AxisType : uint8_t {
    Linear,
    Rotary,
    Fixed
};

struct AxisConfig {
    char name[16]{};
    AxisType type = AxisType::Fixed;
    char parent_name[16]{};
    int8_t parent_index = -1;
    uint8_t channel = 0;
    int8_t motion_axis = -1;
    gles1::Vec3f axis_direction{};
    gles1::Vec3f origin_offset{};
    float travel_min = 0.0f;
    float travel_max = 1.0f;
    float position = 0.0f;
    char mesh[16]{};
    char obj_file[32]{};
    // URDF-style separation: the joint frame (origin_offset + axis_direction)
    // is where motion happens; the visual mesh sits at this offset/orientation
    // relative to the joint frame, so a CAD mesh whose geometric centre is not
    // on the rotation axis can be reseated without dragging the kinematics.
    gles1::Vec3f mesh_offset{};
    gles1::Vec3f mesh_rotation_deg{};
    float mesh_scale = 1.0f;
};

struct LinkTransform {
    gles1::Mat4 local_transform{};
    gles1::Mat4 world_transform{};
    gles1::Mat4 mesh_local_transform{};
    int8_t parent_index = -1;
};

struct KinematicChain {
    AxisConfig axes[MAX_AXES];
    size_t axis_count = 0;
    LinkTransform transforms[MAX_AXES];
    gles1::Mat4 base_transform{};
    uint8_t num_channels = 1;
};

enum class MachineType : uint8_t {
    Mill3Axis,
    MillTurn2Channel,
    Mill5Axis,
    Custom
};

void create_standard_machine(KinematicChain& chain, MachineType type);
void destroy_kinematic_chain(KinematicChain& chain);
bool load_chain_from_tsv(KinematicChain& chain, const char* buf, size_t len);

void update_axis_position(KinematicChain& chain, size_t axis_idx, float position);
void update_axis_by_name(KinematicChain& chain, const char* name, float position);
void compute_forward_kinematics(KinematicChain& chain);
const gles1::Mat4& get_link_transform(const KinematicChain& chain, size_t link_idx);
const gles1::Mat4& get_mesh_local_transform(const KinematicChain& chain, size_t link_idx);
gles1::Mat4 get_mesh_world_transform(const KinematicChain& chain, size_t link_idx);
size_t find_axis_by_name(const KinematicChain& chain, const char* name);

size_t get_channel_axis_count(const KinematicChain& chain, uint8_t channel);
size_t get_channel_axes(const KinematicChain& chain, uint8_t channel, size_t* axis_indices_out, size_t max_out);

} // namespace render::kinematic

#endif
