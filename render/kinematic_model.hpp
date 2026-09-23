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
// Per-channel headroom: a 5-axis mill needs 6 (X/Y/Z/A/C + spindle), and a
// MillTurn lathe channel adds at least 3 (X/Z/C + sub-spindle). 8 covers
// both with a comfortable margin for an extra carriage / tailstock without
// spilling into the other channel's budget.
constexpr size_t MAX_AXES_PER_CHANNEL = 8;
// Total flat-array cap. With 2 channels × 8 axes per channel we get 16,
// which is room for a 5+5+spindles MillTurn or two independent 5-axis
// mills. Each AxisConfig + LinkTransform pair is ~340 B, so the bump from
// 12 → 16 costs ~1.4 KiB per chain — trivial.
constexpr size_t MAX_AXES = MAX_AXES_PER_CHANNEL * MAX_CHANNELS;

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
bool load_chain_from_tsv(KinematicChain& chain, const char* buf, size_t len);

void compute_forward_kinematics(KinematicChain& chain);
const gles1::Mat4& get_link_transform(const KinematicChain& chain, size_t link_idx);
gles1::Mat4 get_mesh_world_transform(const KinematicChain& chain, size_t link_idx);
size_t find_axis_by_name(const KinematicChain& chain, const char* name);

// Motion counts per model unit (mm or deg) for mapping live axis positions
// onto the chain. Matches cnc/interpreter.cpp's units_to_counts (100/mm).
// NOTE: the operator DRO bindings in ui/ divide by 1000 instead — the two
// disagree and are not reconciled yet.
constexpr float kMotionCountsPerUnit = 100.0f;

// ---- Tool pose + inverse kinematics --------------------------------------
// Chains are authored in millimetres and degrees, Z up. A machine is two
// branches off a common root: the tool side ends at the tool link, the work
// side at the work link (the root when the table is not moved by any axis).
// The tool pose is the tool link's origin and +Z axis, expressed in the work
// link's frame. So table-table, head-head and table-head machines use the
// same round trip.
struct ToolFrames {
    int8_t tool_link = -1;
    int8_t work_link = -1;
};

struct ToolPose {
    gles1::Vec3f position{};   // mm, in the work link's frame
    gles1::Vec3f axis{};       // unit, in the work link's frame
};

struct IkResult {
    bool converged = false;
    float position_error = 0.0f;   // mm
    float axis_error = 0.0f;       // |axis - target axis| (≈ radians)
    int iterations = 0;
};

// Tool link = a link named "spindle", else "Z", else the last link. Work link
// = the deepest link that is not on the tool link's ancestor path, else the
// root.
ToolFrames find_tool_frames(const KinematicChain& chain);
// Reads the transforms from the last compute_forward_kinematics().
ToolPose compute_tool_pose(const KinematicChain& chain, const ToolFrames& frames);
// Damped least squares with an analytic Jacobian. Seeds from the chain's
// current axis positions, keeps every joint inside [travel_min, travel_max]
// (rotaries whose span is >= 360 deg wrap instead of clamping), and leaves
// the chain at the solution with FK up to date.
IkResult solve_ik(KinematicChain& chain, const ToolFrames& frames, const ToolPose& target,
                  int max_iterations = 64);

} // namespace render::kinematic

#endif
