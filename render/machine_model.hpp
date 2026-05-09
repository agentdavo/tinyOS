// SPDX-License-Identifier: MIT OR Apache-2.0
// Simple machine model mesh data for 3-axis mill visualization.

#ifndef RENDER_MACHINE_MODEL_HPP
#define RENDER_MACHINE_MODEL_HPP

#include "render/gles1.hpp"
#include "render/kinematic_model.hpp"  // MAX_AXES, KinematicChain
#include <cstddef>
#include <cstdint>

namespace render::machine {

struct MeshPart {
    gles1::Vertex* vertices = nullptr;
    size_t vertex_count = 0;
    uint16_t* indices = nullptr;
    size_t index_count = 0;
    gles1::Color4u8 color;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
};

// MachineModel holds one MeshPart per kinematic axis. populate_axis_meshes
// walks the chain and fills each slot, preferring an OBJ/STL blob from the
// render::obj registry when the axis's `obj_file` resolves and falling back
// to a name-keyed primitive (cube / cylinder) otherwise. The GLES1 render
// path (draw_mesh_wireframe / solid) consumes MeshPart's plain
// {vertices, indices} pair regardless of source.
struct MachineModel {
    MeshPart per_axis[kinematic::MAX_AXES]{};
};

void create_cube(MeshPart& part, float width, float height, float depth, gles1::Color4u8 color);
void create_cylinder(MeshPart& part, float radius, float height, int segments, gles1::Color4u8 color);

// Populate per_axis[] for every axis in `chain`. For each axis:
//  1. If `axis.obj_file` is non-empty and the registry lookup succeeds,
//     parse it (OBJ or STL by extension) and store into per_axis[i].
//  2. Otherwise generate a name-keyed primitive (cube / cylinder) so the
//     axis still renders. "none" mesh hints produce an empty slot.
// Returns the number of axes that ended up with a renderable mesh.
size_t populate_axis_meshes(MachineModel& model, const kinematic::KinematicChain& chain);

// Convenience marker mesh (a small unit cube) shared by overlay paths
// that need a generic indicator — toolpod stations, probe markers, etc.
// Lazy-allocated on first call and reused; not destroyed until the kernel
// exits (which never happens). Not part of MachineModel — independent
// scratch geometry.
const MeshPart& marker_mesh();

void destroy_machine_model(MachineModel& model);

} // namespace render::machine

#endif
