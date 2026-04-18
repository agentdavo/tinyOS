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

// MachineModel holds both the legacy programmatic slots (base/x_axis/…) AND
// a per-axis slot indexed by the axis's position in the KinematicChain. When
// a TSV axis specifies `obj_file=<name>`, that name is looked up in the
// render::obj registry, parsed by ObjImporter, and stored in per_axis[i].
// The GLES1 render path (draw_mesh_wireframe / solid) consumes both kinds
// identically — MeshPart carries a plain {vertices,indices} pair.
struct MachineModel {
    MeshPart base;
    MeshPart x_axis;
    MeshPart y_axis;
    MeshPart z_axis;
    MeshPart spindle;
    MeshPart table;
    MeshPart pivot;
    MeshPart per_axis[kinematic::MAX_AXES]{};
};

void create_cube(MeshPart& part, float width, float height, float depth, gles1::Color4u8 color);
void create_cylinder(MeshPart& part, float radius, float height, int segments, gles1::Color4u8 color);
void create_machine_model(MachineModel& model);

// After `create_machine_model` has populated the programmatic slots, walk
// the given kinematic chain and for every axis that has a non-empty
// `obj_file` field, look up the embedded OBJ blob via render::obj::lookup,
// parse it with render::obj::ObjImporter, and populate model.per_axis[i]
// with the resulting mesh. Returns the number of OBJ meshes successfully
// imported (0 if none requested). Failures are silent in release builds
// and fall back to the legacy programmatic slot; boot should never crash
// because an OBJ is malformed or missing.
size_t apply_axis_obj_meshes(MachineModel& model, const kinematic::KinematicChain& chain);

void destroy_machine_model(MachineModel& model);

} // namespace render::machine

#endif
