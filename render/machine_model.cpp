// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/machine_model.hpp"

#include "render/obj_importer.hpp"
#include "render/obj_registry.hpp"
#include "render/stl_importer.hpp"

#include <cstring>

namespace render::machine {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr size_t MAX_CUBE_VERTICES = 24;
constexpr size_t MAX_CUBE_INDICES = 36;

float sin_approx(float radians) {
    float x = radians;
    while (x > kPi) x -= 2.0f * kPi;
    while (x < -kPi) x += 2.0f * kPi;
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f + (x2 * x2) / 120.0f);
}

float cos_approx(float radians) {
    return sin_approx(radians + kPi * 0.5f);
}

void* alloc_aligned(size_t size) {
    void* ptr = ::operator new(size);
    return ptr;
}

} // namespace

void create_cube(MeshPart& part, float width, float height, float depth, gles1::Color4u8 color) {
    const float w = width * 0.5f;
    const float h = height * 0.5f;
    const float d = depth * 0.5f;

    part.vertex_count = 24;
    part.index_count = 36;
    part.color = color;

    part.vertices = static_cast<gles1::Vertex*>(alloc_aligned(part.vertex_count * sizeof(gles1::Vertex)));
    part.indices = static_cast<uint16_t*>(alloc_aligned(part.index_count * sizeof(uint16_t)));

    gles1::Vertex* v = part.vertices;
    uint16_t* idx = part.indices;

    gles1::Vec3f positions[8] = {
        {-w, -h, -d}, {w, -h, -d}, {w, h, -d}, {-w, h, -d},
        {-w, -h,  d}, {w, -h,  d}, {w, h,  d}, {-w, h,  d}
    };

    gles1::Vec3f normals[6] = {
        {0, 0, -1}, {0, 0, 1}, {-1, 0, 0},
        {1, 0, 0}, {0, -1, 0}, {0, 1, 0}
    };

    uint16_t faces[6][4] = {
        {0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1},
        {2, 6, 7, 3}, {0, 3, 7, 4}, {1, 5, 6, 2}
    };

    for (int face = 0; face < 6; ++face) {
        uint16_t base = face * 4;
        v[base + 0] = {positions[faces[face][0]], normals[face], {0, 0}, color};
        v[base + 1] = {positions[faces[face][1]], normals[face], {1, 0}, color};
        v[base + 2] = {positions[faces[face][2]], normals[face], {1, 1}, color};
        v[base + 3] = {positions[faces[face][3]], normals[face], {0, 1}, color};

        uint16_t i = face * 6;
        idx[i + 0] = base + 0; idx[i + 1] = base + 1; idx[i + 2] = base + 2;
        idx[i + 3] = base + 0; idx[i + 4] = base + 2; idx[i + 5] = base + 3;
    }
}

void create_cylinder(MeshPart& part, float radius, float height, int segments, gles1::Color4u8 color) {
    if (segments < 3) segments = 3;
    if (segments > 32) segments = 32;

    part.vertex_count = static_cast<size_t>(segments * 2 + 2);
    part.index_count = static_cast<size_t>(segments * 6);
    part.color = color;

    part.vertices = static_cast<gles1::Vertex*>(alloc_aligned(part.vertex_count * sizeof(gles1::Vertex)));
    part.indices = static_cast<uint16_t*>(alloc_aligned(part.index_count * sizeof(uint16_t)));

    const float half_h = height * 0.5f;

    for (int i = 0; i < segments; ++i) {
        float angle = (static_cast<float>(i) / static_cast<float>(segments)) * 6.28318530718f;
        float c = radius * cos_approx(angle);
        float s = radius * sin_approx(angle);

        gles1::Vec3f normal = {c / radius, 0.0f, s / radius};

        part.vertices[i] = {{c, -half_h, s}, normal, {0, 0}, color};
        part.vertices[i + segments] = {{c, half_h, s}, normal, {0, 1}, color};
    }

    part.vertices[segments * 2] = {{0, -half_h, 0}, {0, -1, 0}, {0.5, 0.5}, color};
    part.vertices[segments * 2 + 1] = {{0, half_h, 0}, {0, 1, 0}, {0.5, 0.5}, color};

    uint16_t idx = 0;
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        part.indices[idx++] = static_cast<uint16_t>(i);
        part.indices[idx++] = static_cast<uint16_t>(next);
        part.indices[idx++] = static_cast<uint16_t>(segments * 2);

        part.indices[idx++] = static_cast<uint16_t>(segments + next);
        part.indices[idx++] = static_cast<uint16_t>(segments + i);
        part.indices[idx++] = static_cast<uint16_t>(segments * 2 + 1);

        part.indices[idx++] = static_cast<uint16_t>(i);
        part.indices[idx++] = static_cast<uint16_t>(next);
        part.indices[idx++] = static_cast<uint16_t>(segments + next);

        part.indices[idx++] = static_cast<uint16_t>(i);
        part.indices[idx++] = static_cast<uint16_t>(segments + next);
        part.indices[idx++] = static_cast<uint16_t>(segments + i);
    }
}

void create_machine_model(MachineModel& model) {
    create_cube(model.base, 2.0f, 0.3f, 1.5f, {60, 60, 70, 255});

    create_cube(model.x_axis, 1.2f, 0.15f, 0.3f, {180, 80, 60, 255});
    model.x_axis.offset_x = 0.0f;
    model.x_axis.offset_y = 0.3f;
    model.x_axis.offset_z = 0.0f;

    create_cube(model.y_axis, 0.3f, 0.15f, 1.0f, {80, 160, 80, 255});
    model.y_axis.offset_x = 0.0f;
    model.y_axis.offset_y = 0.3f;
    model.y_axis.offset_z = 0.0f;

    create_cube(model.z_axis, 0.25f, 0.6f, 0.25f, {160, 80, 160, 255});
    model.z_axis.offset_x = 0.0f;
    model.z_axis.offset_y = 0.5f;
    model.z_axis.offset_z = 0.0f;

    create_cylinder(model.spindle, 0.12f, 0.3f, 16, {100, 100, 110, 255});
    model.spindle.offset_x = 0.0f;
    model.spindle.offset_y = 0.8f;
    model.spindle.offset_z = 0.0f;

    create_cube(model.table, 0.8f, 0.05f, 0.6f, {90, 90, 100, 255});
    model.table.offset_x = 0.0f;
    model.table.offset_y = 0.4f;
    model.table.offset_z = 0.0f;

    create_cube(model.pivot, 0.28f, 0.28f, 0.28f, {196, 148, 64, 255});
    model.pivot.offset_x = 0.0f;
    model.pivot.offset_y = 0.52f;
    model.pivot.offset_z = 0.0f;
}

namespace {

// Default colour for imported meshes when the TSV doesn't pick one via the
// programmatic `mesh=` slot. Used by `apply_axis_obj_meshes` so the wireframe
// still has a sensible tint for the GLES1 renderer's flat-colour path.
constexpr gles1::Color4u8 kImportedMeshColor = {148, 163, 184, 255};

// Extension sniff on the source filename so the same buffer pool can feed
// the OBJ or ASCII-STL parser depending on what the axis references. An
// empty / unrecognised name falls through to OBJ parsing to preserve the
// pre-STL behaviour.
enum class MeshFormat { Obj, StlAscii };

MeshFormat format_for_name(const char* name) {
    if (!name) return MeshFormat::Obj;
    const size_t n = std::strlen(name);
    auto ends_with_ci = [&](const char* suffix) {
        const size_t s = std::strlen(suffix);
        if (n < s) return false;
        for (size_t i = 0; i < s; ++i) {
            char a = name[n - s + i];
            char b = suffix[i];
            if (a >= 'A' && a <= 'Z') a = static_cast<char>(a + 32);
            if (b >= 'A' && b <= 'Z') b = static_cast<char>(b + 32);
            if (a != b) return false;
        }
        return true;
    };
    if (ends_with_ci(".stl")) return MeshFormat::StlAscii;
    return MeshFormat::Obj;
}

// Allocate a fresh ImportedMesh-facing scratch pool + populate a MeshPart
// from a parsed OBJ or STL. Returns false on import failure (caller leaves
// the MeshPart zeroed so the render path falls back to the programmatic
// slot).
bool import_mesh_into_meshpart(MeshPart& part, const char* text, size_t text_len,
                               MeshFormat format, gles1::Color4u8 color) {
    obj::ImportLimits limits{};
    // Per-mesh heap pools sized to the importer's advertised caps. One-off
    // allocation per axis at boot — no churn.
    auto* positions = static_cast<gles1::Vec3f*>(
        ::operator new(limits.max_positions * sizeof(gles1::Vec3f)));
    auto* normals = static_cast<gles1::Vec3f*>(
        ::operator new(limits.max_normals * sizeof(gles1::Vec3f)));
    auto* uvs = static_cast<gles1::Vec2f*>(
        ::operator new(limits.max_uvs * sizeof(gles1::Vec2f)));
    auto* vertices = static_cast<gles1::Vertex*>(
        ::operator new(limits.max_vertices * sizeof(gles1::Vertex)));
    auto* indices = static_cast<uint16_t*>(
        ::operator new(limits.max_indices * sizeof(uint16_t)));

    obj::ImportedMesh mesh{};
    mesh.positions = positions;
    mesh.normals = normals;
    mesh.uvs = uvs;
    mesh.vertices = vertices;
    mesh.indices = indices;

    obj::ImportReport report;
    if (format == MeshFormat::StlAscii) {
        report = stl::parse_ascii(text, text_len, limits, mesh);
    } else {
        report = obj::ObjImporter::parse(text, text_len, limits, mesh);
    }
    if (report.status != obj::ImportStatus::Ok || mesh.vertex_count == 0 ||
        mesh.index_count == 0) {
        ::operator delete(positions);
        ::operator delete(normals);
        ::operator delete(uvs);
        ::operator delete(vertices);
        ::operator delete(indices);
        return false;
    }

    // Scratch position/normal/UV tables are only needed during parse; the
    // final vertices array already holds baked {pos,normal,uv,color} tuples.
    ::operator delete(positions);
    ::operator delete(normals);
    ::operator delete(uvs);

    // Back-paint vertex colours so the flat-colour render path uses the
    // axis's chosen tint. ObjImporter leaves vertex.color untouched; we own
    // it here.
    for (size_t i = 0; i < mesh.vertex_count; ++i) vertices[i].color = color;

    part.vertices = vertices;
    part.vertex_count = mesh.vertex_count;
    part.indices = indices;
    part.index_count = mesh.index_count;
    part.color = color;
    part.offset_x = 0.0f;
    part.offset_y = 0.0f;
    part.offset_z = 0.0f;
    return true;
}

} // namespace

size_t apply_axis_obj_meshes(MachineModel& model, const kinematic::KinematicChain& chain) {
    size_t imported = 0;
    for (size_t i = 0; i < chain.axis_count && i < kinematic::MAX_AXES; ++i) {
        const auto& axis = chain.axes[i];
        if (axis.obj_file[0] == '\0') continue;
        const char* data = nullptr;
        size_t size = 0;
        if (!obj::lookup(axis.obj_file, data, size)) continue;
        const MeshFormat fmt = format_for_name(axis.obj_file);
        if (import_mesh_into_meshpart(model.per_axis[i], data, size, fmt,
                                      kImportedMeshColor)) {
            ++imported;
        }
    }
    return imported;
}

void destroy_machine_model(MachineModel& model) {
    if (model.base.vertices) { ::operator delete(model.base.vertices); }
    if (model.base.indices) { ::operator delete(model.base.indices); }
    if (model.x_axis.vertices) { ::operator delete(model.x_axis.vertices); }
    if (model.x_axis.indices) { ::operator delete(model.x_axis.indices); }
    if (model.y_axis.vertices) { ::operator delete(model.y_axis.vertices); }
    if (model.y_axis.indices) { ::operator delete(model.y_axis.indices); }
    if (model.z_axis.vertices) { ::operator delete(model.z_axis.vertices); }
    if (model.z_axis.indices) { ::operator delete(model.z_axis.indices); }
    if (model.spindle.vertices) { ::operator delete(model.spindle.vertices); }
    if (model.spindle.indices) { ::operator delete(model.spindle.indices); }
    if (model.table.vertices) { ::operator delete(model.table.vertices); }
    if (model.table.indices) { ::operator delete(model.table.indices); }
    if (model.pivot.vertices) { ::operator delete(model.pivot.vertices); }
    if (model.pivot.indices) { ::operator delete(model.pivot.indices); }

    for (size_t i = 0; i < kinematic::MAX_AXES; ++i) {
        if (model.per_axis[i].vertices) { ::operator delete(model.per_axis[i].vertices); }
        if (model.per_axis[i].indices) { ::operator delete(model.per_axis[i].indices); }
    }

    model = {};
}

} // namespace render::machine
