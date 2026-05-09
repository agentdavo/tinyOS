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

// Per-mesh scratch arena: positions / normals / UVs are only needed during
// parse; once the importer has baked them into vertices the scratch tables
// are dead weight. Share one global scratch so we don't churn through
// ~32 KiB of small allocations on every import. Lazy-initialised on first
// use; never freed (kernel-lifetime).
struct ScratchArena {
    gles1::Vec3f* positions = nullptr;
    gles1::Vec3f* normals = nullptr;
    gles1::Vec2f* uvs = nullptr;
};

ScratchArena& global_scratch() {
    static ScratchArena s{};
    if (!s.positions) {
        const obj::ImportLimits limits{};
        s.positions = static_cast<gles1::Vec3f*>(
            ::operator new(limits.max_positions * sizeof(gles1::Vec3f)));
        s.normals = static_cast<gles1::Vec3f*>(
            ::operator new(limits.max_normals * sizeof(gles1::Vec3f)));
        s.uvs = static_cast<gles1::Vec2f*>(
            ::operator new(limits.max_uvs * sizeof(gles1::Vec2f)));
    }
    return s;
}

// Persistent block: vertices and indices share one allocation so MeshPart
// holds one logical mesh as one heap block. Size-prefixed layout
// [vertices ... | indices ...]; the part's `vertices` pointer is also the
// free pointer at destroy time. sizeof(Vertex) is a multiple of 4 so the
// indices region is naturally 2-byte aligned.
void* allocate_mesh_block(size_t vertex_count, size_t index_count,
                          gles1::Vertex*& vertices_out, uint16_t*& indices_out) {
    const size_t verts_size = vertex_count * sizeof(gles1::Vertex);
    const size_t idx_size = index_count * sizeof(uint16_t);
    void* block = ::operator new(verts_size + idx_size);
    vertices_out = static_cast<gles1::Vertex*>(block);
    indices_out = reinterpret_cast<uint16_t*>(
        static_cast<uint8_t*>(block) + verts_size);
    return block;
}

// Populate a MeshPart from a parsed OBJ or STL. Returns false on import
// failure (caller leaves the MeshPart zeroed so the render path falls back
// to the programmatic slot). Persistent storage is one allocation per mesh
// instead of the previous five — destroy_machine_model frees vertices to
// reclaim both vertices and indices in one shot.
bool import_mesh_into_meshpart(MeshPart& part, const char* text, size_t text_len,
                               MeshFormat format, gles1::Color4u8 color) {
    obj::ImportLimits limits{};
    ScratchArena& scratch = global_scratch();

    gles1::Vertex* vertices = nullptr;
    uint16_t* indices = nullptr;
    (void)allocate_mesh_block(limits.max_vertices, limits.max_indices,
                              vertices, indices);

    obj::ImportedMesh mesh{};
    mesh.positions = scratch.positions;
    mesh.normals = scratch.normals;
    mesh.uvs = scratch.uvs;
    mesh.vertices = vertices;
    mesh.indices = indices;

    obj::ImportReport report;
    if (format == MeshFormat::StlAscii) {
        // stl::parse auto-dispatches ASCII vs binary by checking whether the
        // buffer's size matches the binary record formula. Binary STL is the
        // dominant CAD-export flavour, so the .stl extension shouldn't pin
        // the parser to ASCII only.
        report = stl::parse(text, text_len, limits, mesh);
    } else {
        report = obj::ObjImporter::parse(text, text_len, limits, mesh);
    }
    if (report.status != obj::ImportStatus::Ok || mesh.vertex_count == 0 ||
        mesh.index_count == 0) {
        ::operator delete(vertices);
        return false;
    }

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
    // Legacy programmatic slots still allocate vertices and indices as two
    // separate blocks (create_cube / create_cylinder do that explicitly).
    // OBJ-imported per_axis slots use the new arena layout where indices
    // live in the same allocation as vertices — freeing only `vertices`
    // reclaims both. We can tell them apart by whether `indices` lies
    // inside the vertices block: if so, only delete vertices.
    auto free_part = [](MeshPart& p) {
        if (!p.vertices) return;
        const auto* verts_end = reinterpret_cast<const uint8_t*>(
            p.vertices + p.vertex_count);
        const auto* idx_start = reinterpret_cast<const uint8_t*>(p.indices);
        const bool indices_share_block =
            idx_start >= reinterpret_cast<const uint8_t*>(p.vertices) &&
            idx_start <= verts_end;
        ::operator delete(p.vertices);
        if (!indices_share_block && p.indices) ::operator delete(p.indices);
        p = MeshPart{};
    };

    free_part(model.base);
    free_part(model.x_axis);
    free_part(model.y_axis);
    free_part(model.z_axis);
    free_part(model.spindle);
    free_part(model.table);
    free_part(model.pivot);
    for (size_t i = 0; i < kinematic::MAX_AXES; ++i) free_part(model.per_axis[i]);

    model = {};
}

} // namespace render::machine
