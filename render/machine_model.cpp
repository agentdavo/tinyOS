// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/machine_model.hpp"

#include "render/obj_importer.hpp"
#include "render/obj_registry.hpp"
#include "render/stl_importer.hpp"

#include "miniOS.hpp"
#include "util.hpp"
#include "util_math.hpp"

#include <cstring>
#include <new>

namespace render::machine {

namespace {

constexpr size_t MAX_CUBE_VERTICES = 24;
constexpr size_t MAX_CUBE_INDICES = 36;

using kernel::util::math::cos_approx;
using kernel::util::math::sin_approx;

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
    // 4 triangles per segment (top cap + bottom cap + 2 sides) × 3 indices.
    // Prior `segments * 6` undersized the index buffer by 2× and the loop
    // below at lines 109-127 wrote past the end of the allocation.
    part.index_count = static_cast<size_t>(segments * 12);
    part.color = color;

    part.vertices = static_cast<gles1::Vertex*>(alloc_aligned(part.vertex_count * sizeof(gles1::Vertex)));
    part.indices = static_cast<uint16_t*>(alloc_aligned(part.index_count * sizeof(uint16_t)));

    const float half_h = height * 0.5f;

    for (int i = 0; i < segments; ++i) {
        float angle = (static_cast<float>(i) / static_cast<float>(segments)) * 6.28318530718f;
        float c = radius * cos_approx(angle);
        float s = radius * sin_approx(angle);

        gles1::Vec3f normal = {c / radius, s / radius, 0.0f};

        // Axis along Z (the world is Z-up; a spindle hangs along Z).
        part.vertices[i] = {{c, s, -half_h}, normal, {0, 0}, color};
        part.vertices[i + segments] = {{c, s, half_h}, normal, {0, 1}, color};
    }

    part.vertices[segments * 2] = {{0, 0, -half_h}, {0, 0, -1}, {0.5, 0.5}, color};
    part.vertices[segments * 2 + 1] = {{0, 0, half_h}, {0, 0, 1}, {0.5, 0.5}, color};

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

namespace {

// Name-keyed primitive defaults so a chain whose `obj_file` references
// don't resolve still renders something recognisable (X/Y/Z servo blocks,
// spindle cylinder, A/B/C rotary pivots / tables). Millimetres, Z up, like
// the chains: cubes are x/y/z extents, cylinders radius/height along Z, and
// the offset lifts the block along Z off its joint origin.
struct AxisPrimitiveSpec {
    const char* axis_name;       // Match against AxisConfig::name.
    bool is_cylinder;
    float a, b, c;               // Cube: x/y/z size. Cylinder: radius/height/(unused).
    int segments;                // Cylinder only.
    gles1::Color4u8 color;
    float offset_x, offset_y, offset_z;
};

constexpr AxisPrimitiveSpec kAxisPrimitives[] = {
    {"base",    false, 200.0f, 150.0f, 30.0f,  0,  {60, 60, 70, 255},     0.0f, 0.0f,  0.0f},
    {"X",       false, 120.0f, 30.0f,  15.0f,  0,  {180, 80, 60, 255},    0.0f, 0.0f, 30.0f},
    {"Y",       false, 30.0f,  100.0f, 15.0f,  0,  {80, 160, 80, 255},    0.0f, 0.0f, 30.0f},
    {"Z",       false, 25.0f,  25.0f,  60.0f,  0,  {160, 80, 160, 255},   0.0f, 0.0f, 50.0f},
    {"spindle", true,  12.0f,  30.0f,  0.0f,  16, {100, 100, 110, 255},  0.0f, 0.0f, 80.0f},
    {"C",       false, 80.0f,  60.0f,  5.0f,   0,  {90, 90, 100, 255},    0.0f, 0.0f, 40.0f},
    {"A",       false, 28.0f,  28.0f,  28.0f,  0,  {196, 148, 64, 255},   0.0f, 0.0f, 52.0f},
    {"B",       false, 28.0f,  28.0f,  28.0f,  0,  {196, 148, 64, 255},   0.0f, 0.0f, 52.0f},
};

// Generic fallback for any axis whose name isn't in the spec table.
constexpr gles1::Color4u8 kGenericFallbackColor = {148, 163, 184, 255};

bool axis_name_eq(const char* a, const char* b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        ++a; ++b;
    }
    return *a == *b;
}

bool generate_primitive_for_axis(MeshPart& part, const kinematic::AxisConfig& axis) {
    // "none" mesh hint (used by base / dress slots) intentionally renders
    // nothing — caller leaves the slot empty.
    if (axis_name_eq(axis.mesh, "none")) return false;
    for (const auto& spec : kAxisPrimitives) {
        if (!axis_name_eq(axis.name, spec.axis_name)) continue;
        if (spec.is_cylinder) {
            create_cylinder(part, spec.a, spec.b, spec.segments, spec.color);
        } else {
            create_cube(part, spec.a, spec.b, spec.c, spec.color);
        }
        part.offset_x = spec.offset_x;
        part.offset_y = spec.offset_y;
        part.offset_z = spec.offset_z;
        return true;
    }
    // Generic small cube for unknown axis names. Keeps the chain renderable
    // while making it obvious that the spec table needs an entry.
    create_cube(part, 30.0f, 30.0f, 15.0f, kGenericFallbackColor);
    return true;
}

} // namespace

namespace {

// Default colour for imported meshes; populate_axis_meshes paints every
// vertex with this so the flat-colour render path has a sensible tint
// regardless of what the OBJ/STL file declares.
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

// Import limits for machine meshes. Much larger than the OBJ importer's
// defaults: the MX-850 CAD parts are up to ~6,800 STL facets, i.e. ~12,800
// vertices after welding identical (position, normal) corners and ~20,400
// indices. With the old 4,096 / 8,192 limits four of the seven parts (X, Z,
// and both rotary axes) failed with CapacityExceeded and silently fell back
// to placeholder boxes. Vertex indices are uint16, so vertices stay < 65,535.
constexpr size_t kMeshMaxVertices = 16384;
constexpr size_t kMeshMaxIndices  = 32768;
constexpr size_t kWeldTableSize   = 32768;  // power of two > kMeshMaxVertices

// Import scratch: the importer writes into these kernel-lifetime buffers,
// then the result is copied into an exactly-sized block. Previously every
// mesh permanently took a max-size block (~164 KiB) from the bump heap,
// whatever its real size. Lazy-initialised once; never freed (the kernel
// heap is a bump allocator).
struct ScratchArena {
    gles1::Vec3f* positions = nullptr;
    gles1::Vec3f* normals = nullptr;
    gles1::Vec2f* uvs = nullptr;
    gles1::Vertex* vertices = nullptr;
    uint16_t* indices = nullptr;
    uint16_t* weld = nullptr;
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
        s.vertices = static_cast<gles1::Vertex*>(
            ::operator new(kMeshMaxVertices * sizeof(gles1::Vertex)));
        s.indices = static_cast<uint16_t*>(::operator new(kMeshMaxIndices * sizeof(uint16_t)));
        s.weld = static_cast<uint16_t*>(::operator new(kWeldTableSize * sizeof(uint16_t)));
    }
    return s;
}

// Persistent block: vertices and indices share one exactly-sized allocation
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

// Imported meshes are immutable, so cache them by (name, source buffer):
// switching machine templates re-runs populate_axis_meshes, and with a
// never-freeing bump heap every re-import used to leak the whole mesh set
// (~1.3 MB for the MX-850). A changed source buffer (new VFS contents) is a
// cache miss.
struct CachedMesh {
    char name[32];
    const char* src;
    size_t src_len;
    MeshPart part;
};
constexpr size_t kMeshCacheSize = 16;
CachedMesh g_mesh_cache[kMeshCacheSize];
size_t g_mesh_cache_count = 0;

const CachedMesh* find_cached(const char* name, const char* src, size_t len) {
    for (size_t i = 0; i < g_mesh_cache_count; ++i) {
        const CachedMesh& c = g_mesh_cache[i];
        if (c.src == src && c.src_len == len && std::strcmp(c.name, name) == 0) return &c;
    }
    return nullptr;
}

bool is_cached_part(const MeshPart& p) {
    for (size_t i = 0; i < g_mesh_cache_count; ++i) {
        if (g_mesh_cache[i].part.vertices == p.vertices) return true;
    }
    return false;
}

// Populate a MeshPart from a parsed OBJ or STL. Returns false on import
// failure (caller leaves the MeshPart zeroed so the render path falls back
// to the programmatic slot).
bool import_mesh_into_meshpart(MeshPart& part, const char* name, const char* text,
                               size_t text_len, MeshFormat format, gles1::Color4u8 color) {
    if (const CachedMesh* hit = find_cached(name, text, text_len)) {
        part = hit->part;
        return true;
    }

    obj::ImportLimits limits{};
    limits.max_vertices = kMeshMaxVertices;
    limits.max_indices = kMeshMaxIndices;
    ScratchArena& scratch = global_scratch();

    obj::ImportedMesh mesh{};
    mesh.positions = scratch.positions;
    mesh.normals = scratch.normals;
    mesh.uvs = scratch.uvs;
    mesh.vertices = scratch.vertices;
    mesh.indices = scratch.indices;
    mesh.weld_table = scratch.weld;
    mesh.weld_table_size = kWeldTableSize;

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
        return false;
    }

    gles1::Vertex* vertices = nullptr;
    uint16_t* indices = nullptr;
    (void)allocate_mesh_block(mesh.vertex_count, mesh.index_count, vertices, indices);
    std::memcpy(vertices, mesh.vertices, mesh.vertex_count * sizeof(gles1::Vertex));
    std::memcpy(indices, mesh.indices, mesh.index_count * sizeof(uint16_t));
    // Back-paint vertex colours so the flat-colour render path uses the
    // axis's chosen tint. ObjImporter leaves vertex.color untouched; we own
    // it here.
    for (size_t i = 0; i < mesh.vertex_count; ++i) vertices[i].color = color;

    part = MeshPart{};
    part.vertices = vertices;
    part.vertex_count = mesh.vertex_count;
    part.indices = indices;
    part.index_count = mesh.index_count;
    part.color = color;

    if (g_mesh_cache_count < kMeshCacheSize) {
        CachedMesh& c = g_mesh_cache[g_mesh_cache_count++];
        size_t i = 0;
        for (; name[i] && i + 1 < sizeof(c.name); ++i) c.name[i] = name[i];
        c.name[i] = '\0';
        c.src = text;
        c.src_len = text_len;
        c.part = part;
    }
    return true;
}

} // namespace

size_t populate_axis_meshes(MachineModel& model, const kinematic::KinematicChain& chain) {
    size_t populated = 0;
    for (size_t i = 0; i < chain.axis_count && i < kinematic::MAX_AXES; ++i) {
        const auto& axis = chain.axes[i];
        // OBJ / STL takes precedence — the machine editor or VFS authored
        // mesh wins over the built-in primitive. apply once per chain
        // reload; the destroy step happens in destroy_machine_model.
        bool ok = false;
        if (axis.obj_file[0] != '\0') {
            const char* data = nullptr;
            size_t size = 0;
            if (obj::lookup(axis.obj_file, data, size)) {
                const MeshFormat fmt = format_for_name(axis.obj_file);
                ok = import_mesh_into_meshpart(model.per_axis[i], axis.obj_file, data, size,
                                               fmt, kImportedMeshColor);
            }
        }
        if (axis.obj_file[0] != '\0') {
            // One line per authored mesh so a missing/oversized file is
            // visible instead of silently rendering a placeholder box.
            char msg[128];
            if (ok) {
                kernel::util::k_snprintf(msg, sizeof(msg), "[machine] %s: %s (%zu verts, %zu tris)\n",
                                         axis.name, axis.obj_file, model.per_axis[i].vertex_count,
                                         model.per_axis[i].index_count / 3);
            } else {
                kernel::util::k_snprintf(msg, sizeof(msg),
                                         "[machine] %s: %s not loaded -> placeholder primitive\n",
                                         axis.name, axis.obj_file);
            }
            if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
                kernel::g_platform->get_uart_ops()->puts(msg);
            }
        }
        if (!ok) {
            ok = generate_primitive_for_axis(model.per_axis[i], axis);
        }
        if (ok) ++populated;
    }
    return populated;
}

const MeshPart& marker_mesh() {
    alignas(MeshPart) static unsigned char storage[sizeof(MeshPart)];
    static bool constructed = false;
    if (!constructed) {
        auto* m = new (storage) MeshPart{};
        // 0.28-cube — same dimensions as the historical pivot slot, so
        // overlay code (toolpod / probe markers) reads the same on screen.
        create_cube(*m, 28.0f, 28.0f, 28.0f, {196, 148, 64, 255});   // mm
        constructed = true;
    }
    return *reinterpret_cast<const MeshPart*>(storage);
}

void destroy_machine_model(MachineModel& model) {
    // Two MeshPart memory layouts coexist:
    //  - create_cube / create_cylinder allocate vertices and indices as two
    //    separate ::operator new blocks (the primitive path).
    //  - import_mesh_into_meshpart bundles them into one block, vertices
    //    first followed by indices contiguously (the OBJ/STL arena path).
    // Detect by checking whether `indices` lies inside the vertices block;
    // if so, freeing vertices reclaims both. If not, free each separately.
    auto free_part = [](MeshPart& p) {
        if (!p.vertices) return;
        // Imported meshes are owned by the cache and shared across reloads.
        if (is_cached_part(p)) { p = MeshPart{}; return; }
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

    for (size_t i = 0; i < kinematic::MAX_AXES; ++i) free_part(model.per_axis[i]);

    model = {};
}

} // namespace render::machine
