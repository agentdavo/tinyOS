// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/stl_importer.hpp"

#include "util_math.hpp"

#include <cstring>

namespace render::stl {

namespace {

using render::obj::ImportedMesh;
using render::obj::ImportLimits;
using render::obj::ImportReport;
using render::obj::ImportStatus;

inline bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// Skip whitespace + optional line-comments (STL doesn't define comments,
// but tools occasionally emit `# ...` — we tolerate them).
const char* skip_ws(const char* p, const char* end) {
    while (p < end) {
        if (is_ws(*p)) { ++p; continue; }
        if (*p == '#' || *p == ';') {
            while (p < end && *p != '\n') ++p;
            continue;
        }
        break;
    }
    return p;
}

bool match_token(const char*& p, const char* end, const char* tok) {
    const char* q = p;
    size_t n = 0;
    while (tok[n] != '\0') {
        if (q >= end || q[n] != tok[n]) return false;
        ++n;
    }
    // Token must be followed by whitespace or EOF.
    if (q + n < end && !is_ws(q[n])) return false;
    p = q + n;
    return true;
}

// Parse a float in the relaxed style STL uses (scientific notation is
// common: `1.234560e+02`). Hand-rolled since the kernel is freestanding.
bool parse_float(const char*& p, const char* end, float& out) {
    p = skip_ws(p, end);
    if (p >= end) return false;
    float sign = 1.0f;
    if (*p == '+') ++p;
    else if (*p == '-') { sign = -1.0f; ++p; }

    float whole = 0.0f;
    bool had_digit = false;
    while (p < end && *p >= '0' && *p <= '9') {
        whole = whole * 10.0f + static_cast<float>(*p - '0');
        ++p;
        had_digit = true;
    }
    float frac = 0.0f;
    if (p < end && *p == '.') {
        ++p;
        float place = 0.1f;
        while (p < end && *p >= '0' && *p <= '9') {
            frac += static_cast<float>(*p - '0') * place;
            place *= 0.1f;
            ++p;
            had_digit = true;
        }
    }
    if (!had_digit) return false;
    float value = whole + frac;
    if (p < end && (*p == 'e' || *p == 'E')) {
        ++p;
        int esign = 1;
        if (p < end && *p == '+') ++p;
        else if (p < end && *p == '-') { esign = -1; ++p; }
        // Cap exp at IEEE-754 single's range so the loop stays bounded and
        // mul never silently inf/underflows from a multi-digit exponent in
        // malformed input. Anything past 1e38 already saturates float.
        constexpr int kMaxExp = 38;
        int exp = 0;
        while (p < end && *p >= '0' && *p <= '9') {
            if (exp < kMaxExp + 1) exp = exp * 10 + (*p - '0');
            ++p;
        }
        if (exp > kMaxExp) exp = kMaxExp;
        float mul = 1.0f;
        for (int i = 0; i < exp; ++i) mul *= 10.0f;
        value = (esign > 0) ? value * mul : value / mul;
    }
    out = sign * value;
    return true;
}

bool parse_vec3(const char*& p, const char* end, render::gles1::Vec3f& out) {
    return parse_float(p, end, out.x) &&
           parse_float(p, end, out.y) &&
           parse_float(p, end, out.z);
}

ImportReport make_report(ImportStatus status, const char* msg) {
    ImportReport r{};
    r.status = status;
    r.message = msg ? msg : "";
    return r;
}

inline bool finite3(const render::gles1::Vec3f& v) {
    auto ok = [](float f) { return f == f && f < 1e30f && f > -1e30f; };
    return ok(v.x) && ok(v.y) && ok(v.z);
}

inline uint32_t float_bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}

// Many exporters write a zero (or unnormalised) facet normal. Use the
// winding-derived normal then, and always hand the renderer a unit vector.
render::gles1::Vec3f facet_normal(const render::gles1::Vec3f& n_in,
                                  const render::gles1::Vec3f v[3]) {
    render::gles1::Vec3f n = n_in;
    float len_sq = n.x * n.x + n.y * n.y + n.z * n.z;
    if (!(len_sq > 1e-12f)) {
        const render::gles1::Vec3f a{v[1].x - v[0].x, v[1].y - v[0].y, v[1].z - v[0].z};
        const render::gles1::Vec3f b{v[2].x - v[0].x, v[2].y - v[0].y, v[2].z - v[0].z};
        n = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        len_sq = n.x * n.x + n.y * n.y + n.z * n.z;
        if (!(len_sq > 1e-24f)) return {0.0f, 0.0f, 1.0f};  // degenerate facet
    }
    if (len_sq > 0.9999f && len_sq < 1.0001f) return n;
    const float inv = 1.0f / kernel::util::math::sqrt_approx(len_sq);
    return {n.x * inv, n.y * inv, n.z * inv};
}

// Append one facet corner, reusing an identical (position, normal) vertex
// when the caller supplied a weld table. Returns false on capacity.
bool emit_vertex(ImportedMesh& mesh, const ImportLimits& limits,
                 const render::gles1::Vec3f& pos, const render::gles1::Vec3f& normal) {
    const size_t max_verts = limits.max_vertices < 0xFFFFu ? limits.max_vertices : 0xFFFFu;
    uint16_t* slot = nullptr;
    if (mesh.weld_table && mesh.weld_table_size) {
        uint32_t h = 2166136261u;  // FNV-1a over the six float bit patterns
        const uint32_t k[6] = {float_bits(pos.x), float_bits(pos.y), float_bits(pos.z),
                               float_bits(normal.x), float_bits(normal.y), float_bits(normal.z)};
        for (uint32_t w : k) { h ^= w; h *= 16777619u; }
        const size_t mask = mesh.weld_table_size - 1;
        for (size_t probe = 0; probe < mesh.weld_table_size; ++probe) {
            uint16_t& e = mesh.weld_table[(h + probe) & mask];
            if (e == 0xFFFFu) { slot = &e; break; }
            const render::gles1::Vertex& v = mesh.vertices[e];
            if (float_bits(v.position.x) == k[0] && float_bits(v.position.y) == k[1] &&
                float_bits(v.position.z) == k[2] && float_bits(v.normal.x) == k[3] &&
                float_bits(v.normal.y) == k[4] && float_bits(v.normal.z) == k[5]) {
                mesh.indices[mesh.index_count++] = e;
                return true;
            }
        }
        if (!slot) return false;  // table full
    }
    if (mesh.vertex_count >= max_verts) return false;
    render::gles1::Vertex& v = mesh.vertices[mesh.vertex_count];
    v.position = pos;
    v.normal = normal;
    v.uv = {0.0f, 0.0f};
    v.color = {0xff, 0xff, 0xff, 0xff};
    if (slot) *slot = static_cast<uint16_t>(mesh.vertex_count);
    mesh.indices[mesh.index_count++] = static_cast<uint16_t>(mesh.vertex_count);
    ++mesh.vertex_count;
    return true;
}

// Emit one triangle (skipping non-finite ones). Returns false on capacity.
bool emit_facet(ImportedMesh& mesh, const ImportLimits& limits,
                const render::gles1::Vec3f& normal_in, const render::gles1::Vec3f v[3]) {
    if (!finite3(v[0]) || !finite3(v[1]) || !finite3(v[2])) return true;  // drop garbage
    if (mesh.index_count + 3 > limits.max_indices) return false;
    const render::gles1::Vec3f n = facet_normal(finite3(normal_in) ? normal_in
                                                                  : render::gles1::Vec3f{0, 0, 0}, v);
    for (int i = 0; i < 3; ++i) {
        if (!emit_vertex(mesh, limits, v[i], n)) return false;
    }
    return true;
}

void reset_weld_table(ImportedMesh& mesh) {
    if (!mesh.weld_table) return;
    for (size_t i = 0; i < mesh.weld_table_size; ++i) mesh.weld_table[i] = 0xFFFFu;
}

}  // namespace

ImportReport parse_ascii(const char* text, size_t len, const ImportLimits& limits,
                         ImportedMesh& mesh) {
    if (!text || len == 0) return make_report(ImportStatus::InvalidArgument, "empty input");
    if (!mesh.vertices || !mesh.indices) {
        return make_report(ImportStatus::InvalidArgument, "output buffers not provided");
    }
    mesh.vertex_count = 0;
    mesh.index_count = 0;
    mesh.position_count = 0;
    mesh.normal_count = 0;
    mesh.uv_count = 0;
    reset_weld_table(mesh);

    const char* p = text;
    const char* end = text + len;

    p = skip_ws(p, end);
    // `solid [name]` — name is optional and we don't care about it.
    if (!match_token(p, end, "solid")) {
        return make_report(ImportStatus::ParseError, "expected `solid` header");
    }
    // Skip to end of line — solid name may contain spaces.
    while (p < end && *p != '\n') ++p;

    render::gles1::Vec3f normal{0, 0, 0};
    render::gles1::Vec3f verts[3]{};

    while (p < end) {
        p = skip_ws(p, end);
        if (p >= end) break;
        if (match_token(p, end, "endsolid")) break;

        if (!match_token(p, end, "facet")) {
            return make_report(ImportStatus::ParseError, "expected `facet`");
        }
        p = skip_ws(p, end);
        if (!match_token(p, end, "normal")) {
            return make_report(ImportStatus::ParseError, "expected `normal`");
        }
        if (!parse_vec3(p, end, normal)) {
            return make_report(ImportStatus::ParseError, "malformed facet normal");
        }
        p = skip_ws(p, end);
        if (!match_token(p, end, "outer")) {
            return make_report(ImportStatus::ParseError, "expected `outer`");
        }
        p = skip_ws(p, end);
        if (!match_token(p, end, "loop")) {
            return make_report(ImportStatus::ParseError, "expected `loop`");
        }

        for (int i = 0; i < 3; ++i) {
            p = skip_ws(p, end);
            if (!match_token(p, end, "vertex")) {
                return make_report(ImportStatus::ParseError, "expected `vertex`");
            }
            if (!parse_vec3(p, end, verts[i])) {
                return make_report(ImportStatus::ParseError, "malformed vertex");
            }
        }

        p = skip_ws(p, end);
        if (!match_token(p, end, "endloop")) {
            return make_report(ImportStatus::ParseError, "expected `endloop`");
        }
        p = skip_ws(p, end);
        if (!match_token(p, end, "endfacet")) {
            return make_report(ImportStatus::ParseError, "expected `endfacet`");
        }

        // One vertex per facet corner with the facet's normal (flat
        // shading); corners identical in position+normal are welded when
        // the caller provides a weld table.
        if (!emit_facet(mesh, limits, normal, verts)) {
            return make_report(ImportStatus::CapacityExceeded, "mesh too large");
        }
    }

    if (mesh.vertex_count == 0 || mesh.index_count == 0) {
        return make_report(ImportStatus::ParseError, "no triangles in solid");
    }
    return make_report(ImportStatus::Ok, "ok");
}

namespace {

// Binary STL is little-endian by spec. Build floats / uint32s out of bytes
// instead of memcpy-ing into a struct so the importer doesn't depend on
// host endianness or struct packing — both arches we care about happen to
// be LE today, but this stays portable.
inline uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline float read_f32_le(const uint8_t* p) {
    union { uint32_t u; float f; } v;
    v.u = read_u32_le(p);
    return v.f;
}

constexpr size_t kBinaryHeader = 80;
constexpr size_t kBinaryTriRecord = 50;

}  // namespace

ImportReport parse_binary(const void* data, size_t len, const ImportLimits& limits,
                          ImportedMesh& mesh) {
    if (!data || len < kBinaryHeader + 4) {
        return make_report(ImportStatus::InvalidArgument, "buffer too small for binary STL");
    }
    if (!mesh.vertices || !mesh.indices) {
        return make_report(ImportStatus::InvalidArgument, "output buffers not provided");
    }
    mesh.vertex_count = 0;
    mesh.index_count = 0;
    mesh.position_count = 0;
    mesh.normal_count = 0;
    mesh.uv_count = 0;
    reset_weld_table(mesh);

    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    const uint32_t tri_count = read_u32_le(bytes + kBinaryHeader);
    const size_t expected = kBinaryHeader + 4 + static_cast<size_t>(tri_count) * kBinaryTriRecord;
    if (len != expected) {
        return make_report(ImportStatus::ParseError, "binary STL size mismatch");
    }
    if (tri_count == 0) {
        return make_report(ImportStatus::ParseError, "no triangles in binary STL");
    }

    const uint8_t* p = bytes + kBinaryHeader + 4;
    for (uint32_t t = 0; t < tri_count; ++t) {
        const render::gles1::Vec3f normal{
            read_f32_le(p + 0), read_f32_le(p + 4), read_f32_le(p + 8)};
        render::gles1::Vec3f verts[3];
        for (int i = 0; i < 3; ++i) {
            const uint8_t* vp = p + 12 + i * 12;
            verts[i] = {read_f32_le(vp + 0), read_f32_le(vp + 4), read_f32_le(vp + 8)};
        }
        if (!emit_facet(mesh, limits, normal, verts)) {
            return make_report(ImportStatus::CapacityExceeded, "mesh too large");
        }
        p += kBinaryTriRecord;
    }
    if (mesh.index_count == 0) {
        return make_report(ImportStatus::ParseError, "no finite triangles in binary STL");
    }
    return make_report(ImportStatus::Ok, "ok");
}

ImportReport parse(const void* data, size_t len, const ImportLimits& limits,
                   ImportedMesh& mesh) {
    if (!data || len == 0) return make_report(ImportStatus::InvalidArgument, "empty input");
    // Binary detection: file size must equal header + count + count*record.
    // Some tools write the ASCII "solid " prefix into the binary header,
    // so a textual sniff is unreliable; the size formula is the only thing
    // that's authoritative.
    if (len > kBinaryHeader + 4) {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        const uint32_t tri_count = read_u32_le(bytes + kBinaryHeader);
        const size_t expected = kBinaryHeader + 4 +
                                static_cast<size_t>(tri_count) * kBinaryTriRecord;
        if (len == expected && tri_count > 0) {
            return parse_binary(data, len, limits, mesh);
        }
    }
    return parse_ascii(static_cast<const char*>(data), len, limits, mesh);
}

}  // namespace render::stl
