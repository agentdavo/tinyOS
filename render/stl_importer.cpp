// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/stl_importer.hpp"

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
        int exp = 0;
        while (p < end && *p >= '0' && *p <= '9') {
            exp = exp * 10 + (*p - '0');
            ++p;
        }
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

        if (mesh.vertex_count + 3 > limits.max_vertices ||
            mesh.index_count + 3 > limits.max_indices) {
            return make_report(ImportStatus::CapacityExceeded, "mesh too large");
        }

        // Emit 3 fresh vertices + 3 indices. STL has no vertex sharing,
        // and the importer matches the OBJ flavour — one vertex per face
        // corner, per-face normal applied uniformly.
        for (int i = 0; i < 3; ++i) {
            render::gles1::Vertex& v = mesh.vertices[mesh.vertex_count];
            v.position = verts[i];
            v.normal = normal;
            v.uv = {0.0f, 0.0f};
            v.color = {0xff, 0xff, 0xff, 0xff};
            mesh.indices[mesh.index_count++] = static_cast<uint16_t>(mesh.vertex_count);
            ++mesh.vertex_count;
        }
    }

    if (mesh.vertex_count == 0 || mesh.index_count == 0) {
        return make_report(ImportStatus::ParseError, "no triangles in solid");
    }
    return make_report(ImportStatus::Ok, "ok");
}

}  // namespace render::stl
