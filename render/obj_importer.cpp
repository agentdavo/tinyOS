// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/obj_importer.hpp"

#include <cstring>

namespace render::obj {

const MaterialEntry* MaterialMap::lookup(const char* name) const noexcept {
    if (!name) return nullptr;
    for (size_t i = 0; i < count && i < MAX_MATERIALS; ++i) {
        if (!entries[i].used) continue;
        if (std::strcmp(entries[i].name, name) == 0) return &entries[i];
    }
    return nullptr;
}

namespace {

bool mtl_is_space(char c) { return c == ' ' || c == '\t' || c == '\r'; }

void mtl_skip_spaces(const char*& p, const char* end) {
    while (p < end && mtl_is_space(*p)) ++p;
}
void mtl_skip_line(const char*& p, const char* end) {
    while (p < end && *p != '\n') ++p;
    if (p < end && *p == '\n') ++p;
}

// Hand-rolled float parser; matches the rest of the importer's style.
bool mtl_parse_float(const char*& p, const char* end, float& out) {
    mtl_skip_spaces(p, end);
    if (p >= end) return false;
    int sign = 1;
    if (*p == '-') { sign = -1; ++p; }
    else if (*p == '+') { ++p; }
    if (p >= end) return false;
    float v = 0.0f;
    bool digit = false;
    while (p < end && *p >= '0' && *p <= '9') { v = v * 10.0f + (*p - '0'); ++p; digit = true; }
    if (p < end && *p == '.') {
        ++p;
        float place = 0.1f;
        while (p < end && *p >= '0' && *p <= '9') {
            v += (*p - '0') * place;
            place *= 0.1f;
            ++p;
            digit = true;
        }
    }
    if (!digit) return false;
    out = v * sign;
    return true;
}

bool mtl_keyword(const char*& p, const char* end, const char* word) {
    const size_t n = std::strlen(word);
    if (static_cast<size_t>(end - p) < n) return false;
    for (size_t i = 0; i < n; ++i) if (p[i] != word[i]) return false;
    if (static_cast<size_t>(end - p) > n) {
        const char c = p[n];
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r') return false;
    }
    p += n;
    return true;
}

void mtl_copy_name(char* dst, size_t dst_size, const char* src, size_t src_len) {
    if (!dst || dst_size == 0) return;
    const size_t n = src_len + 1 < dst_size ? src_len : dst_size - 1;
    for (size_t i = 0; i < n; ++i) dst[i] = src[i];
    dst[n] = '\0';
}

uint8_t mtl_clamp_to_u8(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return 0xFF;
    return static_cast<uint8_t>(v * 255.0f);
}

} // namespace

size_t parse_mtl(const char* text, size_t len, MaterialMap& out) noexcept {
    out.count = 0;
    for (auto& e : out.entries) { e = MaterialEntry{}; }
    if (!text || len == 0) return 0;
    const char* p = text;
    const char* end = text + len;
    MaterialEntry* current = nullptr;
    while (p < end) {
        mtl_skip_spaces(p, end);
        if (p >= end) break;
        if (*p == '\n') { ++p; continue; }
        if (*p == '#') { mtl_skip_line(p, end); continue; }
        if (mtl_keyword(p, end, "newmtl")) {
            mtl_skip_spaces(p, end);
            const char* name_start = p;
            while (p < end && *p != '\n' && *p != '\r' && !mtl_is_space(*p)) ++p;
            const size_t nlen = static_cast<size_t>(p - name_start);
            if (out.count < MaterialMap::MAX_MATERIALS) {
                current = &out.entries[out.count++];
                mtl_copy_name(current->name, sizeof(current->name), name_start, nlen);
                current->diffuse = {0xff, 0xff, 0xff, 0xff};
                current->used = true;
            } else {
                current = nullptr;
            }
            mtl_skip_line(p, end);
            continue;
        }
        if (mtl_keyword(p, end, "Kd")) {
            float r = 1.0f, g = 1.0f, b = 1.0f;
            mtl_parse_float(p, end, r);
            mtl_parse_float(p, end, g);
            mtl_parse_float(p, end, b);
            if (current) {
                current->diffuse = {mtl_clamp_to_u8(r), mtl_clamp_to_u8(g),
                                    mtl_clamp_to_u8(b), 0xFF};
            }
            mtl_skip_line(p, end);
            continue;
        }
        // Anything else (Ka, Ks, Ns, illum, map_*, ...) — skip the line.
        mtl_skip_line(p, end);
    }
    return out.count;
}

namespace {

struct Cursor {
    const char* ptr;
    const char* end;
    size_t line;
};

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r';
}

void skip_spaces(Cursor& cur) {
    while (cur.ptr < cur.end && is_space(*cur.ptr)) ++cur.ptr;
}

void skip_line(Cursor& cur) {
    while (cur.ptr < cur.end && *cur.ptr != '\n') ++cur.ptr;
    if (cur.ptr < cur.end && *cur.ptr == '\n') {
        ++cur.ptr;
        ++cur.line;
    }
}

bool consume_char(Cursor& cur, char expected) {
    if (cur.ptr >= cur.end || *cur.ptr != expected) return false;
    ++cur.ptr;
    return true;
}

bool parse_int(Cursor& cur, int32_t& out) {
    skip_spaces(cur);
    if (cur.ptr >= cur.end) return false;

    int sign = 1;
    if (*cur.ptr == '-') {
        sign = -1;
        ++cur.ptr;
    } else if (*cur.ptr == '+') {
        ++cur.ptr;
    }

    if (cur.ptr >= cur.end || *cur.ptr < '0' || *cur.ptr > '9') return false;

    int32_t value = 0;
    while (cur.ptr < cur.end && *cur.ptr >= '0' && *cur.ptr <= '9') {
        value = value * 10 + static_cast<int32_t>(*cur.ptr - '0');
        ++cur.ptr;
    }
    out = value * sign;
    return true;
}

bool parse_float(Cursor& cur, float& out) {
    skip_spaces(cur);
    if (cur.ptr >= cur.end) return false;

    int sign = 1;
    if (*cur.ptr == '-') {
        sign = -1;
        ++cur.ptr;
    } else if (*cur.ptr == '+') {
        ++cur.ptr;
    }

    float value = 0.0f;
    bool saw_digit = false;
    while (cur.ptr < cur.end && *cur.ptr >= '0' && *cur.ptr <= '9') {
        saw_digit = true;
        value = value * 10.0f + static_cast<float>(*cur.ptr - '0');
        ++cur.ptr;
    }

    if (cur.ptr < cur.end && *cur.ptr == '.') {
        ++cur.ptr;
        float place = 0.1f;
        while (cur.ptr < cur.end && *cur.ptr >= '0' && *cur.ptr <= '9') {
            saw_digit = true;
            value += static_cast<float>(*cur.ptr - '0') * place;
            place *= 0.1f;
            ++cur.ptr;
        }
    }

    if (!saw_digit) return false;

    if (cur.ptr < cur.end && (*cur.ptr == 'e' || *cur.ptr == 'E')) {
        ++cur.ptr;
        int32_t exponent = 0;
        if (!parse_int(cur, exponent)) return false;
        float scale = 1.0f;
        if (exponent > 0) {
            for (int32_t i = 0; i < exponent; ++i) scale *= 10.0f;
        } else {
            for (int32_t i = 0; i < -exponent; ++i) scale *= 0.1f;
        }
        value *= scale;
    }

    out = value * static_cast<float>(sign);
    return true;
}

ImportReport make_error(Cursor cur, ImportStatus status, const char* message) {
    ImportReport out{};
    out.status = status;
    out.line = cur.line;
    out.message = message;
    return out;
}

bool within_limits(size_t count, size_t limit) {
    return count < limit;
}

int32_t obj_index_to_zero_based(int32_t obj_index, size_t count) {
    if (obj_index > 0) return obj_index - 1;
    if (obj_index < 0) return static_cast<int32_t>(count) + obj_index;
    return -1;
}

void copy_token_buf(char* dst, size_t dst_size, const char* src, size_t src_len) {
    if (!dst || dst_size == 0) return;
    const size_t n = src_len + 1 < dst_size ? src_len : dst_size - 1;
    for (size_t i = 0; i < n; ++i) dst[i] = src[i];
    dst[n] = '\0';
}

// Read the rest of the line, trimmed, into a stack buffer. Used by `g` /
// `o` / `usemtl` to capture the name token. Stops at newline or EOF.
size_t read_rest_of_line(Cursor& cur, char* dst, size_t dst_size) {
    skip_spaces(cur);
    const char* start = cur.ptr;
    while (cur.ptr < cur.end && *cur.ptr != '\n' && *cur.ptr != '\r') ++cur.ptr;
    size_t n = static_cast<size_t>(cur.ptr - start);
    while (n > 0 && (start[n - 1] == ' ' || start[n - 1] == '\t')) --n;
    copy_token_buf(dst, dst_size, start, n);
    return n;
}

// Close out the current group (if any) by computing its index_count from
// the imported mesh's running index_count, then push the new group.
void close_and_open_group(ImportedMesh& mesh, const char* name, size_t name_len) {
    if (!mesh.groups || mesh.group_capacity == 0) return;
    if (mesh.group_count > 0) {
        GroupRange& cur_grp = mesh.groups[mesh.group_count - 1];
        cur_grp.index_count = mesh.index_count - cur_grp.first_index;
    }
    if (mesh.group_count >= mesh.group_capacity) return;
    GroupRange& g = mesh.groups[mesh.group_count++];
    copy_token_buf(g.name, sizeof(g.name), name, name_len);
    g.first_index = mesh.index_count;
    g.index_count = 0;
}

bool starts_with_word(const Cursor& cur, const char* word) {
    const size_t n = std::strlen(word);
    if (static_cast<size_t>(cur.end - cur.ptr) < n) return false;
    for (size_t i = 0; i < n; ++i) {
        if (cur.ptr[i] != word[i]) return false;
    }
    if (static_cast<size_t>(cur.end - cur.ptr) == n) return true;
    const char c = cur.ptr[n];
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

ImportReport parse_face(Cursor& cur, const ImportLimits& limits, ImportedMesh& mesh,
                        const gles1::Color4u8& active_material_color) {
    uint16_t face_indices[4]{};
    size_t face_count = 0;

    while (cur.ptr < cur.end && *cur.ptr != '\n') {
        skip_spaces(cur);
        if (cur.ptr >= cur.end || *cur.ptr == '\n') break;
        if (face_count >= 4) {
            return make_error(cur, ImportStatus::UnsupportedFeature,
                              "only triangles and quads are supported");
        }

        int32_t v_idx_obj = 0;
        if (!parse_int(cur, v_idx_obj)) {
            return make_error(cur, ImportStatus::ParseError, "invalid face vertex index");
        }

        int32_t vt_idx_obj = 0;
        bool has_vt = false;
        int32_t vn_idx_obj = 0;
        bool has_vn = false;

        if (consume_char(cur, '/')) {
            if (cur.ptr < cur.end && *cur.ptr != '/' && *cur.ptr != ' ' && *cur.ptr != '\n') {
                has_vt = parse_int(cur, vt_idx_obj);
                if (!has_vt) {
                    return make_error(cur, ImportStatus::ParseError, "invalid face uv index");
                }
            }
            if (consume_char(cur, '/')) {
                has_vn = parse_int(cur, vn_idx_obj);
                if (!has_vn) {
                    return make_error(cur, ImportStatus::ParseError, "invalid face normal index");
                }
            }
        }

        if (!within_limits(mesh.vertex_count, limits.max_vertices)) {
            return make_error(cur, ImportStatus::CapacityExceeded, "vertex capacity exceeded");
        }

        const int32_t pos_idx = obj_index_to_zero_based(v_idx_obj, mesh.position_count);
        if (pos_idx < 0 || static_cast<size_t>(pos_idx) >= mesh.position_count) {
            return make_error(cur, ImportStatus::ParseError, "face references missing position");
        }

        gles1::Vertex vertex{};
        vertex.position = mesh.positions[pos_idx];
        vertex.color = active_material_color;

        if (has_vt) {
            const int32_t uv_idx = obj_index_to_zero_based(vt_idx_obj, mesh.uv_count);
            if (uv_idx < 0 || static_cast<size_t>(uv_idx) >= mesh.uv_count) {
                return make_error(cur, ImportStatus::ParseError, "face references missing uv");
            }
            vertex.uv = mesh.uvs[uv_idx];
        }

        if (has_vn) {
            const int32_t n_idx = obj_index_to_zero_based(vn_idx_obj, mesh.normal_count);
            if (n_idx < 0 || static_cast<size_t>(n_idx) >= mesh.normal_count) {
                return make_error(cur, ImportStatus::ParseError, "face references missing normal");
            }
            vertex.normal = mesh.normals[n_idx];
        }

        mesh.vertices[mesh.vertex_count] = vertex;
        face_indices[face_count++] = static_cast<uint16_t>(mesh.vertex_count++);
    }

    if (face_count < 3) {
        return make_error(cur, ImportStatus::ParseError, "face has fewer than 3 vertices");
    }

    const size_t triangles = face_count == 4 ? 2 : 1;
    if (mesh.index_count + triangles * 3 > limits.max_indices) {
        return make_error(cur, ImportStatus::CapacityExceeded, "index capacity exceeded");
    }

    mesh.indices[mesh.index_count++] = face_indices[0];
    mesh.indices[mesh.index_count++] = face_indices[1];
    mesh.indices[mesh.index_count++] = face_indices[2];
    if (face_count == 4) {
        mesh.indices[mesh.index_count++] = face_indices[0];
        mesh.indices[mesh.index_count++] = face_indices[2];
        mesh.indices[mesh.index_count++] = face_indices[3];
    }

    return {};
}

} // namespace

ImportReport ObjImporter::parse(const char* text, size_t len, const ImportLimits& limits,
                                ImportedMesh& mesh) {
    if (!text || !mesh.positions || !mesh.normals || !mesh.uvs ||
        !mesh.vertices || !mesh.indices) {
        return {ImportStatus::InvalidArgument, 0, "null argument"};
    }

    mesh.position_count = 0;
    mesh.normal_count = 0;
    mesh.uv_count = 0;
    mesh.vertex_count = 0;
    mesh.index_count = 0;
    if (mesh.groups && mesh.group_capacity > 0) mesh.group_count = 0;

    Cursor cur{text, text + len, 1};
    // Active material colour for vertex emit. Tracks the most recent
    // `usemtl` name that resolves in mesh.materials. Default white so
    // the caller's back-paint pass (machine_model::import_mesh_into_meshpart)
    // can override per-mesh when no MaterialMap is supplied.
    gles1::Color4u8 active_material_color{0xff, 0xff, 0xff, 0xff};
    while (cur.ptr < cur.end) {
        skip_spaces(cur);
        if (cur.ptr >= cur.end) break;
        if (*cur.ptr == '\n') {
            ++cur.ptr;
            ++cur.line;
            continue;
        }
        if (*cur.ptr == '#') {
            skip_line(cur);
            continue;
        }

        if (*cur.ptr == 'v') {
            ++cur.ptr;
            if (cur.ptr < cur.end && *cur.ptr == ' ') {
                if (!within_limits(mesh.position_count, limits.max_positions)) {
                    return make_error(cur, ImportStatus::CapacityExceeded,
                                      "position capacity exceeded");
                }
                gles1::Vec3f value{};
                if (!parse_float(cur, value.x) || !parse_float(cur, value.y) ||
                    !parse_float(cur, value.z)) {
                    return make_error(cur, ImportStatus::ParseError, "invalid position");
                }
                mesh.positions[mesh.position_count++] = value;
                skip_line(cur);
                continue;
            }
            if (cur.ptr < cur.end && *cur.ptr == 'n') {
                ++cur.ptr;
                if (!within_limits(mesh.normal_count, limits.max_normals)) {
                    return make_error(cur, ImportStatus::CapacityExceeded,
                                      "normal capacity exceeded");
                }
                gles1::Vec3f value{};
                if (!parse_float(cur, value.x) || !parse_float(cur, value.y) ||
                    !parse_float(cur, value.z)) {
                    return make_error(cur, ImportStatus::ParseError, "invalid normal");
                }
                mesh.normals[mesh.normal_count++] = value;
                skip_line(cur);
                continue;
            }
            if (cur.ptr < cur.end && *cur.ptr == 't') {
                ++cur.ptr;
                if (!within_limits(mesh.uv_count, limits.max_uvs)) {
                    return make_error(cur, ImportStatus::CapacityExceeded, "uv capacity exceeded");
                }
                gles1::Vec2f value{};
                if (!parse_float(cur, value.x) || !parse_float(cur, value.y)) {
                    return make_error(cur, ImportStatus::ParseError, "invalid uv");
                }
                mesh.uvs[mesh.uv_count++] = value;
                skip_line(cur);
                continue;
            }
            return make_error(cur, ImportStatus::UnsupportedFeature, "unsupported vertex record");
        }

        if (*cur.ptr == 'f') {
            ++cur.ptr;
            ImportReport report = parse_face(cur, limits, mesh, active_material_color);
            if (report.status != ImportStatus::Ok) return report;
            skip_line(cur);
            continue;
        }

        // Group / object / material directives. We collapse all of them into
        // one mesh (the kernel's MeshPart can hold one), but if the caller
        // supplied a `groups` table we record the index ranges so future
        // multi-part splitting is possible without re-parsing.
        if (starts_with_word(cur, "g") || starts_with_word(cur, "o") ||
            starts_with_word(cur, "usemtl")) {
            const bool was_usemtl = starts_with_word(cur, "usemtl");
            // Skip the directive token itself.
            while (cur.ptr < cur.end && !is_space(*cur.ptr) && *cur.ptr != '\n') ++cur.ptr;
            char name[32];
            const size_t n = read_rest_of_line(cur, name, sizeof(name));
            close_and_open_group(mesh, name, n);
            // Tier 4d: if this was a `usemtl` and the caller supplied a
            // material map, look up the diffuse colour and apply it to
            // subsequent vertex emits. Names that don't resolve fall back
            // to the prior active colour, preserving any prior usemtl.
            if (was_usemtl && mesh.materials) {
                const auto* m = mesh.materials->lookup(name);
                if (m) active_material_color = m->diffuse;
            }
            skip_line(cur);
            continue;
        }
        // Smoothing groups (`s 1`/`s off`) and material libraries (`mtllib`)
        // are silently ignored — the renderer doesn't honour either.
        skip_line(cur);
    }

    // Close out the trailing group's index_count, if any.
    if (mesh.groups && mesh.group_count > 0) {
        GroupRange& last = mesh.groups[mesh.group_count - 1];
        last.index_count = mesh.index_count - last.first_index;
    }

    return {};
}

} // namespace render::obj
