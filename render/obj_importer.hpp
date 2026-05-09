// SPDX-License-Identifier: MIT OR Apache-2.0
// Tiny Wavefront OBJ importer for kernel-side assets.

#ifndef RENDER_OBJ_IMPORTER_HPP
#define RENDER_OBJ_IMPORTER_HPP

#include <cstddef>
#include <cstdint>

#include "render/gles1.hpp"

namespace render::obj {

struct ImportLimits {
    size_t max_positions = 1024;
    size_t max_normals = 1024;
    size_t max_uvs = 1024;
    size_t max_vertices = 4096;
    size_t max_indices = 8192;
};

// Optional per-group metadata so a caller can split a multi-`g` / multi-`o`
// OBJ into separate meshes after the fact. The importer always produces one
// flat mesh (positions/vertices/indices contiguous), but tags the index
// ranges that belong to each `g <name>` / `o <name>` block. Callers that
// don't want to split (the common case, including every kernel call site
// today) can leave `groups` null and get the historical behaviour.
struct GroupRange {
    char name[32];          // Truncated to 31 chars + NUL.
    size_t first_index;     // Index into ImportedMesh::indices.
    size_t index_count;     // Multiple of 3.
};

struct ImportedMesh {
    gles1::Vec3f* positions = nullptr;
    size_t position_count = 0;

    gles1::Vec3f* normals = nullptr;
    size_t normal_count = 0;

    gles1::Vec2f* uvs = nullptr;
    size_t uv_count = 0;

    gles1::Vertex* vertices = nullptr;
    size_t vertex_count = 0;

    uint16_t* indices = nullptr;
    size_t index_count = 0;

    GroupRange* groups = nullptr;
    size_t group_count = 0;
    size_t group_capacity = 0;
};

enum class ImportStatus : uint8_t {
    Ok = 0,
    InvalidArgument,
    CapacityExceeded,
    ParseError,
    UnsupportedFeature,
};

struct ImportReport {
    ImportStatus status = ImportStatus::Ok;
    size_t line = 0;
    const char* message = "ok";
};

class ObjImporter {
public:
    static ImportReport parse(const char* text, size_t len, const ImportLimits& limits,
                              ImportedMesh& mesh);
};

} // namespace render::obj

#endif
