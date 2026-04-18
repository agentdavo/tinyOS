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
