// SPDX-License-Identifier: MIT OR Apache-2.0
// Tiny STL importer (ASCII + binary). Produces the same ImportedMesh shape
// as the OBJ importer so downstream (MachineModel::per_axis, GLES1 render
// path) sees geometry identically regardless of source format.

#ifndef RENDER_STL_IMPORTER_HPP
#define RENDER_STL_IMPORTER_HPP

#include "render/obj_importer.hpp"

namespace render::stl {

// Reuses obj::ImportLimits / ImportedMesh / ImportStatus / ImportReport so
// the caller holds one parser-agnostic buffer pool and can retry across
// formats without reshuffling types. Parsed triangles are welded to one
// vertex per corner (no dedup) and per-face normals are honoured; UVs are
// left zero.
render::obj::ImportReport parse_ascii(const char* text, size_t len,
                                      const render::obj::ImportLimits& limits,
                                      render::obj::ImportedMesh& mesh);

// Binary STL: 80-byte header + uint32 triangle count + N × 50-byte records
// (12 normal + 36 vertices + 2 attribute). Strict size check (header + 4
// + N*50 == len) since binary STLs occasionally start with "solid " and
// would otherwise be misclassified.
render::obj::ImportReport parse_binary(const void* data, size_t len,
                                       const render::obj::ImportLimits& limits,
                                       render::obj::ImportedMesh& mesh);

// Auto-dispatch: if the buffer's size matches the binary STL formula,
// route to parse_binary; otherwise fall back to parse_ascii. Use this
// when callers don't know upfront which flavour they hold.
render::obj::ImportReport parse(const void* data, size_t len,
                                const render::obj::ImportLimits& limits,
                                render::obj::ImportedMesh& mesh);

}  // namespace render::stl

#endif
