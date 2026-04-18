// SPDX-License-Identifier: MIT OR Apache-2.0
// Tiny ASCII-STL importer. Produces the same ImportedMesh shape as the OBJ
// importer so downstream (MachineModel::per_axis, GLES1 render path) sees
// geometry identically regardless of source format. Binary STL is not yet
// supported — ASCII is what SolidWorks / Fusion / FreeCAD / slicers emit
// by default, which matches the MX850 parts in machines/mx850/.

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

}  // namespace render::stl

#endif
