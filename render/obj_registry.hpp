// SPDX-License-Identifier: MIT OR Apache-2.0
// OBJ blob registry. Machine-editor-authored OBJ files are embedded into the
// kernel via `.incbin` in `devices/embedded_kinematic_obj.S`; the registry
// turns a string name (as written in the kinematic TSV's `obj_file` column
// 14) into a pointer to the raw OBJ text. The registry is intentionally
// static; adding a new mesh means adding an `.incbin` line and a registry
// entry, not hot-loading at runtime.
//
// Today the registry is empty — the shipped `kinematic_*.tsv` files use the
// programmatic `MachineModel` slots (`base`/`x_axis`/…). When the machine
// editor ships a built machine, the exported OBJs drop into
// `devices/<name>.obj` and the `.S` + table below gains entries for them.

#ifndef RENDER_OBJ_REGISTRY_HPP
#define RENDER_OBJ_REGISTRY_HPP

#include <cstddef>

namespace render::obj {

// Resolves a chain obj_file basename to its bytes under system/machine/ in
// the VFS (SD card first, then the embedded defaults). Returns false and
// {nullptr,0} if absent.
bool lookup(const char* name, const char*& out_data, size_t& out_size);

} // namespace render::obj

#endif
