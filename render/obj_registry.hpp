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

struct RegistryEntry {
    const char* name;
    const char* data;   // start of OBJ text
    size_t size;        // bytes
};

// Returns a pointer+size pair for the given `name`, or {nullptr,0} if not
// registered. Lookup is linear over the static table; it runs once at chain
// bind time, so O(n) is fine for n<=MAX_AXES.
bool lookup(const char* name, const char*& out_data, size_t& out_size);

// Registry exposed for introspection / CLI listing.
const RegistryEntry* entries();
size_t entry_count();

} // namespace render::obj

#endif
