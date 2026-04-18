// SPDX-License-Identifier: MIT OR Apache-2.0

#include "render/obj_registry.hpp"

#include "fs/vfs.hpp"

#include <cstring>

namespace render::obj {

// The OBJ registry is now a thin wrapper over kernel::vfs. Any file the
// kinematic TSV references via `obj_file=<name>` is resolved to
// `system/machine/<name>` in the VFS. That name namespace matches the
// machine-editor bundle layout, so an editor-authored `demo_box.obj`
// drops straight into the lookup table without touching kernel code.
//
// The SD backend is where new OBJs will come from at runtime; today the
// embedded defaults path in vfs::register_embedded_defaults() pre-loads
// `system/machine/demo_box.obj` so the mill3 TSV's `obj_file=demo_box.obj`
// row resolves without any external action.

namespace {

// Tiny scratch for rewriting "<name>" → "system/machine/<name>". 128 bytes
// covers any reasonable OBJ filename with headroom.
constexpr size_t kPathBuf = 128;
constexpr const char* kMachinePrefix = "system/machine/";

bool build_path(const char* name, char (&out)[kPathBuf]) {
    if (!name || !*name) return false;
    const size_t prefix_len = std::strlen(kMachinePrefix);
    const size_t name_len = std::strlen(name);
    if (prefix_len + name_len + 1 > kPathBuf) return false;
    std::memcpy(out, kMachinePrefix, prefix_len);
    std::memcpy(out + prefix_len, name, name_len + 1);
    return true;
}

struct CountCtx { size_t count; };
bool count_cb(const char*, const char*, size_t, void* user) {
    static_cast<CountCtx*>(user)->count++;
    return true;
}

}  // namespace

bool lookup(const char* name, const char*& out_data, size_t& out_size) {
    out_data = nullptr;
    out_size = 0;
    char path[kPathBuf] = {};
    if (!build_path(name, path)) return false;
    return kernel::vfs::lookup(path, out_data, out_size);
}

const RegistryEntry* entries() { return nullptr; }

size_t entry_count() {
    CountCtx ctx{0};
    kernel::vfs::walk(kMachinePrefix, &count_cb, &ctx);
    return ctx.count;
}

}  // namespace render::obj
