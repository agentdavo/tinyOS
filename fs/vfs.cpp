// SPDX-License-Identifier: MIT OR Apache-2.0

#include "vfs.hpp"

#include "../miniOS.hpp"
#include "../util.hpp"

#include <cstring>

namespace kernel::vfs {

namespace {

struct Entry {
    char path[kMaxPathLen] = {};
    const char* data = nullptr;
    size_t size = 0;
};

Entry g_entries[kMaxEntries] = {};
size_t g_count = 0;

int find_index(const char* path) noexcept {
    if (!path) return -1;
    for (size_t i = 0; i < g_count; ++i) {
        if (std::strcmp(g_entries[i].path, path) == 0) return static_cast<int>(i);
    }
    return -1;
}

// Helper: register a .incbin blob given start/end pointers. Empty blobs
// (start == end, which happens when the .S file ships an empty section)
// are skipped rather than registered as zero-length — zero-length entries
// would just confuse later lookups that assume non-empty means present.
void register_embedded(const char* path, const char* start, const char* end) noexcept {
    if (!start || !end || end <= start) return;
    (void)register_blob(path, start, static_cast<size_t>(end - start));
}

// External .incbin symbols. Declared in the same shape as kernel/main.cpp.
// Keeping them inside this TU makes the VFS the single place that knows
// the `_binary_*` naming convention — no other file has to import those.
extern "C" {
extern const char _binary_embedded_ui_tsv_start[];
extern const char _binary_embedded_ui_tsv_end[];
extern const char _binary_kinematic_mill3_tsv_start[];
extern const char _binary_kinematic_mill3_tsv_end[];
extern const char _binary_kinematic_millturn_tsv_start[];
extern const char _binary_kinematic_millturn_tsv_end[];
extern const char _binary_kinematic_mx850_tsv_start[];
extern const char _binary_kinematic_mx850_tsv_end[];
extern const char _binary_embedded_toolpods_tsv_start[];
extern const char _binary_embedded_toolpods_tsv_end[];
extern const char _binary_demo_box_obj_start[];
extern const char _binary_demo_box_obj_end[];
extern const char _binary_demo_part_stl_start[];
extern const char _binary_demo_part_stl_end[];
}

}  // namespace

bool register_blob(const char* path, const char* data, size_t size) noexcept {
    if (!path || !*path || !data || size == 0) return false;
    if (std::strlen(path) >= kMaxPathLen) return false;
    int idx = find_index(path);
    if (idx < 0) {
        if (g_count >= kMaxEntries) return false;
        idx = static_cast<int>(g_count++);
    }
    Entry& e = g_entries[idx];
    size_t n = 0;
    while (path[n] && n + 1 < kMaxPathLen) { e.path[n] = path[n]; ++n; }
    e.path[n] = '\0';
    e.data = data;
    e.size = size;
    return true;
}

bool lookup(const char* path, const char*& data_out, size_t& size_out) noexcept {
    const int idx = find_index(path);
    if (idx < 0) { data_out = nullptr; size_out = 0; return false; }
    data_out = g_entries[idx].data;
    size_out = g_entries[idx].size;
    return true;
}

void walk(const char* prefix, WalkFn fn, void* user) noexcept {
    if (!fn) return;
    const size_t plen = prefix ? std::strlen(prefix) : 0;
    for (size_t i = 0; i < g_count; ++i) {
        if (plen == 0 || std::strncmp(g_entries[i].path, prefix, plen) == 0) {
            if (!fn(g_entries[i].path, g_entries[i].data, g_entries[i].size, user)) return;
        }
    }
}

size_t entry_count() noexcept { return g_count; }

void register_embedded_defaults() noexcept {
    register_embedded("system/ui/embedded_ui.tsv",
                      _binary_embedded_ui_tsv_start, _binary_embedded_ui_tsv_end);
    register_embedded("system/machine/kinematic_mill3.tsv",
                      _binary_kinematic_mill3_tsv_start, _binary_kinematic_mill3_tsv_end);
    register_embedded("system/machine/kinematic_millturn.tsv",
                      _binary_kinematic_millturn_tsv_start, _binary_kinematic_millturn_tsv_end);
    register_embedded("system/machine/kinematic_mx850.tsv",
                      _binary_kinematic_mx850_tsv_start, _binary_kinematic_mx850_tsv_end);
    register_embedded("system/machine/embedded_toolpods.tsv",
                      _binary_embedded_toolpods_tsv_start, _binary_embedded_toolpods_tsv_end);
    register_embedded("system/machine/demo_box.obj",
                      _binary_demo_box_obj_start, _binary_demo_box_obj_end);
    register_embedded("system/machine/demo_part.stl",
                      _binary_demo_part_stl_start, _binary_demo_part_stl_end);
}

namespace {

// Per-file buffer pool for FS-backed entries. Kept simple — a contiguous
// arena allocated lazily on first mount. The VFS only registers pointers;
// the arena owns the bytes. Sized generously for a whole machine bundle
// (kinematic TSVs + a few MB of STL parts).
constexpr size_t kArenaBytes = 8 * 1024 * 1024;  // 8 MiB
alignas(8) char g_fs_arena[kArenaBytes];
size_t g_fs_arena_used = 0;

char* arena_alloc(size_t n) noexcept {
    if (g_fs_arena_used + n > kArenaBytes) return nullptr;
    char* p = &g_fs_arena[g_fs_arena_used];
    g_fs_arena_used += n;
    return p;
}

struct ScanCtx {
    kernel::hal::FileSystemOps* fs;
    size_t loaded;
};

// Called by the HAL backend's list() for every file in the tree. Each
// regular file's bytes are pulled into the arena and registered in the
// VFS under its full path. Directories are ignored (the flat VFS doesn't
// care about them — only leaf files).
bool scan_cb(const kernel::hal::FileSystemOps::Entry& entry, void* user) noexcept {
    auto* ctx = static_cast<ScanCtx*>(user);
    if (entry.is_dir) return true;
    if (entry.size == 0 || entry.size > kArenaBytes) return true;
    char* buf = arena_alloc(entry.size);
    if (!buf) return false;  // arena full — stop walking
    size_t got = 0;
    if (!ctx->fs->read(entry.name, buf, entry.size, &got) || got == 0) {
        // Roll back the allocation — nothing to register.
        g_fs_arena_used -= entry.size;
        return true;
    }
    (void)register_blob(entry.name, buf, got);
    ++ctx->loaded;
    return true;
}

}  // namespace

bool mount_sd() noexcept {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    auto* fs = kernel::g_platform ? kernel::g_platform->get_fs_ops() : nullptr;
    if (!fs) {
        if (uart) uart->puts("[vfs] no fs ops on this HAL — embedded-only\n");
        return false;
    }
    if (!fs->mount()) {
        if (uart) uart->puts("[vfs] fs mount failed — embedded-only\n");
        return false;
    }
    ScanCtx ctx{fs, 0};
    // Scan the whole FS rooted at `system/` — editor-authored bundles live
    // under `system/machine/` and `system/ui/`. Anything else the backend
    // exposes is also pulled in; the VFS doesn't enforce layout.
    (void)fs->list("system/", &scan_cb, &ctx);
    if (uart) {
        char msg[96];
        kernel::util::k_snprintf(msg, sizeof(msg),
                                 "[vfs] fs mount ok — %u files loaded, arena %u/%u bytes\n",
                                 static_cast<unsigned>(ctx.loaded),
                                 static_cast<unsigned>(g_fs_arena_used),
                                 static_cast<unsigned>(kArenaBytes));
        uart->puts(msg);
    }
    return ctx.loaded > 0;
}

}  // namespace kernel::vfs
