// SPDX-License-Identifier: MIT OR Apache-2.0
// miniOS virtual filesystem.
//
// A tiny name-keyed lookup over {ptr,size} blobs. Every piece of kernel-side
// data that used to be reached via `_binary_*_start/end` is now registered
// here under a canonical path, so the UI builder, kinematic loader, OBJ
// registry, macro/ladder/hmi loaders all call `vfs::lookup(path, ...)`
// without caring whether the bytes came from:
//
//   - embedded `.incbin` blobs seeded at boot (always present — the ROM)
//   - an SD card mount layered on top (optional, shadows embedded)
//   - an HMI upload path (future; same API)
//
// The canonical layout the machine/UI editors produce:
//
//   system/machine/kinematic_mill3.tsv
//   system/machine/kinematic_millturn.tsv
//   system/machine/embedded_toolpods.tsv
//   system/machine/*.obj
//   system/ui/embedded_ui.tsv
//
// Lookup is O(n) over the static entry table. n <= kMaxEntries (32 today)
// and every lookup happens at boot or once per page switch, so the constant
// factor doesn't matter.

#ifndef FS_VFS_HPP
#define FS_VFS_HPP

#include <cstddef>
#include <cstdint>

namespace kernel::vfs {

constexpr size_t kMaxEntries = 32;
constexpr size_t kMaxPathLen = 96;

// Register a blob under `path`. If the path already has an entry, overwrite
// it — this is how SD / HMI backends shadow the embedded defaults. Return
// false only on capacity exhaustion.
bool register_blob(const char* path, const char* data, size_t size) noexcept;

// Retrieve the blob registered under `path`. Returns false if unknown. On
// success, `*data_out` and `*size_out` point into static memory owned by
// whoever registered the blob (usually `.rodata` for embedded, or the SD
// mount's cached buffer once that backend lands).
bool lookup(const char* path, const char*& data_out, size_t& size_out) noexcept;

// Iterate every entry whose path starts with `prefix`. `prefix` may be the
// empty string to walk everything. Callback receives (path, data, size).
// Return false from the callback to stop iteration early.
using WalkFn = bool(*)(const char* path, const char* data, size_t size, void* user);
void walk(const char* prefix, WalkFn fn, void* user) noexcept;

size_t entry_count() noexcept;

// Seed the VFS from the kernel's embedded `.incbin` blobs. Called once, very
// early in boot, before any subsystem that consumes VFS paths. Safe to call
// more than once — re-registration is a no-op for unchanged entries.
void register_embedded_defaults() noexcept;

// Attach an SD card backend. Today this is a stub (logs "not implemented"
// and returns false) — the FAT reader + SD scan is the next session. When
// it lands, this call will mount the card, walk `system/machine/` and
// `system/ui/`, and register_blob() every file it finds (shadowing the
// embedded defaults). Return false if no card is present / FS is unreadable.
bool mount_sd() noexcept;

}  // namespace kernel::vfs

#endif
