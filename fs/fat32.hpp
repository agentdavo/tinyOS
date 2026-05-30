// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal FAT32 reader with a small writable surface for setup persistence.
//
// Read side:
//   - FAT32 only (no FAT12/16 fallback; mkfs.vfat -F 32 required)
//   - Short 8.3 names AND VFAT Long File Names (ASCII-range UCS-2)
//   - Nested subdirectories (arbitrary depth, path-style walks)
//   - open(path) -> file size + cluster chain walk
//
// Write side (deliberately scoped — see write_file/rename_file):
//   - Root directory only (no subdirectory creation)
//   - 8.3 short names only (no LFN write)
//   - Create-or-overwrite a single file, bounded by kMaxWriteFileBytes
//   - Atomic-ish rename within the root via dir-entry name mutation
//
// These limits are enough for setup.cfg persistence today and keep the
// write surface small/auditable. Larger-payload or directory-aware writes
// will need more work (LFN encode, free-cluster bitmap, etc.).
//
// Blocking, single-threaded, polled block I/O through a caller-provided
// sector-reader callback. That keeps the FAT layer independent of
// virtio-blk so the same parser runs against an SPI-SD backend later.

#ifndef FS_FAT32_HPP
#define FS_FAT32_HPP

#include <cstddef>
#include <cstdint>

namespace fs::fat32 {

constexpr size_t MAX_PATH_LEN = 96;
constexpr size_t SECTOR_SIZE = 512;

// Sector read callback: fill `buf` with `count` sectors starting at `lba`.
// Returns true on success.
using SectorReader = bool (*)(uint64_t lba, uint32_t count, void* buf, void* user);

// Sector write callback: drain `count` sectors from `buf` to `lba`. Optional
// — when null, the write paths return false and behave read-only.
using SectorWriter = bool (*)(uint64_t lba, uint32_t count, const void* buf, void* user);

// Hard cap for write_file. Plenty for setup.cfg (kMaxBufBytes = 16 KiB in
// cnc/setup.cpp) plus headroom for a few small config blobs. Larger files
// would need a free-cluster-bitmap walk and aren't on the road today.
constexpr size_t kMaxWriteFileBytes = 64 * 1024;

struct Volume {
    SectorReader read = nullptr;
    SectorWriter write = nullptr;
    void* user = nullptr;

    uint32_t bytes_per_sector = 0;
    uint32_t sectors_per_cluster = 0;
    uint32_t reserved_sectors = 0;
    uint32_t num_fats = 0;
    uint32_t fat_size_sectors = 0;
    uint32_t root_cluster = 0;
    uint64_t fat_start_lba = 0;
    uint64_t data_start_lba = 0;
    // Number of data clusters in the volume (derived at mount from the FAT32
    // total-sector count). Valid data cluster numbers are [2, total_clusters+2).
    // Used to reject out-of-range cluster numbers from a corrupt FAT/dir entry
    // before they turn into an arbitrary cluster_to_lba() block access.
    uint32_t total_clusters = 0;
    bool mounted = false;
};

// Mount via `read` over the backing block device. Returns false if the
// boot sector isn't a readable FAT32. `write` may be null for read-only
// backends; the write APIs below then return false up front.
bool mount(Volume& vol, SectorReader read, void* user);
bool mount(Volume& vol, SectorReader read, SectorWriter write, void* user);

// Walk file entries in the given path prefix (e.g. "system/machine").
// `cb` is called for every file (not directories) whose full path lies
// at or below `prefix`. `full_path` passed to the callback is the
// forward-slash path relative to the FS root ("system/machine/foo.tsv").
// Returning false from cb stops the walk.
using WalkCb = bool (*)(const char* full_path, uint32_t size, void* cb_user);
bool walk(Volume& vol, const char* prefix, WalkCb cb, void* cb_user);

// Read `path` from the volume into `buf`. At most `buf_size` bytes are
// written; `*bytes_read` ends up holding the actual size. Returns false
// if the file is missing or unreadable.
bool read_file(Volume& vol, const char* path, void* buf, size_t buf_size,
               size_t* bytes_read);

// Create-or-overwrite `path` (root-only, 8.3 only) with `bytes` from `buf`.
// Returns false if path is malformed, the volume is read-only, the file
// exceeds kMaxWriteFileBytes, the directory is full, or the FAT runs out
// of free clusters. Writes ALL FAT copies (vol.num_fats) so chkdsk on the
// host doesn't flag mismatches.
bool write_file(Volume& vol, const char* path, const void* buf, size_t bytes);

// Rename `from` -> `to` within the root directory (no cross-dir moves).
// If `to` already exists, its directory entry is freed first (deleted but
// allocated clusters are reclaimed via the FAT). Returns false on missing
// source or malformed paths.
bool rename_file(Volume& vol, const char* from, const char* to);

}  // namespace fs::fat32

#endif
