// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal read-only FAT32 reader.
//
// Supports:
//   - FAT32 only (no FAT12/16 fallback; mkfs.vfat -F 32 required)
//   - Short 8.3 names AND VFAT Long File Names (ASCII-range UCS-2)
//   - Nested subdirectories (arbitrary depth, path-style walks)
//   - open(path) -> file size + cluster chain walk
//
// Does NOT support: writes, file creation, attribute changes, timestamps,
// international characters beyond 7-bit ASCII. The tool chain populating
// the card (mkfs.vfat + mcopy) handles those concerns on the host side.
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

struct Volume {
    SectorReader read = nullptr;
    void* user = nullptr;

    uint32_t bytes_per_sector = 0;
    uint32_t sectors_per_cluster = 0;
    uint32_t reserved_sectors = 0;
    uint32_t num_fats = 0;
    uint32_t fat_size_sectors = 0;
    uint32_t root_cluster = 0;
    uint64_t fat_start_lba = 0;
    uint64_t data_start_lba = 0;
    bool mounted = false;
};

// Mount via `read` over the backing block device. Returns false if the
// boot sector isn't a readable FAT32.
bool mount(Volume& vol, SectorReader read, void* user);

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

}  // namespace fs::fat32

#endif
