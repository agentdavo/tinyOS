// SPDX-License-Identifier: MIT OR Apache-2.0

#include "fs_fat32.hpp"

#include <cstring>

namespace fs {

namespace {

// Bridge the kernel::hal::FileSystemOps WalkFn (which takes an `Entry`) to
// the fat32::WalkCb (which just gives name + size). file entries only —
// FAT32 walk() skips dirs in emission already.
struct WalkBridge {
    kernel::hal::FileSystemOps::WalkFn fn;
    void* user;
};

bool walk_trampoline(const char* full_path, uint32_t size, void* user) {
    auto* b = static_cast<WalkBridge*>(user);
    kernel::hal::FileSystemOps::Entry e{};
    size_t n = 0;
    while (full_path[n] && n + 1 < sizeof(e.name)) { e.name[n] = full_path[n]; ++n; }
    e.name[n] = '\0';
    e.size = size;
    e.is_dir = false;
    return b->fn(e, b->user);
}

}  // namespace

bool Fat32FileSystem::sector_trampoline(uint64_t lba, uint32_t count, void* buf,
                                        void* user) {
    auto* self = static_cast<Fat32FileSystem*>(user);
    if (!self->blk_) return false;
    return self->blk_->read_sectors(lba, count, buf);
}

bool Fat32FileSystem::mount() {
    if (!blk_) return false;
    if (vol_.mounted) return true;
    return fat32::mount(vol_, &Fat32FileSystem::sector_trampoline, this);
}

void Fat32FileSystem::unmount() {
    vol_ = fat32::Volume{};
}

bool Fat32FileSystem::list(const char* prefix, WalkFn fn, void* user) {
    if (!vol_.mounted || !fn) return false;
    WalkBridge b{fn, user};
    return fat32::walk(vol_, prefix ? prefix : "", &walk_trampoline, &b);
}

bool Fat32FileSystem::read(const char* path, void* buf, size_t buf_size,
                           size_t* bytes_read) {
    if (!vol_.mounted) return false;
    size_t got = 0;
    const bool ok = fat32::read_file(vol_, path, buf, buf_size, &got);
    if (bytes_read) *bytes_read = got;
    return ok;
}

size_t Fat32FileSystem::file_size(const char* path) {
    // FAT32 doesn't expose a stat primitive that's cheaper than an open —
    // fall back to the generic behaviour (zero means "I don't know; read
    // up to your buffer size"). The VFS arena allocator passes the known
    // buffer size anyway, so this is only called when an external caller
    // needs to size their buffer.
    (void)path;
    return 0;
}

}  // namespace fs
