// SPDX-License-Identifier: MIT OR Apache-2.0
// FileSystemOps concrete impl over FAT32 + a block-device backend.
//
// The backend is any object exposing `read_sectors(lba, count, void*)` —
// today that's `hal::shared::virtio::VirtioBlkDriver` under QEMU, but the
// same class accepts a `BlockReader*` so a future SPI-SD-over-MMIO driver
// plugs in without touching this file.

#ifndef FS_FS_FAT32_HPP
#define FS_FS_FAT32_HPP

#include "hal.hpp"
#include "fat32.hpp"

namespace fs {

// Minimal block-device interface. Any backend that can read 512-byte
// sectors implements this.
struct BlockReader {
    virtual ~BlockReader() = default;
    virtual bool read_sectors(uint64_t lba, uint32_t count, void* buf) = 0;
};

class Fat32FileSystem : public kernel::hal::FileSystemOps {
public:
    explicit Fat32FileSystem(BlockReader* blk) : blk_(blk) {}

    bool mount() override;
    void unmount() override;
    bool is_mounted() const override { return vol_.mounted; }
    bool list(const char* prefix, WalkFn fn, void* user) override;
    bool read(const char* path, void* buf, size_t buf_size, size_t* bytes_read) override;
    size_t file_size(const char* path) override;

private:
    static bool sector_trampoline(uint64_t lba, uint32_t count, void* buf, void* user);

    BlockReader* blk_ = nullptr;
    fat32::Volume vol_{};
};

}  // namespace fs

#endif
