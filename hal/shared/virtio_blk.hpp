// SPDX-License-Identifier: MIT OR Apache-2.0
// Virtio-mmio block driver (transport v2, blocking polled I/O).
//
// Polling-only design: read_sectors() submits a request, busy-waits on the
// used ring, returns the data. That's fine — the block device is only
// consulted at boot (vfs::mount_sd) and later from the CLI for manual
// reload, never from the 250 us motion cycle. Read-only; writes are
// deferred until the operator-facing "save setup to SD" story matters.
//
// QEMU invocation:
//   -drive file=sdcard.img,format=raw,if=none,id=sd0
//   -device virtio-blk-device,drive=sd0
//
// Attaches to the same virtio-mmio bus as virtio-net (slot scan, 0x200-byte
// slots on arm64 virt). Bind via `discover_virtio_blk(base, ...)` — the
// scanner picks up the first device with ID 2.

#ifndef HAL_SHARED_VIRTIO_BLK_HPP
#define HAL_SHARED_VIRTIO_BLK_HPP

#include "virtio_net.hpp"  // shared virtio-mmio register layout + VirtqDesc/Avail/Used

#include <array>
#include <atomic>
#include <cstdint>

namespace hal::shared::virtio {

constexpr uint32_t VIRTIO_DEV_ID_BLK = 2;

// Virtio-block request types.
constexpr uint32_t VIRTIO_BLK_T_IN  = 0;
constexpr uint32_t VIRTIO_BLK_T_OUT = 1;

// Virtio-block status codes.
constexpr uint8_t VIRTIO_BLK_S_OK     = 0;
constexpr uint8_t VIRTIO_BLK_S_IOERR  = 1;
constexpr uint8_t VIRTIO_BLK_S_UNSUPP = 2;

constexpr size_t SECTOR_SIZE = 512;
// Max sectors per single request. 8 descriptors split into [header][data*6][status],
// so cap the data at 6 * 512 = 3 KiB. FAT code calls one sector at a time; this
// limit is just a sanity guard.
constexpr size_t MAX_SECTORS_PER_REQ = 6;

struct VirtioBlkReqHeader {
    uint32_t type;
    uint32_t reserved;
    uint64_t sector;
} __attribute__((packed));

struct VirtioBlkConfig {
    uint64_t capacity;      // sectors
    uint32_t size_max;
    uint32_t seg_max;
    // ...followed by geometry, blk_size, topology — we only need capacity.
} __attribute__((packed));

class VirtioBlkDriver {
public:
    // Bind to the virtio-blk at `slot_base`. Returns false if this slot has
    // no virtio-blk or init fails. Stays blocking/polling-only.
    bool init(uint64_t slot_base);

    // Total device capacity in 512-byte sectors, 0 if uninitialised.
    uint64_t capacity_sectors() const { return capacity_; }

    // Read `count` consecutive sectors starting at `lba` into `buf`. `buf`
    // must be at least `count * SECTOR_SIZE` bytes. Returns true on success;
    // on error returns false and leaves `buf` in an indeterminate state.
    bool read_sectors(uint64_t lba, uint32_t count, void* buf);

    bool initialized() const { return initialized_; }
    uint64_t base() const { return base_; }

private:
    struct Queue {
        alignas(16) VirtqDesc  desc[VIRTQ_SIZE];
        alignas(2)  VirtqAvail avail;
        alignas(4)  VirtqUsed  used;
        uint16_t    last_used = 0;
    };
    alignas(4096) Queue q_{};

    // Pinned scratch for the request. Separate header / status bytes so
    // the device can see each as a distinct descriptor.
    alignas(16) VirtioBlkReqHeader req_hdr_{};
    alignas(16) uint8_t status_byte_ = 0;

    uint64_t base_ = 0;
    uint64_t capacity_ = 0;
    bool     initialized_ = false;

    uint32_t mmio_read(uint32_t off) const;
    void     mmio_write(uint32_t off, uint32_t val);
    bool     setup_queue();
};

// Scan [base, base + slot_count * slot_size) for virtio-blk devices and
// return a pointer to a static driver if one is found. Returns nullptr if
// no card is attached. Only one block device is supported today.
VirtioBlkDriver* discover_virtio_blk(uint64_t base, size_t slot_size, size_t slot_count);

}  // namespace hal::shared::virtio

#endif
