// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal virtio-gpu MMIO driver for QEMU virt.

#ifndef HAL_SHARED_VIRTIO_GPU_HPP
#define HAL_SHARED_VIRTIO_GPU_HPP

#include "hal.hpp"

#include <cstddef>
#include <cstdint>

namespace hal::shared::virtio_gpu {

constexpr uint32_t VIRTIO_DEV_ID_GPU = 16;

class VirtioGpuDriver : public kernel::hal::DisplayOps {
public:
    void configure_bus(uint64_t base, size_t slot_size, size_t slot_count);
    bool init(void* framebuffer, uint32_t width, uint32_t height,
              uint32_t stride) override;
    bool present() override;
    bool is_connected() override;
    void get_resolution(uint32_t& width, uint32_t& height) override;

    bool init(uint64_t slot_base, uint32_t* framebuffer, uint32_t width,
              uint32_t height, uint32_t stride_bytes);
    bool flush();

    bool initialized() const { return initialized_; }
    uint64_t base() const { return base_; }
    size_t slot_idx() const { return slot_idx_; }
    uint32_t flush_count() const { return flush_count_; }
    uint32_t fb_width() const { return width_; }
    uint32_t fb_height() const { return height_; }
    const uint32_t* fb_pixels() const { return framebuffer_; }

private:
    struct VirtqDesc {
        uint64_t addr;
        uint32_t len;
        uint16_t flags;
        uint16_t next;
    } __attribute__((packed));

    struct VirtqAvail {
        uint16_t flags;
        uint16_t idx;
        uint16_t ring[8];
        uint16_t used_event;
    } __attribute__((packed));

    struct VirtqUsedElem {
        uint32_t id;
        uint32_t len;
    } __attribute__((packed));

    struct VirtqUsed {
        uint16_t flags;
        uint16_t idx;
        VirtqUsedElem ring[8];
        uint16_t avail_event;
    } __attribute__((packed));

    struct Queue {
        alignas(16) VirtqDesc desc[8];
        alignas(2) VirtqAvail avail;
        alignas(4) VirtqUsed used;
        uint16_t last_used = 0;
    };

    struct CtrlHdr {
        uint32_t type;
        uint32_t flags;
        uint64_t fence_id;
        uint32_t ctx_id;
        uint32_t padding;
    } __attribute__((packed));

    struct Rect {
        uint32_t x;
        uint32_t y;
        uint32_t width;
        uint32_t height;
    } __attribute__((packed));

    struct ResourceCreate2D {
        CtrlHdr hdr;
        uint32_t resource_id;
        uint32_t format;
        uint32_t width;
        uint32_t height;
    } __attribute__((packed));

    struct ResourceAttachBacking {
        CtrlHdr hdr;
        uint32_t resource_id;
        uint32_t nr_entries;
    } __attribute__((packed));

    struct MemEntry {
        uint64_t addr;
        uint32_t length;
        uint32_t padding;
    } __attribute__((packed));

    struct SetScanout {
        CtrlHdr hdr;
        Rect rect;
        uint32_t scanout_id;
        uint32_t resource_id;
    } __attribute__((packed));

    struct TransferToHost2D {
        CtrlHdr hdr;
        Rect rect;
        uint64_t offset;
        uint32_t resource_id;
        uint32_t padding;
    } __attribute__((packed));

    struct ResourceFlush {
        CtrlHdr hdr;
        Rect rect;
        uint32_t resource_id;
        uint32_t padding;
    } __attribute__((packed));

    struct AttachBackingCmd {
        ResourceAttachBacking req;
        MemEntry entry;
    } __attribute__((packed));

    Queue queue_{};
    CtrlHdr response_{};
    uint64_t bus_base_ = 0;
    size_t slot_size_ = 0;
    size_t slot_count_ = 0;
    size_t slot_idx_ = 0;
    uint64_t base_ = 0;
    uint32_t* framebuffer_ = nullptr;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t stride_bytes_ = 0;
    uint32_t flush_count_ = 0;
    bool initialized_ = false;
    bool not_init_logged_ = false;

    uint32_t mmio_read(uint32_t off) const;
    void mmio_write(uint32_t off, uint32_t value);
    bool setup_queue(uint32_t queue_idx);
    bool submit_command(const void* req, uint32_t req_len, void* resp,
                        uint32_t resp_len);
    bool create_scanout_resource();
};

bool discover_and_init(uint64_t base, size_t slot_size, size_t slot_count,
                       VirtioGpuDriver& out, uint32_t* framebuffer,
                       uint32_t width, uint32_t height, uint32_t stride_bytes,
                       size_t* slot_idx = nullptr);

} // namespace hal::shared::virtio_gpu

#endif
