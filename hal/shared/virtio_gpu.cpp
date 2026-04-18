// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal virtio-gpu MMIO driver for QEMU virt.

#include "virtio_gpu.hpp"
#include "../../miniOS.hpp"
#include "../../util.hpp"

namespace hal::shared::virtio_gpu {

namespace {

constexpr uint32_t VMMIO_MAGIC         = 0x000;
constexpr uint32_t VMMIO_VERSION       = 0x004;
constexpr uint32_t VMMIO_DEVICE_ID     = 0x008;
constexpr uint32_t VMMIO_DEV_FEAT      = 0x010;
constexpr uint32_t VMMIO_DEV_FEAT_SEL  = 0x014;
constexpr uint32_t VMMIO_DRV_FEAT      = 0x020;
constexpr uint32_t VMMIO_DRV_FEAT_SEL  = 0x024;
constexpr uint32_t VMMIO_QUEUE_SEL     = 0x030;
constexpr uint32_t VMMIO_QUEUE_NUM_MAX = 0x034;
constexpr uint32_t VMMIO_QUEUE_NUM     = 0x038;
constexpr uint32_t VMMIO_QUEUE_READY   = 0x044;
constexpr uint32_t VMMIO_QUEUE_NOTIFY  = 0x050;
constexpr uint32_t VMMIO_STATUS        = 0x070;
constexpr uint32_t VMMIO_QUEUE_DESC_LO   = 0x080;
constexpr uint32_t VMMIO_QUEUE_DESC_HI   = 0x084;
constexpr uint32_t VMMIO_QUEUE_DRIVER_LO = 0x090;
constexpr uint32_t VMMIO_QUEUE_DRIVER_HI = 0x094;
constexpr uint32_t VMMIO_QUEUE_DEVICE_LO = 0x0a0;
constexpr uint32_t VMMIO_QUEUE_DEVICE_HI = 0x0a4;

constexpr uint32_t VIRTIO_STATUS_ACK       = 1u << 0;
constexpr uint32_t VIRTIO_STATUS_DRIVER    = 1u << 1;
constexpr uint32_t VIRTIO_STATUS_DRIVER_OK = 1u << 2;
constexpr uint32_t VIRTIO_STATUS_FEAT_OK   = 1u << 3;
constexpr uint32_t VIRTIO_STATUS_FAILED    = 1u << 7;

constexpr uint16_t VIRTQ_DESC_F_NEXT  = 1u;
constexpr uint16_t VIRTQ_DESC_F_WRITE = 2u;

constexpr uint64_t VIRTIO_F_VERSION_1 = 1ULL << 32;

constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_CREATE_2D     = 0x0101;
constexpr uint32_t VIRTIO_GPU_CMD_SET_SCANOUT            = 0x0103;
constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_FLUSH         = 0x0104;
constexpr uint32_t VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D    = 0x0105;
constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING = 0x0106;
constexpr uint32_t VIRTIO_GPU_RESP_OK_NODATA             = 0x1100;

constexpr uint32_t VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM = 1;
constexpr uint32_t RESOURCE_ID = 1;

inline uint32_t mmio_r32(uint64_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

inline void mmio_w32(uint64_t addr, uint32_t value) {
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

template <typename T>
T zeroed() {
    T v{};
    return v;
}

void gpu_log(const char* msg) {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (uart) uart->puts(msg);
}

inline kernel::hal::MemoryOps* mem_ops() {
    return (kernel::g_platform && kernel::g_platform->get_mem_ops())
        ? kernel::g_platform->get_mem_ops()
        : nullptr;
}

} // namespace

uint32_t VirtioGpuDriver::mmio_read(uint32_t off) const {
    return mmio_r32(base_ + off);
}

void VirtioGpuDriver::mmio_write(uint32_t off, uint32_t value) {
    mmio_w32(base_ + off, value);
}

void VirtioGpuDriver::configure_bus(uint64_t base, size_t slot_size,
                                    size_t slot_count) {
    bus_base_ = base;
    slot_size_ = slot_size;
    slot_count_ = slot_count;
}

bool VirtioGpuDriver::init(void* framebuffer, uint32_t width, uint32_t height,
                           uint32_t stride) {
    if (initialized_) return true;
    if (!framebuffer || width == 0 || height == 0 || stride == 0 ||
        bus_base_ == 0 || slot_size_ == 0 || slot_count_ == 0) {
        return false;
    }
    size_t slot = 0;
    const bool ok = discover_and_init(bus_base_, slot_size_, slot_count_, *this,
                                      static_cast<uint32_t*>(framebuffer),
                                      width, height, stride, &slot);
    if (ok) slot_idx_ = slot;
    return ok;
}

bool VirtioGpuDriver::present() {
    return flush();
}

bool VirtioGpuDriver::is_connected() {
    return initialized_;
}

void VirtioGpuDriver::get_resolution(uint32_t& width, uint32_t& height) {
    width = width_;
    height = height_;
}

bool VirtioGpuDriver::setup_queue(uint32_t queue_idx) {
    mmio_write(VMMIO_QUEUE_SEL, queue_idx);
    uint32_t max = mmio_read(VMMIO_QUEUE_NUM_MAX);
    if (max == 0 || max < 8) return false;
    mmio_write(VMMIO_QUEUE_NUM, 8);

    const uint64_t desc_pa  = reinterpret_cast<uint64_t>(&queue_.desc[0]);
    const uint64_t avail_pa = reinterpret_cast<uint64_t>(&queue_.avail);
    const uint64_t used_pa  = reinterpret_cast<uint64_t>(&queue_.used);

    mmio_write(VMMIO_QUEUE_DESC_LO, static_cast<uint32_t>(desc_pa));
    mmio_write(VMMIO_QUEUE_DESC_HI, static_cast<uint32_t>(desc_pa >> 32));
    mmio_write(VMMIO_QUEUE_DRIVER_LO, static_cast<uint32_t>(avail_pa));
    mmio_write(VMMIO_QUEUE_DRIVER_HI, static_cast<uint32_t>(avail_pa >> 32));
    mmio_write(VMMIO_QUEUE_DEVICE_LO, static_cast<uint32_t>(used_pa));
    mmio_write(VMMIO_QUEUE_DEVICE_HI, static_cast<uint32_t>(used_pa >> 32));
    mmio_write(VMMIO_QUEUE_READY, 1);
    return true;
}

bool VirtioGpuDriver::submit_command(const void* req, uint32_t req_len, void* resp,
                                     uint32_t resp_len) {
    if (!initialized_) return false;

    queue_.desc[0].addr = reinterpret_cast<uint64_t>(req);
    queue_.desc[0].len = req_len;
    queue_.desc[0].flags = VIRTQ_DESC_F_NEXT;
    queue_.desc[0].next = 1;

    queue_.desc[1].addr = reinterpret_cast<uint64_t>(resp);
    queue_.desc[1].len = resp_len;
    queue_.desc[1].flags = VIRTQ_DESC_F_WRITE;
    queue_.desc[1].next = 0;

    const uint16_t avail_pos = queue_.avail.idx % 8;
    queue_.avail.ring[avail_pos] = 0;
    if (auto* mem = mem_ops()) {
        mem->flush_cache_range(req, req_len);
        mem->flush_cache_range(resp, resp_len);
        mem->flush_cache_range(&queue_.desc[0], sizeof(queue_.desc[0]) * 2);
        mem->flush_cache_range(&queue_.avail.ring[avail_pos], sizeof(queue_.avail.ring[avail_pos]));
    }
    __atomic_store_n(&queue_.avail.idx, static_cast<uint16_t>(queue_.avail.idx + 1),
                     __ATOMIC_RELEASE);
    if (auto* mem = mem_ops()) {
        mem->flush_cache_range(&queue_.avail, sizeof(queue_.avail));
    }
    mmio_write(VMMIO_QUEUE_NOTIFY, 0);

    uint32_t spins = 0;
    while (__atomic_load_n(&queue_.used.idx, __ATOMIC_ACQUIRE) == queue_.last_used) {
        if (auto* mem = mem_ops()) {
            mem->invalidate_cache_range(&queue_.used, sizeof(queue_.used));
            mem->invalidate_cache_range(resp, resp_len);
        }
        if (++spins == 50000000u) {
            gpu_log("[virtio-gpu] command timeout waiting used ring\n");
            return false;
        }
        kernel::util::cpu_relax();
    }
    queue_.last_used = __atomic_load_n(&queue_.used.idx, __ATOMIC_RELAXED);

    const auto* hdr = reinterpret_cast<const CtrlHdr*>(resp);
    if (hdr->type != VIRTIO_GPU_RESP_OK_NODATA) {
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[virtio-gpu] bad response type=0x%x\n",
                                 static_cast<unsigned>(hdr->type));
        gpu_log(buf);
    }
    return hdr->type == VIRTIO_GPU_RESP_OK_NODATA;
}

bool VirtioGpuDriver::create_scanout_resource() {
    const Rect rect{0, 0, width_, height_};

    auto create = zeroed<ResourceCreate2D>();
    create.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    create.resource_id = RESOURCE_ID;
    create.format = VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM;
    create.width = width_;
    create.height = height_;
    response_ = {};
    if (!submit_command(&create, sizeof(create), &response_, sizeof(response_))) {
        gpu_log("[virtio-gpu] RESOURCE_CREATE_2D failed\n");
        return false;
    }

    auto attach = zeroed<AttachBackingCmd>();
    attach.req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
    attach.req.resource_id = RESOURCE_ID;
    attach.req.nr_entries = 1;
    attach.entry.addr = reinterpret_cast<uint64_t>(framebuffer_);
    attach.entry.length = stride_bytes_ * height_;
    response_ = {};
    if (!submit_command(&attach, sizeof(attach), &response_, sizeof(response_))) {
        gpu_log("[virtio-gpu] RESOURCE_ATTACH_BACKING failed\n");
        return false;
    }

    auto set_scanout = zeroed<SetScanout>();
    set_scanout.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    set_scanout.rect = rect;
    set_scanout.scanout_id = 0;
    set_scanout.resource_id = RESOURCE_ID;
    response_ = {};
    if (!submit_command(&set_scanout, sizeof(set_scanout), &response_, sizeof(response_))) {
        gpu_log("[virtio-gpu] SET_SCANOUT failed\n");
        return false;
    }
    return true;
}

bool VirtioGpuDriver::init(uint64_t slot_base, uint32_t* framebuffer,
                            uint32_t width, uint32_t height,
                            uint32_t stride_bytes) {
    base_ = slot_base;
    framebuffer_ = framebuffer;
    width_ = width;
    height_ = height;
    stride_bytes_ = stride_bytes;

    if (mmio_read(VMMIO_MAGIC) != 0x74726976) {
        gpu_log("[virtio-gpu] bad magic\n");
        return false;
    }
    if (mmio_read(VMMIO_VERSION) != 2) {
        gpu_log("[virtio-gpu] bad version\n");
        return false;
    }
    if (mmio_read(VMMIO_DEVICE_ID) != VIRTIO_DEV_ID_GPU) {
        gpu_log("[virtio-gpu] bad device id\n");
        return false;
    }

    mmio_write(VMMIO_STATUS, 0);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);

    mmio_write(VMMIO_DEV_FEAT_SEL, 0);
    uint64_t dev_feat = mmio_read(VMMIO_DEV_FEAT);
    mmio_write(VMMIO_DEV_FEAT_SEL, 1);
    dev_feat |= static_cast<uint64_t>(mmio_read(VMMIO_DEV_FEAT)) << 32;
    if ((dev_feat & VIRTIO_F_VERSION_1) == 0) {
        gpu_log("[virtio-gpu] missing version1 feature\n");
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    mmio_write(VMMIO_DRV_FEAT_SEL, 0);
    mmio_write(VMMIO_DRV_FEAT, 0);
    mmio_write(VMMIO_DRV_FEAT_SEL, 1);
    mmio_write(VMMIO_DRV_FEAT, 1);

    mmio_write(VMMIO_STATUS,
               VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK);
    if ((mmio_read(VMMIO_STATUS) & VIRTIO_STATUS_FEAT_OK) == 0) {
        gpu_log("[virtio-gpu] feature negotiation failed\n");
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    if (!setup_queue(0)) {
        gpu_log("[virtio-gpu] queue setup failed\n");
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
                                  VIRTIO_STATUS_FEAT_OK | VIRTIO_STATUS_DRIVER_OK);
    initialized_ = true;

    if (!create_scanout_resource()) {
        gpu_log("[virtio-gpu] scanout resource setup failed\n");
        initialized_ = false;
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }
    gpu_log("[virtio-gpu] scanout resource ready\n");

    if (!flush()) {
        gpu_log("[virtio-gpu] initial flush failed\n");
        return false;
    }
    gpu_log("[virtio-gpu] init ok\n");
    return true;
}

bool VirtioGpuDriver::flush() {
    if (!initialized_) {
        // Log once: the UI main loop flushes ~10 Hz, so spamming this every
        // frame on platforms where the GPU didn't probe (e.g. RV64 today)
        // drowns out the rest of the serial log.
        if (!not_init_logged_) {
            gpu_log("[virtio-gpu] flush called but driver not initialized\n");
            not_init_logged_ = true;
        }
        return false;
    }
    if (auto* mem = mem_ops()) {
        mem->flush_cache_range(framebuffer_, stride_bytes_ * height_);
    }
    const Rect rect{0, 0, width_, height_};

    auto transfer = zeroed<TransferToHost2D>();
    transfer.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
    transfer.rect = rect;
    transfer.offset = 0;
    transfer.resource_id = RESOURCE_ID;
    response_ = {};
    if (!submit_command(&transfer, sizeof(transfer), &response_, sizeof(response_))) return false;

    auto flush_cmd = zeroed<ResourceFlush>();
    flush_cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
    flush_cmd.rect = rect;
    flush_cmd.resource_id = RESOURCE_ID;
    response_ = {};
    const bool ok = submit_command(&flush_cmd, sizeof(flush_cmd), &response_, sizeof(response_));
    ++flush_count_;
    if (!ok) gpu_log("[virtio-gpu] RESOURCE_FLUSH failed\n");
    // Heartbeat every 16 flushes (~1.6 s at UI loop's 100 ms cadence) so a
    // silent GPU path is diagnosable from the serial log without a CLI stat
    // dump. Cheap enough not to clog serial even at sustained 60 Hz.
    if ((flush_count_ & 0xF) == 0) {
        char buf[64];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[virtio-gpu] flushes=%u\n",
                                 static_cast<unsigned>(flush_count_));
        gpu_log(buf);
    }
    return ok;
}

bool discover_and_init(uint64_t base, size_t slot_size, size_t slot_count,
                        VirtioGpuDriver& out, uint32_t* framebuffer,
                        uint32_t width, uint32_t height, uint32_t stride_bytes,
                        size_t* slot_idx) {
    for (size_t i = 0; i < slot_count; ++i) {
        const uint64_t slot_base = base + i * slot_size;
        uint32_t magic = mmio_r32(slot_base + VMMIO_MAGIC);
        if (magic != 0x74726976) continue;
        const uint32_t dev_id = mmio_r32(slot_base + VMMIO_DEVICE_ID);
        if (dev_id != VIRTIO_DEV_ID_GPU) continue;
        char buf[96];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[virtio-gpu] found dev at slot %u (base=0x%llx) %ux%u\n",
                                 static_cast<unsigned>(i),
                                 static_cast<unsigned long long>(slot_base),
                                 static_cast<unsigned>(width),
                                 static_cast<unsigned>(height));
        gpu_log(buf);
        if (!out.init(slot_base, framebuffer, width, height, stride_bytes)) return false;
        if (slot_idx) *slot_idx = i;
        return true;
    }
    gpu_log("[virtio-gpu] no device found\n");
    return false;
}

} // namespace hal::shared::virtio_gpu
