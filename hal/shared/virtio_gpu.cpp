// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal virtio-gpu MMIO driver for QEMU virt.

#include "virtio_gpu.hpp"
#include "virtio_mmio.hpp"
#include "../../miniOS.hpp"
#include "../../util.hpp"

namespace hal::shared::virtio_gpu {

using namespace ::hal::shared::virtio_mmio;

namespace {

constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_CREATE_2D     = 0x0101;
constexpr uint32_t VIRTIO_GPU_CMD_SET_SCANOUT            = 0x0103;
constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_FLUSH         = 0x0104;
constexpr uint32_t VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D    = 0x0105;
constexpr uint32_t VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING = 0x0106;
constexpr uint32_t VIRTIO_GPU_RESP_OK_NODATA             = 0x1100;

constexpr uint32_t VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM = 1;
constexpr uint32_t RESOURCE_ID = 1;

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

uint32_t VirtioGpuDriver::mmio_read(uint32_t off) const { return virtio_mmio::read32(base_, off); }
void     VirtioGpuDriver::mmio_write(uint32_t off, uint32_t v) { virtio_mmio::write32(base_, off, v); }

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

bool VirtioGpuDriver::present_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    return flush_rect(x, y, w, h);
}

bool VirtioGpuDriver::is_connected() {
    return initialized_;
}

void VirtioGpuDriver::get_resolution(uint32_t& width, uint32_t& height) {
    width = width_;
    height = height_;
}

bool VirtioGpuDriver::setup_queue(uint32_t queue_idx) {
    return virtio_mmio::setup_queue(base_, queue_idx, 8, &queue_.desc[0], &queue_.avail, &queue_.used);
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

    InitError err = InitError::None;
    if (!virtio_mmio::begin(base_, VIRTIO_DEV_ID_GPU, 0, nullptr, &err)) {
        switch (err) {
            case InitError::BadMagic:         gpu_log("[virtio-gpu] bad magic\n"); break;
            case InitError::BadVersion:       gpu_log("[virtio-gpu] bad version\n"); break;
            case InitError::WrongDevice:      gpu_log("[virtio-gpu] bad device id\n"); break;
            case InitError::NoVersion1:       gpu_log("[virtio-gpu] missing version1 feature\n"); break;
            case InitError::FeaturesRejected: gpu_log("[virtio-gpu] feature negotiation failed\n"); break;
            case InitError::None:             break;
        }
        return false;
    }

    if (!setup_queue(0)) {
        gpu_log("[virtio-gpu] queue setup failed\n");
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    virtio_mmio::driver_ok(base_);
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
    return flush_rect(0, 0, width_, height_);
}

bool VirtioGpuDriver::flush_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
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
    // Clip the damage rect against the scanout. Negative / zero w/h is a
    // no-op present — the caller will have nothing to display.
    if (x >= width_ || y >= height_) return true;
    if (w == 0 || h == 0) return true;
    if (x + w > width_)  w = width_  - x;
    if (y + h > height_) h = height_ - y;

    // Only flush the dirty band's cache lines, not the whole framebuffer.
    if (auto* mem = mem_ops()) {
        const size_t row_bytes = stride_bytes_;
        const uintptr_t base = reinterpret_cast<uintptr_t>(framebuffer_) +
                               static_cast<uintptr_t>(y) * row_bytes;
        mem->flush_cache_range(reinterpret_cast<void*>(base), row_bytes * h);
    }
    const Rect rect{x, y, w, h};
    const uint64_t offset = static_cast<uint64_t>(y) * stride_bytes_ +
                            static_cast<uint64_t>(x) * 4u;

    auto transfer = zeroed<TransferToHost2D>();
    transfer.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
    transfer.rect = rect;
    transfer.offset = offset;
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
    // No periodic "[virtio-gpu] flushes=N" line any more: at 10 Hz it
    // interleaved with the operator's CLI prompt every 1.6 s.
    if (!ok) gpu_log("[virtio-gpu] RESOURCE_FLUSH failed\n");
    return ok;
}

bool discover_and_init(uint64_t base, size_t slot_size, size_t slot_count,
                        VirtioGpuDriver& out, uint32_t* framebuffer,
                        uint32_t width, uint32_t height, uint32_t stride_bytes,
                        size_t* slot_idx) {
    for (size_t i = 0; i < slot_count; ++i) {
        const uint64_t slot_base = base + i * slot_size;
        uint32_t magic = virtio_mmio::read32(slot_base, VMMIO_MAGIC);
        if (magic != VIRTIO_MMIO_MAGIC_VALUE) continue;
        const uint32_t dev_id = virtio_mmio::read32(slot_base, VMMIO_DEVICE_ID);
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
