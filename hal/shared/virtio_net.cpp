// SPDX-License-Identifier: MIT OR Apache-2.0
// Arch-generic virtio-mmio / virtio-net driver.
//
// Legacy-to-modern hybrid: we request transport v2 and refuse v1 devices. No
// event-index, no mergeable RX buffers, no checksum offload. Good enough for
// L2 frame TX/RX on QEMU's SLIRP or socket backends.

#include "virtio_net.hpp"
#include "../../miniOS.hpp"
#include "../../util.hpp"

#include <cstring>

namespace hal::shared::virtio {

namespace {

inline uint32_t mmio_r32(uint64_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}
inline void mmio_w32(uint64_t addr, uint32_t v) {
    *reinterpret_cast<volatile uint32_t*>(addr) = v;
}

} // namespace

uint32_t VirtioNetDriver::mmio_read(uint32_t off) const  { return mmio_r32(base_ + off); }
void     VirtioNetDriver::mmio_write(uint32_t off, uint32_t v) { mmio_w32(base_ + off, v); }

bool VirtioNetDriver::setup_queue(uint32_t queue_idx, Queue& q) {
    mmio_write(VMMIO_QUEUE_SEL, queue_idx);
    uint32_t max = mmio_read(VMMIO_QUEUE_NUM_MAX);
    if (max == 0 || max < VIRTQ_SIZE) return false;
    mmio_write(VMMIO_QUEUE_NUM, VIRTQ_SIZE);

    uint64_t desc_pa   = reinterpret_cast<uint64_t>(&q.desc[0]);
    uint64_t avail_pa  = reinterpret_cast<uint64_t>(&q.avail);
    uint64_t used_pa   = reinterpret_cast<uint64_t>(&q.used);

    mmio_write(VMMIO_QUEUE_DESC_LO,   desc_pa & 0xFFFFFFFF);
    mmio_write(VMMIO_QUEUE_DESC_HI,   desc_pa >> 32);
    mmio_write(VMMIO_QUEUE_DRIVER_LO, avail_pa & 0xFFFFFFFF);
    mmio_write(VMMIO_QUEUE_DRIVER_HI, avail_pa >> 32);
    mmio_write(VMMIO_QUEUE_DEVICE_LO, used_pa & 0xFFFFFFFF);
    mmio_write(VMMIO_QUEUE_DEVICE_HI, used_pa >> 32);
    mmio_write(VMMIO_QUEUE_READY, 1);
    return true;
}

void VirtioNetDriver::post_rx_buffers() {
    for (size_t i = 0; i < VIRTQ_SIZE; ++i) {
        rx_queue_.desc[i].addr  = reinterpret_cast<uint64_t>(&rx_bufs_[i * NET_BUF_SIZE]);
        rx_queue_.desc[i].len   = NET_BUF_SIZE;
        rx_queue_.desc[i].flags = VIRTQ_DESC_F_WRITE;
        rx_queue_.desc[i].next  = 0;
        rx_queue_.avail.ring[i] = static_cast<uint16_t>(i);
    }
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        auto* mem = kernel::g_platform->get_mem_ops();
        mem->flush_cache_range(&rx_queue_.desc[0], sizeof(rx_queue_.desc));
        mem->flush_cache_range(&rx_queue_.avail, sizeof(rx_queue_.avail));
    }
    __atomic_store_n(&rx_queue_.avail.idx, static_cast<uint16_t>(VIRTQ_SIZE),
                     __ATOMIC_RELEASE);
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&rx_queue_.avail, sizeof(rx_queue_.avail));
    }
    kernel::hal::sync::barrier_dsb();
    mmio_write(VMMIO_QUEUE_NOTIFY, 0); // queue 0 = RX
}

bool VirtioNetDriver::init(uint64_t slot_base) {
    base_ = slot_base;

    if (mmio_read(VMMIO_MAGIC) != 0x74726976) return false;
    uint32_t ver = mmio_read(VMMIO_VERSION);
    if (ver != 2) return false;
    if (mmio_read(VMMIO_DEVICE_ID) != VIRTIO_DEV_ID_NET) return false;

    mmio_write(VMMIO_STATUS, 0);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK);
    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);

    constexpr uint64_t VIRTIO_NET_F_MAC   = 1ULL << 5;
    constexpr uint64_t VIRTIO_F_VERSION_1 = 1ULL << 32;
    mmio_write(VMMIO_DEV_FEAT_SEL, 0);
    uint64_t dev_feat = mmio_read(VMMIO_DEV_FEAT);
    mmio_write(VMMIO_DEV_FEAT_SEL, 1);
    dev_feat |= static_cast<uint64_t>(mmio_read(VMMIO_DEV_FEAT)) << 32;

    uint64_t drv_feat = (dev_feat & VIRTIO_NET_F_MAC) | VIRTIO_F_VERSION_1;
    if ((dev_feat & VIRTIO_F_VERSION_1) == 0) {
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }
    mmio_write(VMMIO_DRV_FEAT_SEL, 0);
    mmio_write(VMMIO_DRV_FEAT, static_cast<uint32_t>(drv_feat));
    mmio_write(VMMIO_DRV_FEAT_SEL, 1);
    mmio_write(VMMIO_DRV_FEAT, static_cast<uint32_t>(drv_feat >> 32));

    mmio_write(VMMIO_STATUS,
               VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK);
    if ((mmio_read(VMMIO_STATUS) & VIRTIO_STATUS_FEAT_OK) == 0) {
        mmio_write(VMMIO_STATUS, VIRTIO_STATUS_FAILED);
        return false;
    }

    if (drv_feat & VIRTIO_NET_F_MAC) {
        for (int i = 0; i < 6; ++i) {
            mac_[i] = static_cast<uint8_t>(
                *reinterpret_cast<volatile uint8_t*>(base_ + VMMIO_CONFIG + i));
        }
    }

    if (!setup_queue(0, rx_queue_)) return false;
    if (!setup_queue(1, tx_queue_)) return false;

    mmio_write(VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
                             VIRTIO_STATUS_FEAT_OK | VIRTIO_STATUS_DRIVER_OK);

    post_rx_buffers();
    initialized_ = true;
    return true;
}

bool VirtioNetDriver::init_interface(int if_idx) {
    (void)if_idx;
    return initialized_;
}

bool VirtioNetDriver::send_packet(int if_idx, const uint8_t* data, size_t len) {
    (void)if_idx;
    if (!initialized_ || !data) { stats_.tx_drops++; return false; }
    if (len + sizeof(VirtioNetHdr) > NET_BUF_SIZE) { stats_.tx_drops++; return false; }

    uint16_t slot = tx_queue_.avail.idx % VIRTQ_SIZE;
    uint8_t* buf = &tx_bufs_[slot * NET_BUF_SIZE];

    VirtioNetHdr* h = reinterpret_cast<VirtioNetHdr*>(buf);
    std::memset(h, 0, sizeof(*h));
    std::memcpy(buf + sizeof(VirtioNetHdr), data, len);

    tx_queue_.desc[slot].addr  = reinterpret_cast<uint64_t>(buf);
    tx_queue_.desc[slot].len   = static_cast<uint32_t>(sizeof(VirtioNetHdr) + len);
    tx_queue_.desc[slot].flags = 0;
    tx_queue_.desc[slot].next  = 0;
    tx_queue_.avail.ring[slot] = slot;
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        auto* mem = kernel::g_platform->get_mem_ops();
        mem->flush_cache_range(buf, sizeof(VirtioNetHdr) + len);
        mem->flush_cache_range(&tx_queue_.desc[slot], sizeof(tx_queue_.desc[slot]));
        mem->flush_cache_range(&tx_queue_.avail.ring[slot], sizeof(tx_queue_.avail.ring[slot]));
    }
    __atomic_store_n(&tx_queue_.avail.idx,
                     static_cast<uint16_t>(tx_queue_.avail.idx + 1),
                     __ATOMIC_RELEASE);
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&tx_queue_.avail, sizeof(tx_queue_.avail));
    }
    kernel::hal::sync::barrier_dsb();
    mmio_write(VMMIO_QUEUE_NOTIFY, 1);
    stats_.tx_packets++;
    stats_.tx_bytes += len;
    return true;
}

size_t VirtioNetDriver::poll_rx(kernel::hal::net::PacketReceivedCallback cb,
                                void* context, size_t budget) {
    if (!initialized_) return 0;
    size_t delivered = 0;
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->invalidate_cache_range(&rx_queue_.used, sizeof(rx_queue_.used));
        kernel::g_platform->get_mem_ops()->invalidate_cache_range(&tx_queue_.used, sizeof(tx_queue_.used));
    }
    uint16_t used_idx = __atomic_load_n(&rx_queue_.used.idx, __ATOMIC_ACQUIRE);
    const uint32_t int_status = mmio_read(VMMIO_INT_STATUS);
    if (int_status != 0) {
        mmio_write(VMMIO_INT_ACK, int_status);
    }
    while (rx_queue_.last_used != used_idx && delivered < budget) {
        const uint16_t ring_pos = rx_queue_.last_used % VIRTQ_SIZE;
        const VirtqUsedElem& ue = rx_queue_.used.ring[ring_pos];
        const uint16_t desc_idx = static_cast<uint16_t>(ue.id & (VIRTQ_SIZE - 1));
        const uint32_t total_len = ue.len;

        if (total_len > sizeof(VirtioNetHdr)) {
            uint8_t* buf = reinterpret_cast<uint8_t*>(rx_queue_.desc[desc_idx].addr);
            const size_t l2_len = total_len - sizeof(VirtioNetHdr);
            if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
                kernel::g_platform->get_mem_ops()->invalidate_cache_range(buf, total_len);
            }
            stats_.rx_packets++;
            stats_.rx_bytes += l2_len;
            if (cb) cb(0, buf + sizeof(VirtioNetHdr), l2_len, context);
        } else {
            stats_.rx_drops++;
        }

        rx_queue_.desc[desc_idx].len   = NET_BUF_SIZE;
        rx_queue_.desc[desc_idx].flags = VIRTQ_DESC_F_WRITE;
        rx_queue_.desc[desc_idx].next  = 0;
        const uint16_t avail_pos = rx_queue_.avail.idx % VIRTQ_SIZE;
        rx_queue_.avail.ring[avail_pos] = desc_idx;
        if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
            auto* mem = kernel::g_platform->get_mem_ops();
            mem->flush_cache_range(&rx_queue_.desc[desc_idx], sizeof(rx_queue_.desc[desc_idx]));
            mem->flush_cache_range(&rx_queue_.avail.ring[avail_pos], sizeof(rx_queue_.avail.ring[avail_pos]));
        }
        __atomic_store_n(&rx_queue_.avail.idx,
                         static_cast<uint16_t>(rx_queue_.avail.idx + 1),
                         __ATOMIC_RELEASE);
        if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
            kernel::g_platform->get_mem_ops()->flush_cache_range(&rx_queue_.avail, sizeof(rx_queue_.avail));
        }
        kernel::hal::sync::barrier_dsb();

        ++rx_queue_.last_used;
        ++delivered;
    }
    if (delivered) {
        mmio_write(VMMIO_QUEUE_NOTIFY, 0);
    }
    return delivered;
}

void VirtioNetDriver::register_packet_receiver(
        kernel::hal::net::PacketReceivedCallback cb, void* context) {
    rx_cb_ = cb;
    rx_ctx_ = context;
}

bool VirtioNetDriver::get_mac(uint8_t out[6]) const {
    if (!out || !initialized_) return false;
    for (size_t i = 0; i < 6; ++i) out[i] = mac_[i];
    return true;
}

void VirtioNetDriver::dump_status(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    uart->puts("  base=");
    uart->uart_put_uint64_hex(base_);
    uart->puts(initialized_ ? "  state=up" : "  state=down");
    uart->puts("\n  mac=");
    static constexpr char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < 6; ++i) {
        char b[3] = { hex[mac_[i] >> 4], hex[mac_[i] & 0xF], 0 };
        uart->puts(b);
        if (i < 5) uart->puts(":");
    }
    uart->puts("\n");
}

size_t discover_virtio_net(uint64_t base, size_t slot_size, size_t slot_count,
                           VirtioNetDriver* out, size_t max,
                           SlotBoundHook hook, void* hook_ctx) {
    size_t found = 0;
    for (size_t scan = slot_count; scan > 0 && found < max; --scan) {
        const size_t i = scan - 1;
        uint64_t slot_base = base + i * slot_size;
        uint32_t magic = mmio_r32(slot_base + VMMIO_MAGIC);
        if (magic != 0x74726976) continue;
        uint32_t dev_id = mmio_r32(slot_base + VMMIO_DEVICE_ID);
        if (dev_id == 0) continue; // slot present but empty
        if (dev_id != VIRTIO_DEV_ID_NET) continue;
        if (out[found].init(slot_base)) {
            if (hook) hook(i, found, out[found], hook_ctx);
            ++found;
        }
    }
    return found;
}

} // namespace hal::shared::virtio
