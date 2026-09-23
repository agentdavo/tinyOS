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

uint32_t VirtioNetDriver::mmio_read(uint32_t off) const { return virtio_mmio::read32(base_, off); }
void     VirtioNetDriver::mmio_write(uint32_t off, uint32_t v) { virtio_mmio::write32(base_, off, v); }

bool VirtioNetDriver::setup_queue(uint32_t queue_idx, Queue& q) {
    return virtio_mmio::setup_queue(base_, queue_idx, VIRTQ_SIZE, &q.desc[0], &q.avail, &q.used);
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

    constexpr uint64_t VIRTIO_NET_F_MAC = 1ULL << 5;
    uint64_t drv_feat = 0;
    if (!virtio_mmio::begin(base_, VIRTIO_DEV_ID_NET, VIRTIO_NET_F_MAC, &drv_feat)) return false;

    if (drv_feat & VIRTIO_NET_F_MAC) {
        for (int i = 0; i < 6; ++i) {
            mac_[i] = static_cast<uint8_t>(
                *reinterpret_cast<volatile uint8_t*>(base_ + VMMIO_CONFIG + i));
        }
    }

    if (!setup_queue(0, rx_queue_)) return false;
    if (!setup_queue(1, tx_queue_)) return false;

    virtio_mmio::driver_ok(base_);

    post_rx_buffers();
    initialized_ = true;
    return true;
}

bool VirtioNetDriver::init_interface(int if_idx) {
    (void)if_idx;
    return initialized_;
}

void VirtioNetDriver::acknowledge_irq() noexcept {
    // Called from IRQ context. virtio-mmio levels the IRQ line until we
    // clear the matching bit in VMMIO_INT_STATUS via VMMIO_INT_ACK; if we
    // don't, the GIC/PLIC keeps re-firing forever. The slow-path poll
    // also does this, but doing it here is what lets the IRQ line
    // de-assert promptly and avoids interrupt storms on busy NICs.
    if (!initialized_) return;
    const uint32_t status = mmio_read(VMMIO_INT_STATUS);
    if (status != 0) mmio_write(VMMIO_INT_ACK, status);
}

bool VirtioNetDriver::send_packet(int if_idx, const uint8_t* data, size_t len) {
    (void)if_idx;
    if (!initialized_ || !data) { stats_.tx_drops++; return false; }
    if (len + sizeof(VirtioNetHdr) > NET_BUF_SIZE) { stats_.tx_drops++; return false; }

    // Reap completed TX descriptors before reusing a slot. Without this the
    // used ring was never advanced and `avail.idx % VIRTQ_SIZE` would reuse a
    // slot's buffer/descriptor after VIRTQ_SIZE sends while the device might
    // still be DMAing it. Advancing last_used to the device's used.idx frees
    // the slots it has finished with.
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->invalidate_cache_range(&tx_queue_.used, sizeof(tx_queue_.used));
    }
    tx_queue_.last_used = __atomic_load_n(&tx_queue_.used.idx, __ATOMIC_ACQUIRE);

    // If every descriptor is still in flight (device hasn't drained the ring),
    // drop rather than clobber an unconsumed slot.
    const uint16_t outstanding =
        static_cast<uint16_t>(tx_queue_.avail.idx - tx_queue_.last_used);
    if (outstanding >= VIRTQ_SIZE) { stats_.tx_drops++; return false; }

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

        // ue.len is device-supplied: bound it by the posted buffer size, or a
        // bogus length would invalidate and hand the callback memory past
        // this RX buffer.
        if (total_len > sizeof(VirtioNetHdr) && total_len <= NET_BUF_SIZE) {
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
        uint32_t magic = virtio_mmio::read32(slot_base, VMMIO_MAGIC);
        if (magic != VIRTIO_MMIO_MAGIC_VALUE) continue;
        uint32_t dev_id = virtio_mmio::read32(slot_base, VMMIO_DEVICE_ID);
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
