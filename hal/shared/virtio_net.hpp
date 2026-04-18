// SPDX-License-Identifier: MIT OR Apache-2.0
// Arch-generic virtio-mmio / virtio-net driver.
//
// Was originally hal/arm64/virtio_net.{hpp,cpp}; the only arch-specific bits
// were (a) the MMIO base address of the virtio-mmio bus and (b) how the IRQ
// line for each bound slot is routed to a CPU (GIC SPI affinity on arm64,
// PLIC enable-on-all-contexts on rv64). Both are now passed in by the caller,
// so the core driver is portable across QEMU virt targets (arm64, rv64, ...).
//
// QEMU virt MMIO bus layout:
//   ARM64:  base = 0x0a000000, 32 slots of 0x200 bytes (slot 0 .. 31)
//   RV64:   base = 0x10001000, 8  slots of 0x1000 bytes (slot 0 .. 7)
// The register map inside each slot is identical (transport v2).

#ifndef HAL_SHARED_VIRTIO_NET_HPP
#define HAL_SHARED_VIRTIO_NET_HPP

#include "hal.hpp"
#include <array>
#include <atomic>
#include <cstdint>

namespace hal::shared::virtio {

// Virtio device IDs we care about.
constexpr uint32_t VIRTIO_DEV_ID_NET = 1;

// Virtio-mmio register offsets (transport v2).
constexpr uint32_t VMMIO_MAGIC        = 0x000; // "virt" = 0x74726976
constexpr uint32_t VMMIO_VERSION      = 0x004;
constexpr uint32_t VMMIO_DEVICE_ID    = 0x008;
constexpr uint32_t VMMIO_DEV_FEAT     = 0x010;
constexpr uint32_t VMMIO_DEV_FEAT_SEL = 0x014;
constexpr uint32_t VMMIO_DRV_FEAT     = 0x020;
constexpr uint32_t VMMIO_DRV_FEAT_SEL = 0x024;
constexpr uint32_t VMMIO_QUEUE_SEL    = 0x030;
constexpr uint32_t VMMIO_QUEUE_NUM_MAX = 0x034;
constexpr uint32_t VMMIO_QUEUE_NUM    = 0x038;
constexpr uint32_t VMMIO_QUEUE_READY  = 0x044;
constexpr uint32_t VMMIO_QUEUE_NOTIFY = 0x050;
constexpr uint32_t VMMIO_INT_STATUS   = 0x060;
constexpr uint32_t VMMIO_INT_ACK      = 0x064;
constexpr uint32_t VMMIO_STATUS       = 0x070;
constexpr uint32_t VMMIO_QUEUE_DESC_LO    = 0x080;
constexpr uint32_t VMMIO_QUEUE_DESC_HI    = 0x084;
constexpr uint32_t VMMIO_QUEUE_DRIVER_LO  = 0x090;
constexpr uint32_t VMMIO_QUEUE_DRIVER_HI  = 0x094;
constexpr uint32_t VMMIO_QUEUE_DEVICE_LO  = 0x0a0;
constexpr uint32_t VMMIO_QUEUE_DEVICE_HI  = 0x0a4;
constexpr uint32_t VMMIO_CONFIG       = 0x100;

// Virtio status bits.
constexpr uint32_t VIRTIO_STATUS_ACK        = 1 << 0;
constexpr uint32_t VIRTIO_STATUS_DRIVER     = 1 << 1;
constexpr uint32_t VIRTIO_STATUS_DRIVER_OK  = 1 << 2;
constexpr uint32_t VIRTIO_STATUS_FEAT_OK    = 1 << 3;
constexpr uint32_t VIRTIO_STATUS_FAILED     = 1 << 7;

// Virtqueue descriptor flags.
constexpr uint16_t VIRTQ_DESC_F_NEXT  = 1;
constexpr uint16_t VIRTQ_DESC_F_WRITE = 2;

constexpr size_t VIRTQ_SIZE = 16;     // Descriptors per queue (power of two).
constexpr size_t NET_BUF_SIZE = 2048; // Max bytes per packet + virtio_net_hdr.

struct VirtqDesc {
    uint64_t addr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
} __attribute__((packed));

struct VirtqAvail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[VIRTQ_SIZE];
    uint16_t used_event;
} __attribute__((packed));

struct VirtqUsedElem {
    uint32_t id;
    uint32_t len;
} __attribute__((packed));

struct VirtqUsed {
    uint16_t flags;
    uint16_t idx;
    VirtqUsedElem ring[VIRTQ_SIZE];
    uint16_t avail_event;
} __attribute__((packed));

// Virtio-net packet header without VIRTIO_NET_F_MRG_RXBUF. Because this driver
// never negotiates mergeable RX buffers, the device-visible header is 10 bytes.
struct VirtioNetHdr {
    uint8_t  flags;
    uint8_t  gso_type;
    uint16_t hdr_len;
    uint16_t gso_size;
    uint16_t csum_start;
    uint16_t csum_offset;
} __attribute__((packed));

class VirtioNetDriver : public kernel::hal::net::NetworkDriverOps {
public:
    // Bind to the virtio-net at `slot_base`. Returns false if this slot has no
    // virtio-net or init fails.
    bool init(uint64_t slot_base);

    bool init_interface(int if_idx) override;
    bool send_packet(int if_idx, const uint8_t* data, size_t len) override;
    void register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb, void* context) override;
    kernel::hal::net::NicStats get_stats() const override { return stats_; }
    bool get_mac(uint8_t out[6]) const override;

    // Poll-mode RX. Walks the used ring up to `budget` entries, invoking `cb`
    // with the L2 frame (virtio-net header stripped) and re-posting the
    // descriptor to the avail ring. Returns the number of frames delivered.
    size_t poll_rx(kernel::hal::net::PacketReceivedCallback cb, void* context,
                   size_t budget = 8) override;

    // Diagnostics for the `net` CLI command.
    void dump_status(kernel::hal::UARTDriverOps* uart) const;
    const uint8_t* mac() const { return mac_; }

    bool initialized() const { return initialized_; }
    uint64_t base() const { return base_; }

private:
    struct Queue {
        alignas(16) VirtqDesc  desc[VIRTQ_SIZE];
        alignas(2)  VirtqAvail avail;
        alignas(4)  VirtqUsed  used;
        uint16_t    last_used = 0;
    };
    alignas(4096) Queue rx_queue_{};
    alignas(4096) Queue tx_queue_{};
    alignas(64) std::array<uint8_t, VIRTQ_SIZE * NET_BUF_SIZE> rx_bufs_{};
    alignas(64) std::array<uint8_t, VIRTQ_SIZE * NET_BUF_SIZE> tx_bufs_{};

    uint64_t base_ = 0;
    bool     initialized_ = false;
    uint8_t  mac_[6] = {};
    kernel::hal::net::PacketReceivedCallback rx_cb_ = nullptr;
    void* rx_ctx_ = nullptr;
    kernel::hal::net::NicStats stats_{};

    uint32_t mmio_read(uint32_t off) const;
    void     mmio_write(uint32_t off, uint32_t val);
    bool     setup_queue(uint32_t queue_idx, Queue& q);
    void     post_rx_buffers();
};

// Per-slot hook invoked after a virtio-net is bound. `slot_idx` is the index
// within the bus (0..slot_count-1); `nic_idx` is the index within `out[]`
// (counted only across successfully bound NICs). Caller uses this to program
// arch-specific IRQ routing. May be null.
using SlotBoundHook = void (*)(size_t slot_idx, size_t nic_idx, VirtioNetDriver& drv,
                               void* ctx);

// Scan [base, base + slot_count * slot_size) for virtio-nets and bind up to
// `max` of them into `out`. Returns the number bound. `slot_size` on QEMU is
// 0x200 for arm64 virt and 0x1000 for rv64 virt.
size_t discover_virtio_net(uint64_t base, size_t slot_size, size_t slot_count,
                           VirtioNetDriver* out, size_t max,
                           SlotBoundHook hook = nullptr, void* hook_ctx = nullptr);

} // namespace hal::shared::virtio

#endif // HAL_SHARED_VIRTIO_NET_HPP
