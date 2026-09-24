// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// virtio-mmio transport (v2) shared by every virtio driver under hal/shared/
// (net, blk, gpu, input). Each driver used to carry its own copy of the
// register map, status bits, the reset/feature-negotiation sequence and the
// queue-address programming; they now call these helpers. Ring layouts stay
// in the drivers because their queue sizes differ.

#include "hal.hpp"

#include <cstddef>
#include <cstdint>

namespace hal::shared::virtio_mmio {

// Register offsets inside a virtio-mmio slot (transport v2).
constexpr uint32_t VMMIO_MAGIC           = 0x000;  // "virt" = 0x74726976
constexpr uint32_t VMMIO_VERSION         = 0x004;
constexpr uint32_t VMMIO_DEVICE_ID       = 0x008;
constexpr uint32_t VMMIO_DEV_FEAT        = 0x010;
constexpr uint32_t VMMIO_DEV_FEAT_SEL    = 0x014;
constexpr uint32_t VMMIO_DRV_FEAT        = 0x020;
constexpr uint32_t VMMIO_DRV_FEAT_SEL    = 0x024;
constexpr uint32_t VMMIO_QUEUE_SEL       = 0x030;
constexpr uint32_t VMMIO_QUEUE_NUM_MAX   = 0x034;
constexpr uint32_t VMMIO_QUEUE_NUM       = 0x038;
constexpr uint32_t VMMIO_QUEUE_READY     = 0x044;
constexpr uint32_t VMMIO_QUEUE_NOTIFY    = 0x050;
constexpr uint32_t VMMIO_INT_STATUS      = 0x060;
constexpr uint32_t VMMIO_INT_ACK         = 0x064;
constexpr uint32_t VMMIO_STATUS          = 0x070;
constexpr uint32_t VMMIO_QUEUE_DESC_LO   = 0x080;
constexpr uint32_t VMMIO_QUEUE_DESC_HI   = 0x084;
constexpr uint32_t VMMIO_QUEUE_DRIVER_LO = 0x090;
constexpr uint32_t VMMIO_QUEUE_DRIVER_HI = 0x094;
constexpr uint32_t VMMIO_QUEUE_DEVICE_LO = 0x0a0;
constexpr uint32_t VMMIO_QUEUE_DEVICE_HI = 0x0a4;
constexpr uint32_t VMMIO_CONFIG          = 0x100;

constexpr uint32_t VIRTIO_MMIO_MAGIC_VALUE = 0x74726976;

// Device status bits.
constexpr uint32_t VIRTIO_STATUS_ACK       = 1u << 0;
constexpr uint32_t VIRTIO_STATUS_DRIVER    = 1u << 1;
constexpr uint32_t VIRTIO_STATUS_DRIVER_OK = 1u << 2;
constexpr uint32_t VIRTIO_STATUS_FEAT_OK   = 1u << 3;
constexpr uint32_t VIRTIO_STATUS_FAILED    = 1u << 7;

// Virtqueue descriptor flags.
constexpr uint16_t VIRTQ_DESC_F_NEXT  = 1u;
constexpr uint16_t VIRTQ_DESC_F_WRITE = 2u;

constexpr uint64_t VIRTIO_F_VERSION_1 = 1ULL << 32;

inline uint32_t read32(uint64_t base, uint32_t off) noexcept {
    return *reinterpret_cast<volatile uint32_t*>(base + off);
}
inline void write32(uint64_t base, uint32_t off, uint32_t v) noexcept {
    *reinterpret_cast<volatile uint32_t*>(base + off) = v;
}
// Byte-wide access for device config fields (e.g. virtio-input select).
inline uint8_t read8(uint64_t base, uint32_t off) noexcept {
    return *reinterpret_cast<volatile uint8_t*>(base + off);
}
inline void write8(uint64_t base, uint32_t off, uint8_t v) noexcept {
    *reinterpret_cast<volatile uint8_t*>(base + off) = v;
}

inline void set_failed(uint64_t base) noexcept {
    write32(base, VMMIO_STATUS, VIRTIO_STATUS_FAILED);
}

// Why begin() rejected a slot, for drivers that log it.
enum class InitError { None, BadMagic, BadVersion, WrongDevice, NoVersion1, FeaturesRejected };

// Probe a slot for a transport-v2 device of `device_id`, reset it, walk
// ACK -> DRIVER, and negotiate features: VIRTIO_F_VERSION_1 is required,
// plus whatever of `wanted` the device offers. On success the device is in
// FEATURES_OK and *accepted holds the negotiated set; queue setup and
// driver_ok() follow. On a negotiation failure the device is left FAILED.
inline bool begin(uint64_t base, uint32_t device_id, uint64_t wanted,
                  uint64_t* accepted = nullptr, InitError* err = nullptr) noexcept {
    auto fail = [&](InitError e) { if (err) *err = e; return false; };
    if (read32(base, VMMIO_MAGIC) != VIRTIO_MMIO_MAGIC_VALUE) return fail(InitError::BadMagic);
    if (read32(base, VMMIO_VERSION) != 2) return fail(InitError::BadVersion);
    if (read32(base, VMMIO_DEVICE_ID) != device_id) return fail(InitError::WrongDevice);

    write32(base, VMMIO_STATUS, 0);
    write32(base, VMMIO_STATUS, VIRTIO_STATUS_ACK);
    write32(base, VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER);

    write32(base, VMMIO_DEV_FEAT_SEL, 0);
    uint64_t dev = read32(base, VMMIO_DEV_FEAT);
    write32(base, VMMIO_DEV_FEAT_SEL, 1);
    dev |= static_cast<uint64_t>(read32(base, VMMIO_DEV_FEAT)) << 32;
    if ((dev & VIRTIO_F_VERSION_1) == 0) {
        set_failed(base);
        return fail(InitError::NoVersion1);
    }
    const uint64_t drv = (dev & wanted) | VIRTIO_F_VERSION_1;
    write32(base, VMMIO_DRV_FEAT_SEL, 0);
    write32(base, VMMIO_DRV_FEAT, static_cast<uint32_t>(drv));
    write32(base, VMMIO_DRV_FEAT_SEL, 1);
    write32(base, VMMIO_DRV_FEAT, static_cast<uint32_t>(drv >> 32));

    write32(base, VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEAT_OK);
    if ((read32(base, VMMIO_STATUS) & VIRTIO_STATUS_FEAT_OK) == 0) {
        set_failed(base);
        return fail(InitError::FeaturesRejected);
    }
    if (accepted) *accepted = drv;
    if (err) *err = InitError::None;
    return true;
}

// Program queue `idx` with `size` entries and its three ring addresses,
// then mark it ready. Fails if the device can't provide `size` entries.
inline bool setup_queue(uint64_t base, uint32_t idx, uint32_t size,
                        const void* desc, const void* avail, const void* used) noexcept {
    write32(base, VMMIO_QUEUE_SEL, idx);
    const uint32_t max = read32(base, VMMIO_QUEUE_NUM_MAX);
    if (max == 0 || max < size) return false;
    write32(base, VMMIO_QUEUE_NUM, size);
    const uint64_t d = reinterpret_cast<uint64_t>(desc);
    const uint64_t a = reinterpret_cast<uint64_t>(avail);
    const uint64_t u = reinterpret_cast<uint64_t>(used);
    write32(base, VMMIO_QUEUE_DESC_LO,   static_cast<uint32_t>(d));
    write32(base, VMMIO_QUEUE_DESC_HI,   static_cast<uint32_t>(d >> 32));
    write32(base, VMMIO_QUEUE_DRIVER_LO, static_cast<uint32_t>(a));
    write32(base, VMMIO_QUEUE_DRIVER_HI, static_cast<uint32_t>(a >> 32));
    write32(base, VMMIO_QUEUE_DEVICE_LO, static_cast<uint32_t>(u));
    write32(base, VMMIO_QUEUE_DEVICE_HI, static_cast<uint32_t>(u >> 32));
    write32(base, VMMIO_QUEUE_READY, 1);
    return true;
}

inline void driver_ok(uint64_t base) noexcept {
    write32(base, VMMIO_STATUS, VIRTIO_STATUS_ACK | VIRTIO_STATUS_DRIVER |
                                VIRTIO_STATUS_FEAT_OK | VIRTIO_STATUS_DRIVER_OK);
}

// Kick queue `idx`. The barrier orders the preceding ring/idx writes before
// the doorbell store (on rv64 this is `fence iorw, iorw`: a memory-only
// fence would not order them against the MMIO write).
inline void notify(uint64_t base, uint32_t idx) noexcept {
    kernel::hal::sync::barrier_dsb();
    write32(base, VMMIO_QUEUE_NOTIFY, idx);
}

// Read and acknowledge pending interrupt causes; returns them (0 = none).
inline uint32_t ack_interrupt(uint64_t base) noexcept {
    const uint32_t s = read32(base, VMMIO_INT_STATUS);
    if (s) write32(base, VMMIO_INT_ACK, s);
    return s;
}

} // namespace hal::shared::virtio_mmio
