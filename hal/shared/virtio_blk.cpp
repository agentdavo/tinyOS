// SPDX-License-Identifier: MIT OR Apache-2.0

#include "virtio_blk.hpp"

#include "../../miniOS.hpp"
#include "../../util.hpp"

#include <cstring>

namespace hal::shared::virtio {

namespace {

// Single statically-allocated driver — QEMU virt has one block slot in
// practice, and the FS layer caches everything at boot so we never need
// concurrent readers.
VirtioBlkDriver g_driver;
bool g_driver_bound = false;

}  // namespace

uint32_t VirtioBlkDriver::mmio_read(uint32_t off) const { return virtio_mmio::read32(base_, off); }
void     VirtioBlkDriver::mmio_write(uint32_t off, uint32_t v) { virtio_mmio::write32(base_, off, v); }

bool VirtioBlkDriver::setup_queue() {
    return virtio_mmio::setup_queue(base_, 0, VIRTQ_SIZE, &q_.desc[0], &q_.avail, &q_.used);
}

bool VirtioBlkDriver::init(uint64_t slot_base) {
    base_ = slot_base;
    // Only VIRTIO_F_VERSION_1 is needed; everything else (read-only flag,
    // segment caps, ...) is ignorable for this polling driver.
    if (!virtio_mmio::begin(base_, VIRTIO_DEV_ID_BLK, 0)) return false;

    // Capacity lives at the start of the config region — 8 bytes little-endian.
    auto* cfg_cap_lo = reinterpret_cast<volatile uint32_t*>(base_ + VMMIO_CONFIG + 0);
    auto* cfg_cap_hi = reinterpret_cast<volatile uint32_t*>(base_ + VMMIO_CONFIG + 4);
    capacity_ = static_cast<uint64_t>(*cfg_cap_lo) | (static_cast<uint64_t>(*cfg_cap_hi) << 32);

    if (!setup_queue()) return false;

    virtio_mmio::driver_ok(base_);
    initialized_ = true;
    return true;
}

bool VirtioBlkDriver::read_sectors(uint64_t lba, uint32_t count, void* buf) {
    if (!initialized_ || !buf) return false;
    if (count == 0 || count > MAX_SECTORS_PER_REQ) return false;
    // One shared descriptor chain + header/status scratch: serialise so a
    // second caller can't stomp an in-flight request.
    kernel::core::ScopedLock lock(io_lock_);

    // Compose a 3-descriptor chain: [header(RO)] -> [data(WO)] -> [status(WO)].
    // A single data descriptor is enough up to 32 KB per the virtio spec,
    // which dwarfs our MAX_SECTORS_PER_REQ * 512 cap.
    req_hdr_.type = VIRTIO_BLK_T_IN;
    req_hdr_.reserved = 0;
    req_hdr_.sector = lba;
    status_byte_ = 0xFF;  // sentinel — device writes 0/1/2 when done

    q_.desc[0].addr  = reinterpret_cast<uint64_t>(&req_hdr_);
    q_.desc[0].len   = sizeof(VirtioBlkReqHeader);
    q_.desc[0].flags = VIRTQ_DESC_F_NEXT;
    q_.desc[0].next  = 1;

    q_.desc[1].addr  = reinterpret_cast<uint64_t>(buf);
    q_.desc[1].len   = count * SECTOR_SIZE;
    q_.desc[1].flags = VIRTQ_DESC_F_NEXT | VIRTQ_DESC_F_WRITE;
    q_.desc[1].next  = 2;

    q_.desc[2].addr  = reinterpret_cast<uint64_t>(&status_byte_);
    q_.desc[2].len   = 1;
    q_.desc[2].flags = VIRTQ_DESC_F_WRITE;
    q_.desc[2].next  = 0;

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        auto* mem = kernel::g_platform->get_mem_ops();
        mem->flush_cache_range(&q_.desc[0], sizeof(VirtqDesc) * 3);
        mem->flush_cache_range(&req_hdr_, sizeof(req_hdr_));
    }

    const uint16_t avail_slot = q_.avail.idx % VIRTQ_SIZE;
    q_.avail.ring[avail_slot] = 0;  // head of the chain is descriptor 0

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.avail, sizeof(q_.avail));
    }
    __atomic_store_n(&q_.avail.idx, static_cast<uint16_t>(q_.avail.idx + 1),
                     __ATOMIC_RELEASE);
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.avail, sizeof(q_.avail));
    }
    kernel::hal::sync::barrier_dsb();
    mmio_write(VMMIO_QUEUE_NOTIFY, 0);

    // Spin on used.idx. Bounded by a large but finite loop so a wedged
    // device doesn't hang boot forever — returns false after ~seconds.
    const uint16_t want = static_cast<uint16_t>(q_.last_used + 1);
    const uint32_t spin_limit = 200'000'000;
    for (uint32_t i = 0; i < spin_limit; ++i) {
        if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
            kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.used, sizeof(q_.used));
        }
        uint16_t cur = __atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE);
        if (cur == want) break;
        kernel::util::cpu_relax();
    }
    if (__atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE) != want) {
        // Device timed out / didn't complete. Resync last_used to whatever the
        // device's used.idx is now, otherwise the next request computes
        // want = last_used+1 against an already-advanced ring and wedges all
        // future I/O on this queue.
        q_.last_used = __atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE);
        return false;
    }
    q_.last_used = want;

    // Drain INT status so edge-triggered GICs don't keep replaying the IRQ.
    // (We don't use IRQs here, but kernel shares the line.)
    const uint32_t isr = mmio_read(VMMIO_INT_STATUS);
    if (isr) mmio_write(VMMIO_INT_ACK, isr);

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&status_byte_, 1);
        kernel::g_platform->get_mem_ops()->flush_cache_range(buf, count * SECTOR_SIZE);
    }
    return status_byte_ == VIRTIO_BLK_S_OK;
}

bool VirtioBlkDriver::write_sectors(uint64_t lba, uint32_t count, const void* buf) {
    if (!initialized_ || !buf) return false;
    if (count == 0 || count > MAX_SECTORS_PER_REQ) return false;
    kernel::core::ScopedLock lock(io_lock_);  // serialise the shared descriptor chain

    // Mirror of read_sectors, but VIRTIO_BLK_T_OUT and the data descriptor
    // is device-read (no VIRTQ_DESC_F_WRITE), so the device pulls payload
    // from `buf` rather than writing into it.
    req_hdr_.type = VIRTIO_BLK_T_OUT;
    req_hdr_.reserved = 0;
    req_hdr_.sector = lba;
    status_byte_ = 0xFF;

    q_.desc[0].addr  = reinterpret_cast<uint64_t>(&req_hdr_);
    q_.desc[0].len   = sizeof(VirtioBlkReqHeader);
    q_.desc[0].flags = VIRTQ_DESC_F_NEXT;
    q_.desc[0].next  = 1;

    q_.desc[1].addr  = reinterpret_cast<uint64_t>(buf);
    q_.desc[1].len   = count * SECTOR_SIZE;
    q_.desc[1].flags = VIRTQ_DESC_F_NEXT;  // device-read; no F_WRITE
    q_.desc[1].next  = 2;

    q_.desc[2].addr  = reinterpret_cast<uint64_t>(&status_byte_);
    q_.desc[2].len   = 1;
    q_.desc[2].flags = VIRTQ_DESC_F_WRITE;
    q_.desc[2].next  = 0;

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        auto* mem = kernel::g_platform->get_mem_ops();
        mem->flush_cache_range(&q_.desc[0], sizeof(VirtqDesc) * 3);
        mem->flush_cache_range(&req_hdr_, sizeof(req_hdr_));
        // Flush the source payload so the device sees the written bytes,
        // not stale cached lines from before the caller filled `buf`.
        mem->flush_cache_range(const_cast<void*>(buf), count * SECTOR_SIZE);
    }

    const uint16_t avail_slot = q_.avail.idx % VIRTQ_SIZE;
    q_.avail.ring[avail_slot] = 0;

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.avail, sizeof(q_.avail));
    }
    __atomic_store_n(&q_.avail.idx, static_cast<uint16_t>(q_.avail.idx + 1),
                     __ATOMIC_RELEASE);
    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.avail, sizeof(q_.avail));
    }
    kernel::hal::sync::barrier_dsb();
    mmio_write(VMMIO_QUEUE_NOTIFY, 0);

    const uint16_t want = static_cast<uint16_t>(q_.last_used + 1);
    const uint32_t spin_limit = 200'000'000;
    for (uint32_t i = 0; i < spin_limit; ++i) {
        if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
            kernel::g_platform->get_mem_ops()->flush_cache_range(&q_.used, sizeof(q_.used));
        }
        uint16_t cur = __atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE);
        if (cur == want) break;
        kernel::util::cpu_relax();
    }
    if (__atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE) != want) {
        // Device timed out / didn't complete. Resync last_used to whatever the
        // device's used.idx is now, otherwise the next request computes
        // want = last_used+1 against an already-advanced ring and wedges all
        // future I/O on this queue.
        q_.last_used = __atomic_load_n(&q_.used.idx, __ATOMIC_ACQUIRE);
        return false;
    }
    q_.last_used = want;

    const uint32_t isr = mmio_read(VMMIO_INT_STATUS);
    if (isr) mmio_write(VMMIO_INT_ACK, isr);

    if (kernel::g_platform && kernel::g_platform->get_mem_ops()) {
        kernel::g_platform->get_mem_ops()->flush_cache_range(&status_byte_, 1);
    }
    return status_byte_ == VIRTIO_BLK_S_OK;
}

VirtioBlkDriver* discover_virtio_blk(uint64_t base, size_t slot_size, size_t slot_count) {
    if (g_driver_bound) return &g_driver;
    for (size_t i = 0; i < slot_count; ++i) {
        const uint64_t slot_base = base + i * slot_size;
        if (virtio_mmio::read32(slot_base, VMMIO_MAGIC) != VIRTIO_MMIO_MAGIC_VALUE) continue;
        if (virtio_mmio::read32(slot_base, VMMIO_VERSION) != 2) continue;
        if (virtio_mmio::read32(slot_base, VMMIO_DEVICE_ID) != VIRTIO_DEV_ID_BLK) continue;
        if (g_driver.init(slot_base)) {
            g_driver_bound = true;
            return &g_driver;
        }
    }
    return nullptr;
}

}  // namespace hal::shared::virtio
