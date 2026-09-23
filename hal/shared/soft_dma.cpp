// SPDX-License-Identifier: MIT OR Apache-2.0

#include "soft_dma.hpp"

#include <cstring>

namespace hal::shared {

using kernel::hal::dma::ChannelID;

kernel::hal::dma::Capabilities SoftwareDMAController::get_capabilities() const {
    kernel::hal::dma::Capabilities caps;
    caps.engine_kind = kernel::hal::dma::EngineKind::Software;
    caps.available = true;
    caps.mem_to_mem = true;
    caps.mem_to_periph = true;
    caps.periph_to_mem = true;
    caps.async_completion = false;
    caps.scatter_gather = false;
    caps.cache_coherent = false;
    caps.driver_name = driver_name_;
    return caps;
}

// Claim/release under a lock: the rv64 copy of this used an unlocked
// check-then-set on a single flag, which raced across harts.
ChannelID SoftwareDMAController::request_channel() {
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < channels_in_use_.size(); ++i) {
        if (!channels_in_use_[i]) {
            channels_in_use_[i] = true;
            return static_cast<ChannelID>(i);
        }
    }
    return kernel::hal::dma::INVALID_CHANNEL;
}

void SoftwareDMAController::release_channel(ChannelID ch) {
    kernel::core::ScopedLock lock(lock_);
    if (ch >= 0 && static_cast<size_t>(ch) < channels_in_use_.size()) {
        channels_in_use_[static_cast<size_t>(ch)] = false;
    }
}

bool SoftwareDMAController::channel_valid_and_claimed(ChannelID ch) noexcept {
    kernel::core::ScopedLock lock(lock_);
    return ch >= 0 && static_cast<size_t>(ch) < channels_in_use_.size() &&
           channels_in_use_[static_cast<size_t>(ch)];
}

bool SoftwareDMAController::configure_and_start_transfer(ChannelID ch,
                                                         const kernel::hal::dma::TransferConfig& cfg,
                                                         kernel::hal::dma::DMACallback cb,
                                                         void* ctx) {
    if (!channel_valid_and_claimed(ch)) return false;
    if (cfg.direction == kernel::hal::dma::Direction::MEM_TO_MEM && cfg.size_bytes != 0) {
        auto* dst = reinterpret_cast<uint8_t*>(cfg.dst_addr);
        auto* src = reinterpret_cast<const uint8_t*>(cfg.src_addr);
        if (!dst || !src) {
            release_channel(ch);
            return false;
        }
        if (cfg.src_increment && cfg.dst_increment) {
            std::memmove(dst, src, cfg.size_bytes);  // overlap-safe, word-wise
        } else {
            // Fixed source and/or destination (FIFO-style) addressing.
            for (size_t i = 0; i < cfg.size_bytes; ++i) {
                dst[cfg.dst_increment ? i : 0] = src[cfg.src_increment ? i : 0];
            }
        }
    }
    if (cb) cb(ch, true, ctx);
    release_channel(ch);
    return true;
}

} // namespace hal::shared
