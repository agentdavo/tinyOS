// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// CPU-copy "DMA" engine shared by both QEMU ports (neither virt machine
// exposes a general-purpose DMA controller). Transfers complete
// synchronously inside configure_and_start_transfer.

#include "hal.hpp"
#include "core.hpp"

#include <array>

namespace hal::shared {

class SoftwareDMAController : public kernel::hal::DMAControllerOps {
public:
    explicit constexpr SoftwareDMAController(const char* driver_name) noexcept
        : driver_name_(driver_name) {}
    kernel::hal::dma::ChannelID request_channel() override;
    void release_channel(kernel::hal::dma::ChannelID channel) override;
    kernel::hal::dma::Capabilities get_capabilities() const override;
    bool configure_and_start_transfer(kernel::hal::dma::ChannelID channel,
                                      const kernel::hal::dma::TransferConfig& cfg,
                                      kernel::hal::dma::DMACallback cb,
                                      void* context) override;
private:
    bool channel_valid_and_claimed(kernel::hal::dma::ChannelID channel) noexcept;

    const char* driver_name_;
    std::array<bool, 8> channels_in_use_{};
    kernel::core::Spinlock lock_;
};

} // namespace hal::shared
