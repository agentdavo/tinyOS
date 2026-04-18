// SPDX-License-Identifier: MIT OR Apache-2.0
// SD/MMC card driver for QEMU virt machine.
// Uses SPI mode for simplicity - works with QEMU's sd-card device.

#ifndef HAL_SHARED_SDCARD_HPP
#define HAL_SHARED_SDCARD_HPP

#include <cstdint>
#include <cstddef>

namespace hal {
namespace shared {
namespace sdcard {

constexpr uint32_t BLOCK_SIZE = 512;

enum class CardState : uint8_t {
    Idle,
    Ready,
    Ident,
    Standby,
    Transfer,
    Receiving,
    Programming,
    Disconnect,
    Error
};

struct CardInfo {
    uint32_t blocks = 0;
    uint32_t block_size = BLOCK_SIZE;
    uint8_t cid[16] = {};
    uint8_t csd[16] = {};
    CardState state = CardState::Idle;
    bool initialized = false;
};

class SDCardDriver {
public:
    bool init(uintptr_t spi_base, uint32_t cs_pin);
    bool read_block(uint32_t block_num, void* buffer);
    bool write_block(uint32_t block_num, const void* buffer);
    bool get_card_info(CardInfo& info);
    bool is_present();

private:
    bool send_command(uint8_t cmd, uint32_t arg, uint8_t* response, size_t resp_len);
    bool wait_ready(uint32_t timeout_ms);
    uint8_t transfer_spi(uint8_t data);

    uintptr_t spi_base_ = 0;
    uint32_t cs_pin_ = 0;
    CardInfo card_info_;
    bool initialized_ = false;
};

bool init_sdcard(uintptr_t spi_base, uint32_t cs_pin);

} // namespace sdcard
} // namespace shared
} // namespace hal

#endif
