// SPDX-License-Identifier: MIT OR Apache-2.0
// SD/MMC card driver for QEMU virt machine using SPI mode.
// Supports reading/writing 512-byte blocks.

#include "hal/shared/sdcard.hpp"
#include <cstring>

namespace hal {
namespace shared {
namespace sdcard {

namespace {

constexpr uint32_t SD_CMD_GO_IDLE_STATE = 0;
constexpr uint32_t SD_CMD_SEND_OP_COND = 1;
constexpr uint32_t SD_CMD_READ_CSD = 9;
constexpr uint32_t SD_CMD_SEND_CID = 10;
constexpr uint32_t SD_CMD_READ_SINGLE_BLOCK = 17;
constexpr uint32_t SD_CMD_WRITE_SINGLE_BLOCK = 24;
constexpr uint32_t SD_CMD_APP_CMD = 55;
constexpr uint32_t SD_CMD_READ_OCR = 58;

constexpr uint8_t SD_TOKEN_START_BLOCK = 0xFE;
constexpr uint8_t SD_TOKEN_WRITE_START = 0xFC;
constexpr uint8_t SD_TOKEN_WRITE_STOP = 0xFD;

constexpr uint32_t SPI_FREQ_SLOW = 400000;
constexpr uint32_t SPI_FREQ_FAST = 25000000;

alignas(64) SDCardDriver g_sdcard;

uint32_t spi_read_reg(uintptr_t base, uint32_t reg) {
    volatile uint32_t* regs = reinterpret_cast<volatile uint32_t*>(base);
    return regs[reg / 4];
}

void spi_write_reg(uintptr_t base, uint32_t reg, uint32_t val) {
    volatile uint32_t* regs = reinterpret_cast<volatile uint32_t*>(base);
    regs[reg / 4] = val;
}

uint8_t spi_transfer_byte(uintptr_t base, uint8_t tx) {
    volatile uint32_t* regs = reinterpret_cast<volatile uint32_t*>(base);
    while ((regs[0x14 / 4] & 0x01) == 0) {}
    regs[0x08 / 4] = tx;
    while ((regs[0x14 / 4] & 0x01) == 0) {}
    return regs[0x0C / 4] & 0xFF;
}

void spi_set_cs(uintptr_t base, bool cs) {
    volatile uint32_t* regs = reinterpret_cast<volatile uint32_t*>(base);
    uint32_t ctrl = regs[0x00 / 4];
    if (cs) {
        ctrl |= 0x02;
    } else {
        ctrl &= ~0x02;
    }
    regs[0x00 / 4] = ctrl;
}

void spi_set_frequency(uintptr_t base, uint32_t freq) {
    (void)base; (void)freq;
}

} // namespace

bool SDCardDriver::init(uintptr_t spi_base, uint32_t cs_pin) {
    spi_base_ = spi_base;
    cs_pin_ = cs_pin;
    initialized_ = false;
    
    if (spi_base_ == 0) {
        return false;
    }
    
    return true;
}

bool SDCardDriver::send_command(uint8_t cmd, uint32_t arg, uint8_t* response, size_t resp_len) {
    if (!initialized_) return false;
    
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, 0xFF);
    spi_set_cs(spi_base_, false);
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, 0xFF);
    spi_set_cs(spi_base_, true);
    
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, 0x40 | cmd);
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, (arg >> 24) & 0xFF);
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, (arg >> 16) & 0xFF);
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, (arg >> 8) & 0xFF);
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, arg & 0xFF);
    
    uint8_t crc = 0xFF;
    if (cmd == 0) crc = 0x95;
    if (cmd == 8) crc = 0x87;
    ::hal::shared::sdcard::spi_transfer_byte(spi_base_, crc);
    
    for (size_t i = 0; i < resp_len + 1; ++i) {
        uint8_t r = ::hal::shared::sdcard::spi_transfer_byte(spi_base_, 0xFF);
        if (i > 0 && response) {
            response[i - 1] = r;
        }
    }
    
    return true;
}

bool SDCardDriver::wait_ready(uint32_t timeout_ms) {
    (void)timeout_ms;
    if (!initialized_) return false;
    
    for (int i = 0; i < 1000; ++i) {
        if (spi_transfer_byte(spi_base_, 0xFF) == 0xFF) {
            return true;
        }
    }
    return false;
}

uint8_t SDCardDriver::transfer_spi(uint8_t data) {
    if (!initialized_) return 0xFF;
    return spi_transfer_byte(spi_base_, data);
}

bool SDCardDriver::read_block(uint32_t block_num, void* buffer) {
    if (!initialized_ || !buffer) return false;
    
    uint8_t cmd[5];
    cmd[0] = 0x40 | 17;
    cmd[1] = (block_num >> 24) & 0xFF;
    cmd[2] = (block_num >> 16) & 0xFF;
    cmd[3] = (block_num >> 8) & 0xFF;
    cmd[4] = block_num & 0xFF;
    
    spi_set_cs(spi_base_, true);
    for (int i = 0; i < 5; ++i) {
        spi_transfer_byte(spi_base_, cmd[i]);
    }
    spi_transfer_byte(spi_base_, 0xFF);
    
    for (int i = 0; i < 10000; ++i) {
        uint8_t r = spi_transfer_byte(spi_base_, 0xFF);
        if (r == SD_TOKEN_START_BLOCK) {
            uint8_t* buf = static_cast<uint8_t*>(buffer);
            for (size_t j = 0; j < 512; ++j) {
                buf[j] = spi_transfer_byte(spi_base_, 0xFF);
            }
            spi_transfer_byte(spi_base_, 0xFF);
            spi_transfer_byte(spi_base_, 0xFF);
            spi_set_cs(spi_base_, false);
            spi_transfer_byte(spi_base_, 0xFF);
            return true;
        }
    }
    
    spi_set_cs(spi_base_, false);
    return false;
}

bool SDCardDriver::write_block(uint32_t block_num, const void* buffer) {
    if (!initialized_ || !buffer) return false;
    
    uint8_t cmd[5];
    cmd[0] = 0x40 | 24;
    cmd[1] = (block_num >> 24) & 0xFF;
    cmd[2] = (block_num >> 16) & 0xFF;
    cmd[3] = (block_num >> 8) & 0xFF;
    cmd[4] = block_num & 0xFF;
    
    spi_set_cs(spi_base_, true);
    for (int i = 0; i < 5; ++i) {
        spi_transfer_byte(spi_base_, cmd[i]);
    }
    spi_transfer_byte(spi_base_, 0xFF);
    
    spi_transfer_byte(spi_base_, SD_TOKEN_WRITE_START);
    const uint8_t* buf = static_cast<const uint8_t*>(buffer);
    for (size_t i = 0; i < 512; ++i) {
        spi_transfer_byte(spi_base_, buf[i]);
    }
    spi_transfer_byte(spi_base_, 0xFF);
    spi_transfer_byte(spi_base_, 0xFF);
    
    for (int i = 0; i < 10000; ++i) {
        uint8_t r = spi_transfer_byte(spi_base_, 0xFF);
        if ((r & 0x1F) == 0x05) {
            break;
        }
    }
    
    for (int i = 0; i < 10000; ++i) {
        if (spi_transfer_byte(spi_base_, 0xFF) == 0xFF) {
            break;
        }
    }
    
    spi_set_cs(spi_base_, false);
    spi_transfer_byte(spi_base_, 0xFF);
    
    return true;
}

bool SDCardDriver::get_card_info(CardInfo& info) {
    if (!initialized_) return false;
    info = card_info_;
    return true;
}

bool SDCardDriver::is_present() {
    return true;
}

bool init_sdcard(uintptr_t spi_base, uint32_t cs_pin) {
    return g_sdcard.init(spi_base, cs_pin);
}

SDCardDriver* get_sdcard() {
    return &g_sdcard;
}

} // namespace sdcard
} // namespace shared
} // namespace hal
