// SPDX-License-Identifier: MIT OR Apache-2.0
// EtherCAT L2 framing — implementation.

#include "frame.hpp"

#include <cstring>

namespace ethercat {

FrameBuilder::FrameBuilder(uint8_t* buf, size_t cap,
                           const uint8_t dst_mac[6],
                           const uint8_t src_mac[6]) noexcept
    : buf_(buf), cap_(cap) {
    if (cap_ < ETH_HDR_LEN + ECAT_HDR_LEN) { buf_ = nullptr; return; }
    std::memset(buf_, 0, cap_);
    std::memcpy(&buf_[0], dst_mac, 6);
    std::memcpy(&buf_[6], src_mac, 6);
    buf_[12] = static_cast<uint8_t>(ETHERTYPE_ECAT_L2 >> 8);
    buf_[13] = static_cast<uint8_t>(ETHERTYPE_ECAT_L2 & 0xff);
    // EtherCAT header patched in finalize(); reserve 2 bytes now.
    size_ = ETH_HDR_LEN + ECAT_HDR_LEN;
}

size_t FrameBuilder::add_datagram(Cmd cmd, uint8_t idx,
                                  uint32_t address,
                                  const uint8_t* data, uint16_t data_len,
                                  bool more) noexcept {
    if (!buf_) return 0;
    const size_t need = DGRAM_HDR_LEN + data_len + DGRAM_TRAIL_LEN;
    if (size_ + need > cap_) return 0;

    uint8_t* p = buf_ + size_;
    p[0] = static_cast<uint8_t>(cmd);
    p[1] = idx;
    put_u32_le(&p[2], address);
    // length field (11 bits) + flags
    uint16_t lf = static_cast<uint16_t>(data_len & 0x07FF);
    if (more) lf |= 0x8000; // 'more' bit
    put_u16_le(&p[6], lf);
    put_u16_le(&p[8], 0); // IRQ
    if (data) std::memcpy(&p[10], data, data_len);
    // WKC trailer already zero (memset).

    // If there was a previous datagram, patch its 'more' bit on.
    if (last_dgram_off_ != 0) {
        uint8_t* prev = buf_ + last_dgram_off_;
        uint16_t plf = get_u16_le(&prev[6]);
        plf |= 0x8000;
        put_u16_le(&prev[6], plf);
    }

    const size_t data_off = size_ + DGRAM_HDR_LEN;
    last_dgram_off_ = size_;
    size_ += need;
    return data_off;
}

size_t FrameBuilder::finalize() noexcept {
    if (!buf_) return 0;
    // EtherCAT header = length (11 bits) | reserved (1) | type (4).
    uint16_t payload_len = static_cast<uint16_t>(size_ - ETH_HDR_LEN - ECAT_HDR_LEN);
    uint16_t hdr = static_cast<uint16_t>((payload_len & 0x07FF) |
                                         ((ECAT_TYPE_CMDS & 0xF) << 12));
    put_u16_le(&buf_[ETH_HDR_LEN], hdr);

    // Pad to minimum Ethernet size (60 B excl FCS).
    if (size_ < MIN_ETH_FRAME_LEN) {
        // buffer was memset'd to 0 at construction; just advance.
        size_ = MIN_ETH_FRAME_LEN;
    }
    return size_;
}

FrameParser::FrameParser(const uint8_t* buf, size_t len) noexcept {
    if (!buf || len < ETH_HDR_LEN + ECAT_HDR_LEN) return;
    uint16_t et = static_cast<uint16_t>((buf[12] << 8) | buf[13]);
    if (et != ETHERTYPE_ECAT_L2) return;
    uint16_t hdr = get_u16_le(&buf[ETH_HDR_LEN]);
    uint16_t plen = hdr & 0x07FF;
    uint16_t type = (hdr >> 12) & 0xF;
    if (type != ECAT_TYPE_CMDS) return;
    size_t begin = ETH_HDR_LEN + ECAT_HDR_LEN;
    size_t stop  = begin + plen;
    if (stop > len) stop = len;
    p_ = buf + begin;
    end_ = buf + stop;
    valid_ = true;
}

bool FrameParser::next(Datagram& out) noexcept {
    if (!valid_ || !p_ || p_ + DGRAM_HDR_LEN + DGRAM_TRAIL_LEN > end_) return false;
    out.cmd      = static_cast<Cmd>(p_[0]);
    out.idx      = p_[1];
    out.address  = get_u32_le(&p_[2]);
    uint16_t lf  = get_u16_le(&p_[6]);
    out.data_len = static_cast<uint16_t>(lf & 0x07FF);
    out.more     = (lf & 0x8000) != 0;
    if (p_ + DGRAM_HDR_LEN + out.data_len + DGRAM_TRAIL_LEN > end_) return false;
    out.data = &p_[DGRAM_HDR_LEN];
    out.wkc  = get_u16_le(p_ + DGRAM_HDR_LEN + out.data_len);
    p_ += DGRAM_HDR_LEN + out.data_len + DGRAM_TRAIL_LEN;
    return true;
}

MutableFrameParser::MutableFrameParser(uint8_t* buf, size_t len) noexcept {
    if (!buf || len < ETH_HDR_LEN + ECAT_HDR_LEN) return;
    uint16_t et = static_cast<uint16_t>((buf[12] << 8) | buf[13]);
    if (et != ETHERTYPE_ECAT_L2) return;
    uint16_t hdr = get_u16_le(&buf[ETH_HDR_LEN]);
    uint16_t plen = hdr & 0x07FF;
    uint16_t type = (hdr >> 12) & 0xF;
    if (type != ECAT_TYPE_CMDS) return;
    size_t begin = ETH_HDR_LEN + ECAT_HDR_LEN;
    size_t stop  = begin + plen;
    if (stop > len) stop = len;
    p_ = buf + begin;
    end_ = buf + stop;
    valid_ = true;
}

bool MutableFrameParser::next(Datagram& out) noexcept {
    if (!valid_ || !p_ || p_ + DGRAM_HDR_LEN + DGRAM_TRAIL_LEN > end_) return false;
    out.cmd      = static_cast<Cmd>(p_[0]);
    out.idx      = p_[1];
    out.address  = get_u32_le(&p_[2]);
    uint16_t lf  = get_u16_le(&p_[6]);
    out.data_len = static_cast<uint16_t>(lf & 0x07FF);
    out.more     = (lf & 0x8000) != 0;
    if (p_ + DGRAM_HDR_LEN + out.data_len + DGRAM_TRAIL_LEN > end_) return false;
    out.data    = &p_[DGRAM_HDR_LEN];
    out.wkc_ptr = p_ + DGRAM_HDR_LEN + out.data_len;
    p_ += DGRAM_HDR_LEN + out.data_len + DGRAM_TRAIL_LEN;
    return true;
}

} // namespace ethercat
