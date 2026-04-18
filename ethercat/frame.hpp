// SPDX-License-Identifier: MIT OR Apache-2.0
// EtherCAT Layer-2 framing. Builds TX frames and walks RX frames.
//
// Frame layout:
//   [ Ethernet header 14 B ]
//   [ EtherCAT header   2 B : len:11 | res:1 | type:4 ]
//   [ Datagram 0 ] [ Datagram 1 ] ... [ padding to 60 B ]
//
// Datagram layout (Type 1 = EtherCAT commands over Ethernet):
//   cmd(1) idx(1) addr(4) len:11|res:3|c:1|more:1(2) irq(2) data(len) wkc(2)

#ifndef ETHERCAT_FRAME_HPP
#define ETHERCAT_FRAME_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace ethercat {

// Helpers for little-endian pack/unpack — forward-declared at namespace top
// because MutableFrameParser::get_wkc/set_wkc/inc_wkc below use them.
inline void put_u16_le(uint8_t* p, uint16_t v) noexcept {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_u32_le(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}
inline uint16_t get_u16_le(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint32_t get_u32_le(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

constexpr uint16_t ETHERTYPE_ECAT_L2 = 0x88A4;

// EtherCAT datagram command codes (subset — full list per spec).
enum class Cmd : uint8_t {
    NOP  = 0x00,
    APRD = 0x01, // Auto-increment physical read
    APWR = 0x02, // Auto-increment physical write
    APRW = 0x03, // Auto-increment physical read/write
    FPRD = 0x04, // Configured addr physical read
    FPWR = 0x05, // Configured addr physical write
    FPRW = 0x06,
    BRD  = 0x07, // Broadcast read
    BWR  = 0x08, // Broadcast write
    BRW  = 0x09, // Broadcast read/write
    LRD  = 0x0A, // Logical read
    LWR  = 0x0B, // Logical write
    LRW  = 0x0C, // Logical read/write
};

// Header byte offsets inside an assembled TX buffer.
constexpr size_t ETH_HDR_LEN        = 14;
constexpr size_t ECAT_HDR_LEN       = 2;
constexpr size_t DGRAM_HDR_LEN      = 10;  // cmd+idx+addr+len_flags+irq
constexpr size_t DGRAM_TRAIL_LEN    = 2;   // wkc
constexpr size_t MIN_ETH_FRAME_LEN  = 60;  // excl. FCS
constexpr size_t MAX_FRAME_LEN      = 1514;
constexpr uint16_t ECAT_TYPE_CMDS   = 0x1; // EtherCAT commands (vs. mailbox etc.)

// Assemble a TX frame datagram-by-datagram.
class FrameBuilder {
public:
    explicit FrameBuilder(uint8_t* buf, size_t cap,
                          const uint8_t dst_mac[6],
                          const uint8_t src_mac[6]) noexcept;

    // Append one datagram. `data` is copied into the frame and may be nullptr
    // (in which case `data_len` bytes of zeros are reserved).
    // Returns the offset of the data payload (for later reply inspection) or 0
    // on overflow.
    size_t add_datagram(Cmd cmd, uint8_t idx,
                        uint32_t address,
                        const uint8_t* data, uint16_t data_len,
                        bool more = false) noexcept;

    // Finalise: patches the EtherCAT header length and pads to 60 B.
    size_t finalize() noexcept;

    size_t size() const noexcept { return size_; }
    uint8_t* buffer() noexcept { return buf_; }

private:
    uint8_t* buf_;
    size_t   cap_;
    size_t   size_ = 0;
    size_t   last_dgram_off_ = 0;
};

// Iterate datagrams in an RX frame. Non-owning.
class FrameParser {
public:
    struct Datagram {
        Cmd      cmd;
        uint8_t  idx;
        uint32_t address;
        const uint8_t* data;
        uint16_t data_len;
        uint16_t wkc;
        bool     more;
    };

    FrameParser(const uint8_t* buf, size_t len) noexcept;

    bool next(Datagram& out) noexcept;
    bool valid() const noexcept { return valid_; }

private:
    const uint8_t* p_ = nullptr;      // Cursor into the datagram region.
    const uint8_t* end_ = nullptr;
    bool           valid_ = false;
};

// Iterate datagrams in a frame buffer with mutable access. Used by the fake
// slave (and any future slave-side transit logic) to rewrite the data field
// for read commands and increment the WKC trailer in place — exactly mirroring
// what a real EtherCAT slave does as a frame transits through it.
class MutableFrameParser {
public:
    struct Datagram {
        Cmd      cmd;
        uint8_t  idx;
        uint32_t address;
        uint8_t* data;       // pointer into the underlying buffer (writable)
        uint16_t data_len;
        uint8_t* wkc_ptr;    // points at the 2-byte WKC trailer (LE u16)
        bool     more;
    };

    MutableFrameParser(uint8_t* buf, size_t len) noexcept;

    bool next(Datagram& out) noexcept;
    bool valid() const noexcept { return valid_; }

    // Read/modify the WKC stored at wkc_ptr.
    static uint16_t get_wkc(const uint8_t* wkc_ptr) noexcept {
        return get_u16_le(wkc_ptr);
    }
    static void set_wkc(uint8_t* wkc_ptr, uint16_t v) noexcept {
        put_u16_le(wkc_ptr, v);
    }
    static void inc_wkc(uint8_t* wkc_ptr, uint16_t delta = 1) noexcept {
        put_u16_le(wkc_ptr,
                   static_cast<uint16_t>(get_u16_le(wkc_ptr) + delta));
    }

private:
    uint8_t* p_   = nullptr;
    uint8_t* end_ = nullptr;
    bool     valid_ = false;
};

} // namespace ethercat

#endif
