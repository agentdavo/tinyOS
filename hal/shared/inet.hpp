// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// Shared Internet-protocol wire formats and checksums. netif (IPv4/UDP),
// tcp and the HMI service each used to carry their own copy of these
// structs, byte swaps and one's-complement checksum loops.

#include <cstddef>
#include <cstdint>

namespace kernel::net::inet {

constexpr uint16_t ETHERTYPE_IPV4 = 0x0800u;
constexpr uint8_t  IPPROTO_ICMP   = 1u;
constexpr uint8_t  IPPROTO_TCP    = 6u;
constexpr uint8_t  IPPROTO_UDP    = 17u;

struct EthernetHeader {
    uint8_t  dst[6];
    uint8_t  src[6];
    uint16_t ethertype_be;
} __attribute__((packed));

struct IPv4Header {
    uint8_t  ver_ihl;
    uint8_t  dscp_ecn;
    uint16_t total_len_be;
    uint16_t ident_be;
    uint16_t flags_frag_be;
    uint8_t  ttl;
    uint8_t  proto;
    uint16_t hdr_checksum_be;
    uint32_t src_ip_be;
    uint32_t dst_ip_be;
} __attribute__((packed));

struct UdpHeader {
    uint16_t src_port_be;
    uint16_t dst_port_be;
    uint16_t length_be;
    uint16_t checksum_be;
} __attribute__((packed));

// Both targets are little-endian; network order is big-endian.
inline uint16_t bswap16(uint16_t v) noexcept { return __builtin_bswap16(v); }
inline uint32_t bswap32(uint32_t v) noexcept { return __builtin_bswap32(v); }

// Running one's-complement sum over big-endian 16-bit words (an odd
// trailing byte is padded with zero).
inline uint32_t csum_add(uint32_t sum, const uint8_t* data, size_t len) noexcept {
    for (size_t i = 0; i + 1 < len; i += 2) {
        sum += static_cast<uint16_t>((static_cast<uint16_t>(data[i]) << 8) | data[i + 1]);
    }
    if (len & 1u) sum += static_cast<uint16_t>(data[len - 1] << 8);
    return sum;
}

inline uint16_t csum_fold(uint32_t sum) noexcept {
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

// RFC 1071 Internet checksum (IPv4 header, ICMP).
inline uint16_t checksum16(const uint8_t* data, size_t len) noexcept {
    return csum_fold(csum_add(0, data, len));
}

// TCP/UDP checksum over the IPv4 pseudo-header (src, dst, proto, length)
// plus the segment. IPs are host-order. Callers apply protocol quirks, e.g.
// UDP's "transmit 0 as 0xFFFF".
inline uint16_t l4_checksum(uint8_t proto, uint32_t src_ip, uint32_t dst_ip,
                            const uint8_t* segment, size_t seg_len) noexcept {
    uint32_t sum = (src_ip >> 16) + (src_ip & 0xFFFFu) +
                   (dst_ip >> 16) + (dst_ip & 0xFFFFu) +
                   proto + static_cast<uint16_t>(seg_len);
    return csum_fold(csum_add(sum, segment, seg_len));
}

} // namespace kernel::net::inet
