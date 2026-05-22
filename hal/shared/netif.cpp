// SPDX-License-Identifier: MIT OR Apache-2.0

#include "netif.hpp"
#include "tcp.hpp"

#include <cstring>

namespace kernel::net {

namespace {

constexpr uint16_t ETHERTYPE_IPV4 = 0x0800u;
constexpr uint8_t  IPPROTO_UDP    = 17u;
constexpr uint8_t  IPPROTO_TCP    = 6u;

// Packed L2/L3/L4 header structs — kept locally so the netif doesn't
// depend on hmi/* layout. Identical wire format.
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

inline uint16_t bswap16(uint16_t v) { return static_cast<uint16_t>((v >> 8) | (v << 8)); }
inline uint32_t bswap32(uint32_t v) {
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) <<  8) |
           ((v & 0x00FF0000u) >>  8) |
           ((v & 0xFF000000u) >> 24);
}

uint16_t ip_checksum(const uint8_t* data, size_t len) noexcept {
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < len; i += 2) {
        sum += static_cast<uint16_t>((static_cast<uint16_t>(data[i]) << 8) | data[i + 1]);
    }
    if (len & 1u) sum += static_cast<uint16_t>(data[len - 1] << 8);
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

} // namespace

Netif g_netifs[MAX_NETIFS];

void Netif::bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept {
    if_idx_ = if_idx;
    nic_ = nic;
}

bool Netif::register_listener(EthListener* l) noexcept {
    if (!l || listener_count_ >= MAX_LISTENERS) return false;
    for (size_t i = 0; i < listener_count_; ++i) {
        if (listeners_[i] == l) return true;  // already registered
    }
    listeners_[listener_count_++] = l;
    return true;
}

void Netif::unregister_listener(EthListener* l) noexcept {
    for (size_t i = 0; i < listener_count_; ++i) {
        if (listeners_[i] == l) {
            for (size_t j = i + 1; j < listener_count_; ++j) {
                listeners_[j - 1] = listeners_[j];
            }
            listeners_[--listener_count_] = nullptr;
            return;
        }
    }
}

void Netif::rx_trampoline(int if_idx, const uint8_t* data, size_t len, void* ctx) noexcept {
    auto* self = static_cast<Netif*>(ctx);
    if (self) self->dispatch(if_idx, data, len);
}

bool Netif::register_udp_listener(UdpListener* l) noexcept {
    if (!l || udp_listener_count_ >= MAX_UDP_LISTENERS) return false;
    for (size_t i = 0; i < udp_listener_count_; ++i) {
        if (udp_listeners_[i] == l) return true;
    }
    udp_listeners_[udp_listener_count_++] = l;
    return true;
}

void Netif::unregister_udp_listener(UdpListener* l) noexcept {
    for (size_t i = 0; i < udp_listener_count_; ++i) {
        if (udp_listeners_[i] == l) {
            for (size_t j = i + 1; j < udp_listener_count_; ++j) {
                udp_listeners_[j - 1] = udp_listeners_[j];
            }
            udp_listeners_[--udp_listener_count_] = nullptr;
            return;
        }
    }
}

bool Netif::udp_port_claimed(uint16_t local_port) const noexcept {
    for (size_t i = 0; i < udp_listener_count_; ++i) {
        auto* l = udp_listeners_[i];
        if (!l) continue;
        const uint16_t p = l->local_port();
        if (p == 0 || p == local_port) return true;
    }
    return false;
}

void Netif::try_dispatch_udp(int if_idx, const uint8_t* data, size_t len) noexcept {
    if (!nic_) return;
    if (len < sizeof(EthernetHeader) + sizeof(IPv4Header) + sizeof(UdpHeader)) return;
    const auto* eth = reinterpret_cast<const EthernetHeader*>(data);
    if (bswap16(eth->ethertype_be) != ETHERTYPE_IPV4) return;
    const auto* ip = reinterpret_cast<const IPv4Header*>(data + sizeof(EthernetHeader));
    const uint8_t ihl = (ip->ver_ihl & 0x0Fu) * 4u;
    if ((ip->ver_ihl >> 4) != 4 || ihl < sizeof(IPv4Header)) return;
    if (ip->proto != IPPROTO_UDP) return;
    // Drop IPv4 fragments — RX reassembly isn't implemented yet. The
    // MF flag (bit 13) or a non-zero fragment offset (bits 0–12) means
    // this is mid-datagram. Hosts behind QEMU's SLIRP / hostfwd never
    // see fragments because Linux / SLIRP reassemble for us, so this
    // matters only on real hardware with a misconfigured peer.
    const uint16_t flags_frag = bswap16(ip->flags_frag_be);
    if ((flags_frag & 0x2000u) || (flags_frag & 0x1FFFu)) return;
    const size_t ip_total = bswap16(ip->total_len_be);
    if (sizeof(EthernetHeader) + ip_total > len) return;
    if (ip_total < ihl + sizeof(UdpHeader)) return;
    const auto* udp = reinterpret_cast<const UdpHeader*>(
        data + sizeof(EthernetHeader) + ihl);
    const uint16_t udp_len = bswap16(udp->length_be);
    if (udp_len < sizeof(UdpHeader)) return;
    if (sizeof(EthernetHeader) + ihl + udp_len > len) return;
    const uint16_t dst_port = bswap16(udp->dst_port_be);
    const uint16_t src_port = bswap16(udp->src_port_be);
    const uint32_t src_ip = bswap32(ip->src_ip_be);
    const uint32_t dst_ip = bswap32(ip->dst_ip_be);
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(udp) + sizeof(UdpHeader);
    const size_t payload_len = udp_len - sizeof(UdpHeader);
    for (size_t i = 0; i < udp_listener_count_; ++i) {
        auto* l = udp_listeners_[i];
        if (!l) continue;
        const uint16_t want = l->local_port();
        if (want != 0 && want != dst_port) continue;
        l->on_udp(if_idx, *nic_, eth->src,
                  src_ip, src_port, dst_ip, dst_port,
                  payload, payload_len);
    }
}

void Netif::dispatch(int if_idx, const uint8_t* data, size_t len) noexcept {
    if (!data || len < 14) return;
    // UDP and TCP listeners are tried first. Eth-frame listeners come
    // after so existing catch-all consumers (HMI's handle_eth_frame)
    // still see every frame; HMI gates its handle_udp on
    // udp_port_claimed() to avoid double-dispatch.
    try_dispatch_udp(if_idx, data, len);
    // TCP dispatch — parse IPv4 header inline and hand the segment to
    // the TCP module. Frames that aren't TCP fall through.
    if (len >= sizeof(EthernetHeader) + sizeof(IPv4Header)) {
        const auto* eth = reinterpret_cast<const EthernetHeader*>(data);
        if (bswap16(eth->ethertype_be) == ETHERTYPE_IPV4) {
            const auto* ip = reinterpret_cast<const IPv4Header*>(data + sizeof(EthernetHeader));
            const uint8_t ihl = (ip->ver_ihl & 0x0Fu) * 4u;
            if ((ip->ver_ihl >> 4) == 4 && ihl >= sizeof(IPv4Header) &&
                ip->proto == IPPROTO_TCP) {
                const uint16_t flags_frag = bswap16(ip->flags_frag_be);
                if (!((flags_frag & 0x2000u) || (flags_frag & 0x1FFFu))) {
                    const size_t ip_total = bswap16(ip->total_len_be);
                    if (sizeof(EthernetHeader) + ip_total <= len &&
                        ip_total >= ihl) {
                        tcp_dispatch(*this, eth->src,
                                     bswap32(ip->src_ip_be), bswap32(ip->dst_ip_be),
                                     data + sizeof(EthernetHeader) + ihl,
                                     ip_total - ihl);
                    }
                }
            }
        }
    }
    const uint16_t et_be = static_cast<uint16_t>((data[12] << 8) | data[13]);
    for (size_t i = 0; i < listener_count_; ++i) {
        auto* l = listeners_[i];
        if (!l) continue;
        const uint16_t want = l->ethertype();
        if (want != ETHERTYPE_ANY && want != et_be) continue;
        if (nic_) l->on_eth_frame(if_idx, *nic_, data, len);
    }
}

size_t Netif::poll(size_t budget) noexcept {
    if (!nic_) return 0;
    return nic_->poll_rx(&rx_trampoline, this, budget);
}

Netif* netif_bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept {
    if (if_idx < 0 || static_cast<size_t>(if_idx) >= MAX_NETIFS) return nullptr;
    g_netifs[if_idx].bind(if_idx, nic);
    return &g_netifs[if_idx];
}

Netif* netif_get(int if_idx) noexcept {
    if (if_idx < 0 || static_cast<size_t>(if_idx) >= MAX_NETIFS) return nullptr;
    Netif* n = &g_netifs[if_idx];
    return (n->nic() != nullptr) ? n : nullptr;
}

namespace {

// IPv4 fragmentation flag bits (in flags_frag_be field, host order).
constexpr uint16_t IP_FLAG_MF = 0x2000;     // more fragments
constexpr uint16_t IP_FRAG_OFFSET_MASK = 0x1FFF;
constexpr size_t   IP_MTU = 1500;
constexpr size_t   MAX_FRAG_PAYLOAD = (IP_MTU - sizeof(IPv4Header)) & ~size_t{7};  // 1480
// Hard cap on the UDP segment we'll fragment. Most consumers send well
// under this (TSV upload chunks at 1400 bytes); the cap exists so the
// staging buffer doesn't grow unbounded.
constexpr size_t MAX_UDP_PAYLOAD = 8192;
constexpr size_t FRAME_CAP = 2048;  // one MTU + slack

// IP datagram identification — incremented per outbound datagram so
// re-fragmentation by a downstream router (if any) won't alias with
// the next datagram from this host.
uint16_t g_ip_ident = 0x4d49;  // 'MI'

uint16_t udp_checksum(uint32_t src_ip, uint32_t dst_ip,
                      const uint8_t* udp_segment, size_t udp_len) noexcept {
    // Pseudo-header: src(4) dst(4) zero(1) proto(1) udp_len(2) = 12 bytes.
    uint32_t sum = 0;
    sum += (src_ip >> 16) & 0xFFFFu;
    sum +=  src_ip        & 0xFFFFu;
    sum += (dst_ip >> 16) & 0xFFFFu;
    sum +=  dst_ip        & 0xFFFFu;
    sum += IPPROTO_UDP;
    sum += static_cast<uint16_t>(udp_len);
    for (size_t i = 0; i + 1 < udp_len; i += 2) {
        sum += static_cast<uint16_t>((static_cast<uint16_t>(udp_segment[i]) << 8) | udp_segment[i + 1]);
    }
    if (udp_len & 1u) sum += static_cast<uint16_t>(udp_segment[udp_len - 1] << 8);
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    uint16_t cs = static_cast<uint16_t>(~sum);
    // RFC 768: a transmitted zero checksum is reserved to mean "no
    // checksum used"; if the real checksum computes to 0, transmit
    // 0xFFFF instead.
    return cs == 0 ? 0xFFFFu : cs;
}

} // namespace

bool udp_sendto(Netif& netif,
                const uint8_t* dst_mac, const uint8_t* src_mac,
                uint32_t src_ip, uint32_t dst_ip,
                uint16_t src_port, uint16_t dst_port,
                const uint8_t* payload, size_t payload_len) noexcept {
    if (!dst_mac || !src_mac) return false;
    if (payload_len > MAX_UDP_PAYLOAD) return false;
    auto* nic = netif.nic();
    if (!nic) return false;

    // Single-threaded per netif (one worker thread owns each NIC).
    static uint8_t udp_segment[MAX_UDP_PAYLOAD + sizeof(UdpHeader)];
    static uint8_t frame[FRAME_CAP];

    // Build the UDP segment once: header + payload, with proper checksum.
    const size_t udp_seg_len = sizeof(UdpHeader) + payload_len;
    auto* udp = reinterpret_cast<UdpHeader*>(udp_segment);
    udp->src_port_be = bswap16(src_port);
    udp->dst_port_be = bswap16(dst_port);
    udp->length_be   = bswap16(static_cast<uint16_t>(udp_seg_len));
    udp->checksum_be = 0;  // zero for checksum computation
    if (payload_len && payload) {
        std::memcpy(udp_segment + sizeof(UdpHeader), payload, payload_len);
    }
    udp->checksum_be = bswap16(udp_checksum(src_ip, dst_ip, udp_segment, udp_seg_len));

    const uint16_t ident = g_ip_ident++;
    size_t offset = 0;
    while (offset < udp_seg_len) {
        const size_t remaining = udp_seg_len - offset;
        const size_t chunk = (remaining > MAX_FRAG_PAYLOAD) ? MAX_FRAG_PAYLOAD : remaining;
        const bool more = (offset + chunk) < udp_seg_len;
        const size_t frame_len = sizeof(EthernetHeader) + sizeof(IPv4Header) + chunk;
        if (frame_len > FRAME_CAP) return false;

        std::memset(frame, 0, sizeof(EthernetHeader) + sizeof(IPv4Header));
        auto* eth = reinterpret_cast<EthernetHeader*>(frame);
        for (size_t i = 0; i < 6; ++i) {
            eth->dst[i] = dst_mac[i];
            eth->src[i] = src_mac[i];
        }
        eth->ethertype_be = bswap16(ETHERTYPE_IPV4);

        auto* ip = reinterpret_cast<IPv4Header*>(frame + sizeof(EthernetHeader));
        ip->ver_ihl       = 0x45;
        ip->dscp_ecn      = 0;
        ip->total_len_be  = bswap16(static_cast<uint16_t>(sizeof(IPv4Header) + chunk));
        ip->ident_be      = bswap16(ident);
        const uint16_t flags_frag = static_cast<uint16_t>(
            (more ? IP_FLAG_MF : 0u) | ((offset / 8u) & IP_FRAG_OFFSET_MASK));
        ip->flags_frag_be = bswap16(flags_frag);
        ip->ttl           = 64;
        ip->proto         = IPPROTO_UDP;
        ip->hdr_checksum_be = 0;
        ip->src_ip_be     = bswap32(src_ip);
        ip->dst_ip_be     = bswap32(dst_ip);
        ip->hdr_checksum_be = bswap16(ip_checksum(reinterpret_cast<const uint8_t*>(ip),
                                                  sizeof(IPv4Header)));

        std::memcpy(frame + sizeof(EthernetHeader) + sizeof(IPv4Header),
                    udp_segment + offset, chunk);

        if (!nic->send_packet(netif.if_idx(), frame, frame_len)) return false;
        offset += chunk;
    }
    return true;
}

} // namespace kernel::net
