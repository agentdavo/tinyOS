// SPDX-License-Identifier: MIT OR Apache-2.0

#include "netif.hpp"

#include <cstring>

namespace kernel::net {

namespace {

constexpr uint16_t ETHERTYPE_IPV4 = 0x0800u;
constexpr uint8_t  IPPROTO_UDP    = 17u;

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
    // UDP listeners are tried first. Eth-frame listeners come after so
    // existing catch-all consumers (HMI's handle_eth_frame) still see
    // every frame; HMI gates its handle_udp on udp_port_claimed() to
    // avoid double-dispatch on ports owned by a UDP listener.
    try_dispatch_udp(if_idx, data, len);
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

bool udp_sendto(Netif& netif,
                const uint8_t* dst_mac, const uint8_t* src_mac,
                uint32_t src_ip, uint32_t dst_ip,
                uint16_t src_port, uint16_t dst_port,
                const uint8_t* payload, size_t payload_len) noexcept {
    constexpr size_t FRAME_CAP = 2048;
    constexpr size_t HDRS = sizeof(EthernetHeader) + sizeof(IPv4Header) + sizeof(UdpHeader);
    if (!dst_mac || !src_mac) return false;
    if (payload_len > FRAME_CAP - HDRS) return false;
    auto* nic = netif.nic();
    if (!nic) return false;

    static uint8_t frame[FRAME_CAP];  // single-threaded per netif (one worker per NIC)
    std::memset(frame, 0, HDRS);

    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = dst_mac[i];
        eth->src[i] = src_mac[i];
    }
    eth->ethertype_be = bswap16(ETHERTYPE_IPV4);

    auto* ip = reinterpret_cast<IPv4Header*>(frame + sizeof(EthernetHeader));
    ip->ver_ihl       = 0x45;
    ip->dscp_ecn      = 0;
    ip->total_len_be  = bswap16(static_cast<uint16_t>(sizeof(IPv4Header) + sizeof(UdpHeader) + payload_len));
    ip->ident_be      = 0;
    ip->flags_frag_be = 0;
    ip->ttl           = 64;
    ip->proto         = IPPROTO_UDP;
    ip->hdr_checksum_be = 0;
    ip->src_ip_be     = bswap32(src_ip);
    ip->dst_ip_be     = bswap32(dst_ip);
    ip->hdr_checksum_be = bswap16(ip_checksum(reinterpret_cast<const uint8_t*>(ip), sizeof(IPv4Header)));

    auto* udp = reinterpret_cast<UdpHeader*>(frame + sizeof(EthernetHeader) + sizeof(IPv4Header));
    udp->src_port_be = bswap16(src_port);
    udp->dst_port_be = bswap16(dst_port);
    udp->length_be   = bswap16(static_cast<uint16_t>(sizeof(UdpHeader) + payload_len));
    udp->checksum_be = 0;  // IPv4 permits zero — Tier 2 follow-up

    if (payload_len && payload) {
        std::memcpy(frame + HDRS, payload, payload_len);
    }
    return nic->send_packet(netif.if_idx(), frame, HDRS + payload_len);
}

} // namespace kernel::net
