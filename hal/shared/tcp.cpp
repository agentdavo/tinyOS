// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Minimal TCP listener. See tcp.hpp for the scope and design notes.
//
// One file, one mutex-free assumption: every entry point (tcp_dispatch,
// TcpConnection::send, TcpConnection::close) runs on the netif's worker
// thread. The listener callbacks (on_open / on_data / on_close) are
// invoked from tcp_dispatch and may call conn.send() / conn.close()
// inline without further locking.

#include "tcp.hpp"

#include <cstring>

namespace kernel::net {

namespace {

constexpr uint8_t  IPPROTO_TCP = 6u;
constexpr uint16_t ETHERTYPE_IPV4 = 0x0800u;
constexpr uint32_t INITIAL_SEQ = 0x12345678u;  // simple ISN; not against spoofing

// TCP flags (control bits in the data offset / flags word).
constexpr uint16_t TCP_FLAG_FIN = 0x0001;
constexpr uint16_t TCP_FLAG_SYN = 0x0002;
constexpr uint16_t TCP_FLAG_RST = 0x0004;
constexpr uint16_t TCP_FLAG_PSH = 0x0008;
constexpr uint16_t TCP_FLAG_ACK = 0x0010;

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

struct TcpHeader {
    uint16_t src_port_be;
    uint16_t dst_port_be;
    uint32_t seq_be;
    uint32_t ack_be;
    uint16_t off_flags_be;   // top 4 bits = data offset (32-bit words), low 12 = flags+reserved
    uint16_t window_be;
    uint16_t checksum_be;
    uint16_t urgent_be;
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

uint16_t tcp_checksum(uint32_t src_ip, uint32_t dst_ip,
                      const uint8_t* segment, size_t seg_len) noexcept {
    uint32_t sum = 0;
    sum += (src_ip >> 16) & 0xFFFFu;
    sum +=  src_ip        & 0xFFFFu;
    sum += (dst_ip >> 16) & 0xFFFFu;
    sum +=  dst_ip        & 0xFFFFu;
    sum += IPPROTO_TCP;
    sum += static_cast<uint16_t>(seg_len);
    for (size_t i = 0; i + 1 < seg_len; i += 2) {
        sum += static_cast<uint16_t>((static_cast<uint16_t>(segment[i]) << 8) | segment[i + 1]);
    }
    if (seg_len & 1u) sum += static_cast<uint16_t>(segment[seg_len - 1] << 8);
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

// One global bind table — every (netif, listener) pair lives here.
// Bindings are stable until tcp_unbind. The connection lives alongside
// the binding; we serve a single connection per listener at a time.
struct Binding {
    Netif*         netif = nullptr;
    TcpListener*   listener = nullptr;
    TcpConnection  conn{};
};

constexpr size_t MAX_BINDINGS = MAX_TCP_LISTENERS * MAX_NETIFS;
Binding g_bindings[MAX_BINDINGS];

Binding* find_binding(Netif& netif, uint16_t local_port) noexcept {
    for (auto& b : g_bindings) {
        if (b.netif == &netif && b.listener && b.listener->local_port() == local_port) return &b;
    }
    return nullptr;
}

Binding* find_binding_for_listener(Netif& netif, TcpListener* l) noexcept {
    for (auto& b : g_bindings) {
        if (b.netif == &netif && b.listener == l) return &b;
    }
    return nullptr;
}

Binding* alloc_binding() noexcept {
    for (auto& b : g_bindings) {
        if (!b.netif) return &b;
    }
    return nullptr;
}

bool send_segment(TcpConnection& conn,
                  uint16_t flags,
                  const uint8_t* payload, size_t payload_len) noexcept {
    constexpr size_t FRAME_CAP = 2048;
    constexpr size_t HDRS = sizeof(EthernetHeader) + sizeof(IPv4Header) + sizeof(TcpHeader);
    if (payload_len > FRAME_CAP - HDRS) return false;
    auto* netif = netif_get(-1);  // sentinel — replaced below
    (void)netif;
    // We don't have a pointer to Netif here. Look it up from listener's
    // binding. Linear scan over MAX_BINDINGS is fine (≤ 16 entries).
    Netif* nif = nullptr;
    for (auto& b : g_bindings) {
        if (&b.conn == &conn) { nif = b.netif; break; }
    }
    if (!nif || !nif->nic()) return false;
    auto* nic = nif->nic();

    static uint8_t frame[FRAME_CAP];
    std::memset(frame, 0, HDRS);

    // We don't currently track our own MAC at the Netif level; reuse the
    // peer MAC for L2 dst, and grab src from the NIC driver. For a real
    // multi-peer kernel we'd need ARP; for the loopback listener case
    // the peer MAC is what arrived in the SYN's Ethernet src, which we
    // stored in conn.peer_mac. Source MAC: query NIC.
    uint8_t our_mac[6]{};
    nic->get_mac(our_mac);

    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = conn.peer_mac[i];
        eth->src[i] = our_mac[i];
    }
    eth->ethertype_be = bswap16(ETHERTYPE_IPV4);

    const size_t tcp_len = sizeof(TcpHeader) + payload_len;
    auto* ip = reinterpret_cast<IPv4Header*>(frame + sizeof(EthernetHeader));
    ip->ver_ihl       = 0x45;
    ip->dscp_ecn      = 0;
    ip->total_len_be  = bswap16(static_cast<uint16_t>(sizeof(IPv4Header) + tcp_len));
    ip->ident_be      = 0;
    ip->flags_frag_be = bswap16(0x4000);  // DF bit
    ip->ttl           = 64;
    ip->proto         = IPPROTO_TCP;
    ip->hdr_checksum_be = 0;
    ip->src_ip_be     = bswap32(conn.local_ip);
    ip->dst_ip_be     = bswap32(conn.peer_ip);
    ip->hdr_checksum_be = bswap16(ip_checksum(reinterpret_cast<const uint8_t*>(ip), sizeof(IPv4Header)));

    auto* tcp = reinterpret_cast<TcpHeader*>(frame + sizeof(EthernetHeader) + sizeof(IPv4Header));
    tcp->src_port_be = bswap16(conn.local_port);
    tcp->dst_port_be = bswap16(conn.peer_port);
    tcp->seq_be      = bswap32(conn.snd_nxt);
    tcp->ack_be      = bswap32(conn.rcv_nxt);
    tcp->off_flags_be = bswap16(static_cast<uint16_t>((5u << 12) | (flags & 0x3FFu)));  // 5 32-bit words = 20 bytes
    tcp->window_be   = bswap16(conn.rcv_wnd);
    tcp->checksum_be = 0;
    tcp->urgent_be   = 0;

    if (payload_len && payload) {
        std::memcpy(frame + HDRS, payload, payload_len);
    }
    tcp->checksum_be = bswap16(tcp_checksum(conn.local_ip, conn.peer_ip,
                                            reinterpret_cast<const uint8_t*>(tcp), tcp_len));
    return nic->send_packet(nif->if_idx(), frame, HDRS + payload_len);
}

void reset_to_listen(Binding& b) noexcept {
    b.conn.state = TcpState::Listen;
    b.conn.peer_ip = 0;
    b.conn.peer_port = 0;
    std::memset(b.conn.peer_mac, 0, 6);
    b.conn.snd_nxt = INITIAL_SEQ;
    b.conn.snd_una = INITIAL_SEQ;
    b.conn.rcv_nxt = 0;
    b.conn.rcv_wnd = TcpConnection::RX_BUF_BYTES;
}

} // namespace

bool TcpConnection::send(const uint8_t* data, size_t len) noexcept {
    if (state != TcpState::Established) return false;
    if (len == 0) return true;
    if (!send_segment(*this, TCP_FLAG_ACK | TCP_FLAG_PSH, data, len)) return false;
    snd_nxt += static_cast<uint32_t>(len);
    snd_una = snd_nxt;  // pretend it was ACKed — no retransmit machinery
    return true;
}

void TcpConnection::close() noexcept {
    if (state == TcpState::Established || state == TcpState::CloseWait) {
        send_segment(*this, TCP_FLAG_ACK | TCP_FLAG_FIN, nullptr, 0);
        snd_nxt += 1;
        state = (state == TcpState::Established) ? TcpState::LastAck : TcpState::LastAck;
    }
}

bool tcp_bind(Netif& netif, TcpListener* l) noexcept {
    if (!l) return false;
    if (find_binding_for_listener(netif, l)) return true;
    if (find_binding(netif, l->local_port())) return false;  // port already taken
    Binding* b = alloc_binding();
    if (!b) return false;
    b->netif = &netif;
    b->listener = l;
    b->conn.listener = l;
    b->conn.local_port = l->local_port();
    reset_to_listen(*b);
    return true;
}

void tcp_unbind(Netif& netif, TcpListener* l) noexcept {
    if (auto* b = find_binding_for_listener(netif, l); b) {
        *b = Binding{};
    }
}

bool tcp_port_claimed(Netif& netif, uint16_t local_port) noexcept {
    return find_binding(netif, local_port) != nullptr;
}

bool tcp_dispatch(Netif& netif,
                  const uint8_t* src_mac,
                  uint32_t src_ip, uint32_t dst_ip,
                  const uint8_t* tcp_segment, size_t tcp_len) noexcept {
    if (tcp_len < sizeof(TcpHeader)) return false;
    const auto* tcp = reinterpret_cast<const TcpHeader*>(tcp_segment);
    const uint16_t off_flags = bswap16(tcp->off_flags_be);
    const uint8_t data_off = (off_flags >> 12) * 4u;
    if (data_off < sizeof(TcpHeader) || data_off > tcp_len) return false;
    const uint16_t flags = off_flags & 0x3FFu;
    const uint16_t dst_port = bswap16(tcp->dst_port_be);
    const uint16_t src_port = bswap16(tcp->src_port_be);
    const uint32_t seq = bswap32(tcp->seq_be);
    const uint32_t ack = bswap32(tcp->ack_be);
    const uint8_t* payload = tcp_segment + data_off;
    const size_t payload_len = tcp_len - data_off;

    Binding* b = find_binding(netif, dst_port);
    if (!b) {
        // Unbound port — send RST so the peer doesn't sit on a half-open
        // connection. We need ephemeral conn state to build the reply.
        TcpConnection tmp;
        tmp.local_ip = dst_ip; tmp.peer_ip = src_ip;
        tmp.local_port = dst_port; tmp.peer_port = src_port;
        for (size_t i = 0; i < 6; ++i) tmp.peer_mac[i] = src_mac[i];
        tmp.snd_nxt = (flags & TCP_FLAG_ACK) ? ack : 0;
        tmp.rcv_nxt = seq + ((flags & (TCP_FLAG_SYN | TCP_FLAG_FIN)) ? 1u : 0u) + payload_len;
        send_segment(tmp, TCP_FLAG_RST | TCP_FLAG_ACK, nullptr, 0);
        return true;
    }
    TcpConnection& conn = b->conn;

    // RST always tears down.
    if (flags & TCP_FLAG_RST) {
        reset_to_listen(*b);
        return true;
    }

    switch (conn.state) {
        case TcpState::Listen: {
            if (!(flags & TCP_FLAG_SYN)) return true;  // ignore non-SYN
            conn.local_ip   = dst_ip;
            conn.peer_ip    = src_ip;
            conn.peer_port  = src_port;
            for (size_t i = 0; i < 6; ++i) conn.peer_mac[i] = src_mac[i];
            conn.rcv_nxt    = seq + 1u;
            conn.snd_nxt    = INITIAL_SEQ;
            conn.snd_una    = INITIAL_SEQ;
            send_segment(conn, TCP_FLAG_SYN | TCP_FLAG_ACK, nullptr, 0);
            conn.snd_nxt   += 1u;  // SYN consumes a sequence number
            conn.state      = TcpState::SynReceived;
            return true;
        }
        case TcpState::SynReceived: {
            if (!(flags & TCP_FLAG_ACK)) return true;
            if (ack != conn.snd_nxt) {
                // Peer ACKed something other than our SYN — bail.
                send_segment(conn, TCP_FLAG_RST, nullptr, 0);
                reset_to_listen(*b);
                return true;
            }
            conn.snd_una = ack;
            conn.state   = TcpState::Established;
            b->listener->on_open(conn);
            // Some clients send data piggybacked on the handshake ACK.
            if (payload_len > 0 && seq == conn.rcv_nxt) {
                b->listener->on_data(conn, payload, payload_len);
                conn.rcv_nxt += static_cast<uint32_t>(payload_len);
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
            }
            return true;
        }
        case TcpState::Established: {
            if (flags & TCP_FLAG_ACK) {
                conn.snd_una = ack;
            }
            if (payload_len > 0) {
                if (seq != conn.rcv_nxt) {
                    // Out-of-order — drop. Re-ACK what we have so the
                    // peer retransmits.
                    send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
                    return true;
                }
                b->listener->on_data(conn, payload, payload_len);
                conn.rcv_nxt += static_cast<uint32_t>(payload_len);
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
            }
            if (flags & TCP_FLAG_FIN) {
                conn.rcv_nxt += 1u;
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
                b->listener->on_close(conn);
                conn.state = TcpState::CloseWait;
                // If the listener didn't call conn.close() in on_close,
                // do it now — for a server we typically have nothing
                // more to say.
                if (conn.state == TcpState::CloseWait) {
                    conn.close();  // → LastAck
                }
            }
            return true;
        }
        case TcpState::CloseWait: {
            // Peer already FINed, we're waiting for app to FIN back.
            // Drop unexpected data; let app drive via conn.close().
            return true;
        }
        case TcpState::LastAck: {
            if ((flags & TCP_FLAG_ACK) && ack == conn.snd_nxt) {
                reset_to_listen(*b);
            }
            return true;
        }
        case TcpState::Closed:
        default:
            return true;
    }
}

} // namespace kernel::net
