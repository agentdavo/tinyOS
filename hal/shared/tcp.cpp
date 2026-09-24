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
#include "inet.hpp"
#include "../../miniOS.hpp"

#include <cstring>

namespace kernel::net {

namespace {

using inet::EthernetHeader;
using inet::IPv4Header;
using inet::UdpHeader;
using inet::bswap16;
using inet::bswap32;
using inet::ETHERTYPE_IPV4;
using inet::IPPROTO_TCP;
using inet::IPPROTO_UDP;
// --- ISN generation (RFC 6528) --------------------------------------------
// ISN = H(4-tuple, secret) + (microseconds / 4). The old fixed 0x12345678
// let any host on the LAN forge in-window segments into someone else's
// connection (e.g. push a whole UI over a spoofed WebSocket). QEMU virt has
// no RNG device we rely on, so the secret is a pool stirred with the arrival
// time of every received segment — timing jitter the attacker can't see.
uint64_t g_entropy = 0x9E3779B97F4A7C15ULL;

inline uint64_t mix64(uint64_t z) noexcept {   // splitmix64 finaliser
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

inline uint64_t now_ns_raw() noexcept {
    if (kernel::g_platform && kernel::g_platform->get_timer_ops()) {
        return kernel::g_platform->get_timer_ops()->get_system_time_ns();
    }
    return 0;
}

inline void stir(uint64_t v) noexcept {
    g_entropy = mix64(g_entropy ^ v ^ (now_ns_raw() << 17));
}

uint32_t make_isn(uint32_t lip, uint16_t lport, uint32_t rip, uint16_t rport) noexcept {
    stir(reinterpret_cast<uintptr_t>(&lip));
    const uint64_t h = mix64(g_entropy ^ ((static_cast<uint64_t>(lip) << 32) | rip) ^
                             (static_cast<uint64_t>(lport) << 48) ^
                             (static_cast<uint64_t>(rport) << 32));
    return static_cast<uint32_t>(h) + static_cast<uint32_t>(now_ns_raw() / 4000u);
}

// Wrap-safe sequence comparisons.
inline bool seq_lt(uint32_t a, uint32_t b) noexcept { return static_cast<int32_t>(a - b) < 0; }
inline bool seq_le(uint32_t a, uint32_t b) noexcept { return static_cast<int32_t>(a - b) <= 0; }

// TCP flags (control bits in the data offset / flags word).
constexpr uint16_t TCP_FLAG_FIN = 0x0001;
constexpr uint16_t TCP_FLAG_SYN = 0x0002;
constexpr uint16_t TCP_FLAG_RST = 0x0004;
constexpr uint16_t TCP_FLAG_PSH = 0x0008;
constexpr uint16_t TCP_FLAG_ACK = 0x0010;

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

inline uint16_t ip_checksum(const uint8_t* data, size_t len) noexcept {
    return inet::checksum16(data, len);
}

inline uint16_t tcp_checksum(uint32_t src_ip, uint32_t dst_ip,
                             const uint8_t* segment, size_t seg_len) noexcept {
    return inet::l4_checksum(IPPROTO_TCP, src_ip, dst_ip, segment, seg_len);
}

// Two-table layout: `Binding` is (netif, listener, port). `ConnSlot`
// holds an active TCP connection that references its binding. SYNs
// allocate a fresh ConnSlot keyed by the 4-tuple. Closing a connection
// just frees the slot.
struct Binding {
    Netif*         netif = nullptr;
    TcpListener*   listener = nullptr;
};

struct ConnSlot {
    bool          in_use = false;
    Binding*      binding = nullptr;
    TcpConnection conn{};
};

constexpr size_t MAX_BINDINGS = MAX_TCP_LISTENERS * MAX_NETIFS;
Binding  g_bindings[MAX_BINDINGS];
ConnSlot g_conns[MAX_TCP_CONNS];

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

ConnSlot* find_conn(Netif& netif, uint32_t peer_ip, uint16_t peer_port,
                    uint16_t local_port) noexcept {
    for (auto& s : g_conns) {
        if (!s.in_use || !s.binding || s.binding->netif != &netif) continue;
        if (s.conn.peer_ip == peer_ip && s.conn.peer_port == peer_port &&
            s.conn.local_port == local_port) return &s;
    }
    return nullptr;
}

ConnSlot* alloc_conn() noexcept {
    for (auto& s : g_conns) {
        if (!s.in_use) return &s;
    }
    return nullptr;
}

uint64_t now_us_or_zero() noexcept {
    if (kernel::g_platform && kernel::g_platform->get_timer_ops()) {
        return kernel::g_platform->get_timer_ops()->get_system_time_us();
    }
    return 0;
}

bool send_segment_at(TcpConnection& conn,
                     uint32_t seq,
                     uint16_t flags,
                     const uint8_t* payload, size_t payload_len) noexcept {
    constexpr size_t FRAME_CAP = 2048;
    constexpr size_t HDRS = sizeof(EthernetHeader) + sizeof(IPv4Header) + sizeof(TcpHeader);
    if (payload_len > FRAME_CAP - HDRS) return false;
    auto* nif = conn.netif;
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
    tcp->seq_be      = bswap32(seq);
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

bool send_segment(TcpConnection& conn,
                  uint16_t flags,
                  const uint8_t* payload, size_t payload_len) noexcept {
    return send_segment_at(conn, conn.snd_nxt, flags, payload, payload_len);
}

void release_conn(ConnSlot& slot) noexcept {
    slot.in_use = false;
    slot.binding = nullptr;
    slot.conn = TcpConnection{};
}

// Tear down a connection the listener already knows about. Every exit
// path (RST, retransmit give-up, idle reap, eviction, final ACK) goes
// through here so the listener always hears on_close — the WebSocket
// server used to keep a pointer to a released slot after a RST and then
// refuse every later client.
void drop_conn(ConnSlot& slot, bool send_rst) noexcept {
    auto& c = slot.conn;
    if (send_rst) send_segment(c, TCP_FLAG_RST | TCP_FLAG_ACK, nullptr, 0);
    if (c.opened && slot.binding && slot.binding->listener) {
        c.opened = false;   // on_close may call close(); don't recurse
        slot.binding->listener->on_close(c);
    }
    release_conn(slot);
}

ConnSlot* slot_of(TcpConnection& conn) noexcept {
    for (auto& s : g_conns) {
        if (&s.conn == &conn) return &s;
    }
    return nullptr;
}

void arm_retx(TcpConnection& c, uint32_t seq, uint16_t len, uint16_t flags) noexcept {
    c.retx_active  = true;
    c.retx_seq     = seq;
    c.retx_len     = len;
    c.retx_flags   = flags;
    c.retx_sent_us = now_us_or_zero();
    c.retx_rto_us  = TcpConnection::RTO_INITIAL_US;
    c.retx_retries = 0;
}

void send_fin(TcpConnection& c) noexcept {
    send_segment(c, TCP_FLAG_ACK | TCP_FLAG_FIN, nullptr, 0);
    // FIN consumes one sequence number — track it so a lost FIN is resent.
    arm_retx(c, c.snd_nxt, 1, TCP_FLAG_ACK | TCP_FLAG_FIN);
    c.snd_nxt += 1;
    c.fin_pending = false;
}

} // namespace

bool TcpConnection::send(const uint8_t* data, size_t len) noexcept {
    if (state != TcpState::Established || fin_pending) return false;
    if (len == 0) return true;
    if (retx_active) return false;  // single-segment in-flight cap
    if (len > RETX_BUF_BYTES) return false;
    const uint16_t flags = TCP_FLAG_ACK | TCP_FLAG_PSH;
    if (!send_segment(*this, flags, data, len)) return false;
    arm_retx(*this, snd_nxt, static_cast<uint16_t>(len), flags);
    if (data) std::memcpy(retx_buf, data, len);
    snd_nxt += static_cast<uint32_t>(len);
    return true;
}

void TcpConnection::close() noexcept {
    if (state == TcpState::Established || state == TcpState::CloseWait) {
        state = TcpState::LastAck;
        // One segment in flight: with data still unACKed the FIN waits
        // for that ACK (tcp_dispatch sends it). Sending it now left the
        // FIN untracked, and a lost FIN parked the slot in LastAck forever.
        if (retx_active) {
            fin_pending = true;
        } else {
            send_fin(*this);
        }
    } else if (state == TcpState::SynReceived) {
        // Refused before the handshake finished: just reset.
        if (ConnSlot* s = slot_of(*this)) drop_conn(*s, true);
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
    return true;
}

void tcp_unbind(Netif& netif, TcpListener* l) noexcept {
    // Tear down all live conns on this listener first.
    for (auto& s : g_conns) {
        if (s.in_use && s.binding && s.binding->netif == &netif &&
            s.binding->listener == l) {
            drop_conn(s, true);
        }
    }
    if (auto* b = find_binding_for_listener(netif, l); b) {
        *b = Binding{};
    }
}

bool tcp_port_claimed(Netif& netif, uint16_t local_port) noexcept {
    return find_binding(netif, local_port) != nullptr;
}

void tcp_tick(uint64_t now_us) noexcept {
    for (auto& s : g_conns) {
        if (!s.in_use) continue;
        auto& c = s.conn;
        if (!c.retx_active) {
            // Idle reaper: a peer that went silent (unplugged laptop,
            // half-open socket) no longer holds its slot forever.
            if (c.last_rx_us != 0 && now_us > c.last_rx_us + TcpConnection::IDLE_TIMEOUT_US) {
                drop_conn(s, true);
            }
            continue;
        }
        if (now_us < c.retx_sent_us + c.retx_rto_us) continue;
        if (c.retx_retries >= TcpConnection::MAX_RETRIES) {
            // Give up — peer is gone.
            drop_conn(s, true);
            continue;
        }
        // Re-send the same segment at the ORIGINAL seq (snd_nxt has
        // advanced past it). SYN/FIN are control segments (phantom seq
        // byte, no payload); data carries retx_buf.
        if (c.retx_flags & (TCP_FLAG_SYN | TCP_FLAG_FIN)) {
            send_segment_at(c, c.retx_seq, c.retx_flags, nullptr, 0);
        } else {
            send_segment_at(c, c.retx_seq, c.retx_flags, c.retx_buf, c.retx_len);
        }
        c.retx_sent_us = now_us;
        c.retx_retries += 1;
        c.retx_rto_us *= 2;
        if (c.retx_rto_us > TcpConnection::RTO_MAX_US) {
            c.retx_rto_us = TcpConnection::RTO_MAX_US;
        }
    }
}

bool tcp_dispatch(Netif& netif,
                  const uint8_t* src_mac,
                  uint32_t src_ip, uint32_t dst_ip,
                  const uint8_t* tcp_segment, size_t tcp_len) noexcept {
    if (tcp_len < sizeof(TcpHeader)) return false;
    // Only segments addressed to this interface, with a valid checksum.
    // Before, any destination IP (broadcast included) and any checksum
    // was accepted, which made blind spoofing much easier.
    const uint32_t our_ip = netif.local_ip();
    if (our_ip == 0 || dst_ip != our_ip) return true;
    if (tcp_checksum(src_ip, dst_ip, tcp_segment, tcp_len) != 0) return true;
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

    const uint64_t now_us = now_us_or_zero();
    stir((static_cast<uint64_t>(seq) << 32) ^ ack ^ src_port);

    // Existing connection? Match by 4-tuple.
    ConnSlot* slot = find_conn(netif, src_ip, src_port, dst_port);

    if (!slot) {
        // No conn for this tuple. Must be a SYN to LISTEN or stray
        // segment to nowhere — the latter gets RST.
        Binding* b = find_binding(netif, dst_port);
        if (!b || !(flags & TCP_FLAG_SYN)) {
            // Send RST from ephemeral conn state. It carries the receiving
            // netif itself, so this also works for an unbound port (the
            // old path went through a ConnSlot whose null binding made the
            // send bail out, so no RST was ever emitted).
            TcpConnection tmp;
            tmp.netif = &netif;
            tmp.local_ip = dst_ip; tmp.peer_ip = src_ip;
            tmp.local_port = dst_port; tmp.peer_port = src_port;
            for (size_t i = 0; i < 6; ++i) tmp.peer_mac[i] = src_mac[i];
            tmp.snd_nxt = (flags & TCP_FLAG_ACK) ? ack : 0;
            tmp.rcv_nxt = seq + ((flags & (TCP_FLAG_SYN | TCP_FLAG_FIN)) ? 1u : 0u) + payload_len;
            if (!(flags & TCP_FLAG_RST)) {  // never answer a RST with a RST
                send_segment(tmp, TCP_FLAG_RST | TCP_FLAG_ACK, nullptr, 0);
            }
            return true;
        }
        if (flags & (TCP_FLAG_RST | TCP_FLAG_ACK)) return true;  // not a plain SYN
        // SYN → new connection.
        ConnSlot* ns = alloc_conn();
        if (!ns) {
            // Table full: evict the longest-idle established connection
            // if it has been silent long enough, else refuse with RST
            // (the old code dropped silently, so the client hung until
            // its own SYN timeout).
            ConnSlot* victim = nullptr;
            for (auto& s : g_conns) {
                if (!s.in_use || s.conn.state != TcpState::Established || s.conn.retx_active) continue;
                if (now_us < s.conn.last_rx_us + TcpConnection::EVICT_IDLE_US) continue;
                if (!victim || s.conn.last_rx_us < victim->conn.last_rx_us) victim = &s;
            }
            if (victim) {
                drop_conn(*victim, true);
                ns = alloc_conn();
            }
            if (!ns) {
                TcpConnection tmp;
                tmp.netif = &netif;
                tmp.local_ip = dst_ip; tmp.peer_ip = src_ip;
                tmp.local_port = dst_port; tmp.peer_port = src_port;
                for (size_t i = 0; i < 6; ++i) tmp.peer_mac[i] = src_mac[i];
                tmp.rcv_nxt = seq + 1u;
                send_segment(tmp, TCP_FLAG_RST | TCP_FLAG_ACK, nullptr, 0);
                return true;
            }
        }
        ns->in_use  = true;
        ns->binding = b;
        ns->conn = TcpConnection{};
        ns->conn.listener   = b->listener;
        ns->conn.netif      = &netif;
        ns->conn.local_ip   = dst_ip;
        ns->conn.local_port = dst_port;
        ns->conn.peer_ip    = src_ip;
        ns->conn.peer_port  = src_port;
        for (size_t i = 0; i < 6; ++i) ns->conn.peer_mac[i] = src_mac[i];
        const uint32_t isn = make_isn(dst_ip, dst_port, src_ip, src_port);
        ns->conn.rcv_nxt = seq + 1u;
        ns->conn.snd_nxt = isn;
        ns->conn.snd_una = isn;
        ns->conn.state   = TcpState::SynReceived;
        ns->conn.last_rx_us = now_us;
        // Retransmit-track our SYN+ACK in case the third-party ACK is lost.
        send_segment(ns->conn, TCP_FLAG_SYN | TCP_FLAG_ACK, nullptr, 0);
        arm_retx(ns->conn, ns->conn.snd_nxt, 1, TCP_FLAG_SYN | TCP_FLAG_ACK);  // SYN = 1 seq
        ns->conn.snd_nxt += 1u;
        return true;
    }
    TcpConnection& conn = slot->conn;

    // RFC 5961 §3: a RST resets only if its sequence number is exactly
    // rcv_nxt; one merely inside the window draws a challenge ACK. Before,
    // any RST with a matching 4-tuple killed the connection.
    if (flags & TCP_FLAG_RST) {
        if (seq == conn.rcv_nxt) {
            drop_conn(*slot, false);
        } else if (seq_lt(conn.rcv_nxt, seq) && seq_lt(seq, conn.rcv_nxt + conn.rcv_wnd)) {
            send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
        }
        return true;
    }
    // RFC 5961 §4: a SYN on an existing connection never resets it; answer
    // with a challenge ACK (a retransmitted SYN in SynReceived re-sends our
    // SYN+ACK through the retransmit timer).
    if (flags & TCP_FLAG_SYN) {
        if (conn.state != TcpState::SynReceived) send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
        return true;
    }
    // RFC 5961 §5: the ACK must cover only data we actually sent.
    if (!(flags & TCP_FLAG_ACK)) return true;
    if (seq_lt(conn.snd_nxt, ack) || seq_lt(ack, conn.snd_una)) {
        send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
        return true;
    }
    // Segment must start inside the receive window.
    if (seq_lt(seq, conn.rcv_nxt - (payload_len ? 0u : 1u)) ||
        !seq_lt(seq, conn.rcv_nxt + conn.rcv_wnd)) {
        send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
        return true;
    }
    conn.last_rx_us = now_us;

    // ACK-driven retx clearing — applies to any state.
    if (conn.retx_active && seq_le(conn.retx_seq + conn.retx_len, ack)) {
        conn.retx_active = false;
        conn.snd_una = ack;
        if (conn.fin_pending) send_fin(conn);
    }

    switch (conn.state) {
        case TcpState::SynReceived: {
            if (!(flags & TCP_FLAG_ACK)) return true;
            if (ack != conn.snd_nxt) {
                drop_conn(*slot, true);
                return true;
            }
            conn.state = TcpState::Established;
            conn.opened = true;
            slot->binding->listener->on_open(conn);
            if (!slot->in_use) return true;   // listener refused (reset)
            if (payload_len > 0 && seq == conn.rcv_nxt) {
                slot->binding->listener->on_data(conn, payload, payload_len);
                conn.rcv_nxt += static_cast<uint32_t>(payload_len);
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
            }
            return true;
        }
        case TcpState::Established: {
            if (payload_len > 0) {
                if (seq != conn.rcv_nxt) {
                    send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
                    return true;
                }
                slot->binding->listener->on_data(conn, payload, payload_len);
                conn.rcv_nxt += static_cast<uint32_t>(payload_len);
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
            }
            // A FIN counts only when it is next in sequence (after this
            // segment's data); an out-of-order FIN used to close the
            // connection regardless of its sequence number.
            if ((flags & TCP_FLAG_FIN) && (payload_len > 0 || seq == conn.rcv_nxt)) {
                conn.rcv_nxt += 1u;
                send_segment(conn, TCP_FLAG_ACK, nullptr, 0);
                conn.opened = false;
                slot->binding->listener->on_close(conn);
                if (conn.state == TcpState::Established) {
                    conn.state = TcpState::CloseWait;
                    conn.close();  // → LastAck (sends FIN, or queues it)
                }
            }
            return true;
        }
        case TcpState::CloseWait:
            return true;
        case TcpState::LastAck: {
            if (!conn.fin_pending && !conn.retx_active && ack == conn.snd_nxt) {
                drop_conn(*slot, false);
            }
            return true;
        }
        case TcpState::Listen:
        case TcpState::Closed:
        default:
            return true;
    }
}

} // namespace kernel::net
