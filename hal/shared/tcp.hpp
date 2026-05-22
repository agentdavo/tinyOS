// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// Minimal TCP server (listener only). Scope:
//   * Multiple concurrent connections per listener (cap MAX_CONNS
//     across the whole TCP module).
//   * 3-way handshake (LISTEN → SYN_RECEIVED → ESTABLISHED).
//   * Receive data via TcpListener::on_data.
//   * Send data via TcpConnection::send().
//   * Passive close (peer FIN → ESTABLISHED → CLOSE_WAIT → LAST_ACK →
//     CLOSED). No initiator FIN; the listener returns to LISTEN.
//   * Single-segment retransmit per connection with simple RTO.
//     Doubles on each retry, gives up after MAX_RETRIES.
//   * No congestion control, no window scaling, no SACK.
//   * Receive window = fixed 4 KB. Send buffer in caller's hands.

#include "netif.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace kernel::net {

// TCP state machine, RFC 793 §3.2 — listener-only subset.
enum class TcpState : uint8_t {
    Closed,
    Listen,
    SynReceived,
    Established,
    CloseWait,
    LastAck,
};

class TcpListener;

struct TcpConnection {
    static constexpr size_t RX_BUF_BYTES = 4096;
    static constexpr size_t RETX_BUF_BYTES = 1024;  // ~ MSS cap; keeps per-conn footprint small
    static constexpr uint64_t RTO_INITIAL_US = 500000;   // 500 ms
    static constexpr uint64_t RTO_MAX_US     = 4000000;  // 4 s ceiling
    static constexpr uint32_t MAX_RETRIES    = 5;        // ~ 8 s with backoff before giving up

    TcpListener* listener = nullptr;
    uint8_t      peer_mac[6]{};
    uint32_t     peer_ip = 0;
    uint16_t     peer_port = 0;
    uint16_t     local_port = 0;
    uint32_t     local_ip = 0;
    TcpState     state = TcpState::Closed;
    uint32_t     snd_nxt = 0;   // next sequence number we'll send
    uint32_t     snd_una = 0;   // oldest unACKed sequence
    uint32_t     rcv_nxt = 0;   // next sequence we expect
    uint16_t     rcv_wnd = RX_BUF_BYTES;

    // Single-segment retransmit slot. retx_active is true between send
    // and ACK; the segment is re-emitted from retx_buf on RTO expiry.
    // Cleared by tcp_dispatch when the peer's ACK advances past
    // retx_seq + retx_len.
    bool      retx_active = false;
    uint32_t  retx_seq = 0;
    uint16_t  retx_len = 0;
    uint16_t  retx_flags = 0;
    uint64_t  retx_sent_us = 0;
    uint64_t  retx_rto_us = RTO_INITIAL_US;
    uint32_t  retx_retries = 0;
    uint8_t   retx_buf[RETX_BUF_BYTES]{};

    bool send(const uint8_t* data, size_t len) noexcept;
    void close() noexcept;
};

class TcpListener {
public:
    virtual ~TcpListener() = default;
    virtual uint16_t local_port() const noexcept = 0;
    // Called once when the 3-way handshake completes. `conn` is the
    // listener's single connection slot — store the pointer if the
    // listener needs to drive sends from elsewhere.
    virtual void on_open(TcpConnection& conn) noexcept = 0;
    // Called for each in-order data segment received on the
    // connection. Out-of-order segments are dropped (no RX reassembly).
    virtual void on_data(TcpConnection& conn,
                         const uint8_t* data, size_t len) noexcept = 0;
    // Called when the peer FINs us, just before the connection
    // transitions to CloseWait. The listener may still call
    // conn.send() before returning; it should then call conn.close()
    // to FIN back.
    virtual void on_close(TcpConnection& conn) noexcept = 0;
};

// Tied to a Netif. MAX_TCP_LISTENERS caps the number of (netif, port)
// bindings; MAX_TCP_CONNS caps simultaneous connections across all
// listeners. SYNs to a full table get RST.
constexpr size_t MAX_TCP_LISTENERS = 4;
constexpr size_t MAX_TCP_CONNS = 16;

bool tcp_bind(Netif& netif, TcpListener* l) noexcept;
void tcp_unbind(Netif& netif, TcpListener* l) noexcept;
bool tcp_dispatch(Netif& netif,
                  const uint8_t* src_mac,
                  uint32_t src_ip, uint32_t dst_ip,
                  const uint8_t* tcp_segment, size_t tcp_len) noexcept;
bool tcp_port_claimed(Netif& netif, uint16_t local_port) noexcept;
// Periodic timer hook — walk the connection table and retransmit any
// segment whose RTO has expired. Called from Netif::poll().
void tcp_tick(uint64_t now_us) noexcept;

} // namespace kernel::net
