// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// Minimal TCP server (listener only). Goal: replace the Python
// WebSocket bridge with a kernel-side socket. Scope:
//   * One ESTABLISHED connection per listener at a time. Subsequent
//     SYNs are RSTed.
//   * 3-way handshake (LISTEN → SYN_RECEIVED → ESTABLISHED).
//   * Receive data via TcpListener::on_data.
//   * Send data via TcpConnection::send().
//   * Passive close (peer FIN → ESTABLISHED → CLOSE_WAIT → LAST_ACK →
//     CLOSED). No initiator FIN; the listener returns to LISTEN.
//   * No retransmit, no congestion control, no window scaling.
//     Per-segment send is one-shot; if the network loses our packet,
//     the peer's TCP will eventually drop the connection. Acceptable
//     over QEMU SLIRP / clean loopback; not a substitute for full TCP
//     on a real network.
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
    TcpListener* listener = nullptr;
    uint8_t      peer_mac[6]{};
    uint32_t     peer_ip = 0;
    uint16_t     peer_port = 0;
    uint16_t     local_port = 0;
    uint32_t     local_ip = 0;
    TcpState     state = TcpState::Closed;
    uint32_t     snd_nxt = 0;   // next sequence number we'll send
    uint32_t     snd_una = 0;   // oldest unACKed sequence (== snd_nxt for no-retransmit build)
    uint32_t     rcv_nxt = 0;   // next sequence we expect
    uint16_t     rcv_wnd = RX_BUF_BYTES;
    // Send a TCP payload on this connection. Builds the TCP segment
    // around `data` and pushes via the bound Netif. Returns false if
    // the connection isn't in ESTABLISHED state or the segment can't
    // fit in the staging buffer. snd_nxt is advanced by `len`.
    bool send(const uint8_t* data, size_t len) noexcept;
    // Initiate a passive close from our side (sends FIN, transitions
    // ESTABLISHED → CloseWait → LastAck once peer ACKs). Useful when
    // the listener has finished serving (e.g. closing a WebSocket).
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

// Tied to a Netif. Each Netif holds at most MAX_TCP_LISTENERS bound
// ports; each listener gets one connection slot. SYNs to an unbound
// port are RSTed.
constexpr size_t MAX_TCP_LISTENERS = 4;

bool tcp_bind(Netif& netif, TcpListener* l) noexcept;
void tcp_unbind(Netif& netif, TcpListener* l) noexcept;
// Called by Netif::dispatch when an IPv4 TCP frame arrives. Returns
// true if the frame was consumed (TCP listener handled or replied with
// RST). Caller should still let eth-frame listeners see the frame.
bool tcp_dispatch(Netif& netif,
                  const uint8_t* src_mac,
                  uint32_t src_ip, uint32_t dst_ip,
                  const uint8_t* tcp_segment, size_t tcp_len) noexcept;
// Returns true if `local_port` has a TcpListener bound. Used by
// Netif::dispatch to filter the catch-all eth path.
bool tcp_port_claimed(Netif& netif, uint16_t local_port) noexcept;

} // namespace kernel::net
