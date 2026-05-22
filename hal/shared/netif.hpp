// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

// Per-NIC RX demux. A Netif binds to one kernel::hal::net::NetworkDriverOps
// and dispatches incoming Ethernet frames to registered listeners. The
// service that "owns" the NIC (typically because its IRQ targets that
// service's worker core) drives the poll loop; additional services can
// register filtered listeners to co-exist on the same wire without
// editing the owner's handler.
//
// Why per-NIC and not global: each NIC's poll/IRQ-affinity is bound to a
// specific worker core via route_net_irq. RX work for nic[i] runs on the
// core that owns nic[i]; running listener dispatch off-thread would lose
// that affinity. The netif is just a dispatcher, not a thread.

#include "../../hal.hpp"

#include <cstddef>
#include <cstdint>

namespace kernel::net {

// Catch-all sentinel for EthListener::ethertype() — receives every frame
// regardless of EtherType. Specific value (e.g. 0x88A4 for EtherCAT,
// 0x0800 for IPv4) filters at dispatch time. The dispatcher walks
// listeners in registration order; multiple matching listeners all see
// the frame.
constexpr uint16_t ETHERTYPE_ANY = 0u;

class EthListener {
public:
    virtual ~EthListener() = default;
    // Return ETHERTYPE_ANY to see every frame, else a specific EtherType
    // (host byte order; the dispatcher byte-swaps the wire value).
    virtual uint16_t ethertype() const noexcept = 0;
    // Called for each matching frame. `data` points at the Ethernet
    // header; `len` includes the header.
    virtual void on_eth_frame(int if_idx,
                              kernel::hal::net::NetworkDriverOps& nic,
                              const uint8_t* data, size_t len) noexcept = 0;
};

// UDP listener — one per (netif, local_port). Registered listeners
// receive parsed IPv4+UDP datagrams; the Netif strips the L2/L3/L4
// headers and hands the payload back. Eth/IP/UDP header validation
// (header length, checksum, length sanity) is the Netif's job.
class UdpListener {
public:
    virtual ~UdpListener() = default;
    // Bound UDP local port (host order). 0 = listen on all ports
    // (catch-all; useful for diagnostics, otherwise leave to a real port).
    virtual uint16_t local_port() const noexcept = 0;
    virtual void on_udp(int if_idx,
                        kernel::hal::net::NetworkDriverOps& nic,
                        const uint8_t* eth_src,
                        uint32_t src_ip, uint16_t src_port,
                        uint32_t dst_ip, uint16_t dst_port,
                        const uint8_t* payload, size_t payload_len) noexcept = 0;
};

class Netif;

// Build and send a UDP/IPv4/Ethernet datagram via `netif`. Caller
// supplies the already-resolved L2 dst MAC and our src MAC + src IP
// (L3 state still lives in the owning service for now). Returns
// false if the payload is too large for the staging buffer.
//
// Max payload is bounded by the staging buffer below; we currently
// reserve 2 KB total which allows ~1958 bytes of UDP payload past
// Eth/IP/UDP overhead. Bigger payloads need IP fragmentation, which
// the kernel doesn't do yet.
bool udp_sendto(Netif& netif,
                const uint8_t* dst_mac, const uint8_t* src_mac,
                uint32_t src_ip, uint32_t dst_ip,
                uint16_t src_port, uint16_t dst_port,
                const uint8_t* payload, size_t payload_len) noexcept;

class Netif {
public:
    static constexpr size_t MAX_LISTENERS = 4;
    static constexpr size_t MAX_UDP_LISTENERS = 8;
    void bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept;
    bool register_listener(EthListener* l) noexcept;
    void unregister_listener(EthListener* l) noexcept;
    bool register_udp_listener(UdpListener* l) noexcept;
    void unregister_udp_listener(UdpListener* l) noexcept;
    // Drain up to `budget` frames from the NIC's RX queue, dispatching
    // each to matching listeners. Returns frame count.
    size_t poll(size_t budget) noexcept;
    int if_idx() const noexcept { return if_idx_; }
    kernel::hal::net::NetworkDriverOps* nic() const noexcept { return nic_; }
    // True if any UdpListener owns this local port — used by the legacy
    // L2-catch-all path in HMI to skip frames that are about to be
    // dispatched (or already were) via the UDP listener route.
    bool udp_port_claimed(uint16_t local_port) const noexcept;
private:
    static void rx_trampoline(int if_idx, const uint8_t* data, size_t len, void* ctx) noexcept;
    void dispatch(int if_idx, const uint8_t* data, size_t len) noexcept;
    void try_dispatch_udp(int if_idx, const uint8_t* data, size_t len) noexcept;
    int if_idx_ = -1;
    kernel::hal::net::NetworkDriverOps* nic_ = nullptr;
    EthListener* listeners_[MAX_LISTENERS]{};
    size_t listener_count_ = 0;
    UdpListener* udp_listeners_[MAX_UDP_LISTENERS]{};
    size_t udp_listener_count_ = 0;
};

constexpr size_t MAX_NETIFS = 4;
extern Netif g_netifs[MAX_NETIFS];

// Bind a Netif to a NIC index. Returns the Netif on success, nullptr if
// idx is out of range. Safe to call multiple times — re-binds the slot.
Netif* netif_bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept;
// Retrieve a previously-bound netif. Returns nullptr if not bound.
Netif* netif_get(int if_idx) noexcept;

} // namespace kernel::net
