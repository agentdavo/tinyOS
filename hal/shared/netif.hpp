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

class Netif {
public:
    static constexpr size_t MAX_LISTENERS = 4;
    void bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept;
    bool register_listener(EthListener* l) noexcept;
    void unregister_listener(EthListener* l) noexcept;
    // Drain up to `budget` frames from the NIC's RX queue, dispatching
    // each to matching listeners. Returns frame count.
    size_t poll(size_t budget) noexcept;
    int if_idx() const noexcept { return if_idx_; }
    kernel::hal::net::NetworkDriverOps* nic() const noexcept { return nic_; }
private:
    static void rx_trampoline(int if_idx, const uint8_t* data, size_t len, void* ctx) noexcept;
    void dispatch(int if_idx, const uint8_t* data, size_t len) noexcept;
    int if_idx_ = -1;
    kernel::hal::net::NetworkDriverOps* nic_ = nullptr;
    EthListener* listeners_[MAX_LISTENERS]{};
    size_t listener_count_ = 0;
};

constexpr size_t MAX_NETIFS = 4;
extern Netif g_netifs[MAX_NETIFS];

// Bind a Netif to a NIC index. Returns the Netif on success, nullptr if
// idx is out of range. Safe to call multiple times — re-binds the slot.
Netif* netif_bind(int if_idx, kernel::hal::net::NetworkDriverOps* nic) noexcept;
// Retrieve a previously-bound netif. Returns nullptr if not bound.
Netif* netif_get(int if_idx) noexcept;

} // namespace kernel::net
