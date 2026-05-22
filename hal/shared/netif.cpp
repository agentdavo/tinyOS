// SPDX-License-Identifier: MIT OR Apache-2.0

#include "netif.hpp"

namespace kernel::net {

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

void Netif::dispatch(int if_idx, const uint8_t* data, size_t len) noexcept {
    if (!data || len < 14) return;
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

} // namespace kernel::net
