// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file net.hpp
 * @brief Networking subsystem header for miniOS v1.7.
 * @details
 * Legacy networking prototype.
 *
 * This header is retained only for old DSP/demo experiments. It is not the
 * controller's authoritative networking path. The real controller uses:
 * - HAL NIC drivers (`hal/shared/virtio_net.*`, `hal/shared/e1000.*`)
 * - raw EtherCAT masters on `eth1` / `eth2`
 * - the raw HMI service on `eth0`
 *
 * Do not extend this path for controller networking.
 *
 * @version 1.7
 * @see net.cpp, core.hpp, hal.hpp
 */

#ifndef NET_HPP
#define NET_HPP

#include "core.hpp"
#include "hal.hpp"
#include <string_view>

namespace net {

constexpr size_t MAX_JITTER_BUFFER = 8;

struct IPv4Addr {
    uint32_t addr;
};

bool from_string(std::string_view ip_str, IPv4Addr& out) noexcept;
void to_string(char* buf, size_t bufsz, IPv4Addr addr) noexcept;

struct Packet {
    IPv4Addr dst_ip;
    uint16_t dst_port;
    std::array<uint8_t, kernel::core::NET_MAX_PACKET_SIZE> data;
    size_t data_len = 0;
};

class NetManager {
public:
    // Legacy/demo API only. Not used by the controller runtime.
    NetManager() = default;
    bool is_initialized() const noexcept { return initialized_; }
    int create_udp_socket(IPv4Addr bind_addr, uint16_t port);
    bool send(int socket_idx, const Packet& packet, bool is_udp);
    bool close_socket(int socket_idx);
    bool set_jitter_buffer_size(int socket_idx, size_t size);
    bool set_socket_priority(int socket_idx, int priority);
    void list_sockets(kernel::hal::UARTDriverOps* uart_ops);
    void ping(IPv4Addr target_ip, int count, kernel::hal::UARTDriverOps* uart_ops);
    void print_if_config(int if_idx, kernel::hal::UARTDriverOps* uart_ops);
private:
    bool initialized_ = false;
    kernel::core::Spinlock manager_lock_;
};

extern NetManager g_net_manager;

} // namespace net

#endif // NET_HPP
