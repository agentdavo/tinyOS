// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file net.cpp
 * @brief Networking subsystem implementation for miniOS v1.7.
 * @details
 * Implements a lightweight networking stack for UDP/TCP sockets and low-latency audio streaming
 * over virtio-net. Enhanced in v1.7 with configurable packet priorities (0-15), jitter buffer for
 * audio streaming, packet timestamping, improved error handling, clearer diagnostics, and modern
 * C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for packet buffers
 * - std::atomic for thread-safe state
 * - std::string_view for string handling
 *
 * @version 1.7
 * @see net.hpp, miniOS.hpp, util.hpp, audio.hpp, dsp.hpp
 */

#include "net.hpp"
#include "util.hpp"
#include <cstring>
#include <algorithm>
#include <span>

namespace net {

NetManager g_net_manager;

bool from_string(std::string_view ip_str, IPv4Addr& out) noexcept {
    uint32_t val;
    if (!kernel::util::ipv4_to_uint32(ip_str, val)) return false;
    out.addr = val;
    return true;
}

void to_string(char* buf, size_t bufsz, IPv4Addr addr) noexcept {
    kernel::util::uint32_to_ipv4_str(addr.addr, std::span<char>(buf, bufsz));
}

bool NetManager::is_valid_socket(int socket_idx) const {
    return socket_idx >= 0 && socket_idx < static_cast<int>(MAX_SOCKETS) && sockets_[socket_idx].active;
}

bool NetManager::init(kernel::hal::net::NetworkDriverOps* ops) {
    if (initialized_.load(std::memory_order_relaxed) || !ops) return false;
    net_ops_ = ops;
    for (auto& socket : sockets_) {
        socket = Socket{};
    }
    initialized_.store(true, std::memory_order_relaxed);
    return true;
}

int NetManager::create_udp_socket(IPv4Addr local_ip, uint16_t local_port) {
    if (!initialized_.load(std::memory_order_relaxed)) return -1;
    for (size_t i = 0; i < MAX_SOCKETS; ++i) {
        if (!sockets_[i].active) {
            sockets_[i].active = true;
            sockets_[i].is_tcp = false;
            sockets_[i].local_ip = local_ip;
            sockets_[i].local_port = local_port;
            sockets_[i].priority = 0; // Default priority
            sockets_[i].jitter_buffer_size = 0; // No jitter buffer by default
            return static_cast<int>(i);
        }
    }
    return -1;
}

int NetManager::create_tcp_socket(IPv4Addr local_ip, uint16_t local_port) {
    if (!initialized_.load(std::memory_order_relaxed)) return -1;
    for (size_t i = 0; i < MAX_SOCKETS; ++i) {
        if (!sockets_[i].active) {
            sockets_[i].active = true;
            sockets_[i].is_tcp = true;
            sockets_[i].local_ip = local_ip;
            sockets_[i].local_port = local_port;
            sockets_[i].priority = 0;
            sockets_[i].jitter_buffer_size = 0;
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool NetManager::send(int socket_idx, const Packet& packet, bool is_audio) {
    if (!is_valid_socket(socket_idx) || !net_ops_ || packet.data_len == 0 || packet.data_len > MAX_PAYLOAD) {
        return false;
    }
    if (is_audio && packet.audio_channels == 0) return false;

    Packet send_packet = packet;
    if (is_audio && send_packet.timestamp_us == 0) {
        send_packet.timestamp_us = kernel::g_platform ? kernel::g_platform->get_timer_ops()->get_system_time_us() : 0;
    }
    send_packet.priority = kernel::util::min(sockets_[socket_idx].priority, static_cast<uint8_t>(15));

    std::array<uint8_t, MAX_PAYLOAD + 64> buffer; // Extra space for headers
    size_t offset = 0;
    // Simplified: Assume HAL handles headers
    util::memcpy(buffer.data() + offset, send_packet.data.data(), send_packet.data_len);
    offset += send_packet.data_len;

    return net_ops_->send_packet(std::span<const uint8_t>(buffer.data(), offset));
}

bool NetManager::receive(int socket_idx, Packet& packet, bool is_audio) {
    if (!is_valid_socket(socket_idx) || !net_ops_) return false;

    auto& socket = sockets_[socket_idx];
    if (is_audio && socket.jitter_buffer_size > 0) {
        // Try to return a packet from jitter buffer
        if (!socket.jitter_buffer.empty()) {
            packet = socket.jitter_buffer[socket.jitter_buffer_head];
            socket.jitter_buffer_head = (socket.jitter_buffer_head + 1) % socket.jitter_buffer.size();
            socket.jitter_buffer.erase(socket.jitter_buffer.begin() + socket.jitter_buffer_head);
            return true;
        }
    }

    std::array<uint8_t, MAX_PAYLOAD + 64> buffer;
    size_t len = 0;
    if (!net_ops_->receive_packet(std::span<uint8_t>(buffer.data(), buffer.size()), len) || len == 0) {
        return false;
    }

    // Simplified: Assume HAL strips headers
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
    packet.data_len = len;
    util::memcpy(packet.data.data(), buffer.data(), len);

    if (is_audio) {
        if (packet.audio_channels == 0) return false; // Not an audio packet
        if (socket.jitter_buffer_size > 0 && socket.jitter_buffer.size() < socket.jitter_buffer_size) {
            socket.jitter_buffer.push_back(packet);
            return false; // Buffer not full yet
        }
    }

    return true;
}

void NetManager::list_sockets(kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return;
    bool found = false;
    uart_ops->puts("Active sockets:\n");
    for (size_t i = 0; i < MAX_SOCKETS; ++i) {
        if (sockets_[i].active) {
            found = true;
            char buf[32];
            uart_ops->puts("  Socket ");
            kernel::util::k_snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(i));
            uart_ops->puts(buf);
            uart_ops->puts(": ");
            uart_ops->puts(sockets_[i].is_tcp ? "TCP" : "UDP");
            uart_ops->puts(", IP=");
            to_string(buf, sizeof(buf), sockets_[i].local_ip);
            uart_ops->puts(buf);
            uart_ops->puts(", Port=");
            kernel::util::k_snprintf(buf, sizeof(buf), "%u", sockets_[i].local_port);
            uart_ops->puts(buf);
            uart_ops->puts(", Priority=");
            kernel::util::k_snprintf(buf, sizeof(buf), "%u", sockets_[i].priority);
            uart_ops->puts(buf);
            if (sockets_[i].jitter_buffer_size > 0) {
                uart_ops->puts(", JitterBuffer=");
                kernel::util::k_snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(sockets_[i].jitter_buffer_size));
                uart_ops->puts(buf);
                uart_ops->puts(" packets");
            }
            uart_ops->puts("\n");
        }
    }
    if (!found) {
        uart_ops->puts("  (none)\n");
    }
}

bool NetManager::set_socket_priority(int socket_idx, uint8_t priority) {
    if (!is_valid_socket(socket_idx) || priority > 15) return false;
    sockets_[socket_idx].priority = priority;
    return true;
}

bool NetManager::set_jitter_buffer_size(int socket_idx, size_t size) {
    if (!is_valid_socket(socket_idx) || size > MAX_JITTER_BUFFER) return false;
    auto& socket = sockets_[socket_idx];
    socket.jitter_buffer_size = size;
    socket.jitter_buffer.clear();
    socket.jitter_buffer_head = 0;
    if (size > 0) {
        socket.jitter_buffer.reserve(size);
    }
    return true;
}

} // namespace net