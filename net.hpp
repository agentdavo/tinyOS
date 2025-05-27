// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file net.hpp
 * @brief Networking subsystem header for miniOS v1.7.
 * @details
 * Defines a lightweight networking stack supporting UDP/TCP sockets and low-latency audio streaming
 * (RTP-like protocol) over virtio-net via HAL. Enhanced in v1.7 with configurable packet priorities,
 * jitter buffer for audio, timestamping, improved error handling, clearer documentation, and modern
 * C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for packet buffers
 * - std::atomic for thread-safe state
 * - std::string_view for string handling
 *
 * @version 1.7
 * @see net.cpp, miniOS.hpp, util.hpp, audio.hpp, dsp.hpp
 */

#ifndef NET_HPP
#define NET_HPP

#include "miniOS.hpp"
#include <span>
#include <atomic>
#include <array>
#include <string_view>
#include <vector>
#include <optional>

namespace net {

constexpr size_t MAX_SOCKETS = 8;
constexpr size_t MAX_PAYLOAD = 1472; // Max UDP payload (MTU 1500 - headers)
constexpr size_t MAX_JITTER_BUFFER = 8; // Max jitter buffer packets

/**
 * @brief IPv4 address structure.
 */
struct IPv4Addr {
    uint32_t addr; ///< Address in network byte order
};

/**
 * @brief Network packet structure.
 */
struct Packet {
    IPv4Addr dst_ip; ///< Destination IP address
    uint16_t dst_port; ///< Destination port
    uint64_t timestamp_us; ///< Packet timestamp (microseconds, for audio sync)
    std::array<uint8_t, MAX_PAYLOAD> data; ///< Packet data
    size_t data_len = 0; ///< Length of data
    uint8_t priority = 0; ///< Priority (0-15, 15 highest)
    uint8_t audio_channels = 0; ///< Number of audio channels (0 for non-audio)
};

/**
 * @brief Networking subsystem class.
 */
class NetManager {
public:
    NetManager() : initialized_(false) {}

    /**
     * @brief Initializes the networking subsystem.
     * @param ops Network driver operations from HAL
     * @return True if initialized, false otherwise
     */
    bool init(kernel::hal::net::NetworkDriverOps* ops);

    /**
     * @brief Creates a UDP socket.
     * @param local_ip Local IP address
     * @param local_port Local port
     * @return Socket index (>=0) or -1 on failure
     */
    int create_udp_socket(IPv4Addr local_ip, uint16_t local_port);

    /**
     * @brief Creates a TCP socket.
     * @param local_ip Local IP address
     * @param local_port Local port
     * @return Socket index (>=0) or -1 on failure
     */
    int create_tcp_socket(IPv4Addr local_ip, uint16_t local_port);

    /**
     * @brief Sends a packet on a socket.
     * @param socket_idx Socket index
     * @param packet Packet to send
     * @param is_audio True if audio packet
     * @return True if sent, false otherwise
     */
    bool send(int socket_idx, const Packet& packet, bool is_audio = false);

    /**
     * @brief Receives a packet on a socket.
     * @param socket_idx Socket index
     * @param packet Output packet
     * @param is_audio True if expecting audio packet
     * @return True if received, false otherwise
     */
    bool receive(int socket_idx, Packet& packet, bool is_audio = false);

    /**
     * @brief Lists active sockets.
     * @param uart_ops UART driver for output
     */
    void list_sockets(kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Sets socket priority.
     * @param socket_idx Socket index
     * @param priority Priority level (0-15)
     * @return True if set, false if invalid
     */
    bool set_socket_priority(int socket_idx, uint8_t priority);

    /**
     * @brief Configures jitter buffer size for audio streaming.
     * @param socket_idx Socket index
     * @param size Number of packets (0-8, 0 disables)
     * @return True if set, false if invalid
     */
    bool set_jitter_buffer_size(int socket_idx, size_t size);

private:
    struct Socket {
        bool active = false;
        bool is_tcp = false;
        IPv4Addr local_ip;
        uint16_t local_port;
        uint8_t priority = 0; ///< Socket priority (0-15)
        std::vector<Packet> jitter_buffer; ///< Audio jitter buffer
        size_t jitter_buffer_size = 0; ///< Desired jitter buffer size
        size_t jitter_buffer_head = 0; ///< Jitter buffer head index
    };
    std::array<Socket, MAX_SOCKETS> sockets_;
    std::atomic<bool> initialized_;
    kernel::hal::net::NetworkDriverOps* net_ops_ = nullptr;

    /**
     * @brief Validates a socket index.
     * @param socket_idx Socket index
     * @return True if valid and active, false otherwise
     */
    bool is_valid_socket(int socket_idx) const;
};

extern NetManager g_net_manager; ///< Global networking manager instance

} // namespace net

#endif // NET_HPP