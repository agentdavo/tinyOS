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

#include "miniOS.hpp" // For kernel::hal::net::NetworkDriverOps, kernel::hal::UARTDriverOps, kernel::Spinlock
#include <span>
#include <atomic>
#include <array>
#include <string_view>
#include <vector>
#include <optional>

// Forward declaration from miniOS.hpp if needed, but miniOS.hpp includes this.
// namespace kernel { namespace hal { namespace net { struct NetworkDriverOps; }}}
// namespace kernel { namespace hal { struct UARTDriverOps; }}

namespace net {

constexpr size_t MAX_SOCKETS = 8;
constexpr size_t MAX_PAYLOAD = 1472; // Max UDP payload (MTU 1500 - IP_HDR_LEN (20) - UDP_HDR_LEN (8))
constexpr size_t MAX_JITTER_BUFFER = 8; // Max jitter buffer packets

/**
 * @brief IPv4 address structure.
 */
struct IPv4Addr {
    uint32_t addr; ///< Address in network byte order (e.g., 0x01020304 for 1.2.3.4)

    /**
     * @brief Default constructor, initializes to 0.0.0.0.
     */
    IPv4Addr() : addr(0) {}

    /**
     * @brief Constructor from uint32_t.
     * @param val IP address as a 32-bit integer in network byte order.
     */
    explicit IPv4Addr(uint32_t val) : addr(val) {}
};

/**
 * @brief Network packet structure.
 */
struct Packet {
    IPv4Addr dst_ip; ///< Destination IP address
    uint16_t dst_port; ///< Destination port (network byte order)
    IPv4Addr src_ip;   ///< Source IP address (filled on receive)
    uint16_t src_port; ///< Source port (filled on receive, network byte order)
    uint64_t timestamp_us; ///< Packet timestamp (microseconds, e.g., for audio sync)
    std::array<uint8_t, MAX_PAYLOAD> data; ///< Packet data payload
    size_t data_len = 0; ///< Length of valid data in the payload array
    uint8_t priority = 0; ///< Priority (0-15, 15 highest, for QoS)
    uint8_t audio_channels = 0; ///< Number of audio channels (0 for non-audio data)
    // Potentially add TTL, protocol type (UDP/TCP implied by socket) if needed at this layer
};

/**
 * @brief Networking subsystem class.
 * @details Manages network interfaces, sockets (UDP/TCP), and packet transmission/reception.
 * Interacts with the hardware abstraction layer (HAL) for actual network operations.
 */
class NetManager {
public:
    /**
     * @brief Default constructor. Initializes the NetManager.
     */
    NetManager(); // Initialize members in .cpp

    /**
     * @brief Initializes the networking subsystem.
     * @param ops Pointer to the network driver operations provided by the HAL.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(kernel::hal::net::NetworkDriverOps* ops);

    /**
     * @brief Creates a UDP socket.
     * @param local_ip Local IP address to bind to. Use 0 (any) for all interfaces.
     * @param local_port Local port to bind to. Use 0 for an ephemeral port.
     * @return Socket index (non-negative integer) on success, or -1 on failure (e.g., no free sockets, port in use).
     */
    int create_udp_socket(IPv4Addr local_ip, uint16_t local_port);

    /**
     * @brief Creates a TCP socket (conceptual, full TCP stack is complex).
     * @param local_ip Local IP address to bind to.
     * @param local_port Local port to bind to.
     * @return Socket index (non-negative integer) on success, or -1 on failure.
     * @note Full TCP implementation is beyond typical lightweight RTOS scope without a library.
     */
    int create_tcp_socket(IPv4Addr local_ip, uint16_t local_port);

    /**
     * @brief Closes an active socket.
     * @param socket_idx The index of the socket to close.
     * @return True if the socket was successfully closed, false otherwise (e.g., invalid index).
     */
    bool close_socket(int socket_idx);

    /**
     * @brief Sends a packet on a specified socket.
     * @param socket_idx The index of the socket to send from.
     * @param packet The packet to send. Source IP/port in packet are ignored (taken from socket).
     * @param block If true, the call may block until the packet is sent (or timeout).
     *              If false, tries to send and returns immediately. (HAL dependent)
     * @return True if the packet was successfully queued for sending or sent, false on error.
     */
    bool send(int socket_idx, const Packet& packet, bool block = false);

    /**
     * @brief Receives a packet on a specified socket.
     * @param socket_idx The index of the socket to receive on.
     * @param[out] packet Reference to a Packet structure to fill with received data.
     * @param block If true, the call blocks until a packet is received (or timeout).
     *              If false, checks for a packet and returns immediately.
     * @return True if a packet was successfully received, false otherwise (e.g., no data, error).
     */
    bool receive(int socket_idx, Packet& packet, bool block = false);

    /**
     * @brief Lists active sockets and their details to the UART console.
     * @param uart_ops Pointer to the UART driver operations for output.
     */
    void list_sockets(kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Sets the priority for a given socket.
     * @details Higher priority packets might be favored by the underlying network driver or QoS mechanisms.
     * @param socket_idx The index of the socket.
     * @param priority The priority level (0-15, where 15 is typically highest).
     * @return True if the priority was successfully set, false if the socket is invalid or priority out of range.
     */
    bool set_socket_priority(int socket_idx, uint8_t priority);

    /**
     * @brief Configures the jitter buffer size for a specific audio streaming socket.
     * @param socket_idx The index of the (UDP) socket used for audio.
     * @param size The desired number of packets in the jitter buffer (0 to MAX_JITTER_BUFFER). 0 disables the buffer.
     * @return True if the size was successfully set, false if the socket is invalid or size out of range.
     */
    bool set_jitter_buffer_size(int socket_idx, size_t size);

    /**
     * @brief Prints network interface configuration.
     * @param if_idx Index of the network interface (e.g., 0 for the primary).
     * @param uart_ops Pointer to the UART driver operations for output.
     */
    void print_if_config(int if_idx, kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Sends ICMP echo requests (ping) to a target IP.
     * @param target_ip The IPv4 address of the target to ping.
     * @param count The number of ping requests to send.
     * @param uart_ops Pointer to the UART driver operations for displaying results.
     */
    void ping(IPv4Addr target_ip, int count, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Checks if the network manager has been successfully initialized.
     * @return True if initialized, false otherwise.
     */
    bool is_initialized() const { return initialized_.load(std::memory_order_acquire); }

    /**
     * @brief Periodic processing tick for the network manager.
     * @details Handles tasks like retransmissions (TCP), timeouts, processing incoming packets from HAL queue.
     */
    void tick();


private:
    /**
     * @brief Internal structure representing a network socket.
     */
    struct Socket {
        bool active = false;          ///< True if the socket is currently in use.
        bool is_tcp = false;          ///< True if this is a TCP socket, false for UDP.
        IPv4Addr local_ip;            ///< Local IP address this socket is bound to.
        uint16_t local_port = 0;      ///< Local port number (network byte order).
        // For TCP:
        // IPv4Addr remote_ip;        ///< Remote IP address (for connected TCP sockets).
        // uint16_t remote_port = 0;  ///< Remote port number (for connected TCP sockets).
        // enum class TcpState { CLOSED, LISTEN, SYN_SENT, ESTABLISHED, ... } tcp_state;
        // std::vector<uint8_t> rx_buffer; ///< TCP receive buffer
        // std::vector<uint8_t> tx_buffer; ///< TCP send buffer
        // uint32_t seq_num = 0;           ///< TCP sequence number
        // uint32_t ack_num = 0;           ///< TCP acknowledgment number
        
        uint8_t priority = 0;         ///< Socket priority for QoS (0-15).
        std::vector<Packet> jitter_buffer; ///< Jitter buffer for incoming audio packets (UDP).
        size_t jitter_buffer_target_size = 0; ///< Desired number of packets in jitter buffer.
        size_t jitter_buffer_write_idx = 0; ///< Next position to write in jitter buffer.
        size_t jitter_buffer_read_idx = 0;  ///< Next position to read from jitter buffer.
        size_t jitter_buffer_current_fill = 0; ///< Current number of packets in jitter buffer.
        uint64_t last_packet_timestamp_us_ = 0; ///< Timestamp of the last packet added to jitter buffer
    };

    std::array<Socket, MAX_SOCKETS> sockets_; ///< Array of available sockets.
    kernel::hal::net::NetworkDriverOps* net_ops_ = nullptr; ///< Pointer to HAL network driver operations.
    std::atomic<bool> initialized_{false}; ///< True if the NetManager is initialized.
    uint16_t next_ephemeral_port_ = 49152; ///< Start range for ephemeral ports.
    kernel::Spinlock manager_lock_; ///< Spinlock to protect access to shared NetManager state (e.g., sockets_ array).

    /**
     * @brief Validates a socket index.
     * @param socket_idx The socket index to validate.
     * @return True if the index is within bounds and the socket is active, false otherwise.
     */
    bool is_valid_socket(int socket_idx) const;

    /**
     * @brief Finds a free socket slot.
     * @return Index of a free socket, or -1 if all sockets are in use.
     */
    int find_free_socket();

    /**
     * @brief Adds a packet to a socket's jitter buffer.
     * @param sock The socket to add the packet to.
     * @param packet The packet to add.
     */
    void add_to_jitter_buffer(Socket& sock, const Packet& packet);

    /**
     * @brief Retrieves a packet from a socket's jitter buffer.
     * @param sock The socket to retrieve from.
     * @param[out] packet The packet structure to fill.
     * @return True if a packet was retrieved, false if buffer is empty or not ready.
     */
    bool get_from_jitter_buffer(Socket& sock, Packet& packet);
};

extern NetManager g_net_manager; ///< Global networking manager instance.

} // namespace net

#endif // NET_HPP