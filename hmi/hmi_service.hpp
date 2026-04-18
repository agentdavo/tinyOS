// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "core.hpp"

#include <cstddef>
#include <cstdint>

namespace kernel { namespace hal { namespace net { struct NetworkDriverOps; }}}

namespace hmi {

struct Config {
    bool dhcp_enable = true;
    uint32_t static_ip = 0x0A00020Fu;   // 10.0.2.15
    uint32_t netmask = 0xFFFFFF00u;     // 255.255.255.0
    uint32_t gateway = 0x0A000202u;     // 10.0.2.2
    uint16_t udp_port = 5000;
    uint32_t dhcp_timeout_ms = 3000;
    bool ping_selftest_enable = true;
    uint32_t ping_selftest_target = 0x08080808u; // 8.8.8.8
    uint32_t ping_selftest_delay_ms = 1500;
    uint32_t ping_selftest_timeout_ms = 3000;
};

class Service {
public:
    static constexpr uint16_t ETHERTYPE = 0x88B5;
    enum class PingResult : uint8_t {
        Ok = 0,
        Busy,
        BadAddress,
        Timeout,
        SendFailed,
    };

    bool load_tsv(const char* buf, size_t len) noexcept;
    void configure(uint8_t nic_idx) noexcept { nic_idx_ = nic_idx; }
    uint8_t nic_idx() const noexcept { return nic_idx_; }
    uint64_t rx_requests() const noexcept { return rx_requests_.load(std::memory_order_relaxed); }
    uint64_t tx_responses() const noexcept { return tx_responses_.load(std::memory_order_relaxed); }
    uint32_t last_opcode() const noexcept { return last_opcode_.load(std::memory_order_relaxed); }
    uint32_t local_ip() const noexcept { return local_ip_.load(std::memory_order_relaxed); }
    uint32_t netmask() const noexcept { return netmask_.load(std::memory_order_relaxed); }
    uint32_t gateway() const noexcept { return gateway_.load(std::memory_order_relaxed); }
    bool dhcp_bound() const noexcept { return dhcp_bound_.load(std::memory_order_relaxed); }
    uint16_t udp_port() const noexcept { return udp_port_; }
    uint32_t last_ping_target() const noexcept { return last_ping_target_.load(std::memory_order_relaxed); }
    uint32_t last_ping_rtt_ms() const noexcept { return last_ping_rtt_ms_.load(std::memory_order_relaxed); }
    PingResult last_ping_result() const noexcept { return static_cast<PingResult>(last_ping_result_.load(std::memory_order_relaxed)); }
    PingResult ping_ipv4(uint32_t dst_ip, uint32_t timeout_ms, uint32_t& rtt_ms) noexcept;
    PingResult ping_ipv4_str(const char* dst_ip, uint32_t timeout_ms, uint32_t& rtt_ms) noexcept;

    static void thread_entry(void* arg);

private:
    struct Context {
        Service* self;
        kernel::hal::net::NetworkDriverOps* nic;
    };

    static void rx_trampoline(int if_idx, const uint8_t* data, size_t len, void* ctx) noexcept;
    void handle_eth_frame(kernel::hal::net::NetworkDriverOps& nic,
                          const uint8_t* data, size_t len) noexcept;
    void handle_raw_hmi(kernel::hal::net::NetworkDriverOps& nic,
                        const uint8_t* src_mac,
                        const uint8_t* data, size_t len) noexcept;
    void handle_ipv4(kernel::hal::net::NetworkDriverOps& nic,
                     const uint8_t* eth_src,
                     const uint8_t* data, size_t len) noexcept;
    void handle_icmp(uint32_t src_ip,
                     const uint8_t* payload, size_t payload_len) noexcept;
    void handle_udp(kernel::hal::net::NetworkDriverOps& nic,
                    const uint8_t* eth_src,
                    uint32_t src_ip, uint16_t src_port,
                    uint32_t dst_ip, uint16_t dst_port,
                    const uint8_t* payload, size_t payload_len) noexcept;
    void handle_dhcp(kernel::hal::net::NetworkDriverOps& nic,
                     uint32_t src_ip,
                     const uint8_t* payload, size_t payload_len) noexcept;
    void send_raw_response(kernel::hal::net::NetworkDriverOps& nic,
                           const uint8_t* dst_mac,
                           uint8_t opcode, uint8_t status,
                           const uint8_t* payload, size_t payload_len) noexcept;
    void send_udp_hmi_response(kernel::hal::net::NetworkDriverOps& nic,
                               const uint8_t* dst_mac, uint32_t dst_ip, uint16_t dst_port,
                               uint8_t opcode, uint8_t status,
                               const uint8_t* payload, size_t payload_len) noexcept;
    void send_ipv4_packet(kernel::hal::net::NetworkDriverOps& nic,
                          const uint8_t* dst_mac,
                          uint32_t src_ip, uint32_t dst_ip,
                          const uint8_t* payload, size_t payload_len,
                          uint8_t protocol) noexcept;
    void send_udp_packet(kernel::hal::net::NetworkDriverOps& nic,
                         const uint8_t* dst_mac,
                         uint32_t src_ip, uint32_t dst_ip,
                         uint16_t src_port, uint16_t dst_port,
                         const uint8_t* payload, size_t payload_len,
                         uint8_t protocol = 17u) noexcept;
    void send_arp_request(kernel::hal::net::NetworkDriverOps& nic,
                          uint32_t target_ip) noexcept;
    void send_arp_reply(kernel::hal::net::NetworkDriverOps& nic,
                        const uint8_t* dst_mac, uint32_t dst_ip) noexcept;
    void process_ping(kernel::hal::net::NetworkDriverOps& nic) noexcept;
    void maybe_start_dhcp(kernel::hal::net::NetworkDriverOps& nic) noexcept;
    void send_dhcp_discover(kernel::hal::net::NetworkDriverOps& nic) noexcept;
    void send_dhcp_request(kernel::hal::net::NetworkDriverOps& nic, uint32_t requested_ip, uint32_t server_id) noexcept;
    void apply_static_config() noexcept;
    bool begin_ping(uint32_t dst_ip, uint32_t timeout_ms) noexcept;
    bool configured() const noexcept { return local_ip_.load(std::memory_order_relaxed) != 0; }
    uint32_t next_hop_for(uint32_t dst_ip) const noexcept;
    bool same_subnet(uint32_t a, uint32_t b) const noexcept;

    uint8_t nic_idx_ = 0;
    uint16_t udp_port_ = 5000;
    Config config_{};
    std::atomic<uint32_t> local_ip_{0};
    std::atomic<uint32_t> netmask_{0};
    std::atomic<uint32_t> gateway_{0};
    std::atomic<bool> dhcp_bound_{false};
    uint8_t mac_[6]{};
    uint32_t dhcp_xid_ = 0;
    uint32_t dhcp_server_id_ = 0;
    uint32_t dhcp_offer_ip_ = 0;
    uint64_t dhcp_deadline_us_ = 0;
    uint64_t dhcp_retry_us_ = 0;
    uint8_t arp_mac_[6]{};
    uint32_t arp_ip_ = 0;
    uint64_t arp_valid_until_us_ = 0;
    uint64_t arp_retry_us_ = 0;
    std::atomic<uint32_t> ping_request_ip_{0};
    std::atomic<uint32_t> ping_timeout_ms_{0};
    std::atomic<uint32_t> ping_rtt_ms_{0};
    std::atomic<uint8_t> ping_state_{0};
    uint16_t ping_ident_ = 0;
    uint16_t ping_seq_ = 0;
    uint64_t ping_deadline_us_ = 0;
    uint64_t ping_sent_at_us_ = 0;
    uint64_t ping_selftest_due_us_ = 0;
    bool ping_selftest_started_ = false;
    std::atomic<uint32_t> last_ping_target_{0};
    std::atomic<uint32_t> last_ping_rtt_ms_{0};
    std::atomic<uint8_t> last_ping_result_{static_cast<uint8_t>(PingResult::SendFailed)};
    std::atomic<uint64_t> rx_requests_{0};
    std::atomic<uint64_t> tx_responses_{0};
    std::atomic<uint32_t> last_opcode_{0};
};

extern Service g_service;

} // namespace hmi
