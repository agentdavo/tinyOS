// SPDX-License-Identifier: MIT OR Apache-2.0

#include "hmi_service.hpp"

#include "config/tsv.hpp"
#include "ethercat/master.hpp"
#include "kernel/main.hpp"
#include "machine/machine_registry.hpp"
#include "miniOS.hpp"
#include "ui/ui_builder_tsv.hpp"
#include "util.hpp"

namespace hmi {

namespace {

constexpr uint32_t RAW_MAGIC = 0x4D484D49u; // "MHMI"
constexpr uint8_t VERSION = 1;
constexpr uint16_t ETHERTYPE_ARP = 0x0806;
constexpr uint16_t ETHERTYPE_IPV4 = 0x0800;
constexpr uint16_t UDP_PORT_DHCP_SERVER = 67;
constexpr uint16_t UDP_PORT_DHCP_CLIENT = 68;
constexpr uint8_t IPPROTO_UDP = 17;
constexpr uint8_t IPPROTO_ICMP = 1;
constexpr uint32_t DHCP_MAGIC_COOKIE = 0x63825363u;
constexpr uint8_t DHCP_OPT_MSG_TYPE = 53;
constexpr uint8_t DHCP_OPT_REQ_IP = 50;
constexpr uint8_t DHCP_OPT_SERVER_ID = 54;
constexpr uint8_t DHCP_OPT_SUBNET = 1;
constexpr uint8_t DHCP_OPT_ROUTER = 3;
constexpr uint8_t DHCP_OPT_END = 255;
constexpr uint8_t DHCP_DISCOVER = 1;
constexpr uint8_t DHCP_OFFER = 2;
constexpr uint8_t DHCP_REQUEST = 3;
constexpr uint8_t DHCP_ACK = 5;

enum class Opcode : uint8_t {
    Discover = 1,
    Status = 2,
    SymbolGet = 3,
    SymbolSet = 4,
};

enum class StatusCode : uint8_t {
    Ok = 0,
    BadFrame = 1,
    Unsupported = 2,
    NotFound = 3,
    NotWritable = 4,
};

struct EthernetHeader {
    uint8_t dst[6];
    uint8_t src[6];
    uint16_t ethertype_be;
} __attribute__((packed));

struct ArpPacket {
    uint16_t htype_be;
    uint16_t ptype_be;
    uint8_t hlen;
    uint8_t plen;
    uint16_t oper_be;
    uint8_t sha[6];
    uint32_t spa_be;
    uint8_t tha[6];
    uint32_t tpa_be;
} __attribute__((packed));

struct IPv4Header {
    uint8_t ver_ihl;
    uint8_t dscp_ecn;
    uint16_t total_len_be;
    uint16_t ident_be;
    uint16_t flags_frag_be;
    uint8_t ttl;
    uint8_t proto;
    uint16_t hdr_checksum_be;
    uint32_t src_ip_be;
    uint32_t dst_ip_be;
} __attribute__((packed));

struct UdpHeader {
    uint16_t src_port_be;
    uint16_t dst_port_be;
    uint16_t length_be;
    uint16_t checksum_be;
} __attribute__((packed));

struct IcmpEchoHeader {
    uint8_t type;
    uint8_t code;
    uint16_t checksum_be;
    uint16_t identifier_be;
    uint16_t sequence_be;
} __attribute__((packed));

struct RawMessageHeader {
    uint32_t magic_le;
    uint8_t version;
    uint8_t opcode;
    uint8_t status;
    uint8_t reserved;
    uint16_t payload_len_le;
    uint16_t sequence_le;
} __attribute__((packed));

struct StatusPayload {
    uint8_t master_a_state;
    uint8_t master_b_state;
    uint8_t active_page_len;
    uint8_t reserved;
    uint16_t master_a_slaves_le;
    uint16_t master_b_slaves_le;
    uint32_t master_a_cycles_le;
    uint32_t master_b_cycles_le;
    uint32_t local_ip_le;
    uint32_t netmask_le;
    uint32_t gateway_le;
    char active_page[24];
} __attribute__((packed));

struct SymbolPayload {
    uint8_t type;
    uint8_t reserved[3];
    int32_t value_le;
    char name[32];
} __attribute__((packed));

struct DhcpHeader {
    uint8_t op;
    uint8_t htype;
    uint8_t hlen;
    uint8_t hops;
    uint32_t xid_be;
    uint16_t secs_be;
    uint16_t flags_be;
    uint32_t ciaddr_be;
    uint32_t yiaddr_be;
    uint32_t siaddr_be;
    uint32_t giaddr_be;
    uint8_t chaddr[16];
    uint8_t sname[64];
    uint8_t file[128];
    uint32_t cookie_be;
    uint8_t options[128];
} __attribute__((packed));

uint16_t bswap16(uint16_t v) {
    return static_cast<uint16_t>((v >> 8) | (v << 8));
}

uint32_t bswap32(uint32_t v) {
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) << 8) |
           ((v & 0x00FF0000u) >> 8) |
           ((v & 0xFF000000u) >> 24);
}

uint16_t host_to_be16(uint16_t v) { return bswap16(v); }
uint16_t be16_to_host(uint16_t v) { return bswap16(v); }
uint32_t host_to_be32(uint32_t v) { return bswap32(v); }
uint32_t be32_to_host(uint32_t v) { return bswap32(v); }

uint32_t host_to_le32(uint32_t v) { return v; }
uint16_t host_to_le16(uint16_t v) { return v; }

uint16_t checksum16(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < len; i += 2) {
        sum += static_cast<uint16_t>((static_cast<uint16_t>(data[i]) << 8) | data[i + 1]);
    }
    if (len & 1u) sum += static_cast<uint16_t>(data[len - 1] << 8);
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

constexpr uint8_t PING_STATE_IDLE = 0;
constexpr uint8_t PING_STATE_REQUESTED = 1;
constexpr uint8_t PING_STATE_INFLIGHT = 2;
constexpr uint8_t PING_STATE_OK = 3;
constexpr uint8_t PING_STATE_TIMEOUT = 4;
constexpr uint8_t PING_STATE_SEND_FAILED = 5;

uint64_t now_us() {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return t ? t->get_system_time_us() : 0;
}

uint32_t clamp_u64_to_u32(uint64_t v) {
    return v > 0xFFFFFFFFULL ? 0xFFFFFFFFu : static_cast<uint32_t>(v);
}

uint8_t symbol_type_byte(machine::SymbolType t) {
    switch (t) {
        case machine::SymbolType::Bool: return 1;
        case machine::SymbolType::Int: return 2;
        case machine::SymbolType::Float: return 3;
    }
    return 0;
}

void fill_symbol_payload(SymbolPayload& out, const machine::Registry::Entry& entry) {
    out.type = symbol_type_byte(entry.type);
    out.reserved[0] = out.reserved[1] = out.reserved[2] = 0;
    int32_t raw = 0;
    switch (entry.type) {
        case machine::SymbolType::Bool: raw = entry.value.bool_value ? 1 : 0; break;
        case machine::SymbolType::Int: raw = entry.value.int_value; break;
        case machine::SymbolType::Float: raw = static_cast<int32_t>(entry.value.float_value); break;
    }
    out.value_le = static_cast<int32_t>(host_to_le32(static_cast<uint32_t>(raw)));
    kernel::util::k_snprintf(out.name, sizeof(out.name), "%s", entry.name);
}

struct HmiLoadCtx {
    Config cfg{};
    bool ok = true;
};

void record_cb(const config::Record& rec, void* raw_ctx) noexcept {
    auto* ctx = static_cast<HmiLoadCtx*>(raw_ctx);
    if (!ctx || rec.type != "hmi") return;
    const auto key = rec.get("key");
    const auto value = rec.get("value");
    if (key.empty() || value.empty()) {
        ctx->ok = false;
        return;
    }
    uint32_t u32 = 0;
    if (key == "dhcp_enable") {
        ctx->cfg.dhcp_enable = value != "0";
    } else if (key == "static_ip") {
        ctx->ok = kernel::util::ipv4_to_uint32(value, ctx->cfg.static_ip);
    } else if (key == "netmask") {
        ctx->ok = kernel::util::ipv4_to_uint32(value, ctx->cfg.netmask);
    } else if (key == "gateway") {
        ctx->ok = kernel::util::ipv4_to_uint32(value, ctx->cfg.gateway);
    } else if (key == "udp_port") {
        ctx->ok = rec.get_u32("value", u32);
        if (ctx->ok) ctx->cfg.udp_port = static_cast<uint16_t>(u32);
    } else if (key == "dhcp_timeout_ms") {
        ctx->ok = rec.get_u32("value", ctx->cfg.dhcp_timeout_ms);
    } else if (key == "ping_selftest_enable") {
        ctx->cfg.ping_selftest_enable = value != "0";
    } else if (key == "ping_selftest_target") {
        ctx->ok = kernel::util::ipv4_to_uint32(value, ctx->cfg.ping_selftest_target);
    } else if (key == "ping_selftest_delay_ms") {
        ctx->ok = rec.get_u32("value", ctx->cfg.ping_selftest_delay_ms);
    } else if (key == "ping_selftest_timeout_ms") {
        ctx->ok = rec.get_u32("value", ctx->cfg.ping_selftest_timeout_ms);
    }
}

void log_ip_line(const char* prefix, uint32_t ip) {
    char ip_buf[16];
    char line[96];
    kernel::util::uint32_to_ipv4_str(ip, std::span<char>(ip_buf, sizeof(ip_buf)));
    kernel::util::k_snprintf(line, sizeof(line), "%s%s\n", prefix, ip_buf);
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) uart->puts(line);
}

} // namespace

Service g_service;

bool Service::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    HmiLoadCtx ctx;
    if (!config::parse(buf, len, &record_cb, &ctx) || !ctx.ok) return false;
    config_ = ctx.cfg;
    udp_port_ = config_.udp_port;
    return true;
}

bool Service::begin_ping(uint32_t dst_ip, uint32_t timeout_ms) noexcept {
    if (dst_ip == 0) return false;
    uint8_t expected = PING_STATE_IDLE;
    if (!ping_state_.compare_exchange_strong(expected, PING_STATE_REQUESTED, std::memory_order_acq_rel)) {
        return false;
    }
    last_ping_target_.store(dst_ip, std::memory_order_relaxed);
    last_ping_rtt_ms_.store(0, std::memory_order_relaxed);
    last_ping_result_.store(static_cast<uint8_t>(PingResult::Busy), std::memory_order_relaxed);
    ping_request_ip_.store(dst_ip, std::memory_order_relaxed);
    ping_timeout_ms_.store(timeout_ms ? timeout_ms : 1000u, std::memory_order_relaxed);
    ping_rtt_ms_.store(0, std::memory_order_relaxed);
    return true;
}

Service::PingResult Service::ping_ipv4(uint32_t dst_ip, uint32_t timeout_ms, uint32_t& rtt_ms) noexcept {
    if (dst_ip == 0) return PingResult::BadAddress;
    if (!begin_ping(dst_ip, timeout_ms)) return PingResult::Busy;
    const uint64_t wait_until = now_us() + static_cast<uint64_t>(timeout_ms ? timeout_ms : 1000u) * 1000ULL + 500000ULL;
    for (;;) {
        const uint8_t state = ping_state_.load(std::memory_order_acquire);
        if (state == PING_STATE_OK) {
            rtt_ms = ping_rtt_ms_.load(std::memory_order_relaxed);
            last_ping_rtt_ms_.store(rtt_ms, std::memory_order_relaxed);
            last_ping_result_.store(static_cast<uint8_t>(PingResult::Ok), std::memory_order_relaxed);
            ping_state_.store(PING_STATE_IDLE, std::memory_order_release);
            return PingResult::Ok;
        }
        if (state == PING_STATE_TIMEOUT) {
            last_ping_result_.store(static_cast<uint8_t>(PingResult::Timeout), std::memory_order_relaxed);
            ping_state_.store(PING_STATE_IDLE, std::memory_order_release);
            return PingResult::Timeout;
        }
        if (state == PING_STATE_SEND_FAILED) {
            last_ping_result_.store(static_cast<uint8_t>(PingResult::SendFailed), std::memory_order_relaxed);
            ping_state_.store(PING_STATE_IDLE, std::memory_order_release);
            return PingResult::SendFailed;
        }
        if (now_us() >= wait_until) {
            last_ping_result_.store(static_cast<uint8_t>(PingResult::Timeout), std::memory_order_relaxed);
            ping_state_.store(PING_STATE_IDLE, std::memory_order_release);
            return PingResult::Timeout;
        }
        if (kernel::g_scheduler_ptr && kernel::g_platform) kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        else kernel::util::cpu_relax();
    }
}

Service::PingResult Service::ping_ipv4_str(const char* dst_ip, uint32_t timeout_ms, uint32_t& rtt_ms) noexcept {
    if (!dst_ip) return PingResult::BadAddress;
    uint32_t ip = 0;
    if (!kernel::util::ipv4_to_uint32(dst_ip, ip)) return PingResult::BadAddress;
    return ping_ipv4(ip, timeout_ms, rtt_ms);
}

void Service::apply_static_config() noexcept {
    local_ip_.store(config_.static_ip, std::memory_order_relaxed);
    netmask_.store(config_.netmask, std::memory_order_relaxed);
    gateway_.store(config_.gateway, std::memory_order_relaxed);
    dhcp_bound_.store(false, std::memory_order_relaxed);
}

void Service::rx_trampoline(int, const uint8_t* data, size_t len, void* ctx) noexcept {
    auto* c = static_cast<Context*>(ctx);
    if (!c || !c->self || !c->nic) return;
    c->self->handle_eth_frame(*c->nic, data, len);
}

void Service::send_raw_response(kernel::hal::net::NetworkDriverOps& nic,
                                const uint8_t* dst_mac,
                                uint8_t opcode, uint8_t status,
                                const uint8_t* payload, size_t payload_len) noexcept {
    uint8_t frame[256]{};
    if (!dst_mac || payload_len + sizeof(EthernetHeader) + sizeof(RawMessageHeader) > sizeof(frame)) return;
    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = dst_mac[i];
        eth->src[i] = mac_[i];
    }
    eth->ethertype_be = host_to_be16(ETHERTYPE);
    auto* hdr = reinterpret_cast<RawMessageHeader*>(frame + sizeof(EthernetHeader));
    hdr->magic_le = host_to_le32(RAW_MAGIC);
    hdr->version = VERSION;
    hdr->opcode = opcode;
    hdr->status = status;
    hdr->reserved = 0;
    hdr->payload_len_le = host_to_le16(static_cast<uint16_t>(payload_len));
    hdr->sequence_le = 0;
    if (payload_len && payload) {
        kernel::util::kmemcpy(frame + sizeof(EthernetHeader) + sizeof(RawMessageHeader), payload, payload_len);
    }
    const size_t frame_len = sizeof(EthernetHeader) + sizeof(RawMessageHeader) + payload_len;
    if (nic.send_packet(nic_idx_, frame, frame_len)) {
        tx_responses_.fetch_add(1, std::memory_order_relaxed);
    }
}

void Service::send_ipv4_packet(kernel::hal::net::NetworkDriverOps& nic,
                               const uint8_t* dst_mac,
                               uint32_t src_ip, uint32_t dst_ip,
                               const uint8_t* payload, size_t payload_len,
                               uint8_t protocol) noexcept {
    uint8_t frame[512]{};
    const size_t total = sizeof(EthernetHeader) + sizeof(IPv4Header) + payload_len;
    if (!dst_mac || total > sizeof(frame)) return;
    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = dst_mac[i];
        eth->src[i] = mac_[i];
    }
    eth->ethertype_be = host_to_be16(ETHERTYPE_IPV4);

    auto* ip = reinterpret_cast<IPv4Header*>(frame + sizeof(EthernetHeader));
    ip->ver_ihl = 0x45;
    ip->dscp_ecn = 0;
    ip->total_len_be = host_to_be16(static_cast<uint16_t>(sizeof(IPv4Header) + payload_len));
    ip->ident_be = 0;
    ip->flags_frag_be = 0;
    ip->ttl = 64;
    ip->proto = protocol;
    ip->hdr_checksum_be = 0;
    ip->src_ip_be = host_to_be32(src_ip);
    ip->dst_ip_be = host_to_be32(dst_ip);
    ip->hdr_checksum_be = host_to_be16(
        checksum16(reinterpret_cast<const uint8_t*>(ip), sizeof(IPv4Header)));

    if (payload_len && payload) {
        kernel::util::kmemcpy(reinterpret_cast<uint8_t*>(ip) + sizeof(IPv4Header), payload, payload_len);
    }

    if (nic.send_packet(nic_idx_, frame, total)) {
        tx_responses_.fetch_add(1, std::memory_order_relaxed);
    }
}

void Service::send_udp_packet(kernel::hal::net::NetworkDriverOps& nic,
                              const uint8_t* dst_mac,
                              uint32_t src_ip, uint32_t dst_ip,
                              uint16_t src_port, uint16_t dst_port,
                              const uint8_t* payload, size_t payload_len,
                              uint8_t protocol) noexcept {
    uint8_t packet[512]{};
    const size_t total = sizeof(UdpHeader) + payload_len;
    if (total > sizeof(packet) || protocol != IPPROTO_UDP) return;
    auto* udp = reinterpret_cast<UdpHeader*>(packet);
    udp->src_port_be = host_to_be16(src_port);
    udp->dst_port_be = host_to_be16(dst_port);
    udp->length_be = host_to_be16(static_cast<uint16_t>(sizeof(UdpHeader) + payload_len));
    udp->checksum_be = 0; // IPv4 permits zero UDP checksum

    if (payload_len && payload) {
        kernel::util::kmemcpy(packet + sizeof(UdpHeader), payload, payload_len);
    }
    send_ipv4_packet(nic, dst_mac, src_ip, dst_ip, packet, total, protocol);
}

void Service::send_arp_request(kernel::hal::net::NetworkDriverOps& nic,
                               uint32_t target_ip) noexcept {
    uint8_t frame[sizeof(EthernetHeader) + sizeof(ArpPacket)]{};
    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    auto* arp = reinterpret_cast<ArpPacket*>(frame + sizeof(EthernetHeader));
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = 0xFF;
        eth->src[i] = mac_[i];
        arp->sha[i] = mac_[i];
        arp->tha[i] = 0;
    }
    eth->ethertype_be = host_to_be16(ETHERTYPE_ARP);
    arp->htype_be = host_to_be16(1);
    arp->ptype_be = host_to_be16(ETHERTYPE_IPV4);
    arp->hlen = 6;
    arp->plen = 4;
    arp->oper_be = host_to_be16(1);
    arp->spa_be = host_to_be32(local_ip_.load(std::memory_order_relaxed));
    arp->tpa_be = host_to_be32(target_ip);
    log_ip_line("[hmi] arp who-has ", target_ip);
    (void)nic.send_packet(nic_idx_, frame, sizeof(frame));
}

void Service::send_udp_hmi_response(kernel::hal::net::NetworkDriverOps& nic,
                                    const uint8_t* dst_mac, uint32_t dst_ip, uint16_t dst_port,
                                    uint8_t opcode, uint8_t status,
                                    const uint8_t* payload, size_t payload_len) noexcept {
    uint8_t packet[256]{};
    if (payload_len + sizeof(RawMessageHeader) > sizeof(packet)) return;
    auto* hdr = reinterpret_cast<RawMessageHeader*>(packet);
    hdr->magic_le = host_to_le32(RAW_MAGIC);
    hdr->version = VERSION;
    hdr->opcode = opcode;
    hdr->status = status;
    hdr->reserved = 0;
    hdr->payload_len_le = host_to_le16(static_cast<uint16_t>(payload_len));
    hdr->sequence_le = 0;
    if (payload_len && payload) {
        kernel::util::kmemcpy(packet + sizeof(RawMessageHeader), payload, payload_len);
    }
    send_udp_packet(nic, dst_mac,
                    local_ip_.load(std::memory_order_relaxed), dst_ip,
                    udp_port_, dst_port,
                    packet, sizeof(RawMessageHeader) + payload_len);
}

void Service::send_arp_reply(kernel::hal::net::NetworkDriverOps& nic,
                             const uint8_t* dst_mac, uint32_t dst_ip) noexcept {
    uint8_t frame[sizeof(EthernetHeader) + sizeof(ArpPacket)]{};
    auto* eth = reinterpret_cast<EthernetHeader*>(frame);
    auto* arp = reinterpret_cast<ArpPacket*>(frame + sizeof(EthernetHeader));
    for (size_t i = 0; i < 6; ++i) {
        eth->dst[i] = dst_mac[i];
        eth->src[i] = mac_[i];
        arp->sha[i] = mac_[i];
        arp->tha[i] = dst_mac[i];
    }
    eth->ethertype_be = host_to_be16(ETHERTYPE_ARP);
    arp->htype_be = host_to_be16(1);
    arp->ptype_be = host_to_be16(ETHERTYPE_IPV4);
    arp->hlen = 6;
    arp->plen = 4;
    arp->oper_be = host_to_be16(2);
    arp->spa_be = host_to_be32(local_ip_.load(std::memory_order_relaxed));
    arp->tpa_be = host_to_be32(dst_ip);
    (void)nic.send_packet(nic_idx_, frame, sizeof(frame));
}

void Service::maybe_start_dhcp(kernel::hal::net::NetworkDriverOps& nic) noexcept {
    if (!config_.dhcp_enable || dhcp_bound_.load(std::memory_order_relaxed)) return;
    const uint64_t now = now_us();
    if (dhcp_deadline_us_ == 0) {
        dhcp_deadline_us_ = now + static_cast<uint64_t>(config_.dhcp_timeout_ms) * 1000ULL;
        dhcp_retry_us_ = 0;
    } else if (dhcp_deadline_us_ == UINT64_MAX) {
        return;
    }
    if (now >= dhcp_deadline_us_) {
        if (!configured()) apply_static_config();
        if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
            uart->puts("[hmi] dhcp timeout, using fallback config\n");
        }
        dhcp_deadline_us_ = UINT64_MAX;
        dhcp_retry_us_ = UINT64_MAX;
        return;
    }
    if (now < dhcp_retry_us_) return;
    dhcp_xid_ = 0x484D4900u ^ static_cast<uint32_t>(now);
    send_dhcp_discover(nic);
    dhcp_retry_us_ = now + 500000ULL;
}

void Service::send_dhcp_discover(kernel::hal::net::NetworkDriverOps& nic) noexcept {
    DhcpHeader dhcp{};
    dhcp.op = 1;
    dhcp.htype = 1;
    dhcp.hlen = 6;
    dhcp.xid_be = host_to_be32(dhcp_xid_);
    dhcp.flags_be = host_to_be16(0x8000);
    for (size_t i = 0; i < 6; ++i) dhcp.chaddr[i] = mac_[i];
    dhcp.cookie_be = host_to_be32(DHCP_MAGIC_COOKIE);
    size_t o = 0;
    dhcp.options[o++] = DHCP_OPT_MSG_TYPE; dhcp.options[o++] = 1; dhcp.options[o++] = DHCP_DISCOVER;
    dhcp.options[o++] = 55; dhcp.options[o++] = 3; dhcp.options[o++] = DHCP_OPT_SUBNET; dhcp.options[o++] = DHCP_OPT_ROUTER; dhcp.options[o++] = DHCP_OPT_SERVER_ID;
    dhcp.options[o++] = 61; dhcp.options[o++] = 7; dhcp.options[o++] = 1;
    for (size_t i = 0; i < 6; ++i) dhcp.options[o++] = mac_[i];
    dhcp.options[o++] = DHCP_OPT_END;
    uint8_t bcast[6]{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    send_udp_packet(nic, bcast, 0u, 0xFFFFFFFFu, UDP_PORT_DHCP_CLIENT, UDP_PORT_DHCP_SERVER,
                    reinterpret_cast<const uint8_t*>(&dhcp), sizeof(dhcp));
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) uart->puts("[hmi] dhcp discover\n");
}

void Service::send_dhcp_request(kernel::hal::net::NetworkDriverOps& nic, uint32_t requested_ip, uint32_t server_id) noexcept {
    DhcpHeader dhcp{};
    dhcp.op = 1;
    dhcp.htype = 1;
    dhcp.hlen = 6;
    dhcp.xid_be = host_to_be32(dhcp_xid_);
    dhcp.flags_be = host_to_be16(0x8000);
    for (size_t i = 0; i < 6; ++i) dhcp.chaddr[i] = mac_[i];
    dhcp.cookie_be = host_to_be32(DHCP_MAGIC_COOKIE);
    size_t o = 0;
    dhcp.options[o++] = DHCP_OPT_MSG_TYPE; dhcp.options[o++] = 1; dhcp.options[o++] = DHCP_REQUEST;
    dhcp.options[o++] = DHCP_OPT_REQ_IP; dhcp.options[o++] = 4;
    const uint32_t req_be = host_to_be32(requested_ip);
    kernel::util::kmemcpy(&dhcp.options[o], &req_be, 4); o += 4;
    dhcp.options[o++] = DHCP_OPT_SERVER_ID; dhcp.options[o++] = 4;
    const uint32_t srv_be = host_to_be32(server_id);
    kernel::util::kmemcpy(&dhcp.options[o], &srv_be, 4); o += 4;
    dhcp.options[o++] = 61; dhcp.options[o++] = 7; dhcp.options[o++] = 1;
    for (size_t i = 0; i < 6; ++i) dhcp.options[o++] = mac_[i];
    dhcp.options[o++] = DHCP_OPT_END;
    uint8_t bcast[6]{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    send_udp_packet(nic, bcast, 0u, 0xFFFFFFFFu, UDP_PORT_DHCP_CLIENT, UDP_PORT_DHCP_SERVER,
                    reinterpret_cast<const uint8_t*>(&dhcp), sizeof(dhcp));
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) uart->puts("[hmi] dhcp request\n");
}

bool Service::same_subnet(uint32_t a, uint32_t b) const noexcept {
    const uint32_t mask = netmask_.load(std::memory_order_relaxed);
    return (a & mask) == (b & mask);
}

uint32_t Service::next_hop_for(uint32_t dst_ip) const noexcept {
    const uint32_t gw = gateway_.load(std::memory_order_relaxed);
    return (gw != 0 && !same_subnet(local_ip_.load(std::memory_order_relaxed), dst_ip)) ? gw : dst_ip;
}

void Service::handle_icmp(uint32_t src_ip,
                          const uint8_t* payload, size_t payload_len) noexcept {
    if (!payload || payload_len < sizeof(IcmpEchoHeader)) return;
    const auto* icmp = reinterpret_cast<const IcmpEchoHeader*>(payload);
    if (icmp->type != 0 || icmp->code != 0) return;
    if (be16_to_host(icmp->identifier_be) != ping_ident_ || be16_to_host(icmp->sequence_be) != ping_seq_) return;
    if (src_ip != ping_request_ip_.load(std::memory_order_relaxed)) return;
    const uint64_t rtt_us = now_us() - ping_sent_at_us_;
    ping_rtt_ms_.store(static_cast<uint32_t>((rtt_us + 999ULL) / 1000ULL), std::memory_order_relaxed);
    last_ping_rtt_ms_.store(static_cast<uint32_t>((rtt_us + 999ULL) / 1000ULL), std::memory_order_relaxed);
    last_ping_result_.store(static_cast<uint8_t>(PingResult::Ok), std::memory_order_relaxed);
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        char ip_buf[16];
        char line[96];
        kernel::util::uint32_to_ipv4_str(src_ip, std::span<char>(ip_buf, sizeof(ip_buf)));
        kernel::util::k_snprintf(line, sizeof(line), "[hmi] ping reply src=%s rtt=%lums\n",
                                 ip_buf, static_cast<unsigned long>((rtt_us + 999ULL) / 1000ULL));
        uart->puts(line);
    }
    ping_state_.store(PING_STATE_OK, std::memory_order_release);
}

void Service::process_ping(kernel::hal::net::NetworkDriverOps& nic) noexcept {
    const uint8_t state = ping_state_.load(std::memory_order_acquire);
    if (state != PING_STATE_REQUESTED && state != PING_STATE_INFLIGHT) return;
    const uint64_t now = now_us();
    const uint32_t dst_ip = ping_request_ip_.load(std::memory_order_relaxed);
    if (dst_ip == 0 || !configured()) {
        last_ping_result_.store(static_cast<uint8_t>(PingResult::SendFailed), std::memory_order_relaxed);
        ping_state_.store(PING_STATE_SEND_FAILED, std::memory_order_release);
        return;
    }
    if (state == PING_STATE_REQUESTED) {
        ping_deadline_us_ = now + static_cast<uint64_t>(ping_timeout_ms_.load(std::memory_order_relaxed)) * 1000ULL;
        ping_ident_ = static_cast<uint16_t>(0x484dU ^ nic_idx_);
        ++ping_seq_;
    }
    if (now >= ping_deadline_us_) {
        if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
            char ip_buf[16];
            char line[96];
            kernel::util::uint32_to_ipv4_str(dst_ip, std::span<char>(ip_buf, sizeof(ip_buf)));
            kernel::util::k_snprintf(line, sizeof(line), "[hmi] ping timeout dst=%s seq=%u\n",
                                     ip_buf, static_cast<unsigned>(ping_seq_));
            uart->puts(line);
        }
        last_ping_result_.store(static_cast<uint8_t>(PingResult::Timeout), std::memory_order_relaxed);
        ping_state_.store(PING_STATE_TIMEOUT, std::memory_order_release);
        return;
    }

    const uint32_t next_hop = next_hop_for(dst_ip);
    if (arp_ip_ != next_hop || arp_valid_until_us_ <= now) {
        if (now >= arp_retry_us_) {
            if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
                char dst_buf[16];
                char hop_buf[16];
                char line[128];
                kernel::util::uint32_to_ipv4_str(dst_ip, std::span<char>(dst_buf, sizeof(dst_buf)));
                kernel::util::uint32_to_ipv4_str(next_hop, std::span<char>(hop_buf, sizeof(hop_buf)));
                kernel::util::k_snprintf(line, sizeof(line), "[hmi] ping route dst=%s via=%s\n", dst_buf, hop_buf);
                uart->puts(line);
            }
            send_arp_request(nic, next_hop);
            arp_retry_us_ = now + 250000ULL;
            arp_ip_ = next_hop;
        }
        ping_state_.store(PING_STATE_REQUESTED, std::memory_order_release);
        return;
    }

    if (state == PING_STATE_INFLIGHT) return;

    uint8_t packet[64]{};
    auto* icmp = reinterpret_cast<IcmpEchoHeader*>(packet);
    icmp->type = 8;
    icmp->code = 0;
    icmp->checksum_be = 0;
    icmp->identifier_be = host_to_be16(ping_ident_);
    icmp->sequence_be = host_to_be16(ping_seq_);
    const char probe[] = "miniOS-ping";
    kernel::util::kmemcpy(packet + sizeof(IcmpEchoHeader), probe, sizeof(probe));
    icmp->checksum_be = host_to_be16(
        checksum16(packet, sizeof(IcmpEchoHeader) + sizeof(probe)));
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        char ip_buf[16];
        char line[96];
        kernel::util::uint32_to_ipv4_str(dst_ip, std::span<char>(ip_buf, sizeof(ip_buf)));
        kernel::util::k_snprintf(line, sizeof(line), "[hmi] ping send dst=%s seq=%u\n",
                                 ip_buf, static_cast<unsigned>(ping_seq_));
        uart->puts(line);
    }
    send_ipv4_packet(nic, arp_mac_, local_ip_.load(std::memory_order_relaxed), dst_ip,
                     packet, sizeof(IcmpEchoHeader) + sizeof(probe), IPPROTO_ICMP);
    ping_sent_at_us_ = now;
    ping_state_.store(PING_STATE_INFLIGHT, std::memory_order_release);
}

void Service::handle_dhcp(kernel::hal::net::NetworkDriverOps& nic,
                          uint32_t,
                          const uint8_t* payload, size_t payload_len) noexcept {
    if (!payload || payload_len < sizeof(DhcpHeader)) return;
    const auto* dhcp = reinterpret_cast<const DhcpHeader*>(payload);
    if (be32_to_host(dhcp->xid_be) != dhcp_xid_ || be32_to_host(dhcp->cookie_be) != DHCP_MAGIC_COOKIE) return;
    uint8_t msg_type = 0;
    uint32_t subnet = config_.netmask;
    uint32_t router = config_.gateway;
    uint32_t server_id = 0;
    size_t o = 0;
    while (o < sizeof(dhcp->options)) {
        const uint8_t code = dhcp->options[o++];
        if (code == DHCP_OPT_END) break;
        if (code == 0 || o >= sizeof(dhcp->options)) continue;
        const uint8_t len = dhcp->options[o++];
        if (o + len > sizeof(dhcp->options)) break;
        switch (code) {
            case DHCP_OPT_MSG_TYPE:
                if (len >= 1) msg_type = dhcp->options[o];
                break;
            case DHCP_OPT_SUBNET:
                if (len == 4) subnet = (static_cast<uint32_t>(dhcp->options[o]) << 24) |
                                       (static_cast<uint32_t>(dhcp->options[o + 1]) << 16) |
                                       (static_cast<uint32_t>(dhcp->options[o + 2]) << 8) |
                                       static_cast<uint32_t>(dhcp->options[o + 3]);
                break;
            case DHCP_OPT_ROUTER:
                if (len >= 4) router = (static_cast<uint32_t>(dhcp->options[o]) << 24) |
                                       (static_cast<uint32_t>(dhcp->options[o + 1]) << 16) |
                                       (static_cast<uint32_t>(dhcp->options[o + 2]) << 8) |
                                       static_cast<uint32_t>(dhcp->options[o + 3]);
                break;
            case DHCP_OPT_SERVER_ID:
                if (len == 4) server_id = (static_cast<uint32_t>(dhcp->options[o]) << 24) |
                                          (static_cast<uint32_t>(dhcp->options[o + 1]) << 16) |
                                          (static_cast<uint32_t>(dhcp->options[o + 2]) << 8) |
                                          static_cast<uint32_t>(dhcp->options[o + 3]);
                break;
            default:
                break;
        }
        o += len;
    }
    const uint32_t yiaddr = be32_to_host(dhcp->yiaddr_be);
    if (msg_type == DHCP_OFFER && yiaddr != 0) {
        dhcp_offer_ip_ = yiaddr;
        dhcp_server_id_ = server_id;
        log_ip_line("[hmi] dhcp offer ip=", yiaddr);
        send_dhcp_request(nic, yiaddr, server_id);
    } else if (msg_type == DHCP_ACK && yiaddr != 0) {
        local_ip_.store(yiaddr, std::memory_order_relaxed);
        netmask_.store(subnet, std::memory_order_relaxed);
        gateway_.store(router, std::memory_order_relaxed);
        dhcp_bound_.store(true, std::memory_order_relaxed);
        log_ip_line("[hmi] dhcp lease ip=", yiaddr);
    }
}

void Service::handle_raw_hmi(kernel::hal::net::NetworkDriverOps& nic,
                             const uint8_t* src_mac,
                             const uint8_t* data, size_t len) noexcept {
    if (!data || len < sizeof(RawMessageHeader)) return;
    const auto* hdr = reinterpret_cast<const RawMessageHeader*>(data);
    if (hdr->magic_le != RAW_MAGIC || hdr->version != VERSION) {
        send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::BadFrame), nullptr, 0);
        return;
    }
    const size_t payload_len = hdr->payload_len_le;
    if (sizeof(RawMessageHeader) + payload_len > len) {
        send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::BadFrame), nullptr, 0);
        return;
    }
    const uint8_t* payload = data + sizeof(RawMessageHeader);
    rx_requests_.fetch_add(1, std::memory_order_relaxed);
    last_opcode_.store(hdr->opcode, std::memory_order_relaxed);

    switch (static_cast<Opcode>(hdr->opcode)) {
        case Opcode::Discover: {
            char payload_buf[64]{};
            kernel::util::k_snprintf(payload_buf, sizeof(payload_buf),
                                     "miniOS hmi nic=eth%u git=%s",
                                     static_cast<unsigned>(nic_idx_),
                                     MINIOS_GIT_HASH);
            send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::Ok),
                              reinterpret_cast<const uint8_t*>(payload_buf), kernel::util::kstrlen(payload_buf) + 1u);
            return;
        }
        case Opcode::Status: {
            StatusPayload out{};
            out.master_a_state = static_cast<uint8_t>(ethercat::g_master_a.state());
            out.master_b_state = static_cast<uint8_t>(ethercat::g_master_b.state());
            out.master_a_slaves_le = host_to_le16(static_cast<uint16_t>(ethercat::g_master_a.slave_count()));
            out.master_b_slaves_le = host_to_le16(static_cast<uint16_t>(ethercat::g_master_b.slave_count()));
            out.master_a_cycles_le = host_to_le32(clamp_u64_to_u32(ethercat::g_master_a.stats().cycles.load(std::memory_order_relaxed)));
            out.master_b_cycles_le = host_to_le32(clamp_u64_to_u32(ethercat::g_master_b.stats().cycles.load(std::memory_order_relaxed)));
            out.local_ip_le = host_to_le32(local_ip_.load(std::memory_order_relaxed));
            out.netmask_le = host_to_le32(netmask_.load(std::memory_order_relaxed));
            out.gateway_le = host_to_le32(gateway_.load(std::memory_order_relaxed));
            const char* page = ui_builder::active_page_id();
            kernel::util::k_snprintf(out.active_page, sizeof(out.active_page), "%s", page ? page : "none");
            out.active_page_len = static_cast<uint8_t>(kernel::util::kstrlen(out.active_page));
            send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::Ok),
                              reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
        case Opcode::SymbolGet: {
            if (payload_len < sizeof(SymbolPayload)) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::BadFrame), nullptr, 0);
                return;
            }
            const auto* req = reinterpret_cast<const SymbolPayload*>(payload);
            size_t idx = 0;
            if (!machine::g_registry.find(req->name, idx)) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::NotFound), nullptr, 0);
                return;
            }
            const auto* entry = machine::g_registry.entry(idx);
            if (!entry) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::NotFound), nullptr, 0);
                return;
            }
            SymbolPayload out{};
            fill_symbol_payload(out, *entry);
            send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::Ok),
                              reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
        case Opcode::SymbolSet: {
            if (payload_len < sizeof(SymbolPayload)) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::BadFrame), nullptr, 0);
                return;
            }
            const auto* req = reinterpret_cast<const SymbolPayload*>(payload);
            size_t idx = 0;
            if (!machine::g_registry.find(req->name, idx)) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::NotFound), nullptr, 0);
                return;
            }
            const auto* entry = machine::g_registry.entry(idx);
            if (!entry || !entry->writable) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::NotWritable), nullptr, 0);
                return;
            }
            const int32_t value = req->value_le;
            bool ok = false;
            switch (entry->type) {
                case machine::SymbolType::Bool: ok = machine::g_registry.set_bool(req->name, value != 0); break;
                case machine::SymbolType::Int: ok = machine::g_registry.set_int(req->name, value); break;
                case machine::SymbolType::Float: ok = machine::g_registry.set_float(req->name, static_cast<float>(value)); break;
            }
            if (!ok) {
                send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::NotWritable), nullptr, 0);
                return;
            }
            const auto* updated = machine::g_registry.entry(idx);
            SymbolPayload out{};
            if (updated) fill_symbol_payload(out, *updated);
            send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::Ok),
                              reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
    }
    send_raw_response(nic, src_mac, hdr->opcode, static_cast<uint8_t>(StatusCode::Unsupported), nullptr, 0);
}

void Service::handle_udp(kernel::hal::net::NetworkDriverOps& nic,
                         const uint8_t* eth_src,
                         uint32_t src_ip, uint16_t src_port,
                         uint32_t, uint16_t dst_port,
                         const uint8_t* payload, size_t payload_len) noexcept {
    if (dst_port == UDP_PORT_DHCP_CLIENT) {
        handle_dhcp(nic, src_ip, payload, payload_len);
        return;
    }
    if (dst_port != udp_port_ || !configured()) return;
    if (!payload || payload_len < sizeof(RawMessageHeader)) return;
    const auto* hdr = reinterpret_cast<const RawMessageHeader*>(payload);
    if (hdr->magic_le != RAW_MAGIC || hdr->version != VERSION) return;
    const size_t inner_len = hdr->payload_len_le;
    if (sizeof(RawMessageHeader) + inner_len > payload_len) return;
    const uint8_t* inner = payload + sizeof(RawMessageHeader);
    rx_requests_.fetch_add(1, std::memory_order_relaxed);
    last_opcode_.store(hdr->opcode, std::memory_order_relaxed);

    auto respond = [&](StatusCode status, const uint8_t* resp_payload, size_t resp_len) {
        send_udp_hmi_response(nic, eth_src, src_ip, src_port, hdr->opcode, static_cast<uint8_t>(status), resp_payload, resp_len);
    };

    switch (static_cast<Opcode>(hdr->opcode)) {
        case Opcode::Discover: {
            char payload_buf[64]{};
            kernel::util::k_snprintf(payload_buf, sizeof(payload_buf),
                                     "miniOS hmi ip nic=eth%u git=%s",
                                     static_cast<unsigned>(nic_idx_),
                                     MINIOS_GIT_HASH);
            respond(StatusCode::Ok, reinterpret_cast<const uint8_t*>(payload_buf), kernel::util::kstrlen(payload_buf) + 1u);
            return;
        }
        case Opcode::Status: {
            StatusPayload out{};
            out.master_a_state = static_cast<uint8_t>(ethercat::g_master_a.state());
            out.master_b_state = static_cast<uint8_t>(ethercat::g_master_b.state());
            out.master_a_slaves_le = host_to_le16(static_cast<uint16_t>(ethercat::g_master_a.slave_count()));
            out.master_b_slaves_le = host_to_le16(static_cast<uint16_t>(ethercat::g_master_b.slave_count()));
            out.master_a_cycles_le = host_to_le32(clamp_u64_to_u32(ethercat::g_master_a.stats().cycles.load(std::memory_order_relaxed)));
            out.master_b_cycles_le = host_to_le32(clamp_u64_to_u32(ethercat::g_master_b.stats().cycles.load(std::memory_order_relaxed)));
            out.local_ip_le = host_to_le32(local_ip_.load(std::memory_order_relaxed));
            out.netmask_le = host_to_le32(netmask_.load(std::memory_order_relaxed));
            out.gateway_le = host_to_le32(gateway_.load(std::memory_order_relaxed));
            const char* page = ui_builder::active_page_id();
            kernel::util::k_snprintf(out.active_page, sizeof(out.active_page), "%s", page ? page : "none");
            out.active_page_len = static_cast<uint8_t>(kernel::util::kstrlen(out.active_page));
            respond(StatusCode::Ok, reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
        case Opcode::SymbolGet: {
            if (inner_len < sizeof(SymbolPayload)) { respond(StatusCode::BadFrame, nullptr, 0); return; }
            const auto* req = reinterpret_cast<const SymbolPayload*>(inner);
            size_t idx = 0;
            if (!machine::g_registry.find(req->name, idx)) { respond(StatusCode::NotFound, nullptr, 0); return; }
            const auto* entry = machine::g_registry.entry(idx);
            if (!entry) { respond(StatusCode::NotFound, nullptr, 0); return; }
            SymbolPayload out{};
            fill_symbol_payload(out, *entry);
            respond(StatusCode::Ok, reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
        case Opcode::SymbolSet: {
            if (inner_len < sizeof(SymbolPayload)) { respond(StatusCode::BadFrame, nullptr, 0); return; }
            const auto* req = reinterpret_cast<const SymbolPayload*>(inner);
            size_t idx = 0;
            if (!machine::g_registry.find(req->name, idx)) { respond(StatusCode::NotFound, nullptr, 0); return; }
            const auto* entry = machine::g_registry.entry(idx);
            if (!entry || !entry->writable) { respond(StatusCode::NotWritable, nullptr, 0); return; }
            const int32_t value = req->value_le;
            bool ok = false;
            switch (entry->type) {
                case machine::SymbolType::Bool: ok = machine::g_registry.set_bool(req->name, value != 0); break;
                case machine::SymbolType::Int: ok = machine::g_registry.set_int(req->name, value); break;
                case machine::SymbolType::Float: ok = machine::g_registry.set_float(req->name, static_cast<float>(value)); break;
            }
            if (!ok) { respond(StatusCode::NotWritable, nullptr, 0); return; }
            const auto* updated = machine::g_registry.entry(idx);
            SymbolPayload out{};
            if (updated) fill_symbol_payload(out, *updated);
            respond(StatusCode::Ok, reinterpret_cast<const uint8_t*>(&out), sizeof(out));
            return;
        }
    }
    respond(StatusCode::Unsupported, nullptr, 0);
}

void Service::handle_ipv4(kernel::hal::net::NetworkDriverOps& nic,
                          const uint8_t* eth_src,
                          const uint8_t* data, size_t len) noexcept {
    if (!data || len < sizeof(IPv4Header)) return;
    const auto* ip = reinterpret_cast<const IPv4Header*>(data);
    const uint8_t ihl = static_cast<uint8_t>((ip->ver_ihl & 0x0Fu) * 4u);
    if ((ip->ver_ihl >> 4) != 4 || ihl < sizeof(IPv4Header) || len < ihl) return;
    const uint16_t total_len = be16_to_host(ip->total_len_be);
    if (total_len < ihl || total_len > len) return;
    const uint32_t dst_ip = be32_to_host(ip->dst_ip_be);
    const uint32_t src_ip = be32_to_host(ip->src_ip_be);
    const uint32_t our_ip = local_ip_.load(std::memory_order_relaxed);
    if (dst_ip != our_ip && dst_ip != 0xFFFFFFFFu && dst_ip != 0u) return;
    if (ip->proto == IPPROTO_UDP) {
        if (total_len < ihl + sizeof(UdpHeader)) return;
        const auto* udp = reinterpret_cast<const UdpHeader*>(data + ihl);
        const uint16_t udp_len = be16_to_host(udp->length_be);
        if (udp_len < sizeof(UdpHeader) || ihl + udp_len > total_len) return;
        handle_udp(nic, eth_src, src_ip, be16_to_host(udp->src_port_be), dst_ip, be16_to_host(udp->dst_port_be),
                   reinterpret_cast<const uint8_t*>(udp) + sizeof(UdpHeader), udp_len - sizeof(UdpHeader));
        return;
    }
    if (ip->proto == IPPROTO_ICMP) {
        handle_icmp(src_ip, data + ihl, total_len - ihl);
    }
}

void Service::handle_eth_frame(kernel::hal::net::NetworkDriverOps& nic,
                               const uint8_t* data, size_t len) noexcept {
    if (!data || len < sizeof(EthernetHeader)) return;
    const auto* eth = reinterpret_cast<const EthernetHeader*>(data);
    const uint16_t type = be16_to_host(eth->ethertype_be);
    const uint8_t* payload = data + sizeof(EthernetHeader);
    const size_t payload_len = len - sizeof(EthernetHeader);
    switch (type) {
        case ETHERTYPE:
            handle_raw_hmi(nic, eth->src, payload, payload_len);
            return;
        case ETHERTYPE_ARP: {
            if (payload_len < sizeof(ArpPacket) || !configured()) return;
            const auto* arp = reinterpret_cast<const ArpPacket*>(payload);
            const uint16_t oper = be16_to_host(arp->oper_be);
            const uint32_t spa = be32_to_host(arp->spa_be);
            const uint32_t tpa = be32_to_host(arp->tpa_be);
            if (oper == 2 && tpa == local_ip_.load(std::memory_order_relaxed)) {
                arp_ip_ = spa;
                for (size_t i = 0; i < 6; ++i) arp_mac_[i] = arp->sha[i];
                arp_valid_until_us_ = now_us() + 5000000ULL;
                if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
                    char ip_buf[16];
                    char line[128];
                    kernel::util::uint32_to_ipv4_str(spa, std::span<char>(ip_buf, sizeof(ip_buf)));
                    kernel::util::k_snprintf(line, sizeof(line),
                                             "[hmi] arp learned ip=%s mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
                                             ip_buf,
                                             static_cast<unsigned>(arp_mac_[0]), static_cast<unsigned>(arp_mac_[1]),
                                             static_cast<unsigned>(arp_mac_[2]), static_cast<unsigned>(arp_mac_[3]),
                                             static_cast<unsigned>(arp_mac_[4]), static_cast<unsigned>(arp_mac_[5]));
                    uart->puts(line);
                }
            }
            if (oper == 1 && tpa == local_ip_.load(std::memory_order_relaxed)) {
                send_arp_reply(nic, eth->src, spa);
            }
            return;
        }
        case ETHERTYPE_IPV4:
            handle_ipv4(nic, eth->src, payload, payload_len);
            return;
        default:
            return;
    }
}

void Service::thread_entry(void* arg) {
    auto* self = arg ? static_cast<Service*>(arg) : &g_service;
    if (!self || !kernel::g_platform) return;
    auto* nic = kernel::g_platform->get_net_ops(static_cast<int>(self->nic_idx_));
    auto* uart = kernel::g_platform->get_uart_ops();
    if (!nic) {
        if (uart) uart->puts("[hmi] no NIC bound\n");
        return;
    }
    if (!nic->get_mac(self->mac_)) {
        self->mac_[0] = 0x02; self->mac_[1] = 0; self->mac_[2] = 0; self->mac_[3] = 0; self->mac_[4] = 0; self->mac_[5] = self->nic_idx_;
    }
    self->udp_port_ = self->config_.udp_port;
    self->local_ip_.store(0, std::memory_order_relaxed);
    self->netmask_.store(0, std::memory_order_relaxed);
    self->gateway_.store(0, std::memory_order_relaxed);
    self->apply_static_config();
    self->arp_ip_ = 0;
    self->arp_valid_until_us_ = 0;
    self->arp_retry_us_ = 0;
    self->ping_state_.store(PING_STATE_IDLE, std::memory_order_relaxed);
    self->ping_selftest_due_us_ = now_us() + static_cast<uint64_t>(self->config_.ping_selftest_delay_ms) * 1000ULL;
    self->ping_selftest_started_ = false;
    self->last_ping_target_.store(0, std::memory_order_relaxed);
    self->last_ping_rtt_ms_.store(0, std::memory_order_relaxed);
    self->last_ping_result_.store(static_cast<uint8_t>(PingResult::SendFailed), std::memory_order_relaxed);
    if (uart) {
        char buf[128];
        kernel::util::k_snprintf(buf, sizeof(buf),
                                 "[hmi] service online on eth%u raw=0x%04x udp=%u dhcp=%u\n",
                                 static_cast<unsigned>(self->nic_idx_),
                                 static_cast<unsigned>(ETHERTYPE),
                                 static_cast<unsigned>(self->udp_port_),
                                 self->config_.dhcp_enable ? 1u : 0u);
        uart->puts(buf);
        if (self->config_.dhcp_enable) {
            log_ip_line("[hmi] provisional ip=", self->config_.static_ip);
        } else {
            log_ip_line("[hmi] static ip=", self->config_.static_ip);
        }
        if (self->config_.ping_selftest_enable) {
            log_ip_line("[hmi] selftest target=", self->config_.ping_selftest_target);
        }
    }
    Context ctx{self, nic};
    for (;;) {
        self->maybe_start_dhcp(*nic);
        (void)nic->poll_rx(&rx_trampoline, &ctx, 8);
        self->process_ping(*nic);
        if (self->config_.ping_selftest_enable &&
            !self->ping_selftest_started_ &&
            now_us() >= self->ping_selftest_due_us_) {
            self->ping_selftest_started_ = true;
            if (uart) {
                char ip_buf[16];
                char line[160];
                kernel::util::uint32_to_ipv4_str(self->config_.ping_selftest_target,
                                                 std::span<char>(ip_buf, sizeof(ip_buf)));
                const bool queued = self->begin_ping(self->config_.ping_selftest_target,
                                                     self->config_.ping_selftest_timeout_ms);
                kernel::util::k_snprintf(line, sizeof(line),
                                         "[hmi] selftest ping %s queued=%u timeout=%lums\n",
                                         ip_buf,
                                         queued ? 1u : 0u,
                                         static_cast<unsigned long>(self->config_.ping_selftest_timeout_ms));
                uart->puts(line);
            }
        }
        if (kernel::g_scheduler_ptr) kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        else kernel::util::cpu_relax();
    }
}

} // namespace hmi
