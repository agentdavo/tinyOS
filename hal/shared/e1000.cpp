// SPDX-License-Identifier: MIT OR Apache-2.0
// Intel e1000/e1000e NIC driver implementation - low latency optimized.

#include "hal/shared/e1000.hpp"
#include "util.hpp"
#include <cstring>

namespace hal::shared::e1000 {

alignas(64) static uint8_t rx_buffer_mem[NUM_RX_DESC * RX_BUF_SIZE];
alignas(64) static uint8_t tx_buffer_mem[NUM_TX_DESC * TX_BUF_SIZE];
alignas(64) static RxDesc rx_ring_mem[NUM_RX_DESC];
alignas(64) static TxDesc tx_ring_mem[NUM_TX_DESC];

static bool loopback_mode = false;

E1000Driver::E1000Driver() = default;

E1000Driver::~E1000Driver() = default;

inline void e1000_prefetch(const void* ptr) {
#if defined(__aarch64__)
    asm volatile("prfm pldl1keep, [%0]" : : "r"(ptr));
#else
    (void)ptr;
#endif
}

inline void e1000_dmb() {
#if defined(__aarch64__)
    asm volatile("dmb sy" : : : "memory");
#elif defined(__riscv)
    asm volatile("fence" : : : "memory");
#endif
}

bool E1000Driver::init_pci(uint16_t, uint16_t,
                           uintptr_t mmio_base, uint8_t irq) {
    mmio_base_ = mmio_base;
    irq_ = irq;

    uint32_t devid = read_reg(0x02);
    if (devid != DEV_E1000 && devid != DEV_E1000E && devid != DEV_IGB) {
        return false;
    }

    rx_ring_ = rx_ring_mem;
    tx_ring_ = tx_ring_mem;

    std::memset(rx_ring_, 0, NUM_RX_DESC * sizeof(RxDesc));
    std::memset(tx_ring_, 0, NUM_TX_DESC * sizeof(TxDesc));

    for (uint32_t i = 0; i < NUM_RX_DESC; ++i) {
        rx_buffers_[i] = &rx_buffer_mem[i * RX_BUF_SIZE];
    }

    for (uint32_t i = 0; i < NUM_TX_DESC; ++i) {
        tx_buffers_[i] = &tx_buffer_mem[i * TX_BUF_SIZE];
    }

    init_rx_ring();
    init_tx_ring();
    init_mac();

    uint32_t ctrl = read_reg(E1000_CTRL);
    ctrl |= (1 << 5);
    write_reg(E1000_CTRL, ctrl);

    write_reg(E1000_IMC, 0xFFFFFFFF);

    initialized_ = true;
    return true;
}

void E1000Driver::enable_loopback() noexcept {
    if (!initialized_) return;
    uint32_t ctrl = read_reg(E1000_CTRL);
    ctrl |= CTRL_LR | CTRL_SLU;
    write_reg(E1000_CTRL, ctrl);
    loopback_mode = true;
}

bool E1000Driver::init_interface(int) {
    return initialized_;
}

bool E1000Driver::send_packet(int, const uint8_t* data, size_t len) {
    if (!initialized_ || len > TX_BUF_SIZE) {
        tx_drops_++;
        return false;
    }

    uint32_t idx = tx_head_;
    if ((tx_ring_[idx].status & TX_STA_DD) == 0) {
        return false;
    }

    e1000_prefetch(data);
    std::memcpy(tx_buffers_[idx], data, len);

    tx_ring_[idx].buffer_addr = reinterpret_cast<uint64_t>(tx_buffers_[idx]);
    tx_ring_[idx].length = static_cast<uint32_t>(len);
    tx_ring_[idx].cmd = CMD_EOP | CMD_IFCS | CMD_RS;
    tx_ring_[idx].status = 0;

    e1000_dmb();

    tx_head_ = (tx_head_ + 1) % NUM_TX_DESC;
    write_reg(E1000_TDT, tx_head_);

    if (loopback_mode) {
        rx_head_ = (rx_head_ + 1) % NUM_RX_DESC;
    }

    tx_packets_++;
    return true;
}

void E1000Driver::register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb,
                                            void* context) {
    callback_ = cb;
    callback_context_ = context;
}

kernel::hal::net::NicStats E1000Driver::get_stats() const {
    kernel::hal::net::NicStats s{};
    s.tx_packets = tx_packets_.load();
    s.tx_drops   = tx_drops_.load();
    s.rx_packets = rx_packets_.load();
    s.rx_drops   = rx_drops_.load();
    return s;
}

bool E1000Driver::get_mac(uint8_t out[6]) const {
    if (!out || !initialized_) return false;
    for (size_t i = 0; i < 6; ++i) out[i] = mac_[i];
    return true;
}

size_t E1000Driver::poll_rx(kernel::hal::net::PacketReceivedCallback cb,
                             void* ctx, size_t) {
    if (!cb) {
        return 0;
    }

    uint32_t rdt = read_reg(E1000_RDT);
    size_t received = 0;

    while (rx_head_ != rdt) {
        RxDesc& desc = rx_ring_[rx_head_];

        if ((desc.status & RX_STA_DD) == 0) {
            break;
        }

        if (desc.status & RX_STA_EOP) {
            size_t len = desc.length;
            if (len > 0 && len <= RX_BUF_SIZE) {
                cb(0, rx_buffers_[rx_head_], len, ctx);
                rx_packets_++;
                received++;
            } else {
                rx_drops_++;
            }
        }

        desc.status = 0;
        desc.buffer_addr = reinterpret_cast<uint64_t>(rx_buffers_[rx_head_]);

        rx_head_ = (rx_head_ + 1) % NUM_RX_DESC;
    }

    write_reg(E1000_RDT, rx_head_);
    return received;
}

void E1000Driver::write_reg(uint32_t offset, uint32_t val) {
    volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
        mmio_base_ + offset);
    *reg = val;
    (void)read_reg(E1000_STATUS);
}

uint32_t E1000Driver::read_reg(uint32_t offset) const {
    volatile uint32_t* reg = reinterpret_cast<volatile uint32_t*>(
        mmio_base_ + offset);
    return *reg;
}

void E1000Driver::init_rx_ring() {
    uint64_t rx_phys = reinterpret_cast<uint64_t>(rx_ring_);
    write_reg(E1000_RDBAL, static_cast<uint32_t>(rx_phys & 0xFFFFFFFF));
    write_reg(E1000_RDBAH, static_cast<uint32_t>(rx_phys >> 32));
    write_reg(E1000_RDLEN, NUM_RX_DESC * sizeof(RxDesc));
    write_reg(E1000_RDH, 0);
    write_reg(E1000_RDT, NUM_RX_DESC - 1);

    for (uint32_t i = 0; i < NUM_RX_DESC; ++i) {
        rx_ring_[i].buffer_addr = reinterpret_cast<uint64_t>(rx_buffers_[i]);
        rx_ring_[i].status = 0;
    }

    uint32_t rctl = RCTL_EN | RCTL_BAM;
    write_reg(E1000_RCTL, rctl);
}

void E1000Driver::init_tx_ring() {
    uint64_t tx_phys = reinterpret_cast<uint64_t>(tx_ring_);
    write_reg(E1000_TDBAL, static_cast<uint32_t>(tx_phys & 0xFFFFFFFF));
    write_reg(E1000_TDBAH, static_cast<uint32_t>(tx_phys >> 32));
    write_reg(E1000_TDLEN, NUM_TX_DESC * sizeof(TxDesc));
    write_reg(E1000_TDH, 0);
    write_reg(E1000_TDT, 0);

    tx_head_ = 0;
    tx_tail_ = 0;

    uint32_t tctl = TCTL_EN | TCTL_PSP;
    tctl |= (0x0F << 8);
    tctl |= (0x3 << 20);
    write_reg(E1000_TCTL, tctl);
}

void E1000Driver::init_mac() {
    uint8_t default_mac[6] = { 0x52, 0x54, 0x00, 0x12, 0x34, 0x56 };
    for (size_t i = 0; i < 6; ++i) mac_[i] = default_mac[i];

    uint32_t ral = static_cast<uint32_t>(default_mac[0]) |
                   (static_cast<uint32_t>(default_mac[1]) << 8) |
                   (static_cast<uint32_t>(default_mac[2]) << 16) |
                   (static_cast<uint32_t>(default_mac[3]) << 24);
    uint32_t rah = static_cast<uint32_t>(default_mac[4]) |
                   (static_cast<uint32_t>(default_mac[5]) << 8) | (1 << 31);

    write_reg(E1000_RAL0, ral);
    write_reg(E1000_RAH0, rah);

    write_reg(0x5210, 0);
    write_reg(0x5214, 0);
    write_reg(0x5218, 0);
    write_reg(0x521C, 0);
    write_reg(0x5220, 0);
    write_reg(0x5224, 0);
    write_reg(0x5228, 0);
    write_reg(0x522C, 0);
}

} // namespace hal::shared::e1000
