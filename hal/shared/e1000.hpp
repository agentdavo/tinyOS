// SPDX-License-Identifier: MIT OR Apache-2.0
// Intel e1000/e1000e NIC driver.
//
// Supports QEMU PCI passthrough of e1000 NICs. The driver implements
// the NetworkDriverOps interface for EtherCAT frame TX/RX.
//
// Register layout (e1000 MMIO):
//   0x0000 - Device Control (CTRL)
//   0x0008 - Device Status (STATUS)
//   0x0010 - Interrupt Cause Read (ICR)
//   0x0018 - Interrupt Mask Set/Read (IMS)
//   0x0020 - Interrupt Mask Set/Read (IMC)
//   0x4000 - RX Descriptor Base (RDBAL/RDBAH)
//   0x5000 - TX Descriptor Base (TDBAL/TDBAH)
//   0x7000 - RX Control (RCTL)
//   0x8000 - TX Control (TCTL)
//
// PCI config space:
//   0x00 - Vendor ID (0x8086 for Intel)
//   0x02 - Device ID (0x100E for e1000, 0x10D3 for e1000e)

#ifndef HAL_SHARED_E1000_HPP
#define HAL_SHARED_E1000_HPP

#include "hal.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace hal::shared::e1000 {

// e1000 register offsets (MMIO).
constexpr uint32_t E1000_CTRL     = 0x0000;
constexpr uint32_t E1000_STATUS  = 0x0008;
constexpr uint32_t E1000_ICR     = 0x0010;
constexpr uint32_t E1000_IMS     = 0x0018;
constexpr uint32_t E1000_IMC     = 0x0020;
constexpr uint32_t E1000_RDBAL   = 0x4000;
constexpr uint32_t E1000_RDBAH   = 0x4004;
constexpr uint32_t E1000_RDLEN   = 0x4008;
constexpr uint32_t E1000_RDH     = 0x4010;
constexpr uint32_t E1000_RDT     = 0x4018;
constexpr uint32_t E1000_TDBAL   = 0x5000;
constexpr uint32_t E1000_TDBAH   = 0x5004;
constexpr uint32_t E1000_TDLEN   = 0x5008;
constexpr uint32_t E1000_TDH     = 0x5010;
constexpr uint32_t E1000_TDT     = 0x5018;
constexpr uint32_t E1000_RCTL    = 0x7000;
constexpr uint32_t E1000_TCTL    = 0x8000;
constexpr uint32_t E1000_RA      = 0x8400;
constexpr uint32_t E1000_RAL0   = 0x8400;
constexpr uint32_t E1000_RAH0   = 0x8404;
constexpr uint32_t E1000_MTA    = 0x5200;

// RX/TX descriptor.
struct [[gnu::packed]] TxDesc {
    volatile uint64_t buffer_addr;
    volatile uint32_t length;
    volatile uint16_t csum_offset;
    volatile uint16_t cmd;
    volatile uint8_t  status;
    volatile uint8_t  css;
    volatile uint16_t special;
    volatile uint16_t reserved;
};

struct [[gnu::packed]] RxDesc {
    volatile uint64_t buffer_addr;
    volatile uint16_t length;
    volatile uint16_t header_buffer_length;
    volatile uint8_t  status;
    volatile uint8_t  errors;
    volatile uint16_t special;
    volatile uint16_t reserved;
};

// Descriptor command/eop byte.
constexpr uint8_t CMD_EOP   = 0x01;
constexpr uint8_t CMD_IFCS  = 0x02;
constexpr uint8_t CMD_IC    = 0x04;
constexpr uint8_t CMD_RS   = 0x08;
constexpr uint8_t CMD_VLE  = 0x40;
constexpr uint8_t CMD_IDE  = 0x80;

// RX status byte.
constexpr uint8_t RX_STA_DD  = 0x01;
constexpr uint8_t RX_STA_EOP = 0x02;
constexpr uint8_t RX_STA_EOF = 0x04;
constexpr uint8_t RX_STA_FEOS = 0x10;

// TX status byte.
constexpr uint8_t TX_STA_DD  = 0x01;

// RX Control bits.
constexpr uint32_t RCTL_EN      = 0x00000002;
constexpr uint32_t RCTL_BAM     = 0x00008000;
constexpr uint32_t RCTL_UPE     = 0x00040000;
constexpr uint32_t RCTL_MPE     = 0x00080000;

// TX Control bits.
constexpr uint32_t TCTL_EN      = 0x00000002;
constexpr uint32_t TCTL_PSP    = 0x00000008;
constexpr uint32_t TCTL_CT     = 0x00000F00;
constexpr uint32_t TCTL_COLD   = 0x00300000;

// CTRL register bits for loopback.
constexpr uint32_t CTRL_LR      = 0x00000008;
constexpr uint32_t CTRL_ASDE    = 0x00000020;
constexpr uint32_t CTRL_SLU     = 0x00000040;

// Interrupt cause bits.
constexpr uint32_t ICR_TXDW    = 0x00000001;
constexpr uint32_t ICR_TXQE    = 0x00000002;
constexpr uint32_t ICR_LSC     = 0x00000004;
constexpr uint32_t ICR_RXSEQ   = 0x00000008;
constexpr uint32_t ICR_RXDMT0  = 0x00000010;
constexpr uint32_t ICR_RXO     = 0x00000040;
constexpr uint32_t ICR_RXT0    = 0x00000080;
constexpr uint32_t ICR_TXDD    = 0x00020000;
constexpr uint32_t ICR_RXDD    = 0x00040000;

// Intel vendor ID.
constexpr uint16_t INTEL_VENDOR = 0x8086;

// e1000 device IDs (common ones).
constexpr uint16_t DEV_E1000    = 0x100E; // 82540EM
constexpr uint16_t DEV_E1000E   = 0x10D3; // 82574L
constexpr uint16_t DEV_IGB      = 0x10C9; // IGB

// RX/TX ring size.
constexpr uint32_t NUM_RX_DESC = 64;
constexpr uint32_t NUM_TX_DESC = 64;
constexpr uint32_t RX_BUF_SIZE = 2048;
constexpr uint32_t TX_BUF_SIZE = 2048;

class E1000Driver : public kernel::hal::net::NetworkDriverOps {
public:
    E1000Driver();
    ~E1000Driver() override;

    bool init_interface(int if_idx) override;
    bool send_packet(int if_idx, const uint8_t* data, size_t len) override;
    void register_packet_receiver(kernel::hal::net::PacketReceivedCallback cb,
                                   void* context) override;
    kernel::hal::net::NicStats get_stats() const override;
    bool get_mac(uint8_t out[6]) const override;
    size_t poll_rx(kernel::hal::net::PacketReceivedCallback cb, void* context,
                   size_t budget = 8) override;

    bool init_pci(uint16_t vendor, uint16_t device,
                  uintptr_t mmio_base, uint8_t irq);
    void enable_loopback() noexcept;
    bool is_initialized() const { return initialized_; }

private:
    void write_reg(uint32_t offset, uint32_t val);
    uint32_t read_reg(uint32_t offset) const;
    void init_rx_ring();
    void init_tx_ring();
    void init_mac();

    uintptr_t mmio_base_ = 0;
    uint8_t  irq_ = 0;
    bool     initialized_ = false;
    uint8_t  mac_[6] = {};

    kernel::hal::net::PacketReceivedCallback callback_ = nullptr;
    void* callback_context_ = nullptr;

    RxDesc* rx_ring_ = nullptr;
    TxDesc* tx_ring_ = nullptr;
    uint8_t* rx_buffers_[NUM_RX_DESC] = {};
    uint8_t* tx_buffers_[NUM_TX_DESC] = {};

    uint32_t rx_head_ = 0;
    uint32_t tx_head_ = 0;
    uint32_t tx_tail_ = 0;

    std::atomic<uint64_t> tx_packets_{0};
    std::atomic<uint64_t> tx_drops_{0};
    std::atomic<uint64_t> rx_packets_{0};
    std::atomic<uint64_t> rx_drops_{0};
};

} // namespace hal::shared::e1000

#endif // HAL_SHARED_E1000_HPP
