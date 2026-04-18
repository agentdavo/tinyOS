// SPDX-License-Identifier: MIT OR Apache-2.0

#ifndef HAL_SHARED_PCI_HPP
#define HAL_SHARED_PCI_HPP

#include <cstdint>

namespace hal::shared::pci {

struct HostBridge {
    uint64_t ecam_base = 0;
    uint64_t mmio_base = 0;
    uint64_t mmio_size = 0;
    uint8_t bus_start = 0;
    uint8_t bus_end = 0xff;
    bool valid = false;
};

struct DeviceAddress {
    uint8_t bus = 0;
    uint8_t device = 0;
    uint8_t function = 0;
};

struct DeviceInfo {
    DeviceAddress addr{};
    uint16_t vendor_id = 0xffff;
    uint16_t device_id = 0xffff;
    uint8_t revision = 0;
    uint8_t prog_if = 0;
    uint8_t subclass = 0;
    uint8_t class_code = 0;
    uint8_t header_type = 0;
    uint8_t irq_line = 0xff;
    uint64_t bar0 = 0;
    uint64_t bar0_size = 0;
    bool bar0_is_io = false;
    bool bar0_is_64 = false;
};

uint8_t read_config8(const HostBridge& host, DeviceAddress addr, uint16_t offset);
uint16_t read_config16(const HostBridge& host, DeviceAddress addr, uint16_t offset);
uint32_t read_config32(const HostBridge& host, DeviceAddress addr, uint16_t offset);
void write_config16(const HostBridge& host, DeviceAddress addr, uint16_t offset, uint16_t value);
void write_config32(const HostBridge& host, DeviceAddress addr, uint16_t offset, uint32_t value);
bool enable_memory_busmaster(const HostBridge& host, DeviceAddress addr);
bool probe_device(const HostBridge& host, DeviceAddress addr, DeviceInfo& out);
bool assign_bar0(const HostBridge& host, DeviceAddress addr, uint64_t cpu_addr);
bool find_device_by_vendor_device(const HostBridge& host,
                                  uint16_t vendor_id,
                                  uint16_t device_id,
                                  DeviceInfo& out);
bool find_device_by_class(const HostBridge& host,
                          uint8_t class_code,
                          uint8_t subclass,
                          uint8_t prog_if,
                          DeviceInfo& out);

} // namespace hal::shared::pci

#endif // HAL_SHARED_PCI_HPP
