// SPDX-License-Identifier: MIT OR Apache-2.0

#include "hal/shared/pci.hpp"

namespace hal::shared::pci {
namespace {

uintptr_t config_addr(const HostBridge& host, DeviceAddress addr, uint16_t offset) {
    return static_cast<uintptr_t>(host.ecam_base) +
           (static_cast<uintptr_t>(addr.bus) << 20) +
           (static_cast<uintptr_t>(addr.device) << 15) +
           (static_cast<uintptr_t>(addr.function) << 12) +
           (offset & 0xfffu);
}

} // namespace

uint8_t read_config8(const HostBridge& host, DeviceAddress addr, uint16_t offset) {
    if (!host.valid) return 0xff;
    return *reinterpret_cast<volatile uint8_t*>(config_addr(host, addr, offset));
}

uint16_t read_config16(const HostBridge& host, DeviceAddress addr, uint16_t offset) {
    if (!host.valid) return 0xffff;
    return *reinterpret_cast<volatile uint16_t*>(config_addr(host, addr, offset));
}

uint32_t read_config32(const HostBridge& host, DeviceAddress addr, uint16_t offset) {
    if (!host.valid) return 0xffffffffu;
    return *reinterpret_cast<volatile uint32_t*>(config_addr(host, addr, offset));
}

void write_config16(const HostBridge& host, DeviceAddress addr, uint16_t offset, uint16_t value) {
    if (!host.valid) return;
    *reinterpret_cast<volatile uint16_t*>(config_addr(host, addr, offset)) = value;
}

void write_config32(const HostBridge& host, DeviceAddress addr, uint16_t offset, uint32_t value) {
    if (!host.valid) return;
    *reinterpret_cast<volatile uint32_t*>(config_addr(host, addr, offset)) = value;
}

bool enable_memory_busmaster(const HostBridge& host, DeviceAddress addr) {
    uint16_t command = read_config16(host, addr, 0x04);
    if (command == 0xffff) return false;
    command |= static_cast<uint16_t>((1u << 1) | (1u << 2));
    write_config16(host, addr, 0x04, command);
    return true;
}

bool probe_device(const HostBridge& host, DeviceAddress addr, DeviceInfo& out) {
    const uint32_t id = read_config32(host, addr, 0x00);
    if (id == 0xffffffffu || (id & 0xffffu) == 0xffffu) return false;

    out = {};
    out.addr = addr;
    out.vendor_id = static_cast<uint16_t>(id & 0xffffu);
    out.device_id = static_cast<uint16_t>((id >> 16) & 0xffffu);
    const uint32_t classreg = read_config32(host, addr, 0x08);
    out.revision = static_cast<uint8_t>(classreg & 0xffu);
    out.prog_if = static_cast<uint8_t>((classreg >> 8) & 0xffu);
    out.subclass = static_cast<uint8_t>((classreg >> 16) & 0xffu);
    out.class_code = static_cast<uint8_t>((classreg >> 24) & 0xffu);
    out.header_type = static_cast<uint8_t>(read_config8(host, addr, 0x0e) & 0x7fu);
    out.irq_line = read_config8(host, addr, 0x3c);

    const uint32_t bar0_lo = read_config32(host, addr, 0x10);
    out.bar0_is_io = (bar0_lo & 0x1u) != 0;
    out.bar0_is_64 = !out.bar0_is_io && ((bar0_lo >> 1) & 0x3u) == 0x2u;
    if (!out.bar0_is_io) {
        if (out.bar0_is_64) {
            const uint32_t bar0_hi = read_config32(host, addr, 0x14);
            out.bar0 = (static_cast<uint64_t>(bar0_hi) << 32) | static_cast<uint64_t>(bar0_lo & ~0xfu);
        } else {
            out.bar0 = static_cast<uint64_t>(bar0_lo & ~0xfu);
        }
        const uint32_t saved_lo = bar0_lo;
        const uint32_t saved_hi = out.bar0_is_64 ? read_config32(host, addr, 0x14) : 0;
        write_config32(host, addr, 0x10, 0xffffffffu);
        if (out.bar0_is_64) write_config32(host, addr, 0x14, 0xffffffffu);
        const uint32_t sized_lo = read_config32(host, addr, 0x10);
        const uint32_t sized_hi = out.bar0_is_64 ? read_config32(host, addr, 0x14) : 0;
        write_config32(host, addr, 0x10, saved_lo);
        if (out.bar0_is_64) write_config32(host, addr, 0x14, saved_hi);
        uint64_t mask = out.bar0_is_64
            ? ((static_cast<uint64_t>(sized_hi) << 32) | static_cast<uint64_t>(sized_lo & ~0xfu))
            : static_cast<uint64_t>(sized_lo & ~0xfu);
        if (mask != 0) {
            out.bar0_size = (~mask) + 1u;
        }
    }
    return true;
}

bool assign_bar0(const HostBridge& host, DeviceAddress addr, uint64_t cpu_addr) {
    DeviceInfo info{};
    if (!probe_device(host, addr, info) || info.bar0_is_io) return false;
    const uint64_t value = info.bar0_is_64 ? (cpu_addr & ~0xfULL) : (cpu_addr & ~0xfULL);
    write_config32(host, addr, 0x10, static_cast<uint32_t>(value & 0xffffffffu));
    if (info.bar0_is_64) {
        write_config32(host, addr, 0x14, static_cast<uint32_t>(value >> 32));
    }
    return true;
}

bool find_device_by_vendor_device(const HostBridge& host,
                                  uint16_t vendor_id,
                                  uint16_t device_id,
                                  DeviceInfo& out) {
    if (!host.valid) return false;
    for (uint16_t bus = host.bus_start; bus <= host.bus_end; ++bus) {
        for (uint8_t dev = 0; dev < 32; ++dev) {
            DeviceAddress addr{static_cast<uint8_t>(bus), dev, 0};
            DeviceInfo fn0{};
            if (!probe_device(host, addr, fn0)) continue;
            const bool multifunction = (read_config8(host, addr, 0x0e) & 0x80u) != 0;
            if (fn0.vendor_id == vendor_id && fn0.device_id == device_id) {
                out = fn0;
                return true;
            }
            if (!multifunction) continue;
            for (uint8_t fn = 1; fn < 8; ++fn) {
                addr.function = fn;
                DeviceInfo info{};
                if (!probe_device(host, addr, info)) continue;
                if (info.vendor_id == vendor_id && info.device_id == device_id) {
                    out = info;
                    return true;
                }
            }
        }
        if (bus == 0xffu) break;
    }
    return false;
}

bool find_device_by_class(const HostBridge& host,
                          uint8_t class_code,
                          uint8_t subclass,
                          uint8_t prog_if,
                          DeviceInfo& out) {
    if (!host.valid) return false;
    for (uint16_t bus = host.bus_start; bus <= host.bus_end; ++bus) {
        for (uint8_t dev = 0; dev < 32; ++dev) {
            DeviceAddress addr{static_cast<uint8_t>(bus), dev, 0};
            DeviceInfo fn0{};
            if (!probe_device(host, addr, fn0)) continue;
            const bool multifunction = (read_config8(host, addr, 0x0e) & 0x80u) != 0;
            if (fn0.class_code == class_code && fn0.subclass == subclass && fn0.prog_if == prog_if) {
                out = fn0;
                return true;
            }
            if (!multifunction) continue;
            for (uint8_t fn = 1; fn < 8; ++fn) {
                addr.function = fn;
                DeviceInfo info{};
                if (!probe_device(host, addr, info)) continue;
                if (info.class_code == class_code && info.subclass == subclass && info.prog_if == prog_if) {
                    out = info;
                    return true;
                }
            }
        }
        if (bus == 0xffu) break;
    }
    return false;
}

} // namespace hal::shared::pci
