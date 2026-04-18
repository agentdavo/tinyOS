// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "core.hpp"

#include <cstddef>
#include <cstdint>

namespace machine::placement {

struct Config {
    uint8_t cli_core = 0;
    uint8_t uart_io_core = 0;
    uint8_t ui_core = 0;
    uint8_t hmi_irq_core = 0;
    uint8_t motion_core = 1;
    uint8_t gcode_core = 0;
    uint8_t macro_core = 0;
    uint8_t ladder_core = 0;
    uint8_t probe_core = 0;
    uint8_t bus_config_core = 0;
    uint8_t ec_a_core = 2;
    uint8_t ec_b_core = 3;
    uint8_t fake_slave_core = 3;
    uint8_t hmi_nic = 0;
    uint8_t ec_a_nic = 1;
    uint8_t ec_b_nic = 2;
    uint8_t fake_slave_nic = 2;
};

class Service {
public:
    bool load_tsv(const char* buf, size_t len) noexcept;
    void snapshot(Config& out) const noexcept;
    uint8_t sanitize_core(uint8_t requested, uint32_t num_cores) const noexcept;
    uint8_t sanitize_nic(uint8_t requested, uint32_t num_nics) const noexcept;

private:
    Config config_{};
    mutable kernel::core::Spinlock lock_;
};

extern Service g_service;

} // namespace machine::placement
