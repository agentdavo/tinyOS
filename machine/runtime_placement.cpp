// SPDX-License-Identifier: MIT OR Apache-2.0

#include "runtime_placement.hpp"

#include "config/tsv.hpp"

namespace machine::placement {

namespace {

bool eq(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

bool assign_key(Config& cfg, std::string_view key, uint32_t value) noexcept {
    if (value > 0xFFu) return false;
    const uint8_t v = static_cast<uint8_t>(value);
    if (eq(key, "cli_core")) cfg.cli_core = v;
    else if (eq(key, "uart_io_core")) cfg.uart_io_core = v;
    else if (eq(key, "ui_core")) cfg.ui_core = v;
    else if (eq(key, "hmi_irq_core")) cfg.hmi_irq_core = v;
    else if (eq(key, "motion_core")) cfg.motion_core = v;
    else if (eq(key, "gcode_core")) cfg.gcode_core = v;
    else if (eq(key, "macro_core")) cfg.macro_core = v;
    else if (eq(key, "ladder_core")) cfg.ladder_core = v;
    else if (eq(key, "probe_core")) cfg.probe_core = v;
    else if (eq(key, "bus_config_core")) cfg.bus_config_core = v;
    else if (eq(key, "ec_a_core")) cfg.ec_a_core = v;
    else if (eq(key, "ec_b_core")) cfg.ec_b_core = v;
    else if (eq(key, "fake_slave_core")) cfg.fake_slave_core = v;
    else if (eq(key, "hmi_nic")) cfg.hmi_nic = v;
    else if (eq(key, "ec_a_nic")) cfg.ec_a_nic = v;
    else if (eq(key, "ec_b_nic")) cfg.ec_b_nic = v;
    else if (eq(key, "fake_slave_nic")) cfg.fake_slave_nic = v;
    else if (eq(key, "lan0_irq_core")) cfg.ec_a_core = v; // legacy alias
    else if (eq(key, "lan1_irq_core")) cfg.ec_b_core = v; // legacy alias
    else return false;
    return true;
}

struct LoadCtx {
    Config cfg{};
    bool ok = true;
};

void record_cb(const config::Record& rec, void* raw_ctx) noexcept {
    auto* ctx = static_cast<LoadCtx*>(raw_ctx);
    if (!ctx || !eq(rec.type, "placement")) return;
    const auto key = rec.get("key");
    uint32_t value = 0;
    if (key.empty() || !rec.get_u32("value", value) || !assign_key(ctx->cfg, key, value)) {
        ctx->ok = false;
    }
}

} // namespace

Service g_service;

bool Service::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    LoadCtx ctx;
    const bool parsed = config::parse(buf, len, &record_cb, &ctx);
    if (!parsed || !ctx.ok) return false;
    kernel::core::ScopedLock lock(lock_);
    config_ = ctx.cfg;
    return true;
}

void Service::snapshot(Config& out) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    out = config_;
}

uint8_t Service::sanitize_core(uint8_t requested, uint32_t num_cores) const noexcept {
    if (num_cores == 0) return 0;
    return requested < num_cores ? requested : static_cast<uint8_t>(num_cores - 1u);
}

uint8_t Service::sanitize_nic(uint8_t requested, uint32_t num_nics) const noexcept {
    if (num_nics == 0) return 0;
    return requested < num_nics ? requested : static_cast<uint8_t>(num_nics - 1u);
}

} // namespace machine::placement
