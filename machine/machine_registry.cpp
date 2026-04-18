// SPDX-License-Identifier: MIT OR Apache-2.0

#include "machine_registry.hpp"

#include "cnc/interpreter.hpp"
#include "devices/device_db.hpp"
#include "ethercat/fake_slave.hpp"
#include "ethercat/master.hpp"
#include "ethercat/esm.hpp"
#include "machine_topology.hpp"
#include "motion/motion.hpp"
#include "toolpods.hpp"
#include "util.hpp"

namespace machine {

namespace {

bool streq(const char* a, const char* b) {
    return a && b && kernel::util::kstrcmp(a, b) == 0;
}

SymbolValue bool_value(bool v) {
    SymbolValue out{};
    out.bool_value = v;
    out.int_value = v ? 1 : 0;
    return out;
}

SymbolValue int_value(int32_t v) {
    SymbolValue out{};
    out.bool_value = v != 0;
    out.int_value = v;
    return out;
}

SymbolValue float_value(float v) {
    SymbolValue out{};
    out.bool_value = v != 0.0f;
    out.int_value = static_cast<int32_t>(v);
    out.float_value = v;
    return out;
}

const char* next_line(const char* p, const char* end, char* out, size_t out_size) {
    size_t i = 0;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    while (p < end && *p != '\r' && *p != '\n' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    while (p < end && *p != '\r' && *p != '\n') ++p;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    return p;
}

const char* field_value(const char* line, const char* key, char* scratch, size_t scratch_size) {
    if (!line || !key || !scratch || scratch_size == 0) return nullptr;
    const char* p = line;
    while (*p && *p != '\t') ++p;
    while (*p == '\t') {
        ++p;
        const char* field = p;
        while (*p && *p != '\t') ++p;
        const char* eq = field;
        while (eq < p && *eq != '=') ++eq;
        if (eq >= p) continue;
        const size_t key_len = static_cast<size_t>(eq - field);
        if (kernel::util::kstrlen(key) == key_len &&
            kernel::util::kmemcmp(field, key, key_len) == 0) {
            const char* value = eq + 1;
            const size_t value_len = static_cast<size_t>(p - value);
            const size_t copy = value_len + 1 < scratch_size ? value_len : scratch_size - 1;
            for (size_t i = 0; i < copy; ++i) scratch[i] = value[i];
            scratch[copy] = '\0';
            return scratch;
        }
    }
    return nullptr;
}

long parse_long(const char* s, long fallback = 0) {
    if (!s || !*s) return fallback;
    bool neg = false;
    if (*s == '-') { neg = true; ++s; }
    long value = 0;
    while (*s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        ++s;
    }
    return neg ? -value : value;
}

Registry::SignalSource parse_signal_source(const char* s) {
    if (!s) return Registry::SignalSource::None;
    if (streq(s, "ec.di")) return Registry::SignalSource::EcDi;
    if (streq(s, "ec.do")) return Registry::SignalSource::EcDo;
    if (streq(s, "ec.ai_uni")) return Registry::SignalSource::EcAiUni;
    if (streq(s, "ec.ai_bip")) return Registry::SignalSource::EcAiBip;
    if (streq(s, "ec.tx_bit")) return Registry::SignalSource::EcTxBit;
    if (streq(s, "ec.rx_bit")) return Registry::SignalSource::EcRxBit;
    if (streq(s, "ec.tx_s16")) return Registry::SignalSource::EcTxS16;
    if (streq(s, "ec.tx_u16")) return Registry::SignalSource::EcTxU16;
    if (streq(s, "ec.tx_s32")) return Registry::SignalSource::EcTxS32;
    if (streq(s, "ec.tx_u32")) return Registry::SignalSource::EcTxU32;
    if (streq(s, "ec.rx_s16")) return Registry::SignalSource::EcRxS16;
    if (streq(s, "ec.rx_u16")) return Registry::SignalSource::EcRxU16;
    if (streq(s, "ec.rx_s32")) return Registry::SignalSource::EcRxS32;
    if (streq(s, "ec.rx_u32")) return Registry::SignalSource::EcRxU32;
    if (streq(s, "ec.tx_pin")) return Registry::SignalSource::EcTxPin;
    if (streq(s, "ec.rx_pin")) return Registry::SignalSource::EcRxPin;
    if (streq(s, "ec.statusword")) return Registry::SignalSource::EcStatusWord;
    if (streq(s, "ec.controlword")) return Registry::SignalSource::EcControlWord;
    if (streq(s, "ec.position_actual")) return Registry::SignalSource::EcPositionActual;
    if (streq(s, "ec.velocity_actual")) return Registry::SignalSource::EcVelocityActual;
    if (streq(s, "ec.error_code")) return Registry::SignalSource::EcErrorCode;
    if (streq(s, "ec.drive_mode")) return Registry::SignalSource::EcDriveMode;
    if (streq(s, "ec.digital_inputs")) return Registry::SignalSource::EcDigitalInputs;
    if (streq(s, "ec.digital_outputs")) return Registry::SignalSource::EcDigitalOutputs;
    return Registry::SignalSource::None;
}

ethercat::Master* master_for_id(uint8_t id) {
    if (id == 0) return &ethercat::g_master_a;
    if (id == 1) return &ethercat::g_master_b;
    return nullptr;
}

const ethercat::SlaveInfo* bound_slave(uint8_t master_id, uint8_t slave_index) {
    auto* master = master_for_id(master_id);
    if (!master || slave_index >= ethercat::MAX_SLAVES) return nullptr;
    return &master->slave(slave_index);
}

ethercat::SlaveInfo* bound_slave_mut(uint8_t master_id, uint8_t slave_index) {
    auto* master = master_for_id(master_id);
    if (!master || slave_index >= ethercat::MAX_SLAVES) return nullptr;
    return &master->slave(slave_index);
}

const devices::DeviceEntry* bound_device(uint8_t master_id, uint8_t slave_index) {
    const auto* slave = bound_slave(master_id, slave_index);
    if (!slave) return nullptr;
    return devices::g_device_db.find({slave->observed_vid, slave->observed_pid});
}

machine::topology::Role role_for_slot(uint8_t master_id, uint8_t slave_index) {
    for (size_t i = 0; i < machine::topology::g_service.binding_count(); ++i) {
        const auto* binding = machine::topology::g_service.binding(i);
        if (!binding || !binding->used) continue;
        if (binding->master_id == master_id && binding->slave_index == slave_index) {
            return binding->role;
        }
    }
    return machine::topology::Role::Unknown;
}

bool first_slot_for_role(machine::topology::Role role, uint8_t& master_out, uint8_t& slave_out) {
    for (size_t i = 0; i < machine::topology::g_service.binding_count(); ++i) {
        const auto* binding = machine::topology::g_service.binding(i);
        if (!binding || !binding->used || binding->role != role) continue;
        master_out = binding->master_id;
        slave_out = binding->slave_index;
        return true;
    }
    return false;
}

bool read_slot_di_bit(uint8_t master_id, uint8_t slave_index, uint8_t bit) {
    const auto* slave = bound_slave(master_id, slave_index);
    if (!slave) return false;
    const auto role = role_for_slot(master_id, slave_index);
    if (role == machine::topology::Role::Servo) {
        return (slave->drive.digital_inputs & (1u << bit)) != 0;
    }
    const size_t byte = static_cast<size_t>(bit / 8u);
    if (byte >= slave->tx_process_data.size()) return false;
    return (slave->tx_process_data[byte] & static_cast<uint8_t>(1u << (bit & 7u))) != 0;
}

bool read_slot_do_bit(uint8_t master_id, uint8_t slave_index, uint8_t bit) {
    const auto* slave = bound_slave(master_id, slave_index);
    if (!slave) return false;
    const auto role = role_for_slot(master_id, slave_index);
    if (role == machine::topology::Role::Servo) {
        return (slave->drive.digital_outputs & (1u << bit)) != 0;
    }
    const size_t byte = static_cast<size_t>(bit / 8u);
    if (byte >= slave->rx_process_data.size()) return false;
    return (slave->rx_process_data[byte] & static_cast<uint8_t>(1u << (bit & 7u))) != 0;
}

void write_slot_do_bit(uint8_t master_id, uint8_t slave_index, uint8_t bit, bool value) {
    auto* slave = bound_slave_mut(master_id, slave_index);
    if (!slave) return;
    const auto role = role_for_slot(master_id, slave_index);
    if (role == machine::topology::Role::Servo) {
        if (value) slave->drive.digital_outputs |= (1u << bit);
        else slave->drive.digital_outputs &= ~(1u << bit);
        return;
    }
    const size_t byte = static_cast<size_t>(bit / 8u);
    if (byte >= slave->rx_process_data.size()) return;
    const uint8_t mask = static_cast<uint8_t>(1u << (bit & 7u));
    if (value) slave->rx_process_data[byte] = static_cast<uint8_t>(slave->rx_process_data[byte] | mask);
    else slave->rx_process_data[byte] = static_cast<uint8_t>(slave->rx_process_data[byte] & ~mask);
}

int32_t read_slot_ai_channel(uint8_t master_id, uint8_t slave_index, uint8_t channel) {
    const auto* slave = bound_slave(master_id, slave_index);
    if (!slave) return 0;
    const size_t off = static_cast<size_t>(channel) * 4u + 2u;
    if (off + 1u >= slave->tx_process_data.size()) return 0;
    const uint16_t raw = static_cast<uint16_t>(slave->tx_process_data[off]) |
                         static_cast<uint16_t>(static_cast<uint16_t>(slave->tx_process_data[off + 1u]) << 8);
    return static_cast<int16_t>(raw);
}

int32_t sign_extend(uint32_t value, uint8_t bits) {
    if (bits == 0 || bits >= 32) return static_cast<int32_t>(value);
    const uint32_t sign = 1u << (bits - 1u);
    const uint32_t mask = (1u << bits) - 1u;
    value &= mask;
    if ((value & sign) == 0) return static_cast<int32_t>(value);
    return static_cast<int32_t>(value | ~mask);
}

bool read_process_bits(uint8_t master_id, uint8_t slave_index, bool tx,
                       uint16_t bit_offset, uint8_t bit_width, bool is_signed,
                       SymbolValue& out) {
    const auto* slave = bound_slave(master_id, slave_index);
    if (!slave || bit_width == 0 || bit_width > 32) return false;
    const auto& image = tx ? slave->tx_process_data : slave->rx_process_data;
    const uint16_t byte_offset = static_cast<uint16_t>(bit_offset / 8u);
    const uint8_t start_bit = static_cast<uint8_t>(bit_offset & 7u);
    const uint8_t bytes_needed = static_cast<uint8_t>((start_bit + bit_width + 7u) / 8u);
    if (byte_offset + bytes_needed > image.size()) return false;
    uint64_t window = 0;
    for (uint8_t i = 0; i < bytes_needed; ++i) {
        window |= static_cast<uint64_t>(image[byte_offset + i]) << (8u * i);
    }
    window >>= start_bit;
    const uint32_t mask = bit_width == 32 ? 0xFFFFFFFFu : ((1u << bit_width) - 1u);
    const uint32_t raw = static_cast<uint32_t>(window) & mask;
    if (bit_width == 1) {
        out = bool_value(raw != 0);
    } else if (is_signed) {
        out = int_value(sign_extend(raw, bit_width));
    } else {
        out = int_value(static_cast<int32_t>(raw));
    }
    return true;
}

bool write_process_bits(uint8_t master_id, uint8_t slave_index,
                        uint16_t bit_offset, uint8_t bit_width, int32_t value) {
    auto* slave = bound_slave_mut(master_id, slave_index);
    if (!slave || bit_width == 0 || bit_width > 32) return false;
    auto& image = slave->rx_process_data;
    const uint16_t byte_offset = static_cast<uint16_t>(bit_offset / 8u);
    const uint8_t start_bit = static_cast<uint8_t>(bit_offset & 7u);
    const uint8_t bytes_needed = static_cast<uint8_t>((start_bit + bit_width + 7u) / 8u);
    if (byte_offset + bytes_needed > image.size()) return false;
    uint64_t window = 0;
    for (uint8_t i = 0; i < bytes_needed; ++i) {
        window |= static_cast<uint64_t>(image[byte_offset + i]) << (8u * i);
    }
    const uint64_t mask = ((bit_width == 32 ? 0xFFFFFFFFULL : ((1ULL << bit_width) - 1ULL)) << start_bit);
    window &= ~mask;
    window |= ((static_cast<uint64_t>(static_cast<uint32_t>(value)) &
                (bit_width == 32 ? 0xFFFFFFFFULL : ((1ULL << bit_width) - 1ULL))) << start_bit);
    for (uint8_t i = 0; i < bytes_needed; ++i) {
        image[byte_offset + i] = static_cast<uint8_t>((window >> (8u * i)) & 0xFFu);
    }
    return true;
}

bool resolve_pin_binding(const devices::DeviceEntry* dev, const char* pin_name,
                         bool tx, uint16_t& bit_offset_out, uint8_t& width_out) {
    if (!dev || !pin_name || !*pin_name) return false;
    uint16_t bit_offset = 0;
    for (size_t i = 0; i < dev->num_pdo; ++i) {
        const auto& p = dev->pdo[i];
        if ((p.dir == 1) != tx) continue;
        if (!streq(p.pin, pin_name)) {
            bit_offset = static_cast<uint16_t>(bit_offset + p.bits);
            continue;
        }
        bit_offset_out = bit_offset;
        width_out = p.bits;
        return true;
    }
    return false;
}

bool read_pin_binding(uint8_t master_id, uint8_t slave_index, const char* pin_name,
                      bool tx, SymbolValue& out) {
    const auto* dev = bound_device(master_id, slave_index);
    uint16_t bit_offset = 0;
    uint8_t width = 0;
    if (!resolve_pin_binding(dev, pin_name, tx, bit_offset, width)) return false;
    return read_process_bits(master_id, slave_index, tx, bit_offset, width, false, out);
}

bool write_pin_binding(uint8_t master_id, uint8_t slave_index, const char* pin_name, int32_t value) {
    const auto* dev = bound_device(master_id, slave_index);
    uint16_t bit_offset = 0;
    uint8_t width = 0;
    if (!resolve_pin_binding(dev, pin_name, false, bit_offset, width)) return false;
    return write_process_bits(master_id, slave_index, bit_offset, width, value);
}

} // namespace

Registry g_registry;

Registry::Registry() noexcept {
    bind_builtins();
}

void Registry::bind_builtins() noexcept {
    (void)define_symbol("macro_busy", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("macro_fault", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("macro_fault_code", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("cycle_inhibit", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("cycle_allowed", SymbolType::Bool, bool_value(true), true, true);
    (void)define_symbol("door_closed", SymbolType::Bool, bool_value(true), true, true);
    (void)define_symbol("door_open", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("drawer_open_cmd", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("drawer_close_cmd", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("drawer_open_fb", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("drawer_closed_fb", SymbolType::Bool, bool_value(true), true, true);
    (void)define_symbol("probe_tripped", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("probe_target_mode", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("probe_target_x", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("probe_target_y", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("probe_target_z", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("spindle_at_speed", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("probe_cal_active", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("probe_cal_fault", SymbolType::Bool, bool_value(false), true, true);
    (void)define_symbol("probe_cal_stage", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("probe_cal_point", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("probe_cal_hit", SymbolType::Int, int_value(0), true, true);
    (void)define_symbol("channel0_running", SymbolType::Bool, bool_value(false), false, true);
    (void)define_symbol("channel1_running", SymbolType::Bool, bool_value(false), false, true);
    (void)define_symbol("channel0_waiting", SymbolType::Bool, bool_value(false), false, true);
    (void)define_symbol("channel1_waiting", SymbolType::Bool, bool_value(false), false, true);
    (void)define_symbol("axis_x_pos", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("axis_y_pos", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("axis_z_pos", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("axis_a_pos", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_count", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_active_pod", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_active_station", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_virtual_tool", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_physical_tool", SymbolType::Int, int_value(0), false, true);
    (void)define_symbol("toolpod_locked", SymbolType::Bool, bool_value(true), true, true);
    for (size_t i = 0; i < 16; ++i) {
        char name[16];
        kernel::util::k_snprintf(name, sizeof(name), "di_%lu", static_cast<unsigned long>(i));
        (void)define_symbol(name, SymbolType::Bool, bool_value(false), false, true);
        kernel::util::k_snprintf(name, sizeof(name), "do_%lu", static_cast<unsigned long>(i));
        (void)define_symbol(name, SymbolType::Bool, bool_value(false), true, true);
    }
    for (size_t i = 0; i < 4; ++i) {
        char name[16];
        kernel::util::k_snprintf(name, sizeof(name), "ai_uni_%lu", static_cast<unsigned long>(i));
        (void)define_symbol(name, SymbolType::Int, int_value(0), false, true);
        kernel::util::k_snprintf(name, sizeof(name), "ai_bip_%lu", static_cast<unsigned long>(i));
        (void)define_symbol(name, SymbolType::Int, int_value(0), false, true);
    }
}

size_t Registry::count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return count_;
}

bool Registry::find_locked(const char* name, size_t& idx_out) const noexcept {
    if (!name || !*name) return false;
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].used && streq(entries_[i].name, name)) {
            idx_out = i;
            return true;
        }
    }
    return false;
}

const Registry::Entry* Registry::entry(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return idx < count_ ? &entries_[idx] : nullptr;
}

bool Registry::find(const char* name, size_t& idx_out) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return find_locked(name, idx_out);
}

bool Registry::define_symbol(const char* name, SymbolType type, SymbolValue initial,
                             bool writable, bool builtin) noexcept {
    if (!name || !*name) return false;
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].used && streq(entries_[i].name, name)) return true;
    }
    if (count_ >= MAX_SYMBOLS) return false;
    auto& entry = entries_[count_++];
    kernel::util::k_snprintf(entry.name, sizeof(entry.name), "%s", name);
    entry.type = type;
    entry.writable = writable;
    entry.builtin = builtin;
    entry.used = true;
    entry.value = initial;
    return true;
}

bool Registry::set_locked(size_t idx, SymbolValue value) noexcept {
    if (idx >= count_ || !entries_[idx].used) return false;
    switch (entries_[idx].type) {
        case SymbolType::Bool:
            entries_[idx].value = bool_value(value.bool_value || value.int_value != 0 || value.float_value != 0.0f);
            return true;
        case SymbolType::Int:
            entries_[idx].value = int_value(value.int_value);
            return true;
        case SymbolType::Float:
            entries_[idx].value = float_value(value.float_value);
            return true;
    }
    return false;
}

bool Registry::get_bool(const char* name, bool& out) const noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    out = entries_[idx].value.bool_value;
    return true;
}

bool Registry::get_int(const char* name, int32_t& out) const noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    out = entries_[idx].value.int_value;
    return true;
}

bool Registry::get_float(const char* name, float& out) const noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    out = entries_[idx].value.float_value;
    return true;
}

bool Registry::set_bool(const char* name, bool value) noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    if (!entries_[idx].writable) return false;
    return set_locked(idx, bool_value(value));
}

bool Registry::set_int(const char* name, int32_t value) noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    if (!entries_[idx].writable) return false;
    return set_locked(idx, int_value(value));
}

bool Registry::set_float(const char* name, float value) noexcept {
    size_t idx = 0;
    if (!find(name, idx)) return false;
    kernel::core::ScopedLock lock(lock_);
    if (!entries_[idx].writable) return false;
    return set_locked(idx, float_value(value));
}

bool Registry::load_signal_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    kernel::core::ScopedLock lock(lock_);
    for (auto& binding : signal_bindings_) binding = SignalBinding{};
    size_t binding_count = 0;
    const char* p = buf;
    const char* end = buf + len;
    char line[256];
    while (p < end) {
        p = next_line(p, end, line, sizeof(line));
        if (line[0] == '\0' || line[0] == '#') continue;
        char* rest = line;
        while (*rest && *rest != '\t') ++rest;
        if (*rest == '\t') *rest++ = '\0';
        if (!streq(line, "signal") || binding_count >= MAX_SIGNAL_BINDINGS) continue;
        char a[64], b[64], c[64], d[64];
        const char* name = field_value(rest, "name", a, sizeof(a));
        const auto source = parse_signal_source(field_value(rest, "source", b, sizeof(b)));
        const char* value_type = field_value(rest, "value_type", c, sizeof(c));
        const auto type = streq(value_type, "int") ? SymbolType::Int
            : (streq(value_type, "float") ? SymbolType::Float : SymbolType::Bool);
        const bool writable = parse_long(field_value(rest, "writable", d, sizeof(d)), 0) != 0;
        if (!name || source == SignalSource::None) continue;
        auto& binding = signal_bindings_[binding_count++];
        binding.used = true;
        kernel::util::k_snprintf(binding.name, sizeof(binding.name), "%s", name);
        binding.source = source;
        binding.type = type;
        const long master = parse_long(field_value(rest, "master", c, sizeof(c)), -1);
        const long slave = parse_long(field_value(rest, "slave", d, sizeof(d)), -1);
        binding.master = (master >= 0 && master <= 0xFF) ? static_cast<uint8_t>(master) : 0xFF;
        binding.slave = (slave >= 0 && slave <= 0xFF) ? static_cast<uint8_t>(slave) : 0xFF;
        binding.channel = static_cast<uint8_t>(parse_long(field_value(rest, "channel", a, sizeof(a)), 0));
        binding.bit = static_cast<uint8_t>(parse_long(field_value(rest, "bit", b, sizeof(b)), 0));
        binding.offset = static_cast<uint16_t>(parse_long(field_value(rest, "offset", c, sizeof(c)), 0));
        binding.width_bits = static_cast<uint8_t>(parse_long(field_value(rest, "width", d, sizeof(d)), 0));
        binding.writable = writable;
        if (const char* pin = field_value(rest, "pin", a, sizeof(a))) {
            kernel::util::k_snprintf(binding.pin, sizeof(binding.pin), "%s", pin);
        }
        size_t symbol_idx = 0;
        if (!find_locked(name, symbol_idx) && count_ < MAX_SYMBOLS) {
            auto& entry = entries_[count_++];
            kernel::util::k_snprintf(entry.name, sizeof(entry.name), "%s", name);
            entry.type = type;
            entry.writable = writable;
            entry.builtin = false;
            entry.used = true;
            entry.value = type == SymbolType::Int ? int_value(0) : bool_value(false);
        }
    }
    return binding_count != 0;
}

size_t Registry::signal_binding_count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    size_t count = 0;
    for (const auto& binding : signal_bindings_) count += binding.used ? 1u : 0u;
    return count;
}

const Registry::SignalBinding* Registry::signal_binding(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    size_t seen = 0;
    for (const auto& binding : signal_bindings_) {
        if (!binding.used) continue;
        if (seen++ == idx) return &binding;
    }
    return nullptr;
}

void Registry::update_io_locked() noexcept {
    uint16_t di = ethercat::g_fake_slave.dio_in();
    uint16_t dout = ethercat::g_fake_slave.dio_out();
    int32_t ai_uni[4] = {
        ethercat::g_fake_slave.uni(0),
        ethercat::g_fake_slave.uni(1),
        ethercat::g_fake_slave.uni(2),
        ethercat::g_fake_slave.uni(3),
    };
    int32_t ai_bip[4] = {
        ethercat::g_fake_slave.bip(0),
        ethercat::g_fake_slave.bip(1),
        ethercat::g_fake_slave.bip(2),
        ethercat::g_fake_slave.bip(3),
    };

    uint8_t master_id = 0;
    uint8_t slave_index = 0;
    if (first_slot_for_role(machine::topology::Role::DigitalInput, master_id, slave_index)) {
        di = 0;
        for (size_t i = 0; i < 16; ++i) {
            if (read_slot_di_bit(master_id, slave_index, static_cast<uint8_t>(i))) {
                di = static_cast<uint16_t>(di | (1u << i));
            }
        }
    }
    if (first_slot_for_role(machine::topology::Role::DigitalOutput, master_id, slave_index)) {
        dout = 0;
        for (size_t i = 0; i < 16; ++i) {
            if (read_slot_do_bit(master_id, slave_index, static_cast<uint8_t>(i))) {
                dout = static_cast<uint16_t>(dout | (1u << i));
            }
        }
    }
    if (first_slot_for_role(machine::topology::Role::AnalogInput, master_id, slave_index)) {
        for (size_t i = 0; i < 4; ++i) {
            ai_uni[i] = read_slot_ai_channel(master_id, slave_index, static_cast<uint8_t>(i));
            ai_bip[i] = ai_uni[i];
        }
    }

    for (size_t i = 0; i < 16; ++i) {
        char name[16];
        kernel::util::k_snprintf(name, sizeof(name), "di_%lu", static_cast<unsigned long>(i));
        size_t idx = 0;
        if (find_locked(name, idx)) entries_[idx].value = bool_value((di & (1u << i)) != 0);
        kernel::util::k_snprintf(name, sizeof(name), "do_%lu", static_cast<unsigned long>(i));
        if (find_locked(name, idx)) entries_[idx].value = bool_value((dout & (1u << i)) != 0);
    }
    for (size_t i = 0; i < 4; ++i) {
        char name[16];
        kernel::util::k_snprintf(name, sizeof(name), "ai_uni_%lu", static_cast<unsigned long>(i));
        size_t idx = 0;
        if (find_locked(name, idx)) entries_[idx].value = int_value(ai_uni[i]);
        kernel::util::k_snprintf(name, sizeof(name), "ai_bip_%lu", static_cast<unsigned long>(i));
        if (find_locked(name, idx)) entries_[idx].value = int_value(ai_bip[i]);
    }
}

void Registry::update_signal_inputs_locked() noexcept {
    for (const auto& binding : signal_bindings_) {
        if (!binding.used) continue;
        size_t idx = 0;
        if (!find_locked(binding.name, idx)) continue;
        const bool has_slot = binding.master != 0xFF && binding.slave != 0xFF;
        SymbolValue value{};
        bool handled = true;
        switch (binding.source) {
            case SignalSource::EcDi:
                value = bool_value(
                    has_slot ? read_slot_di_bit(binding.master, binding.slave, binding.bit)
                             : ((ethercat::g_fake_slave.dio_in() & (1u << binding.bit)) != 0));
                break;
            case SignalSource::EcAiUni:
                value = int_value(
                    has_slot ? read_slot_ai_channel(binding.master, binding.slave, binding.channel)
                             : ethercat::g_fake_slave.uni(binding.channel));
                break;
            case SignalSource::EcAiBip:
                value = int_value(
                    has_slot ? read_slot_ai_channel(binding.master, binding.slave, binding.channel)
                             : ethercat::g_fake_slave.bip(binding.channel));
                break;
            case SignalSource::EcTxBit:
                handled = has_slot && read_process_bits(binding.master, binding.slave, true,
                                                        binding.offset, binding.width_bits ? binding.width_bits : 1u,
                                                        false, value);
                break;
            case SignalSource::EcRxBit:
                handled = has_slot && read_process_bits(binding.master, binding.slave, false,
                                                        binding.offset, binding.width_bits ? binding.width_bits : 1u,
                                                        false, value);
                break;
            case SignalSource::EcTxS16:
                handled = has_slot && read_process_bits(binding.master, binding.slave, true, binding.offset, 16, true, value);
                break;
            case SignalSource::EcTxU16:
                handled = has_slot && read_process_bits(binding.master, binding.slave, true, binding.offset, 16, false, value);
                break;
            case SignalSource::EcTxS32:
                handled = has_slot && read_process_bits(binding.master, binding.slave, true, binding.offset, 32, true, value);
                break;
            case SignalSource::EcTxU32:
                handled = has_slot && read_process_bits(binding.master, binding.slave, true, binding.offset, 32, false, value);
                break;
            case SignalSource::EcRxS16:
                handled = has_slot && read_process_bits(binding.master, binding.slave, false, binding.offset, 16, true, value);
                break;
            case SignalSource::EcRxU16:
                handled = has_slot && read_process_bits(binding.master, binding.slave, false, binding.offset, 16, false, value);
                break;
            case SignalSource::EcRxS32:
                handled = has_slot && read_process_bits(binding.master, binding.slave, false, binding.offset, 32, true, value);
                break;
            case SignalSource::EcRxU32:
                handled = has_slot && read_process_bits(binding.master, binding.slave, false, binding.offset, 32, false, value);
                break;
            case SignalSource::EcTxPin:
                handled = has_slot && read_pin_binding(binding.master, binding.slave, binding.pin, true, value);
                break;
            case SignalSource::EcRxPin:
                handled = has_slot && read_pin_binding(binding.master, binding.slave, binding.pin, false, value);
                break;
            case SignalSource::EcStatusWord:
                if (has_slot) value = int_value(bound_slave(binding.master, binding.slave)->drive.statusword);
                else handled = false;
                break;
            case SignalSource::EcControlWord:
                if (has_slot) value = int_value(bound_slave(binding.master, binding.slave)->drive.controlword);
                else handled = false;
                break;
            case SignalSource::EcPositionActual:
                if (has_slot) value = int_value(bound_slave(binding.master, binding.slave)->drive.actual_position);
                else handled = false;
                break;
            case SignalSource::EcVelocityActual:
                if (has_slot) value = int_value(bound_slave(binding.master, binding.slave)->drive.actual_velocity);
                else handled = false;
                break;
            case SignalSource::EcErrorCode:
                if (has_slot) value = int_value(bound_slave(binding.master, binding.slave)->drive.error_code);
                else handled = false;
                break;
            case SignalSource::EcDriveMode:
                if (has_slot) value = int_value(static_cast<int32_t>(bound_slave(binding.master, binding.slave)->drive.mode_op_display));
                else handled = false;
                break;
            case SignalSource::EcDigitalInputs:
                if (has_slot) value = int_value(static_cast<int32_t>(bound_slave(binding.master, binding.slave)->drive.digital_inputs));
                else handled = false;
                break;
            case SignalSource::EcDigitalOutputs:
                if (has_slot) value = int_value(static_cast<int32_t>(bound_slave(binding.master, binding.slave)->drive.digital_outputs));
                else handled = false;
                break;
            case SignalSource::EcDo:
            case SignalSource::None:
                handled = false;
                break;
        }
        if (handled) entries_[idx].value = value;
    }
}

void Registry::update_signal_outputs_locked() noexcept {
    uint16_t do_bitmap = ethercat::g_fake_slave.dio_out();
    for (const auto& binding : signal_bindings_) {
        if (!binding.used || binding.source != SignalSource::EcDo) continue;
        size_t idx = 0;
        if (!find_locked(binding.name, idx)) continue;
        const bool bit_value = entries_[idx].value.bool_value;
        const int32_t int_value_raw = entries_[idx].value.int_value;
        if (binding.master != 0xFF && binding.slave != 0xFF) {
            switch (binding.source) {
                case SignalSource::EcDo:
                    write_slot_do_bit(binding.master, binding.slave, binding.bit, bit_value);
                    break;
                case SignalSource::EcRxBit:
                    (void)write_process_bits(binding.master, binding.slave, binding.offset,
                                             binding.width_bits ? binding.width_bits : 1u,
                                             bit_value ? 1 : 0);
                    break;
                case SignalSource::EcRxS16:
                case SignalSource::EcRxU16:
                    (void)write_process_bits(binding.master, binding.slave, binding.offset, 16, int_value_raw);
                    break;
                case SignalSource::EcRxS32:
                case SignalSource::EcRxU32:
                    (void)write_process_bits(binding.master, binding.slave, binding.offset, 32, int_value_raw);
                    break;
                case SignalSource::EcRxPin:
                    (void)write_pin_binding(binding.master, binding.slave, binding.pin, int_value_raw);
                    break;
                case SignalSource::EcControlWord: {
                    auto* slave = bound_slave_mut(binding.master, binding.slave);
                    if (slave) slave->drive.controlword = static_cast<uint16_t>(int_value_raw);
                    break;
                }
                case SignalSource::EcDigitalOutputs: {
                    auto* slave = bound_slave_mut(binding.master, binding.slave);
                    if (slave) slave->drive.digital_outputs = static_cast<uint32_t>(int_value_raw);
                    break;
                }
                default:
                    break;
            }
        } else {
            if (bit_value) do_bitmap = static_cast<uint16_t>(do_bitmap | (1u << binding.bit));
            else do_bitmap = static_cast<uint16_t>(do_bitmap & ~(1u << binding.bit));
        }
    }
    ethercat::g_fake_slave.set_do(do_bitmap);
}

void Registry::update_motion_locked() noexcept {
    static constexpr const char* kAxisNames[4] = {"axis_x_pos", "axis_y_pos", "axis_z_pos", "axis_a_pos"};
    for (size_t i = 0; i < 4; ++i) {
        size_t idx = 0;
        if (find_locked(kAxisNames[i], idx)) {
            entries_[idx].value = int_value(motion::g_motion.axis(i).actual_pos.load(std::memory_order_relaxed));
        }
    }
}

void Registry::update_interpreter_locked() noexcept {
    for (size_t ch = 0; ch < 2; ++ch) {
        const auto snap = cnc::interp::g_runtime.snapshot(ch);
        const bool running = snap.state == cnc::interp::State::Running ||
                             snap.state == cnc::interp::State::Dwell ||
                             snap.state == cnc::interp::State::WaitingBarrier ||
                             snap.state == cnc::interp::State::WaitingMacro;
        const bool waiting = snap.state == cnc::interp::State::WaitingBarrier ||
                             snap.state == cnc::interp::State::WaitingMacro;
        char name[24];
        kernel::util::k_snprintf(name, sizeof(name), "channel%lu_running", static_cast<unsigned long>(ch));
        size_t idx = 0;
        if (find_locked(name, idx)) entries_[idx].value = bool_value(running);
        kernel::util::k_snprintf(name, sizeof(name), "channel%lu_waiting", static_cast<unsigned long>(ch));
        if (find_locked(name, idx)) entries_[idx].value = bool_value(waiting);
    }
}

void Registry::update_toolpods_locked() noexcept {
    size_t idx = 0;
    if (find_locked("toolpod_count", idx)) {
        entries_[idx].value = int_value(static_cast<int32_t>(machine::toolpods::g_service.pod_count()));
    }
    const auto* pod = machine::toolpods::g_service.pod(0);
    if (!pod) return;
    if (find_locked("toolpod_active_pod", idx)) {
        entries_[idx].value = int_value(0);
    }
    if (find_locked("toolpod_locked", idx)) {
        entries_[idx].value = bool_value(pod->locked);
    }
    if (const auto* station = machine::toolpods::g_service.active_station(static_cast<size_t>(0))) {
        if (find_locked("toolpod_active_station", idx)) {
            entries_[idx].value = int_value(static_cast<int32_t>(station->index));
        }
        if (find_locked("toolpod_virtual_tool", idx)) {
            entries_[idx].value = int_value(static_cast<int32_t>(station->virtual_tool));
        }
        if (find_locked("toolpod_physical_tool", idx)) {
            entries_[idx].value = int_value(static_cast<int32_t>(station->physical_tool));
        }
    }
}

void Registry::update_derived_locked() noexcept {
    bool door_closed = true;
    int32_t spindle = 0;
    size_t idx = 0;
    if (find_locked("door_closed", idx)) door_closed = entries_[idx].value.bool_value;
    if (find_locked("axis_a_pos", idx)) spindle = entries_[idx].value.int_value;
    if (find_locked("door_open", idx)) entries_[idx].value = bool_value(!door_closed);
    if (find_locked("spindle_at_speed", idx)) entries_[idx].value = bool_value(spindle != 0);
    bool cycle_inhibit = false;
    bool macro_busy = false;
    if (find_locked("cycle_inhibit", idx)) cycle_inhibit = entries_[idx].value.bool_value;
    if (find_locked("macro_busy", idx)) macro_busy = entries_[idx].value.bool_value;
    if (find_locked("cycle_allowed", idx)) entries_[idx].value = bool_value(!cycle_inhibit && door_closed && !macro_busy);
    bool drawer_cmd = false;
    if (find_locked("drawer_open_cmd", idx)) drawer_cmd = entries_[idx].value.bool_value;
    if (find_locked("drawer_open_fb", idx) && drawer_cmd) entries_[idx].value = bool_value(true);
    if (find_locked("drawer_closed_fb", idx) && drawer_cmd) entries_[idx].value = bool_value(false);

    bool probe_sim_enable = false;
    bool probe_arm_request = false;
    int32_t probe_target_axis = -1;
    int32_t probe_target_pos = 0;
    int32_t probe_target_tol = 10;
    int32_t probe_target_mode = 0;
    int32_t probe_target_x = 0;
    int32_t probe_target_y = 0;
    int32_t probe_target_z = 0;
    if (find_locked("probe_sim_enable", idx)) probe_sim_enable = entries_[idx].value.bool_value;
    if (find_locked("probe_arm_request", idx)) probe_arm_request = entries_[idx].value.bool_value;
    if (find_locked("probe_target_axis", idx)) probe_target_axis = entries_[idx].value.int_value;
    if (find_locked("probe_target_pos", idx)) probe_target_pos = entries_[idx].value.int_value;
    if (find_locked("probe_target_tol", idx)) probe_target_tol = entries_[idx].value.int_value;
    if (find_locked("probe_target_mode", idx)) probe_target_mode = entries_[idx].value.int_value;
    if (find_locked("probe_target_x", idx)) probe_target_x = entries_[idx].value.int_value;
    if (find_locked("probe_target_y", idx)) probe_target_y = entries_[idx].value.int_value;
    if (find_locked("probe_target_z", idx)) probe_target_z = entries_[idx].value.int_value;
    if (!probe_sim_enable && find_locked("probe_tripped", idx)) {
        size_t di_idx = 0;
        if (find_locked("di_0", di_idx)) entries_[idx].value = bool_value(entries_[di_idx].value.bool_value);
    }
    if (probe_sim_enable && find_locked("probe_tripped", idx)) {
        bool trip = false;
        if (probe_arm_request) {
            if (probe_target_mode == 1) {
                const int32_t actual_x = motion::g_motion.axis(0).actual_pos.load(std::memory_order_relaxed);
                const int32_t actual_y = motion::g_motion.axis(1).actual_pos.load(std::memory_order_relaxed);
                const int32_t actual_z = motion::g_motion.axis(2).actual_pos.load(std::memory_order_relaxed);
                const int64_t dx = static_cast<int64_t>(actual_x) - probe_target_x;
                const int64_t dy = static_cast<int64_t>(actual_y) - probe_target_y;
                const int64_t dz = static_cast<int64_t>(actual_z) - probe_target_z;
                const int64_t tol2 = static_cast<int64_t>(probe_target_tol) * probe_target_tol;
                trip = (dx * dx + dy * dy + dz * dz) <= tol2;
            } else if (probe_target_axis >= 0 && probe_target_axis < 4) {
                const int32_t actual = motion::g_motion.axis(static_cast<size_t>(probe_target_axis))
                                          .actual_pos.load(std::memory_order_relaxed);
                const int32_t delta = actual - probe_target_pos;
                const int32_t abs_delta = delta < 0 ? -delta : delta;
                trip = abs_delta <= probe_target_tol;
            }
        }
        entries_[idx].value = bool_value(trip);
    }
}

void Registry::sync_from_runtime() noexcept {
    kernel::core::ScopedLock lock(lock_);
    update_io_locked();
    update_signal_inputs_locked();
    update_motion_locked();
    update_interpreter_locked();
    update_toolpods_locked();
    update_derived_locked();
    update_signal_outputs_locked();
}

} // namespace machine
