// SPDX-License-Identifier: MIT OR Apache-2.0
// Runtime device database — populated by parsing TSV records.

#include "device_db.hpp"
#include "config/tsv.hpp"
#include "hal.hpp"
#include "util.hpp"

#include <cstring>

namespace devices {

DeviceDB g_device_db;

namespace {

bool sv_eq(std::string_view a, const char* b) noexcept {
    size_t bl = 0; while (b[bl]) ++bl;
    if (a.size() != bl) return false;
    for (size_t i = 0; i < bl; ++i) if (a[i] != b[i]) return false;
    return true;
}

// Parse an integer allowing a leading '-' sign. Supports decimal and 0x hex.
// Returns false if value is empty, starts with '#', or contains junk.
bool parse_signed(std::string_view s, int64_t& out) noexcept {
    if (s.empty()) return false;
    bool neg = false;
    size_t i = 0;
    if (s[0] == '-') { neg = true; i = 1; }
    if (i >= s.size()) return false;
    int base = 10;
    if (i + 1 < s.size() && s[i] == '0' && (s[i + 1] == 'x' || s[i + 1] == 'X')) {
        base = 16; i += 2;
    }
    if (i >= s.size()) return false;
    uint64_t acc = 0;
    for (; i < s.size(); ++i) {
        char c = s[i];
        uint64_t d;
        if (c >= '0' && c <= '9') d = static_cast<uint64_t>(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = 10 + static_cast<uint64_t>(c - 'a');
        else if (base == 16 && c >= 'A' && c <= 'F') d = 10 + static_cast<uint64_t>(c - 'A');
        else return false;
        if (d >= static_cast<uint64_t>(base)) return false;
        acc = acc * static_cast<uint64_t>(base) + d;
    }
    out = neg ? -static_cast<int64_t>(acc) : static_cast<int64_t>(acc);
    return true;
}

// Mask a signed value into `bits` bits (two's complement).
uint64_t mask_bits(int64_t v, uint8_t bits) noexcept {
    if (bits == 0 || bits >= 64) return static_cast<uint64_t>(v);
    uint64_t mask = (static_cast<uint64_t>(1) << bits) - 1;
    return static_cast<uint64_t>(v) & mask;
}

void copy_name(char (&dst)[32], std::string_view src) noexcept {
    size_t n = src.size();
    if (n > 31) n = 31;
    for (size_t i = 0; i < n; ++i) dst[i] = src[i];
    dst[n] = '\0';
}

struct LoadCtx {
    DeviceDB*    db;
    DeviceEntry* cur;   // currently-being-filled entry
};

void handle_device(LoadCtx& ctx, const config::Record& r) noexcept {
    ctx.cur = ctx.db->allocate_slot();
    if (!ctx.cur) {
        ctx.db->mark_truncated();
        return;
    }
    uint32_t u32;
    if (r.get_u32("vid", u32)) ctx.cur->id.vid = u32;
    if (r.get_u32("pid", u32)) ctx.cur->id.pid = u32;
    copy_name(ctx.cur->name, r.get("name"));
    if (r.get_u32("dc_assign", u32)) ctx.cur->dc_assign_activate = static_cast<uint16_t>(u32);
    if (r.get_u32("cycle_min_us", u32)) ctx.cur->cycle_min_us = u32;
    if (r.get_u32("cycle_quant_us", u32)) ctx.cur->cycle_quant_us = u32;
    if (r.get_u32("ebus_current_ma", u32))
        ctx.cur->ebus_current_ma = static_cast<uint16_t>(u32);

    // Classify from the TSV `type=` key. Anything else defaults to Servo.
    const auto t = r.get("type");
    if (!t.empty()) {
        if      (t == "coupler")         ctx.cur->type = DeviceType::Coupler;
        else if (t == "encoder" ||
                 t == "encoder_input")   ctx.cur->type = DeviceType::EncoderInput;
        else if (t == "servo")           ctx.cur->type = DeviceType::Servo;
        else if (t == "digital_input")   ctx.cur->type = DeviceType::DigitalInput;
        else if (t == "digital_output")  ctx.cur->type = DeviceType::DigitalOutput;
        else if (t == "analog_input")    ctx.cur->type = DeviceType::AnalogInput;
        else if (t == "analog_output")   ctx.cur->type = DeviceType::AnalogOutput;
        else                              ctx.cur->type = DeviceType::Unknown;
    }
}

void handle_sdo_init(LoadCtx& ctx, const config::Record& r) noexcept {
    if (!ctx.cur) return;
    if (ctx.cur->num_sdo_init >= DeviceEntry::MAX_SDO_INIT) {
        ctx.db->mark_truncated();
        return;
    }
    // sdo_init records use positional (unkeyed) fields:
    //   sdo_init\t<index>\t<sub>\t<bits>\t<value>\t<comment...>
    // tsv.cpp treats bare tokens (no '=') as key=token, value=empty, so each
    // positional token appears in r.fields[i].key.
    if (r.num_fields < 4) return;
    SdoInit& s = ctx.cur->sdo_init[ctx.cur->num_sdo_init];
    int64_t v;
    if (!parse_signed(r.fields[0].key, v)) return;
    s.index = static_cast<uint16_t>(v);
    if (!parse_signed(r.fields[1].key, v)) return;
    s.sub = static_cast<uint8_t>(v);
    if (!parse_signed(r.fields[2].key, v)) return;
    s.bits = static_cast<uint8_t>(v);
    if (!parse_signed(r.fields[3].key, v)) return;
    s.value = mask_bits(v, s.bits);
    ++ctx.cur->num_sdo_init;
}

void handle_pdo(LoadCtx& ctx, const config::Record& r) noexcept {
    if (!ctx.cur) return;
    if (ctx.cur->num_pdo >= DeviceEntry::MAX_PDO) {
        ctx.db->mark_truncated();
        return;
    }
    PdoEntry& p = ctx.cur->pdo[ctx.cur->num_pdo];
    uint32_t u;
    if (!r.get_u32("pdo", u))   return;
    p.pdo_idx = static_cast<uint16_t>(u);
    if (!r.get_u32("entry", u)) return;
    p.entry_idx = static_cast<uint16_t>(u);
    if (!r.get_u32("sub", u)) u = 0;
    p.sub = static_cast<uint8_t>(u);
    if (!r.get_u32("bits", u)) return;
    p.bits = static_cast<uint8_t>(u);
    if (!r.get_u32("sm", u)) u = 0;
    p.sm = static_cast<uint8_t>(u);
    const auto pin = r.get("pin");
    if (!pin.empty()) {
        const size_t copy = pin.size() < sizeof(p.pin) ? pin.size() : sizeof(p.pin) - 1u;
        for (size_t i = 0; i < copy; ++i) p.pin[i] = pin[i];
        p.pin[copy] = '\0';
    } else {
        p.pin[0] = '\0';
    }

    auto dir = r.get("dir");
    if (sv_eq(dir, "rx") || sv_eq(dir, "out") || sv_eq(dir, "0")) {
        p.dir = 0;
    } else if (sv_eq(dir, "tx") || sv_eq(dir, "in") || sv_eq(dir, "1")) {
        p.dir = 1;
    } else {
        p.dir = 0;
    }
    ++ctx.cur->num_pdo;
}

void handle_od(LoadCtx& ctx, const config::Record& r) noexcept {
    if (!ctx.cur) return;
    if (ctx.cur->num_od >= DeviceEntry::MAX_OD) {
        ctx.db->mark_truncated();
        return;
    }
    // od records use positional fields:
    //   od\t<index>\t<sub>\t<bits>\t<type>\t<access>\t<default>\t<note...>
    if (r.num_fields < 6) return;
    OdParam& o = ctx.cur->od[ctx.cur->num_od];
    int64_t v;
    if (!parse_signed(r.fields[0].key, v)) return;
    o.index = static_cast<uint16_t>(v);
    if (!parse_signed(r.fields[1].key, v)) return;
    o.sub = static_cast<uint8_t>(v);
    if (!parse_signed(r.fields[2].key, v)) return;
    o.bits = static_cast<uint8_t>(v);
    // r.fields[3].key = type string (u32/i16/str21/...) — skipped.
    std::string_view acc = r.fields[4].key;
    o.access = sv_eq(acc, "rw") ? 1 : 0;

    std::string_view deflt = r.fields[5].key;
    if (deflt.empty() || (deflt.size() == 1 && deflt[0] == '-')) {
        o.default_value = 0;
        o.has_default = false;
    } else if (parse_signed(deflt, v)) {
        o.default_value = mask_bits(v, o.bits);
        o.has_default = true;
    } else {
        // Non-numeric default (e.g. "12800|51200" multi-variant) — treat as unset.
        o.default_value = 0;
        o.has_default = false;
    }
    ++ctx.cur->num_od;
}

void record_cb(const config::Record& r, void* ctx_) noexcept {
    auto* ctx = static_cast<LoadCtx*>(ctx_);
    if (sv_eq(r.type, "device"))        handle_device(*ctx, r);
    else if (sv_eq(r.type, "sdo_init")) handle_sdo_init(*ctx, r);
    else if (sv_eq(r.type, "pdo"))      handle_pdo(*ctx, r);
    else if (sv_eq(r.type, "od"))       handle_od(*ctx, r);
    // Unknown record types are silently ignored.
}

} // namespace

DeviceEntry* DeviceDB::allocate_slot() noexcept {
    if (num_devices_ >= MAX_DEVICES) return nullptr;
    DeviceEntry* e = &devices_[num_devices_++];
    std::memset(static_cast<void*>(e), 0, sizeof(DeviceEntry));
    return e;
}

bool DeviceDB::load_tsv(const char* buf, size_t len) noexcept {
    LoadCtx ctx{ this, nullptr };
    return config::parse(buf, len, &record_cb, &ctx);
}

const DeviceEntry* DeviceDB::find(DeviceId id) const noexcept {
    for (size_t i = 0; i < num_devices_; ++i) {
        if (devices_[i].id.vid == id.vid && devices_[i].id.pid == id.pid) {
            return &devices_[i];
        }
    }
    return nullptr;
}

bool DeviceDB::get_od_default(DeviceId id, uint16_t index, uint8_t sub,
                              uint64_t& out_value) const noexcept {
    const DeviceEntry* d = find(id);
    if (!d) return false;
    for (size_t i = 0; i < d->num_od; ++i) {
        const auto& o = d->od[i];
        if (o.index == index && o.sub == sub && o.has_default) {
            out_value = o.default_value;
            return true;
        }
    }
    return false;
}

void DeviceDB::dump(kernel::hal::UARTDriverOps* uart) const {
    if (!uart) return;
    char line[96];
    kernel::util::k_snprintf(line, sizeof(line),
                             "devices (%zu loaded):\n", num_devices_);
    uart->puts(line);
    for (size_t i = 0; i < num_devices_; ++i) {
        const DeviceEntry& d = devices_[i];
        kernel::util::k_snprintf(line, sizeof(line),
                                 "  [%zu] vid=0x%04X pid=0x%04X name=%s\n",
                                 i,
                                 static_cast<unsigned>(d.id.vid),
                                 static_cast<unsigned>(d.id.pid),
                                 d.name);
        uart->puts(line);
        kernel::util::k_snprintf(line, sizeof(line),
                                 "      sdo_init=%zu pdo=%zu od=%zu cycle_min=%uus\n",
                                 d.num_sdo_init, d.num_pdo, d.num_od,
                                 static_cast<unsigned>(d.cycle_min_us));
        uart->puts(line);
    }
    if (truncated_) {
        uart->puts("  (warning: one or more fixed-size limits were exceeded)\n");
    }
}

} // namespace devices
