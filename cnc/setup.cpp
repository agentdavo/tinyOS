// SPDX-License-Identifier: MIT OR Apache-2.0

#include "setup.hpp"

#include "../core.hpp"
#include "../fs/vfs.hpp"
#include "../hal.hpp"
#include "../miniOS.hpp"
#include "../motion/motion.hpp"
#include "../util.hpp"
#include "interpreter.hpp"
#include "offsets.hpp"

#include <cstddef>
#include <cstdint>

namespace cnc::setup {

namespace {

// Bytes are budgeted for ~32 PEC points/axis × 4 axes + tool table +
// work offsets + active selections + comp metadata. Comfortable headroom
// for forward-compatible additions before the format needs revisiting.
constexpr size_t kMaxBufBytes = 16 * 1024;

alignas(8) char g_save_buf[kMaxBufBytes];
alignas(8) char g_load_buf[kMaxBufBytes];

void log(const char* msg) noexcept {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (uart) uart->puts(msg);
}

// k_snprintf has no %f; serialize floats as signed micro-units (×1e6) so
// the file stays integer + base-10 and the round-trip preserves all the
// precision a single-precision float carries.
int32_t to_micro(float v) noexcept {
    if (v >= 0.0f) return static_cast<int32_t>(v * 1000000.0f + 0.5f);
    return static_cast<int32_t>(v * 1000000.0f - 0.5f);
}
float from_micro(int32_t u) noexcept { return static_cast<float>(u) / 1000000.0f; }

// Append-formatted-line helper. Truncates on overflow rather than failing
// the whole save — the file is still parseable, just missing the tail.
struct Writer {
    char* buf;
    size_t cap;
    size_t used = 0;

    template <typename... Args>
    bool printf(const char* fmt, Args... args) noexcept {
        if (used >= cap) return false;
        const int n = kernel::util::k_snprintf(buf + used, cap - used, fmt, args...);
        if (n <= 0) return false;
        used += static_cast<size_t>(n);
        if (used > cap) used = cap;
        return true;
    }
};

void write_active(Writer& w) noexcept {
    const auto interp = cnc::interp::g_runtime.snapshot(0);
    uint16_t feed_pm = 1000;
    uint16_t rapid_pm = 1000;
    if (motion::g_motion.channel_count() > 0) {
        const auto& ch = motion::g_motion.channel(0);
        feed_pm = ch.overrides.feed_permille;
        rapid_pm = ch.overrides.rapid_permille;
    }
    w.printf("[active]\n");
    w.printf("wcs=%u\n", static_cast<unsigned>(cnc::offsets::g_service.active_work()));
    w.printf("tool=%u\n", static_cast<unsigned>(cnc::offsets::g_service.active_tool()));
    w.printf("units=%s\n", interp.inch_mode ? "inch" : "mm");
    w.printf("feed_permille=%u\n", static_cast<unsigned>(feed_pm));
    w.printf("rapid_permille=%u\n", static_cast<unsigned>(rapid_pm));
    w.printf("\n");
}

void write_wcs(Writer& w) noexcept {
    static const char kAxis[cnc::offsets::AXIS_COUNT] = {'X', 'Y', 'Z', 'A'};
    const auto& work = cnc::offsets::g_service.work_offsets();
    w.printf("[wcs]\n");
    for (size_t i = 0; i < work.size(); ++i) {
        for (size_t a = 0; a < cnc::offsets::AXIS_COUNT; ++a) {
            w.printf("%s.%c=%ld\n", work[i].name, kAxis[a],
                     static_cast<long>(to_micro(work[i].value.axis[a])));
        }
    }
    w.printf("\n");
}

void write_tools(Writer& w) noexcept {
    const auto& tool = cnc::offsets::g_service.tool_offsets();
    w.printf("[tool]\n");
    for (size_t i = 0; i < tool.size(); ++i) {
        const unsigned id = static_cast<unsigned>(tool[i].tool);
        w.printf("T%u.length=%ld\n", id, static_cast<long>(to_micro(tool[i].length)));
        w.printf("T%u.radius=%ld\n", id, static_cast<long>(to_micro(tool[i].radius)));
        w.printf("T%u.wear=%ld\n",   id, static_cast<long>(to_micro(tool[i].wear)));
    }
    w.printf("\n");
}

void write_pec(Writer& w) noexcept {
    static const char kAxisName[4] = {'X', 'Y', 'Z', 'A'};
    for (uint8_t axis = 0; axis < 4; ++axis) {
        const uint16_t n = motion::g_motion.linear_cal_point_count(axis);
        if (n == 0 && !motion::g_motion.linear_cal_enabled(axis)) continue;
        w.printf("[pec.%c]\n", kAxisName[axis]);
        w.printf("enabled=%u\n", motion::g_motion.linear_cal_enabled(axis) ? 1u : 0u);
        for (uint16_t i = 0; i < n; ++i) {
            const auto p = motion::g_motion.linear_cal_point(axis, i);
            w.printf("point.%u=%ld:%ld\n", static_cast<unsigned>(i),
                     static_cast<long>(p.position_counts),
                     static_cast<long>(p.error_counts));
        }
        w.printf("\n");
    }
}

void write_geometry(Writer& w) noexcept {
    w.printf("[geometry]\n");
    w.printf("xy_urad=%ld\n", static_cast<long>(motion::g_motion.get_geometry_error("XY")));
    w.printf("xz_urad=%ld\n", static_cast<long>(motion::g_motion.get_geometry_error("XZ")));
    w.printf("yz_urad=%ld\n", static_cast<long>(motion::g_motion.get_geometry_error("YZ")));
    w.printf("enabled=%u\n", motion::g_motion.geometry_enabled() ? 1u : 0u);
    w.printf("\n");
}

void write_sphere(Writer& w) noexcept {
    w.printf("[sphere]\n");
    w.printf("enabled=%u\n", motion::g_motion.sphere_enabled() ? 1u : 0u);
    w.printf("diameter_um=%ld\n",
             static_cast<long>(to_micro(motion::g_motion.sphere_diameter())));
    w.printf("probe_radius_um=%ld\n", static_cast<long>(motion::g_motion.sphere_probe_radius()));
    w.printf("probe_hits=%ld\n", static_cast<long>(motion::g_motion.sphere_probe_hits()));
    w.printf("rapid_speed_mm_min=%ld\n", static_cast<long>(motion::g_motion.sphere_rapid_speed()));
    w.printf("probe_speed_mm_min=%ld\n", static_cast<long>(motion::g_motion.sphere_probe_speed()));
    w.printf("\n");
}

// ===== Parsing =====

bool eq(const char* a, const char* b) noexcept { return kernel::util::kstrcmp(a, b) == 0; }
bool starts_with(const char* s, const char* prefix) noexcept {
    while (*prefix) { if (*s++ != *prefix++) return false; }
    return true;
}

// Trim trailing CR/spaces in place.
void rtrim(char* s) noexcept {
    size_t n = kernel::util::kstrlen(s);
    while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t' || s[n-1] == '\r')) {
        s[--n] = '\0';
    }
}

// Best-effort signed long parse. Stops at first non-digit (post sign).
// Returns true if at least one digit was consumed.
bool parse_long(const char* s, long& out) noexcept {
    if (!s) return false;
    while (*s == ' ' || *s == '\t') ++s;
    bool neg = false;
    if (*s == '-') { neg = true; ++s; }
    else if (*s == '+') { ++s; }
    if (!kernel::util::isdigit(*s)) return false;
    long v = 0;
    while (kernel::util::isdigit(*s)) {
        v = v * 10 + (*s - '0');
        ++s;
    }
    out = neg ? -v : v;
    return true;
}

bool parse_long_pair(const char* s, long& a, long& b) noexcept {
    long va = 0;
    if (!s) return false;
    while (*s == ' ' || *s == '\t') ++s;
    bool neg = false;
    if (*s == '-') { neg = true; ++s; } else if (*s == '+') { ++s; }
    if (!kernel::util::isdigit(*s)) return false;
    long v = 0;
    while (kernel::util::isdigit(*s)) { v = v * 10 + (*s - '0'); ++s; }
    va = neg ? -v : v;
    if (*s != ':') return false;
    ++s;
    if (!parse_long(s, b)) return false;
    a = va;
    return true;
}

// Walk text line-by-line, callback gets (section, key, value). The line
// buffer is reused per line so callbacks must copy data they want to
// keep.
using LineCb = void (*)(const char* section, const char* key, const char* value, void* user);

void walk_ini(const char* text, size_t len, LineCb cb, void* user) noexcept {
    char line[256];
    char section[64] = "";
    size_t i = 0;
    while (i < len) {
        size_t lp = 0;
        while (i < len && text[i] != '\n' && lp + 1 < sizeof(line)) {
            line[lp++] = text[i++];
        }
        line[lp] = '\0';
        // Skip the rest of an over-long line.
        while (i < len && text[i] != '\n') ++i;
        if (i < len) ++i;  // consume '\n'
        rtrim(line);
        if (line[0] == '\0' || line[0] == '#' || line[0] == ';') continue;
        if (line[0] == '[') {
            // [section]
            size_t j = 1;
            size_t sp = 0;
            while (line[j] && line[j] != ']' && sp + 1 < sizeof(section)) {
                section[sp++] = line[j++];
            }
            section[sp] = '\0';
            continue;
        }
        // key=value
        char* eq_pos = nullptr;
        for (size_t k = 0; line[k]; ++k) {
            if (line[k] == '=') { eq_pos = &line[k]; break; }
        }
        if (!eq_pos) continue;
        *eq_pos = '\0';
        const char* key = line;
        const char* val = eq_pos + 1;
        cb(section, key, val, user);
    }
}

struct LoadCtx {
    size_t applied = 0;
    size_t unknown_keys = 0;
    size_t unknown_sections = 0;
};

void apply_active(const char* key, const char* val, LoadCtx& ctx) noexcept {
    long n = 0;
    if (eq(key, "wcs")) {
        if (parse_long(val, n)) { (void)cnc::offsets::g_service.select_work(static_cast<size_t>(n)); ++ctx.applied; }
    } else if (eq(key, "tool")) {
        if (parse_long(val, n)) { (void)cnc::offsets::g_service.select_tool(static_cast<size_t>(n)); ++ctx.applied; }
    } else if (eq(key, "feed_permille")) {
        if (parse_long(val, n) && motion::g_motion.channel_count() > 0) {
            (void)motion::g_motion.set_override(0, motion::Kernel::OverrideKind::Feed,
                                                static_cast<uint16_t>(n));
            ++ctx.applied;
        }
    } else if (eq(key, "rapid_permille")) {
        if (parse_long(val, n) && motion::g_motion.channel_count() > 0) {
            (void)motion::g_motion.set_override(0, motion::Kernel::OverrideKind::Rapid,
                                                static_cast<uint16_t>(n));
            ++ctx.applied;
        }
    } else if (eq(key, "units")) {
        // Units are interpreter-mode driven (G20/G21); there's no public
        // setter on cnc::interp::Runtime, so we don't apply it here. Saved
        // for diagnostic round-trip only.
        ++ctx.applied;
    } else {
        ++ctx.unknown_keys;
    }
}

// Decode "G54.X" / "G55.A" -> (work_idx, axis_idx). Returns true on
// recognized labels, false otherwise.
bool decode_wcs_key(const char* key, size_t& work_out, size_t& axis_out) noexcept {
    if (!key || key[0] != 'G' || !kernel::util::isdigit(key[1]) || !kernel::util::isdigit(key[2])) return false;
    if (key[3] != '.' || key[4] == '\0' || key[5] != '\0') return false;
    const int wcs = (key[1] - '0') * 10 + (key[2] - '0');
    if (wcs < 54 || wcs > 59) return false;
    work_out = static_cast<size_t>(wcs - 54);
    switch (key[4]) {
        case 'X': axis_out = 0; return true;
        case 'Y': axis_out = 1; return true;
        case 'Z': axis_out = 2; return true;
        case 'A': axis_out = 3; return true;
        default:  return false;
    }
}

void apply_wcs(const char* key, const char* val, LoadCtx& ctx) noexcept {
    size_t w = 0, a = 0;
    long u = 0;
    if (!decode_wcs_key(key, w, a) || !parse_long(val, u)) {
        ++ctx.unknown_keys;
        return;
    }
    (void)cnc::offsets::g_service.set_work_axis(w, a, from_micro(static_cast<int32_t>(u)));
    ++ctx.applied;
}

// Decode "T1.length" / "T8.wear" -> (tool_idx, field). Field encoding:
// 0 = length, 1 = radius, 2 = wear.
bool decode_tool_key(const char* key, size_t& idx_out, int& field_out) noexcept {
    if (!key || key[0] != 'T') return false;
    size_t i = 1;
    long v = 0;
    if (!kernel::util::isdigit(key[i])) return false;
    while (kernel::util::isdigit(key[i])) { v = v * 10 + (key[i] - '0'); ++i; }
    if (key[i] != '.') return false;
    const char* tail = &key[i + 1];
    if (eq(tail, "length")) { field_out = 0; }
    else if (eq(tail, "radius")) { field_out = 1; }
    else if (eq(tail, "wear")) { field_out = 2; }
    else return false;
    // Tool ids in cnc::offsets are 1-based in the .tool field but slot
    // index is 0..TOOL_OFFSET_COUNT-1; treat the saved id as a slot
    // index via (id - 1) with clamp.
    if (v <= 0) return false;
    idx_out = static_cast<size_t>(v - 1);
    return idx_out < cnc::offsets::TOOL_OFFSET_COUNT;
}

void apply_tool(const char* key, const char* val, LoadCtx& ctx) noexcept {
    size_t idx = 0; int field = 0;
    long u = 0;
    if (!decode_tool_key(key, idx, field) || !parse_long(val, u)) {
        ++ctx.unknown_keys;
        return;
    }
    const auto& tools = cnc::offsets::g_service.tool_offsets();
    float length = tools[idx].length;
    float radius = tools[idx].radius;
    float wear   = tools[idx].wear;
    const float v = from_micro(static_cast<int32_t>(u));
    switch (field) {
        case 0: length = v; break;
        case 1: radius = v; break;
        case 2: wear   = v; break;
    }
    (void)cnc::offsets::g_service.set_tool_value(idx, length, radius, wear);
    ++ctx.applied;
}

uint8_t section_pec_axis(const char* section) noexcept {
    if (!starts_with(section, "pec.")) return 0xFF;
    switch (section[4]) {
        case 'X': return 0;
        case 'Y': return 1;
        case 'Z': return 2;
        case 'A': return 3;
        default:  return 0xFF;
    }
}

void apply_pec(uint8_t axis, const char* key, const char* val, LoadCtx& ctx) noexcept {
    if (eq(key, "enabled")) {
        long n = 0;
        if (parse_long(val, n)) { (void)motion::g_motion.enable_linear_cal(axis, n != 0); ++ctx.applied; }
        return;
    }
    if (starts_with(key, "point.")) {
        long pos = 0, err = 0;
        if (parse_long_pair(val, pos, err)) {
            (void)motion::g_motion.add_cal_point(axis,
                                                 static_cast<int32_t>(pos),
                                                 static_cast<int32_t>(err));
            ++ctx.applied;
        }
        return;
    }
    ++ctx.unknown_keys;
}

void apply_geometry(const char* key, const char* val, LoadCtx& ctx) noexcept {
    long n = 0;
    if (!parse_long(val, n)) { ++ctx.unknown_keys; return; }
    if (eq(key, "xy_urad")) { (void)motion::g_motion.set_geometry_error("XY", static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "xz_urad")) { (void)motion::g_motion.set_geometry_error("XZ", static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "yz_urad")) { (void)motion::g_motion.set_geometry_error("YZ", static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "enabled")) { (void)motion::g_motion.enable_geometry(n != 0); ++ctx.applied; }
    else { ++ctx.unknown_keys; }
}

void apply_sphere(const char* key, const char* val, LoadCtx& ctx) noexcept {
    long n = 0;
    if (!parse_long(val, n)) { ++ctx.unknown_keys; return; }
    if (eq(key, "enabled")) { motion::g_motion.sphere_enable(n != 0); ++ctx.applied; }
    else if (eq(key, "diameter_um")) { motion::g_motion.sphere_set_diameter(from_micro(static_cast<int32_t>(n))); ++ctx.applied; }
    else if (eq(key, "probe_radius_um")) { motion::g_motion.sphere_set_probe_radius(static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "probe_hits")) { motion::g_motion.sphere_set_probe_hits(static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "rapid_speed_mm_min")) { motion::g_motion.sphere_set_rapid_speed(static_cast<int32_t>(n)); ++ctx.applied; }
    else if (eq(key, "probe_speed_mm_min")) { motion::g_motion.sphere_set_probe_speed(static_cast<int32_t>(n)); ++ctx.applied; }
    else { ++ctx.unknown_keys; }
}

void load_dispatch(const char* section, const char* key, const char* value, void* user) noexcept {
    auto* ctx = static_cast<LoadCtx*>(user);
    if (eq(section, "version")) {
        // Version metadata is informational; format=1 is the only thing
        // emitted today. Accept anything to ease forward migration.
        ++ctx->applied;
        return;
    }
    if (eq(section, "active"))   { apply_active(key, value, *ctx); return; }
    if (eq(section, "wcs"))      { apply_wcs(key, value, *ctx); return; }
    if (eq(section, "tool"))     { apply_tool(key, value, *ctx); return; }
    if (eq(section, "geometry")) { apply_geometry(key, value, *ctx); return; }
    if (eq(section, "sphere"))   { apply_sphere(key, value, *ctx); return; }
    const uint8_t pec_axis = section_pec_axis(section);
    if (pec_axis != 0xFF)        { apply_pec(pec_axis, key, value, *ctx); return; }
    ++ctx->unknown_sections;
}

}  // namespace

bool save_to(const char* path) noexcept {
    if (!path || !*path) return false;
    Writer w{g_save_buf, sizeof(g_save_buf)};
    w.printf("[version]\n");
    w.printf("format=1\n\n");
    write_active(w);
    write_wcs(w);
    write_tools(w);
    write_pec(w);
    write_geometry(w);
    write_sphere(w);

    bool persistent = false;
    const bool ok = kernel::vfs::write_blob(path, g_save_buf, w.used, &persistent);
    char msg[128];
    kernel::util::k_snprintf(msg, sizeof(msg),
                             "[setup] save '%s' %s (%u bytes, %s)\n",
                             path,
                             ok ? "ok" : "FAILED",
                             static_cast<unsigned>(w.used),
                             persistent ? "persistent" : "in-ram (FAT32 read-only)");
    log(msg);
    return ok;
}

bool load_from(const char* path) noexcept {
    if (!path || !*path) return false;
    const char* data = nullptr;
    size_t size = 0;
    if (!kernel::vfs::lookup(path, data, size) || !data || size == 0) {
        char msg[96];
        kernel::util::k_snprintf(msg, sizeof(msg),
                                 "[setup] load '%s' not found (skipping)\n", path);
        log(msg);
        return false;
    }
    if (size > sizeof(g_load_buf)) size = sizeof(g_load_buf);
    for (size_t i = 0; i < size; ++i) g_load_buf[i] = data[i];
    LoadCtx ctx{};
    walk_ini(g_load_buf, size, &load_dispatch, &ctx);
    char msg[160];
    kernel::util::k_snprintf(msg, sizeof(msg),
                             "[setup] load '%s' applied=%u unknown_keys=%u unknown_sections=%u\n",
                             path,
                             static_cast<unsigned>(ctx.applied),
                             static_cast<unsigned>(ctx.unknown_keys),
                             static_cast<unsigned>(ctx.unknown_sections));
    log(msg);
    return ctx.applied > 0;
}

}  // namespace cnc::setup
