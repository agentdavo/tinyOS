// SPDX-License-Identifier: MIT OR Apache-2.0
#include "tsv.hpp"

#include <cstdint>

namespace config {

namespace {

bool eq(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) if (a[i] != b[i]) return false;
    return true;
}

bool parse_int(std::string_view s, uint64_t& out) noexcept {
    if (s.empty()) return false;
    size_t i = 0;
    int base = 10;
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        i = 2;
    }
    if (i >= s.size()) return false;
    uint64_t acc = 0;
    for (; i < s.size(); ++i) {
        char c = s[i];
        uint64_t d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (base == 16 && c >= 'a' && c <= 'f') d = 10 + (c - 'a');
        else if (base == 16 && c >= 'A' && c <= 'F') d = 10 + (c - 'A');
        else return false;
        if (d >= static_cast<uint64_t>(base)) return false;
        acc = acc * base + d;
    }
    out = acc;
    return true;
}

} // namespace

std::string_view Record::get(std::string_view key) const noexcept {
    for (size_t i = 0; i < num_fields; ++i) {
        if (eq(fields[i].key, key)) return fields[i].value;
    }
    return {};
}

bool Record::get_u32(std::string_view key, uint32_t& out) const noexcept {
    uint64_t v;
    if (!get_u64(key, v) || v > 0xFFFFFFFFu) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

bool Record::get_u64(std::string_view key, uint64_t& out) const noexcept {
    auto v = get(key);
    if (v.empty()) return false;
    return parse_int(v, out);
}

bool parse(const char* buf, size_t len, RecordCallback cb, void* ctx) noexcept {
    size_t i = 0;
    while (i < len) {
        // Line start.
        size_t line_start = i;
        while (i < len && buf[i] != '\n') ++i;
        size_t line_end = i;
        if (i < len) ++i; // consume '\n'
        // Trim trailing '\r'.
        if (line_end > line_start && buf[line_end - 1] == '\r') --line_end;
        // Skip blank / comment.
        size_t s = line_start;
        while (s < line_end && (buf[s] == ' ' || buf[s] == '\t')) ++s;
        if (s >= line_end) continue;
        if (buf[s] == '#') continue;
        // Tokenise on '\t'.
        Record rec;
        size_t ts = s;
        size_t te = ts;
        bool first = true;
        while (te <= line_end) {
            if (te == line_end || buf[te] == '\t') {
                std::string_view tok(&buf[ts], te - ts);
                if (first) { rec.type = tok; first = false; }
                else {
                    if (rec.num_fields >= MAX_FIELDS) return false;
                    // Split on first '='.
                    size_t eq_pos = 0;
                    while (eq_pos < tok.size() && tok[eq_pos] != '=') ++eq_pos;
                    if (eq_pos == tok.size() || eq_pos == 0) {
                        // Allow bare-flag fields (no '=') as key with empty value.
                        rec.fields[rec.num_fields++] = { tok, std::string_view{} };
                    } else {
                        rec.fields[rec.num_fields++] = {
                            tok.substr(0, eq_pos),
                            tok.substr(eq_pos + 1)
                        };
                    }
                }
                if (te == line_end) break;
                ts = te + 1;
            }
            ++te;
        }
        if (!rec.type.empty()) cb(rec, ctx);
    }
    return true;
}

} // namespace config
