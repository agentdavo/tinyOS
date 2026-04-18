// SPDX-License-Identifier: MIT OR Apache-2.0
// Tiny tab-separated config parser. One line per record; first field is the
// record type (e.g. `master`, `slave`, `pdo`, `axis`); remaining fields are
// `key=value` pairs separated by tabs.
//
// Example:
//   master   id=0   netif=0   period_ns=1000000   dc=1
//   slave    master=0   idx=1  type=generic  name=ax0  vid=0x2  pid=0x13ed3052
//   pdo      slave=ax0  sm=3   dir=out  pdo=0x1600  entry=0x607a  sub=0x00 ...
//
// Records are parsed in-place without heap allocation; the caller provides a
// callback that sees the tokenised record.

#ifndef CONFIG_TSV_HPP
#define CONFIG_TSV_HPP

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace config {

constexpr size_t MAX_FIELDS = 16;

struct Record {
    std::string_view type;
    struct Field { std::string_view key; std::string_view value; };
    Field fields[MAX_FIELDS];
    size_t num_fields = 0;

    // Look up a field by key. Returns empty if missing.
    std::string_view get(std::string_view key) const noexcept;

    // Convenience: parse a hex-or-decimal integer field.
    bool get_u32(std::string_view key, uint32_t& out) const noexcept;
    bool get_u64(std::string_view key, uint64_t& out) const noexcept;
};

using RecordCallback = void (*)(const Record& rec, void* ctx);

// Parses `buf` of `len` bytes (no trailing null required). `cb` is called for
// each record; lines starting with `#` and blank lines are skipped. Returns
// true on clean parse, false if any line has too many fields or a malformed
// key=value.
bool parse(const char* buf, size_t len, RecordCallback cb, void* ctx) noexcept;

} // namespace config

#endif
