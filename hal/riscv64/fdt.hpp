// SPDX-License-Identifier: MIT OR Apache-2.0
// Minimal flattened-device-tree (FDT / DTB) parser for miniOS rv64.
//
// Freestanding, header-only. No heap, no libc. Just enough to walk the tree
// and pull out a u32 property. We use this on boot to read
// /cpus/timebase-frequency from the DTB QEMU passes in a1, so the CLINT
// timer driver doesn't have to hard-code the 10 MHz constant.
//
// Token stream (big-endian u32 on disk; we byteswap):
//   FDT_BEGIN_NODE (1) <name> (NUL-terminated, padded to 4)
//   FDT_END_NODE   (2)
//   FDT_PROP       (3) <len:u32> <nameoff:u32> <data[len]> (padded to 4)
//   FDT_NOP        (4) — skip
//   FDT_END        (9) — end of struct block
//
// Node depth convention below:
//   node_depth == 0  : before any BEGIN_NODE (haven't entered root yet)
//   node_depth == 1  : inside the implicit root node (name == "")
//   node_depth == 2  : inside a named top-level node (e.g. /cpus)
//   ...
// For a node_path "/cpus", the "current node" tip is at node_depth == 2.
// Properties of that node appear at node_depth == 2 too (between its
// BEGIN_NODE and its END_NODE / nested children). We track path_tip_depth
// = node_depth at which the full path matches, and on FDT_PROP we check
// node_depth == path_tip_depth.

#ifndef MINIOS_HAL_RV64_FDT_HPP
#define MINIOS_HAL_RV64_FDT_HPP

#include <cstdint>
#include <string_view>

namespace hal::qemu_virt_rv64::fdt {

// Global DTB pointer, stashed from a1 in cpu_rv64.S before caller-regs are
// zeroed. May be 0 or invalid if the bootloader didn't pass one.
extern "C" uint64_t g_dtb_ptr;

inline uint32_t load_be32(const uint8_t* p) {
    // Unaligned-safe load. DTB is 4-byte aligned in practice but we stay safe.
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
           (static_cast<uint32_t>(p[3]));
}

struct Header {
    uint32_t magic;
    uint32_t totalsize;
    uint32_t off_dt_struct;
    uint32_t off_dt_strings;
    uint32_t off_mem_rsvmap;
    uint32_t version;
    uint32_t last_comp_version;
    uint32_t boot_cpuid_phys;
    uint32_t size_dt_strings;
    uint32_t size_dt_struct;
};

constexpr uint32_t MAGIC          = 0xD00DFEEDu;
constexpr uint32_t FDT_BEGIN_NODE = 1;
constexpr uint32_t FDT_END_NODE   = 2;
constexpr uint32_t FDT_PROP       = 3;
constexpr uint32_t FDT_NOP        = 4;
constexpr uint32_t FDT_END        = 9;

// Lightweight view of a parsed DTB. Construct from the raw pointer; call
// valid() before use.
class Blob {
public:
    explicit Blob(const void* dtb) : base_(reinterpret_cast<const uint8_t*>(dtb)) {
        if (!base_) return;
        const uint8_t* p = base_;
        hdr_.magic            = load_be32(p + 0);
        hdr_.totalsize        = load_be32(p + 4);
        hdr_.off_dt_struct    = load_be32(p + 8);
        hdr_.off_dt_strings   = load_be32(p + 12);
        hdr_.off_mem_rsvmap   = load_be32(p + 16);
        hdr_.version          = load_be32(p + 20);
        hdr_.last_comp_version= load_be32(p + 24);
        hdr_.boot_cpuid_phys  = load_be32(p + 28);
        hdr_.size_dt_strings  = load_be32(p + 32);
        hdr_.size_dt_struct   = load_be32(p + 36);
    }

    bool valid() const { return base_ && hdr_.magic == MAGIC; }

    // Find a u32 property by path. The last segment is the property name;
    // everything before it is the node path (leading '/' selects root).
    // Examples: "/cpus/timebase-frequency", "/cpus/cpu@0/timebase-frequency".
    // Returns true on success, writing the host-order value to *out.
    bool get_u32(std::string_view path, uint32_t* out) const {
        if (!valid() || !out) return false;

        size_t last_slash = rfind_slash(path);
        if (last_slash == npos) return false;
        std::string_view node_path = path.substr(0, last_slash);
        std::string_view prop_name = path.substr(last_slash + 1);
        if (prop_name.empty()) return false;

        // Strip leading '/' from node_path. Root-only ("/") => empty segments.
        if (!node_path.empty() && node_path.front() == '/') {
            node_path.remove_prefix(1);
        }

        // Count segments and compute the tip depth. Tip depth is 1 (root
        // itself) + number of named segments.
        int segment_count = 0;
        {
            std::string_view p = node_path;
            while (!p.empty()) {
                size_t s = find_slash(p);
                if (s == npos) { segment_count++; break; }
                segment_count++;
                p = p.substr(s + 1);
            }
        }
        const int tip_depth = 1 + segment_count;

        const uint8_t* struct_base = base_ + hdr_.off_dt_struct;
        const uint8_t* struct_end  = struct_base + hdr_.size_dt_struct;
        const char*    strings_base= reinterpret_cast<const char*>(
                                         base_ + hdr_.off_dt_strings);
        const char*    strings_end = strings_base + hdr_.size_dt_strings;

        const uint8_t* cur = struct_base;

        // Walk state. path_matched_depth = depth of the deepest node we've
        // confirmed is on-path. Starts at 1 (root matches implicitly as
        // soon as we see the first BEGIN_NODE).
        int node_depth = 0;
        int path_matched_depth = 0;

        // Iterator over node_path segments; we bump it each time we
        // successfully match a segment while descending.
        std::string_view remaining = node_path;

        while (cur < struct_end) {
            uint32_t tok = load_be32(cur); cur += 4;
            if (tok == FDT_NOP) continue;
            if (tok == FDT_END) break;

            if (tok == FDT_BEGIN_NODE) {
                const char* name = reinterpret_cast<const char*>(cur);
                size_t nlen = cstr_len(name, struct_end);
                cur += align4(nlen + 1);
                node_depth++;

                if (node_depth == 1) {
                    // Root — always on-path. (FDT root has empty name.)
                    path_matched_depth = 1;
                    continue;
                }
                // We are considering a child node at this depth. It is a
                // candidate next-segment match only if we were on-path at
                // its parent depth.
                if (node_depth - 1 == path_matched_depth) {
                    std::string_view seg = pop_segment(remaining);
                    if (!seg.empty() && name_matches(name, nlen, seg)) {
                        path_matched_depth = node_depth;
                        remaining = advance_past_segment(remaining);
                    }
                    // else: off-path; we'll bail on the matching END_NODE.
                }
                continue;
            }
            if (tok == FDT_END_NODE) {
                if (node_depth == path_matched_depth) {
                    path_matched_depth--;
                }
                node_depth--;
                if (node_depth == 0) break;
                continue;
            }
            if (tok == FDT_PROP) {
                uint32_t plen    = load_be32(cur); cur += 4;
                uint32_t nameoff = load_be32(cur); cur += 4;
                const uint8_t* data = cur;
                cur += align4(plen);

                // Property belongs to the currently-open node (at
                // node_depth). It matches our target iff this node is the
                // path tip AND the name matches.
                if (node_depth != tip_depth) continue;
                if (node_depth != path_matched_depth) continue;

                if (nameoff >= hdr_.size_dt_strings) return false;
                const char* pn = strings_base + nameoff;
                size_t pnlen = cstr_len(pn, strings_end);
                if (!name_matches(pn, pnlen, prop_name)) continue;

                if (plen < 4) return false;
                *out = load_be32(data);
                return true;
            }
            // Unknown token — abort rather than walk off the rails.
            return false;
        }
        return false;
    }

private:
    static size_t align4(size_t n) { return (n + 3u) & ~size_t{3u}; }

    static size_t cstr_len(const char* s, const uint8_t* end) {
        const char* e = reinterpret_cast<const char*>(end);
        size_t n = 0;
        while (s + n < e && s[n] != '\0') ++n;
        return n;
    }
    static size_t cstr_len(const char* s, const char* end) {
        size_t n = 0;
        while (s + n < end && s[n] != '\0') ++n;
        return n;
    }

    // Exact-match a DTB name (e.g. "cpus" or "cpu@0") against a requested
    // path segment.
    static bool name_matches(const char* s, size_t slen, std::string_view want) {
        if (slen != want.size()) return false;
        for (size_t i = 0; i < slen; ++i) {
            if (s[i] != want[i]) return false;
        }
        return true;
    }

    static constexpr size_t npos = static_cast<size_t>(-1);

    // Manual find/rfind of '/' — std::string_view::find pulls in memchr via
    // char_traits, which we don't have in freestanding.
    static size_t find_slash(std::string_view p) {
        for (size_t i = 0; i < p.size(); ++i) if (p[i] == '/') return i;
        return npos;
    }
    static size_t rfind_slash(std::string_view p) {
        for (size_t i = p.size(); i > 0; --i) if (p[i - 1] == '/') return i - 1;
        return npos;
    }

    static std::string_view pop_segment(std::string_view p) {
        if (p.empty()) return {};
        size_t slash = find_slash(p);
        if (slash == npos) return p;
        return p.substr(0, slash);
    }

    static std::string_view advance_past_segment(std::string_view p) {
        size_t slash = find_slash(p);
        if (slash == npos) return {};
        return p.substr(slash + 1);
    }

    const uint8_t* base_ = nullptr;
    Header hdr_{};
};

// Convenience: look up /cpus/timebase-frequency, falling back to the per-cpu
// node if the property isn't present on the parent. Returns 0 on failure.
inline uint32_t read_timebase_frequency(const void* dtb) {
    Blob b(dtb);
    if (!b.valid()) return 0;
    uint32_t freq = 0;
    if (b.get_u32("/cpus/timebase-frequency", &freq) && freq) return freq;
    if (b.get_u32("/cpus/cpu@0/timebase-frequency", &freq) && freq) return freq;
    return 0;
}

} // namespace hal::qemu_virt_rv64::fdt

#endif // MINIOS_HAL_RV64_FDT_HPP
