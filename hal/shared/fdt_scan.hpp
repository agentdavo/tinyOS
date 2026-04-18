// SPDX-License-Identifier: MIT OR Apache-2.0

#ifndef HAL_SHARED_FDT_SCAN_HPP
#define HAL_SHARED_FDT_SCAN_HPP

#include <cstdint>
#include <cstddef>
#include <string_view>

namespace hal::shared::fdt {

inline uint32_t load_be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
            static_cast<uint32_t>(p[3]);
}

inline uint64_t load_be64(const uint8_t* p) {
    return (static_cast<uint64_t>(load_be32(p)) << 32) |
            static_cast<uint64_t>(load_be32(p + 4));
}

struct Header {
    uint32_t magic = 0;
    uint32_t totalsize = 0;
    uint32_t off_dt_struct = 0;
    uint32_t off_dt_strings = 0;
    uint32_t off_mem_rsvmap = 0;
    uint32_t version = 0;
    uint32_t last_comp_version = 0;
    uint32_t boot_cpuid_phys = 0;
    uint32_t size_dt_strings = 0;
    uint32_t size_dt_struct = 0;
};

constexpr uint32_t MAGIC          = 0xD00DFEEDu;
constexpr uint32_t FDT_BEGIN_NODE = 1u;
constexpr uint32_t FDT_END_NODE   = 2u;
constexpr uint32_t FDT_PROP       = 3u;
constexpr uint32_t FDT_NOP        = 4u;
constexpr uint32_t FDT_END        = 9u;

inline size_t align4(size_t value) { return (value + 3u) & ~size_t{3u}; }

inline size_t cstr_len(const char* s, const uint8_t* end) {
    const char* limit = reinterpret_cast<const char*>(end);
    size_t n = 0;
    while (s + n < limit && s[n] != '\0') ++n;
    return n;
}

inline bool str_eq(const char* s, size_t len, std::string_view want) {
    if (len != want.size()) return false;
    for (size_t i = 0; i < len; ++i) {
        if (s[i] != want[i]) return false;
    }
    return true;
}

inline bool compat_list_contains(const uint8_t* data, size_t len, std::string_view compat) {
    size_t off = 0;
    while (off < len) {
        const char* s = reinterpret_cast<const char*>(data + off);
        size_t slen = 0;
        while (off + slen < len && s[slen] != '\0') ++slen;
        if (str_eq(s, slen, compat)) return true;
        off += slen + 1;
    }
    return false;
}

class Blob {
public:
    explicit Blob(const void* dtb) : base_(reinterpret_cast<const uint8_t*>(dtb)) {
        if (!base_) return;
        hdr_.magic             = load_be32(base_ + 0);
        hdr_.totalsize         = load_be32(base_ + 4);
        hdr_.off_dt_struct     = load_be32(base_ + 8);
        hdr_.off_dt_strings    = load_be32(base_ + 12);
        hdr_.off_mem_rsvmap    = load_be32(base_ + 16);
        hdr_.version           = load_be32(base_ + 20);
        hdr_.last_comp_version = load_be32(base_ + 24);
        hdr_.boot_cpuid_phys   = load_be32(base_ + 28);
        hdr_.size_dt_strings   = load_be32(base_ + 32);
        hdr_.size_dt_struct    = load_be32(base_ + 36);
    }

    bool valid() const { return base_ && hdr_.magic == MAGIC; }

    bool find_compatible_reg(std::string_view compat,
                             uint64_t* out_base,
                             uint64_t* out_size,
                             uint64_t* out_mmio_base = nullptr,
                             uint64_t* out_mmio_size = nullptr,
                             uint32_t* out_bus_start = nullptr,
                             uint32_t* out_bus_end = nullptr) const {
        if (!valid() || !out_base || !out_size) return false;

        const uint8_t* struct_base = base_ + hdr_.off_dt_struct;
        const uint8_t* struct_end = struct_base + hdr_.size_dt_struct;
        const char* strings = reinterpret_cast<const char*>(base_ + hdr_.off_dt_strings);

        struct NodeState {
            bool compat_match = false;
            bool has_reg = false;
            bool has_bus_range = false;
            bool has_mmio_range = false;
            uint64_t reg_base = 0;
            uint64_t reg_size = 0;
            uint64_t mmio_base = 0;
            uint64_t mmio_size = 0;
            uint32_t bus_start = 0;
            uint32_t bus_end = 0xff;
        };

        NodeState stack[16]{};
        int depth = -1;
        const uint8_t* cur = struct_base;
        while (cur < struct_end) {
            const uint32_t tok = load_be32(cur);
            cur += 4;
            if (tok == FDT_NOP) continue;
            if (tok == FDT_END) break;
            if (tok == FDT_BEGIN_NODE) {
                const char* name = reinterpret_cast<const char*>(cur);
                const size_t nlen = cstr_len(name, struct_end);
                cur += align4(nlen + 1);
                if (depth + 1 >= static_cast<int>(sizeof(stack) / sizeof(stack[0]))) return false;
                ++depth;
                stack[depth] = {};
                continue;
            }
            if (tok == FDT_END_NODE) {
                if (depth >= 0 && stack[depth].compat_match && stack[depth].has_reg) {
                    *out_base = stack[depth].reg_base;
                    *out_size = stack[depth].reg_size;
                    if (out_mmio_base) *out_mmio_base = stack[depth].has_mmio_range ? stack[depth].mmio_base : 0;
                    if (out_mmio_size) *out_mmio_size = stack[depth].has_mmio_range ? stack[depth].mmio_size : 0;
                    if (out_bus_start) *out_bus_start = stack[depth].has_bus_range ? stack[depth].bus_start : 0;
                    if (out_bus_end) *out_bus_end = stack[depth].has_bus_range ? stack[depth].bus_end : 0xff;
                    return true;
                }
                --depth;
                continue;
            }
            if (tok != FDT_PROP || depth < 0) return false;

            const uint32_t plen = load_be32(cur);
            cur += 4;
            const uint32_t nameoff = load_be32(cur);
            cur += 4;
            const uint8_t* data = cur;
            cur += align4(plen);
            const char* prop = strings + nameoff;

            if (str_eq(prop, cstr_len(prop, reinterpret_cast<const uint8_t*>(strings + hdr_.size_dt_strings)), "compatible")) {
                stack[depth].compat_match = compat_list_contains(data, plen, compat);
                continue;
            }
            if (str_eq(prop, cstr_len(prop, reinterpret_cast<const uint8_t*>(strings + hdr_.size_dt_strings)), "reg") && plen >= 16) {
                stack[depth].reg_base = load_be64(data);
                stack[depth].reg_size = load_be64(data + 8);
                stack[depth].has_reg = true;
                continue;
            }
            if (str_eq(prop, cstr_len(prop, reinterpret_cast<const uint8_t*>(strings + hdr_.size_dt_strings)), "bus-range") && plen >= 8) {
                stack[depth].bus_start = load_be32(data);
                stack[depth].bus_end = load_be32(data + 4);
                stack[depth].has_bus_range = true;
                continue;
            }
            if (str_eq(prop, cstr_len(prop, reinterpret_cast<const uint8_t*>(strings + hdr_.size_dt_strings)), "ranges") && plen >= 28) {
                for (size_t off = 0; off + 28 <= plen; off += 28) {
                    const uint32_t space = load_be32(data + off) & 0x03000000u;
                    if (space != 0x02000000u && space != 0x03000000u) continue;
                    stack[depth].mmio_base = load_be64(data + off + 12);
                    stack[depth].mmio_size = load_be64(data + off + 20);
                    stack[depth].has_mmio_range = true;
                    break;
                }
                continue;
            }
        }
        return false;
    }

private:
    const uint8_t* base_ = nullptr;
    Header hdr_{};
};

inline bool find_compatible_reg(const void* dtb,
                                std::string_view compat,
                                uint64_t* out_base,
                                uint64_t* out_size,
                                uint64_t* out_mmio_base = nullptr,
                                uint64_t* out_mmio_size = nullptr,
                                uint32_t* out_bus_start = nullptr,
                                uint32_t* out_bus_end = nullptr) {
    Blob blob(dtb);
    return blob.find_compatible_reg(compat, out_base, out_size,
                                    out_mmio_base, out_mmio_size,
                                    out_bus_start, out_bus_end);
}

} // namespace hal::shared::fdt

#endif // HAL_SHARED_FDT_SCAN_HPP
