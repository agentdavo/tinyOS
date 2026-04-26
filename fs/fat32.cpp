// SPDX-License-Identifier: MIT OR Apache-2.0

#include "fat32.hpp"

#include <cstring>

namespace fs::fat32 {

namespace {

// 32-byte directory entry.
struct DirEntry {
    uint8_t  name[11];       // 8.3 name (space-padded). First byte: 0x00 end,
                             // 0xE5 deleted, 0x2E '.' or '..'. 0x05 aliases 0xE5.
    uint8_t  attr;           // 0x01 RO, 0x02 Hidden, 0x04 System, 0x08 Vol ID,
                             // 0x10 Dir, 0x20 Archive, 0x0F = LFN entry.
    uint8_t  ntres;          // reserved
    uint8_t  crt_time_tenth;
    uint16_t crt_time;
    uint16_t crt_date;
    uint16_t lst_acc_date;
    uint16_t fst_clus_hi;
    uint16_t wrt_time;
    uint16_t wrt_date;
    uint16_t fst_clus_lo;
    uint32_t file_size;
} __attribute__((packed));

// 32-byte LFN entry (attr == 0x0F). Chains precede the short entry in
// reverse sequence order; ord bit 0x40 flags the first-written (last seen
// by the scanner).
struct LfnEntry {
    uint8_t  ord;
    uint16_t name1[5];       // UCS-2 LE
    uint8_t  attr;           // 0x0F
    uint8_t  type;           // 0
    uint8_t  checksum;
    uint16_t name2[6];       // UCS-2 LE
    uint16_t fst_clus_lo;    // 0 for LFN
    uint16_t name3[2];       // UCS-2 LE
} __attribute__((packed));

static_assert(sizeof(DirEntry) == 32, "fat dir entry must be 32 bytes");
static_assert(sizeof(LfnEntry) == 32, "fat lfn entry must be 32 bytes");

// LFN char slots per entry: 5 + 6 + 2 = 13 UCS-2 codepoints.
constexpr size_t LFN_CHARS_PER = 13;
// Max LFN entries chained (spec max = 20 -> 260 char filename, but we cap
// at MAX_PATH_LEN bytes anyway).
constexpr size_t LFN_MAX_ENTRIES = 16;

constexpr uint8_t ATTR_READ_ONLY = 0x01;
constexpr uint8_t ATTR_HIDDEN    = 0x02;
constexpr uint8_t ATTR_SYSTEM    = 0x04;
constexpr uint8_t ATTR_VOLUME_ID = 0x08;
constexpr uint8_t ATTR_DIRECTORY = 0x10;
constexpr uint8_t ATTR_ARCHIVE   = 0x20;
constexpr uint8_t ATTR_LFN       = 0x0F;

inline uint16_t read_le16(const void* p) {
    const auto* b = static_cast<const uint8_t*>(p);
    return static_cast<uint16_t>(b[0] | (b[1] << 8));
}
inline uint32_t read_le32(const void* p) {
    const auto* b = static_cast<const uint8_t*>(p);
    return static_cast<uint32_t>(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
}

uint64_t cluster_to_lba(const Volume& v, uint32_t cluster) {
    if (cluster < 2) return 0;
    return v.data_start_lba + static_cast<uint64_t>(cluster - 2) * v.sectors_per_cluster;
}

// Read the FAT entry for `cluster`, return the next cluster index (or one
// of the EOF sentinels — anything >= 0x0FFFFFF8).
uint32_t fat_next_cluster(Volume& v, uint32_t cluster, uint8_t* fat_sector_buf,
                          uint64_t& cached_lba) {
    const uint64_t entry_byte = static_cast<uint64_t>(cluster) * 4;
    const uint64_t lba = v.fat_start_lba + entry_byte / v.bytes_per_sector;
    const uint32_t off = static_cast<uint32_t>(entry_byte % v.bytes_per_sector);
    if (cached_lba != lba) {
        if (!v.read(lba, 1, fat_sector_buf, v.user)) return 0x0FFFFFFF;
        cached_lba = lba;
    }
    return read_le32(fat_sector_buf + off) & 0x0FFFFFFF;
}

// Build a lowercase-ASCII 8.3 fallback from the dir entry's `name` field.
// Space-padded to 11 bytes on disk; we emit "name.ext".
void short_name_to_string(const uint8_t name[11], char* out, size_t out_size) {
    if (out_size == 0) return;
    size_t w = 0;
    // Restore 0x05 -> 0xE5 alias. 0xE5 in real data means deleted (filtered
    // by caller already).
    auto restore = [](uint8_t c) -> uint8_t {
        if (c == 0x05) return 0xE5;
        return c;
    };
    for (int i = 0; i < 8; ++i) {
        const uint8_t c = restore(name[i]);
        if (c == 0x20) break;
        if (w + 1 < out_size) out[w++] = static_cast<char>(c >= 'A' && c <= 'Z' ? c + 32 : c);
    }
    bool has_ext = false;
    for (int i = 8; i < 11; ++i) if (name[i] != 0x20) { has_ext = true; break; }
    if (has_ext && w + 1 < out_size) out[w++] = '.';
    for (int i = 8; i < 11; ++i) {
        const uint8_t c = restore(name[i]);
        if (c == 0x20) break;
        if (w + 1 < out_size) out[w++] = static_cast<char>(c >= 'A' && c <= 'Z' ? c + 32 : c);
    }
    out[w] = '\0';
}

// Copy LFN UCS-2 chars into an ASCII buffer (truncating non-ASCII).
// `ucs` points at an array of `count` little-endian UCS-2 codepoints.
// Returns true if a terminator (0x0000 or 0xFFFF) was reached.
bool copy_lfn_segment(const uint16_t* ucs, size_t count, char* dst, size_t& dst_pos,
                      size_t dst_cap) {
    for (size_t i = 0; i < count; ++i) {
        const uint16_t c = ucs[i];
        if (c == 0x0000 || c == 0xFFFF) return true;
        if (dst_pos + 1 < dst_cap) {
            dst[dst_pos++] = (c < 0x80) ? static_cast<char>(c) : '?';
        }
    }
    return false;
}

// Walk a directory cluster chain, invoking `entry_cb` for each regular file
// or subdirectory. `entry_cb(name, attr, cluster, size, user)` returns
// false to stop the walk.
using DirCb = bool (*)(const char* name, uint8_t attr, uint32_t cluster,
                       uint32_t size, void* user);

bool walk_directory(Volume& v, uint32_t dir_cluster, DirCb cb, void* user) {
    // Per-directory scratch must be stack-local — walk_directory recurses
    // into subdirectories via the callback, and static buffers would be
    // clobbered by the inner call and the outer iteration would then walk
    // the wrong directory's data when it resumed. 512 bytes per recursion
    // level × typical depth of 2-3 is comfortable within the thread stack.
    alignas(8) uint8_t dir_sector[SECTOR_SIZE];
    alignas(8) uint8_t fat_sector[SECTOR_SIZE];
    uint64_t fat_cached_lba = 0xFFFFFFFFFFFFFFFFULL;

    uint32_t cluster = dir_cluster;

    // Accumulate LFN entries across dir entries. lfn_buf holds up to
    // LFN_MAX_ENTRIES * LFN_CHARS_PER ASCII chars.
    char lfn_accum[LFN_MAX_ENTRIES * LFN_CHARS_PER + 1] = {};
    size_t lfn_pos = 0;

    while (cluster >= 2 && cluster < 0x0FFFFFF8) {
        const uint64_t first_lba = cluster_to_lba(v, cluster);
        for (uint32_t s = 0; s < v.sectors_per_cluster; ++s) {
            if (!v.read(first_lba + s, 1, dir_sector, v.user)) return false;
            for (uint32_t off = 0; off + sizeof(DirEntry) <= v.bytes_per_sector;
                 off += sizeof(DirEntry)) {
                const uint8_t first = dir_sector[off];
                if (first == 0x00) {
                    // End of directory.
                    return true;
                }
                if (first == 0xE5) {
                    lfn_pos = 0;
                    lfn_accum[0] = '\0';
                    continue;
                }
                const uint8_t attr = dir_sector[off + 11];
                if ((attr & ATTR_LFN) == ATTR_LFN) {
                    // LFN entry. Sequence bits: low 5 bits = order. The
                    // first-written entry has the 0x40 "last" flag set,
                    // which since LFN chains are stored in reverse order
                    // actually means "highest-order in the filename".
                    const auto* lfn = reinterpret_cast<const LfnEntry*>(&dir_sector[off]);
                    const uint8_t ord = lfn->ord;
                    const uint8_t seq = static_cast<uint8_t>(ord & 0x3F);
                    const bool is_last = (ord & 0x40) != 0;
                    if (seq == 0 || seq > LFN_MAX_ENTRIES) { lfn_pos = 0; continue; }

                    // Place this entry's 13 chars at position (seq-1) * 13.
                    size_t pos = (seq - 1) * LFN_CHARS_PER;
                    if (pos + LFN_CHARS_PER >= sizeof(lfn_accum)) continue;
                    size_t local_pos = pos;
                    bool terminator_seen = false;
                    // The LfnEntry struct has name1[5], name2[6], name3[2]
                    // but some compilers may pad — read raw bytes instead.
                    const uint8_t* raw = &dir_sector[off];
                    uint16_t ucs[LFN_CHARS_PER];
                    for (size_t i = 0; i < 5; ++i)
                        ucs[i] = read_le16(raw + 1 + i * 2);
                    for (size_t i = 0; i < 6; ++i)
                        ucs[5 + i] = read_le16(raw + 14 + i * 2);
                    for (size_t i = 0; i < 2; ++i)
                        ucs[11 + i] = read_le16(raw + 28 + i * 2);
                    for (size_t i = 0; i < LFN_CHARS_PER; ++i) {
                        if (local_pos + 1 >= sizeof(lfn_accum)) break;
                        const uint16_t c = ucs[i];
                        if (c == 0x0000 || c == 0xFFFF) { terminator_seen = true; break; }
                        lfn_accum[local_pos++] = (c < 0x80) ? static_cast<char>(c) : '?';
                    }
                    if (is_last || local_pos > lfn_pos) lfn_pos = local_pos;
                    if (terminator_seen) lfn_accum[local_pos] = '\0';
                    else lfn_accum[lfn_pos] = '\0';
                    continue;
                }

                // Regular 8.3 entry — consume any accumulated LFN.
                if ((attr & (ATTR_VOLUME_ID | ATTR_HIDDEN | ATTR_SYSTEM)) != 0) {
                    lfn_pos = 0; lfn_accum[0] = '\0'; continue;
                }
                const auto* de = reinterpret_cast<const DirEntry*>(&dir_sector[off]);
                if (de->name[0] == 0x2E) {
                    // "." or ".." — skip.
                    lfn_pos = 0; lfn_accum[0] = '\0'; continue;
                }

                char name_buf[MAX_PATH_LEN] = {};
                if (lfn_pos > 0 && lfn_accum[0] != '\0') {
                    size_t n = 0;
                    while (lfn_accum[n] && n + 1 < sizeof(name_buf)) {
                        name_buf[n] = lfn_accum[n];
                        ++n;
                    }
                    name_buf[n] = '\0';
                } else {
                    short_name_to_string(de->name, name_buf, sizeof(name_buf));
                }
                lfn_pos = 0; lfn_accum[0] = '\0';

                const uint32_t first_cluster =
                    (static_cast<uint32_t>(read_le16(&de->fst_clus_hi)) << 16) |
                    read_le16(&de->fst_clus_lo);
                const uint32_t size = read_le32(&de->file_size);
                if (!cb(name_buf, attr, first_cluster, size, user)) return true;
            }
        }
        cluster = fat_next_cluster(v, cluster, fat_sector, fat_cached_lba);
    }
    return true;
}

// Per-component path match context.
struct FindCtx {
    const char* target;
    size_t target_len;
    bool target_is_final;
    bool found;
    uint8_t attr;
    uint32_t cluster;
    uint32_t size;
};

bool find_cb(const char* name, uint8_t attr, uint32_t cluster, uint32_t size,
             void* user) {
    auto* ctx = static_cast<FindCtx*>(user);
    size_t n = 0;
    while (name[n]) ++n;
    if (n != ctx->target_len) return true;
    for (size_t i = 0; i < n; ++i) {
        char a = name[i];
        char b = ctx->target[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a + 32);
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b + 32);
        if (a != b) return true;
    }
    ctx->found = true;
    ctx->attr = attr;
    ctx->cluster = cluster;
    ctx->size = size;
    return false;  // stop walking
}

// Split `path` at the next '/' and return the component length + whether
// this is the last component.
bool next_component(const char*& path, size_t& comp_len, bool& is_last) {
    while (*path == '/') ++path;  // tolerate leading/duplicate slashes
    if (*path == '\0') return false;
    const char* start = path;
    while (*path && *path != '/') ++path;
    comp_len = static_cast<size_t>(path - start);
    is_last = (*path == '\0' || *(path + 1) == '\0' && *path == '/');
    path = start;
    return comp_len > 0;
}

// Resolve `path` to (cluster, size, attr). Returns false if missing.
bool resolve_path(Volume& v, const char* path, uint32_t& out_cluster,
                  uint32_t& out_size, uint8_t& out_attr) {
    uint32_t cur_cluster = v.root_cluster;
    out_attr = ATTR_DIRECTORY;

    while (*path == '/') ++path;
    while (*path) {
        const char* comp_start = path;
        while (*path && *path != '/') ++path;
        const size_t comp_len = static_cast<size_t>(path - comp_start);
        if (comp_len == 0) break;

        FindCtx ctx{};
        ctx.target = comp_start;
        ctx.target_len = comp_len;
        ctx.found = false;
        if (!walk_directory(v, cur_cluster, &find_cb, &ctx)) return false;
        if (!ctx.found) return false;

        cur_cluster = ctx.cluster;
        out_size = ctx.size;
        out_attr = ctx.attr;

        while (*path == '/') ++path;
        if (*path == '\0') {
            out_cluster = cur_cluster;
            return true;
        }
        if ((out_attr & ATTR_DIRECTORY) == 0) return false;
    }
    // `path` is empty -> root.
    out_cluster = v.root_cluster;
    out_size = 0;
    out_attr = ATTR_DIRECTORY;
    return true;
}

// Recursive walk helper used by walk().
struct WalkState {
    Volume* vol;
    WalkCb cb;
    void* cb_user;
    char path_buf[MAX_PATH_LEN];
    size_t path_len;  // current length, always points at the null terminator
    const char* prefix;
    size_t prefix_len;
    bool prefix_satisfied;  // true once we've descended into the prefix
};

bool recurse_walk(Volume& v, uint32_t dir_cluster, WalkState& ws);

bool walk_entry_cb(const char* name, uint8_t attr, uint32_t cluster, uint32_t size,
                   void* user) {
    auto* ws = static_cast<WalkState*>(user);
    const size_t saved_len = ws->path_len;

    // Append "/<name>" to the path.
    if (ws->path_len > 0 && ws->path_len + 1 < MAX_PATH_LEN) {
        ws->path_buf[ws->path_len++] = '/';
    }
    size_t n = 0;
    while (name[n] && ws->path_len + 1 < MAX_PATH_LEN) {
        ws->path_buf[ws->path_len++] = name[n++];
    }
    ws->path_buf[ws->path_len] = '\0';

    bool cont = true;
    if ((attr & ATTR_DIRECTORY) != 0) {
        // Directory: always recurse. The prefix filter is applied at the
        // file-emit step, so "system/machine" as a prefix still needs us
        // to descend "system" first.
        cont = recurse_walk(*ws->vol, cluster, *ws);
    } else {
        // File: emit if the current path starts with the requested prefix.
        if (ws->prefix_len == 0 ||
            (ws->path_len >= ws->prefix_len &&
             std::memcmp(ws->path_buf, ws->prefix, ws->prefix_len) == 0)) {
            cont = ws->cb(ws->path_buf, size, ws->cb_user);
        }
    }

    ws->path_len = saved_len;
    ws->path_buf[saved_len] = '\0';
    return cont;
}

bool recurse_walk(Volume& v, uint32_t dir_cluster, WalkState& ws) {
    return walk_directory(v, dir_cluster, &walk_entry_cb, &ws);
}

// --- write-side helpers ---------------------------------------------------
//
// All restricted to the root directory and 8.3 names. Each helper
// independently validates volume state so any caller path that wanders
// out of the supported envelope returns false instead of corrupting.

inline void write_le16(void* p, uint16_t v) {
    auto* b = static_cast<uint8_t*>(p);
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline void write_le32(void* p, uint32_t v) {
    auto* b = static_cast<uint8_t*>(p);
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    b[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    b[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

// Encode a path-component into the 11-byte 8.3 packed form. Returns false
// if the name doesn't fit the FAT short-name shape (uppercased ASCII,
// 1-8 base chars, 0-3 ext chars, no embedded dots beyond the separator,
// no spaces). setup.cfg passes; long names don't.
bool encode_short_name(const char* name, size_t name_len, uint8_t out[11]) {
    if (name_len == 0 || name_len > 12) return false;  // 8 + 1 + 3
    for (int i = 0; i < 11; ++i) out[i] = 0x20;

    size_t dot = name_len;
    for (size_t i = 0; i < name_len; ++i) {
        if (name[i] == '.') { dot = i; break; }
    }
    const size_t base_len = dot;
    const size_t ext_len = (dot < name_len) ? (name_len - dot - 1) : 0;
    if (base_len == 0 || base_len > 8) return false;
    if (ext_len > 3) return false;

    auto enc = [](char c) -> int {
        if (c >= 'a' && c <= 'z') return c - 'a' + 'A';
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) return c;
        // Permit a small set of safe punctuation that's legal in 8.3.
        if (c == '_' || c == '-' || c == '~' || c == '!' || c == '#' || c == '$') return c;
        return -1;
    };

    for (size_t i = 0; i < base_len; ++i) {
        const int e = enc(name[i]);
        if (e < 0) return false;
        out[i] = static_cast<uint8_t>(e);
    }
    for (size_t i = 0; i < ext_len; ++i) {
        const int e = enc(name[dot + 1 + i]);
        if (e < 0) return false;
        out[8 + i] = static_cast<uint8_t>(e);
    }
    // 0xE5 in slot 0 means "deleted"; remap to 0x05 like the spec wants.
    if (out[0] == 0xE5) out[0] = 0x05;
    return true;
}

bool short_names_equal(const uint8_t a[11], const uint8_t b[11]) {
    for (int i = 0; i < 11; ++i) if (a[i] != b[i]) return false;
    return true;
}

// Write FAT entry [cluster] = value, replicating across all FAT copies so
// host fsck doesn't whinge. Only updates the bottom 28 bits as the spec
// reserves the top 4 bits of every entry.
bool fat_set_entry(Volume& v, uint32_t cluster, uint32_t value) {
    if (!v.write) return false;
    const uint64_t entry_byte = static_cast<uint64_t>(cluster) * 4;
    const uint64_t off_in_sec = entry_byte % v.bytes_per_sector;
    alignas(8) uint8_t sec[SECTOR_SIZE];
    for (uint32_t i = 0; i < v.num_fats; ++i) {
        const uint64_t lba = v.fat_start_lba +
                             static_cast<uint64_t>(i) * v.fat_size_sectors +
                             entry_byte / v.bytes_per_sector;
        if (!v.read(lba, 1, sec, v.user)) return false;
        const uint32_t cur = read_le32(sec + off_in_sec) & 0xF0000000;  // preserve reserved bits
        write_le32(sec + off_in_sec, cur | (value & 0x0FFFFFFF));
        if (!v.write(lba, 1, sec, v.user)) return false;
    }
    return true;
}

uint32_t fat_get_entry(Volume& v, uint32_t cluster) {
    alignas(8) uint8_t sec[SECTOR_SIZE];
    const uint64_t entry_byte = static_cast<uint64_t>(cluster) * 4;
    const uint64_t lba = v.fat_start_lba + entry_byte / v.bytes_per_sector;
    if (!v.read(lba, 1, sec, v.user)) return 0x0FFFFFFF;
    return read_le32(sec + (entry_byte % v.bytes_per_sector)) & 0x0FFFFFFF;
}

// Walk the existing chain starting at `start_cluster` and free every entry
// (set to 0). Stops at EOC. Used to truncate before re-writing or to
// reclaim deleted-file storage.
bool fat_free_chain(Volume& v, uint32_t start_cluster) {
    uint32_t cur = start_cluster;
    // Bound the walk so a corrupt loop doesn't hang the kernel.
    for (uint32_t guard = 0; guard < (1u << 20); ++guard) {
        if (cur < 2 || cur >= 0x0FFFFFF8) return true;
        const uint32_t nxt = fat_get_entry(v, cur);
        if (!fat_set_entry(v, cur, 0)) return false;
        cur = nxt;
    }
    return false;
}

// Linear scan of the FAT for the next free cluster (entry == 0). Starts at
// cluster 2 (first usable) and stops at total cluster count derived from
// the FAT size. Returns 0 if the volume is full.
uint32_t fat_alloc_one(Volume& v) {
    if (!v.write) return 0;
    alignas(8) uint8_t sec[SECTOR_SIZE];
    const uint32_t entries_per_sec = v.bytes_per_sector / 4;
    const uint32_t total_entries = v.fat_size_sectors * entries_per_sec;
    for (uint32_t cluster = 2; cluster < total_entries; ) {
        const uint64_t entry_byte = static_cast<uint64_t>(cluster) * 4;
        const uint64_t lba = v.fat_start_lba + entry_byte / v.bytes_per_sector;
        if (!v.read(lba, 1, sec, v.user)) return 0;
        const uint32_t off0 = static_cast<uint32_t>(entry_byte % v.bytes_per_sector);
        for (uint32_t off = off0; off + 4 <= v.bytes_per_sector; off += 4, ++cluster) {
            const uint32_t e = read_le32(sec + off) & 0x0FFFFFFF;
            if (e == 0) return cluster;
        }
    }
    return 0;
}

// Locate a directory entry slot in the root for `target_name` (8.3 packed).
// On hit: returns true and fills *out_lba / *out_off / fills the 32-byte
// entry into out_entry. On miss: returns true with *found == false and the
// first reusable (0xE5) or end-of-dir (0x00) slot filled in for caller use.
struct RootSlot {
    bool found = false;       // existing entry with matching short name
    uint64_t lba = 0;         // sector containing the slot
    uint32_t off = 0;         // byte offset into that sector
    uint8_t entry[32]{};      // a copy of the existing slot (only valid if found)

    bool free_found = false;  // a 0x00/0xE5 slot we can reuse
    uint64_t free_lba = 0;
    uint32_t free_off = 0;
    bool free_is_end = false; // true if the free slot was 0x00 (end marker)
};

bool root_scan_for_slot(Volume& v, const uint8_t target_name[11], RootSlot& slot) {
    slot = RootSlot{};
    alignas(8) uint8_t sec[SECTOR_SIZE];
    alignas(8) uint8_t fat_sec[SECTOR_SIZE];
    uint64_t fat_cached = 0xFFFFFFFFFFFFFFFFULL;
    uint32_t cluster = v.root_cluster;
    while (cluster >= 2 && cluster < 0x0FFFFFF8) {
        const uint64_t base = cluster_to_lba(v, cluster);
        for (uint32_t s = 0; s < v.sectors_per_cluster; ++s) {
            const uint64_t lba = base + s;
            if (!v.read(lba, 1, sec, v.user)) return false;
            for (uint32_t off = 0; off + 32 <= v.bytes_per_sector; off += 32) {
                const uint8_t first = sec[off];
                if (first == 0x00) {
                    // End of directory marker — claim it as the free slot.
                    if (!slot.free_found) {
                        slot.free_found = true;
                        slot.free_lba = lba;
                        slot.free_off = off;
                        slot.free_is_end = true;
                    }
                    return true;  // can't see beyond end-of-dir
                }
                if (first == 0xE5) {
                    if (!slot.free_found) {
                        slot.free_found = true;
                        slot.free_lba = lba;
                        slot.free_off = off;
                        slot.free_is_end = false;
                    }
                    continue;
                }
                const uint8_t attr = sec[off + 11];
                if ((attr & ATTR_LFN) == ATTR_LFN) continue;  // skip LFN runs
                if ((attr & ATTR_VOLUME_ID) != 0) continue;
                // Compare short name.
                if (short_names_equal(&sec[off], target_name)) {
                    slot.found = true;
                    slot.lba = lba;
                    slot.off = off;
                    for (int i = 0; i < 32; ++i) slot.entry[i] = sec[off + i];
                    return true;
                }
            }
        }
        cluster = fat_next_cluster(v, cluster, fat_sec, fat_cached);
    }
    return true;  // not found, no free slot either (caller checks)
}

// Write `bytes` of `data` into the chain starting at `first_cluster`.
// Allocates additional clusters if needed. Returns the chain head (==
// first_cluster on success). Returns 0 on out-of-space / IO error.
uint32_t write_chain(Volume& v, uint32_t first_cluster, const uint8_t* data, uint32_t bytes) {
    if (!v.write) return 0;
    const uint32_t cluster_bytes = v.bytes_per_sector * v.sectors_per_cluster;
    alignas(8) uint8_t sec[SECTOR_SIZE];

    uint32_t cur = first_cluster;
    uint32_t prev = 0;
    uint32_t written = 0;
    while (written < bytes || prev == 0) {
        if (cur < 2 || cur >= 0x0FFFFFF8) {
            // Need to extend.
            const uint32_t nxt = fat_alloc_one(v);
            if (nxt == 0) return 0;
            // Mark the new cluster as EOC; chain prev->nxt.
            if (!fat_set_entry(v, nxt, 0x0FFFFFFF)) return 0;
            if (prev != 0 && !fat_set_entry(v, prev, nxt)) return 0;
            cur = nxt;
        }
        const uint64_t lba = cluster_to_lba(v, cur);
        const uint32_t to_write_in_cluster =
            (bytes - written) < cluster_bytes ? (bytes - written) : cluster_bytes;
        for (uint32_t s = 0; s < v.sectors_per_cluster; ++s) {
            const uint32_t off = s * v.bytes_per_sector;
            if (off >= to_write_in_cluster) {
                // Tail sector beyond the data — zero-fill so old bytes don't leak.
                for (uint32_t i = 0; i < v.bytes_per_sector; ++i) sec[i] = 0;
            } else {
                const uint32_t copy =
                    (to_write_in_cluster - off) < v.bytes_per_sector ?
                        (to_write_in_cluster - off) : v.bytes_per_sector;
                for (uint32_t i = 0; i < copy; ++i) sec[i] = data[written + off + i];
                for (uint32_t i = copy; i < v.bytes_per_sector; ++i) sec[i] = 0;
            }
            if (!v.write(lba + s, 1, sec, v.user)) return 0;
        }
        written += to_write_in_cluster;
        prev = cur;
        // Move on to next cluster from FAT (might be EOC for the very last one).
        if (written < bytes) {
            cur = fat_get_entry(v, cur);
        } else {
            // Truncate any tail of the existing chain.
            const uint32_t tail = fat_get_entry(v, cur);
            if (!fat_set_entry(v, cur, 0x0FFFFFFF)) return 0;
            if (tail >= 2 && tail < 0x0FFFFFF8) {
                if (!fat_free_chain(v, tail)) return 0;
            }
            break;
        }
    }
    return first_cluster;
}

}  // namespace

bool mount(Volume& vol, SectorReader read, void* user) {
    return mount(vol, read, nullptr, user);
}

bool mount(Volume& vol, SectorReader read, SectorWriter write, void* user) {
    if (!read) return false;
    vol = Volume{};
    vol.read = read;
    vol.write = write;
    vol.user = user;

    alignas(8) uint8_t boot[SECTOR_SIZE];
    if (!read(0, 1, boot, user)) return false;
    // 0x55AA signature at 510.
    if (boot[510] != 0x55 || boot[511] != 0xAA) return false;

    vol.bytes_per_sector = read_le16(&boot[11]);
    vol.sectors_per_cluster = boot[13];
    vol.reserved_sectors = read_le16(&boot[14]);
    vol.num_fats = boot[16];
    vol.fat_size_sectors = read_le32(&boot[36]);
    vol.root_cluster = read_le32(&boot[44]);

    if (vol.bytes_per_sector != SECTOR_SIZE) return false;
    if (vol.sectors_per_cluster == 0) return false;
    if (vol.num_fats == 0) return false;
    if (vol.fat_size_sectors == 0) return false;
    if (vol.root_cluster < 2) return false;

    vol.fat_start_lba = vol.reserved_sectors;
    vol.data_start_lba = static_cast<uint64_t>(vol.reserved_sectors) +
                         static_cast<uint64_t>(vol.num_fats) * vol.fat_size_sectors;
    vol.mounted = true;
    return true;
}

bool walk(Volume& vol, const char* prefix, WalkCb cb, void* cb_user) {
    if (!vol.mounted || !cb) return false;
    WalkState ws{};
    ws.vol = &vol;
    ws.cb = cb;
    ws.cb_user = cb_user;
    ws.path_buf[0] = '\0';
    ws.path_len = 0;
    ws.prefix = prefix ? prefix : "";
    // Skip any leading '/' in the prefix — our path_buf never includes one.
    while (ws.prefix[0] == '/') ++ws.prefix;
    ws.prefix_len = 0;
    while (ws.prefix[ws.prefix_len]) ++ws.prefix_len;
    return recurse_walk(vol, vol.root_cluster, ws);
}

bool read_file(Volume& vol, const char* path, void* buf, size_t buf_size,
               size_t* bytes_read) {
    if (!vol.mounted || !path || !buf || !bytes_read) return false;
    *bytes_read = 0;

    // Skip any leading '/'.
    while (*path == '/') ++path;

    uint32_t cluster = 0, size = 0;
    uint8_t attr = 0;
    if (!resolve_path(vol, path, cluster, size, attr)) return false;
    if ((attr & ATTR_DIRECTORY) != 0) return false;
    if (size == 0) { *bytes_read = 0; return true; }

    const uint32_t cluster_bytes = vol.bytes_per_sector * vol.sectors_per_cluster;
    alignas(8) static uint8_t fat_sector[SECTOR_SIZE];
    alignas(8) static uint8_t cluster_buf[SECTOR_SIZE * 32];  // up to 16 KB cluster
    if (cluster_bytes > sizeof(cluster_buf)) return false;

    uint64_t fat_cached_lba = 0xFFFFFFFFFFFFFFFFULL;
    size_t written = 0;
    uint32_t remaining = size;
    uint32_t cur = cluster;

    while (remaining > 0 && cur >= 2 && cur < 0x0FFFFFF8) {
        const uint64_t lba = cluster_to_lba(vol, cur);
        if (!vol.read(lba, vol.sectors_per_cluster, cluster_buf, vol.user)) return false;
        const uint32_t take = remaining < cluster_bytes ? remaining : cluster_bytes;
        const size_t space = (written < buf_size) ? (buf_size - written) : 0;
        const size_t copy = take < space ? take : space;
        if (copy > 0) {
            std::memcpy(static_cast<uint8_t*>(buf) + written, cluster_buf, copy);
            written += copy;
        }
        remaining -= take;
        cur = fat_next_cluster(vol, cur, fat_sector, fat_cached_lba);
    }
    *bytes_read = written;
    return true;
}

bool write_file(Volume& vol, const char* path, const void* buf, size_t bytes) {
    if (!vol.mounted || !vol.write || !path || (!buf && bytes != 0)) return false;
    if (bytes > kMaxWriteFileBytes) return false;

    // Strip leading slashes; reject any embedded slash (root-only by design).
    while (*path == '/') ++path;
    size_t name_len = 0;
    while (path[name_len]) {
        if (path[name_len] == '/') return false;
        ++name_len;
    }
    if (name_len == 0) return false;

    uint8_t target[11];
    if (!encode_short_name(path, name_len, target)) return false;

    RootSlot slot{};
    if (!root_scan_for_slot(vol, target, slot)) return false;

    uint32_t first_cluster = 0;
    if (slot.found) {
        // Reuse existing chain head if present; otherwise allocate one.
        first_cluster =
            (static_cast<uint32_t>(read_le16(&slot.entry[0x14])) << 16) |
            read_le16(&slot.entry[0x1A]);
    }
    if (first_cluster < 2 || first_cluster >= 0x0FFFFFF8) {
        first_cluster = (bytes == 0) ? 0 : fat_alloc_one(vol);
        if (bytes != 0 && first_cluster == 0) return false;
        if (first_cluster != 0) {
            if (!fat_set_entry(vol, first_cluster, 0x0FFFFFFF)) return false;
        }
    }

    if (bytes > 0 && first_cluster != 0) {
        if (write_chain(vol, first_cluster,
                        static_cast<const uint8_t*>(buf),
                        static_cast<uint32_t>(bytes)) == 0) {
            return false;
        }
    } else if (bytes == 0 && slot.found) {
        // Truncate existing chain to nothing.
        const uint32_t old =
            (static_cast<uint32_t>(read_le16(&slot.entry[0x14])) << 16) |
            read_le16(&slot.entry[0x1A]);
        if (old >= 2 && old < 0x0FFFFFF8) {
            if (!fat_free_chain(vol, old)) return false;
        }
        first_cluster = 0;
    }

    // Build the directory entry.
    alignas(8) uint8_t entry[32] = {};
    for (int i = 0; i < 11; ++i) entry[i] = target[i];
    entry[11] = ATTR_ARCHIVE;
    write_le16(&entry[0x14], static_cast<uint16_t>((first_cluster >> 16) & 0xFFFF));
    write_le16(&entry[0x1A], static_cast<uint16_t>(first_cluster & 0xFFFF));
    write_le32(&entry[0x1C], static_cast<uint32_t>(bytes));

    // Choose where to land the entry: existing slot (in place), or the first
    // reusable / end-of-dir slot the scan found.
    uint64_t target_lba = 0;
    uint32_t target_off = 0;
    bool was_end = false;
    if (slot.found) {
        target_lba = slot.lba;
        target_off = slot.off;
    } else if (slot.free_found) {
        target_lba = slot.free_lba;
        target_off = slot.free_off;
        was_end = slot.free_is_end;
    } else {
        return false;  // root directory full — would need to grow it
    }

    alignas(8) uint8_t sec[SECTOR_SIZE];
    if (!vol.read(target_lba, 1, sec, vol.user)) return false;
    for (int i = 0; i < 32; ++i) sec[target_off + i] = entry[i];
    // If we just consumed the end-of-dir marker, write a fresh terminator
    // immediately after if room allows. If not (slot was last 32B of the
    // sector / cluster), the next cluster's existing 0x00 slot keeps us
    // honest — root_scan walks until it sees one anyway.
    if (was_end && target_off + 64 <= vol.bytes_per_sector) {
        sec[target_off + 32] = 0x00;
    }
    if (!vol.write(target_lba, 1, sec, vol.user)) return false;
    return true;
}

bool rename_file(Volume& vol, const char* from, const char* to) {
    if (!vol.mounted || !vol.write || !from || !to) return false;
    while (*from == '/') ++from;
    while (*to == '/') ++to;
    size_t from_len = 0, to_len = 0;
    while (from[from_len]) {
        if (from[from_len] == '/') return false;
        ++from_len;
    }
    while (to[to_len]) {
        if (to[to_len] == '/') return false;
        ++to_len;
    }
    if (from_len == 0 || to_len == 0) return false;

    uint8_t from_n[11], to_n[11];
    if (!encode_short_name(from, from_len, from_n)) return false;
    if (!encode_short_name(to, to_len, to_n)) return false;
    if (short_names_equal(from_n, to_n)) return true;  // no-op rename

    // Find source.
    RootSlot from_slot{};
    if (!root_scan_for_slot(vol, from_n, from_slot)) return false;
    if (!from_slot.found) return false;

    // If `to` exists, free its chain and clear its dir entry first. This is
    // the small atomicity weakness called out in CLAUDE.md — accepted for
    // first-cut so write-then-rename works at all.
    RootSlot to_slot{};
    if (!root_scan_for_slot(vol, to_n, to_slot)) return false;
    if (to_slot.found) {
        const uint32_t old =
            (static_cast<uint32_t>(read_le16(&to_slot.entry[0x14])) << 16) |
            read_le16(&to_slot.entry[0x1A]);
        if (old >= 2 && old < 0x0FFFFFF8) {
            if (!fat_free_chain(vol, old)) return false;
        }
        alignas(8) uint8_t sec[SECTOR_SIZE];
        if (!vol.read(to_slot.lba, 1, sec, vol.user)) return false;
        sec[to_slot.off + 0] = 0xE5;  // delete marker
        if (!vol.write(to_slot.lba, 1, sec, vol.user)) return false;
    }

    // Mutate the source entry's name in place. This is atomic at sector
    // granularity (one 512 B write); on power loss the entry either has
    // the old or the new name, never a torn 11-byte field.
    alignas(8) uint8_t sec[SECTOR_SIZE];
    if (!vol.read(from_slot.lba, 1, sec, vol.user)) return false;
    for (int i = 0; i < 11; ++i) sec[from_slot.off + i] = to_n[i];
    if (!vol.write(from_slot.lba, 1, sec, vol.user)) return false;
    return true;
}

}  // namespace fs::fat32
