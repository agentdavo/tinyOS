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

}  // namespace

bool mount(Volume& vol, SectorReader read, void* user) {
    if (!read) return false;
    vol = Volume{};
    vol.read = read;
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

}  // namespace fs::fat32
