// SPDX-License-Identifier: MIT OR Apache-2.0

#include "pallet.hpp"

#include "util.hpp"

namespace machine::pallet {

Service g_service;

namespace {

// Mirrors the toolpods loader's helpers — keep them local rather than
// pulling in another header. TSV format: lines start with a record kind
// token, then tab-separated key=value fields.
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
    long v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); ++s; }
    return neg ? -v : v;
}

void copy_string(char* dst, size_t cap, const char* src) {
    if (!dst || cap == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    kernel::util::k_snprintf(dst, cap, "%s", src);
}

}  // namespace

const char* Service::status_name(PalletStatus s) noexcept {
    switch (s) {
        case PalletStatus::Empty:   return "empty";
        case PalletStatus::Loaded:  return "loaded";
        case PalletStatus::Cutting: return "cutting";
        case PalletStatus::Done:    return "done";
        case PalletStatus::Fault:   return "fault";
        case PalletStatus::Hold:    return "hold";
    }
    return "?";
}

bool Service::parse_status(const char* token, PalletStatus& out) noexcept {
    if (!token) return false;
    if (kernel::util::kstrcmp(token, "empty")   == 0) { out = PalletStatus::Empty;   return true; }
    if (kernel::util::kstrcmp(token, "loaded")  == 0) { out = PalletStatus::Loaded;  return true; }
    if (kernel::util::kstrcmp(token, "cutting") == 0) { out = PalletStatus::Cutting; return true; }
    if (kernel::util::kstrcmp(token, "done")    == 0) { out = PalletStatus::Done;    return true; }
    if (kernel::util::kstrcmp(token, "fault")   == 0) { out = PalletStatus::Fault;   return true; }
    if (kernel::util::kstrcmp(token, "hold")    == 0) { out = PalletStatus::Hold;    return true; }
    return false;
}

void Service::reset() noexcept {
    for (auto& p : pallets_) p = Pallet{};
    count_  = 0;
    active_ = SIZE_MAX;
}

bool Service::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    kernel::core::ScopedLock lock(lock_);
    reset();
    const char* p = buf;
    const char* end = buf + len;
    char line[256];
    while (p < end) {
        p = next_line(p, end, line, sizeof(line));
        if (line[0] == '\0' || line[0] == '#') continue;
        char* rest = line;
        while (*rest && *rest != '\t') ++rest;
        if (*rest == '\t') *rest++ = '\0';
        if (kernel::util::kstrcmp(line, "pallet") == 0) {
            if (count_ >= MAX_PALLETS) continue;
            auto& q = pallets_[count_++];
            q = Pallet{};
            q.used = true;
            char a[64], b[64], c[64], d[64], e[64], f[64];
            copy_string(q.id,         sizeof(q.id),         field_value(rest, "id",      a, sizeof(a)));
            copy_string(q.fixture_id, sizeof(q.fixture_id), field_value(rest, "fixture", b, sizeof(b)));
            const char* prog = field_value(rest, "program", c, sizeof(c));
            if (prog) copy_string(q.assigned_program, sizeof(q.assigned_program), prog);
            q.station_index     = static_cast<uint8_t>(
                parse_long(field_value(rest, "station", d, sizeof(d)), static_cast<long>(count_ - 1)));
            q.work_offset_index = static_cast<int>(
                parse_long(field_value(rest, "wcs", e, sizeof(e)), -1));
            q.target_cycles     = static_cast<uint32_t>(
                parse_long(field_value(rest, "target", f, sizeof(f)), 0));
            // Status is optional; Empty unless a program is wired up too,
            // in which case Loaded is the more useful default.
            const char* status_tok = field_value(rest, "status", a, sizeof(a));
            PalletStatus s = q.assigned_program[0] != '\0' ? PalletStatus::Loaded
                                                           : PalletStatus::Empty;
            if (status_tok) (void)parse_status(status_tok, s);
            q.status = s;
        }
    }
    return count_ != 0;
}

size_t Service::pallet_count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return count_;
}

const Pallet* Service::pallet(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return nullptr;
    return &pallets_[idx];
}

const Pallet* Service::pallet_by_id(const char* id) const noexcept {
    if (!id) return nullptr;
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (pallets_[i].used && kernel::util::kstrcmp(pallets_[i].id, id) == 0) {
            return &pallets_[i];
        }
    }
    return nullptr;
}

bool Service::find_pallet(const char* id, size_t& idx_out) const noexcept {
    if (!id) return false;
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (pallets_[i].used && kernel::util::kstrcmp(pallets_[i].id, id) == 0) {
            idx_out = i;
            return true;
        }
    }
    return false;
}

bool Service::set_status(size_t idx, PalletStatus status) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    pallets_[idx].status = status;
    return true;
}

bool Service::set_status(const char* id, PalletStatus status) noexcept {
    size_t idx = 0;
    if (!find_pallet(id, idx)) return false;
    return set_status(idx, status);
}

bool Service::assign_program(size_t idx, const char* program, int work_offset_index) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    copy_string(pallets_[idx].assigned_program,
                sizeof(pallets_[idx].assigned_program),
                program ? program : "");
    pallets_[idx].work_offset_index = work_offset_index;
    return true;
}

bool Service::set_message(size_t idx, const char* msg) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    copy_string(pallets_[idx].last_message, sizeof(pallets_[idx].last_message),
                msg ? msg : "");
    return true;
}

bool Service::bump_cycles_completed(size_t idx) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    if (pallets_[idx].cycles_completed != UINT32_MAX) {
        ++pallets_[idx].cycles_completed;
    }
    return true;
}

bool Service::set_target_cycles(size_t idx, uint32_t target) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    pallets_[idx].target_cycles = target;
    return true;
}

size_t Service::active_pallet() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return active_;
}

bool Service::set_active_pallet(size_t idx) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= count_ || !pallets_[idx].used) return false;
    active_ = idx;
    return true;
}

bool Service::clear_active_pallet() noexcept {
    kernel::core::ScopedLock lock(lock_);
    active_ = SIZE_MAX;
    return true;
}

size_t Service::next_loaded_after(size_t start_idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (count_ == 0) return SIZE_MAX;
    const size_t begin = (start_idx >= count_) ? 0 : start_idx;
    for (size_t off = 0; off < count_; ++off) {
        const size_t i = (begin + off) % count_;
        const auto& q = pallets_[i];
        if (!q.used) continue;
        if (q.status != PalletStatus::Loaded) continue;
        // Pallet must have a program to be schedulable.
        if (q.assigned_program[0] == '\0') continue;
        return i;
    }
    return SIZE_MAX;
}

}  // namespace machine::pallet
