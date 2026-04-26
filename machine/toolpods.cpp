// SPDX-License-Identifier: MIT OR Apache-2.0

#include "toolpods.hpp"

#include "cnc/offsets.hpp"
#include "motion/motion.hpp"
#include "util.hpp"

namespace machine::toolpods {

namespace {

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
    if (*s == '-') {
        neg = true;
        ++s;
    }
    long value = 0;
    while (*s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        ++s;
    }
    return neg ? -value : value;
}

float parse_float(const char* s, float fallback = 0.0f) {
    if (!s || !*s) return fallback;
    bool neg = false;
    if (*s == '-') {
        neg = true;
        ++s;
    }
    float value = 0.0f;
    while (*s >= '0' && *s <= '9') {
        value = value * 10.0f + static_cast<float>(*s - '0');
        ++s;
    }
    if (*s == '.') {
        ++s;
        float place = 0.1f;
        while (*s >= '0' && *s <= '9') {
            value += static_cast<float>(*s - '0') * place;
            place *= 0.1f;
            ++s;
        }
    }
    return neg ? -value : value;
}

} // namespace

Service g_service;

void Service::reset() noexcept {
    count_ = 0;
    for (auto& pod : pods_) pod = Pod{};
}

size_t Service::pod_count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return count_;
}

const Pod* Service::pod(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return idx < count_ ? &pods_[idx] : nullptr;
}

bool Service::find_pod(const char* id, size_t& idx_out) const noexcept {
    if (!id || !*id) return false;
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        if (pods_[i].used && kernel::util::kstrcmp(pods_[i].id, id) == 0) {
            idx_out = i;
            return true;
        }
    }
    return false;
}

const Pod* Service::pod_by_id(const char* id) const noexcept {
    size_t idx = 0;
    return find_pod(id, idx) ? pod(idx) : nullptr;
}

int Service::resolve_motion_axis(const char* axis_name) const noexcept {
    if (!axis_name || !*axis_name) return -1;
    switch (axis_name[0]) {
        case 'X': case 'x': return 0;
        case 'Y': case 'y': return 1;
        case 'Z': case 'z': return 2;
        case 'A': case 'a': return 3;
        case 'B': case 'b': return 17;
        case 'C': case 'c': return 16;
        default: return -1;
    }
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
        char a[64], b[64], c[64], d[64], e[64], f[64], g[64], h[64];
        if (kernel::util::kstrcmp(line, "pod") == 0) {
            if (count_ >= MAX_PODS) continue;
            auto& pod = pods_[count_++];
            pod.used = true;
            kernel::util::k_snprintf(pod.id, sizeof(pod.id), "%s", field_value(rest, "id", a, sizeof(a)));
            const char* title = field_value(rest, "title", b, sizeof(b));
            kernel::util::k_snprintf(pod.title, sizeof(pod.title), "%s", title ? title : pod.id);
            kernel::util::k_snprintf(pod.axis_name, sizeof(pod.axis_name), "%s", field_value(rest, "axis", c, sizeof(c)));
            const char* motion = field_value(rest, "motion", d, sizeof(d));
            pod.motion = (motion && kernel::util::kstrcmp(motion, "rotary") == 0) ? PodMotion::Rotary : PodMotion::Linear;
            pod.motion_axis = static_cast<int8_t>(resolve_motion_axis(pod.axis_name));
            pod.clamp_required = parse_long(field_value(rest, "clamp_required", e, sizeof(e)), 0) != 0;
            pod.machine_x = parse_float(field_value(rest, "x", f, sizeof(f)), 0.0f);
            pod.machine_y = parse_float(field_value(rest, "y", g, sizeof(g)), 0.0f);
            pod.machine_z = parse_float(field_value(rest, "z", h, sizeof(h)), 0.0f);
            pod.locked = true;
            continue;
        }
        if (kernel::util::kstrcmp(line, "station") == 0 && count_ > 0) {
            const char* pod_id = field_value(rest, "pod", a, sizeof(a));
            size_t pod_idx = count_ - 1;
            for (size_t i = 0; i < count_; ++i) {
                if (pod_id && kernel::util::kstrcmp(pods_[i].id, pod_id) == 0) {
                    pod_idx = i;
                    break;
                }
            }
            auto& pod = pods_[pod_idx];
            if (pod.station_count >= 16) continue;
            auto& station = pod.stations[pod.station_count++];
            station.used = true;
            station.index = static_cast<uint16_t>(parse_long(field_value(rest, "index", b, sizeof(b)), 0));
            station.position_counts = static_cast<int32_t>(parse_long(field_value(rest, "position", c, sizeof(c)), 0));
            station.machine_x = parse_float(field_value(rest, "x", d, sizeof(d)), 0.0f);
            station.machine_y = parse_float(field_value(rest, "y", e, sizeof(e)), 0.0f);
            station.machine_z = parse_float(field_value(rest, "z", f, sizeof(f)), 0.0f);
            station.machine_a = parse_float(field_value(rest, "a", g, sizeof(g)), 0.0f);
            station.physical_tool = static_cast<uint16_t>(parse_long(field_value(rest, "physical_tool", h, sizeof(h)), station.index + 1));
            station.virtual_tool = static_cast<uint16_t>(parse_long(field_value(rest, "virtual_tool", a, sizeof(a)), station.physical_tool));
            const char* name = field_value(rest, "name", b, sizeof(b));
            kernel::util::k_snprintf(station.name, sizeof(station.name), "%s", name ? name : "station");
        }
    }
    return count_ != 0;
}

bool Service::select_station(size_t pod_idx, size_t station_idx) noexcept {
    if (pod_idx >= count_) return false;
    kernel::core::ScopedLock lock(lock_);
    auto& pod = pods_[pod_idx];
    if (station_idx >= pod.station_count) return false;
    pod.active_station = station_idx;
    const auto& station = pod.stations[station_idx];
    if (pod.motion_axis >= 0 && static_cast<size_t>(pod.motion_axis) < motion::MAX_AXES) {
        motion::g_motion.move_to(static_cast<size_t>(pod.motion_axis), station.position_counts);
    }
    if (station.virtual_tool > 0 && station.virtual_tool <= cnc::offsets::TOOL_OFFSET_COUNT) {
        (void)cnc::offsets::g_service.select_tool(static_cast<size_t>(station.virtual_tool - 1));
    }
    return true;
}

bool Service::select_station(const char* pod_id, size_t station_idx) noexcept {
    size_t idx = 0;
    return find_pod(pod_id, idx) && select_station(idx, station_idx);
}

bool Service::assign_virtual_tool(size_t pod_idx, size_t station_idx, uint16_t virtual_tool) noexcept {
    if (pod_idx >= count_) return false;
    kernel::core::ScopedLock lock(lock_);
    auto& pod = pods_[pod_idx];
    if (station_idx >= pod.station_count) return false;
    pod.stations[station_idx].virtual_tool = virtual_tool;
    return true;
}

bool Service::assign_virtual_tool(const char* pod_id, size_t station_idx, uint16_t virtual_tool) noexcept {
    size_t idx = 0;
    return find_pod(pod_id, idx) && assign_virtual_tool(idx, station_idx, virtual_tool);
}

bool Service::set_locked(size_t pod_idx, bool locked) noexcept {
    if (pod_idx >= count_) return false;
    kernel::core::ScopedLock lock_guard(lock_);
    pods_[pod_idx].locked = locked;
    return true;
}

bool Service::set_locked(const char* pod_id, bool locked) noexcept {
    size_t idx = 0;
    return find_pod(pod_id, idx) && set_locked(idx, locked);
}

bool Service::lookup_virtual_tool(uint16_t virtual_tool, size_t& pod_idx, size_t& station_idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < count_; ++i) {
        for (size_t j = 0; j < pods_[i].station_count; ++j) {
            if (pods_[i].stations[j].virtual_tool == virtual_tool) {
                pod_idx = i;
                station_idx = j;
                return true;
            }
        }
    }
    return false;
}

const Station* Service::active_station(size_t pod_idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (pod_idx >= count_) return nullptr;
    const auto& pod = pods_[pod_idx];
    return pod.active_station < pod.station_count ? &pod.stations[pod.active_station] : nullptr;
}

const Station* Service::active_station(const char* pod_id) const noexcept {
    size_t idx = 0;
    return find_pod(pod_id, idx) ? active_station(idx) : nullptr;
}

// ---- Tool change wizard (first-cut: operator-confirmation flow) -----------
// The state machine is intentionally simple: request_tool_change() resolves
// the target pod+station via lookup_virtual_tool() and sits in Moving until
// the operator confirms with accept_tool_change(). accept then performs the
// real station change via select_station(). abort returns to Idle without
// touching the active station. The total_steps counter exists so the UI can
// render progress; for first-cut both Releasing and Picking are skipped.

namespace {
void copy_msg(char* dst, size_t dst_size, const char* src) noexcept {
    if (!dst || dst_size == 0) return;
    size_t i = 0;
    if (src) {
        for (; src[i] && i + 1 < dst_size; ++i) dst[i] = src[i];
    }
    dst[i] = '\0';
}
}  // namespace

Service::ChangeStatus Service::tool_change_state() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    // Refresh current_tool against whatever the toolpod registry currently
    // reports as active so the UI always sees a live value, not a stale
    // capture from the last request_tool_change call.
    ChangeStatus snap = change_status_;
    for (size_t i = 0; i < count_; ++i) {
        if (const Station* st = (pods_[i].active_station < pods_[i].station_count)
                                 ? &pods_[i].stations[pods_[i].active_station]
                                 : nullptr;
            st && st->virtual_tool != 0) {
            snap.current_tool = st->virtual_tool;
            break;
        }
    }
    return snap;
}

bool Service::request_tool_change(uint32_t target_tool) noexcept {
    if (target_tool == 0 || target_tool > 0xFFFFu) return false;
    kernel::core::ScopedLock lock(lock_);
    change_status_.target_tool = static_cast<uint16_t>(target_tool);
    change_status_.target_resolved = false;
    change_status_.step = 0;
    change_status_.total_steps = 3;  // Moving → Verifying → Done
    // Resolve target → (pod, station) via the registry. Done inline rather
    // than calling lookup_virtual_tool because we already hold the lock.
    for (size_t i = 0; i < count_; ++i) {
        for (size_t j = 0; j < pods_[i].station_count; ++j) {
            if (pods_[i].stations[j].virtual_tool == target_tool) {
                change_status_.target_pod_idx = i;
                change_status_.target_station_idx = j;
                change_status_.target_resolved = true;
                break;
            }
        }
        if (change_status_.target_resolved) break;
    }
    if (!change_status_.target_resolved) {
        change_status_.state = ChangeState::Faulted;
        copy_msg(change_status_.message, sizeof(change_status_.message),
                 "target tool not assigned to any pod");
        return false;
    }
    change_status_.state = ChangeState::Moving;
    change_status_.step = 1;
    copy_msg(change_status_.message, sizeof(change_status_.message),
             "Manual tool change — verify pod, then ACCEPT");
    return true;
}

bool Service::abort_tool_change() noexcept {
    kernel::core::ScopedLock lock(lock_);
    change_status_.state = ChangeState::Idle;
    change_status_.step = 0;
    change_status_.target_tool = 0;
    change_status_.target_resolved = false;
    copy_msg(change_status_.message, sizeof(change_status_.message), "Aborted");
    return true;
}

bool Service::accept_tool_change() noexcept {
    // Resolve target inside the lock, then drop it before calling
    // select_station (which re-acquires) — same pattern probe_wizard_accept
    // uses on operator_api's lock.
    size_t pod_idx = 0, station_idx = 0;
    bool resolved = false;
    {
        kernel::core::ScopedLock lock(lock_);
        if (change_status_.state != ChangeState::Moving &&
            change_status_.state != ChangeState::Verifying) {
            return false;
        }
        if (!change_status_.target_resolved) return false;
        pod_idx = change_status_.target_pod_idx;
        station_idx = change_status_.target_station_idx;
        change_status_.state = ChangeState::Verifying;
        change_status_.step = 2;
        resolved = true;
    }
    bool ok = false;
    if (resolved) ok = select_station(pod_idx, station_idx);
    kernel::core::ScopedLock lock(lock_);
    if (ok) {
        change_status_.state = ChangeState::Done;
        change_status_.step = change_status_.total_steps;
        copy_msg(change_status_.message, sizeof(change_status_.message),
                 "Tool change confirmed");
    } else {
        change_status_.state = ChangeState::Faulted;
        copy_msg(change_status_.message, sizeof(change_status_.message),
                 "select_station failed");
    }
    return ok;
}

} // namespace machine::toolpods
