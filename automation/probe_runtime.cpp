// SPDX-License-Identifier: MIT OR Apache-2.0

#include "probe_runtime.hpp"

#include "machine/machine_registry.hpp"
#include "miniOS.hpp"
#include "motion/motion.hpp"
#include "util.hpp"

namespace probe {

namespace {

inline int32_t iabs32(int32_t v) {
    return v < 0 ? -v : v;
}

inline int64_t square64(int32_t v) {
    return static_cast<int64_t>(v) * static_cast<int64_t>(v);
}

int32_t isqrt64(int64_t value) {
    if (value <= 0) return 0;
    int64_t x = value;
    int64_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + value / x) / 2;
    }
    return static_cast<int32_t>(x);
}

uint64_t now_us() {
    auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return tm ? tm->get_system_time_us() : 0;
}

} // namespace

Runtime g_runtime;

void Runtime::reset_state() noexcept {
    active_ = false;
    done_ = false;
    fault_ = false;
    command_issued_ = false;
    captured_this_pass_ = false;
    stage_ = Stage::Idle;
    point_idx_ = 0;
    hit_idx_ = 0;
    configured_hits_ = 3;
    clearance_counts_ = 50000;
    overshoot_counts_ = 2000;
    center_ = Vec3{};
    for (size_t i = 0; i < kPointCount; ++i) {
        targets_[i] = Vec3{};
        approach_[i] = Vec3{};
        retract_[i] = Vec3{};
        infeed_[i] = Vec3{};
        sum_x_[i] = 0;
        sum_y_[i] = 0;
        sum_z_[i] = 0;
    }
    message_[0] = '\0';
}

void Runtime::set_fault(const char* msg) noexcept {
    fault_ = true;
    active_ = false;
    stage_ = Stage::Fault;
    kernel::util::k_snprintf(message_, sizeof(message_), "%s", msg ? msg : "probe fault");
    publish_status();
}

bool Runtime::load_config() noexcept {
    float cx_mm = 0.0f;
    float cy_mm = 0.0f;
    float cz_mm = 0.0f;
    float radius_mm = 15.0f;
    float sphere_diam_mm = motion::g_motion.sphere_diameter();
    float stylus_radius_mm = 0.0f;
    int32_t hits = 3;
    (void)machine::g_registry.get_float("probe_sphere_center_x_mm", cx_mm);
    (void)machine::g_registry.get_float("probe_sphere_center_y_mm", cy_mm);
    (void)machine::g_registry.get_float("probe_sphere_center_z_mm", cz_mm);
    (void)machine::g_registry.get_float("probe_sphere_radius_mm", radius_mm);
    (void)machine::g_registry.get_float("probe_sphere_diameter_mm", sphere_diam_mm);
    (void)machine::g_registry.get_float("probe_stylus_radius_mm", stylus_radius_mm);
    (void)machine::g_registry.get_int("probe_sphere_hits", hits);
    if (radius_mm <= 0.0f) radius_mm = 15.0f;
    if (hits <= 0) hits = 3;
    if (hits > static_cast<int32_t>(kMaxHits)) hits = static_cast<int32_t>(kMaxHits);
    if (sphere_diam_mm <= 0.0f) sphere_diam_mm = 20.0f;
    configured_hits_ = hits;
    motion::g_motion.sphere_set_diameter(sphere_diam_mm);
    motion::g_motion.sphere_set_probe_radius(static_cast<int32_t>(stylus_radius_mm * 1000.0f));
    motion::g_motion.sphere_set_probe_hits(hits);
    center_.x = static_cast<int32_t>(cx_mm * 10000.0f);
    center_.y = static_cast<int32_t>(cy_mm * 10000.0f);
    center_.z = static_cast<int32_t>(cz_mm * 10000.0f);

    const int32_t r = static_cast<int32_t>(radius_mm * 10000.0f);
    const int32_t h = static_cast<int32_t>(radius_mm * 707.0f / 1000.0f * 10000.0f);
    const int32_t d = static_cast<int32_t>(radius_mm * 500.0f / 1000.0f * 10000.0f);
    targets_[0] = {center_.x, center_.y, center_.z + r};
    targets_[1] = {center_.x, center_.y + h, center_.z + h};
    targets_[2] = {center_.x, center_.y - h, center_.z + h};
    targets_[3] = {center_.x + h, center_.y, center_.z + h};
    targets_[4] = {center_.x - h, center_.y, center_.z + h};
    targets_[5] = {center_.x + d, center_.y + d, center_.z + h};
    targets_[6] = {center_.x - d, center_.y - d, center_.z + h};

    for (size_t i = 0; i < kPointCount; ++i) {
        const int32_t nx = targets_[i].x - center_.x;
        const int32_t ny = targets_[i].y - center_.y;
        const int32_t nz = targets_[i].z - center_.z;
        const int32_t norm = isqrt64(square64(nx) + square64(ny) + square64(nz));
        if (norm <= 0) return false;
        const int32_t ux = static_cast<int32_t>((static_cast<int64_t>(nx) * clearance_counts_) / norm);
        const int32_t uy = static_cast<int32_t>((static_cast<int64_t>(ny) * clearance_counts_) / norm);
        const int32_t uz = static_cast<int32_t>((static_cast<int64_t>(nz) * clearance_counts_) / norm);
        const int32_t ix = static_cast<int32_t>((static_cast<int64_t>(nx) * overshoot_counts_) / norm);
        const int32_t iy = static_cast<int32_t>((static_cast<int64_t>(ny) * overshoot_counts_) / norm);
        const int32_t iz = static_cast<int32_t>((static_cast<int64_t>(nz) * overshoot_counts_) / norm);
        approach_[i] = {targets_[i].x + ux, targets_[i].y + uy, targets_[i].z + uz};
        retract_[i] = approach_[i];
        infeed_[i] = {targets_[i].x - ix, targets_[i].y - iy, targets_[i].z - iz};
    }
    return true;
}

void Runtime::publish_status() noexcept {
    (void)machine::g_registry.set_bool("probe_cal_active", active_);
    (void)machine::g_registry.set_bool("probe_cal_fault", fault_);
    (void)machine::g_registry.set_int("probe_cal_stage", static_cast<int32_t>(stage_));
    (void)machine::g_registry.set_int("probe_cal_point", static_cast<int32_t>(point_idx_));
    (void)machine::g_registry.set_int("probe_cal_hit", static_cast<int32_t>(hit_idx_));
}

bool Runtime::issue_sync_move(const Vec3& target) noexcept {
    int32_t targets[motion::MAX_AXES]{};
    targets[0] = target.x;
    targets[1] = target.y;
    targets[2] = target.z;
    return motion::g_motion.sync_move((1ull << 0) | (1ull << 1) | (1ull << 2),
                                      targets, 0, 20, 3, 20000, false);
}

bool Runtime::move_settled(const Vec3& target, int32_t tolerance) const noexcept {
    const int32_t ax = motion::g_motion.axis(0).actual_pos.load(std::memory_order_relaxed);
    const int32_t ay = motion::g_motion.axis(1).actual_pos.load(std::memory_order_relaxed);
    const int32_t az = motion::g_motion.axis(2).actual_pos.load(std::memory_order_relaxed);
    return iabs32(ax - target.x) <= tolerance &&
           iabs32(ay - target.y) <= tolerance &&
           iabs32(az - target.z) <= tolerance;
}

bool Runtime::start_reference_sphere() noexcept {
    if (active_) return false;
    reset_state();
    if (!load_config()) {
        set_fault("bad sphere config");
        return false;
    }
    motion::g_motion.sphere_clear_points();
    motion::g_motion.sphere_enable(false);
    (void)machine::g_registry.set_bool("probe_sphere_ready", false);
    (void)machine::g_registry.set_int("probe_sphere_point_count", 0);
    stage_ = Stage::Approach;
    active_ = true;
    kernel::util::k_snprintf(message_, sizeof(message_), "sphere acquisition started");
    publish_status();
    return true;
}

void Runtime::capture_hit() noexcept {
    sum_x_[point_idx_] += motion::g_motion.axis(0).actual_pos.load(std::memory_order_relaxed);
    sum_y_[point_idx_] += motion::g_motion.axis(1).actual_pos.load(std::memory_order_relaxed);
    sum_z_[point_idx_] += motion::g_motion.axis(2).actual_pos.load(std::memory_order_relaxed);
    captured_this_pass_ = true;
    (void)machine::g_registry.set_bool("probe_arm_request", false);
    (void)motion::g_motion.feedhold(0, true);
}

void Runtime::commit_points() noexcept {
    motion::g_motion.sphere_clear_points();
    for (size_t i = 0; i < kPointCount; ++i) {
        const int32_t mx = static_cast<int32_t>(sum_x_[i] / configured_hits_);
        const int32_t my = static_cast<int32_t>(sum_y_[i] / configured_hits_);
        const int32_t mz = static_cast<int32_t>(sum_z_[i] / configured_hits_);
        (void)motion::g_motion.sphere_add_point(targets_[i].x, targets_[i].y, targets_[i].z, mx, my, mz);
    }
    (void)machine::g_registry.set_int("probe_sphere_point_count", static_cast<int32_t>(kPointCount));
}

void Runtime::advance_stage() noexcept {
    command_issued_ = false;
    switch (stage_) {
        case Stage::Approach:
            stage_ = Stage::Infeed;
            captured_this_pass_ = false;
            break;
        case Stage::Infeed:
            stage_ = Stage::Retract;
            break;
        case Stage::Retract:
            ++hit_idx_;
            if (hit_idx_ >= static_cast<size_t>(configured_hits_)) {
                hit_idx_ = 0;
                ++point_idx_;
                if (point_idx_ >= kPointCount) {
                    stage_ = Stage::Compute;
                } else {
                    stage_ = Stage::Approach;
                }
            } else {
                stage_ = Stage::Approach;
            }
            break;
        case Stage::Compute:
            if (!motion::g_motion.sphere_compute_errors()) {
                set_fault("sphere solve failed");
                return;
            }
            motion::g_motion.sphere_enable(true);
            (void)machine::g_registry.set_bool("probe_sphere_ready", true);
            active_ = false;
            done_ = true;
            stage_ = Stage::Complete;
            kernel::util::k_snprintf(message_, sizeof(message_), "sphere model enabled");
            break;
        case Stage::Complete:
        case Stage::Fault:
        case Stage::Idle:
            break;
    }
    publish_status();
}

void Runtime::tick() noexcept {
    if (!active_) {
        publish_status();
        return;
    }
    publish_status();
    switch (stage_) {
        case Stage::Approach:
            if (!command_issued_) {
                (void)motion::g_motion.feedhold(0, false);
                if (!issue_sync_move(approach_[point_idx_])) {
                    set_fault("approach move failed");
                    return;
                }
                kernel::util::k_snprintf(message_, sizeof(message_), "approach p%lu h%lu",
                                         static_cast<unsigned long>(point_idx_),
                                         static_cast<unsigned long>(hit_idx_));
                command_issued_ = true;
            } else if (move_settled(approach_[point_idx_])) {
                advance_stage();
            }
            break;
        case Stage::Infeed:
            if (!command_issued_) {
                (void)machine::g_registry.set_bool("probe_tripped", false);
                (void)machine::g_registry.set_bool("probe_arm_request", true);
                (void)machine::g_registry.set_int("probe_target_mode", 1);
                (void)machine::g_registry.set_int("probe_target_x", targets_[point_idx_].x);
                (void)machine::g_registry.set_int("probe_target_y", targets_[point_idx_].y);
                (void)machine::g_registry.set_int("probe_target_z", targets_[point_idx_].z);
                if (!issue_sync_move(infeed_[point_idx_])) {
                    set_fault("probe infeed failed");
                    return;
                }
                kernel::util::k_snprintf(message_, sizeof(message_), "probe p%lu h%lu",
                                         static_cast<unsigned long>(point_idx_),
                                         static_cast<unsigned long>(hit_idx_));
                command_issued_ = true;
                captured_this_pass_ = false;
            } else {
                bool tripped = false;
                (void)machine::g_registry.get_bool("probe_tripped", tripped);
                if (tripped && !captured_this_pass_) {
                    capture_hit();
                }
                if (captured_this_pass_) {
                    advance_stage();
                } else if (move_settled(infeed_[point_idx_], 200)) {
                    set_fault("probe missed sphere");
                    return;
                }
            }
            break;
        case Stage::Retract:
            if (!command_issued_) {
                (void)motion::g_motion.feedhold(0, false);
                if (!issue_sync_move(retract_[point_idx_])) {
                    set_fault("retract move failed");
                    return;
                }
                kernel::util::k_snprintf(message_, sizeof(message_), "retract p%lu h%lu",
                                         static_cast<unsigned long>(point_idx_),
                                         static_cast<unsigned long>(hit_idx_));
                command_issued_ = true;
            } else if (move_settled(retract_[point_idx_])) {
                advance_stage();
            }
            break;
        case Stage::Compute:
            commit_points();
            advance_stage();
            break;
        case Stage::Complete:
        case Stage::Fault:
        case Stage::Idle:
            break;
    }
}

void Runtime::thread_entry(void*) {
    for (;;) {
        g_runtime.tick();
        auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
        if (tm) tm->wait_until_ns((now_us() + 1000ULL) * 1000ULL);
    }
}

} // namespace probe
