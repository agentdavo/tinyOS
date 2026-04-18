// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>

namespace probe {

class Runtime {
public:
    enum class Stage : uint8_t {
        Idle = 0,
        Approach,
        Infeed,
        Retract,
        Compute,
        Complete,
        Fault,
    };

    bool start_reference_sphere() noexcept;
    void tick() noexcept;
    bool active() const noexcept { return active_; }
    bool done() const noexcept { return done_; }
    bool fault() const noexcept { return fault_; }
    const char* message() const noexcept { return message_; }
    void clear_done() noexcept { done_ = false; }

    static void thread_entry(void* arg);

private:
    struct Vec3 {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
    };

    static constexpr size_t kPointCount = 7;
    static constexpr size_t kMaxHits = 10;

    void reset_state() noexcept;
    void set_fault(const char* msg) noexcept;
    bool load_config() noexcept;
    void publish_status() noexcept;
    bool issue_sync_move(const Vec3& target) noexcept;
    bool move_settled(const Vec3& target, int32_t tolerance = 50) const noexcept;
    void advance_stage() noexcept;
    void capture_hit() noexcept;
    void commit_points() noexcept;

    bool active_ = false;
    bool done_ = false;
    bool fault_ = false;
    bool command_issued_ = false;
    bool captured_this_pass_ = false;
    Stage stage_ = Stage::Idle;
    size_t point_idx_ = 0;
    size_t hit_idx_ = 0;
    int32_t configured_hits_ = 3;
    int32_t clearance_counts_ = 50000;
    int32_t overshoot_counts_ = 2000;
    Vec3 center_{};
    Vec3 targets_[kPointCount]{};
    Vec3 approach_[kPointCount]{};
    Vec3 retract_[kPointCount]{};
    Vec3 infeed_[kPointCount]{};
    int64_t sum_x_[kPointCount]{};
    int64_t sum_y_[kPointCount]{};
    int64_t sum_z_[kPointCount]{};
    char message_[96]{};
};

extern Runtime g_runtime;

} // namespace probe
