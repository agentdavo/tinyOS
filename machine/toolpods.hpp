// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "core.hpp"

#include <cstddef>
#include <cstdint>

namespace machine::toolpods {

enum class PodMotion : uint8_t {
    Linear = 0,
    Rotary,
};

struct Station {
    bool used = false;
    uint16_t index = 0;
    int32_t position_counts = 0;
    float machine_x = 0.0f;
    float machine_y = 0.0f;
    float machine_z = 0.0f;
    float machine_a = 0.0f;
    uint16_t physical_tool = 0;
    uint16_t virtual_tool = 0;
    char name[24]{};
};

struct Pod {
    bool used = false;
    char id[24]{};
    char title[32]{};
    char axis_name[8]{};
    int8_t motion_axis = -1;
    PodMotion motion = PodMotion::Linear;
    // Machine-space anchor of the pod body. Station offsets are expressed
    // relative to this anchor in the render path (toolpod_station_transform
    // composes pod.machine_* with station.machine_*).
    float machine_x = 0.0f;
    float machine_y = 0.0f;
    float machine_z = 0.0f;
    size_t station_count = 0;
    size_t active_station = 0;
    bool locked = true;
    bool clamp_required = false;
    Station stations[16]{};
};

class Service {
public:
    static constexpr size_t MAX_PODS = 8;

    bool load_tsv(const char* buf, size_t len) noexcept;
    size_t pod_count() const noexcept;
    const Pod* pod(size_t idx) const noexcept;
    const Pod* pod_by_id(const char* id) const noexcept;
    bool find_pod(const char* id, size_t& idx_out) const noexcept;
    bool select_station(const char* pod_id, size_t station_idx) noexcept;
    bool select_station(size_t pod_idx, size_t station_idx) noexcept;
    bool assign_virtual_tool(const char* pod_id, size_t station_idx, uint16_t virtual_tool) noexcept;
    bool assign_virtual_tool(size_t pod_idx, size_t station_idx, uint16_t virtual_tool) noexcept;
    bool set_locked(const char* pod_id, bool locked) noexcept;
    bool set_locked(size_t pod_idx, bool locked) noexcept;
    bool lookup_virtual_tool(uint16_t virtual_tool, size_t& pod_idx, size_t& station_idx) const noexcept;
    const Station* active_station(size_t pod_idx) const noexcept;
    const Station* active_station(const char* pod_id) const noexcept;

private:
    void reset() noexcept;
    int resolve_motion_axis(const char* axis_name) const noexcept;

    Pod pods_[MAX_PODS]{};
    size_t count_ = 0;
    mutable kernel::core::Spinlock lock_;
};

extern Service g_service;

} // namespace machine::toolpods
