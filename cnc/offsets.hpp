// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "../core.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace cnc::offsets {

constexpr size_t WORK_OFFSET_COUNT = 6;
constexpr size_t TOOL_OFFSET_COUNT = 16;
constexpr size_t AXIS_COUNT = 4;

struct OffsetVector {
    float axis[AXIS_COUNT]{0.0f, 0.0f, 0.0f, 0.0f};
};

struct WorkOffset {
    const char* name = "G54";
    OffsetVector value{};
};

struct ToolOffset {
    uint32_t tool = 0;
    // Tool-length vector in user units. `length` (= length_z) remains the
    // common case — a straight tool sticking out in +Z. length_x / length_y
    // exist so 5-axis head/tail kinematics or angled spindles can express
    // the tool tip's offset from the spindle reference frame as a full
    // vector, applied per-axis by the interpreter under G43.
    float length = 0.0f;        // legacy alias for length_z
    float length_x = 0.0f;
    float length_y = 0.0f;
    float radius = 0.0f;
    float wear = 0.0f;
};

class Service {
public:
    Service();

    const std::array<WorkOffset, WORK_OFFSET_COUNT>& work_offsets() const noexcept { return work_offsets_; }
    const std::array<ToolOffset, TOOL_OFFSET_COUNT>& tool_offsets() const noexcept { return tool_offsets_; }
    size_t active_work() const noexcept { return active_work_; }
    size_t active_tool() const noexcept { return active_tool_; }

    bool select_work(size_t idx) noexcept;
    bool select_tool(size_t idx) noexcept;
    bool set_work_axis(size_t idx, size_t axis, float value) noexcept;
    bool set_tool_value(size_t idx, float length, float radius, float wear) noexcept;

private:
    std::array<WorkOffset, WORK_OFFSET_COUNT> work_offsets_{};
    std::array<ToolOffset, TOOL_OFFSET_COUNT> tool_offsets_{};
    size_t active_work_ = 0;
    size_t active_tool_ = 0;
    mutable kernel::core::Spinlock lock_;
};

extern Service g_service;

} // namespace cnc::offsets
