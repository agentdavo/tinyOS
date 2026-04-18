// SPDX-License-Identifier: MIT OR Apache-2.0

#include "offsets.hpp"

namespace cnc::offsets {

Service g_service;

Service::Service() {
    static constexpr const char* kNames[WORK_OFFSET_COUNT] = {"G54", "G55", "G56", "G57", "G58", "G59"};
    for (size_t i = 0; i < WORK_OFFSET_COUNT; ++i) {
        work_offsets_[i].name = kNames[i];
        work_offsets_[i].value.axis[0] = static_cast<float>(i) * 12.5f;
        work_offsets_[i].value.axis[1] = static_cast<float>(i) * 3.0f;
        work_offsets_[i].value.axis[2] = 0.0f;
        work_offsets_[i].value.axis[3] = 0.0f;
    }
    for (size_t i = 0; i < TOOL_OFFSET_COUNT; ++i) {
        tool_offsets_[i].tool = static_cast<uint32_t>(i + 1);
        tool_offsets_[i].length = 25.0f + static_cast<float>(i) * 2.5f;
        tool_offsets_[i].radius = (i % 4 == 0) ? 6.0f : 3.0f;
        tool_offsets_[i].wear = 0.0f;
    }
}

bool Service::select_work(size_t idx) noexcept {
    if (idx >= WORK_OFFSET_COUNT) return false;
    kernel::core::ScopedLock lock(lock_);
    active_work_ = idx;
    return true;
}

bool Service::select_tool(size_t idx) noexcept {
    if (idx >= TOOL_OFFSET_COUNT) return false;
    kernel::core::ScopedLock lock(lock_);
    active_tool_ = idx;
    return true;
}

bool Service::set_work_axis(size_t idx, size_t axis, float value) noexcept {
    if (idx >= WORK_OFFSET_COUNT || axis >= AXIS_COUNT) return false;
    kernel::core::ScopedLock lock(lock_);
    work_offsets_[idx].value.axis[axis] = value;
    return true;
}

bool Service::set_tool_value(size_t idx, float length, float radius, float wear) noexcept {
    if (idx >= TOOL_OFFSET_COUNT) return false;
    kernel::core::ScopedLock lock(lock_);
    tool_offsets_[idx].length = length;
    tool_offsets_[idx].radius = radius;
    tool_offsets_[idx].wear = wear;
    return true;
}

} // namespace cnc::offsets
