// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "../motion/motion.hpp"
#include "programs.hpp"
#include <cstddef>
#include <cstdint>

namespace cnc::interp {

enum class MotionMode : uint8_t {
    Rapid = 0,
    Linear,
    ArcCW,
    ArcCCW,
};

enum class Plane : uint8_t {
    XY = 0,
    XZ,
    YZ,
};

enum class State : uint8_t {
    Idle = 0,
    Ready,
    Running,
    WaitingBarrier,
    WaitingMacro,
    Dwell,
    Complete,
    Fault,
};

struct ChannelSnapshot {
    State state = State::Idle;
    size_t channel = 0;
    size_t program_index = 0;
    size_t line = 0;
    size_t block = 0;
    uint16_t barrier_token = 0;
    uint8_t barrier_mask = 0;
    uint32_t feed = 0;
    int32_t spindle = 0;
    bool absolute = true;
    MotionMode motion_mode = MotionMode::Rapid;
    Plane plane = Plane::XY;
    bool inch_mode = false;
    size_t active_work = 0;
    size_t active_tool = 0;
    bool tool_length_active = false;
    bool coolant_mist = false;
    bool coolant_flood = false;
    const char* program_name = "";
};

class Runtime {
public:
    struct ChannelState {
        struct ArcState {
            bool active = false;
            uint8_t primary_axis = 0;
            uint8_t secondary_axis = 0;
            uint8_t linear_axis = 0;
            bool has_linear_axis = false;
            uint64_t axis_mask = 0;
            int32_t start_primary = 0;
            int32_t start_secondary = 0;
            int32_t start_linear = 0;
            int32_t end_primary = 0;
            int32_t end_secondary = 0;
            int32_t end_linear = 0;
            int32_t center_primary = 0;
            int32_t center_secondary = 0;
            float radius = 0.0f;
            float start_angle = 0.0f;
            float delta_angle = 0.0f;
            uint16_t total_segments = 0;
            uint16_t current_segment = 0;
        };

        State state = State::Idle;
        size_t program_index = 0;
        size_t line = 0;
        size_t block = 0;
        size_t text_offset = 0;
        uint16_t barrier_token = 0;
        uint8_t barrier_mask = 0;
        uint64_t dwell_until_us = 0;
        uint32_t feed = 0;
        int32_t spindle = 0;
        bool absolute = true;
        MotionMode motion_mode = MotionMode::Rapid;
        Plane plane = Plane::XY;
        bool inch_mode = false;
        size_t active_work = 0;
        size_t active_tool = 0;
        size_t pending_tool = 0;
        bool tool_length_active = false;
        int32_t tool_length_counts = 0;
        bool coolant_mist = false;
        bool coolant_flood = false;
        int32_t targets[motion::MAX_AXES]{};
        ArcState arc{};
        bool loaded = false;
    };

    Runtime() noexcept;

    bool load_selected(size_t channel) noexcept;
    bool start(size_t channel) noexcept;
    bool start_all_loaded() noexcept;
    bool stop(size_t channel) noexcept;
    void tick() noexcept;

    // Mid-program restart. Reloads the selected program on `channel`, then
    // does a motion-free scan of lines 0..target_line-1, applying only modal
    // side effects (absolute/inch/plane/motion-mode, feed/spindle, active
    // WCS, active tool, tool-length comp, coolant, axis targets). Axis
    // motion, homing, barriers, macros, dwells, M3/4/5 spindle commands,
    // and physical tool changes are deliberately skipped so the machine
    // doesn't move during the scan. Leaves the channel in State::Ready at
    // `target_line`; caller invokes start(channel) to begin execution.
    // Returns false on out-of-range line or unloaded program.
    bool restart_at_line(size_t channel, size_t target_line) noexcept;

    ChannelSnapshot snapshot(size_t channel) const noexcept;
    bool resolve_axis_word(size_t channel, char word, uint8_t& axis_out) const noexcept;

    static void thread_entry(void* arg);

private:
    bool tick_channel(size_t channel) noexcept;
    bool channel_settled(size_t channel) const noexcept;
    int spindle_axis_for_channel(size_t channel) const noexcept;
    uint64_t now_us() const noexcept;

    ChannelState channels_[programs::MAX_CHANNELS]{};
};

extern Runtime g_runtime;

} // namespace cnc::interp
