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
        // M48 enables feed/spindle override sliders, M49 disables them
        // (treats permille as fixed at 1000). Default true so a fresh
        // program respects the operator's overrides until told otherwise.
        bool override_active = true;
        bool tool_length_active = false;
        // Tool-Centre-Point (TCP) mode. When enabled with a non-zero
        // tool-length vector, the interpreter treats X/Y/Z axis words in
        // motion lines as TOOL-TIP coordinates in the part frame. The
        // current (or move-target) B/C orientation is composed with the
        // tool vector to yield the spindle-reference position the
        // mechanical axes need to move to. Disabled = motion words
        // command the spindle reference directly (3-axis convention).
        // Future: G43.4 / G49.1 to enable/disable from G-code (parser
        // needs decimal G-code dispatch first); CLI 'tcp on/off' until
        // then.
        bool tcp_active = false;
        // Tool-tip position in part-frame counts, kept separately from
        // the axis-frame state.targets so the F-word feedrate cap can
        // measure path length in the tool-tip frame when TCP is on.
        // For 3-axis (TCP off) this stays in sync with state.targets[X/Y/Z]
        // and the axis-frame path length is identical to the tool-tip
        // path length, so the existing min_t_final logic doesn't change.
        int32_t tool_tip_pos[3]{0, 0, 0};
        // Tool-length vector in axis counts. tool_length_counts is the Z
        // component for 3-axis backwards compatibility; tool_length_x_counts
        // / tool_length_y_counts default to 0 and are honoured by
        // tool_length_counts() so any axis whose physical letter is X/Y/Z
        // gets the right scalar component automatically. 5-axis TCP work
        // populates all three; 3-axis mills only see the Z field move.
        int32_t tool_length_counts = 0;
        int32_t tool_length_x_counts = 0;
        int32_t tool_length_y_counts = 0;
        bool coolant_mist = false;
        bool coolant_flood = false;
        int32_t targets[motion::MAX_AXES]{};
        ArcState arc{};
        bool loaded = false;

        // M62/M63 sync-to-motion-queue-end: each line records into this
        // queue; tick_channel drains it on the next channel-settled edge
        // (which today = "the most recently committed motion completed",
        // and once block look-ahead lands = "the predecessor block in the
        // pipeline finished"). Cap is small — typical use is one or two
        // M62/M63 between motion blocks. Overflow drops the new entry
        // and logs nothing visible; that's worse than wrong-timing but
        // arguably more obvious as a programming error.
        struct PendingOutput {
            char name[16] = {};
            bool on = false;
            bool used = false;
            // Block id of the motion block this op is gated on. Drain
            // fires when channel_completed_block_id >= gate_block_id.
            // 0 = no motion ahead (drain immediately at next motion-
            // complete edge — same as the pre-tier-1d behaviour).
            uint16_t gate_block_id = 0;
        };
        static constexpr size_t MAX_PENDING_OUTPUTS = 4;
        PendingOutput pending_outputs[MAX_PENDING_OUTPUTS]{};
        uint8_t pending_outputs_count = 0;
        // Block id of the most recent motion-bearing line dispatched on
        // this channel. Captured from sync_move's out_block_id so M62/M63
        // can tag their pending output with "predecessor block done".
        uint16_t last_dispatched_block_id = 0;
    };

    Runtime() noexcept;

    bool load_selected(size_t channel) noexcept;
    bool start(size_t channel) noexcept;
    bool start_all_loaded() noexcept;
    bool stop(size_t channel) noexcept;
    void tick() noexcept;

    // Tool-Centre-Point mode toggle. Returns false on out-of-range
    // channel. Independent of tool_length_active — TCP without a tool
    // length is effectively a no-op since the rotation is applied to a
    // zero vector. CLI / setup persistence drive this until a G-code
    // G43.4 / G49.1 path lands.
    bool set_tcp_active(size_t channel, bool active) noexcept;
    bool tcp_active(size_t channel) const noexcept;

    // Rotation order for the head-kinematics composition. CB = R_C·R_B (the
    // mill-turn TSV's C-then-B parenting, the historical default). BC = R_B
    // ·R_C, used on AC/BC heads where B is parented to base. Set globally;
    // read per-tick by apply_tcp_correction. Tail-kinematics gets its own
    // mode; rotation order still applies inside it.
    enum class TcpOrder : uint8_t { CB = 0, BC = 1 };
    enum class TcpMode  : uint8_t { Head = 0, Tail = 1 };
    void      set_tcp_order(TcpOrder order) noexcept { tcp_order_ = order; }
    TcpOrder  tcp_order() const noexcept             { return tcp_order_; }
    void      set_tcp_mode(TcpMode mode) noexcept    { tcp_mode_ = mode; }
    TcpMode   tcp_mode() const noexcept              { return tcp_mode_; }

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
    // Look-ahead-aware "ready to dispatch the next block" predicate. Returns
    // true when every channel-axis has an empty chain slot — motion may
    // still be physically in flight on the active target. Used to gate the
    // line-read loop so the interpreter can run one block ahead.
    bool channel_settled(size_t channel) const noexcept;
    // Strict "all motion has stopped" predicate — Holding/Idle on every
    // axis and no chained target queued. Used by the M62/M63 sync-end
    // drain so deferred-output ops fire when the preceding motion truly
    // finishes, not when the chain slot opens up.
    bool channel_motion_complete(size_t channel) const noexcept;
    int spindle_axis_for_channel(size_t channel) const noexcept;
    uint64_t now_us() const noexcept;

    ChannelState channels_[programs::MAX_CHANNELS]{};
    TcpOrder     tcp_order_ = TcpOrder::CB;
    TcpMode      tcp_mode_  = TcpMode::Head;
};

extern Runtime g_runtime;

} // namespace cnc::interp
