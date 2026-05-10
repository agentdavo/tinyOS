// SPDX-License-Identifier: MIT OR Apache-2.0

#include "interpreter.hpp"

#include "../cnc/offsets.hpp"
#include "../automation/macro_runtime.hpp"
#include "../automation/signals.hpp"
#include "../ethercat/master.hpp"
#include "../ui/operator_api.hpp"
#include "../miniOS.hpp"
#include "../klog.hpp"
#include "../util_math.hpp"

namespace cnc::interp {

namespace {

using kernel::util::math::absf;
using kernel::util::math::sqrt_approx;
using kernel::util::math::sin_approx;
using kernel::util::math::cos_approx;
using kernel::util::math::atan2_approx;
using kernel::util::math::acos_approx;
using kernel::util::math::clampf;
using kernel::util::math::kPi;
using kernel::util::math::kTwoPi;

struct ParsedLine {
    bool set_absolute = false;
    bool absolute = true;
    bool set_inch_mode = false;
    bool inch_mode = false;
    bool set_plane = false;
    Plane plane = Plane::XY;
    bool set_motion_mode = false;
    MotionMode motion_mode = MotionMode::Rapid;
    bool dwell = false;
    bool machine_coordinates = false;
    int select_work = -1;
    bool home = false;
    bool tool_length_enable = false;
    bool tool_length_cancel = false;
    // G43.4 / G43.5 TCP enable; G49.1 TCP cancel. Routed through the
    // Runtime::set_tcp_active path so the executor mirrors the operator
    // CLI verb's effect — keeps the snapshot and the interpreter modal
    // state coherent regardless of which path flipped it.
    bool tcp_enable = false;
    bool tcp_cancel = false;
    long tool_word = -1;
    long h_word = -1;
    long m_code = -1;
    long p_word = -1;
    long q_word = -1;
    uint32_t feed = 0;
    bool set_feed = false;
    int32_t spindle = 0;
    bool set_spindle = false;
    bool axis_words[6]{};
    float axis_values[6]{};
    float i_word = 0.0f;
    float j_word = 0.0f;
    float k_word = 0.0f;
    float r_word = 0.0f;
    bool has_i = false;
    bool has_j = false;
    bool has_k = false;
    bool has_r = false;
};

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

float arc_delta_for_mode(float start_angle, float end_angle, MotionMode mode) {
    float delta = end_angle - start_angle;
    if (mode == MotionMode::ArcCW) {
        if (delta >= 0.0f) delta -= kTwoPi;
    } else {
        if (delta <= 0.0f) delta += kTwoPi;
    }
    return delta;
}

bool solve_arc_center_from_radius(float start_u, float start_v,
                                  float end_u, float end_v,
                                  float radius_word,
                                  MotionMode mode,
                                  float& center_u, float& center_v) {
    const float chord_u = end_u - start_u;
    const float chord_v = end_v - start_v;
    const float chord = sqrt_approx(chord_u * chord_u + chord_v * chord_v);
    const float radius = absf(radius_word);
    if (chord <= 0.0f || radius < (chord * 0.5f)) return false;

    const float mid_u = (start_u + end_u) * 0.5f;
    const float mid_v = (start_v + end_v) * 0.5f;
    const float half = chord * 0.5f;
    const float height = sqrt_approx(radius * radius - half * half);
    const float perp_u = -chord_v / chord;
    const float perp_v = chord_u / chord;

    const float cand_u[2] = {mid_u + perp_u * height, mid_u - perp_u * height};
    const float cand_v[2] = {mid_v + perp_v * height, mid_v - perp_v * height};
    int best = 0;
    float best_score = -1.0f;
    const bool want_long = radius_word < 0.0f;
    for (int idx = 0; idx < 2; ++idx) {
        const float start_angle = atan2_approx(start_v - cand_v[idx], start_u - cand_u[idx]);
        const float end_angle = atan2_approx(end_v - cand_v[idx], end_u - cand_u[idx]);
        const float delta = arc_delta_for_mode(start_angle, end_angle, mode);
        const bool long_arc = absf(delta) > kPi;
        const float score = long_arc == want_long ? absf(delta) : -absf(delta);
        if (score > best_score) {
            best_score = score;
            best = idx;
        }
    }
    center_u = cand_u[best];
    center_v = cand_v[best];
    return true;
}

float parse_float_token(const char* s, size_t& idx) {
    bool neg = false;
    if (s[idx] == '-') { neg = true; ++idx; }
    else if (s[idx] == '+') { ++idx; }

    float value = 0.0f;
    while (s[idx] >= '0' && s[idx] <= '9') {
        value = value * 10.0f + static_cast<float>(s[idx] - '0');
        ++idx;
    }
    if (s[idx] == '.') {
        ++idx;
        float place = 0.1f;
        while (s[idx] >= '0' && s[idx] <= '9') {
            value += static_cast<float>(s[idx] - '0') * place;
            place *= 0.1f;
            ++idx;
        }
    }
    return neg ? -value : value;
}

void copy_line(const char* text, size_t text_size, size_t offset, char* out, size_t out_size, size_t& next_offset) {
    size_t i = 0;
    next_offset = offset;
    while (next_offset < text_size && (text[next_offset] == '\n' || text[next_offset] == '\r')) ++next_offset;
    while (next_offset < text_size && text[next_offset] != '\n' && text[next_offset] != '\r' &&
           i + 1 < out_size) {
        out[i++] = text[next_offset++];
    }
    out[i] = '\0';
    while (next_offset < text_size && text[next_offset] != '\n' && text[next_offset] != '\r') ++next_offset;
    while (next_offset < text_size && (text[next_offset] == '\n' || text[next_offset] == '\r')) ++next_offset;
}

size_t axis_letter_index(char word) {
    switch (word) {
        case 'X': return 0;
        case 'Y': return 1;
        case 'Z': return 2;
        case 'A': return 3;
        case 'B': return 4;
        case 'C': return 5;
        default: return 6;
    }
}

} // namespace

// Forward declaration: parse_line is a file-scope static helper defined later
// in this TU (near tick_channel for locality). Needed here so Runtime member
// functions above that point — specifically Runtime::restart_at_line — can
// call it.
static bool parse_line(const char* s, ParsedLine& parsed);

Runtime g_runtime;

Runtime::Runtime() noexcept = default;

uint64_t Runtime::now_us() const noexcept {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    return t ? t->get_system_time_us() : 0;
}

bool Runtime::load_selected(size_t channel) noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    if (!programs::g_store.open_selected(channel)) return false;
    auto& ch = channels_[channel];
    ch.program_index = programs::g_store.loaded(channel);
    ch.text_offset = 0;
    ch.line = 0;
    ch.block = 0;
    ch.barrier_token = 0;
    ch.barrier_mask = 0;
    ch.dwell_until_us = 0;
    ch.feed = 1000;
    ch.spindle = 0;
    ch.absolute = true;
    ch.motion_mode = MotionMode::Rapid;
    ch.plane = Plane::XY;
    ch.inch_mode = false;
    ch.active_work = offsets::g_service.active_work();
    ch.active_tool = offsets::g_service.active_tool();
    ch.pending_tool = ch.active_tool;
    ch.tool_length_active = false;
    ch.tool_length_counts = 0;
    ch.tool_length_x_counts = 0;
    ch.tool_length_y_counts = 0;
    ch.coolant_mist = false;
    ch.coolant_flood = false;
    ch.arc = Runtime::ChannelState::ArcState{};
    for (size_t i = 0; i < motion::MAX_AXES; ++i) {
        ch.targets[i] = motion::g_motion.axis(i).actual_pos.load(std::memory_order_relaxed);
    }
    ch.loaded = true;
    ch.state = State::Ready;
    return true;
}

bool Runtime::start(size_t channel) noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    auto& ch = channels_[channel];
    if (!ch.loaded && !load_selected(channel)) return false;
    ch.state = State::Running;
    return true;
}

bool Runtime::start_all_loaded() noexcept {
    bool any = false;
    for (size_t ch = 0; ch < programs::MAX_CHANNELS; ++ch) {
        if (programs::g_store.selected_program(ch) || channels_[ch].loaded) {
            any |= start(ch);
        }
    }
    return any;
}

bool Runtime::stop(size_t channel) noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    channels_[channel].state = State::Complete;
    channels_[channel].arc = ChannelState::ArcState{};
    return true;
}

bool Runtime::set_tcp_active(size_t channel, bool active) noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    channels_[channel].tcp_active = active;
    return true;
}

bool Runtime::tcp_active(size_t channel) const noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    return channels_[channel].tcp_active;
}

bool Runtime::restart_at_line(size_t channel, size_t target_line) noexcept {
    if (channel >= programs::MAX_CHANNELS) return false;
    if (!load_selected(channel)) return false;
    auto& state = channels_[channel];

    const auto* program = programs::g_store.loaded_program(channel);
    if (!program || !program->loaded) {
        state.state = State::Fault;
        return false;
    }

    // target_line == 0 means "start from the top" — load_selected already did it.
    while (state.line < target_line && state.text_offset < program->text_size) {
        char line[256];
        size_t next_offset = state.text_offset;
        copy_line(program->text.data(), program->text_size, state.text_offset,
                  line, sizeof(line), next_offset);
        state.text_offset = next_offset;
        ++state.line;

        ParsedLine parsed{};
        if (!parse_line(line, parsed)) continue;

        // Modal-only application. Mirrors tick_channel's state updates but
        // skips anything that would command motion or external peripherals:
        //   - no motion::g_motion calls (move_to, set_axis_velocity, homing,
        //     barriers)
        //   - no macros::g_runtime.start_mcode
        //   - no offsets::g_service.select_tool (physical tool change)
        //   - dwell just decays; M0/M1/M2/M30 ignored (dry scan runs through)
        if (parsed.set_absolute) state.absolute = parsed.absolute;
        if (parsed.set_inch_mode) state.inch_mode = parsed.inch_mode;
        if (parsed.set_plane) state.plane = parsed.plane;
        if (parsed.set_motion_mode) state.motion_mode = parsed.motion_mode;
        if (parsed.set_feed) state.feed = parsed.feed;
        if (parsed.set_spindle) state.spindle = parsed.spindle;

        if (parsed.select_work >= 0) {
            state.active_work = static_cast<size_t>(parsed.select_work);
            // Keep offsets service in sync so post-resume motion uses the
            // right WCS. This is a coherent state change, not a motion
            // command, so it's safe during the dry scan.
            (void)offsets::g_service.select_work(state.active_work);
        }

        if (parsed.tool_word > 0 &&
            parsed.tool_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
            state.pending_tool = static_cast<size_t>(parsed.tool_word - 1);
        }
        if (parsed.tool_length_cancel) {
            state.tool_length_active = false;
            state.tool_length_counts = 0;
            state.tool_length_x_counts = 0;
            state.tool_length_y_counts = 0;
        }
        if (parsed.tool_length_enable) {
            size_t tool = state.active_tool;
            if (parsed.h_word > 0 &&
                parsed.h_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
                tool = static_cast<size_t>(parsed.h_word - 1);
            }
            state.tool_length_active = true;
            const auto& tos = offsets::g_service.tool_offsets()[tool];
            state.tool_length_counts   = static_cast<int32_t>(tos.length   * 100.0f);
            state.tool_length_x_counts = static_cast<int32_t>(tos.length_x * 100.0f);
            state.tool_length_y_counts = static_cast<int32_t>(tos.length_y * 100.0f);
        }
        // G43.4 / G49.1 — TCP modal flag mirrors the operator-side toggle so
        // a dry-scan restart leaves the channel in the same TCP state the
        // program reached, regardless of which line we resumed from.
        if (parsed.tcp_cancel) state.tcp_active = false;
        if (parsed.tcp_enable) state.tcp_active = true;
        // M6 in the source program would physically change the tool, but on
        // a dry scan we only promote the pending index to active so the
        // reconstructed state reflects "this is the tool the program expects".
        // The operator is responsible for mounting that tool before resuming.
        if (parsed.m_code == 6) {
            state.active_tool = state.pending_tool;
            if (state.tool_length_active) {
                const auto& tos = offsets::g_service.tool_offsets()[state.active_tool];
                state.tool_length_counts   = static_cast<int32_t>(tos.length   * 100.0f);
                state.tool_length_x_counts = static_cast<int32_t>(tos.length_x * 100.0f);
                state.tool_length_y_counts = static_cast<int32_t>(tos.length_y * 100.0f);
            }
        }
        if (parsed.m_code == 7) state.coolant_mist = true;
        else if (parsed.m_code == 8) state.coolant_flood = true;
        else if (parsed.m_code == 9) {
            state.coolant_mist = false;
            state.coolant_flood = false;
        } else if (parsed.m_code == 3 || parsed.m_code == 4 || parsed.m_code == 5) {
            // Record spindle direction/speed intent without commanding the axis.
            int32_t speed = parsed.set_spindle ? parsed.spindle : state.spindle;
            if (speed < 0) speed = -speed;
            if (parsed.m_code == 5) state.spindle = 0;
            else state.spindle = (parsed.m_code == 4) ? -speed : speed;
        }

        // Track axis targets so "where the program thinks we are" reflects
        // everything up to target_line. This isn't motion — it's just
        // bookkeeping inside state.targets[].
        if (parsed.axis_words[0] || parsed.axis_words[1] || parsed.axis_words[2] ||
            parsed.axis_words[3] || parsed.axis_words[4] || parsed.axis_values[5]) {
            static constexpr char kAxisLetters[6] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
            for (size_t word_idx = 0; word_idx < 6; ++word_idx) {
                if (!parsed.axis_words[word_idx]) continue;
                uint8_t axis_idx = 0;
                if (!resolve_axis_word(channel, kAxisLetters[word_idx], axis_idx)) continue;
                if (axis_idx >= motion::MAX_AXES) continue;
                const float value = parsed.axis_values[word_idx];
                const float scale = state.inch_mode ? 2540.0f : 100.0f;
                const int32_t counts = static_cast<int32_t>(value * scale);
                if (state.absolute) state.targets[axis_idx] = counts;
                else state.targets[axis_idx] += counts;
            }
        }
    }

    if (state.line < target_line) {
        // Target line beyond program end — leave at EOF but signal Complete
        // so the operator sees that they asked for a line past the program.
        state.state = State::Complete;
        return false;
    }

    // text_offset now points at target_line; next tick_channel will execute it.
    state.block = state.line;  // resync block counter to match
    state.state = State::Ready;
    return true;
}

bool Runtime::channel_settled(size_t channel) const noexcept {
    // Look-ahead-aware "ready to dispatch the next block" predicate. With
    // motion's depth-N chain ring, the interpreter dispatches whenever
    // EVERY channel-axis has chain headroom. axis_chain_room returns
    // false when an axis's ring is at CHAIN_DEPTH; the interpreter
    // pauses until the trajectory pops at least one entry.
    if (channel >= motion::g_motion.channel_count()) return true;
    const auto& ch = motion::g_motion.channel(channel);
    if (ch.state != motion::ChannelState::Running) return false;
    for (uint8_t i = 0; i < ch.axis_count; ++i) {
        if (!motion::g_motion.axis_chain_room(ch.axis_indices[i])) return false;
    }
    return true;
}

bool Runtime::channel_motion_complete(size_t channel) const noexcept {
    if (channel >= motion::g_motion.channel_count()) return true;
    const auto& ch = motion::g_motion.channel(channel);
    if (ch.state != motion::ChannelState::Running) return false;
    for (uint8_t i = 0; i < ch.axis_count; ++i) {
        const auto& axis = motion::g_motion.axis(ch.axis_indices[i]);
        if (axis.traj_state != motion::TrajState::Idle &&
            axis.traj_state != motion::TrajState::Holding) {
            return false;
        }
        if (motion::g_motion.axis_chain_count(ch.axis_indices[i]) > 0) return false;
    }
    return true;
}

bool Runtime::resolve_axis_word(size_t channel, char word, uint8_t& axis_out) const noexcept {
    if (channel >= motion::g_motion.channel_count()) return false;
    const auto& ch = motion::g_motion.channel(channel);
    static constexpr char kChannel0Order[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
    static constexpr char kChannel1Order[] = {'X', 'Z', 'C', 'B', 'Y', 'A'};
    const char* order = channel == 0 ? kChannel0Order : kChannel1Order;
    for (uint8_t i = 0; i < ch.axis_count && i < 6; ++i) {
        if (order[i] == word) {
            axis_out = ch.axis_indices[i];
            return true;
        }
    }
    return false;
}

int Runtime::spindle_axis_for_channel(size_t channel) const noexcept {
    if (channel >= motion::g_motion.channel_count()) return -1;
    const auto& ch = motion::g_motion.channel(channel);
    if (channel == 1 && ch.axis_count > 2) return static_cast<int>(ch.axis_indices[2]);
    if (channel == 0 && ch.axis_count > 3) return static_cast<int>(ch.axis_indices[3]);
    return ch.axis_count > 0 ? static_cast<int>(ch.axis_indices[ch.axis_count - 1]) : -1;
}

static int32_t units_to_counts(const Runtime::ChannelState& state, float value) {
    const float scale = state.inch_mode ? 2540.0f : 100.0f;
    return static_cast<int32_t>(value * scale);
}

// state.feed is in units-per-minute (mm/min in metric, inch/min in imperial).
// Convert to counts-per-second so the motion kernel's path-velocity clamp
// can compare like-with-like. Returns 0 when the F-word hasn't been seen
// (state.feed defaults to 1000 in load_selected, but interpret as mm/min).
//
// The channel's feed override (set via the `override` CLI verb or the HMI
// slider) scales the resulting cps when state.override_active is true.
// M48 enables the slider; M49 disables it (treat permille as 1000).
static uint32_t feed_cps_for_state(const Runtime::ChannelState& state,
                                   size_t channel) noexcept {
    if (state.feed == 0) return 0;
    const uint64_t scale = state.inch_mode ? 2540u : 100u;
    uint64_t cps = (static_cast<uint64_t>(state.feed) * scale) / 60u;
    if (state.override_active && channel < motion::g_motion.channel_count()) {
        const uint16_t permille = motion::g_motion.channel(channel).overrides.feed_permille;
        cps = (cps * permille) / 1000u;
    }
    return static_cast<uint32_t>(cps);
}

// Combined-axis Euclidean distance in counts for the participating axes.
// Used to derive min_t_final_us = path / feed_cps so sync_move stretches
// the move to honour the F-word when an axis-aligned move would otherwise
// run at axis-vmax (faster than F when the move has multiple axis words).
static uint32_t combined_path_counts(const int32_t start[motion::MAX_AXES],
                                     const int32_t targets[motion::MAX_AXES],
                                     uint64_t axis_mask) noexcept {
    uint64_t sum_sq = 0;
    for (size_t i = 0; i < motion::MAX_AXES; ++i) {
        if ((axis_mask & (1ull << i)) == 0) continue;
        const int64_t d = static_cast<int64_t>(targets[i]) - start[i];
        sum_sq += static_cast<uint64_t>(d * d);
    }
    return static_cast<uint32_t>(sqrt_approx(static_cast<float>(sum_sq)));
}

static int axis_offset_slot(char word) {
    switch (word) {
        case 'X': return 0;
        case 'Y': return 1;
        case 'Z': return 2;
        case 'A': return 3;
        default: return -1;
    }
}

static motion::HomingPlan default_homing_plan() {
    motion::HomingPlan plan{};
    plan.trigger = motion::TriggerSource::LimitSwitch;
    plan.fast_speed_cps = 10000;
    plan.creep_speed_cps = 1000;
    plan.backoff_counts = 1000;
    plan.timeout_ms = 5000;
    return plan;
}

// `frac` is the first decimal digit of the G-code (43.4 → frac=4, 49.1 →
// frac=1). Most G-codes are integer (frac == 0) and the existing dispatch
// runs unchanged. Decimal cases (G43.4 / G49.1) route into separate
// ParsedLine flags so executor logic stays a flat set of if-statements.
static void parse_g_word(int code, int frac, ParsedLine& parsed) {
    if (code == 0) {
        parsed.set_motion_mode = true;
        parsed.motion_mode = MotionMode::Rapid;
    } else if (code == 1) {
        parsed.set_motion_mode = true;
        parsed.motion_mode = MotionMode::Linear;
    } else if (code == 2) {
        parsed.set_motion_mode = true;
        parsed.motion_mode = MotionMode::ArcCW;
    } else if (code == 3) {
        parsed.set_motion_mode = true;
        parsed.motion_mode = MotionMode::ArcCCW;
    } else if (code == 4) {
        parsed.dwell = true;
    } else if (code == 17) {
        parsed.set_plane = true;
        parsed.plane = Plane::XY;
    } else if (code == 18) {
        parsed.set_plane = true;
        parsed.plane = Plane::XZ;
    } else if (code == 19) {
        parsed.set_plane = true;
        parsed.plane = Plane::YZ;
    } else if (code == 20) {
        parsed.set_inch_mode = true;
        parsed.inch_mode = true;
    } else if (code == 21) {
        parsed.set_inch_mode = true;
        parsed.inch_mode = false;
    } else if (code == 28) {
        parsed.home = true;
    } else if (code == 43) {
        // G43          — classic tool-length compensation enable.
        // G43.4 / G43.5 — TCP enable. The active orientation lives on
        //                 cnc::interp::Runtime::set_tcp_active(channel, true);
        //                 the rotary axes are read by apply_tcp_correction.
        if (frac == 4 || frac == 5) parsed.tcp_enable = true;
        else                        parsed.tool_length_enable = true;
    } else if (code == 49) {
        // G49   — tool-length comp cancel. G49.1 also cancels TCP.
        if (frac == 1) parsed.tcp_cancel = true;
        else           parsed.tool_length_cancel = true;
    } else if (code == 53) {
        parsed.machine_coordinates = true;
    } else if (code >= 54 && code <= 59) {
        parsed.select_work = code - 54;
    } else if (code == 90) {
        parsed.set_absolute = true;
        parsed.absolute = true;
    } else if (code == 91) {
        parsed.set_absolute = true;
        parsed.absolute = false;
    }
}

static bool parse_line(const char* s, ParsedLine& parsed) {
    size_t i = 0;
    while (is_space(s[i])) ++i;
    if (s[i] == '\0' || s[i] == ';' || s[i] == '(') return false;

    while (s[i] != '\0') {
        if (is_space(s[i])) { ++i; continue; }
        if (s[i] == ';' || s[i] == '(') break;
        const char word = s[i++];
        if (word == 'N') {
            (void)parse_float_token(s, i);
        } else if (word == 'G') {
            // Decimal G-code dispatch (G43.4 / G49.1 / G33.1 / ...). The
            // integer part picks the major case in parse_g_word; the first
            // decimal digit selects sub-case. Round-half-up so 0.45 reads as
            // 4 — anything finer than .X is below the spec we ship.
            const float gv = parse_float_token(s, i);
            const int   gi = static_cast<int>(gv);
            const float gf = gv - static_cast<float>(gi);
            int frac = static_cast<int>(gf * 10.0f + 0.5f);
            if (frac >= 10) frac = 9;
            if (frac < 0)   frac = 0;
            parse_g_word(gi, frac, parsed);
        } else if (word == 'M') {
            parsed.m_code = static_cast<long>(parse_float_token(s, i));
        } else if (word == 'F') {
            parsed.feed = static_cast<uint32_t>(parse_float_token(s, i));
            parsed.set_feed = true;
        } else if (word == 'S') {
            parsed.spindle = static_cast<int32_t>(parse_float_token(s, i));
            parsed.set_spindle = true;
        } else if (word == 'P') {
            parsed.p_word = static_cast<long>(parse_float_token(s, i));
        } else if (word == 'Q') {
            parsed.q_word = static_cast<long>(parse_float_token(s, i));
        } else if (word == 'T') {
            parsed.tool_word = static_cast<long>(parse_float_token(s, i));
        } else if (word == 'H') {
            parsed.h_word = static_cast<long>(parse_float_token(s, i));
        } else if (word == 'I') {
            parsed.i_word = parse_float_token(s, i);
            parsed.has_i = true;
        } else if (word == 'J') {
            parsed.j_word = parse_float_token(s, i);
            parsed.has_j = true;
        } else if (word == 'K') {
            parsed.k_word = parse_float_token(s, i);
            parsed.has_k = true;
        } else if (word == 'R') {
            parsed.r_word = parse_float_token(s, i);
            parsed.has_r = true;
        } else {
            const size_t axis = axis_letter_index(word);
            if (axis < 6) {
                parsed.axis_words[axis] = true;
                parsed.axis_values[axis] = parse_float_token(s, i);
            } else {
                while (s[i] && !is_space(s[i]) && s[i] != ';' && s[i] != '(') ++i;
            }
        }
    }
    return true;
}

static int32_t work_offset_counts(const Runtime::ChannelState& state, char word) {
    const int slot = axis_offset_slot(word);
    if (slot < 0) return 0;
    const auto& work = offsets::g_service.work_offsets()[state.active_work];
    return static_cast<int32_t>(work.value.axis[slot] * 100.0f);
}

static int32_t tool_length_counts(const Runtime::ChannelState& state, char word) {
    if (!state.tool_length_active) return 0;
    switch (word) {
        case 'X': return state.tool_length_x_counts;
        case 'Y': return state.tool_length_y_counts;
        case 'Z': return state.tool_length_counts;
        default:  return 0;
    }
}

// Tool-tip → spindle-reference transform for 5-axis TCP. Composes a
// head-kinematics rotation of the tool vector and subtracts it from the
// requested tool-tip target so the X/Y/Z axes drive the spindle to the
// position that puts the tip at the requested point.
//
// Rotary targets[B] / targets[C] are expected in axis counts; conversion
// to radians uses the kernel's 100 cnt/degree convention. Linear length
// components are in axis counts (already pre-scaled into
// state.tool_length_*_counts at G43 time).
//
// Convention matches render::kinematic forward kinematics: R_C is yaw
// around +Z (machine vertical), R_B is pitch around +Y (head tilt). For
// a typical mill head with a spindle pointing -Z when B = 0 and C = 0
// and a positive length_z, the tool vector in the machine frame is
// (0, 0, -length_z), so the spindle reference position equals
// tool_tip - R · (length_x, length_y, length_z).
//
// Rotation composition is selectable via Runtime::tcp_order():
//   CB (default) — R = R_C · R_B  (B applied first, then C; matches the
//                  millturn TSV's C-then-B parent chain).
//   BC           — R = R_B · R_C  (C applied first, then B; AC/BC heads
//                  where B is the outer joint).
//
// Mode selectable via Runtime::tcp_mode():
//   Head — rotaries carry the tool. The composition above runs unchanged.
//   Tail — rotaries carry the workpiece. The relationship between the
//          requested tool-tip (in workpiece coords) and the machine-frame
//          target depends on the rotary pivot offset, which today is not
//          captured in the kinematic_model TSV. Until that lands, Tail
//          falls through to the Head path and emits a one-shot klog
//          warning so the operator notices.
static void apply_tcp_correction(Runtime::ChannelState& state,
                                 Runtime& runtime,
                                 size_t channel,
                                 int32_t targets[motion::MAX_AXES]) {
    if (!state.tcp_active) return;
    if (state.tool_length_counts == 0 &&
        state.tool_length_x_counts == 0 &&
        state.tool_length_y_counts == 0) return;
    uint8_t x_idx=0, y_idx=0, z_idx=0, b_idx=0, c_idx=0;
    const bool has_xy =
        runtime.resolve_axis_word(channel, 'X', x_idx) &&
        runtime.resolve_axis_word(channel, 'Y', y_idx);
    const bool has_z = runtime.resolve_axis_word(channel, 'Z', z_idx);
    if (!has_xy || !has_z) return;
    const bool has_b = runtime.resolve_axis_word(channel, 'B', b_idx);
    const bool has_c = runtime.resolve_axis_word(channel, 'C', c_idx);
    if (runtime.tcp_mode() == Runtime::TcpMode::Tail) {
        // One-shot warning so the operator sees the limitation in klog;
        // motion still advances using the head path so a multi-axis
        // toolpath at least retains the rotary correction's primary
        // contribution. Pivot-offset support lands with the kinematic_
        // model TSV extension (see CHANNELS.md "TCP" task).
        static bool warned_once = false;
        if (!warned_once) {
            warned_once = true;
            const char msg[] =
                "[tcp] tail-kinematics not implemented; falling back to head\n";
            kernel::klog::record(msg, sizeof(msg) - 1);
        }
    }
    // Convention: rotary axis counts at 100 cnt/degree → degrees/100 →
    // radians. cos(0)=1, sin(0)=0, so a 3-axis machine with no B/C falls
    // through with the tool vector unrotated and the correction reduces
    // to a constant offset on Z (the same effect tool_length_counts
    // already added in resolve_targets — TCP would double-correct, so
    // this path subtracts the tool_length component before applying the
    // rotated correction. Net: TCP supersedes G43 Z-only when active).
    const float beta_rad  = has_b
        ? (static_cast<float>(targets[b_idx]) / 100.0f) *
          kernel::util::math::kDegToRad
        : 0.0f;
    const float gamma_rad = has_c
        ? (static_cast<float>(targets[c_idx]) / 100.0f) *
          kernel::util::math::kDegToRad
        : 0.0f;
    const float cb = cos_approx(beta_rad),  sb = sin_approx(beta_rad);
    const float cc = cos_approx(gamma_rad), sc = sin_approx(gamma_rad);
    const float lx = static_cast<float>(state.tool_length_x_counts);
    const float ly = static_cast<float>(state.tool_length_y_counts);
    const float lz = static_cast<float>(state.tool_length_counts);

    float xw, yw, zw;
    if (runtime.tcp_order() == Runtime::TcpOrder::CB) {
        // R = R_C · R_B applied to (lx, ly, lz):
        //   step 1 — R_B (pitch around Y):
        //   step 2 — R_C (yaw around Z) on the R_B output.
        const float xb =  lx * cb + lz * sb;
        const float yb =  ly;
        const float zb = -lx * sb + lz * cb;
        xw = xb * cc - yb * sc;
        yw = xb * sc + yb * cc;
        zw = zb;
    } else {
        // R = R_B · R_C — apply R_C first, then R_B.
        const float xc =  lx * cc - ly * sc;
        const float yc =  lx * sc + ly * cc;
        const float zc =  lz;
        xw =  xc * cb + zc * sb;
        yw =  yc;
        zw = -xc * sb + zc * cb;
    }
    // Undo the G43-scalar Z addition that resolve_targets applied via
    // tool_length_counts() so we don't correct twice. tool_length_x/y
    // weren't applied (3-axis G43 path is Z-only) so no undo there.
    targets[z_idx] -= state.tool_length_counts;
    // Apply the rotated tool offset.
    targets[x_idx] -= static_cast<int32_t>(xw);
    targets[y_idx] -= static_cast<int32_t>(yw);
    targets[z_idx] -= static_cast<int32_t>(zw);
}

static bool resolve_targets(Runtime::ChannelState& state,
                            Runtime& runtime,
                            size_t channel,
                            const ParsedLine& parsed,
                            int32_t targets[motion::MAX_AXES],
                            uint64_t& axis_mask,
                            int32_t out_tip_targets[3] = nullptr) {
    axis_mask = 0;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) targets[ax] = state.targets[ax];
    if (out_tip_targets) {
        out_tip_targets[0] = state.tool_tip_pos[0];
        out_tip_targets[1] = state.tool_tip_pos[1];
        out_tip_targets[2] = state.tool_tip_pos[2];
    }
    static constexpr char kAxisLetters[6] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
    bool saw_axis = false;
    for (size_t i = 0; i < 6; ++i) {
        if (!parsed.axis_words[i]) continue;
        uint8_t axis_idx = 0;
        if (!runtime.resolve_axis_word(channel, kAxisLetters[i], axis_idx)) continue;
        const int32_t delta = units_to_counts(state, parsed.axis_values[i]);
        if (state.absolute) {
            int32_t target = delta;
            if (!parsed.machine_coordinates) {
                target += work_offset_counts(state, kAxisLetters[i]);
                target += tool_length_counts(state, kAxisLetters[i]);
            }
            targets[axis_idx] = target;
        } else {
            targets[axis_idx] = state.targets[axis_idx] + delta;
        }
        axis_mask |= (1ull << axis_idx);
        saw_axis = true;
    }
    // Capture the pre-correction X/Y/Z target as the new tool-tip target
    // before apply_tcp_correction rewrites the axis values into spindle-
    // ref coords. For 3-axis (TCP off) tool-tip and spindle-ref coincide;
    // the values are identical to targets[X/Y/Z]. Caller (execute_axes)
    // uses out_tip_targets vs state.tool_tip_pos to compute tool-tip path
    // length for the feedrate cap and updates state.tool_tip_pos AFTER
    // the move commits.
    if (saw_axis && out_tip_targets) {
        uint8_t x_idx=0, y_idx=0, z_idx=0;
        if (runtime.resolve_axis_word(channel, 'X', x_idx))
            out_tip_targets[0] = targets[x_idx];
        if (runtime.resolve_axis_word(channel, 'Y', y_idx))
            out_tip_targets[1] = targets[y_idx];
        if (runtime.resolve_axis_word(channel, 'Z', z_idx))
            out_tip_targets[2] = targets[z_idx];
    }
    if (saw_axis) apply_tcp_correction(state, runtime, channel, targets);
    return saw_axis;
}

static bool execute_axes(Runtime::ChannelState& state,
                         Runtime& runtime,
                         size_t channel,
                         const ParsedLine& parsed) {
    uint64_t axis_mask = 0;
    int32_t targets[motion::MAX_AXES]{};
    int32_t tip_targets[3]{};
    if (!resolve_targets(state, runtime, channel, parsed, targets, axis_mask,
                         tip_targets)) return false;
    // Path-velocity cap from the F-word: enforce that combined motion
    // equals the requested feedrate. With TCP active, "combined motion"
    // is measured in tool-tip frame so a reorientation move (B/C words
    // changing tip orientation while X/Y/Z chase the rotated tool
    // vector) doesn't run faster than the requested surface feedrate.
    // Skipped on Rapid (G0 runs at axis vmax) — F doesn't apply there
    // per spec.
    uint64_t min_t_final_us = 0;
    if (state.motion_mode != MotionMode::Rapid) {
        const uint32_t feed_cps = feed_cps_for_state(state, channel);
        if (feed_cps > 0) {
            uint32_t path_counts;
            if (state.tcp_active) {
                // Tool-tip frame distance: sqrt of XYZ tip deltas.
                const int64_t dx = static_cast<int64_t>(tip_targets[0]) - state.tool_tip_pos[0];
                const int64_t dy = static_cast<int64_t>(tip_targets[1]) - state.tool_tip_pos[1];
                const int64_t dz = static_cast<int64_t>(tip_targets[2]) - state.tool_tip_pos[2];
                const uint64_t sum_sq =
                    static_cast<uint64_t>(dx * dx) +
                    static_cast<uint64_t>(dy * dy) +
                    static_cast<uint64_t>(dz * dz);
                path_counts = static_cast<uint32_t>(sqrt_approx(static_cast<float>(sum_sq)));
            } else {
                path_counts = combined_path_counts(state.targets, targets, axis_mask);
            }
            min_t_final_us = (static_cast<uint64_t>(path_counts) * 1'000'000ULL) / feed_cps;
        }
    }
    uint16_t block_id = 0;
    const bool ok = motion::g_motion.sync_move(
        axis_mask, targets, 0, 1, 3, 20000, false, min_t_final_us, &block_id);
    if (!ok) return false;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) {
        if ((axis_mask & (1ull << ax)) != 0) state.targets[ax] = targets[ax];
    }
    state.tool_tip_pos[0] = tip_targets[0];
    state.tool_tip_pos[1] = tip_targets[1];
    state.tool_tip_pos[2] = tip_targets[2];
    if (block_id != 0) state.last_dispatched_block_id = block_id;
    // Block counter is now incremented unconditionally in tick_channel for
    // every parsed line, not per-helper. See the comment there.
    return true;
}

static bool plan_arc(Runtime::ChannelState& state,
                     Runtime& runtime,
                     size_t channel,
                     const ParsedLine& parsed) {
    int32_t targets[motion::MAX_AXES]{};
    uint64_t axis_mask = 0;
    if (!resolve_targets(state, runtime, channel, parsed, targets, axis_mask)) return false;

    char primary_letter = 'X';
    char secondary_letter = 'Y';
    char linear_letter = 'Z';
    float center_primary_off = 0.0f;
    float center_secondary_off = 0.0f;
    bool has_primary_off = false;
    bool has_secondary_off = false;
    switch (state.plane) {
        case Plane::XY:
            primary_letter = 'X'; secondary_letter = 'Y'; linear_letter = 'Z';
            center_primary_off = parsed.i_word; center_secondary_off = parsed.j_word;
            has_primary_off = parsed.has_i; has_secondary_off = parsed.has_j;
            break;
        case Plane::XZ:
            primary_letter = 'X'; secondary_letter = 'Z'; linear_letter = 'Y';
            center_primary_off = parsed.i_word; center_secondary_off = parsed.k_word;
            has_primary_off = parsed.has_i; has_secondary_off = parsed.has_k;
            break;
        case Plane::YZ:
            primary_letter = 'Y'; secondary_letter = 'Z'; linear_letter = 'X';
            center_primary_off = parsed.j_word; center_secondary_off = parsed.k_word;
            has_primary_off = parsed.has_j; has_secondary_off = parsed.has_k;
            break;
    }
    uint8_t primary_axis = 0;
    uint8_t secondary_axis = 0;
    if (!runtime.resolve_axis_word(channel, primary_letter, primary_axis) ||
        !runtime.resolve_axis_word(channel, secondary_letter, secondary_axis)) {
        return false;
    }

    uint8_t linear_axis = 0;
    const bool has_linear_axis = runtime.resolve_axis_word(channel, linear_letter, linear_axis);
    const int32_t start_primary = state.targets[primary_axis];
    const int32_t start_secondary = state.targets[secondary_axis];
    const int32_t start_linear = has_linear_axis ? state.targets[linear_axis] : 0;
    const int32_t end_primary = targets[primary_axis];
    const int32_t end_secondary = targets[secondary_axis];
    const int32_t end_linear = has_linear_axis ? targets[linear_axis] : 0;
    int32_t center_primary = 0;
    int32_t center_secondary = 0;
    if (has_primary_off && has_secondary_off) {
        center_primary = start_primary + units_to_counts(state, center_primary_off);
        center_secondary = start_secondary + units_to_counts(state, center_secondary_off);
    } else if (parsed.has_r) {
        float solved_u = 0.0f;
        float solved_v = 0.0f;
        if (!solve_arc_center_from_radius(static_cast<float>(start_primary), static_cast<float>(start_secondary),
                                          static_cast<float>(end_primary), static_cast<float>(end_secondary),
                                          units_to_counts(state, parsed.r_word),
                                          state.motion_mode, solved_u, solved_v)) {
            return false;
        }
        center_primary = static_cast<int32_t>(solved_u);
        center_secondary = static_cast<int32_t>(solved_v);
    } else {
        return false;
    }
    const float start_dx = static_cast<float>(start_primary - center_primary);
    const float start_dy = static_cast<float>(start_secondary - center_secondary);
    const float end_dx = static_cast<float>(end_primary - center_primary);
    const float end_dy = static_cast<float>(end_secondary - center_secondary);
    const float radius = sqrt_approx(start_dx * start_dx + start_dy * start_dy);
    if (radius <= 0.5f) return false;

    const float start_angle = atan2_approx(start_dy, start_dx);
    float end_angle = atan2_approx(end_dy, end_dx);
    float delta_angle = arc_delta_for_mode(start_angle, end_angle, state.motion_mode);
    // Chord-error segmentation: bound the deviation between the polygonal
    // approximation and the true arc to `tol_counts`. From geometry,
    // chord_err = R * (1 - cos(theta/2)), so theta_max = 2 * acos(1 - tol/R).
    // tol = 1 count (~10 µm at 100 cnt/mm) is conservative for finishing
    // cuts; opens up to many segments at large radii but caps below at the
    // old 96-segment ceiling so a degenerate radius can't cost the kernel a
    // multi-thousand-segment plan. The previous arc_length/200 rule gave
    // ~30 mm of chord error on a 100 mm-radius full circle — visibly faceted.
    constexpr float kChordTolCounts = 1.0f;
    uint16_t segments = 4;
    if (radius > kChordTolCounts) {
        const float ratio = 1.0f - kChordTolCounts / radius;
        const float theta_max = 2.0f * acos_approx(ratio);
        if (theta_max > 1e-4f) {
            const float n = absf(delta_angle) / theta_max;
            int32_t want = static_cast<int32_t>(n) + 1;
            if (want < 4) want = 4;
            if (want > 256) want = 256;
            segments = static_cast<uint16_t>(want);
        }
    }

    auto& arc = state.arc;
    arc.active = true;
    arc.primary_axis = primary_axis;
    arc.secondary_axis = secondary_axis;
    arc.linear_axis = linear_axis;
    arc.has_linear_axis = has_linear_axis;
    arc.axis_mask = axis_mask | (1ull << primary_axis) | (1ull << secondary_axis);
    if (has_linear_axis && (parsed.axis_words[axis_letter_index(linear_letter)] || start_linear != end_linear)) {
        arc.axis_mask |= (1ull << linear_axis);
    }
    arc.start_primary = start_primary;
    arc.start_secondary = start_secondary;
    arc.start_linear = start_linear;
    arc.end_primary = end_primary;
    arc.end_secondary = end_secondary;
    arc.end_linear = end_linear;
    arc.center_primary = center_primary;
    arc.center_secondary = center_secondary;
    arc.radius = radius;
    arc.start_angle = start_angle;
    arc.delta_angle = delta_angle;
    arc.total_segments = segments;
    arc.current_segment = 0;
    return true;
}

static bool advance_arc(Runtime::ChannelState& state, size_t channel) {
    auto& arc = state.arc;
    if (!arc.active || arc.total_segments == 0) return false;
    ++arc.current_segment;
    const bool last = arc.current_segment >= arc.total_segments;
    int32_t targets[motion::MAX_AXES]{};
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) targets[ax] = state.targets[ax];

    if (last) {
        targets[arc.primary_axis] = arc.end_primary;
        targets[arc.secondary_axis] = arc.end_secondary;
        if (arc.has_linear_axis) targets[arc.linear_axis] = arc.end_linear;
    } else {
        const float t = static_cast<float>(arc.current_segment) / static_cast<float>(arc.total_segments);
        const float angle = arc.start_angle + arc.delta_angle * t;
        targets[arc.primary_axis] = arc.center_primary + static_cast<int32_t>(arc.radius * cos_approx(angle));
        targets[arc.secondary_axis] = arc.center_secondary + static_cast<int32_t>(arc.radius * sin_approx(angle));
        if (arc.has_linear_axis) {
            const float linear = static_cast<float>(arc.start_linear) +
                                 static_cast<float>(arc.end_linear - arc.start_linear) * t;
            targets[arc.linear_axis] = static_cast<int32_t>(linear);
        }
    }

    // Helical feedrate: per-segment combined path length =
    // sqrt(arc_chord_xy² + linear_segment_z²). Splitting the F-word across
    // arc and linear components keeps the tool moving at the requested
    // surface feedrate regardless of pitch — without this, a 360° helix
    // with 50 mm rise ran arc-only at F and the linear axis was just
    // dragged along, so the actual surface speed went up by sqrt(1 + (rise/circ)²).
    const uint32_t feed_cps = feed_cps_for_state(state, channel);
    uint64_t min_t_final_us = 0;
    if (feed_cps > 0) {
        const float arc_seg_len =
            arc.radius * absf(arc.delta_angle) /
            static_cast<float>(arc.total_segments);
        float linear_seg_len = 0.0f;
        if (arc.has_linear_axis) {
            linear_seg_len = static_cast<float>(arc.end_linear - arc.start_linear) /
                             static_cast<float>(arc.total_segments);
            if (linear_seg_len < 0.0f) linear_seg_len = -linear_seg_len;
        }
        const float combined = sqrt_approx(
            arc_seg_len * arc_seg_len + linear_seg_len * linear_seg_len);
        if (combined > 0.0f) {
            min_t_final_us = static_cast<uint64_t>(
                (combined * 1'000'000.0f) / static_cast<float>(feed_cps));
        }
    }

    uint16_t arc_block_id = 0;
    const bool ok = motion::g_motion.sync_move(
        arc.axis_mask, targets, 0, 1, 3, 20000, false, min_t_final_us, &arc_block_id);
    if (!ok) return false;
    if (arc_block_id != 0) state.last_dispatched_block_id = arc_block_id;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) {
        if ((arc.axis_mask & (1ull << ax)) != 0) state.targets[ax] = targets[ax];
    }
    if (last) arc.active = false;
    return true;
}

static bool execute_home(Runtime::ChannelState& /*state*/,
                         Runtime& runtime,
                         size_t channel,
                         const ParsedLine& parsed) {
    if (channel >= motion::g_motion.channel_count()) return false;
    const auto& ch = motion::g_motion.channel(channel);
    const motion::HomingPlan plan = default_homing_plan();
    bool started = false;
    static constexpr char kAxisLetters[6] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
    for (uint8_t i = 0; i < ch.axis_count; ++i) {
        bool included = true;
        if (parsed.axis_words[0] || parsed.axis_words[1] || parsed.axis_words[2] ||
            parsed.axis_words[3] || parsed.axis_words[4] || parsed.axis_words[5]) {
            included = false;
            for (size_t word_idx = 0; word_idx < 6; ++word_idx) {
                uint8_t resolved = 0;
                if (parsed.axis_words[word_idx] &&
                    runtime.resolve_axis_word(channel, kAxisLetters[word_idx], resolved) &&
                    resolved == ch.axis_indices[i]) {
                    included = true;
                    break;
                }
            }
        }
        if (!included) continue;
        motion::g_motion.start_homing(ch.axis_indices[i], plan);
        started = true;
    }
    return started;
}

bool Runtime::tick_channel(size_t channel) noexcept {
    auto& state = channels_[channel];
    if (state.state == State::Idle || state.state == State::Complete || state.state == State::Fault) return true;

    if (state.state == State::WaitingBarrier) {
        if (channel < motion::g_motion.channel_count() &&
            motion::g_motion.channel(channel).state == motion::ChannelState::Running) {
            state.state = State::Running;
        }
        return true;
    }
    if (state.state == State::WaitingMacro) {
        if (macros::g_runtime.channel_fault(channel)) {
            state.state = State::Fault;
            return false;
        }
        if (macros::g_runtime.channel_done(channel)) {
            state.state = State::Running;
        }
        return true;
    }
    if (state.state == State::Dwell) {
        if (now_us() >= state.dwell_until_us) state.state = State::Running;
        return true;
    }
    if (state.state == State::Ready) return true;
    if (!channel_settled(channel)) return true;
    // M62/M63 sync-to-motion-end drain. Each pending op is gated on its
    // own predecessor block id; with the depth-N chain ring, multiple
    // motion blocks can be queued ahead and an M62 inserted between them
    // should fire when ITS particular predecessor finishes, not when the
    // entire queue drains. We walk the pending list and only fire ops
    // whose gate_block_id has been retired by the channel — others stay
    // queued for later. Ops with gate=0 (queued when no motion was yet
    // dispatched on this channel) wait for channel_motion_complete, the
    // pre-tagging behaviour.
    const uint16_t completed = motion::g_motion.channel_completed_block_id(channel);
    if (state.pending_outputs_count > 0) {
        const bool fully_complete = channel_motion_complete(channel);
        uint8_t kept = 0;
        for (uint8_t k = 0; k < state.pending_outputs_count; ++k) {
            auto& slot = state.pending_outputs[k];
            if (!slot.used) continue;
            const bool gate_passed =
                (slot.gate_block_id == 0) ? fully_complete
                                          : (completed >= slot.gate_block_id);
            if (gate_passed) {
                (void)automation::signals::set_named_signal_bool(slot.name, slot.on);
                slot.used = false;
                continue;
            }
            // Compact the kept entry forward.
            if (kept != k) state.pending_outputs[kept] = slot;
            ++kept;
        }
        state.pending_outputs_count = kept;
    }
    if (state.arc.active) {
        // Tier 1c: arcs feed segments into the chain ring as fast as the
        // ring accepts them, instead of one segment per ~1 ms tick. A
        // 32-segment arc that previously took 32 ticks now finishes
        // chain-fill in 4 ticks (CHAIN_DEPTH = 8) and the trajectory runs
        // at chained-cruise speed across the segment seams. Bound the
        // inner loop at CHAIN_DEPTH * 2 so a degenerate arc (huge segment
        // count, all of them direction-mismatched and stalling the chain
        // pop) can't spin forever.
        for (size_t guard = 0; guard < motion::Axis::CHAIN_DEPTH * 2; ++guard) {
            if (!channel_settled(channel)) break;  // chain ring full
            if (!state.arc.active) break;          // arc completed mid-burst
            if (!advance_arc(state, channel)) {
                state.state = State::Fault;
                return false;
            }
        }
        return true;
    }

    const auto* program = programs::g_store.loaded_program(channel);
    if (!program || !program->loaded) {
        state.state = State::Fault;
        return false;
    }
    if (state.text_offset >= program->text_size) {
        state.state = State::Complete;
        return true;
    }

    char line[256];
    size_t next_offset = state.text_offset;
    copy_line(program->text.data(), program->text_size, state.text_offset, line, sizeof(line), next_offset);
    state.text_offset = next_offset;
    ++state.line;

    ParsedLine parsed{};
    if (!parse_line(line, parsed)) return true;

    // Block counter advances on every non-empty parsed line, not only on
    // motion-bearing or homing lines. Pure-modal lines (G90, F1000, M8)
    // and macro/M-code lines used to leave state.block stuck at the last
    // motion block, which made the HMI's "block %u" counter undercount.
    ++state.block;

    if (parsed.set_absolute) state.absolute = parsed.absolute;
    if (parsed.set_inch_mode) state.inch_mode = parsed.inch_mode;
    if (parsed.set_plane) state.plane = parsed.plane;
    if (parsed.set_motion_mode) state.motion_mode = parsed.motion_mode;
    if (parsed.set_feed) state.feed = parsed.feed;
    if (parsed.set_spindle) state.spindle = parsed.spindle;

    if (parsed.select_work >= 0 &&
        !offsets::g_service.select_work(static_cast<size_t>(parsed.select_work))) {
        state.state = State::Fault;
        return false;
    }
    if (parsed.select_work >= 0) state.active_work = static_cast<size_t>(parsed.select_work);

    if (parsed.tool_word > 0 && parsed.tool_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
        state.pending_tool = static_cast<size_t>(parsed.tool_word - 1);
    }
    if (parsed.tool_length_cancel) {
        state.tool_length_active = false;
        state.tool_length_counts = 0;
        state.tool_length_x_counts = 0;
        state.tool_length_y_counts = 0;
    }
    if (parsed.tool_length_enable) {
        size_t tool = state.active_tool;
        if (parsed.h_word > 0 && parsed.h_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
            tool = static_cast<size_t>(parsed.h_word - 1);
        }
        state.tool_length_active = true;
        const auto& tos = offsets::g_service.tool_offsets()[tool];
        state.tool_length_counts   = static_cast<int32_t>(tos.length   * 100.0f);
        state.tool_length_x_counts = static_cast<int32_t>(tos.length_x * 100.0f);
        state.tool_length_y_counts = static_cast<int32_t>(tos.length_y * 100.0f);
    }
    // G43.4 / G43.5 enable TCP; G49.1 cancels. set_tcp_active here ensures
    // operator API + interpreter modal flag stay coherent — the operator
    // could have flipped TCP via the `tcp` CLI verb mid-program; G43.4 in
    // the part program then re-asserts the same flag with no surprise.
    if (parsed.tcp_cancel) (void)set_tcp_active(channel, false);
    if (parsed.tcp_enable) (void)set_tcp_active(channel, true);

    if (parsed.home) {
        if (!execute_home(state, *this, channel, parsed)) {
            state.state = State::Fault;
            return false;
        }
        return true;
    }

    if (parsed.m_code == 0 || parsed.m_code == 1 || parsed.m_code == 2 || parsed.m_code == 30) {
        state.state = State::Complete;
        return true;
    }
    if (parsed.m_code == 6) {
        if (!offsets::g_service.select_tool(state.pending_tool)) {
            state.state = State::Fault;
            return false;
        }
        state.active_tool = state.pending_tool;
        if (state.tool_length_active) {
            const auto& tos = offsets::g_service.tool_offsets()[state.active_tool];
            state.tool_length_counts   = static_cast<int32_t>(tos.length   * 100.0f);
            state.tool_length_x_counts = static_cast<int32_t>(tos.length_x * 100.0f);
            state.tool_length_y_counts = static_cast<int32_t>(tos.length_y * 100.0f);
        }
        return true;
    }
    if (parsed.m_code == 7) {
        state.coolant_mist = true;
        // Modal flag mirrors the bus assertion so restart_at_line and the
        // snapshot reader stay coherent even when the signal isn't bound.
        (void)automation::signals::set_named_signal_bool("coolant_mist", true);
        return true;
    }
    if (parsed.m_code == 8) {
        state.coolant_flood = true;
        (void)automation::signals::set_named_signal_bool("coolant_flood", true);
        return true;
    }
    if (parsed.m_code == 9) {
        state.coolant_mist = false;
        state.coolant_flood = false;
        (void)automation::signals::set_named_signal_bool("coolant_mist", false);
        (void)automation::signals::set_named_signal_bool("coolant_flood", false);
        return true;
    }
    if (parsed.m_code == 10) {
        // M10/M11 chuck/clamp engage/release. Optional per spec — silently
        // no-ops when no chuck_clamp signal is bound to a real slot.
        (void)automation::signals::set_named_signal_bool("chuck_clamp", true);
        return true;
    }
    if (parsed.m_code == 11) {
        (void)automation::signals::set_named_signal_bool("chuck_clamp", false);
        return true;
    }
    if (parsed.m_code == 62 || parsed.m_code == 63 ||
        parsed.m_code == 64 || parsed.m_code == 65) {
        // LinuxCNC convention: M62/M63 sync to motion-queue end (defer the
        // output flip until the preceding motion block completes), M64/M65
        // are immediate. M64/M65 stay on the immediate path; M62/M63
        // enqueue into state.pending_outputs[] and tick_channel drains the
        // queue at its next channel-settled edge.
        if (parsed.p_word < 0) return true;
        const uint32_t idx = static_cast<uint32_t>(parsed.p_word);
        char name[16];
        if (!automation::signals::aux_dout_signal_name(idx, name, sizeof(name))) {
            return true;  // out-of-range P-word: log-and-continue per spec
        }
        const bool on = (parsed.m_code == 62 || parsed.m_code == 64);
        const bool sync_to_motion_end = (parsed.m_code == 62 || parsed.m_code == 63);
        if (sync_to_motion_end) {
            if (state.pending_outputs_count <
                ChannelState::MAX_PENDING_OUTPUTS) {
                auto& slot = state.pending_outputs[state.pending_outputs_count];
                size_t k = 0;
                while (name[k] && k + 1 < sizeof(slot.name)) {
                    slot.name[k] = name[k];
                    ++k;
                }
                slot.name[k] = '\0';
                slot.on = on;
                slot.used = true;
                // Tag with the most recently dispatched motion block.
                // Drain fires when channel_completed_block_id reaches
                // (or passes) this value. If no motion has run yet on
                // this channel (gate=0), the drain fires at the next
                // motion-complete edge — same as pre-tier-1d.
                slot.gate_block_id = state.last_dispatched_block_id;
                ++state.pending_outputs_count;
            }
            // Overflow silently drops; deeper M62 chains are unusual.
        } else {
            (void)automation::signals::set_named_signal_bool(name, on);
        }
        return true;
    }
    if (parsed.m_code == 3 || parsed.m_code == 4 || parsed.m_code == 5) {
        // Skip when either master is deadline-faulted: same gate as the
        // operator spindle button (operator_api::spindle_set_rpm). The
        // already-running spindle keeps spinning via the held velocity
        // command on the drive — we just don't push a new one.
        if (ethercat::g_master_a.is_deadline_faulted() ||
            ethercat::g_master_b.is_deadline_faulted()) {
            return true;
        }
        // Resolve through operator_api::spindle_axis_index so M3/M4/M5 hit
        // the same axis the operator's spindle button does, regardless of
        // topology layout (legacy axis 3 vs whatever the loaded topology
        // names "spindle").
        const int spindle_axis = kernel::ui::operator_api::spindle_axis_index();
        if (spindle_axis >= 0) {
            int32_t speed = parsed.set_spindle ? parsed.spindle : state.spindle;
            if (speed < 0) speed = -speed;
            if (parsed.m_code == 5) state.spindle = 0;
            else state.spindle = (parsed.m_code == 4) ? -speed : speed;
            int32_t commanded = (parsed.m_code == 5) ? 0 : state.spindle;
            // Apply the channel's spindle override when M48 is in effect.
            if (state.override_active && commanded != 0 &&
                channel < motion::g_motion.channel_count()) {
                const uint16_t permille =
                    motion::g_motion.channel(channel).overrides.spindle_permille;
                commanded = static_cast<int32_t>(
                    (static_cast<int64_t>(commanded) * permille) / 1000);
            }
            motion::g_motion.set_axis_velocity(static_cast<size_t>(spindle_axis),
                                               commanded);
        }
        return true;
    }
    // M48 / M49 — feed/spindle override enable/disable. Modal flag only;
    // takes effect on the next feed_cps_for_state / spindle dispatch.
    if (parsed.m_code == 48) { state.override_active = true;  return true; }
    if (parsed.m_code == 49) { state.override_active = false; return true; }
    // M100/M110/M200 — historical sync barrier verbs.
    // M101..M109 — named alternates. Token defaults to (mcode * 1000 + line)
    // so a program with multiple M101s at different lines doesn't collide
    // unless P explicitly forces a shared token. Q (participants mask)
    // defaults to all-channels (0x3 today; 0xFF leaves room for >2).
    if (parsed.m_code == 100 || parsed.m_code == 110 || parsed.m_code == 200 ||
        (parsed.m_code >= 101 && parsed.m_code <= 109)) {
        const long mcode = parsed.m_code;
        const uint16_t default_token = static_cast<uint16_t>(
            (mcode * 1000l + static_cast<long>(state.line)) & 0xFFFF);
        const uint16_t token = static_cast<uint16_t>(
            parsed.p_word >= 0 ? parsed.p_word : default_token);
        const uint8_t mask = static_cast<uint8_t>(parsed.q_word >= 0 ? parsed.q_word : 0x3);
        const bool ok = motion::g_motion.arrive_at_barrier(channel, token, mask, 1, 3, 20000);
        if (!ok) {
            state.state = State::Fault;
            return false;
        }
        state.barrier_token = token;
        state.barrier_mask = mask;
        state.state = State::WaitingBarrier;
        return true;
    }
    if (parsed.m_code >= 300 && parsed.m_code <= 399) {
        if (!macros::g_runtime.start_mcode(channel, static_cast<uint16_t>(parsed.m_code))) {
            state.state = State::Fault;
            return false;
        }
        state.state = State::WaitingMacro;
        return true;
    }

    if (parsed.dwell) {
        state.dwell_until_us = now_us() + static_cast<uint64_t>(parsed.p_word >= 0 ? parsed.p_word : 0) * 1000ULL;
        state.state = State::Dwell;
        return true;
    }

    if (parsed.set_motion_mode &&
        (state.motion_mode == MotionMode::ArcCW || state.motion_mode == MotionMode::ArcCCW)) {
        state.motion_mode = parsed.motion_mode;
    }

    if (parsed.axis_words[0] || parsed.axis_words[1] || parsed.axis_words[2] ||
        parsed.axis_words[3] || parsed.axis_words[4] || parsed.axis_words[5]) {
        const bool ok = (state.motion_mode == MotionMode::ArcCW || state.motion_mode == MotionMode::ArcCCW)
            ? plan_arc(state, *this, channel, parsed)
            : execute_axes(state, *this, channel, parsed);
        if (!ok) {
            state.state = State::Fault;
            return false;
        }
    }
    return true;
}

void Runtime::tick() noexcept {
    for (size_t channel = 0; channel < programs::MAX_CHANNELS; ++channel) {
        (void)tick_channel(channel);
    }
}

ChannelSnapshot Runtime::snapshot(size_t channel) const noexcept {
    ChannelSnapshot snap{};
    snap.channel = channel;
    if (channel >= programs::MAX_CHANNELS) return snap;
    const auto& st = channels_[channel];
    snap.state = st.state;
    snap.program_index = st.program_index;
    snap.line = st.line;
    snap.block = st.block;
    snap.barrier_token = st.barrier_token;
    snap.barrier_mask = st.barrier_mask;
    snap.feed = st.feed;
    snap.spindle = st.spindle;
    snap.absolute = st.absolute;
    snap.motion_mode = st.motion_mode;
    snap.plane = st.plane;
    snap.inch_mode = st.inch_mode;
    snap.active_work = st.active_work;
    snap.active_tool = st.active_tool;
    snap.tool_length_active = st.tool_length_active;
    snap.coolant_mist = st.coolant_mist;
    snap.coolant_flood = st.coolant_flood;
    if (const auto* program = programs::g_store.loaded_program(channel)) snap.program_name = program->name;
    return snap;
}

void Runtime::thread_entry(void*) {
    for (;;) {
        g_runtime.tick();
        auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
        if (t) {
            const uint64_t target = t->get_system_time_us() + 1000ULL;
            t->wait_until_ns(target * 1000ULL);
        }
    }
}

} // namespace cnc::interp
