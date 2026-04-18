// SPDX-License-Identifier: MIT OR Apache-2.0

#include "interpreter.hpp"

#include "../cnc/offsets.hpp"
#include "../automation/macro_runtime.hpp"
#include "../miniOS.hpp"

namespace cnc::interp {

namespace {

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

float absf(float v) {
    return v < 0.0f ? -v : v;
}

float sqrt_approx(float v) {
    if (v <= 0.0f) return 0.0f;
    float x = v > 1.0f ? v : 1.0f;
    for (int i = 0; i < 6; ++i) x = 0.5f * (x + v / x);
    return x;
}

float wrap_pi(float x) {
    static constexpr float kPi = 3.14159265359f;
    static constexpr float kTwoPi = 6.28318530718f;
    while (x > kPi) x -= kTwoPi;
    while (x < -kPi) x += kTwoPi;
    return x;
}

float sin_approx(float x) {
    static constexpr float kB = 1.27323954474f;
    static constexpr float kC = -0.40528473457f;
    x = wrap_pi(x);
    const float y = kB * x + kC * x * absf(x);
    return 0.225f * (y * absf(y) - y) + y;
}

float cos_approx(float x) {
    static constexpr float kHalfPi = 1.57079632679f;
    return sin_approx(x + kHalfPi);
}

float atan_approx(float z) {
    const float az = absf(z);
    if (az <= 1.0f) return z / (1.0f + 0.28f * z * z);
    const float base = 1.57079632679f - (az / (az * az + 0.28f));
    return z < 0.0f ? -base : base;
}

float atan2_approx(float y, float x) {
    static constexpr float kPi = 3.14159265359f;
    if (x > 0.0f) return atan_approx(y / x);
    if (x < 0.0f && y >= 0.0f) return atan_approx(y / x) + kPi;
    if (x < 0.0f && y < 0.0f) return atan_approx(y / x) - kPi;
    if (y > 0.0f) return 1.57079632679f;
    if (y < 0.0f) return -1.57079632679f;
    return 0.0f;
}

float arc_delta_for_mode(float start_angle, float end_angle, MotionMode mode) {
    static constexpr float kTwoPi = 6.28318530718f;
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
        const bool long_arc = absf(delta) > 3.14159265359f;
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
        }
        if (parsed.tool_length_enable) {
            size_t tool = state.active_tool;
            if (parsed.h_word > 0 &&
                parsed.h_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
                tool = static_cast<size_t>(parsed.h_word - 1);
            }
            state.tool_length_active = true;
            state.tool_length_counts =
                static_cast<int32_t>(offsets::g_service.tool_offsets()[tool].length * 100.0f);
        }
        // M6 in the source program would physically change the tool, but on
        // a dry scan we only promote the pending index to active so the
        // reconstructed state reflects "this is the tool the program expects".
        // The operator is responsible for mounting that tool before resuming.
        if (parsed.m_code == 6) {
            state.active_tool = state.pending_tool;
            if (state.tool_length_active) {
                state.tool_length_counts = static_cast<int32_t>(
                    offsets::g_service.tool_offsets()[state.active_tool].length * 100.0f);
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
    if (channel >= motion::g_motion.channel_count()) return true;
    const auto& ch = motion::g_motion.channel(channel);
    if (ch.state != motion::ChannelState::Running) return false;
    for (uint8_t i = 0; i < ch.axis_count; ++i) {
        const auto& axis = motion::g_motion.axis(ch.axis_indices[i]);
        if (axis.traj_state != motion::TrajState::Idle &&
            axis.traj_state != motion::TrajState::Holding) {
            return false;
        }
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

static void parse_g_word(int code, ParsedLine& parsed) {
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
        parsed.tool_length_enable = true;
    } else if (code == 49) {
        parsed.tool_length_cancel = true;
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
            parse_g_word(static_cast<int>(parse_float_token(s, i)), parsed);
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
    if (!state.tool_length_active || word != 'Z') return 0;
    return state.tool_length_counts;
}

static bool resolve_targets(Runtime::ChannelState& state,
                            Runtime& runtime,
                            size_t channel,
                            const ParsedLine& parsed,
                            int32_t targets[motion::MAX_AXES],
                            uint64_t& axis_mask) {
    axis_mask = 0;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) targets[ax] = state.targets[ax];
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
    return saw_axis;
}

static bool execute_axes(Runtime::ChannelState& state,
                         Runtime& runtime,
                         size_t channel,
                         const ParsedLine& parsed) {
    uint64_t axis_mask = 0;
    int32_t targets[motion::MAX_AXES]{};
    if (!resolve_targets(state, runtime, channel, parsed, targets, axis_mask)) return false;
    const bool ok = motion::g_motion.sync_move(axis_mask, targets, 0, 1, 3, 20000, false);
    if (!ok) return false;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) {
        if ((axis_mask & (1ull << ax)) != 0) state.targets[ax] = targets[ax];
    }
    ++state.block;
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
    const float arc_length = absf(delta_angle) * radius;
    uint16_t segments = static_cast<uint16_t>(arc_length / 200.0f) + 1;
    if (segments < 4) segments = 4;
    if (segments > 96) segments = 96;

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
    ++state.block;
    return true;
}

static bool advance_arc(Runtime::ChannelState& state) {
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

    const bool ok = motion::g_motion.sync_move(arc.axis_mask, targets, 0, 1, 3, 20000, false);
    if (!ok) return false;
    for (size_t ax = 0; ax < motion::MAX_AXES; ++ax) {
        if ((arc.axis_mask & (1ull << ax)) != 0) state.targets[ax] = targets[ax];
    }
    if (last) arc.active = false;
    return true;
}

static bool execute_home(Runtime::ChannelState& state,
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
    if (started) ++state.block;
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
    if (state.arc.active) {
        if (!advance_arc(state)) {
            state.state = State::Fault;
            return false;
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
    }
    if (parsed.tool_length_enable) {
        size_t tool = state.active_tool;
        if (parsed.h_word > 0 && parsed.h_word <= static_cast<long>(offsets::TOOL_OFFSET_COUNT)) {
            tool = static_cast<size_t>(parsed.h_word - 1);
        }
        state.tool_length_active = true;
        state.tool_length_counts = static_cast<int32_t>(offsets::g_service.tool_offsets()[tool].length * 100.0f);
    }

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
            state.tool_length_counts = static_cast<int32_t>(
                offsets::g_service.tool_offsets()[state.active_tool].length * 100.0f);
        }
        return true;
    }
    if (parsed.m_code == 7) {
        state.coolant_mist = true;
        return true;
    }
    if (parsed.m_code == 8) {
        state.coolant_flood = true;
        return true;
    }
    if (parsed.m_code == 9) {
        state.coolant_mist = false;
        state.coolant_flood = false;
        return true;
    }
    if (parsed.m_code == 3 || parsed.m_code == 4 || parsed.m_code == 5) {
        const int spindle_axis = spindle_axis_for_channel(channel);
        if (spindle_axis >= 0) {
            int32_t speed = parsed.set_spindle ? parsed.spindle : state.spindle;
            if (speed < 0) speed = -speed;
            if (parsed.m_code == 5) state.spindle = 0;
            else state.spindle = (parsed.m_code == 4) ? -speed : speed;
            motion::g_motion.set_axis_velocity(static_cast<size_t>(spindle_axis),
                                               parsed.m_code == 5 ? 0 : state.spindle);
        }
        return true;
    }
    if (parsed.m_code == 100 || parsed.m_code == 110 || parsed.m_code == 200) {
        const uint16_t token = static_cast<uint16_t>(parsed.p_word >= 0 ? parsed.p_word : (state.line & 0xFFFF));
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
