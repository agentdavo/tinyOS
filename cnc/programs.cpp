// SPDX-License-Identifier: MIT OR Apache-2.0

#include "programs.hpp"
#include "../util.hpp"

namespace cnc::programs {

namespace {

bool is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

bool nearly_equal(float a, float b) {
    float delta = a - b;
    if (delta < 0.0f) delta = -delta;
    return delta < 0.0001f;
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

float arc_delta_for_mode(float start_angle, float end_angle, int motion_mode) {
    static constexpr float kTwoPi = 6.28318530718f;
    float delta = end_angle - start_angle;
    if (motion_mode == 2) {
        if (delta >= 0.0f) delta -= kTwoPi;
    } else {
        if (delta <= 0.0f) delta += kTwoPi;
    }
    return delta;
}

bool solve_arc_center_from_radius(float start_u, float start_v,
                                  float end_u, float end_v,
                                  float radius_word,
                                  int motion_mode,
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
        const float delta = arc_delta_for_mode(start_angle, end_angle, motion_mode);
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

float rotary_to_preview(float a, float b, float c) {
    return a * 0.01f + b * 0.015f + c * 0.005f;
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

} // namespace

Store g_store;

Store::Store() {
    add_seed("FACING.NGC",
             "G90\n"
             "G0 X0 Y0 Z5\n"
             "G1 Z-1 F400\n"
             "G1 X40 Y0\n"
             "G1 X40 Y20\n"
             "G1 X0 Y20\n"
             "G1 X0 Y0\n"
             "G0 Z5\n");
    add_seed("POCKET.NGC",
             "G90\n"
             "G0 X5 Y5 Z5\n"
             "G1 Z-2 F250\n"
             "G1 X35 Y5\n"
             "G1 X35 Y15\n"
             "G1 X5 Y15\n"
             "G1 X5 Y5\n"
             "G1 X20 Y10\n"
             "G0 Z5\n");
    add_seed("DRILL.NGC",
             "G90\n"
             "G0 X10 Y10 Z5\n"
             "G1 Z-6 F120\n"
             "G0 Z5\n"
             "G91\n"
             "G0 X20 Y0\n"
             "G1 Z-6\n"
             "G0 Z5\n");
    for (size_t channel = 0; channel < MAX_CHANNELS; ++channel) {
        channels_[channel].selected = 0;
        channels_[channel].loaded = 0;
    }
}

size_t Store::selected(size_t channel) const noexcept {
    return channel < MAX_CHANNELS ? channels_[channel].selected : 0;
}

size_t Store::loaded(size_t channel) const noexcept {
    return channel < MAX_CHANNELS ? channels_[channel].loaded : 0;
}

const ProgramEntry* Store::selected_program(size_t channel) const noexcept {
    if (channel >= MAX_CHANNELS) return nullptr;
    const size_t idx = channels_[channel].selected;
    return idx < count_ ? &programs_[idx] : nullptr;
}

ProgramEntry* Store::selected_program(size_t channel) noexcept {
    if (channel >= MAX_CHANNELS) return nullptr;
    const size_t idx = channels_[channel].selected;
    return idx < count_ ? &programs_[idx] : nullptr;
}

const ProgramEntry* Store::loaded_program(size_t channel) const noexcept {
    if (channel >= MAX_CHANNELS) return nullptr;
    const size_t idx = channels_[channel].loaded;
    return idx < count_ ? &programs_[idx] : nullptr;
}

ProgramEntry* Store::loaded_program(size_t channel) noexcept {
    if (channel >= MAX_CHANNELS) return nullptr;
    const size_t idx = channels_[channel].loaded;
    return idx < count_ ? &programs_[idx] : nullptr;
}

bool Store::select(size_t channel, size_t idx) noexcept {
    if (channel >= MAX_CHANNELS || idx >= count_) return false;
    kernel::core::ScopedLock lock(lock_);
    channels_[channel].selected = idx;
    return true;
}

bool Store::find_by_name(const char* name, size_t& out_idx) const noexcept {
    if (!name) return false;
    for (size_t i = 0; i < count_; ++i) {
        if (kernel::util::kstrcmp(programs_[i].name, name) == 0) {
            out_idx = i;
            return true;
        }
    }
    return false;
}

bool Store::write_program(const char* name, const char* text) noexcept {
    if (!name || !text) return false;
    kernel::core::ScopedLock lock(lock_);
    size_t idx = 0;
    if (!find_by_name(name, idx)) {
        if (count_ >= MAX_PROGRAMS) return false;
        idx = count_++;
    }
    kernel::util::safe_strcpy(programs_[idx].name, name, sizeof(programs_[idx].name));
    kernel::util::safe_strcpy(programs_[idx].path, name, sizeof(programs_[idx].path));
    programs_[idx].text_size = kernel::util::kstrlen(text);
    if (programs_[idx].text_size >= MAX_PROGRAM_SIZE) programs_[idx].text_size = MAX_PROGRAM_SIZE - 1;
    for (size_t i = 0; i < programs_[idx].text_size; ++i) programs_[idx].text[i] = text[i];
    programs_[idx].text[programs_[idx].text_size] = '\0';
    programs_[idx].loaded = true;
    return parse_preview(programs_[idx]);
}

bool Store::open_selected() noexcept {
    return open_selected(0);
}

bool Store::open_selected(size_t channel) noexcept {
    if (channel >= MAX_CHANNELS) return false;
    kernel::core::ScopedLock lock(lock_);
    const size_t selected_idx = channels_[channel].selected;
    if (selected_idx >= count_) return false;
    programs_[selected_idx].loaded = true;
    channels_[channel].loaded = selected_idx;
    return parse_preview(programs_[selected_idx]);
}

bool Store::rebuild_preview(size_t idx) noexcept {
    if (idx >= count_) return false;
    kernel::core::ScopedLock lock(lock_);
    return parse_preview(programs_[idx]);
}

bool Store::add_seed(const char* name, const char* text) noexcept {
    if (count_ >= MAX_PROGRAMS) return false;
    ProgramEntry& e = programs_[count_++];
    kernel::util::safe_strcpy(e.name, name, sizeof(e.name));
    kernel::util::safe_strcpy(e.path, name, sizeof(e.path));
    e.text_size = kernel::util::kstrlen(text);
    if (e.text_size >= MAX_PROGRAM_SIZE) e.text_size = MAX_PROGRAM_SIZE - 1;
    for (size_t i = 0; i < e.text_size; ++i) e.text[i] = text[i];
    e.text[e.text_size] = '\0';
    e.loaded = true;
    return parse_preview(e);
}

bool Store::parse_preview(ProgramEntry& entry) noexcept {
    entry.preview.point_count = 0;
    entry.preview.motion_blocks = 0;
    entry.preview.parsed_lines = 0;
    entry.preview.valid = false;
    entry.preview.truncated = false;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    bool absolute = true;
    int motion_mode = 1;
    int plane = 17;
    bool first = true;
    const char* s = entry.text.data();
    size_t i = 0;

    auto push_point = [&](float px, float py, float pz) {
        if (entry.preview.point_count >= MAX_PREVIEW_POINTS) {
            entry.preview.truncated = true;
            return;
        }
        auto& p = entry.preview.points[entry.preview.point_count++];
        p = {px, py, pz};
        if (first) {
            entry.preview.min = p;
            entry.preview.max = p;
            first = false;
        } else {
            if (px < entry.preview.min.x) entry.preview.min.x = px;
            if (py < entry.preview.min.y) entry.preview.min.y = py;
            if (pz < entry.preview.min.z) entry.preview.min.z = pz;
            if (px > entry.preview.max.x) entry.preview.max.x = px;
            if (py > entry.preview.max.y) entry.preview.max.y = py;
            if (pz > entry.preview.max.z) entry.preview.max.z = pz;
        }
    };

    push_point(x, y, z);
    while (s[i] != '\0') {
        while (is_space(s[i])) ++i;
        if (s[i] == '\0') break;

        ++entry.preview.parsed_lines;
        float nx = x;
        float ny = y;
        float nz = z;
        float na = a;
        float nb = b;
        float nc = c;
        float i_off = 0.0f;
        float j_off = 0.0f;
        float k_off = 0.0f;
        float r_word = 0.0f;
        bool saw_axis = false;
        bool saw_linear_y = false;
        bool saw_rotary = false;
        bool has_i = false;
        bool has_j = false;
        bool has_k = false;
        bool has_r = false;

        while (s[i] != '\0' && s[i] != '\n') {
            if (s[i] == ';' || s[i] == '(') {
                while (s[i] != '\0' && s[i] != '\n') ++i;
                break;
            }
            if (s[i] == 'G') {
                ++i;
                size_t tmp = i;
                const int g = static_cast<int>(parse_float_token(s, tmp));
                if (g == 90) absolute = true;
                else if (g == 91) absolute = false;
                else if (g == 0 || g == 1 || g == 2 || g == 3) motion_mode = g;
                else if (g == 17 || g == 18 || g == 19) plane = g;
                i = tmp;
            } else if (s[i] == 'X') {
                ++i;
                const float value = parse_float_token(s, i);
                nx = absolute ? value : (x + value);
                saw_axis = true;
            } else if (s[i] == 'Y') {
                ++i;
                const float value = parse_float_token(s, i);
                ny = absolute ? value : (y + value);
                saw_axis = true;
                saw_linear_y = true;
            } else if (s[i] == 'Z') {
                ++i;
                const float value = parse_float_token(s, i);
                nz = absolute ? value : (z + value);
                saw_axis = true;
            } else if (s[i] == 'A') {
                ++i;
                const float value = parse_float_token(s, i);
                na = absolute ? value : (a + value);
                saw_axis = true;
                saw_rotary = true;
            } else if (s[i] == 'B') {
                ++i;
                const float value = parse_float_token(s, i);
                nb = absolute ? value : (b + value);
                saw_axis = true;
                saw_rotary = true;
            } else if (s[i] == 'C') {
                ++i;
                const float value = parse_float_token(s, i);
                nc = absolute ? value : (c + value);
                saw_axis = true;
                saw_rotary = true;
            } else if (s[i] == 'I') {
                ++i;
                i_off = parse_float_token(s, i);
                has_i = true;
            } else if (s[i] == 'J') {
                ++i;
                j_off = parse_float_token(s, i);
                has_j = true;
            } else if (s[i] == 'K') {
                ++i;
                k_off = parse_float_token(s, i);
                has_k = true;
            } else if (s[i] == 'R') {
                ++i;
                r_word = parse_float_token(s, i);
                has_r = true;
            } else {
                ++i;
            }
        }

        if (!saw_linear_y && saw_rotary) {
            ny = rotary_to_preview(na, nb, nc);
        }

        if (saw_axis &&
            (!nearly_equal(nx, x) || !nearly_equal(ny, y) || !nearly_equal(nz, z) ||
             !nearly_equal(na, a) || !nearly_equal(nb, b) || !nearly_equal(nc, c))) {
            if (motion_mode == 2 || motion_mode == 3) {
                float start_u = x;
                float start_v = plane == 17 ? y : z;
                float end_u = nx;
                float end_v = plane == 17 ? ny : nz;
                float center_u = 0.0f;
                float center_v = 0.0f;
                if (plane == 19) {
                    start_u = y;
                    end_u = ny;
                    start_v = z;
                    end_v = nz;
                } else if (plane == 18) {
                    start_v = z;
                    end_v = nz;
                }
                bool have_center = false;
                if ((plane == 17 && has_i && has_j) ||
                    (plane == 18 && has_i && has_k) ||
                    (plane == 19 && has_j && has_k)) {
                    center_u = plane == 19 ? (y + j_off) : (x + i_off);
                    center_v = plane == 17 ? (y + j_off) : (z + k_off);
                    have_center = true;
                } else if (has_r) {
                    have_center = solve_arc_center_from_radius(start_u, start_v, end_u, end_v,
                                                               r_word, motion_mode, center_u, center_v);
                }
                if (have_center) {
                    const float start_angle = atan2_approx(start_v - center_v, start_u - center_u);
                    float end_angle = atan2_approx(end_v - center_v, end_u - center_u);
                    float delta = arc_delta_for_mode(start_angle, end_angle, motion_mode);
                    const float radius = sqrt_approx((start_u - center_u) * (start_u - center_u) +
                                                     (start_v - center_v) * (start_v - center_v));
                    int segments = static_cast<int>((absf(delta) * radius) / 2.5f) + 1;
                    if (segments < 4) segments = 4;
                    if (segments > 64) segments = 64;
                    for (int seg = 1; seg <= segments; ++seg) {
                        const float t = static_cast<float>(seg) / static_cast<float>(segments);
                        const float angle = start_angle + delta * t;
                        float px = nx;
                        float py = ny;
                        float pz = nz;
                        if (plane == 17) {
                            px = center_u + radius * cos_approx(angle);
                            py = center_v + radius * sin_approx(angle);
                            pz = z + (nz - z) * t;
                        } else if (plane == 18) {
                            px = center_u + radius * cos_approx(angle);
                            pz = center_v + radius * sin_approx(angle);
                            py = y + (ny - y) * t;
                        } else {
                            py = center_u + radius * cos_approx(angle);
                            pz = center_v + radius * sin_approx(angle);
                            px = x + (nx - x) * t;
                        }
                        push_point(px, py, pz);
                    }
                } else {
                    push_point(nx, ny, nz);
                }
            } else {
                push_point(nx, ny, nz);
            }
            x = nx;
            y = ny;
            z = nz;
            a = na;
            b = nb;
            c = nc;
            ++entry.preview.motion_blocks;
        }

        while (s[i] == '\n' || s[i] == '\r') ++i;
    }

    entry.preview.valid = entry.preview.point_count >= 2;
    return entry.preview.valid;
}

} // namespace cnc::programs
