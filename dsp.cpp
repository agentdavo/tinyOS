// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.cpp
 * @brief Digital Signal Processing (DSP) subsystem implementation for miniOS v1.7.
 * @details
 * Implements a comprehensive set of audio processing nodes with advanced crossover support for up
 * to 4-way speaker systems, using Butterworth, Bessel, and Linkwitz-Riley filters (6, 12, 18, 24
 * dB/octave) and high/low shelf filters. Includes all professional audio nodes: gain, mixer,
 * parametric EQ, limiter, noise gate, stereo width, waveshaper, envelope follower, compressor,
 * delay, reverb, chorus, flanger, pitch shifter, convolution reverb, phase correction, SRC,
 * FIR/IIR, FFT equalizer, and network audio. Updated in v1.7 with improved error handling,
 * clearer documentation, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::string_view for string processing
 * - std::atomic for thread-safe updates
 *
 * @version 1.7
 * @see dsp.hpp, miniOS.hpp, audio.hpp, util.hpp, net.hpp
 */

#include "dsp.hpp"
#include "util.hpp"
#include "net.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <random>
#include <limits>

namespace kernel::dsp {

namespace {
// Constants
constexpr float PI = 3.14159265358979323846f;
constexpr float MIN_FREQ = 20.0f;
constexpr float MAX_FREQ = 20000.0f;
constexpr float MIN_Q = 0.1f;
constexpr float MAX_Q = 10.0f;
constexpr float MIN_GAIN_DB = -24.0f;
constexpr float MAX_GAIN_DB = 24.0f;
} // namespace

// --- Utility Functions ---
void generate_hann_window(std::span<float> window) noexcept {
    if (window.empty()) return;
    size_t n = window.size();
    for (size_t i = 0; i < n; ++i) {
        window[i] = 0.5f * (1.0f - std::cos(2.0f * PI * i / (n - 1)));
    }
}

void generate_biquad_coeffs(std::string_view type, float fc, float q, float gain_db, float sample_rate,
                           std::array<float, 3>& b, std::array<float, 3>& a) noexcept {
    if (fc < MIN_FREQ || fc > MAX_FREQ || q < MIN_Q || sample_rate <= 0.0f) {
        b = {1.0f, 0.0f, 0.0f};
        a = {1.0f, 0.0f, 0.0f};
        return;
    }

    float w0 = 2.0f * PI * fc / sample_rate;
    float alpha = std::sin(w0) / (2.0f * std::max(q, MIN_Q));
    float A = std::sqrt(db_to_linear(gain_db));
    a[0] = 1.0f; // Normalized

    if (type == "lowpass") {
        b[0] = (1.0f - std::cos(w0)) / 2.0f;
        b[1] = 1.0f - std::cos(w0);
        b[2] = b[0];
        a[1] = -2.0f * std::cos(w0);
        a[2] = 1.0f - alpha;
    } else if (type == "highpass") {
        b[0] = (1.0f + std::cos(w0)) / 2.0f;
        b[1] = -(1.0f + std::cos(w0));
        b[2] = b[0];
        a[1] = -2.0f * std::cos(w0);
        a[2] = 1.0f - alpha;
    } else if (type == "peaking") {
        b[0] = 1.0f + alpha * A;
        b[1] = -2.0f * std::cos(w0);
        b[2] = 1.0f - alpha * A;
        a[1] = -2.0f * std::cos(w0);
        a[2] = 1.0f - alpha / A;
    } else if (type == "lowshelf") {
        b[0] = A * ((A + 1.0f) - (A - 1.0f) * std::cos(w0) + 2.0f * std::sqrt(A) * alpha);
        b[1] = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * std::cos(w0));
        b[2] = A * ((A + 1.0f) - (A - 1.0f) * std::cos(w0) - 2.0f * std::sqrt(A) * alpha);
        a[1] = -2.0f * ((A - 1.0f) + (A + 1.0f) * std::cos(w0));
        a[2] = (A + 1.0f) + (A - 1.0f) * std::cos(w0) - 2.0f * std::sqrt(A) * alpha;
    } else if (type == "highshelf") {
        b[0] = A * ((A + 1.0f) + (A - 1.0f) * std::cos(w0) + 2.0f * std::sqrt(A) * alpha);
        b[1] = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * std::cos(w0));
        b[2] = A * ((A + 1.0f) + (A - 1.0f) * std::cos(w0) - 2.0f * std::sqrt(A) * alpha);
        a[1] = 2.0f * ((A - 1.0f) - (A + 1.0f) * std::cos(w0));
        a[2] = (A + 1.0f) - (A - 1.0f) * std::cos(w0) - 2.0f * std::sqrt(A) * alpha;
    } else {
        b = {1.0f, 0.0f, 0.0f};
        a = {1.0f, 0.0f, 0.0f};
    }
}

void generate_crossover_coeffs(std::string_view type, int order, float fc, float sample_rate,
                              std::vector<std::array<float, 3>>& b, std::vector<std::array<float, 3>>& a,
                              bool is_highpass) noexcept {
    b.clear();
    a.clear();
    if (order < 1 || order > 4 || fc < MIN_FREQ || fc > MAX_FREQ || sample_rate <= 0.0f) return;

    float w0 = 2.0f * PI * fc / sample_rate;
    float q = type == "bessel" ? 0.577f : 0.707f;
    bool linkwitz = type == "linkwitz";

    int stages = (order + 1) / 2;
    for (int i = 0; i < stages; ++i) {
        std::array<float, 3> b_stage, a_stage;
        float stage_q = q;
        if (type == "bessel") {
            stage_q = 0.577f + 0.1f * i;
        } else if (linkwitz && order % 2 == 0) {
            stage_q = 0.5f;
        }
        generate_biquad_coeffs(is_highpass ? "highpass" : "lowpass", fc, stage_q, 0.0f, sample_rate, b_stage, a_stage);
        b.push_back(b_stage);
        a.push_back(a_stage);
    }

    if (order % 2 == 1) {
        std::array<float, 3> b_stage = {1.0f, 0.0f, 0.0f};
        std::array<float, 3> a_stage = {1.0f, 0.0f, 0.0f};
        float alpha = std::tan(w0 / 2.0f);
        if (is_highpass) {
            b_stage[0] = 1.0f / (1.0f + alpha);
            b_stage[1] = -b_stage[0];
            a_stage[1] = (1.0f - alpha) / (1.0f + alpha);
        } else {
            b_stage[0] = alpha / (1.0f + alpha);
            b_stage[1] = b_stage[0];
            a_stage[1] = (1.0f - alpha) / (1.0f + alpha);
        }
        b.push_back(b_stage);
        a.push_back(a_stage);
    }

    if (linkwitz && order >= 2) {
        auto b_temp = b;
        auto a_temp = a;
        b.clear();
        a.clear();
        b.insert(b.end(), b_temp.begin(), b_temp.end());
        a.insert(a.end(), a_temp.begin(), a_temp.end());
        if (order == 2 || order == 4) {
            b.insert(b.end(), b_temp.begin(), b_temp.end());
            a.insert(a.end(), a_temp.begin(), a_temp.end());
        }
    }
}

// --- DSPNode ---
DSPNode::DSPNode(std::string_view n) : name_(n) {}

// --- GainDSP ---
GainDSP::GainDSP(float g, std::string_view n) : DSPNode(n) {
    gain_.current = gain_.target = g;
}

void GainDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float g = gain_.current;
    for (float& sample : buffer) {
        sample *= g;
    }
}

void GainDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: gain <value> [ramp_ms]\n");
        return;
    }
    float gain, ramp_ms = 0.0f;
    if (std::sscanf(args, "%f %f", &gain, &ramp_ms) >= 1 && gain >= 0.0f) {
        if (ramp_ms > 0.0f) {
            gain_.start(gain, ramp_ms, 48000.0f);
        } else {
            gain_.current = gain_.target = gain;
        }
        if (uart_ops) {
            uart_ops->puts("Gain set to ");
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.2f", gain);
            uart_ops->puts(buf);
            uart_ops->puts(" (ramp ");
            std::snprintf(buf, sizeof(buf), "%.2f", ramp_ms);
            uart_ops->puts(buf);
            uart_ops->puts(" ms)\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid gain: format 'gain <value> [ramp_ms]'\n");
    }
}

void GainDSP::ramp_params() {
    gain_.next();
}

// --- MixerDSP ---
MixerDSP::MixerDSP(std::string_view n, uint8_t inputs) : DSPNode(n), input_count_(inputs), gains_(inputs) {
    for (auto& g : gains_) {
        g.current = g.target = 1.0f;
    }
}

void MixerDSP::process(std::span<float> buffer) {
    if (buffer.size() % input_count_ != 0 || buffer.empty()) return;
    ramp_params();
    size_t samples_per_channel = buffer.size() / input_count_;
    std::vector<float> output(samples_per_channel, 0.0f);
    for (size_t ch = 0; ch < input_count_; ++ch) {
        float gain = gains_[ch].current;
        for (size_t i = 0; i < samples_per_channel; ++i) {
            output[i] += buffer[ch * samples_per_channel + i] * gain;
        }
    }
    util::memcpy(buffer.data(), output.data(), samples_per_channel * sizeof(float));
}

void MixerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: mixer <gain1> <gain2> ... [ramp_ms]\n");
        return;
    }
    std::vector<float> new_gains(input_count_);
    float ramp_ms = 0.0f;
    char* p = const_cast<char*>(args);
    size_t count = 0;
    while (count < input_count_ && std::sscanf(p, "%f", &new_gains[count]) == 1) {
        while (*p && (*p == ' ' || (*p >= '0' && *p <= '9') || *p == '.' || *p == '-')) ++p;
        ++count;
    }
    if (*p && std::sscanf(p, "%f", &ramp_ms) == 1) {
        // Ramp time provided
    }
    if (count == input_count_) {
        for (size_t i = 0; i < input_count_; ++i) {
            if (ramp_ms > 0.0f) {
                gains_[i].start(new_gains[i], ramp_ms, 48000.0f);
            } else {
                gains_[i].current = gains_[i].target = new_gains[i];
            }
        }
        if (uart_ops) {
            uart_ops->puts("Mixer gains set for ");
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%u", input_count_);
            uart_ops->puts(buf);
            uart_ops->puts(" inputs\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid mixer gains: expected ");
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%u", input_count_);
        uart_ops->puts(buf);
        uart_ops->puts(" values\n");
    }
}

void MixerDSP::ramp_params() {
    for (auto& g : gains_) {
        g.next();
    }
}

void MixerDSP::reset() {
    for (auto& g : gains_) {
        g.current = g.target = 1.0f;
    }
}

// --- ParametricEQDSP ---
ParametricEQDSP::ParametricEQDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void ParametricEQDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    for (float& sample : buffer) {
        float x = sample;
        for (size_t i = 0; i < band_count_; ++i) {
            auto& band = bands_[i];
            float y = band.b[0] * x + band.b[1] * band.z[0] + band.b[2] * band.z[1]
                    - band.a[1] * band.z[0] - band.a[2] * band.z[1];
            band.z[1] = band.z[0];
            band.z[0] = y;
            x = y;
        }
        sample = x;
    }
}

void ParametricEQDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: band <idx> <peak|lowshelf|highshelf> <freq> <q> <gain_db> [ramp_ms]\n");
        return;
    }
    char type[16];
    float freq, q, gain_db, ramp_ms = 0.0f;
    int band_idx;
    if (std::sscanf(args, "band %d %15s %f %f %f %f", &band_idx, type, &freq, &q, &gain_db, &ramp_ms) >= 5 &&
        band_idx >= 0 && static_cast<size_t>(band_idx) < MAX_BANDS && freq >= MIN_FREQ && freq <= MAX_FREQ &&
        q >= MIN_Q && q <= MAX_Q && gain_db >= MIN_GAIN_DB && gain_db <= MAX_GAIN_DB) {
        if (band_idx >= static_cast<int>(band_count_)) {
            band_count_ = band_idx + 1;
        }
        auto& band = bands_[band_idx];
        if (util::strcmp(type, "peak") == 0) band.type = BandType::PEAK;
        else if (util::strcmp(type, "lowshelf") == 0) band.type = BandType::LOWSHELF;
        else if (util::strcmp(type, "highshelf") == 0) band.type = BandType::HIGHSHELF;
        else {
            if (uart_ops) uart_ops->puts("Invalid band type: peak, lowshelf, highshelf\n");
            return;
        }
        if (ramp_ms > 0.0f) {
            band.freq.start(freq, ramp_ms, sample_rate_);
            band.q.start(q, ramp_ms, sample_rate_);
            band.gain_db.start(gain_db, ramp_ms, sample_rate_);
        } else {
            band.freq.current = band.freq.target = freq;
            band.q.current = band.q.target = q;
            band.gain_db.current = band.gain_db.target = gain_db;
        }
        update_band_coeffs(band_idx);
        if (uart_ops) {
            uart_ops->puts("EQ band ");
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%d", band_idx);
            uart_ops->puts(buf);
            uart_ops->puts(" set: type=");
            uart_ops->puts(type);
            uart_ops->puts(", freq=");
            std::snprintf(buf, sizeof(buf), "%.2f", freq);
            uart_ops->puts(buf);
            uart_ops->puts(" Hz\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid EQ: format 'band <idx> <type> <freq> <q> <gain_db> [ramp_ms]'\n");
    }
}

void ParametricEQDSP::ramp_params() {
    for (size_t i = 0; i < band_count_; ++i) {
        auto& band = bands_[i];
        bool updated = band.freq.active() || band.q.active() || band.gain_db.active();
        band.freq.next();
        band.q.next();
        band.gain_db.next();
        if (updated) {
            update_band_coeffs(i);
        }
    }
}

void ParametricEQDSP::reset() {
    band_count_ = 0;
    for (auto& band : bands_) {
        band.freq.current = band.freq.target = 1000.0f;
        band.q.current = band.q.target = 1.0f;
        band.gain_db.current = band.gain_db.target = 0.0f;
        band.b = {1.0f, 0.0f, 0.0f};
        band.a = {1.0f, 0.0f, 0.0f};
        band.z = {0.0f, 0.0f};
    }
}

void ParametricEQDSP::update_band_coeffs(size_t band_idx) {
    auto& band = bands_[band_idx];
    const char* type_str = band.type == BandType::PEAK ? "peaking" :
                           band.type == BandType::LOWSHELF ? "lowshelf" : "highshelf";
    generate_biquad_coeffs(type_str, band.freq.current, band.q.current, band.gain_db.current, sample_rate_, band.b, band.a);
}

// --- LimiterDSP ---
LimiterDSP::LimiterDSP(std::string_view n) : DSPNode(n) {
    threshold_.current = threshold_.target = 0.98f;
}

void LimiterDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float thresh = threshold_.current;
    for (float& sample : buffer) {
        sample = clip(sample, -thresh, thresh);
    }
}

void LimiterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: threshold <value> [ramp_ms]\n");
        return;
    }
    float thresh, ramp_ms = 0.0f;
    if (std::sscanf(args, "threshold %f %f", &thresh, &ramp_ms) >= 1 && thresh > 0.0f && thresh <= 1.0f) {
        if (ramp_ms > 0.0f) {
            threshold_.start(thresh, ramp_ms, 48000.0f);
        } else {
            threshold_.current = threshold_.target = thresh;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Limiter threshold set to ");
            std::snprintf(buf, sizeof(buf), "%.2f", thresh);
            uart_ops->puts(buf);
            uart_ops->puts("\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid limiter: format 'threshold <value> [ramp_ms]'\n");
    }
}

void LimiterDSP::ramp_params() {
    threshold_.next();
}

// --- NoiseGateDSP ---
NoiseGateDSP::NoiseGateDSP(std::string_view n) : DSPNode(n) {
    threshold_.current = threshold_.target = -60.0f;
    attack_ms_.current = attack_ms_.target = 10.0f;
    release_ms_.current = release_ms_.target = 100.0f;
}

void NoiseGateDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float thresh_lin = db_to_linear(threshold_.current);
    float attack_coeff = std::exp(-1.0f / (attack_ms_.current * sample_rate_ / 1000.0f));
    float release_coeff = std::exp(-1.0f / (release_ms_.current * sample_rate_ / 1000.0f));

    for (float& sample : buffer) {
        float level = std::abs(sample);
        envelope_ = envelope_ * (level > envelope_ ? attack_coeff : release_coeff) +
                   level * (1.0f - (level > envelope_ ? attack_coeff : release_coeff));
        float gain = envelope_ > thresh_lin ? 1.0f : 0.0f;
        sample *= gain;
    }
}

void NoiseGateDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: gate <threshold_db> <attack_ms> <release_ms> [ramp_ms]\n");
        return;
    }
    float thresh, attack, release, ramp_ms = 0.0f;
    if (std::sscanf(args, "gate %f %f %f %f", &thresh, &attack, &release, &ramp_ms) >= 3 &&
        thresh <= 0.0f && attack > 0.0f && release > 0.0f) {
        if (ramp_ms > 0.0f) {
            threshold_.start(thresh, ramp_ms, sample_rate_);
            attack_ms_.start(attack, ramp_ms, sample_rate_);
            release_ms_.start(release, ramp_ms, sample_rate_);
        } else {
            threshold_.current = threshold_.target = thresh;
            attack_ms_.current = attack_ms_.target = attack;
            release_ms_.current = release_ms_.target = release;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Noise gate set: threshold=");
            std::snprintf(buf, sizeof(buf), "%.2f", thresh);
            uart_ops->puts(buf);
            uart_ops->puts(" dB\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid noise gate: format 'gate <threshold_db> <attack_ms> <release_ms> [ramp_ms]'\n");
    }
}

void NoiseGateDSP::reset() {
    envelope_ = 0.0f;
}

// --- StereoWidthDSP ---
StereoWidthDSP::StereoWidthDSP(std::string_view n) : DSPNode(n) {
    width_.current = width_.target = 1.0f;
    balance_.current = balance_.target = 0.0f;
}

void StereoWidthDSP::process(std::span<float> buffer) {
    if (buffer.size() % 2 != 0 || buffer.empty()) return; // Stereo only
    ramp_params();
    float w = width_.current;
    float b = balance_.current;
    float mid_gain = 1.0f;
    float side_gain = w * 0.5f;
    float left_gain = 1.0f - b * 0.5f;
    float right_gain = 1.0f + b * 0.5f;

    for (size_t i = 0; i < buffer.size(); i += 2) {
        float left = buffer[i];
        float right = buffer[i + 1];
        float mid = (left + right) * 0.5f * mid_gain;
        float side = (left - right) * side_gain;
        buffer[i] = (mid + side) * left_gain;
        buffer[i + 1] = (mid - side) * right_gain;
    }
}

void StereoWidthDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: stereo <width> <balance> [ramp_ms]\n");
        return;
    }
    float width, balance, ramp_ms = 0.0f;
    if (std::sscanf(args, "stereo %f %f %f", &width, &balance, &ramp_ms) >= 2 &&
        width >= 0.0f && width <= 2.0f && balance >= -1.0f && balance <= 1.0f) {
        if (ramp_ms > 0.0f) {
            width_.start(width, ramp_ms, 48000.0f);
            balance_.start(balance, ramp_ms, 48000.0f);
        } else {
            width_.current = width_.target = width;
            balance_.current = balance_.target = balance;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Stereo width=");
            std::snprintf(buf, sizeof(buf), "%.2f", width);
            uart_ops->puts(buf);
            uart_ops->puts(", balance=");
            std::snprintf(buf, sizeof(buf), "%.2f", balance);
            uart_ops->puts(buf);
            uart_ops->puts("\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid stereo: format 'stereo <width> <balance> [ramp_ms]'\n");
    }
}

void StereoWidthDSP::ramp_params() {
    width_.next();
    balance_.next();
}

// --- WaveshaperDSP ---
WaveshaperDSP::WaveshaperDSP(std::string_view n) : DSPNode(n) {
    drive_.current = drive_.target = 1.0f;
    bias_.current = bias_.target = 0.0f;
}

void WaveshaperDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float drive = drive_.current;
    float bias = bias_.current;

    for (float& sample : buffer) {
        float x = sample * drive + bias;
        switch (shape_) {
            case ShapeType::TANH:
                x = std::tanh(x);
                break;
            case ShapeType::HARD_CLIP:
                x = clip(x, -1.0f, 1.0f);
                break;
            case ShapeType::SOFT_CLIP:
                x = x / (1.0f + std::abs(x));
                break;
        }
        sample = x;
    }
}

void WaveshaperDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: shape <tanh|hard|soft> <drive> <bias> [ramp_ms]\n");
        return;
    }
    char shape[16];
    float drive, bias, ramp_ms = 0.0f;
    if (std::sscanf(args, "shape %15s %f %f %f", shape, &drive, &bias, &ramp_ms) >= 3 &&
        drive >= 0.0f && std::abs(bias) <= 1.0f) {
        if (util::strcmp(shape, "tanh") == 0) shape_ = ShapeType::TANH;
        else if (util::strcmp(shape, "hard") == 0) shape_ = ShapeType::HARD_CLIP;
        else if (util::strcmp(shape, "soft") == 0) shape_ = ShapeType::SOFT_CLIP;
        else {
            if (uart_ops) uart_ops->puts("Invalid shape: tanh, hard, soft\n");
            return;
        }
        if (ramp_ms > 0.0f) {
            drive_.start(drive, ramp_ms, 48000.0f);
            bias_.start(bias, ramp_ms, 48000.0f);
        } else {
            drive_.current = drive_.target = drive;
            bias_.current = bias_.target = bias;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Waveshaper set: shape=");
            uart_ops->puts(shape);
            uart_ops->puts(", drive=");
            std::snprintf(buf, sizeof(buf), "%.2f", drive);
            uart_ops->puts(buf);
            uart_ops->puts("\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid waveshaper: format 'shape <type> <drive> <bias> [ramp_ms]'\n");
    }
}

void WaveshaperDSP::ramp_params() {
    drive_.next();
    bias_.next();
}

// --- EnvelopeFollowerDSP ---
EnvelopeFollowerDSP::EnvelopeFollowerDSP(std::string_view n) : DSPNode(n) {
    attack_ms_.current = attack_ms_.target = 5.0f;
    release_ms_.current = release_ms_.target = 50.0f;
}

void EnvelopeFollowerDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float attack_coeff = std::exp(-1.0f / (attack_ms_.current * sample_rate_ / 1000.0f));
    float release_coeff = std::exp(-1.0f / (release_ms_.current * sample_rate_ / 1000.0f));

    for (float sample : buffer) {
        float level = use_rms_ ? sample * sample : std::abs(sample);
        envelope_ = envelope_ * (level > envelope_ ? attack_coeff : release_coeff) +
                   level * (1.0f - (level > envelope_ ? attack_coeff : release_coeff));
        if (use_rms_) envelope_ = std::sqrt(envelope_);
    }
}

void EnvelopeFollowerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: env <rms|peak> <attack_ms> <release_ms> [ramp_ms]\n");
        return;
    }
    char mode[8];
    float attack, release, ramp_ms = 0.0f;
    if (std::sscanf(args, "env %7s %f %f %f", mode, &attack, &release, &ramp_ms) >= 3 &&
        attack > 0.0f && release > 0.0f) {
        use_rms_ = util::strcmp(mode, "rms") == 0;
        if (!use_rms_ && util::strcmp(mode, "peak") != 0) {
            if (uart_ops) uart_ops->puts("Invalid mode: rms, peak\n");
            return;
        }
        if (ramp_ms > 0.0f) {
            attack_ms_.start(attack, ramp_ms, sample_rate_);
            release_ms_.start(release, ramp_ms, sample_rate_);
        } else {
            attack_ms_.current = attack_ms_.target = attack;
            release_ms_.current = release_ms_.target = release;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Envelope follower set: mode=");
            uart_ops->puts(mode);
            uart_ops->puts(", attack=");
            std::snprintf(buf, sizeof(buf), "%.2f", attack);
            uart_ops->puts(buf);
            uart_ops->puts(" ms\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid envelope: format 'env <rms|peak> <attack_ms> <release_ms> [ramp_ms]'\n");
    }
}

void EnvelopeFollowerDSP::reset() {
    envelope_ = 0.0f;
}

// --- CompressorDSP ---
CompressorDSP::CompressorDSP(std::string_view n) : DSPNode(n) {
    threshold_db_.current = threshold_db_.target = -12.0f;
    ratio_.current = ratio_.target = 2.0f;
    attack_ms_.current = attack_ms_.target = 10.0f;
    release_ms_.current = release_ms_.target = 100.0f;
    makeup_db_.current = makeup_db_.target = 0.0f;
}

void CompressorDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float thresh = db_to_linear(threshold_db_.current);
    float ratio = ratio_.current;
    float attack_coeff = std::exp(-1.0f / (attack_ms_.current * sample_rate_ / 1000.0f));
    float release_coeff = std::exp(-1.0f / (release_ms_.current * sample_rate_ / 1000.0f));
    float makeup = db_to_linear(makeup_db_.current);

    for (float& sample : buffer) {
        float level = std::abs(sample);
        envelope_ = envelope_ * (level > envelope_ ? attack_coeff : release_coeff) +
                   level * (1.0f - (level > envelope_ ? attack_coeff : release_coeff));
        float gain = 1.0f;
        if (envelope_ > thresh) {
            float excess_db = linear_to_db(envelope_ / thresh);
            gain = db_to_linear(-excess_db * (1.0f - 1.0f / ratio));
        }
        sample *= gain * makeup;
    }
}

void CompressorDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: compressor <threshold_db> <ratio> <attack_ms> <release_ms> <makeup_db> [ramp_ms]\n");
        return;
    }
    float thresh, ratio, attack, release, makeup, ramp_ms = 0.0f;
    if (std::sscanf(args, "compressor %f %f %f %f %f %f", &thresh, &ratio, &attack, &release, &makeup, &ramp_ms) >= 5 &&
        thresh <= 0.0f && ratio >= 1.0f && attack > 0.0f && release > 0.0f) {
        if (ramp_ms > 0.0f) {
            threshold_db_.start(thresh, ramp_ms, sample_rate_);
            ratio_.start(ratio, ramp_ms, sample_rate_);
            attack_ms_.start(attack, ramp_ms, sample_rate_);
            release_ms_.start(release, ramp_ms, sample_rate_);
            makeup_db_.start(makeup, ramp_ms, sample_rate_);
        } else {
            threshold_db_.current = threshold_db_.target = thresh;
            ratio_.current = ratio_.target = ratio;
            attack_ms_.current = attack_ms_.target = attack;
            release_ms_.current = release_ms_.target = release;
            makeup_db_.current = makeup_db_.target = makeup;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Compressor set: threshold=");
            std::snprintf(buf, sizeof(buf), "%.2f", thresh);
            uart_ops->puts(buf);
            uart_ops->puts(" dB\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid compressor: format 'compressor <threshold_db> <ratio> <attack_ms> <release_ms> <makeup_db> [ramp_ms]'\n");
    }
}

void CompressorDSP::ramp_params() {
    threshold_db_.next();
    ratio_.next();
    attack_ms_.next();
    release_ms_.next();
    makeup_db_.next();
}

void CompressorDSP::reset() {
    envelope_ = 0.0f;
}

// --- DelayDSP ---
DelayDSP::DelayDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void DelayDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    for (float& sample : buffer) {
        buffer_[index_] = sample;
        sample = buffer_[(index_ - delay_samples_) % MAX_DELAY_SAMPLES];
        index_ = (index_ + 1) % MAX_DELAY_SAMPLES;
    }
}

void DelayDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: delay <ms>\n");
        return;
    }
    float delay_ms;
    if (std::sscanf(args, "delay %f", &delay_ms) == 1 && delay_ms >= 0.0f) {
        delay_samples_ = static_cast<size_t>(delay_ms * 48000.0f / 1000.0f);
        if (delay_samples_ > MAX_DELAY_SAMPLES) delay_samples_ = MAX_DELAY_SAMPLES;
        reset();
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Delay set to ");
            std::snprintf(buf, sizeof(buf), "%.2f", delay_ms);
            uart_ops->puts(buf);
            uart_ops->puts(" ms\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid delay: format 'delay <ms>'\n");
    }
}

void DelayDSP::reset() {
    index_ = 0;
    util::memset(buffer_.data(), 0, buffer_.size() * sizeof(float));
}

// --- ReverbDSP ---
ReverbDSP::ReverbDSP(std::string_view n) : DSPNode(n) {
    pre_delay_ms_.current = pre_delay_ms_.target = 0.0f;
    room_size_.current = room_size_.target = 0.5f;
    damping_.current = damping_.target = 0.5f;
    wet_dry_.current = wet_dry_.target = 0.3f;
    for (auto& buf : comb_buffers_) {
        buf.resize(2048);
    }
    for (auto& buf : allpass_buffers_) {
        buf.resize(512);
    }
    pre_delay_buffer_.resize(4800); // 100ms at 48kHz
    reset();
}

void ReverbDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    static const size_t comb_delays[NUM_COMBS] = {1116, 1188, 1277, 1355, 1422, 1491, 1557, 1617};
    static const size_t allpass_delays[NUM_ALLPASS] = {556, 441, 341, 225};
    float comb_gain = 0.8f * room_size_.current;
    float allpass_gain = 0.7f;
    float wet = wet_dry_.current;
    float dry = 1.0f - wet;

    for (size_t i = 0; i < buffer.size(); ++i) {
        float input = buffer[i];
        pre_delay_buffer_[pre_delay_index_] = input;
        float delayed = pre_delay_buffer_[(pre_delay_index_ - pre_delay_samples_) % pre_delay_buffer_.size()];
        pre_delay_index_ = (pre_delay_index_ + 1) % pre_delay_buffer_.size();

        float comb_sum = 0.0f;
        for (size_t j = 0; j < NUM_COMBS; ++j) {
            size_t idx = (comb_indices_[j] - comb_delays[j]) % comb_buffers_[j].size();
            float comb_out = comb_buffers_[j][idx];
            comb_sum += comb_out;
            comb_buffers_[j][comb_indices_[j]] = delayed + comb_out * comb_gain * (1.0f - damping_.current);
            comb_indices_[j] = (comb_indices_[j] + 1) % comb_buffers_[j].size();
        }

        float allpass_out = comb_sum / NUM_COMBS;
        for (size_t j = 0; j < NUM_ALLPASS; ++j) {
            size_t idx = (allpass_indices_[j] - allpass_delays[j]) % allpass_buffers_[j].size();
            float temp = allpass_buffers_[j][idx];
            allpass_buffers_[j][allpass_indices_[j]] = allpass_out + temp * allpass_gain;
            allpass_out = temp - allpass_out * allpass_gain;
            allpass_indices_[j] = (allpass_indices_[j] + 1) % allpass_buffers_[j].size();
        }

        buffer[i] = input * dry + allpass_out * wet;
    }
}

void ReverbDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: reverb <pre_delay_ms> <room_size> <damping> <wet_dry> [ramp_ms]\n");
        return;
    }
    float pre_delay, room, damping, wet_dry, ramp_ms = 0.0f;
    if (std::sscanf(args, "reverb %f %f %f %f %f", &pre_delay, &room, &damping, &wet_dry, &ramp_ms) >= 4 &&
        pre_delay >= 0.0f && room >= 0.0f && room <= 1.0f && damping >= 0.0f && damping <= 1.0f &&
        wet_dry >= 0.0f && wet_dry <= 1.0f) {
        if (ramp_ms > 0.0f) {
            pre_delay_ms_.start(pre_delay, ramp_ms, sample_rate_);
            room_size_.start(room, ramp_ms, sample_rate_);
            damping_.start(damping, ramp_ms, sample_rate_);
            wet_dry_.start(wet_dry, ramp_ms, sample_rate_);
        } else {
            pre_delay_ms_.current = pre_delay_ms_.target = pre_delay;
            room_size_.current = room_size_.target = room;
            damping_.current = damping_.target = damping;
            wet_dry_.current = wet_dry_.target = wet_dry;
        }
        pre_delay_samples_ = static_cast<size_t>(pre_delay * sample_rate_ / 1000.0f);
        if (pre_delay_samples_ > pre_delay_buffer_.size()) pre_delay_samples_ = pre_delay_buffer_.size();
        reset();
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Reverb set: room=");
            std::snprintf(buf, sizeof(buf), "%.2f", room);
            uart_ops->puts(buf);
            uart_ops->puts(", pre_delay=");
            std::snprintf(buf, sizeof(buf), "%.2f", pre_delay);
            uart_ops->puts(buf);
            uart_ops->puts(" ms\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid reverb: format 'reverb <pre_delay_ms> <room_size> <damping> <wet_dry> [ramp_ms]'\n");
    }
}

void ReverbDSP::reset() {
    for (auto& buf : comb_buffers_) {
        util::memset(buf.data(), 0, buf.size() * sizeof(float));
    }
    for (auto& buf : allpass_buffers_) {
        util::memset(buf.data(), 0, buf.size() * sizeof(float));
    }
    util::memset(comb_indices_.data(), 0, comb_indices_.size() * sizeof(size_t));
    util::memset(allpass_indices_.data(), 0, allpass_indices_.size() * sizeof(size_t));
    util::memset(pre_delay_buffer_.data(), 0, pre_delay_buffer_.size() * sizeof(float));
    pre_delay_index_ = 0;
}

// --- ChorusDSP ---
ChorusDSP::ChorusDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void ChorusDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float depth_samples = depth_ms_.current * sample_rate_ / 1000.0f;
    float rate_rad = 2.0f * PI * rate_hz_.current / sample_rate_;
    float wet = mix_.current;
    float dry = 1.0f - wet;

    for (size_t i = 0; i < buffer.size(); ++i) {
        buffer_[index_] = buffer[i];
        float mod = depth_samples * 0.5f * (1.0f + std::sin(phase_));
        size_t delay_idx = (index_ + MAX_DELAY_SAMPLES - static_cast<size_t>(mod)) % MAX_DELAY_SAMPLES;
        float delayed = buffer_[delay_idx];
        buffer[i] = buffer[i] * dry + delayed * wet;
        index_ = (index_ + 1) % MAX_DELAY_SAMPLES;
        phase_ += rate_rad;
        if (phase_ > 2.0f * PI) phase_ -= 2.0f * PI;
    }
}

void ChorusDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: chorus <depth_ms> <rate_hz> <mix> [ramp_ms]\n");
        return;
    }
    float depth, rate, mix, ramp_ms = 0.0f;
    if (std::sscanf(args, "chorus %f %f %f %f", &depth, &rate, &mix, &ramp_ms) >= 3 &&
        depth >= 0.0f && depth <= 50.0f && rate >= 0.0f && rate <= 10.0f && mix >= 0.0f && mix <= 1.0f) {
        if (ramp_ms > 0.0f) {
            depth_ms_.start(depth, ramp_ms, sample_rate_);
            rate_hz_.start(rate, ramp_ms, sample_rate_);
            mix_.start(mix, ramp_ms, sample_rate_);
        } else {
            depth_ms_.current = depth_ms_.target = depth;
            rate_hz_.current = rate_hz_.target = rate;
            mix_.current = mix_.target = mix;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Chorus set: depth=");
            std::snprintf(buf, sizeof(buf), "%.2f", depth);
            uart_ops->puts(buf);
            uart_ops->puts(" ms\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid chorus: format 'chorus <depth_ms> <rate_hz> <mix> [ramp_ms]'\n");
    }
}

void ChorusDSP::reset() {
    util::memset(buffer_.data(), 0, buffer_.size() * sizeof(float));
    index_ = 0;
    phase_ = 0.0f;
}

// --- FlangerDSP ---
FlangerDSP::FlangerDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void FlangerDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float depth_samples = depth_ms_.current * sample_rate_ / 1000.0f;
    float rate_rad = 2.0f * PI * rate_hz_.current / sample_rate_;
    float fb = feedback_.current;
    float wet = mix_.current;
    float dry = 1.0f - wet;

    for (size_t i = 0; i < buffer.size(); ++i) {
        float mod = depth_samples * 0.5f * (1.0f + std::sin(phase_));
        size_t delay_idx = (index_ + MAX_DELAY_SAMPLES - static_cast<size_t>(mod)) % MAX_DELAY_SAMPLES;
        float delayed = buffer_[delay_idx];
        buffer_[index_] = buffer[i] + delayed * fb;
        buffer[i] = buffer[i] * dry + delayed * wet;
        index_ = (index_ + 1) % MAX_DELAY_SAMPLES;
        phase_ += rate_rad;
        if (phase_ > 2.0f * PI) phase_ -= 2.0f * PI;
    }
}

void FlangerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: flanger <depth_ms> <rate_hz> <feedback> <mix> [ramp_ms]\n");
        return;
    }
    float depth, rate, feedback, mix, ramp_ms = 0.0f;
    if (std::sscanf(args, "flanger %f %f %f %f %f", &depth, &rate, &feedback, &mix, &ramp_ms) >= 4 &&
        depth >= 0.0f && depth <= 10.0f && rate >= 0.0f && rate <= 10.0f &&
        feedback >= -1.0f && feedback <= 1.0f && mix >= 0.0f && mix <= 1.0f) {
        if (ramp_ms > 0.0f) {
            depth_ms_.start(depth, ramp_ms, sample_rate_);
            rate_hz_.start(rate, ramp_ms, sample_rate_);
            feedback_.start(feedback, ramp_ms, sample_rate_);
            mix_.start(mix, ramp_ms, sample_rate_);
        } else {
            depth_ms_.current = depth_ms_.target = depth;
            rate_hz_.current = rate_hz_.target = rate;
            feedback_.current = feedback_.target = feedback;
            mix_.current = mix_.target = mix;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Flanger set: depth=");
            std::snprintf(buf, sizeof(buf), "%.2f", depth);
            uart_ops->puts(buf);
            uart_ops->puts(" ms\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid flanger: format 'flanger <depth_ms> <rate_hz> <feedback> <mix> [ramp_ms]'\n");
    }
}

void FlangerDSP::reset() {
    util::memset(buffer_.data(), 0, buffer_.size() * sizeof(float));
    index_ = 0;
    phase_ = 0.0f;
}

// --- PitchShifterDSP ---
PitchShifterDSP::PitchShifterDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void PitchShifterDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float pitch = pitch_factor_.current;
    std::vector<float> output(buffer.size());
    size_t out_pos = 0;

    for (size_t i = 0; i < buffer.size(); ++i) {
        input_buffer_[input_index_] = buffer[i];
        input_index_ = (input_index_ + 1) % input_buffer_.size();

        phase_ += 1.0f / pitch;
        while (phase_ >= 1.0f && out_pos < output.size()) {
            size_t idx = static_cast<size_t>(phase_ * WINDOW_SIZE);
            float frac = phase_ * WINDOW_SIZE - idx;
            size_t idx0 = (input_index_ + input_buffer_.size() - idx - 1) % input_buffer_.size();
            size_t idx1 = (idx0 + 1) % input_buffer_.size();
            output[out_pos++] = input_buffer_[idx0] * (1.0f - frac) + input_buffer_[idx1] * frac;
            phase_ -= 1.0f;
        }
    }

    util::memcpy(buffer.data(), output.data(), std::min(buffer.size(), out_pos) * sizeof(float));
}

void PitchShifterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: pitch <factor> [ramp_ms]\n");
        return;
    }
    float pitch, ramp_ms = 0.0f;
    if (std::sscanf(args, "pitch %f %f", &pitch, &ramp_ms) >= 1 && pitch >= 0.5f && pitch <= 2.0f) {
        if (ramp_ms > 0.0f) {
            pitch_factor_.start(pitch, ramp_ms, sample_rate_);
        } else {
            pitch_factor_.current = pitch_factor_.target = pitch;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Pitch shifter set to ");
            std::snprintf(buf, sizeof(buf), "%.2f", pitch);
            uart_ops->puts(buf);
            uart_ops->puts("x\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid pitch: format 'pitch <factor> [ramp_ms]'\n");
    }
}

void PitchShifterDSP::reset() {
    util::memset(input_buffer_.data(), 0, input_buffer_.size() * sizeof(float));
    util::memset(output_buffer_.data(), 0, output_buffer_.size() * sizeof(float));
    input_index_ = 0;
    output_index_ = 0;
    phase_ = 0.0f;
}

// --- ConvolutionReverbDSP ---
ConvolutionReverbDSP::ConvolutionReverbDSP(std::string_view n) : DSPNode(n) {
    wet_dry_.current = wet_dry_.target = 0.3f;
    generate_impulse_response();
    reset();
}

void ConvolutionReverbDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    float wet = wet_dry_.current;
    float dry = 1.0f - wet;

    for (size_t i = 0; i < buffer.size(); ++i) {
        history_[history_index_] = buffer[i];
        float sum = 0.0f;
        for (size_t j = 0; j < IR_LENGTH; ++j) {
            size_t idx = (history_index_ + IR_LENGTH - j) % IR_LENGTH;
            sum += history_[idx] * impulse_response_[j];
        }
        buffer[i] = buffer[i] * dry + sum * wet;
        history_index_ = (history_index_ + 1) % IR_LENGTH;
    }
}

void ConvolutionReverbDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: conv <wet_dry> [ramp_ms]\n");
        return;
    }
    float wet_dry, ramp_ms = 0.0f;
    if (std::sscanf(args, "conv %f %f", &wet_dry, &ramp_ms) >= 1 && wet_dry >= 0.0f && wet_dry <= 1.0f) {
        if (ramp_ms > 0.0f) {
            wet_dry_.start(wet_dry, ramp_ms, sample_rate_);
        } else {
            wet_dry_.current = wet_dry_.target = wet_dry;
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Convolution reverb set: wet_dry=");
            std::snprintf(buf, sizeof(buf), "%.2f", wet_dry);
            uart_ops->puts(buf);
            uart_ops->puts("\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid conv reverb: format 'conv <wet_dry> [ramp_ms]'\n");
    }
}

void ConvolutionReverbDSP::reset() {
    util::memset(history_.data(), 0, history_.size() * sizeof(float));
    history_index_ = 0;
}

void ConvolutionReverbDSP::generate_impulse_response() {
    float decay = 0.999f;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (size_t i = 0; i < IR_LENGTH; ++i) {
        float t = i / sample_rate_;
        impulse_response_[i] = dist(gen) * std::exp(-t * 3.0f) * decay;
    }
    float max = 0.0f;
    for (float x : impulse_response_) {
        max = std::max(max, std::abs(x));
    }
    if (max > 0.0f) {
        for (float& x : impulse_response_) {
            x /= max;
        }
    }
}

// --- CrossoverDSP ---
CrossoverDSP::CrossoverDSP(std::string_view n) : DSPNode(n) {
    bands_[0].cutoff_hz.current = bands_[0].cutoff_hz.target = 200.0f; // Subwoofer
    bands_[1].cutoff_hz.current = bands_[1].cutoff_hz.target = 1000.0f; // Woofer
    bands_[2].cutoff_hz.current = bands_[2].cutoff_hz.target = 5000.0f; // Midrange
    bands_[3].cutoff_hz.current = bands_[3].cutoff_hz.target = 20000.0f; // Tweeter
    for (auto& band : bands_) {
        band.gain_db.current = band.gain_db.target = 0.0f;
        band.order = 2;
        band.type = FilterType::BUTTERWORTH;
        band.shape = FilterShape::LOWPASS;
    }
    band_count_ = 2; // Default: 2-way
    update_band_coeffs(0);
    update_band_coeffs(1);
}

void CrossoverDSP::process(std::span<float> buffer) {
    if (buffer.size() < band_count_ || buffer.empty()) return;
    ramp_params();
    std::vector<std::vector<float>> band_outputs(band_count_, std::vector<float>(buffer.size()));
    for (size_t b = 0; b < band_count_; ++b) {
        std::copy(buffer.begin(), buffer.end(), band_outputs[b].begin());
        auto& band = bands_[b];
        for (size_t i = 0; i < buffer.size(); ++i) {
            float x = band_outputs[b][i];
            for (auto& stage : band.stages) {
                float y = stage.b[0] * x + stage.b[1] * stage.z[0] + stage.b[2] * stage.z[1]
                        - stage.a[1] * stage.z[0] - stage.a[2] * stage.z[1];
                stage.z[1] = stage.z[0];
                stage.z[0] = y;
                x = y;
            }
            band_outputs[b][i] = x * db_to_linear(band.gain_db.current);
        }
    }

    size_t samples_per_band = buffer.size() / band_count_;
    for (size_t b = 0; b < band_count_; ++b) {
        util::memcpy(buffer.data() + b * samples_per_band, band_outputs[b].data(), samples_per_band * sizeof(float));
    }
}

void CrossoverDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: band <idx> <butterworth|bessel|linkwitz> <lowpass|highpass|lowshelf|highshelf> <order> <cutoff_hz> <gain_db> [ramp_ms] or bands <count>\n");
        return;
    }
    char type[16], shape[16];
    int band_idx, order;
    float cutoff, gain_db, ramp_ms = 0.0f;
    if (std::sscanf(args, "band %d %15s %15s %d %f %f %f", &band_idx, type, shape, &order, &cutoff, &gain_db, &ramp_ms) == 7 &&
        band_idx >= 0 && static_cast<size_t>(band_idx) < MAX_BANDS && order >= 1 && order <= 4 &&
        cutoff >= MIN_FREQ && cutoff <= MAX_FREQ && gain_db >= MIN_GAIN_DB && gain_db <= MAX_GAIN_DB) {
        if (band_idx >= static_cast<int>(band_count_)) {
            band_count_ = band_idx + 1;
        }
        auto& band = bands_[band_idx];
        if (util::strcmp(type, "butterworth") == 0) band.type = FilterType::BUTTERWORTH;
        else if (util::strcmp(type, "bessel") == 0) band.type = FilterType::BESSEL;
        else if (util::strcmp(type, "linkwitz") == 0) band.type = FilterType::LINKWITZ_RILEY;
        else {
            if (uart_ops) uart_ops->puts("Invalid filter type: butterworth, bessel, linkwitz\n");
            return;
        }
        if (util::strcmp(shape, "lowpass") == 0) band.shape = FilterShape::LOWPASS;
        else if (util::strcmp(shape, "highpass") == 0) band.shape = FilterShape::HIGHPASS;
        else if (util::strcmp(shape, "lowshelf") == 0) band.shape = FilterShape::LOWSHELF;
        else if (util::strcmp(shape, "highshelf") == 0) band.shape = FilterShape::HIGHSHELF;
        else {
            if (uart_ops) uart_ops->puts("Invalid shape: lowpass, highpass, lowshelf, highshelf\n");
            return;
        }
        band.order = order;
        if (ramp_ms > 0.0f) {
            band.cutoff_hz.start(cutoff, ramp_ms, sample_rate_);
            band.gain_db.start(gain_db, ramp_ms, sample_rate_);
        } else {
            band.cutoff_hz.current = band.cutoff_hz.target = cutoff;
            band.gain_db.current = band.gain_db.target = gain_db;
        }
        update_band_coeffs(band_idx);
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Crossover band ");
            std::snprintf(buf, sizeof(buf), "%d", band_idx);
            uart_ops->puts(buf);
            uart_ops->puts(" set: type=");
            uart_ops->puts(type);
            uart_ops->puts(", shape=");
            uart_ops->puts(shape);
            uart_ops->puts(", cutoff=");
            std::snprintf(buf, sizeof(buf), "%.2f", cutoff);
            uart_ops->puts(buf);
            uart_ops->puts(" Hz\n");
        }
    } else if (util::strcmp(args, "bands") == 0) {
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Crossover bands: ");
            std::snprintf(buf, sizeof(buf), "%u", band_count_);
            uart_ops->puts(buf);
            uart_ops->puts("\n");
        }
    } else if (std::sscanf(args, "bands %d", &band_idx) == 1 && band_idx >= 1 && band_idx <= static_cast<int>(MAX_BANDS)) {
        band_count_ = band_idx;
        for (size_t i = 0; i < band_count_; ++i) {
            update_band_coeffs(i);
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Crossover set to ");
            std::snprintf(buf, sizeof(buf), "%d", band_idx);
            uart_ops->puts(buf);
            uart_ops->puts("-way\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid crossover: format 'band <idx> <type> <shape> <order> <cutoff_hz> <gain_db> [ramp_ms]' or 'bands <count>'\n");
    }
}

void CrossoverDSP::ramp_params() {
    for (size_t i = 0; i < band_count_; ++i) {
        auto& band = bands_[i];
        bool updated = band.cutoff_hz.active() || band.gain_db.active();
        band.cutoff_hz.next();
        band.gain_db.next();
        if (updated) {
            update_band_coeffs(i);
        }
    }
}

void CrossoverDSP::reset() {
    for (size_t i = 0; i < band_count_; ++i) {
        auto& band = bands_[i];
        for (auto& stage : band.stages) {
            stage.z = {0.0f, 0.0f};
        }
    }
}

void CrossoverDSP::update_band_coeffs(size_t band_idx) {
    auto& band = bands_[band_idx];
    band.stages.clear();
    std::vector<std::array<float, 3>> b, a;
    const char* type_str = band.type == FilterType::BUTTERWORTH ? "butterworth" :
                           band.type == FilterType::BESSEL ? "bessel" : "linkwitz";
    bool is_highpass = band.shape == FilterShape::HIGHPASS || band.shape == FilterShape::HIGHSHELF;
    if (band.shape == FilterShape::LOWSHELF || band.shape == FilterShape::HIGHSHELF) {
        std::array<float, 3> b_stage, a_stage;
        generate_biquad_coeffs(band.shape == FilterShape::LOWSHELF ? "lowshelf" : "highshelf",
                               band.cutoff_hz.current, 0.707f, band.gain_db.current, sample_rate_, b_stage, a_stage);
        b.push_back(b_stage);
        a.push_back(a_stage);
    } else {
        generate_crossover_coeffs(type_str, band.order, band.cutoff_hz.current, sample_rate_, b, a, is_highpass);
    }

    band.stages.resize(b.size());
    for (size_t i = 0; i < b.size(); ++i) {
        band.stages[i].b = b[i];
        band.stages[i].a = a[i];
        band.stages[i].z = {0.0f, 0.0f};
    }
}

// --- PhaseCorrectionDSP ---
PhaseCorrectionDSP::PhaseCorrectionDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void PhaseCorrectionDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    for (size_t i = 0; i < buffer.size(); ++i) {
        float x = buffer[i];
        float y = a1_ * (x - buffer_[(index_ - 1) % MAX_DELAY_SAMPLES]) + buffer_[(index_ - 1) % MAX_DELAY_SAMPLES];
        buffer_[index_] = x - a1_ * y;
        buffer[i] = y;
        index_ = (index_ + 1) % MAX_DELAY_SAMPLES;
    }
}

void PhaseCorrectionDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: phase <degrees> [ramp_ms]\n");
        return;
    }
    float phase, ramp_ms = 0.0f;
    if (std::sscanf(args, "phase %f %f", &phase, &ramp_ms) >= 1 && phase >= -180.0f && phase <= 180.0f) {
        if (ramp_ms > 0.0f) {
            phase_deg_.start(phase, ramp_ms, 48000.0f);
        } else {
            phase_deg_.current = phase_deg_.target = phase;
        }
        update_coeff();
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("Phase correction set to ");
            std::snprintf(buf, sizeof(buf), "%.2f", phase);
            uart_ops->puts(buf);
            uart_ops->puts(" degrees\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid phase: format 'phase <degrees> [ramp_ms]'\n");
    }
}

void PhaseCorrectionDSP::ramp_params() {
    if (phase_deg_.active()) {
        phase_deg_.next();
        update_coeff();
    }
}

void PhaseCorrectionDSP::reset() {
    index_ = 0;
    util::memset(buffer_.data(), 0, buffer_.size() * sizeof(float));
    a1_ = 0.0f;
}

void PhaseCorrectionDSP::update_coeff() {
    float phase_rad = phase_deg_.current * PI / 180.0f;
    a1_ = std::tan(phase_rad / 2.0f);
}

// --- SRCDSP ---
SRCDSP::SRCDSP(std::string_view n, uint32_t in_rate, uint32_t out_rate) : DSPNode(n), in_rate_(in_rate), out_rate_(out_rate) {
    history_.resize(FILTER_TAPS);
    generate_filter();
    reset();
}

void SRCDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float ratio = static_cast<float>(out_rate_) / in_rate_;
    std::vector<float> output;
    output.reserve(static_cast<size_t>(buffer.size() * ratio) + 1);

    for (size_t i = 0; i < buffer.size(); ++i) {
        history_[i % FILTER_TAPS] = buffer[i];
        while (phase_ <= 1.0f) {
            float sum = 0.0f;
            for (size_t j = 0; j < FILTER_TAPS; ++j) {
                size_t idx = (i + FILTER_TAPS - j) % FILTER_TAPS;
                size_t filter_idx = static_cast<size_t>(phase_ * FILTER_TAPS + j);
                if (filter_idx < filter_coeffs_.size()) {
                    sum += history_[idx] * filter_coeffs_[filter_idx];
                }
            }
            output.push_back(sum);
            phase_ += 1.0f / ratio;
        }
        phase_ -= 1.0f;
    }

    util::memcpy(buffer.data(), output.data(), std::min(buffer.size(), output.size()) * sizeof(float));
}


void SRCDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: src <in_rate> <out_rate>\n");
        return;
    }
    uint32_t in_rate, out_rate;
    if (std::sscanf(args, "src %u %u", &in_rate, &out_rate) == 2 && in_rate > 0 && out_rate > 0) {
        in_rate_ = in_rate;
        out_rate_ = out_rate;
        generate_filter();
        reset();
        if (uart_ops) {
            char buf[32];
            uart_ops->puts("SRC set: in_rate=");
            std::snprintf(buf, sizeof(buf), "%u", in_rate);
            uart_ops->puts(buf);
            uart_ops->puts(" Hz, out_rate=");
            std::snprintf(buf, sizeof(buf), "%u", out_rate);
            uart_ops->puts(buf);
            uart_ops->puts(" Hz\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid SRC: format 'src <in_rate> <out_rate>'\n");
    }
}

void SRCDSP::reset() {
    phase_ = 0.0f;
    util::memset(history_.data(), 0, history_.size() * sizeof(float));
}

void SRCDSP::generate_filter() {
    filter_coeffs_.resize(FILTER_TAPS * 2);
    float cutoff = 0.5f * std::min(in_rate_, out_rate_);
    float w = 2.0f * PI * cutoff / sample_rate_;
    for (size_t i = 0; i < FILTER_TAPS * 2; ++i) {
        float t = (i - FILTER_TAPS) / static_cast<float>(FILTER_TAPS);
        filter_coeffs_[i] = w / PI * std::sin(w * t) / (w * t + 1e-6f);
    }
    generate_hann_window(std::span<float>(filter_coeffs_.data(), filter_coeffs_.size()));
}

// --- FIRFilterDSP ---
FIRFilterDSP::FIRFilterDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void FIRFilterDSP::process(std::span<float> buffer) {
    if (buffer.empty() || tap_count_ == 0) return;
    for (size_t i = 0; i < buffer.size(); ++i) {
        history_[history_index_] = buffer[i];
        float sum = 0.0f;
        for (size_t j = 0; j < tap_count_; ++j) {
            size_t idx = (history_index_ + MAX_TAPS - j) % MAX_TAPS;
            sum += history_[idx] * taps_[j];
        }
        buffer[i] = sum;
        history_index_ = (history_index_ + 1) % MAX_TAPS;
    }
}

void FIRFilterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: fir <tap1> <tap2> ... <tapN>\n");
        return;
    }
    std::vector<float> new_taps;
    float tap;
    char* p = const_cast<char*>(args);
    while (std::sscanf(p, "%f", &tap) == 1) {
        new_taps.push_back(tap);
        while (*p && (*p == ' ' || (*p >= '0' && *p <= '9') || *p == '.' || *p == '-')) ++p;
    }
    if (!new_taps.empty() && new_taps.size() <= MAX_TAPS) {
        tap_count_ = new_taps.size();
        std::copy(new_taps.begin(), new_taps.end(), taps_.begin());
        reset();
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("FIR filter set with ");
            std::snprintf(buf, sizeof(buf), "%u", tap_count_);
            uart_ops->puts(buf);
            uart_ops->puts(" taps\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid FIR: format 'fir <tap1> <tap2> ...' (max 64 taps)\n");
    }
}

void FIRFilterDSP::reset() {
    util::memset(history_.data(), 0, history_.size() * sizeof(float));
    history_index_ = 0;
}

// --- IIRFilterDSP ---
IIRFilterDSP::IIRFilterDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void IIRFilterDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    for (float& sample : buffer) {
        float x = sample;
        float y = b_[0] * x + b_[1] * z_[0] + b_[2] * z_[1]
                - a_[1] * z_[0] - a_[2] * z_[1];
        z_[1] = z_[0];
        z_[0] = y;
        sample = y;
    }
}

void IIRFilterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: iir <b0> <b1> <b2> <a1> <a2>\n");
        return;
    }
    float b0, b1, b2, a1, a2;
    if (std::sscanf(args, "iir %f %f %f %f %f", &b0, &b1, &b2, &a1, &a2) == 5) {
        b_ = {b0, b1, b2};
        a_ = {1.0f, a1, a2};
        reset();
        if (uart_ops) {
            uart_ops->puts("IIR filter coefficients set\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid IIR: format 'iir <b0> <b1> <b2> <a1> <a2>'\n");
    }
}

void IIRFilterDSP::reset() {
    z_ = {0.0f, 0.0f};
}

// --- FFTEqualizerDSP ---
FFTEqualizerDSP::FFTEqualizerDSP(std::string_view n) : DSPNode(n) {
    reset();
}

void FFTEqualizerDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    ramp_params();
    // Simplified: Apply gain per band (no FFT, placeholder)
    for (size_t i = 0; i < buffer.size(); ++i) {
        float gain = 1.0f;
        for (size_t j = 0; j < band_count_; ++j) {
            gain *= db_to_linear(band_gains_[j].current);
        }
        buffer[i] *= gain;
    }
}

void FFTEqualizerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: ffteq <band1_db> <band2_db> ... <bandN_db> [ramp_ms]\n");
        return;
    }
    std::vector<float> gains;
    float gain, ramp_ms = 0.0f;
    char* p = const_cast<char*>(args);
    while (std::sscanf(p, "%f", &gain) == 1 && gains.size() < MAX_BANDS) {
        gains.push_back(gain);
        while (*p && (*p == ' ' || (*p >= '0' && *p <= '9') || *p == '.' || *p == '-')) ++p;
    }
    if (*p && std::sscanf(p, "%f", &ramp_ms) == 1) {
        // Ramp time provided
    }
    if (!gains.empty()) {
        band_count_ = gains.size();
        for (size_t i = 0; i < band_count_; ++i) {
            if (ramp_ms > 0.0f) {
                band_gains_[i].start(gains[i], ramp_ms, 48000.0f);
            } else {
                band_gains_[i].current = band_gains_[i].target = gains[i];
            }
        }
        if (uart_ops) {
            char buf[16];
            uart_ops->puts("FFT equalizer set with ");
            std::snprintf(buf, sizeof(buf), "%u", band_count_);
            uart_ops->puts(buf);
            uart_ops->puts(" bands\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid FFT EQ: format 'ffteq <band1_db> <band2_db> ... [ramp_ms]'\n");
    }
}

void FFTEqualizerDSP::ramp_params() {
    for (size_t i = 0; i < band_count_; ++i) {
        band_gains_[i].next();
    }
}

// --- NetworkAudioSinkSource ---
NetworkAudioSinkSource::NetworkAudioSinkSource(std::string_view n) : DSPNode(n) {}

void NetworkAudioSinkSource::process(std::span<float> buffer) {
    if (!socket_idx_valid_ || buffer.empty()) return;
    if (is_sink_) {
        net::Packet packet;
        packet.dst_ip = remote_ip_;
        packet.dst_port = remote_port_;
        packet.audio_channels = channels_;
        size_t bytes = std::min(buffer.size() * sizeof(float), net::MAX_PAYLOAD);
        util::memcpy(packet.data.data(), buffer.data(), bytes);
        packet.data_len = bytes;
        packet.timestamp_us = kernel::g_platform ? kernel::g_platform->get_timer_ops()->get_system_time_us() : 0;
        net::g_net_manager.send(socket_idx_, packet, true);
    } else {
        net::Packet packet;
        if (net::g_net_manager.receive(socket_idx_, packet, true) && packet.audio_channels == channels_) {
            if (packet.timestamp_us > 0) {
                // Simplified: Log timestamp for synchronization
            }
            size_t samples = std::min(packet.data_len / sizeof(float), buffer.size());
            util::memcpy(buffer.data(), packet.data.data(), samples * sizeof(float));
        }
    }
}

void NetworkAudioSinkSource::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: netaudio <sink|source> <ip> <port> <channels>\n");
        return;
    }
    char mode[8], ip_str[16];
    int port, channels;
    if (std::sscanf(args, "netaudio %7s %15s %d %d", mode, ip_str, &port, &channels) == 4 &&
        port > 0 && port <= 65535 && channels >= 1 && channels <= 8) {
        uint32_t ip_addr;
        if (!util::ipv4_to_uint32(ip_str, ip_addr)) {
            if (uart_ops) uart_ops->puts("Invalid IP address\n");
            return;
        }
        is_sink_ = util::strcmp(mode, "sink") == 0;
        if (!is_sink_ && util::strcmp(mode, "source") != 0) {
            if (uart_ops) uart_ops->puts("Invalid mode: sink, source\n");
            return;
        }
        remote_ip_.addr = ip_addr;
        remote_port_ = static_cast<uint16_t>(port);
        channels_ = static_cast<uint8_t>(channels);
        socket_idx_ = net::g_net_manager.create_udp_socket({0x0A000001}, 1234 + socket_idx_);
        socket_idx_valid_ = socket_idx_ >= 0;
        if (socket_idx_valid_) {
            if (uart_ops) {
                char buf[16];
                uart_ops->puts("Network audio ");
                uart_ops->puts(is_sink_ ? "sink" : "source");
                uart_ops->puts(" set: ip=");
                uart_ops->puts(ip_str);
                uart_ops->puts(", port=");
                std::snprintf(buf, sizeof(buf), "%d", port);
                uart_ops->puts(buf);
                uart_ops->puts("\n");
            }
        } else if (uart_ops) {
            uart_ops->puts("Failed to create network audio socket\n");
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid netaudio: format 'netaudio <sink|source> <ip> <port> <channels>'\n");
    }
}

// --- DSPGraph ---
DSPGraph::DSPGraph(std::string_view name) : name_(name) {}

bool DSPGraph::add_node(DSPNode* node) {
    if (!node) return false;
    for (const auto& n : nodes_) {
        if (n->name() == node->name()) return false;
    }
    nodes_.emplace_back(node);
    return true;
}

bool DSPGraph::remove_node(std::string_view name) {
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
                           [&name](const auto& n) { return n->name() == name; });
    if (it == nodes_.end()) return false;
    nodes_.erase(it);
    return true;
}

bool DSPGraph::configure_node(std::string_view name, const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
                           [&name](const auto& n) { return n->name() == name; });
    if (it == nodes_.end()) {
        if (uart_ops) {
            uart_ops->puts("Node not found: ");
            uart_ops->puts(name.data());
            uart_ops->puts("\n");
        }
        return false;
    }
    (*it)->configure(args, uart_ops);
    return true;
}

void DSPGraph::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    for (auto& node : nodes_) {
        node->process(buffer);
    }
}

void DSPGraph::reset() {
    for (auto& node : nodes_) {
        node->reset();
    }
}

} // namespace kernel::dsp