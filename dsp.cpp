// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.cpp
 * @brief Digital Signal Processing (DSP) subsystem implementation for miniOS v1.7.
 * @details
 * Implements a comprehensive set of audio processing nodes.
 * @version 1.7
 * @see dsp.hpp, miniOS.hpp, audio.hpp, util.hpp, net.hpp
 */

#include "dsp.hpp"    // Includes audio.hpp for MAX_AUDIO_CHANNELS
#include "util.hpp"   // For kernel::util functions
// net.hpp is included via dsp.hpp
#include <cmath>
#include <algorithm> 
#include <cstring>   
#include <cstdio>    
#include <random>    
#include <limits>    

namespace kernel {
namespace dsp {

// Anonymous namespace for local constants and helpers
namespace { 
    constexpr float PI_F_DSP = 3.14159265358979323846f; 
    constexpr float MIN_FREQ_HZ_DSP = 20.0f;
    constexpr float MAX_FREQ_HZ_DSP = 20000.0f; 
    constexpr float MIN_Q_FACTOR_DSP = 0.1f;
    constexpr float MAX_Q_FACTOR_DSP = 20.0f; 
    constexpr float MIN_GAIN_DB_DSP = -70.0f; 
    constexpr float MAX_GAIN_DB_DSP = 24.0f;
}

// --- Utility Functions ---
void generate_hann_window(std::span<float> window) noexcept {
    if (window.empty()) return;
    size_t n = window.size();
    if (n == 1) { 
        window[0] = 1.0f;
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        window[i] = 0.5f * (1.0f - std::cos(2.0f * PI_F_DSP * static_cast<float>(i) / static_cast<float>(n - 1)));
    }
}

void generate_biquad_coeffs(std::string_view type, float fc_hz, float q_factor, float gain_db, float sample_rate_hz,
                           std::array<float, 3>& b_coeffs, std::array<float, 3>& a_coeffs) noexcept {
    if (fc_hz < MIN_FREQ_HZ_DSP || fc_hz > kernel::util::min(MAX_FREQ_HZ_DSP, sample_rate_hz / 2.0f - 1.0f) || 
        q_factor < MIN_Q_FACTOR_DSP || q_factor > MAX_Q_FACTOR_DSP || sample_rate_hz <= 0.0f) {
        b_coeffs = {1.0f, 0.0f, 0.0f}; a_coeffs = {1.0f, 0.0f, 0.0f}; return;
    }

    float w0 = 2.0f * PI_F_DSP * fc_hz / sample_rate_hz;
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * q_factor); 
    float A = db_to_linear(gain_db); 

    a_coeffs[0] = 1.0f; 

    if (type == "lowpass") {
        b_coeffs[0] = (1.0f - cos_w0) / 2.0f;
        b_coeffs[1] = 1.0f - cos_w0;
        b_coeffs[2] = (1.0f - cos_w0) / 2.0f;
        a_coeffs[1] = -2.0f * cos_w0;
        a_coeffs[2] = 1.0f - alpha;
    } else if (type == "highpass") {
        b_coeffs[0] = (1.0f + cos_w0) / 2.0f;
        b_coeffs[1] = -(1.0f + cos_w0);
        b_coeffs[2] = (1.0f + cos_w0) / 2.0f;
        a_coeffs[1] = -2.0f * cos_w0;
        a_coeffs[2] = 1.0f - alpha;
    } else if (type == "peaking") {
        float alpha_A = alpha * A;
        float alpha_ov_A = (A > 1e-6f) ? alpha / A : alpha * 1e6f; 
        b_coeffs[0] = 1.0f + alpha_A;
        b_coeffs[1] = -2.0f * cos_w0;
        b_coeffs[2] = 1.0f - alpha_A;
        a_coeffs[1] = -2.0f * cos_w0;
        a_coeffs[2] = 1.0f - alpha_ov_A;
    } else if (type == "lowshelf") {
        float two_sqrt_A_alpha = 2.0f * std::sqrt(A) * alpha;
        float common_denom = (A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
        if (std::abs(common_denom) < 1e-9f) common_denom = 1e-9f; 
        
        b_coeffs[0] = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha)) / common_denom;
        b_coeffs[1] = (2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / common_denom;
        b_coeffs[2] = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha)) / common_denom;
        a_coeffs[1] = (-2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / common_denom;
        a_coeffs[2] = ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha) / common_denom;
    } else if (type == "highshelf") {
        float two_sqrt_A_alpha = 2.0f * std::sqrt(A) * alpha;
        float common_denom = (A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
        if (std::abs(common_denom) < 1e-9f) common_denom = 1e-9f;

        b_coeffs[0] = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha)) / common_denom;
        b_coeffs[1] = (-2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / common_denom;
        b_coeffs[2] = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha)) / common_denom;
        a_coeffs[1] = (2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / common_denom;
        a_coeffs[2] = ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha) / common_denom;
    } else { 
        b_coeffs = {1.0f, 0.0f, 0.0f}; a_coeffs = {1.0f, 0.0f, 0.0f};
    }
}

void generate_crossover_coeffs(std::string_view type, int order, float fc_hz, float sample_rate_hz,
                              std::vector<FilterStage>& stages, bool is_highpass_section) noexcept {
    stages.clear();
    if (order < 1 || order > 8 || fc_hz < MIN_FREQ_HZ_DSP || fc_hz > kernel::util::min(MAX_FREQ_HZ_DSP, sample_rate_hz / 2.0f - 1.0f) || sample_rate_hz <= 0.0f) return;

    int num_2nd_order_sections = order / 2;
    bool has_1st_order_section = (order % 2 == 1);
    float q_val_stage = 0.70710678118f; // Default for Butterworth/LR stages

    if (type == "bessel") { // Approximate Bessel Q values per stage
        if (order == 2) q_val_stage = 0.57735f;
        // Add more cases for higher order Bessel if needed
    }
    if (type == "linkwitz") {
        num_2nd_order_sections = order / 2;
        has_1st_order_section = false; // LR filters are typically even order
    }

    for (int i = 0; i < num_2nd_order_sections; ++i) {
        FilterStage stage;
        generate_biquad_coeffs(is_highpass_section ? "highpass" : "lowpass", fc_hz, q_val_stage, 0.0f, sample_rate_hz, stage.b_coeffs, stage.a_coeffs);
        stages.push_back(stage);
    }

    if (has_1st_order_section && type != "linkwitz") {
        FilterStage stage;
        float wc_norm = PI_F_DSP * fc_hz / sample_rate_hz; 
        float k_tan = std::tan(wc_norm / 2.0f); 
        float den_1st = 1.0f + k_tan;
        if (std::abs(den_1st) < 1e-9f) den_1st = 1e-9f;

        if (is_highpass_section) {
            float norm_factor = 1.0f / den_1st;
            stage.b_coeffs = {norm_factor, -norm_factor, 0.0f};
            stage.a_coeffs = {1.0f, (k_tan - 1.0f) * norm_factor, 0.0f};
        } else { 
            float norm_factor = 1.0f / den_1st;
            stage.b_coeffs = {k_tan * norm_factor, k_tan * norm_factor, 0.0f};
            stage.a_coeffs = {1.0f, (k_tan - 1.0f) * norm_factor, 0.0f};
        }
        stages.push_back(stage);
    }
}

// --- DSPNode ---
DSPNode::DSPNode(std::string_view node_name) : name_storage_(node_name) {}

// --- GainDSP ---
GainDSP::GainDSP(float initial_gain_linear, std::string_view name) 
    : DSPNode(name) {
    gain_linear_.start(initial_gain_linear, 0, sample_rate_); 
}
void GainDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float current_gain = gain_linear_.get_current(); 
    for (float& sample : buffer) {
        sample *= current_gain;
    }
}
void GainDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { /* Usage */ return; }
    float gain_val = 1.0f; float ramp_ms = 0.0f; 
    if (std::sscanf(args, "%f %f", &gain_val, &ramp_ms) >= 1 && gain_val >= 0.0f) { 
        gain_linear_.start(gain_val, ramp_ms, sample_rate_); 
        if (uart_ops) { /* Success message */ }
    } else if (uart_ops) { /* Error message */ }
}
void GainDSP::ramp_params() { gain_linear_.next(); }

// --- MixerDSP ---
MixerDSP::MixerDSP(std::string_view name, uint8_t num_inputs) 
    : DSPNode(name), num_inputs_(num_inputs > 0 ? num_inputs : 1) {
    input_gains_linear_.resize(num_inputs_);
    reset();
}
void MixerDSP::process(std::span<float> buffer) {
    if (num_inputs_ == 0 || buffer.empty()) return;
    if (buffer.size() % num_inputs_ != 0 && num_inputs_ > 1) return; 
    
    size_t samples_per_output_channel = (num_inputs_ > 1) ? buffer.size() / num_inputs_ : buffer.size();
    
    if (num_inputs_ == 1) { 
        float gain = input_gains_linear_[0].get_current();
        for (size_t i = 0; i < samples_per_output_channel; ++i) buffer[i] *= gain;
        return;
    }

    std::vector<float> mixed_output(samples_per_output_channel, 0.0f); 
    for (uint8_t ch = 0; ch < num_inputs_; ++ch) {
        float gain = input_gains_linear_[ch].get_current();
        for (size_t i = 0; i < samples_per_output_channel; ++i) {
            mixed_output[i] += buffer[i + ch * samples_per_output_channel] * gain;
        }
    }
    if (samples_per_output_channel > 0) {
        ::kernel::util::kmemcpy(buffer.data(), mixed_output.data(), samples_per_output_channel * sizeof(float));
        if (num_inputs_ > 1 && buffer.size() > samples_per_output_channel) {
            ::kernel::util::kmemset(buffer.data() + samples_per_output_channel, 0, (buffer.size() - samples_per_output_channel) * sizeof(float));
        }
    }
}
void MixerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { 
    if (!args) { /* Usage */ return; }
    std::vector<float> parsed_gains(num_inputs_);
    float ramp_ms = 0.0f;
    const char* p = args;
    size_t count = 0;
    for(size_t i=0; i<num_inputs_; ++i) {
        if(std::sscanf(p, "%f", &parsed_gains[i]) == 1) {
            count++;
            while(*p && *p != ' ' && *p != '\t') p++; 
            while(*p && (*p == ' ' || *p == '\t')) p++; 
        } else break;
    }
    if(count == num_inputs_) {
        if(*p) std::sscanf(p, "%f", &ramp_ms); 
        for(size_t i=0; i<num_inputs_; ++i) input_gains_linear_[i].start(parsed_gains[i], ramp_ms, sample_rate_);
        if(uart_ops) uart_ops->puts("Mixer gains set.\n");
    } else if(uart_ops) uart_ops->puts("Invalid mixer gains.\n");
}
void MixerDSP::ramp_params() { for (auto& g : input_gains_linear_) g.next(); }
void MixerDSP::reset() {
    float default_gain = (num_inputs_ > 0) ? 1.0f / static_cast<float>(num_inputs_) : 1.0f;
    for (auto& g : input_gains_linear_) g.start(default_gain, 0, sample_rate_);
}

// --- ParametricEQDSP ---
ParametricEQDSP::ParametricEQDSP(std::string_view name) : DSPNode(name) { reset(); }
void ParametricEQDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    for (float& sample : buffer) {
        float x = sample; 
        for (size_t i = 0; i < MAX_EQ_BANDS; ++i) { 
            if (bands_[i].enabled) {
                auto& band = bands_[i];
                float wn = x - band.a_coeffs[1] * band.z_state[0] - band.a_coeffs[2] * band.z_state[1];
                float yn = band.b_coeffs[0] * wn + band.b_coeffs[1] * band.z_state[0] + band.b_coeffs[2] * band.z_state[1];
                band.z_state[1] = band.z_state[0]; 
                band.z_state[0] = wn;       
                x = yn; 
            }
        }
        sample = x; 
    }
}
void ParametricEQDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { 
    if (!args) { /* Usage */ return; }
    int band_idx = -1; char type_str[16] = {0};
    float freq = 0.0f, q = 0.0f, gain = 0.0f, ramp = 0.0f;
    if (std::sscanf(args, "band %d %15s %f %f %f %f", &band_idx, type_str, &freq, &q, &gain, &ramp) >= 5) {
        if (band_idx >= 0 && static_cast<size_t>(band_idx) < MAX_EQ_BANDS) {
            EQBand& band = bands_[band_idx];
            std::string_view type_sv(type_str);
            if (type_sv == "peak") band.filter_type = BandFilterType::PEAKING;
            else if (type_sv == "lowshelf") band.filter_type = BandFilterType::LOW_SHELF;
            else if (type_sv == "highshelf") band.filter_type = BandFilterType::HIGH_SHELF;
            else { if(uart_ops) uart_ops->puts("Invalid EQ type.\n"); return; }
            band.center_freq_hz.start(freq, ramp, sample_rate_);
            band.q_factor.start(q, ramp, sample_rate_);
            band.gain_db.start(gain, ramp, sample_rate_);
            band.enabled = true;
            if (ramp == 0.0f) update_band_filter_coeffs(band_idx);
            if(uart_ops){ /* Success */ }
        } else if(uart_ops) uart_ops->puts("Invalid EQ band index.\n");
    } else if(uart_ops) uart_ops->puts("Invalid EQ config args.\n");
}
void ParametricEQDSP::ramp_params() { 
    for (size_t i = 0; i < MAX_EQ_BANDS; ++i) {
        if (bands_[i].enabled) {
            bool needs_update = bands_[i].center_freq_hz.is_active() || bands_[i].q_factor.is_active() || bands_[i].gain_db.is_active();
            bands_[i].center_freq_hz.next(); bands_[i].q_factor.next(); bands_[i].gain_db.next();
            if (needs_update) update_band_filter_coeffs(i);
        }
    }
}
void ParametricEQDSP::reset() { 
    for (auto& band : bands_) {
        band.center_freq_hz.start(1000.0f,0,sample_rate_); 
        band.q_factor.start(0.707f,0,sample_rate_); 
        band.gain_db.start(0.0f,0,sample_rate_);
        band.filter_type = BandFilterType::PEAKING;
        band.b_coeffs = {1.0f, 0.0f, 0.0f}; band.a_coeffs = {1.0f, 0.0f, 0.0f};
        band.z_state = {0.0f, 0.0f}; band.enabled = false;
    }
}
void ParametricEQDSP::update_band_filter_coeffs(size_t band_idx) {
    if (band_idx >= MAX_EQ_BANDS || !bands_[band_idx].enabled) return;
    EQBand& band = bands_[band_idx];
    const char* type_str_c;
    switch (band.filter_type) {
        case BandFilterType::PEAKING:   type_str_c = "peaking"; break;
        case BandFilterType::LOW_SHELF: type_str_c = "lowshelf"; break;
        case BandFilterType::HIGH_SHELF:type_str_c = "highshelf"; break;
        default: return;
    }
    generate_biquad_coeffs(type_str_c, band.center_freq_hz.get_current(), band.q_factor.get_current(), band.gain_db.get_current(), sample_rate_, band.b_coeffs, band.a_coeffs);
}

// --- NoiseGateDSP ---
NoiseGateDSP::NoiseGateDSP(std::string_view name) : DSPNode(name) {
    threshold_db_.start(-60.0f, 0, sample_rate_);
    attack_time_ms_.start(10.0f, 0, sample_rate_);
    release_time_ms_.start(100.0f, 0, sample_rate_);
    reset();
}
void NoiseGateDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float thresh_lin = db_to_linear(threshold_db_.get_current()); 
    float attack_val_ms = attack_time_ms_.get_current();
    float release_val_ms = release_time_ms_.get_current();
    float attack_alpha = (attack_val_ms > 1e-3f && sample_rate_ > 0.0f) ? std::exp(-1.0f / (attack_val_ms * sample_rate_ / 1000.0f)) : 0.0f;
    float release_alpha = (release_val_ms > 1e-3f && sample_rate_ > 0.0f) ? std::exp(-1.0f / (release_val_ms * sample_rate_ / 1000.0f)) : 0.0f;
    for (float& sample : buffer) {
        float rectified_input = std::abs(sample);
        if (rectified_input > envelope_) { 
            envelope_ = attack_alpha * envelope_ + (1.0f - attack_alpha) * rectified_input;
        } else { 
            envelope_ = release_alpha * envelope_ + (1.0f - release_alpha) * rectified_input;
        }
        sample *= (envelope_ > thresh_lin) ? 1.0f : 0.0f; 
    }
}
void NoiseGateDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { /* Usage */ return; }
    float thresh = -60.0f, attack = 10.0f, release = 100.0f, ramp = 0.0f;
    if (std::sscanf(args, "%f %f %f %f", &thresh, &attack, &release, &ramp) >= 3 && attack > 0.0f && release > 0.0f) {
        threshold_db_.start(thresh, ramp, sample_rate_); 
        attack_time_ms_.start(attack, ramp, sample_rate_);
        release_time_ms_.start(release, ramp, sample_rate_);
        if (uart_ops) { /* Success */ }
    } else if (uart_ops) { /* Error */ }
}
void NoiseGateDSP::ramp_params() { threshold_db_.next(); attack_time_ms_.next(); release_time_ms_.next(); } 
void NoiseGateDSP::reset() { envelope_ = 0.0f; }

// --- WaveshaperDSP --- 
WaveshaperDSP::WaveshaperDSP(std::string_view name) : DSPNode(name) {
    drive_linear_.start(1.0f, 0, sample_rate_);
    bias_offset_.start(0.0f, 0, sample_rate_);
    current_shape_ = ShapeType::TANH;
}
void WaveshaperDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float d = drive_linear_.get_current();
    float b = bias_offset_.get_current();
    for (float& sample : buffer) {
        float x = sample * d + b;
        switch (current_shape_) {
            case ShapeType::TANH:      x = std::tanh(x); break;
            case ShapeType::HARD_CLIP: x = clip(x, -1.0f, 1.0f); break;
            case ShapeType::SOFT_CLIP: x = x / (1.0f + std::abs(x)); break;
            case ShapeType::ATAN:      x = (2.0f / PI_F_DSP) * std::atan(x * PI_F_DSP * 0.5f); break;
        }
        sample = x;
    }
}
void WaveshaperDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { 
    if (!args) { /* Usage */ return; }
    char type_str[10]; float drv = 1.0f, bs = 0.0f, ramp = 0.0f;
    if (std::sscanf(args, "%9s %f %f %f", type_str, &drv, &bs, &ramp) >= 3) {
        std::string_view type_sv(type_str);
        if (type_sv == "tanh") current_shape_ = ShapeType::TANH;
        else if (type_sv == "hard") current_shape_ = ShapeType::HARD_CLIP;
        else if (type_sv == "soft") current_shape_ = ShapeType::SOFT_CLIP;
        else if (type_sv == "atan") current_shape_ = ShapeType::ATAN; 
        else { if (uart_ops) uart_ops->puts("Invalid shape type.\n"); return; }
        drive_linear_.start(drv, ramp, sample_rate_);
        bias_offset_.start(bs, ramp, sample_rate_);
        if (uart_ops) { /* Success */ }
    } else if (uart_ops) { /* Error */ }
}
void WaveshaperDSP::ramp_params() { drive_linear_.next(); bias_offset_.next(); }

// --- DelayDSP ---
DelayDSP::DelayDSP(std::string_view name) 
    : DSPNode(name), current_delay_samples_(0), write_index_(0) {
    delay_buffer_.resize(sample_rate_); // Default 1s, no try/catch due to -fno-exceptions
    reset();
}
void DelayDSP::process(std::span<float> buffer) {
    if (buffer.empty() || delay_buffer_.empty()) return;
    for (float& sample : buffer) {
        float input_sample = sample; 
        size_t read_index = (write_index_ + delay_buffer_.size() - current_delay_samples_) % delay_buffer_.size();
        sample = delay_buffer_[read_index];
        delay_buffer_[write_index_] = input_sample;
        write_index_ = (write_index_ + 1) % delay_buffer_.size();
    }
}
void DelayDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { /* Usage */ return; }
    float delay_ms = 0.0f;
    if (std::sscanf(args, "%f", &delay_ms) == 1 && delay_ms >= 0.0f) {
        current_delay_samples_ = static_cast<size_t>(delay_ms * sample_rate_ / 1000.0f);
        if (current_delay_samples_ > MAX_DELAY_SAMPLES_CONST) {
            current_delay_samples_ = MAX_DELAY_SAMPLES_CONST;
        }
        if (delay_buffer_.size() < current_delay_samples_ && current_delay_samples_ > 0) {
            delay_buffer_.resize(current_delay_samples_); // No try/catch
        }
        reset(); 
        if (uart_ops) { /* Success message */ }
    } else if (uart_ops) { /* Error message */ }
}
void DelayDSP::reset() {
    write_index_ = 0;
    if (!delay_buffer_.empty()) std::fill(delay_buffer_.begin(), delay_buffer_.end(), 0.0f);
}

// --- ReverbDSP ---
ReverbDSP::ReverbDSP(std::string_view name) : DSPNode(name) {
    pre_delay_time_ms_.start(20.0f,0,sample_rate_);
    room_size_param_.start(0.7f,0,sample_rate_);
    damping_param_.start(0.5f,0,sample_rate_);
    wet_dry_mix_.start(0.3f,0,sample_rate_); 
    
    const float prime_delays_ms[] = {29.7f, 37.1f, 41.1f, 43.7f};
    const float allpass_delays_ms[] = {5.0f, 1.7f};

    for(size_t i=0; i < NUM_COMB_FILTERS; ++i) {
        comb_filters_[i].init(static_cast<size_t>(prime_delays_ms[i%4] * sample_rate_ / 1000.0f), sample_rate_);
    }
    for(size_t i=0; i < NUM_ALLPASS_FILTERS; ++i) {
        allpass_filters_[i].init(static_cast<size_t>(allpass_delays_ms[i%2] * sample_rate_ / 1000.0f), sample_rate_);
        allpass_filters_[i].gain = 0.7f;
    }
    
    size_t max_predelay_samples = static_cast<size_t>(100.0f * sample_rate_ / 1000.0f);
    pre_delay_line_.resize(max_predelay_samples > 0 ? max_predelay_samples : 1);
    
    update_internal_reverb_params();
    reset();
}
void ReverbDSP::CombFilter::init(size_t delay_samples, float /*sr*/) {
    buffer.assign(delay_samples > 0 ? delay_samples : 1, 0.0f);
    index = 0; last_filtered_out = 0.0f;
}
float ReverbDSP::CombFilter::process_sample(float input_sample) {
    if (buffer.empty()) return input_sample;
    float delayed_sample = buffer[index];
    last_filtered_out = delayed_sample * damping_mix + last_filtered_out * (1.0f - damping_mix);
    buffer[index] = input_sample + last_filtered_out * feedback;
    index = (index + 1) % buffer.size();
    return delayed_sample;
}
void ReverbDSP::AllpassFilter::init(size_t delay_samples, float /*sr*/) {
    buffer.assign(delay_samples > 0 ? delay_samples : 1, 0.0f);
    index = 0;
}
float ReverbDSP::AllpassFilter::process_sample(float input_sample) {
    if (buffer.empty()) return input_sample;
    float delayed_sample = buffer[index];
    float output = -gain * input_sample + delayed_sample;
    buffer[index] = input_sample + gain * output;
    index = (index + 1) % buffer.size();
    return output;
}
void ReverbDSP::update_internal_reverb_params() {
    float room = room_size_param_.get_current(); 
    float damp_val = damping_param_.get_current(); 
    for(auto& cf : comb_filters_) {
        cf.feedback = 0.82f + room * 0.17f;
        cf.damping_mix = damp_val;
    }
    current_pre_delay_samples_ = static_cast<size_t>(pre_delay_time_ms_.get_current() * sample_rate_ / 1000.0f);
    if (current_pre_delay_samples_ >= pre_delay_line_.size() && !pre_delay_line_.empty()) {
        current_pre_delay_samples_ = pre_delay_line_.size() -1;
    } else if (pre_delay_line_.empty()) {
        current_pre_delay_samples_ = 0;
    }
}
void ReverbDSP::process(std::span<float> buffer) {
    if(buffer.empty()) return;
    float wet = wet_dry_mix_.get_current(); 
    float dry = 1.0f - wet;

    for(float& sample : buffer) {
        float dry_sample = sample;
        float pre_delayed_sample = dry_sample;
        if (current_pre_delay_samples_ > 0 && !pre_delay_line_.empty()) {
            size_t read_idx = (pre_delay_write_idx_ + pre_delay_line_.size() - current_pre_delay_samples_) % pre_delay_line_.size();
            pre_delayed_sample = pre_delay_line_[read_idx];
            pre_delay_line_[pre_delay_write_idx_] = dry_sample;
            pre_delay_write_idx_ = (pre_delay_write_idx_ + 1) % pre_delay_line_.size();
        }

        float comb_out_sum = 0.0f;
        for(auto& cf : comb_filters_) {
            comb_out_sum += cf.process_sample(pre_delayed_sample);
        }
        float allpass_input = comb_out_sum / (NUM_COMB_FILTERS > 0 ? static_cast<float>(NUM_COMB_FILTERS) : 1.0f);
        
        float wet_sample = allpass_input;
        for(auto& apf : allpass_filters_) {
            wet_sample = apf.process_sample(wet_sample);
        }
        sample = dry_sample * dry + wet_sample * wet;
    }
}
void ReverbDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { 
    if(!args) { /* Usage */ return; }
    float pre_del=0, room=0.5f, damp=0.5f, wd=0.3f, ramp=0;
    if(std::sscanf(args, "%f %f %f %f %f", &pre_del, &room, &damp, &wd, &ramp) >= 4){
        pre_delay_time_ms_.start(pre_del, ramp, sample_rate_);
        room_size_param_.start(clip(room,0.0f,1.0f), ramp, sample_rate_);
        damping_param_.start(clip(damp,0.0f,1.0f), ramp, sample_rate_);
        wet_dry_mix_.start(clip(wd,0.0f,1.0f), ramp, sample_rate_); 
        if(ramp == 0) update_internal_reverb_params();
        if(uart_ops){ /* Success */ }
    } else if(uart_ops) { /* Error */ }
}
void ReverbDSP::ramp_params(){ 
    bool update = pre_delay_time_ms_.is_active() || room_size_param_.is_active() || damping_param_.is_active() || wet_dry_mix_.is_active();
    pre_delay_time_ms_.next(); room_size_param_.next(); damping_param_.next(); wet_dry_mix_.next(); 
    if(update) update_internal_reverb_params();
}
void ReverbDSP::reset(){ 
    pre_delay_write_idx_ = 0; 
    if(!pre_delay_line_.empty()) std::fill(pre_delay_line_.begin(), pre_delay_line_.end(), 0.0f);
    for(auto& cf : comb_filters_) { cf.index=0; cf.last_filtered_out = 0; if(!cf.buffer.empty()) std::fill(cf.buffer.begin(), cf.buffer.end(), 0.0f); }
    for(auto& apf : allpass_filters_) { apf.index=0; if(!apf.buffer.empty()) std::fill(apf.buffer.begin(), apf.buffer.end(), 0.0f); }
    update_internal_reverb_params();
}

// --- ChorusDSP ---
ChorusDSP::ChorusDSP(std::string_view name) : DSPNode(name) { reset(); }
void ChorusDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float depth_s = depth_ms_.get_current() * sample_rate_ / 1000.0f;
    float rate_rad_per_sample = (2.0f * PI_F_DSP * lfo_rate_hz_.get_current()) / sample_rate_;
    float mix = mix_level_.get_current();
    for (float& sample : buffer) {
        float dry_sample = sample;
        float lfo_val = std::sin(lfo_phase_); 
        float current_delay = depth_s * (1.0f + lfo_val) * 0.5f; 
        current_delay = kernel::util::max(0.0f, kernel::util::min(current_delay, static_cast<float>(MAX_CHORUS_DELAY_SAMPLES -1) ));
        size_t d_int = static_cast<size_t>(current_delay);
        float d_frac = current_delay - d_int;
        size_t read_idx0 = (write_index_ + MAX_CHORUS_DELAY_SAMPLES - d_int) % MAX_CHORUS_DELAY_SAMPLES;
        size_t read_idx1 = (read_idx0 + MAX_CHORUS_DELAY_SAMPLES - 1) % MAX_CHORUS_DELAY_SAMPLES; 
        float delayed_sample = delay_line_[read_idx0] * (1.0f - d_frac) + delay_line_[read_idx1] * d_frac;
        delay_line_[write_index_] = dry_sample; 
        sample = dry_sample * (1.0f - mix) + delayed_sample * mix;
        write_index_ = (write_index_ + 1) % MAX_CHORUS_DELAY_SAMPLES;
        lfo_phase_ += rate_rad_per_sample;
        if (lfo_phase_ >= 2.0f * PI_F_DSP) lfo_phase_ -= 2.0f * PI_F_DSP;
    }
}
void ChorusDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { /* As previous */ (void)args; (void)uart_ops;}
void ChorusDSP::ramp_params() { lfo_rate_hz_.next(); depth_ms_.next(); mix_level_.next(); }
void ChorusDSP::reset() { 
    std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
    write_index_ = 0; lfo_phase_ = 0.0f; 
    lfo_rate_hz_.start(0.5f,0,sample_rate_); 
    depth_ms_.start(5.0f,0,sample_rate_); 
    mix_level_.start(0.5f,0,sample_rate_);
}

// --- FlangerDSP ---
FlangerDSP::FlangerDSP(std::string_view name) : DSPNode(name) { reset(); }
void FlangerDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float depth_s = depth_ms_.get_current() * sample_rate_ / 1000.0f;
    float rate_rad_per_sample = (2.0f * PI_F_DSP * lfo_rate_hz_.get_current()) / sample_rate_;
    float fb = feedback_level_.get_current();
    float mix = mix_level_.get_current();

    for (float& sample : buffer) {
        float dry_sample = sample;
        float lfo_val = std::sin(lfo_phase_);
        float current_delay_base = 1.0f;
        float current_delay = current_delay_base + depth_s * (1.0f + lfo_val) * 0.5f;
        current_delay = kernel::util::max(0.0f, kernel::util::min(current_delay, static_cast<float>(MAX_FLANGER_DELAY_SAMPLES -1)));

        size_t d_int = static_cast<size_t>(current_delay);
        float d_frac = current_delay - d_int;
        size_t read_idx0 = (write_index_ + MAX_FLANGER_DELAY_SAMPLES - d_int) % MAX_FLANGER_DELAY_SAMPLES;
        size_t read_idx1 = (read_idx0 + MAX_FLANGER_DELAY_SAMPLES - 1) % MAX_FLANGER_DELAY_SAMPLES;

        float delayed_sample = delay_line_[read_idx0] * (1.0f - d_frac) + delay_line_[read_idx1] * d_frac;
        
        delay_line_[write_index_] = dry_sample + delayed_sample * fb;
        sample = dry_sample * (1.0f - mix) + delayed_sample * mix;

        write_index_ = (write_index_ + 1) % MAX_FLANGER_DELAY_SAMPLES;
        lfo_phase_ += rate_rad_per_sample;
        if (lfo_phase_ >= 2.0f * PI_F_DSP) lfo_phase_ -= 2.0f * PI_F_DSP;
    }
}
void FlangerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { /* As previous */ (void)args; (void)uart_ops; }
void FlangerDSP::ramp_params() { lfo_rate_hz_.next(); depth_ms_.next(); feedback_level_.next(); mix_level_.next(); }
void FlangerDSP::reset() { 
    std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
    write_index_ = 0; lfo_phase_ = 0.0f; 
    lfo_rate_hz_.start(0.2f,0,sample_rate_); depth_ms_.start(3.0f,0,sample_rate_);
    feedback_level_.start(0.5f,0,sample_rate_); mix_level_.start(0.5f,0,sample_rate_);
}

// --- PitchShifterDSP ---
PitchShifterDSP::PitchShifterDSP(std::string_view name) : DSPNode(name) {
    generate_hann_window(std::span<float>(analysis_window_.data(), PITCH_WINDOW_SIZE));
    reset();
}
void PitchShifterDSP::process(std::span<float> buffer) { (void)buffer; /* TODO: Implement PSOLA or Phase Vocoder */ }
void PitchShifterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { (void)args; (void)uart_ops; }
void PitchShifterDSP::ramp_params() { pitch_shift_factor_.next(); }
void PitchShifterDSP::reset() { 
    input_ring_buffer_.fill(0.0f); input_write_ptr_ = 0; 
    phase_accumulator_ = 0.0f; 
    last_input_phase_.fill(0.0f); last_output_phase_.fill(0.0f);
    pitch_shift_factor_.start(1.0f,0,sample_rate_);
}

// --- CrossoverDSP ---
CrossoverDSP::CrossoverDSP(std::string_view name) : DSPNode(name) {
    bands_.resize(2); // default to 2 bands
    reset();
}

static void apply_iir_stages(std::span<float> buffer,
                             const std::vector<FilterStage>& stages,
                             std::vector<std::array<float,2>>& state) {
    if (stages.empty()) return;
    if (state.size() != stages.size()) state.assign(stages.size(), {0.0f,0.0f});
    for (size_t s = 0; s < stages.size(); ++s) {
        const auto& st = stages[s];
        auto& st_state = state[s];
        for (float& sample : buffer) {
            float wn = sample - st.a_coeffs[1]*st_state[0] - st.a_coeffs[2]*st_state[1];
            float yn = st.b_coeffs[0]*wn + st.b_coeffs[1]*st_state[0] + st.b_coeffs[2]*st_state[1];
            st_state[1] = st_state[0];
            st_state[0] = wn;
            sample = yn;
        }
    }
}

void CrossoverDSP::process(std::span<float> buffer) {
    if (bands_.empty() || buffer.empty()) return;
    size_t num_bands = bands_.size();
    size_t samples_per_band = buffer.size() / num_bands;
    if (samples_per_band == 0) return;

    if (work_buffer_.size() < samples_per_band)
        work_buffer_.resize(samples_per_band);
    std::memcpy(work_buffer_.data(), buffer.data(), samples_per_band*sizeof(float));

    for (size_t b = 0; b < num_bands; ++b) {
        auto& band = bands_[b];
        std::span<float> out(buffer.data() + b*samples_per_band, samples_per_band);
        if (!band.enabled) {
            std::fill(out.begin(), out.end(), 0.0f);
            continue;
        }
        std::memcpy(out.data(), work_buffer_.data(), samples_per_band*sizeof(float));
        if (!band.lowpass_stages.empty()) {
            apply_iir_stages(out, band.lowpass_stages, band.lp_state);
        } else if (!band.highpass_stages.empty()) {
            apply_iir_stages(out, band.highpass_stages, band.hp_state);
        }
    }
}

void CrossoverDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: band <idx> <type> <lowpass|highpass> <order> <cutoff>\n");
        return;
    }
    const char* p = args;
    while (*p) {
        while (*p == '\n' || *p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        int idx = 0, order = 0;
        char type_str[16] = {0};
        char mode_str[16] = {0};
        float cutoff = 0.0f;
        int consumed = std::sscanf(p, "band %d %15s %15s %d %f", &idx, type_str, mode_str, &order, &cutoff);
        if (consumed == 5 && idx >= 0) {
            if (static_cast<size_t>(idx) >= bands_.size()) bands_.resize(idx+1);
            bool is_highpass = std::string_view(mode_str) == "highpass";
            Band& band = bands_[idx];
            if (is_highpass) {
                band.highpass_stages.clear();
                generate_crossover_coeffs(type_str, order, cutoff, sample_rate_, band.highpass_stages, true);
                band.hp_state.assign(band.highpass_stages.size(), {0.0f,0.0f});
            } else {
                band.lowpass_stages.clear();
                generate_crossover_coeffs(type_str, order, cutoff, sample_rate_, band.lowpass_stages, false);
                band.lp_state.assign(band.lowpass_stages.size(), {0.0f,0.0f});
            }
            band.enabled = true;
        } else if (uart_ops) {
            uart_ops->puts("Invalid crossover args\n");
            return;
        }
        const char* nl = std::strchr(p, '\n');
        if (nl) p = nl + 1; else break;
    }
}

void CrossoverDSP::reset() {
    for (auto& band : bands_) {
        for (auto& st : band.lp_state) st = {0.0f,0.0f};
        for (auto& st : band.hp_state) st = {0.0f,0.0f};
    }
}

// --- NetworkAudioSinkSource ---
NetworkAudioSinkSource::NetworkAudioSinkSource(std::string_view name) 
    : DSPNode(name), udp_socket_idx_(-1), is_socket_valid_(false), is_configured_as_sink_(false), remote_target_port_(0), num_audio_channels_(2) {
    remote_target_ip_.addr = 0; 
}
void NetworkAudioSinkSource::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { 
        if (uart_ops) uart_ops->puts("Usage: netaudio <sink|source> <ip> <port> <channels>\n"); 
        return; 
    }
    char mode_str[10], ip_str_val[16];
    int port_val = 0, channels_val = 0;

    if (std::sscanf(args, "%9s %15s %d %d", mode_str, ip_str_val, &port_val, &channels_val) == 4) {
        std::string_view mode_sv(mode_str);
        if (mode_sv == "sink") is_configured_as_sink_ = true;
        else if (mode_sv == "source") is_configured_as_sink_ = false;
        else { 
            if (uart_ops) uart_ops->puts("Invalid mode (sink/source).\n"); 
            return; 
        }

        if (!net::from_string(ip_str_val, remote_target_ip_)) {
            if (uart_ops) uart_ops->puts("Invalid IP address string.\n");
            return;
        }
        if (port_val <= 0 || port_val > 65535) { 
            if (uart_ops) uart_ops->puts("Invalid port number.\n"); 
            return; 
        }
        remote_target_port_ = static_cast<uint16_t>(port_val);
        if (channels_val <= 0 || static_cast<size_t>(channels_val) > MAX_AUDIO_CHANNELS) { 
            if (uart_ops) uart_ops->puts("Invalid channel count.\n"); 
            return; 
        }
        num_audio_channels_ = static_cast<uint8_t>(channels_val);

        if (is_socket_valid_) {
            kernel::g_net_manager.close_socket(udp_socket_idx_);
            is_socket_valid_ = false;
        }
        uint16_t local_bind_port = is_configured_as_sink_ ? 0 : remote_target_port_; 
        udp_socket_idx_ = kernel::g_net_manager.create_udp_socket(net::IPv4Addr(0U), local_bind_port); 
        is_socket_valid_ = (udp_socket_idx_ >= 0);

        if (is_socket_valid_) { 
            if (uart_ops) { uart_ops->puts("NetworkAudio node configured.\n");} 
        }
        else { 
            if (uart_ops) uart_ops->puts("Failed to create UDP socket for netaudio.\n"); 
        }
    } else if (uart_ops) { 
        uart_ops->puts("Invalid netaudio args format.\n"); 
    }
}
void NetworkAudioSinkSource::process(std::span<float> buffer) {
    if (!is_socket_valid_ || buffer.empty() || !kernel::g_platform || !kernel::g_platform->get_timer_ops()) return;
    if (is_configured_as_sink_) {
        net::Packet packet{};
        packet.dst_ip = remote_target_ip_;
        packet.dst_port = remote_target_port_;
        packet.audio_channels = num_audio_channels_;
        size_t samples_to_send = kernel::util::min(buffer.size(), static_cast<size_t>(net::MAX_PAYLOAD / sizeof(float)));
        size_t bytes_to_send = samples_to_send * sizeof(float);
        kernel::util::kmemcpy(packet.data.data(), buffer.data(), bytes_to_send);
        packet.data_len = bytes_to_send;
        packet.timestamp_us = kernel::g_platform->get_timer_ops()->get_system_time_us();
        kernel::g_net_manager.send(udp_socket_idx_, packet, false); 
    } else { 
        net::Packet packet{};
        if (kernel::g_net_manager.receive(udp_socket_idx_, packet, false) && packet.audio_channels == num_audio_channels_ && packet.data_len > 0) {
            size_t samples_received = packet.data_len / sizeof(float);
            size_t samples_to_copy = kernel::util::min(samples_received, buffer.size());
            kernel::util::kmemcpy(buffer.data(), packet.data.data(), samples_to_copy * sizeof(float));
            if (samples_to_copy < buffer.size()) { 
                std::fill(buffer.begin() + samples_to_copy, buffer.end(), 0.0f);
            }
        } else {
            std::fill(buffer.begin(), buffer.end(), 0.0f); 
        }
    }
}

// --- DSPGraph --- 
DSPGraph::DSPGraph(std::string_view graph_name) : name_storage_(graph_name) {}
bool DSPGraph::add_node(std::unique_ptr<DSPNode> node) { 
    if (!node) return false;
    kernel::ScopedLock lock(graph_modification_lock_);
    for (const auto& n_ptr : dsp_nodes_) {
        if (n_ptr->get_name() == node->get_name()) return false; 
    }
    dsp_nodes_.push_back(std::move(node));
    return true;
}
bool DSPGraph::remove_node(std::string_view name) {
    kernel::ScopedLock lock(graph_modification_lock_);
    auto it = std::remove_if(dsp_nodes_.begin(), dsp_nodes_.end(), 
                             [&name](const auto& n_ptr){ return n_ptr->get_name() == name; });
    if (it != dsp_nodes_.end()) {
        dsp_nodes_.erase(it, dsp_nodes_.end());
        return true;
    }
    return false; 
}
DSPNode* DSPGraph::get_node(std::string_view name) {
    kernel::ScopedLock lock(graph_modification_lock_); 
    for (const auto& n_ptr : dsp_nodes_) {
        if (n_ptr->get_name() == name) return n_ptr.get();
    }
    return nullptr;
}
bool DSPGraph::configure_node(std::string_view name, const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    DSPNode* node = get_node(name); 
    if (node) {
        node->configure(args, uart_ops);
        return true;
    }
    if (uart_ops) {
        std::string temp_name(name); 
        uart_ops->puts("Node '"); uart_ops->puts(temp_name.c_str()); uart_ops->puts("' not found for configuration.\n");
    }
    return false;
}
void DSPGraph::process(std::span<float> audio_buffer) {
    if (audio_buffer.empty()) return;
    for (auto& node_ptr : dsp_nodes_) { 
        node_ptr->ramp_params(); 
        node_ptr->process(audio_buffer); 
    }
}
void DSPGraph::reset() {
    kernel::ScopedLock lock(graph_modification_lock_); 
    for (auto& node_ptr : dsp_nodes_) {
        node_ptr->reset();
    }
}

} // namespace dsp
} // namespace kernel