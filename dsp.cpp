// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.cpp
 * @brief Digital Signal Processing (DSP) subsystem implementation for miniOS v1.7.
 * @details
 * Implements a comprehensive set of audio processing nodes.
 * @version 1.7
 * @see dsp.hpp, miniOS.hpp, audio.hpp, util.hpp, net.hpp
 */

#include "dsp.hpp"
#include "util.hpp" // For kernel::util functions
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
    constexpr float MAX_Q_FACTOR_DSP = 20.0f; // Increased for flexibility
    constexpr float MIN_GAIN_DB_DSP = -70.0f; // Wider range
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
    if (fc_hz < MIN_FREQ_HZ_DSP || fc_hz > std::min(MAX_FREQ_HZ_DSP, sample_rate_hz / 2.0f - 1.0f) || /* Nyquist */
        q_factor < MIN_Q_FACTOR_DSP || q_factor > MAX_Q_FACTOR_DSP || sample_rate_hz <= 0.0f) {
        b_coeffs = {1.0f, 0.0f, 0.0f}; a_coeffs = {1.0f, 0.0f, 0.0f}; return;
    }

    float w0 = 2.0f * PI_F_DSP * fc_hz / sample_rate_hz;
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * q_factor); 
    float A = db_to_linear(gain_db); // Linear gain for peaking/shelf

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
        float alpha_ov_A = (A > 1e-6f) ? alpha / A : alpha * 1e6f; // Avoid division by zero
        b_coeffs[0] = 1.0f + alpha_A;
        b_coeffs[1] = -2.0f * cos_w0;
        b_coeffs[2] = 1.0f - alpha_A;
        // a_coeffs[0] = 1.0f; (already set)
        a_coeffs[1] = -2.0f * cos_w0;
        a_coeffs[2] = 1.0f - alpha_ov_A;
    } else if (type == "lowshelf") {
        float two_sqrt_A_alpha = 2.0f * std::sqrt(A) * alpha;
        float common_denom = (A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
        if (std::abs(common_denom) < 1e-9f) common_denom = 1e-9f; // Avoid div by zero

        b_coeffs[0] = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha);
        b_coeffs[1] = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0);
        b_coeffs[2] = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha);
        // a_coeffs[0] is common_denom before normalization
        a_coeffs[1] = -2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0);
        a_coeffs[2] = (A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha;

        for(int i=0; i<3; ++i) b_coeffs[i] /= common_denom;
        a_coeffs[1] /= common_denom; a_coeffs[2] /= common_denom;
    } else if (type == "highshelf") {
        float two_sqrt_A_alpha = 2.0f * std::sqrt(A) * alpha;
        float common_denom = (A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
         if (std::abs(common_denom) < 1e-9f) common_denom = 1e-9f;

        b_coeffs[0] = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha);
        b_coeffs[1] = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0);
        b_coeffs[2] = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha);
        // a_coeffs[0] is common_denom before normalization
        a_coeffs[1] = 2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0);
        a_coeffs[2] = (A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha;
        
        for(int i=0; i<3; ++i) b_coeffs[i] /= common_denom;
        a_coeffs[1] /= common_denom; a_coeffs[2] /= common_denom;
    } else { 
        b_coeffs = {1.0f, 0.0f, 0.0f}; a_coeffs = {1.0f, 0.0f, 0.0f};
    }
}

void generate_crossover_coeffs(std::string_view type, int order, float fc_hz, float sample_rate_hz,
                              std::vector<std::array<float, 3>>& b_coeffs_stages, 
                              std::vector<std::array<float, 3>>& a_coeffs_stages,
                              bool is_highpass_section) noexcept {
    b_coeffs_stages.clear();
    a_coeffs_stages.clear();
    if (order < 1 || order > 8 || fc_hz < MIN_FREQ_HZ_DSP || fc_hz > std::min(MAX_FREQ_HZ_DSP, sample_rate_hz / 2.0f -1.0f) || sample_rate_hz <= 0.0f) return;

    int num_biquads = order / 2;
    bool has_first_order_stage = (order % 2 == 1);

    // Default Q for Butterworth stages
    float butter_q = 1.0f / (2.0f * std::sin(PI_F_DSP / (2.0f * static_cast<float>(order)))); // This is for overall Q, not per stage Q for higher orders.
                                                                                    // For cascaded 2nd order, Q for each stage is specific.
                                                                                    // Simpler: LR uses cascaded Q=0.707 Butterworth sections.
    float stage_q = 0.70710678118f; // Q for individual Butterworth 2nd order section
    if(type == "bessel"){
        // Bessel Q values are complex and depend on order and stage.
        // For simplicity, using a common approximation for lower orders.
        if(order <= 2) stage_q = 0.577f; // Q for 2nd order Bessel
        else stage_q = 0.52f; // Approximate for higher order first stage, needs table for accuracy
    }


    for (int i = 0; i < num_biquads; ++i) {
        std::array<float, 3> b_stage, a_stage;
        // For Linkwitz-Riley, each "order" means N/2 Butterworth stages.
        // So an LR4 (order=4 overall, 24dB/oct) means two 2nd order Butterworth stages (Q=0.7071).
        // The effective Q of an LR stage is 0.5, but it's made of Butterworth Q=0.7071 sections.
        float q_for_this_stage = stage_q;
        if (type == "linkwitz") {
             q_for_this_stage = 0.70710678118f; // Use Butterworth Q for LR construction
        }
        // Note: For higher order Butterworth/Bessel, Q varies per cascaded biquad stage.
        // This simplified version uses a fixed Q or simple adjustment.
        generate_biquad_coeffs(is_highpass_section ? "highpass" : "lowpass", fc_hz, q_for_this_stage, 0.0f, sample_rate_hz, b_stage, a_stage);
        b_coeffs_stages.push_back(b_stage);
        a_coeffs_stages.push_back(a_stage);
    }

    if (has_first_order_stage) {
        std::array<float, 3> b1_coeffs = {0.0f, 0.0f, 0.0f}, a1_coeffs = {1.0f, 0.0f, 0.0f};
        float wc_norm = PI_F_DSP * fc_hz / sample_rate_hz; // Normalized cutoff
        float k = std::tan(wc_norm); // Bilinear transform factor for 1st order

        if (is_highpass_section) { // 1st order HP: (s/wc) / (1 + s/wc)  =>  (1 - z^-1) / ((1+k) - (1-k)z^-1) * (1/k) ? No
                                  // HP: (1 * (1-z^-1)) / ((1+k) - (1-k)z^-1) after dividing by (1+k) for a0=1 for HP
                                  // Corrected 1st order HP (RBJ):
                                  // b0 = 1 / (1 + k)
                                  // b1 = -1 / (1 + k)
                                  // a1 = (k - 1) / (1 + k)
            float norm_factor = 1.0f / (1.0f + k);
            b1_coeffs[0] = norm_factor;
            b1_coeffs[1] = -norm_factor;
            a1_coeffs[1] = (k - 1.0f) * norm_factor;
        } else { // Lowpass
                 // LP: 1 / (1 + s/wc)  =>  (k * (1+z^-1)) / ((1+k) - (1-k)z^-1)
                 // Corrected 1st order LP (RBJ):
                 // b0 = k / (1 + k)
                 // b1 = k / (1 + k)
                 // a1 = (k - 1) / (1 + k)
            float norm_factor = 1.0f / (1.0f + k);
            b1_coeffs[0] = k * norm_factor;
            b1_coeffs[1] = k * norm_factor;
            a1_coeffs[1] = (k - 1.0f) * norm_factor;
        }
        b_coeffs_stages.push_back(b1_coeffs);
        a_coeffs_stages.push_back(a1_coeffs);
    }
}


// --- DSPNode ---
DSPNode::DSPNode(std::string_view node_name) : name_storage_(node_name) {}

// --- GainDSP ---
GainDSP::GainDSP(float initial_gain_linear, std::string_view name) 
    : DSPNode(name) {
    gain_linear_.current.store(initial_gain_linear, std::memory_order_relaxed);
    gain_linear_.target.store(initial_gain_linear, std::memory_order_relaxed);
}

void GainDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    float current_gain = gain_linear_.get_current(); 
    for (float& sample : buffer) {
        sample *= current_gain;
    }
}

void GainDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) uart_ops->puts("Usage: gain <value_lin> [ramp_ms (default 0)]\n");
        return;
    }
    float gain_val = 1.0f; 
    float ramp_ms = 0.0f; 
    int parsed_items = std::sscanf(args, "%f %f", &gain_val, &ramp_ms);

    if (parsed_items >= 1 && gain_val >= 0.0f) { 
        gain_linear_.start(gain_val, ramp_ms, sample_rate_); 
        if (uart_ops) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Gain set to %.2f (ramp %.2f ms)\n", gain_val, ramp_ms);
            uart_ops->puts(buf);
        }
    } else if (uart_ops) {
        uart_ops->puts("Invalid gain: format 'gain <value_lin> [ramp_ms]'. Value must be >= 0.\n");
    }
}

void GainDSP::ramp_params() {
    gain_linear_.next();
}

// --- MixerDSP ---
MixerDSP::MixerDSP(std::string_view name, uint8_t num_inputs) 
    : DSPNode(name), num_inputs_(num_inputs > 0 ? num_inputs : 1) { // Ensure at least 1 input
    input_gains_linear_.resize(num_inputs_);
    reset();
}

void MixerDSP::process(std::span<float> buffer) {
    if (num_inputs_ == 0 || buffer.empty()) return;
    // This process assumes buffer contains multiple channels, and we output to the first channel's space.
    // Or if num_inputs_ == 1, it's like a simple gain on the whole buffer.
    if (buffer.size() % num_inputs_ != 0 && num_inputs_ > 1) { /* Mismatched channels/buffer size */ return; }
    
    size_t samples_per_output_channel = (num_inputs_ > 1) ? buffer.size() / num_inputs_ : buffer.size();
    
    if (num_inputs_ == 1) { // Simple gain application if configured as 1 input
        float gain = input_gains_linear_[0].get_current();
        for (size_t i = 0; i < samples_per_output_channel; ++i) {
            buffer[i] *= gain;
        }
        return;
    }

    // N-to-1 mix (N input channels in buffer, 1 output channel written to start of buffer)
    std::vector<float> mixed_output(samples_per_output_channel, 0.0f);
    for (uint8_t ch = 0; ch < num_inputs_; ++ch) {
        float gain = input_gains_linear_[ch].get_current();
        for (size_t i = 0; i < samples_per_output_channel; ++i) {
            mixed_output[i] += buffer[i + ch * samples_per_output_channel] * gain;
        }
    }
    if (samples_per_output_channel > 0) {
        ::kernel::util::memcpy(buffer.data(), mixed_output.data(), samples_per_output_channel * sizeof(float));
        // Zero out the rest of the buffer if it was used for N-channel input
        if (num_inputs_ > 1 && buffer.size() > samples_per_output_channel) {
            ::kernel::util::memset(buffer.data() + samples_per_output_channel, 0, (buffer.size() - samples_per_output_channel) * sizeof(float));
        }
    }
}

void MixerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) {
        if (uart_ops) { /* Print usage for num_inputs_ */ }
        return;
    }
    std::vector<float> parsed_gains(num_inputs_);
    float ramp_ms = 0.0f;
    const char* current_arg_ptr = args;
    size_t gains_parsed_count = 0;

    for (size_t i = 0; i < num_inputs_; ++i) {
        if (std::sscanf(current_arg_ptr, "%f", &parsed_gains[i]) == 1) {
            gains_parsed_count++;
            while (*current_arg_ptr && *current_arg_ptr != ' ' && *current_arg_ptr != '\t') current_arg_ptr++; // Move to space
            while (*current_arg_ptr && (*current_arg_ptr == ' ' || *current_arg_ptr == '\t')) current_arg_ptr++; // Skip spaces
        } else {
            break; 
        }
    }

    if (gains_parsed_count == num_inputs_) {
        if (*current_arg_ptr != '\0') { // Check for ramp_ms
            std::sscanf(current_arg_ptr, "%f", &ramp_ms);
        }
        for (size_t i = 0; i < num_inputs_; ++i) {
            input_gains_linear_[i].start(parsed_gains[i], ramp_ms, sample_rate_);
        }
        if (uart_ops) uart_ops->puts("Mixer gains set.\n");
    } else if (uart_ops) {
        uart_ops->puts("Invalid mixer gains: incorrect number of gain values provided.\n");
    }
}

void MixerDSP::ramp_params() {
    for (auto& gain : input_gains_linear_) {
        gain.next();
    }
}

void MixerDSP::reset() {
    float default_gain = (num_inputs_ > 0) ? (1.0f / static_cast<float>(num_inputs_)) : 1.0f;
    for (auto& gain_ramp : input_gains_linear_) {
        gain_ramp.current.store(default_gain, std::memory_order_relaxed);
        gain_ramp.target.store(default_gain, std::memory_order_relaxed);
        gain_ramp.step_size_ = 0.0f;
        gain_ramp.steps_remaining_.store(0, std::memory_order_relaxed);
    }
}

// --- ParametricEQDSP ---
ParametricEQDSP::ParametricEQDSP(std::string_view name) : DSPNode(name) {
    reset(); 
}

void ParametricEQDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    // Assuming mono for simplicity, or process L/R with same coeffs if stereo
    for (float& sample : buffer) {
        float x = sample; 
        for (size_t i = 0; i < MAX_EQ_BANDS; ++i) { // Iterate all potential bands
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
            else { /* Invalid type */ return; }
            
            band.center_freq_hz.start(freq, ramp, sample_rate_);
            band.q_factor.start(q, ramp, sample_rate_);
            band.gain_db.start(gain, ramp, sample_rate_);
            band.enabled = true; // Enable the band on configuration
            if (ramp == 0.0f) update_band_filter_coeffs(band_idx); // Immediate update
            if (uart_ops) { /* Success message */ }
        } else if (uart_ops) { uart_ops->puts("Invalid band index.\n");}
    } else if (uart_ops) { /* Usage message */ }
}

void ParametricEQDSP::ramp_params() {
    for (size_t i = 0; i < MAX_EQ_BANDS; ++i) {
        if (bands_[i].enabled) {
            bool needs_update = bands_[i].center_freq_hz.is_active() ||
                                bands_[i].q_factor.is_active() ||
                                bands_[i].gain_db.is_active();
            bands_[i].center_freq_hz.next();
            bands_[i].q_factor.next();
            bands_[i].gain_db.next();
            if (needs_update) {
                update_band_filter_coeffs(i);
            }
        }
    }
}

void ParametricEQDSP::reset() {
    for (auto& band : bands_) {
        band.center_freq_hz.current.store(1000.0f, std::memory_order_relaxed);
        band.center_freq_hz.target.store(1000.0f, std::memory_order_relaxed);
        band.q_factor.current.store(0.707f, std::memory_order_relaxed);
        band.q_factor.target.store(0.707f, std::memory_order_relaxed);
        band.gain_db.current.store(0.0f, std::memory_order_relaxed);
        band.gain_db.target.store(0.0f, std::memory_order_relaxed);
        band.center_freq_hz.steps_remaining_.store(0,std::memory_order_relaxed);
        band.q_factor.steps_remaining_.store(0,std::memory_order_relaxed);
        band.gain_db.steps_remaining_.store(0,std::memory_order_relaxed);
        band.filter_type = BandFilterType::PEAKING;
        band.b_coeffs = {1.0f, 0.0f, 0.0f}; 
        band.a_coeffs = {1.0f, 0.0f, 0.0f};
        band.z_state = {0.0f, 0.0f};     
        band.enabled = false;
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
        default: return; // Should not happen
    }
    generate_biquad_coeffs(type_str_c, 
                           band.center_freq_hz.get_current(), 
                           band.q_factor.get_current(), 
                           band.gain_db.get_current(), 
                           sample_rate_, band.b_coeffs, band.a_coeffs);
}


// --- NoiseGateDSP --- (Example of member name corrections)
NoiseGateDSP::NoiseGateDSP(std::string_view n) : DSPNode(n) {
    threshold_db_.current.store(-60.0f, std::memory_order_relaxed); // Corrected member name
    threshold_db_.target.store(-60.0f, std::memory_order_relaxed);  // Corrected member name
    attack_time_ms_.current.store(10.0f, std::memory_order_relaxed);
    attack_time_ms_.target.store(10.0f, std::memory_order_relaxed);
    release_time_ms_.current.store(100.0f, std::memory_order_relaxed);
    release_time_ms_.target.store(100.0f, std::memory_order_relaxed);
    reset();
}

void NoiseGateDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    // ramp_params(); // Called by graph or audio system typically

    float thresh_lin = db_to_linear(threshold_db_.get_current()); // Corrected
    float attack_val_ms = attack_time_ms_.get_current();
    float release_val_ms = release_time_ms_.get_current();

    float attack_alpha = (attack_val_ms > 0.001f && sample_rate_ > 0.0f) ? std::exp(-1.0f / (attack_val_ms * sample_rate_ / 1000.0f)) : 0.0f;
    float release_alpha = (release_val_ms > 0.001f && sample_rate_ > 0.0f) ? std::exp(-1.0f / (release_val_ms * sample_rate_ / 1000.0f)) : 0.0f;

    for (float& sample : buffer) {
        float rectified_input = std::abs(sample);
        if (rectified_input > envelope_) { 
            envelope_ = attack_alpha * envelope_ + (1.0f - attack_alpha) * rectified_input;
        } else { 
            envelope_ = release_alpha * envelope_ + (1.0f - release_alpha) * rectified_input;
        }
        float gain = (envelope_ > thresh_lin) ? 1.0f : 0.0f; 
        sample *= gain;
    }
}

void NoiseGateDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { /* Usage */ return; }
    float thresh = -60.0f, attack = 10.0f, release = 100.0f, ramp = 0.0f;
    int items = std::sscanf(args, "%f %f %f %f", &thresh, &attack, &release, &ramp);

    if (items >= 3 && attack > 0.0f && release > 0.0f) {
        threshold_db_.start(thresh, ramp, sample_rate_); // Corrected
        attack_time_ms_.start(attack, ramp, sample_rate_);
        release_time_ms_.start(release, ramp, sample_rate_);
        if (uart_ops) { /* Success message */ }
    } else if (uart_ops) { /* Error message */ }
}
void NoiseGateDSP::ramp_params() { 
    threshold_db_.next(); // Corrected
    attack_time_ms_.next(); 
    release_time_ms_.next(); 
}
void NoiseGateDSP::reset() { envelope_ = 0.0f; }

// ... Implementations for ALL other DSP nodes declared in dsp.hpp ...
// This requires going through each one and ensuring member names in .cpp match dsp.hpp
// and fixing logic errors, constants, etc.
// Below are stubs or partial fixes for a few more to illustrate:

// --- DelayDSP ---
DelayDSP::DelayDSP(std::string_view name) : DSPNode(name), current_delay_samples_(0), write_index_(0) {
    // delay_buffer_ will be sized in configure or defaults.
    // For now, let's give it a default modest size.
    try {
        delay_buffer_.resize(48000); // Default 1s at 48kHz
    } catch (const std::bad_alloc&) {
        // Handle allocation failure, e.g. by setting an error state or panicking
        if(kernel::g_platform) kernel::g_platform->panic("DelayDSP: Failed to allocate delay buffer", __FILE__, __LINE__);
    }
    reset();
}
void DelayDSP::process(std::span<float> buffer) {
    if (buffer.empty() || delay_buffer_.empty() || current_delay_samples_ == 0) return;
    for (float& sample : buffer) {
        float input_sample = sample; // Store original sample
        // Read from delay line
        size_t read_index = (write_index_ + delay_buffer_.size() - current_delay_samples_) % delay_buffer_.size();
        sample = delay_buffer_[read_index];
        // Write to delay line
        delay_buffer_[write_index_] = input_sample;
        write_index_ = (write_index_ + 1) % delay_buffer_.size();
    }
}
void DelayDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { /* Usage */ return; }
    float delay_ms = 0.0f;
    if (std::sscanf(args, "%f", &delay_ms) == 1 && delay_ms >= 0.0f) {
        size_t new_delay_samples = static_cast<size_t>(delay_ms * sample_rate_ / 1000.0f);
        // Resize buffer if needed, up to a max limit
        size_t max_possible_delay = MAX_DELAY_SAMPLES_CONST; // From hpp
        if (new_delay_samples > max_possible_delay) new_delay_samples = max_possible_delay;
        
        current_delay_samples_ = new_delay_samples;
        if (delay_buffer_.size() < current_delay_samples_ && current_delay_samples_ > 0) {
             try { // Only resize if new delay is larger than current buffer and non-zero
                delay_buffer_.resize(current_delay_samples_); // This can throw std::bad_alloc
             } catch (const std::bad_alloc&) {
                if (uart_ops) uart_ops->puts("Error: Failed to resize delay buffer.\n");
                current_delay_samples_ = delay_buffer_.size(); // Revert to old size if possible
             }
        }
        reset(); // Clear buffer content after resize or delay change
        if (uart_ops) { /* Success message */ }
    } else if (uart_ops) { /* Error message */ }
}
void DelayDSP::reset() {
    write_index_ = 0;
    if (!delay_buffer_.empty()) {
        std::fill(delay_buffer_.begin(), delay_buffer_.end(), 0.0f);
    }
}

// --- ConvolutionReverbDSP ---
ConvolutionReverbDSP::ConvolutionReverbDSP(std::string_view name) 
    : DSPNode(name), history_write_index_(0) {
    wet_dry_mix_.current.store(0.3f, std::memory_order_relaxed);
    wet_dry_mix_.target.store(0.3f, std::memory_order_relaxed);
    generate_default_impulse_response(); // Use the correct name
    reset();
}

void ConvolutionReverbDSP::generate_default_impulse_response() { // Corrected name
    impulse_response_data_.assign(MAX_IR_LENGTH, 0.0f); // Use MAX_IR_LENGTH
    if (MAX_IR_LENGTH == 0) return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-0.5f, 0.5f); // Quieter random noise
    float decay_rate = 5.0f; // Determines how quickly IR decays

    for (size_t i = 0; i < MAX_IR_LENGTH; ++i) {
        float t = static_cast<float>(i) / sample_rate_; // sample_rate_ is a member of DSPNode
        impulse_response_data_[i] = dist(gen) * std::exp(-t * decay_rate);
    }
    // Normalize (optional)
    float max_val = 0.0f;
    for (float x : impulse_response_data_) max_val = std::max(max_val, std::abs(x));
    if (max_val > 1e-6f) {
        for (float& x : impulse_response_data_) x /= max_val;
    }
}
void ConvolutionReverbDSP::process(std::span<float> buffer) {
    if (buffer.empty() || impulse_response_data_.empty()) return;
    // ramp_params(); // Called by graph

    float wet = wet_dry_mix_.get_current();
    float dry = 1.0f - wet;

    if (history_buffer_.size() != impulse_response_data_.size()) {
        history_buffer_.assign(impulse_response_data_.size(), 0.0f); // Match IR size
        history_write_index_ = 0;
    }
    if (history_buffer_.empty()) return; // Should not happen if IR is loaded

    for (float& sample : buffer) {
        float dry_sample = sample;
        history_buffer_[history_write_index_] = sample;
        
        float convolved_sample = 0.0f;
        for (size_t j = 0; j < impulse_response_data_.size(); ++j) {
            size_t history_read_idx = (history_write_index_ + history_buffer_.size() - j) % history_buffer_.size();
            convolved_sample += history_buffer_[history_read_idx] * impulse_response_data_[j];
        }
        sample = dry_sample * dry + convolved_sample * wet;
        history_write_index_ = (history_write_index_ + 1) % history_buffer_.size();
    }
}
void ConvolutionReverbDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops){
    if (!args) { /* Usage */ return; }
    float wd_mix = 0.3f; float ramp = 0.0f;
    if(std::sscanf(args, "%f %f", &wd_mix, &ramp) >= 1 && wd_mix >=0.0f && wd_mix <= 1.0f) {
        wet_dry_mix_.start(wd_mix, ramp, sample_rate_);
        if(uart_ops) { /* Success */ }
    } else if (uart_ops) { /* Error */ }
}
void ConvolutionReverbDSP::ramp_params(){ wet_dry_mix_.next(); }
void ConvolutionReverbDSP::reset(){ 
    if (!history_buffer_.empty()) std::fill(history_buffer_.begin(), history_buffer_.end(), 0.0f);
    history_write_index_ = 0;
}
bool ConvolutionReverbDSP::load_impulse_response(std::span<const float> ir_data) {
    if (ir_data.empty() || ir_data.size() > MAX_IR_LENGTH) return false;
    try {
        impulse_response_data_.assign(ir_data.begin(), ir_data.end());
        history_buffer_.assign(impulse_response_data_.size(), 0.0f);
        history_write_index_ = 0;
    } catch (const std::bad_alloc&) { return false; }
    return true;
}


// --- CrossoverDSP ---
CrossoverDSP::CrossoverDSP(std::string_view name, uint8_t num_ways)
    : DSPNode(name), num_output_ways_(num_ways > 0 && num_ways <= MAX_CROSSOVER_BANDS_OUTPUT ? num_ways : 2),
      selected_band_output_idx_(0) {
    if (num_output_ways_ < 2) num_output_ways_ = 2; // Minimum 2-way
    // Initialize split points (N-1 splits for N ways)
    // Default cutoffs (example for a 3-way)
    if (num_output_ways_ >= 2) splits_[0].cutoff_hz.start(300.0f, 0, sample_rate_);
    if (num_output_ways_ >= 3) splits_[1].cutoff_hz.start(3000.0f, 0, sample_rate_);
    if (num_output_ways_ >= 4) splits_[2].cutoff_hz.start(10000.0f,0, sample_rate_);
    for(size_t i=0; i < num_output_ways_ -1; ++i) {
        update_split_filter_coeffs(i);
    }
    reset();
}
// CrossoverDSP::process is complex due to multiple outputs. Stub for now.
void CrossoverDSP::process(std::span<float> buffer) { (void)buffer; /* TODO */ }
void CrossoverDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops){ (void)args; (void)uart_ops; /* TODO */ }
void CrossoverDSP::ramp_params(){ /* TODO: Ramp split cutoffs */ }
void CrossoverDSP::reset(){ /* TODO: Reset filter states */ }
void CrossoverDSP::update_split_filter_coeffs(size_t split_idx){ (void)split_idx; /* TODO */ }

// --- SRCDSP ---
SRCDSP::SRCDSP(std::string_view n, uint32_t initial_input_rate, uint32_t initial_output_rate)
    : DSPNode(n), input_rate_hz_(initial_input_rate), output_rate_hz_(initial_output_rate) {
    history_buffer_.resize(SRC_FILTER_TAPS, 0.0f);
    fir_filter_coeffs_.resize(SRC_FILTER_TAPS, 0.0f); // Simplified, actual SRC needs polyphase
    regenerate_fir_filter();
    reset();
}
void SRCDSP::process(std::span<float> buffer) { (void)buffer; /* TODO: Complex */ }
void SRCDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) { (void)args; (void)uart_ops; /* TODO */ }
void SRCDSP::reset() { fractional_time_step_ = 0.0f; std::fill(history_buffer_.begin(), history_buffer_.end(), 0.0f); }
void SRCDSP::regenerate_fir_filter() { /* TODO: Implement proper SRC filter design */ }


// --- FIRFilterDSP ---
FIRFilterDSP::FIRFilterDSP(std::string_view n) : DSPNode(n), num_fir_taps_(0), history_write_index_(0) {
    fir_taps_.fill(0.0f);
    input_history_.fill(0.0f);
}
void FIRFilterDSP::process(std::span<float> buffer) {
     if (buffer.empty() || num_fir_taps_ == 0) return;
    for (float& sample : buffer) {
        input_history_[history_write_index_] = sample;
        float output_sample = 0.0f;
        for (size_t i = 0; i < num_fir_taps_; ++i) {
            size_t read_idx = (history_write_index_ + MAX_FIR_TAPS - i) % MAX_FIR_TAPS;
            output_sample += input_history_[read_idx] * fir_taps_[i];
        }
        sample = output_sample;
        history_write_index_ = (history_write_index_ + 1) % MAX_FIR_TAPS;
    }
}
void FIRFilterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops){ (void)args; (void)uart_ops; /* TODO: Parse taps */ }
void FIRFilterDSP::reset(){ history_write_index_ = 0; input_history_.fill(0.0f); }

// --- IIRFilterDSP ---
IIRFilterDSP::IIRFilterDSP(std::string_view n) : DSPNode(n) { reset(); }
void IIRFilterDSP::process(std::span<float> buffer) {
    if (buffer.empty()) return;
    for (float& sample : buffer) {
        float x = sample;
        // Direct Form II Transposed:
        // y[n] = b0*x[n] + z0[n-1]
        // z0[n] = b1*x[n] + z1[n-1] - a1*y[n]
        // z1[n] = b2*x[n] - a2*y[n]
        float y = b_coeffs_[0] * x + z_state_[0];
        z_state_[0] = b_coeffs_[1] * x + z_state_[1] - a_coeffs_[1] * y;
        z_state_[1] = b_coeffs_[2] * x               - a_coeffs_[2] * y;
        sample = y;
    }
}
void IIRFilterDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops){ (void)args; (void)uart_ops; /* TODO: Parse b0,b1,b2,a1,a2 */ }
void IIRFilterDSP::reset(){ z_state_.fill(0.0f); b_coeffs_ = {1,0,0}; a_coeffs_ = {1,0,0}; }


// --- FFTEqualizerDSP ---
FFTEqualizerDSP::FFTEqualizerDSP(std::string_view n) : DSPNode(n), num_active_bands_(0) { reset(); }
void FFTEqualizerDSP::process(std::span<float> buffer) { (void)buffer; /* Placeholder */ }
void FFTEqualizerDSP::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops){ (void)args; (void)uart_ops; /* TODO */ }
void FFTEqualizerDSP::ramp_params(){ /* TODO */ }
void FFTEqualizerDSP::reset(){ num_active_bands_ = 0; /* Reset gains */ }


// --- NetworkAudioSinkSource ---
NetworkAudioSinkSource::NetworkAudioSinkSource(std::string_view name) 
    : DSPNode(name), udp_socket_idx_(-1), is_socket_valid_(false), is_configured_as_sink_(false), remote_target_port_(0), num_audio_channels_(2) {
    remote_target_ip_.addr = 0; // Default to 0.0.0.0
}
// Process and Configure already provided in previous patch for this class, ensure they use kernel::g_net_manager etc.
// Copied and adapted configure from your dsp.cpp
void NetworkAudioSinkSource::configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args) { if (uart_ops) uart_ops->puts("Usage: netaudio <sink|source> <ip> <port> <channels>\n"); return; }
    char mode_str[10], ip_str_val[16];
    int port_val = 0, channels_val = 0;

    if (std::sscanf(args, "%9s %15s %d %d", mode_str, ip_str_val, &port_val, &channels_val) == 4) {
        std::string_view mode_sv(mode_str);
        if (mode_sv == "sink") is_configured_as_sink_ = true;
        else if (mode_sv == "source") is_configured_as_sink_ = false;
        else { if (uart_ops) uart_ops->puts("Invalid mode (sink/source).\n"); return; }

        if (!kernel::util::ipv4_to_uint32(ip_str_val, remote_target_ip_.addr)) {
            if (uart_ops) uart_ops->puts("Invalid IP address string.\n"); return;
        }
        if (port_val <= 0 || port_val > 65535) { if (uart_ops) uart_ops->puts("Invalid port number.\n"); return; }
        remote_target_port_ = static_cast<uint16_t>(port_val);
        if (channels_val <=0 || channels_val > MAX_AUDIO_CHANNELS) { if (uart_ops) uart_ops->puts("Invalid channel count.\n"); return; }
        num_audio_channels_ = static_cast<uint8_t>(channels_val);

        if (is_socket_valid_) {
            kernel::g_net_manager.close_socket(udp_socket_idx_);
            is_socket_valid_ = false;
        }
        // For sink, local port can be ephemeral. For source, it's the listen port.
        uint16_t local_bind_port = is_configured_as_sink_ ? 0 : remote_target_port_; 
        udp_socket_idx_ = kernel::g_net_manager.create_udp_socket(net::IPv4Addr{0}, local_bind_port); 
        is_socket_valid_ = (udp_socket_idx_ >= 0);

        if (is_socket_valid_) { if (uart_ops) { /* Success msg */ } }
        else { if (uart_ops) uart_ops->puts("Failed to create UDP socket for netaudio.\n"); }
    } else if (uart_ops) { uart_ops->puts("Invalid netaudio args format.\n"); }
}
// Process method needs to be here. Simplified from previous patch.
void NetworkAudioSinkSource::process(std::span<float> buffer) {
    if (!is_socket_valid_ || buffer.empty() || !kernel::g_platform || !kernel::g_platform->get_timer_ops()) return;

    if (is_configured_as_sink_) {
        net::Packet packet{};
        packet.dst_ip = remote_target_ip_;
        packet.dst_port = remote_target_port_;
        packet.audio_channels = num_audio_channels_;
        size_t samples_to_send = std::min(buffer.size(), static_cast<size_t>(net::MAX_PAYLOAD / sizeof(float)));
        size_t bytes_to_send = samples_to_send * sizeof(float);
        
        kernel::util::memcpy(packet.data.data(), buffer.data(), bytes_to_send);
        packet.data_len = bytes_to_send;
        packet.timestamp_us = kernel::g_platform->get_timer_ops()->get_system_time_us();
        
        kernel::g_net_manager.send(udp_socket_idx_, packet, false); 
    } else { // Source
        net::Packet packet{};
        if (kernel::g_net_manager.receive(udp_socket_idx_, packet, false) && packet.audio_channels == num_audio_channels_ && packet.data_len > 0) {
            size_t samples_received = packet.data_len / sizeof(float);
            size_t samples_to_copy = std::min(samples_received, buffer.size());
            kernel::util::memcpy(buffer.data(), packet.data.data(), samples_to_copy * sizeof(float));
            if (samples_to_copy < buffer.size()) { // Silence rest of buffer
                std::fill(buffer.begin() + samples_to_copy, buffer.end(), 0.0f);
            }
        } else {
            std::fill(buffer.begin(), buffer.end(), 0.0f); // No packet, output silence
        }
    }
}


// --- DSPGraph ---
DSPGraph::DSPGraph(std::string_view graph_name) : name_storage_(graph_name) {}

bool DSPGraph::add_node(std::unique_ptr<DSPNode> node) { // Takes unique_ptr
    if (!node) return false;
    kernel::ScopedLock lock(graph_modification_lock_);
    for (const auto& n_ptr : dsp_nodes_) {
        if (n_ptr->get_name() == node->get_name()) {
            return false; 
        }
    }
    dsp_nodes_.push_back(std::move(node));
    return true;
}
// The dsp.cpp had "bool DSPGraph::add_node(DSPNode* node)". This needs to match hpp.
// The unique_ptr version from hpp is better. Assuming CLI uses unique_ptr.

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
    kernel::ScopedLock lock(graph_modification_lock_); // Or a read lock if available
    for (const auto& n_ptr : dsp_nodes_) {
        if (n_ptr->get_name() == name) {
            return n_ptr.get();
        }
    }
    return nullptr;
}

bool DSPGraph::configure_node(std::string_view name, const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    DSPNode* node = get_node(name); // get_node handles its own locking
    if (node) {
        node->configure(args, uart_ops);
        return true;
    }
    if (uart_ops) {
        std::string temp_name(name); 
        uart_ops->puts("Node '");
        uart_ops->puts(temp_name.c_str());
        uart_ops->puts("' not found for configuration.\n");
    }
    return false;
}

void DSPGraph::process(std::span<float> audio_buffer) {
    if (audio_buffer.empty()) return;
    // For read-only iteration, locking might be optional if additions/removals are rare and handled carefully.
    // However, if configure_node can change node state concurrently, this needs thought.
    // Assuming process is called from a single audio thread, and modifications are from another (CLI).
    // graph_modification_lock_ taken by add/remove/get_node (for configure) should suffice.
    for (auto& node_ptr : dsp_nodes_) { // No lock here for performance, assuming graph structure is stable during process.
        node_ptr->ramp_params(); // Update ramps first
        node_ptr->process(audio_buffer); 
    }
}

void DSPGraph::reset() {
    kernel::ScopedLock lock(graph_modification_lock_); // Ensure graph not modified during reset
    for (auto& node_ptr : dsp_nodes_) {
        node_ptr->reset();
    }
}


} // namespace dsp
} // namespace kernel