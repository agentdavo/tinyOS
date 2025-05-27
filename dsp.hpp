// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.hpp
 * @brief Digital Signal Processing (DSP) subsystem header for miniOS v1.7.
 * @details
 * Defines a modular DSP framework for real-time audio processing, supporting professional audio nodes:
 * gain, mixer, parametric EQ, limiter, noise gate, stereo width, waveshaper, envelope follower,
 * compressor, delay, reverb, chorus, flanger, pitch shifter, convolution reverb, crossover (up to
 * 4-way with Butterworth, Bessel, Linkwitz-Riley filters), phase correction, sample rate converter,
 * FIR/IIR filters, FFT equalizer, and network audio sink/source. Updated in v1.7 with improved error
 * handling, clearer documentation, and modern C++20 practices, retaining all v1.6 functionality
 * including 4-way crossover with 6/12/18/24 dB/octave slopes and high/low shelf filters.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::string_view for string processing
 * - std::atomic for thread-safe updates
 *
 * @version 1.7
 * @see dsp.cpp, miniOS.hpp, audio.hpp, util.hpp, net.hpp
 */

#ifndef DSP_HPP
#define DSP_HPP

#include "miniOS.hpp" // Includes kernel types, kernel::hal::UARTDriverOps
#include "net.hpp"    // For net::IPv4Addr
#include <span>
#include <vector>
#include <array>
#include <string_view>
#include <atomic>
#include <memory>
#include <cmath>

namespace kernel {
namespace dsp {

/**
 * @brief Converts dB to linear gain.
 * @param db Decibel value
 * @return Linear gain (0.0 to inf)
 */
inline float db_to_linear(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

/**
 * @brief Converts linear gain to dB.
 * @param lin Linear value
 * @return Decibel value (-inf to inf)
 */
inline float linear_to_db(float gain) noexcept { return 20.0f * std::log10(std::max(gain, 1e-6f)); }

/**
 * @brief Clips a value to a specified range.
 * @param x Input value
 * @param min Minimum value
 * @param max Maximum value
 * @return Clipped value
 */
inline float clip(float x, float min_val, float max_val) noexcept { return std::max(min_val, std::min(max_val, x)); }


/**
 * @brief Generates Hann window coefficients.
 * @param window Output buffer for window coefficients
 */
void generate_hann_window(std::span<float> window) noexcept;

/**
 * @brief Generates biquad filter coefficients.
 * @param type Filter type (lowpass, highpass, bandpass, notch, peaking, lowshelf, highshelf)
 * @param fc Center/cutoff frequency (Hz)
 * @param q Quality factor
 * @param gain_db Gain for peaking/shelf (dB)
 * @param sample_rate Sample rate (Hz)
 * @param b Output feedforward coefficients (b0, b1, b2)
 * @param a Output feedback coefficients (a0=1, a1, a2)
 */
void generate_biquad_coeffs(std::string_view type, float fc, float q, float gain_db, float sample_rate,
                           std::array<float, 3>& b, std::array<float, 3>& a) noexcept;

/**
 * @brief Generates crossover filter coefficients.
 * @param type Filter type (butterworth, bessel, linkwitz)
 * @param order Filter order (1 to 4, for 6 to 24 dB/octave)
 * @param fc Cutoff frequency (Hz)
 * @param sample_rate Sample rate (Hz)
 * @param b Output feedforward coefficients
 * @param a Output feedback coefficients
 * @param is_highpass True for highpass, false for lowpass
 */
void generate_crossover_coeffs(std::string_view type, int order, float fc, float sample_rate,
                              std::vector<std::array<float, 3>>& b, std::vector<std::array<float, 3>>& a,
                              bool is_highpass) noexcept;

/**
 * @brief Parameter ramping for smooth transitions.
 */
struct ParamRamp {
    float current = 0.0f;
    float target = 0.0f;
    float step = 0.0f;

    void start(float to, float ms, float sample_rate) noexcept {
        target = to;
        if (ms <= 0.0f || sample_rate <= 0.0f) { // Instant or invalid
            current = target;
            step = 0.0f;
            return;
        }
        float samples = ms * sample_rate / 1000.0f;
        step = samples > 1.0f ? (to - current) / samples : (to - current); // Avoid division by zero or very small numbers
        if (std::abs(step) < 1e-9f && to != current) { // If step is too small but target isn't current, set instantly
             current = target; // or step = (to - current) to make at least one step
             step = 0.0f;
        }
    }

    float next() noexcept {
        if (step == 0.0f) { // Not ramping or already reached target
             current = target; // Ensure current is exactly target if not ramping
             return current;
        }
        if ( (step > 0.0f && current >= target - step * 0.5f) || // Check if close enough to target
             (step < 0.0f && current <= target - step * 0.5f) ||
             std::abs(current - target) < std::abs(step * 0.5f) ) { // handles very small steps
            current = target;
            step = 0.0f;
        } else {
            current += step;
        }
        return current;
    }

    bool active() const noexcept { return step != 0.0f || current != target; }
};

/**
 * @brief Base class for DSP nodes.
 */
class DSPNode {
public:
    explicit DSPNode(std::string_view n);
    virtual ~DSPNode() = default;
    virtual void process(std::span<float> buffer) = 0;
    virtual void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) = 0;
    virtual void ramp_params() {} // Process parameter ramping
    virtual void reset() {} // Reset internal state
    std::string_view name() const noexcept { return name_; }

protected:
    std::string name_; // Using std::string for easier management if names are dynamic
};

/**
 * @brief Gain node.
 */
class GainDSP : public DSPNode {
public:
    GainDSP(float g, std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    ParamRamp gain_;
};

/**
 * @brief Mixer node.
 */
class MixerDSP : public DSPNode {
public:
    MixerDSP(std::string_view n, uint8_t inputs = 2);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    uint8_t input_count_;
    std::vector<ParamRamp> gains_;
};

/**
 * @brief Parametric equalizer node.
 */
class ParametricEQDSP : public DSPNode {
public:
    static constexpr size_t MAX_BANDS = 4;
    enum class BandType { PEAK, LOWSHELF, HIGHSHELF };

    struct Band {
        ParamRamp freq;
        ParamRamp q;
        ParamRamp gain_db;
        BandType type = BandType::PEAK;
        std::array<float, 3> b = {1.0f, 0.0f, 0.0f};
        std::array<float, 3> a = {1.0f, 0.0f, 0.0f}; // a[0] is always 1.0 after normalization
        std::array<float, 2> z = {0.0f, 0.0f}; // state variables z_n-1, z_n-2
    };

    ParametricEQDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    std::array<Band, MAX_BANDS> bands_;
    size_t band_count_ = 0;
    float sample_rate_ = 48000.0f; // TODO: Get from AudioSystem or config
    void update_band_coeffs(size_t band_idx);
};

/**
 * @brief Limiter node.
 */
class LimiterDSP : public DSPNode {
public:
    LimiterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    ParamRamp threshold_; // Linear threshold 0.0 to 1.0
};

/**
 * @brief Noise gate node.
 */
class NoiseGateDSP : public DSPNode {
public:
    NoiseGateDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp threshold_db_; // Threshold in dBFS
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Stereo width/balance node.
 */
class StereoWidthDSP : public DSPNode {
public:
    StereoWidthDSP(std::string_view n);
    void process(std::span<float> buffer) override; // Expects stereo interleaved buffer
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    ParamRamp width_;   // 0.0 (mono) to 2.0 (extra wide)
    ParamRamp balance_; // -1.0 (full left) to 1.0 (full right)
};

/**
 * @brief Waveshaper node.
 */
class WaveshaperDSP : public DSPNode {
public:
    WaveshaperDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    ParamRamp drive_; // Input gain
    ParamRamp bias_;  // DC offset before shaping
    enum class ShapeType { TANH, HARD_CLIP, SOFT_CLIP, ATAN } shape_ = ShapeType::TANH;
};

/**
 * @brief Envelope follower node.
 */
class EnvelopeFollowerDSP : public DSPNode {
public:
    EnvelopeFollowerDSP(std::string_view n);
    void process(std::span<float> buffer) override; // Output is not written to buffer, call get_envelope()
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
    float get_envelope() const noexcept { return envelope_; }

private:
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f; // TODO: Get from config
    bool use_rms_ = false;
};

/**
 * @brief Compressor node.
 */
class CompressorDSP : public DSPNode {
public:
    CompressorDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp threshold_db_;
    ParamRamp ratio_;      // e.g., 2.0 for 2:1
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    ParamRamp makeup_db_;  // Makeup gain
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Delay node.
 */
class DelayDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 96000; // ~2 seconds at 48kHz
    DelayDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    size_t delay_samples_ = 0;
    size_t write_index_ = 0;
    std::array<float, MAX_DELAY_SAMPLES> delay_buffer_;
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Reverb node (simplified Schroeder-style).
 */
class ReverbDSP : public DSPNode {
public:
    static constexpr size_t NUM_COMBS = 8;
    static constexpr size_t NUM_ALLPASS = 4;
    ReverbDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp pre_delay_ms_;
    ParamRamp room_size_;  // 0.0 to 1.0 (influences comb filter feedback/decay)
    ParamRamp damping_;    // 0.0 to 1.0 (high-frequency damping in feedback)
    ParamRamp wet_dry_mix_;// 0.0 (dry) to 1.0 (wet)

    std::array<std::vector<float>, NUM_COMBS> comb_buffers_;
    std::array<size_t, NUM_COMBS> comb_indices_;
    std::array<float, NUM_COMBS> comb_feedback_; // Calculated from room_size
    std::array<float, NUM_COMBS> comb_damping_factor_; // Calculated from damping

    std::array<std::vector<float>, NUM_ALLPASS> allpass_buffers_;
    std::array<size_t, NUM_ALLPASS> allpass_indices_;
    static constexpr float ALLPASS_GAIN = 0.5f; // Fixed gain for allpass

    std::vector<float> pre_delay_buffer_;
    size_t pre_delay_write_index_ = 0;
    size_t current_pre_delay_samples_ = 0;

    float sample_rate_ = 48000.0f; // TODO: Get from config
    void update_internal_params();
};

/**
 * @brief Chorus node.
 */
class ChorusDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 2400; // ~50ms at 48kHz
    ChorusDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp depth_ms_;  // Modulation depth
    ParamRamp rate_hz_;   // LFO rate
    ParamRamp mix_;       // Wet/dry mix 0.0 to 1.0
    std::array<float, MAX_DELAY_SAMPLES> delay_buffer_;
    size_t write_index_ = 0;
    float lfo_phase_ = 0.0f;
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Flanger node.
 */
class FlangerDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 480; // ~10ms at 48kHz
    FlangerDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp depth_ms_;
    ParamRamp rate_hz_;
    ParamRamp feedback_; // -0.99 to 0.99
    ParamRamp mix_;
    std::array<float, MAX_DELAY_SAMPLES> delay_buffer_;
    size_t write_index_ = 0;
    float lfo_phase_ = 0.0f;
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Pitch shifter node (simple delay-line based, for demonstration).
 */
class PitchShifterDSP : public DSPNode {
public:
    static constexpr size_t WINDOW_SIZE = 1024; // For granular synthesis / PSOLA-like approach
    PitchShifterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp pitch_factor_; // e.g., 1.0 no change, 0.5 octave down, 2.0 octave up
    std::array<float, WINDOW_SIZE * 2> grain_buffer_; // Buffer for input grains
    std::array<float, WINDOW_SIZE> hann_window_;
    size_t input_write_pos_ = 0;
    float output_read_pos_ = 0.0f; // Fractional position for resampling
    float sample_rate_ = 48000.0f; // TODO: Get from config
};

/**
 * @brief Convolution reverb node.
 */
class ConvolutionReverbDSP : public DSPNode {
public:
    // IR_LENGTH should be a power of 2 for efficient FFT if used.
    // For direct convolution, any length is fine. Max practical length depends on performance.
    static constexpr size_t MAX_IR_LENGTH = 4096; // ~85ms at 48kHz
    ConvolutionReverbDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override; // Args might load IR from FS
    void ramp_params() override;
    void reset() override;
    bool load_ir(std::span<const float> ir_data); // Method to load impulse response

private:
    ParamRamp wet_dry_mix_;
    std::vector<float> impulse_response_;
    std::vector<float> history_buffer_; // Stores past input samples for convolution
    size_t history_write_index_ = 0;
    // float sample_rate_ = 48000.0f; // TODO: Get from config (if IR needs resampling)
    void generate_default_ir(); // Fallback if no IR is loaded
};

/**
 * @brief Crossover node for multi-way speaker systems.
 */
class CrossoverDSP : public DSPNode {
public:
    static constexpr size_t MAX_CROSSOVER_BANDS = 4;
    enum class FilterType { BUTTERWORTH, BESSEL, LINKWITZ_RILEY };
    // Shape is implicitly determined by low/high pass sections of crossover bands
    // For shelf, use ParametricEQDSP.

    struct FilterStage { // Represents one biquad section
        std::array<float, 3> b = {1.0f, 0.0f, 0.0f}; ///< Feedforward coeffs (b0, b1, b2)
        std::array<float, 3> a = {1.0f, 0.0f, 0.0f}; ///< Feedback coeffs (a0=1, a1, a2)
        std::array<float, 2> z = {0.0f, 0.0f}; ///< State variables (z_n-1, z_n-2)
    };

    struct BandSplit {
        ParamRamp cutoff_hz; ///< Cutoff frequency for this split point
        FilterType type = FilterType::LINKWITZ_RILEY;
        int order = 2; ///< Filter order (1=6dB, 2=12dB, 3=18dB, 4=24dB LR is 2nd order sections)
        
        std::vector<FilterStage> lowpass_stages;
        std::vector<FilterStage> highpass_stages;
    };

    CrossoverDSP(std::string_view n, uint8_t num_ways = 2); // e.g. 2-way, 3-way, 4-way
    // Process takes a single input buffer and outputs multiple buffers (e.g., via a callback or member spans)
    // This is a change from single buffer in/out.
    // For simplicity here, let's assume it outputs to separate parts of a larger buffer or is handled externally.
    // Or, process one band at a time if the graph structure allows.
    // For now, let's assume 'buffer' is for one band, and we need a way to select which band.
    // This design is complex for a generic DSPNode.
    // A common approach is multiple output pins for the node.
    // Let's simplify: process() modifies the input buffer with ONE band's output. User configures which one.
    void process(std::span<float> buffer) override; // Output buffer for the selected_band_output_
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    uint8_t num_ways_; // 2, 3, or 4
    std::array<BandSplit, MAX_CROSSOVER_BANDS - 1> splits_; // N-way crossover has N-1 split points
    // std::vector<std::span<float>> output_buffers_; // Needs to be set up by graph
    uint8_t selected_band_output_ = 0; // Which band's output to write to the process buffer (0 to num_ways-1)

    float sample_rate_ = 48000.0f; // TODO: Get from config
    void update_split_coeffs(size_t split_idx);
    // Internal buffer for processing intermediate signals if needed
    std::vector<float> internal_buffer_;
};

/**
 * @brief Phase correction node (All-pass filter).
 */
class PhaseCorrectionDSP : public DSPNode {
public:
    // MAX_DELAY_SAMPLES not strictly needed for simple allpass, but useful if implementing FIR allpass
    PhaseCorrectionDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp phase_deg_at_freq_; // Target phase shift in degrees
    ParamRamp target_freq_hz_;    // Frequency at which phase_deg is targeted
    // For a simple first-order all-pass: y[n] = a*x[n] + x[n-1] - a*y[n-1]
    // Or: y[n] = ( (1-a)/(1+a) ) * (x[n] + x[n-1]) - ( (1-a)/(1+a) ) * y[n-1] where a = tan(PI*fc/fs)
    float coeff_a1_ = 0.0f; // All-pass filter coefficient
    float z1_x_ = 0.0f; // x[n-1]
    float z1_y_ = 0.0f; // y[n-1]
    float sample_rate_ = 48000.0f; // TODO: Get from config
    void update_coeffs();
};

/**
 * @brief Sample rate converter node (simple linear interpolation or basic FIR).
 */
class SRCDSP : public DSPNode {
public:
    static constexpr size_t FILTER_TAPS = 32; // For FIR-based SRC
    SRCDSP(std::string_view n, uint32_t in_rate = 48000, uint32_t out_rate = 44100);
    // Process needs to handle variable output size. This is tricky for fixed buffer span.
    // Assume output buffer is large enough, or use a callback for output.
    // For simplicity, let's assume buffer is modified in-place if output samples <= input samples,
    // or it's a placeholder for a more complex mechanism.
    void process(std::span<float> buffer) override; // Output might be shorter or longer
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    uint32_t input_rate_, output_rate_;
    float current_ratio_ = 1.0f;
    float fractional_pos_ = 0.0f; // For tracking sub-sample position
    std::vector<float> history_buffer_; // Stores previous input samples for interpolation/filtering
    std::vector<float> fir_coeffs_;     // For FIR-based SRC
    // float sample_rate_ = 48000.0f; // This is input_rate_
    void generate_fir_coeffs();
};

/**
 * @brief FIR filter node.
 */
class FIRFilterDSP : public DSPNode {
public:
    static constexpr size_t MAX_TAPS = 64;
    FIRFilterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override; // Args provide tap coefficients
    void reset() override;

private:
    std::array<float, MAX_TAPS> taps_;
    size_t num_taps_ = 0;
    std::array<float, MAX_TAPS> history_; // Input sample history
    size_t history_write_index_ = 0;
};

/**
 * @brief IIR filter node (Direct Form II Transposed biquad).
 */
class IIRFilterDSP : public DSPNode {
public:
    // For a single biquad section
    static constexpr size_t NUM_COEFFS_B = 3; // b0, b1, b2
    static constexpr size_t NUM_COEFFS_A = 3; // a0 (usually 1), a1, a2
    IIRFilterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override; // Args provide biquad coeffs
    void reset() override;

private:
    std::array<float, NUM_COEFFS_B> b_coeffs_; // Feedforward
    std::array<float, NUM_COEFFS_A> a_coeffs_; // Feedback (a0 is implicitly 1)
    std::array<float, 2> z_state_; // State variables (w[n-1], w[n-2])
};

/**
 * @brief FFT equalizer node (conceptual, actual FFT not implemented here for simplicity).
 */
class FFTEqualizerDSP : public DSPNode {
public:
    static constexpr size_t MAX_FFT_BANDS = 8; // Number of frequency bands to control
    FFTEqualizerDSP(std::string_view n);
    void process(std::span<float> buffer) override; // Placeholder processing
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    std::array<ParamRamp, MAX_FFT_BANDS> band_gains_db_; // Gain for each band in dB
    size_t num_bands_ = 0;
    // Actual FFT implementation would require FFT/IFFT routines and windowing/overlap-add.
};

/**
 * @brief Network audio sink/source node.
 */
class NetworkAudioSinkSource : public DSPNode {
public:
    NetworkAudioSinkSource(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;

private:
    int socket_idx_ = -1;
    bool socket_idx_valid_ = false;
    bool is_sink_ = false; // true for sink (send), false for source (receive)
    net::IPv4Addr remote_ip_;
    uint16_t remote_port_ = 0;
    uint8_t channels_ = 2; // Number of audio channels expected/sent
};

/**
 * @brief DSP processing graph.
 */
class DSPGraph {
public:
    explicit DSPGraph(std::string_view name);
    bool add_node(std::unique_ptr<DSPNode> node); // Take ownership
    bool remove_node(std::string_view name);
    DSPNode* get_node(std::string_view name);
    bool configure_node(std::string_view name, const char* args, kernel::hal::UARTDriverOps* uart_ops);
    void process(std::span<float> buffer); // Process through all nodes in order
    void reset(); // Reset all nodes

private:
    std::string name_;
    std::vector<std::unique_ptr<DSPNode>> nodes_;
    // Could add routing logic here for complex graphs
};

} // namespace dsp
} // namespace kernel

#endif // DSP_HPP