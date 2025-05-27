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

#include "miniOS.hpp"
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
inline float clip(float x, float min, float max) noexcept { return std::max(min, std::min(max, x)); }

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
        float samples = ms * sample_rate / 1000.0f;
        step = samples > 1.0f ? (to - current) / samples : (to - current);
    }

    float next() noexcept {
        if (std::abs(current - target) < std::abs(step)) {
            current = target;
            step = 0.0f;
        } else {
            current += step;
        }
        return current;
    }

    bool active() const noexcept { return step != 0.0f; }
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
    virtual void ramp_params() {}
    virtual void reset() {}
    std::string_view name() const noexcept { return name_; }

protected:
    std::string name_;
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
        std::array<float, 3> a = {1.0f, 0.0f, 0.0f};
        std::array<float, 2> z = {0.0f, 0.0f};
    };

    ParametricEQDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    std::array<Band, MAX_BANDS> bands_;
    size_t band_count_ = 0;
    float sample_rate_ = 48000.0f;
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
    ParamRamp threshold_;
};

/**
 * @brief Noise gate node.
 */
class NoiseGateDSP : public DSPNode {
public:
    NoiseGateDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp threshold_;
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Stereo width/balance node.
 */
class StereoWidthDSP : public DSPNode {
public:
    StereoWidthDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    ParamRamp width_;
    ParamRamp balance_;
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
    ParamRamp drive_;
    ParamRamp bias_;
    enum class ShapeType { TANH, HARD_CLIP, SOFT_CLIP } shape_ = ShapeType::TANH;
};

/**
 * @brief Envelope follower node.
 */
class EnvelopeFollowerDSP : public DSPNode {
public:
    EnvelopeFollowerDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
    float get_envelope() const noexcept { return envelope_; }

private:
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f;
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
    ParamRamp ratio_;
    ParamRamp attack_ms_;
    ParamRamp release_ms_;
    ParamRamp makeup_db_;
    float envelope_ = 0.0f;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Delay node.
 */
class DelayDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 96000;
    DelayDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    size_t delay_samples_ = 0;
    size_t index_ = 0;
    std::array<float, MAX_DELAY_SAMPLES> buffer_;
};

/**
 * @brief Reverb node.
 */
class ReverbDSP : public DSPNode {
public:
    static constexpr size_t NUM_COMBS = 8;
    static constexpr size_t NUM_ALLPASS = 4;
    ReverbDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp pre_delay_ms_;
    ParamRamp room_size_;
    ParamRamp damping_;
    ParamRamp wet_dry_;
    std::array<std::vector<float>, NUM_COMBS> comb_buffers_;
    std::array<size_t, NUM_COMBS> comb_indices_;
    std::array<std::vector<float>, NUM_ALLPASS> allpass_buffers_;
    std::array<size_t, NUM_ALLPASS> allpass_indices_;
    std::vector<float> pre_delay_buffer_;
    size_t pre_delay_index_ = 0;
    size_t pre_delay_samples_ = 0;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Chorus node.
 */
class ChorusDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 2400;
    ChorusDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp depth_ms_;
    ParamRamp rate_hz_;
    ParamRamp mix_;
    std::array<float, MAX_DELAY_SAMPLES> buffer_;
    size_t index_ = 0;
    float phase_ = 0.0f;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Flanger node.
 */
class FlangerDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 480;
    FlangerDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp depth_ms_;
    ParamRamp rate_hz_;
    ParamRamp feedback_;
    ParamRamp mix_;
    std::array<float, MAX_DELAY_SAMPLES> buffer_;
    size_t index_ = 0;
    float phase_ = 0.0f;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Pitch shifter node.
 */
class PitchShifterDSP : public DSPNode {
public:
    static constexpr size_t WINDOW_SIZE = 1024;
    PitchShifterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp pitch_factor_;
    std::array<float, WINDOW_SIZE * 2> input_buffer_;
    std::array<float, WINDOW_SIZE * 2> output_buffer_;
    size_t input_index_ = 0;
    size_t output_index_ = 0;
    float phase_ = 0.0f;
    float sample_rate_ = 48000.0f;
};

/**
 * @brief Convolution reverb node.
 */
class ConvolutionReverbDSP : public DSPNode {
public:
    static constexpr size_t IR_LENGTH = 4096;
    ConvolutionReverbDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    ParamRamp wet_dry_;
    std::array<float, IR_LENGTH> impulse_response_;
    std::array<float, IR_LENGTH> history_;
    size_t history_index_ = 0;
    float sample_rate_ = 48000.0f;
    void generate_impulse_response();
};

/**
 * @brief Crossover node for multi-way speaker systems.
 */
class CrossoverDSP : public DSPNode {
public:
    static constexpr size_t MAX_BANDS = 4;
    enum class FilterType { BUTTERWORTH, BESSEL, LINKWITZ_RILEY };
    enum class FilterShape { LOWPASS, HIGHPASS, LOWSHELF, HIGHSHELF };

    struct FilterStage {
        std::array<float, 3> b = {1.0f, 0.0f, 0.0f}; ///< Feedforward coeffs
        std::array<float, 3> a = {1.0f, 0.0f, 0.0f}; ///< Feedback coeffs
        std::array<float, 2> z = {0.0f, 0.0f}; ///< State
    };

    struct Band {
        ParamRamp cutoff_hz; ///< Cutoff frequency (Hz)
        ParamRamp gain_db; ///< Gain for shelf filters (dB)
        FilterType type = FilterType::BUTTERWORTH;
        FilterShape shape = FilterShape::LOWPASS;
        int order = 2; ///< 1 to 4 (6 to 24 dB/octave)
        std::vector<FilterStage> stages; ///< Biquad stages
    };

    CrossoverDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    std::array<Band, MAX_BANDS> bands_;
    size_t band_count_ = 2;
    float sample_rate_ = 48000.0f;
    void update_band_coeffs(size_t band_idx);
};

/**
 * @brief Phase correction node.
 */
class PhaseCorrectionDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES = 4800;
    PhaseCorrectionDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;

private:
    ParamRamp phase_deg_;
    std::array<float, MAX_DELAY_SAMPLES> buffer_;
    size_t index_ = 0;
    float a1_ = 0.0f;
    void update_coeff();
};

/**
 * @brief Sample rate converter node.
 */
class SRCDSP : public DSPNode {
public:
    static constexpr size_t FILTER_TAPS = 32;
    SRCDSP(std::string_view n, uint32_t in_rate = 48000, uint32_t out_rate = 44100);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    uint32_t in_rate_, out_rate_;
    float phase_ = 0.0f;
    std::vector<float> history_;
    std::vector<float> filter_coeffs_;
    float sample_rate_ = 48000.0f;
    void generate_filter();
};

/**
 * @brief FIR filter node.
 */
class FIRFilterDSP : public DSPNode {
public:
    static constexpr size_t MAX_TAPS = 64;
    FIRFilterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    std::array<float, MAX_TAPS> taps_;
    size_t tap_count_ = 0;
    std::array<float, MAX_TAPS> history_;
    size_t history_index_ = 0;
};

/**
 * @brief IIR filter node.
 */
class IIRFilterDSP : public DSPNode {
public:
    static constexpr size_t MAX_COEFFS = 3;
    IIRFilterDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;

private:
    std::array<float, MAX_COEFFS> b_;
    std::array<float, MAX_COEFFS> a_;
    std::array<float, 2> z_;
};

/**
 * @brief FFT equalizer node.
 */
class FFTEqualizerDSP : public DSPNode {
public:
    static constexpr size_t MAX_BANDS = 8;
    FFTEqualizerDSP(std::string_view n);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;

private:
    std::array<ParamRamp, MAX_BANDS> band_gains_;
    size_t band_count_ = 0;
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
    bool is_sink_ = false;
    net::IPv4Addr remote_ip_;
    uint16_t remote_port_ = 0;
    uint8_t channels_ = 2;
};

/**
 * @brief DSP processing graph.
 */
class DSPGraph {
public:
    explicit DSPGraph(std::string_view name);
    bool add_node(DSPNode* node);
    bool remove_node(std::string_view name);
    bool configure_node(std::string_view name, const char* args, kernel::hal::UARTDriverOps* uart_ops);
    void process(std::span<float> buffer);
    void reset();

private:
    std::string name_;
    std::vector<std::unique_ptr<DSPNode>> nodes_;
};

} // namespace dsp

} // namespace kernel

#endif // DSP_HPP