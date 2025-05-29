// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.hpp
 * @brief Digital Signal Processing (DSP) subsystem header for miniOS v1.7.
 * @details
 * Defines a comprehensive set of audio processing nodes and the DSP graph for
 * real-time audio processing. Supports various DSP effects and network audio streaming.
 * Updated in v1.7 with improved error handling, clearer documentation, and modern
 * C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::string_view for string operations
 * - std::unique_ptr for resource management
 *
 * @version 1.7
 * @see dsp.cpp, core.hpp, hal.hpp
 */

#ifndef DSP_HPP
#define DSP_HPP

#include "core.hpp"
#include "hal.hpp"
#include <string_view>
#include <memory>
#include <vector>
#include <array>
#include <span>
#include <cstdint>
#include <cmath>

namespace dsp {

constexpr size_t MAX_EQ_BANDS = 4;

inline float linear_to_db(float linear) noexcept {
    return linear > 0.0f ? 20.0f * std::log10(linear) : -70.0f;
}

inline float db_to_linear(float db) noexcept {
    return std::pow(10.0f, db / 20.0f);
}

inline float clip(float value, float min_val, float max_val) noexcept {
    return kernel::util::max(min_val, kernel::util::min(max_val, value));
}

struct FilterStage {
    std::array<float, 3> b_coeffs{1.0f, 0.0f, 0.0f};
    std::array<float, 3> a_coeffs{1.0f, 0.0f, 0.0f};
};

void generate_hann_window(std::span<float> window) noexcept;

void generate_biquad_coeffs(std::string_view type, float fc_hz, float q_factor, float gain_db, float sample_rate_hz,
                           std::array<float, 3>& b_coeffs, std::array<float, 3>& a_coeffs) noexcept;

void generate_crossover_coeffs(std::string_view type, int order, float fc_hz, float sample_rate_hz,
                              std::vector<FilterStage>& stages, bool is_highpass_section) noexcept;

template<typename T>
class LinearRamp {
    T target_ = T{0};
    T current_ = T{0};
    T step_ = T{0};
    size_t steps_remaining_ = 0;
    float sample_rate_hz_ = 0.0f;
public:
    void start(T target, float ramp_ms, float sample_rate_hz) noexcept {
        target_ = target;
        sample_rate_hz_ = sample_rate_hz;
        if (ramp_ms <= 0.0f || sample_rate_hz <= 0.0f) {
            current_ = target_;
            steps_remaining_ = 0;
            step_ = T{0};
            return;
        }
        steps_remaining_ = static_cast<size_t>((ramp_ms / 1000.0f) * sample_rate_hz);
        if (steps_remaining_ == 0) {
            current_ = target_;
            step_ = T{0};
            return;
        }
        step_ = (target_ - current_) / static_cast<T>(steps_remaining_);
    }
    bool is_active() const noexcept { return steps_remaining_ > 0; }
    T get_current() const noexcept { return current_; }
    void next() noexcept {
        if (steps_remaining_ == 0) return;
        current_ += step_;
        steps_remaining_--;
        if (steps_remaining_ == 0) current_ = target_;
    }
};

class DSPNode {
protected:
    explicit DSPNode(std::string_view node_name);
public:
    virtual ~DSPNode() = default;
    virtual void process(std::span<float> buffer) = 0;
    virtual void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) = 0;
    virtual void ramp_params() {}
    virtual void reset() {}
    std::string_view get_name() const noexcept { return name_storage_; }
    void set_sample_rate(float sample_rate_hz) noexcept { sample_rate_ = sample_rate_hz; }
protected:
    std::string_view name_storage_;
    float sample_rate_ = 48000.0f;
};

class GainDSP : public DSPNode {
    LinearRamp<float> gain_linear_;
public:
    GainDSP(float initial_gain_linear, std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
};

class MixerDSP : public DSPNode {
    std::vector<LinearRamp<float>> input_gains_linear_;
    uint8_t num_inputs_;
public:
    MixerDSP(std::string_view name, uint8_t num_inputs = 2);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

enum class BandFilterType { PEAKING, LOW_SHELF, HIGH_SHELF };

class ParametricEQDSP : public DSPNode {
    struct EQBand {
        LinearRamp<float> center_freq_hz;
        LinearRamp<float> q_factor;
        LinearRamp<float> gain_db;
        BandFilterType filter_type = BandFilterType::PEAKING;
        std::array<float, 3> b_coeffs{1.0f, 0.0f, 0.0f};
        std::array<float, 3> a_coeffs{1.0f, 0.0f, 0.0f};
        std::array<float, 2> z_state{0.0f, 0.0f};
        bool enabled = false;
    };
    std::array<EQBand, MAX_EQ_BANDS> bands_;
    void update_band_filter_coeffs(size_t band_idx);
public:
    explicit ParametricEQDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class LimiterDSP : public DSPNode {
    LinearRamp<float> threshold_linear_;
public:
    explicit LimiterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
};

class NoiseGateDSP : public DSPNode {
    LinearRamp<float> threshold_db_;
    LinearRamp<float> attack_time_ms_;
    LinearRamp<float> release_time_ms_;
    float envelope_ = 0.0f;
public:
    explicit NoiseGateDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class StereoWidthDSP : public DSPNode {
    LinearRamp<float> width_factor_;
    LinearRamp<float> balance_factor_;
public:
    explicit StereoWidthDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
};

class WaveshaperDSP : public DSPNode {
    LinearRamp<float> drive_;
    LinearRamp<float> output_gain_;
public:
    explicit WaveshaperDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
};

class EnvelopeFollowerDSP : public DSPNode {
    LinearRamp<float> attack_ms_;
    LinearRamp<float> release_ms_;
    float envelope_ = 0.0f;
public:
    explicit EnvelopeFollowerDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class CompressorDSP : public DSPNode {
    LinearRamp<float> threshold_db_;
    LinearRamp<float> ratio_;
    LinearRamp<float> attack_ms_;
    LinearRamp<float> release_ms_;
    LinearRamp<float> makeup_gain_db_;
    float envelope_ = 0.0f;
public:
    explicit CompressorDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class DelayDSP : public DSPNode {
    std::vector<float> delay_buffer_;
    size_t write_idx_ = 0;
    LinearRamp<float> delay_ms_;
    LinearRamp<float> feedback_;
    LinearRamp<float> mix_;
public:
    explicit DelayDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class ReverbDSP : public DSPNode {
    std::vector<float> delay_buffers_;
    std::vector<size_t> write_indices_;
    LinearRamp<float> decay_;
    LinearRamp<float> mix_;
    LinearRamp<float> pre_delay_ms_;
public:
    explicit ReverbDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class ChorusDSP : public DSPNode {
    std::vector<float> delay_buffer_;
    size_t write_idx_ = 0;
    LinearRamp<float> rate_hz_;
    LinearRamp<float> depth_ms_;
    LinearRamp<float> mix_;
public:
    explicit ChorusDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class FlangerDSP : public DSPNode {
    std::vector<float> delay_buffer_;
    size_t write_idx_ = 0;
    LinearRamp<float> rate_hz_;
    LinearRamp<float> depth_ms_;
    LinearRamp<float> feedback_;
    LinearRamp<float> mix_;
public:
    explicit FlangerDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class PitchShifterDSP : public DSPNode {
    std::vector<float> buffer_;
    size_t write_idx_ = 0;
    LinearRamp<float> pitch_shift_semitones_;
public:
    explicit PitchShifterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class ConvolutionReverbDSP : public DSPNode {
    std::vector<float> impulse_response_;
    std::vector<float> buffer_;
    size_t write_idx_ = 0;
    LinearRamp<float> mix_;
public:
    explicit ConvolutionReverbDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
};

class CrossoverDSP : public DSPNode {
    struct Band {
        std::vector<FilterStage> lowpass_stages;
        std::vector<FilterStage> highpass_stages;
        bool enabled = false;
    };
    std::vector<Band> bands_;
public:
    explicit CrossoverDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class PhaseCorrectionDSP : public DSPNode {
    std::array<float, kernel::core::MAX_AUDIO_CHANNELS> last_input_phase_ = {0.0f};
    std::array<float, kernel::core::MAX_AUDIO_CHANNELS> last_output_phase_ = {0.0f};
public:
    explicit PhaseCorrectionDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class SRCDSP : public DSPNode {
public:
    explicit SRCDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class FIRFilterDSP : public DSPNode {
    std::vector<float> coeffs_;
    std::vector<float> state_;
public:
    explicit FIRFilterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class IIRFilterDSP : public DSPNode {
    std::vector<FilterStage> stages_;
public:
    explicit IIRFilterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class FFTEqualizerDSP : public DSPNode {
public:
    explicit FFTEqualizerDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class NetworkAudioSinkSource : public DSPNode {
public:
    explicit NetworkAudioSinkSource(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
};

class DSPGraph {
    std::vector<std::unique_ptr<DSPNode>> nodes_;
    kernel::core::Spinlock graph_modification_lock_;
public:
    DSPGraph() = default;

    bool add_node(std::unique_ptr<DSPNode> node);
    bool remove_node(std::string_view name);
    bool configure_node(const char* name, const char* args, kernel::hal::UARTDriverOps* uart_ops);
    void process(std::span<float> buffer);
    void reset();
};

} // namespace dsp

#endif // DSP_HPP