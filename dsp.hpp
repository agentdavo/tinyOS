// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file dsp.hpp
 * @brief Digital Signal Processing (DSP) subsystem header for miniOS v1.7.
 * @details
 * Defines a modular DSP framework for real-time audio processing, supporting professional audio nodes.
 * Updated in v1.7 with improved error handling, clearer documentation, and modern C++20 practices.
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
#include "net.hpp"    // For net::IPv4Addr (used by NetworkAudioSinkSource)
#include <span>
#include <vector>
#include <array>
#include <string_view>
#include <atomic>
#include <memory>      // For std::unique_ptr
#include <cmath>       // For std::pow, std::log10, std::sin, std::cos, std::tanh etc.
#include <string>      // For std::string (used in DSPNode name_)
#include <algorithm>   // For std::min, std::max

namespace kernel {
namespace dsp {

// --- DSP Utility Functions ---

/** @brief Converts decibels to linear gain. */
inline float db_to_linear(float db) noexcept { return std::pow(10.0f, db / 20.0f); }

/** @brief Converts linear gain to decibels. */
inline float linear_to_db(float gain) noexcept { return 20.0f * std::log10(std::max(gain, 1e-9f)); } // Avoid log10(0)

/** @brief Clips a value to a specified range [min_val, max_val]. */
inline float clip(float x, float min_val, float max_val) noexcept { return std::max(min_val, std::min(x, max_val)); }

/** @brief Generates coefficients for a Hann window. */
void generate_hann_window(std::span<float> window) noexcept;

/** @brief Generates biquad filter coefficients (Direct Form I or II). */
void generate_biquad_coeffs(std::string_view type, float fc_hz, float q_factor, float gain_db, float sample_rate_hz,
                           std::array<float, 3>& b_coeffs, std::array<float, 3>& a_coeffs) noexcept;

/** @brief Generates crossover filter coefficients (cascaded biquads). */
void generate_crossover_coeffs(std::string_view type, int order, float fc_hz, float sample_rate_hz,
                              std::vector<std::array<float, 3>>& b_coeffs_stages, 
                              std::vector<std::array<float, 3>>& a_coeffs_stages,
                              bool is_highpass_section) noexcept;

/**
 * @brief Parameter ramping for smooth transitions of DSP parameters.
 */
struct ParamRamp {
    std::atomic<float> current{0.0f}; ///< Current value of the parameter.
    std::atomic<float> target{0.0f};  ///< Target value the parameter is ramping towards.
    float step_size_ = 0.0f;          ///< Increment/decrement per sample; not atomic as it's set once per ramp.
    std::atomic<int> steps_remaining_{0}; ///< Number of steps remaining in the current ramp.

    /**
     * @brief Starts a new ramp.
     * @param to_value The target value to ramp to.
     * @param duration_ms Duration of the ramp in milliseconds.
     * @param sample_rate Current audio sample rate in Hz.
     */
    void start(float to_value, float duration_ms, float sample_rate) noexcept {
        float current_val = current.load(std::memory_order_relaxed);
        target.store(to_value, std::memory_order_relaxed);

        if (duration_ms <= 0.0f || sample_rate <= 0.0f) { // Instant change or invalid params
            current.store(to_value, std::memory_order_relaxed);
            step_size_ = 0.0f;
            steps_remaining_.store(0, std::memory_order_relaxed);
            return;
        }

        float total_samples_float = duration_ms * sample_rate / 1000.0f;
        int total_steps = static_cast<int>(std::max(1.0f, total_samples_float)); // At least one step

        if (std::abs(to_value - current_val) < 1e-6f) { // Already at target (or very close)
             current.store(to_value, std::memory_order_relaxed);
             step_size_ = 0.0f;
             steps_remaining_.store(0, std::memory_order_relaxed);
        } else {
            step_size_ = (to_value - current_val) / static_cast<float>(total_steps);
            steps_remaining_.store(total_steps, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Gets the next value in the ramp. Call once per sample.
     * @return The current (possibly updated) value of the parameter.
     */
    float next() noexcept {
        int remaining = steps_remaining_.load(std::memory_order_relaxed);
        if (remaining > 0) {
            float current_val = current.load(std::memory_order_relaxed);
            current_val += step_size_;
            current.store(current_val, std::memory_order_relaxed);
            if (steps_remaining_.fetch_sub(1, std::memory_order_acq_rel) == 1) { // Was this the last step?
                // Ensure target is precisely reached to avoid floating point drift
                current.store(target.load(std::memory_order_relaxed), std::memory_order_relaxed);
                step_size_ = 0.0f; // Mark ramp as finished
            }
            return current_val;
        }
        return current.load(std::memory_order_relaxed); // Ramp finished or not started
    }
    
    /** @brief Returns the current value without advancing the ramp. */
    float get_current() const noexcept { return current.load(std::memory_order_relaxed); }

    /** @brief Checks if the parameter is currently ramping. */
    bool is_active() const noexcept { return steps_remaining_.load(std::memory_order_relaxed) > 0; }
};


// --- Base DSP Node ---
/**
 * @brief Abstract base class for all Digital Signal Processing (DSP) nodes.
 */
class DSPNode {
public:
    /**
     * @brief Constructor.
     * @param node_name Unique name for this DSP node.
     */
    explicit DSPNode(std::string_view node_name);
    virtual ~DSPNode() = default; // Ensure virtual destructor for proper cleanup of derived classes

    DSPNode(const DSPNode&) = delete;
    DSPNode& operator=(const DSPNode&) = delete;
    DSPNode(DSPNode&&) = delete;
    DSPNode& operator=(DSPNode&&) = delete;

    /**
     * @brief Processes a block of audio data.
     * @details Derived classes implement their specific DSP algorithm here.
     * The buffer typically contains interleaved stereo audio samples.
     * @param audio_buffer A span representing the interleaved audio data to process in-place.
     */
    virtual void process(std::span<float> audio_buffer) = 0;

    /**
     * @brief Configures the DSP node with string-based arguments.
     * @param args A C-string containing configuration parameters for the node.
     * @param uart_ops Pointer to UART driver operations for printing status or errors during configuration.
     */
    virtual void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) = 0;

    /**
     * @brief Called typically once per processing block to update ramped parameters.
     */
    virtual void ramp_params() {} 

    /**
     * @brief Resets the internal state of the DSP node (e.g., filter states, delay lines).
     */
    virtual void reset() {} 

    /**
     * @brief Gets the name of the DSP node.
     * @return The name of the node as a string_view.
     */
    std::string_view get_name() const noexcept { return name_storage_; }

protected:
    std::string name_storage_; ///< Storage for the node's name.
    float sample_rate_ = 48000.0f; ///< Assumed sample rate, should be configurable or obtained from system.
};

// --- Specific DSP Node Declarations ---

class GainDSP : public DSPNode {
public:
    GainDSP(float initial_gain_linear, std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
private:
    ParamRamp gain_linear_; // Gain as a linear factor
};

class MixerDSP : public DSPNode {
public:
    MixerDSP(std::string_view name, uint8_t num_inputs = 2);
    void process(std::span<float> buffer) override; // Assumes inputs are already mixed into buffer or it's a conceptual mix
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    uint8_t num_inputs_;
    std::vector<ParamRamp> input_gains_linear_;
};

class ParametricEQDSP : public DSPNode {
public:
    static constexpr size_t MAX_EQ_BANDS = 4;
    enum class BandFilterType { PEAKING, LOW_SHELF, HIGH_SHELF };
    struct EQBand {
        ParamRamp center_freq_hz;
        ParamRamp q_factor;
        ParamRamp gain_db;
        BandFilterType filter_type = BandFilterType::PEAKING;
        std::array<float, 3> b_coeffs = {1.0f, 0.0f, 0.0f};
        std::array<float, 3> a_coeffs = {1.0f, 0.0f, 0.0f};
        std::array<float, 2> z_state = {0.0f, 0.0f}; // For DF-II Transposed: w[n-1], w[n-2]
        bool enabled = false;
    };
    ParametricEQDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    std::array<EQBand, MAX_EQ_BANDS> bands_;
    void update_band_filter_coeffs(size_t band_idx);
};

class LimiterDSP : public DSPNode {
public:
    LimiterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
private:
    ParamRamp threshold_linear_; // Threshold as linear value (0 to 1.0)
};

class NoiseGateDSP : public DSPNode {
public:
    NoiseGateDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    ParamRamp threshold_db_;
    ParamRamp attack_time_ms_;
    ParamRamp release_time_ms_;
    float envelope_ = 0.0f;
};

class StereoWidthDSP : public DSPNode {
public:
    StereoWidthDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
private:
    ParamRamp width_factor_; // 0.0 (mono) to 2.0 (extra wide)
    ParamRamp balance_factor_; // -1.0 (L) to 1.0 (R)
};

class WaveshaperDSP : public DSPNode {
public:
    enum class ShapeType { TANH, HARD_CLIP, SOFT_CLIP, ATAN };
    WaveshaperDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
private:
    ParamRamp drive_linear_;
    ParamRamp bias_offset_;
    ShapeType current_shape_ = ShapeType::TANH;
};

class EnvelopeFollowerDSP : public DSPNode {
public:
    EnvelopeFollowerDSP(std::string_view name);
    void process(std::span<float> buffer) override; // Reads input, updates internal envelope
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
    float get_envelope_value() const noexcept { return envelope_value_.load(std::memory_order_relaxed); }
private:
    ParamRamp attack_time_ms_;
    ParamRamp release_time_ms_;
    std::atomic<float> envelope_value_{0.0f};
    bool use_rms_detection_ = false;
};

class CompressorDSP : public DSPNode {
public:
    CompressorDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    ParamRamp threshold_db_;
    ParamRamp ratio_val_; // e.g., 4.0 for 4:1
    ParamRamp attack_time_ms_;
    ParamRamp release_time_ms_;
    ParamRamp makeup_gain_db_;
    float envelope_ = 0.0f; // Sidechain envelope
};

class DelayDSP : public DSPNode {
public:
    static constexpr size_t MAX_DELAY_SAMPLES_CONST = 96000; // ~2s @ 48kHz
    DelayDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
private:
    std::vector<float> delay_buffer_; // Dynamically sized for flexibility
    size_t current_delay_samples_ = 0;
    size_t write_index_ = 0;
};

class ReverbDSP : public DSPNode {
public:
    ReverbDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    // Simplified Schroeder-style reverb components
    static constexpr size_t NUM_COMB_FILTERS = 4;
    static constexpr size_t NUM_ALLPASS_FILTERS = 2;
    struct CombFilter {
        std::vector<float> buffer;
        size_t index = 0;
        float feedback = 0.0f;
        float damping = 0.0f; // Damping factor for feedback
        float last_out = 0.0f;
        void init(size_t delay_samples, float sr);
        float process(float input);
    };
    struct AllpassFilter {
        std::vector<float> buffer;
        size_t index = 0;
        float gain = 0.5f; // Typically around 0.5-0.7
        void init(size_t delay_samples, float sr);
        float process(float input);
    };
    std::array<CombFilter, NUM_COMB_FILTERS> comb_filters_;
    std::array<AllpassFilter, NUM_ALLPASS_FILTERS> allpass_filters_;
    
    ParamRamp pre_delay_time_ms_;
    std::vector<float> pre_delay_line_;
    size_t pre_delay_write_idx_ = 0;
    size_t current_pre_delay_samples_ = 0;

    ParamRamp room_size_param_; // 0.0 to 1.0
    ParamRamp damping_param_;   // 0.0 to 1.0
    ParamRamp wet_dry_mix_;     // 0.0 (dry) to 1.0 (wet)

    void update_internal_reverb_params();
};

class ChorusDSP : public DSPNode {
public:
    static constexpr size_t MAX_CHORUS_DELAY_SAMPLES = 2400; // ~50ms @ 48kHz
    ChorusDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    std::array<float, MAX_CHORUS_DELAY_SAMPLES> delay_line_;
    size_t write_index_ = 0;
    float lfo_phase_ = 0.0f; // Radians
    ParamRamp lfo_rate_hz_;
    ParamRamp depth_ms_;
    ParamRamp mix_level_; // 0.0 (dry) to 1.0 (wet)
    // ParamRamp feedback_level_; // Optional for chorus
};

class FlangerDSP : public DSPNode {
public:
    static constexpr size_t MAX_FLANGER_DELAY_SAMPLES = 480; // ~10ms @ 48kHz
    FlangerDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    std::array<float, MAX_FLANGER_DELAY_SAMPLES> delay_line_;
    size_t write_index_ = 0;
    float lfo_phase_ = 0.0f;
    ParamRamp lfo_rate_hz_;
    ParamRamp depth_ms_;
    ParamRamp feedback_level_; // -0.99 to +0.99
    ParamRamp mix_level_;
};

class PitchShifterDSP : public DSPNode {
public:
    static constexpr size_t PITCH_WINDOW_SIZE = 1024; // Analysis/synthesis window
    PitchShifterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    ParamRamp pitch_shift_factor_; // e.g., 1.0 = no shift, 2.0 = up octave
    std::array<float, PITCH_WINDOW_SIZE * 2> input_ring_buffer_; // Overlap-add requires larger buffer
    std::array<float, PITCH_WINDOW_SIZE> analysis_window_; // e.g., Hann window
    std::array<float, PITCH_WINDOW_SIZE> synthesis_window_;
    size_t input_write_ptr_ = 0;
    float phase_accumulator_ = 0.0f;
    float last_input_phase_[MAX_AUDIO_CHANNELS] = {0.0f}; // Assuming stereo max
    float last_output_phase_[MAX_AUDIO_CHANNELS] = {0.0f};
    // More state for FFT based pitch shifting (phase vocoder)
};

class ConvolutionReverbDSP : public DSPNode {
public:
    static constexpr size_t MAX_IR_LENGTH = 4096;
    ConvolutionReverbDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
    bool load_impulse_response(std::span<const float> ir_data);
private:
    std::vector<float> impulse_response_data_;
    std::vector<float> history_buffer_; // Input history for convolution
    size_t history_write_index_ = 0;
    ParamRamp wet_dry_mix_;
    void generate_default_impulse_response(); // Private helper
};

class CrossoverDSP : public DSPNode {
public:
    static constexpr size_t MAX_CROSSOVER_BANDS_OUTPUT = 4; // Max output bands (e.g., 4-way)
    enum class CrossoverFilterType { BUTTERWORTH, BESSEL, LINKWITZ_RILEY };
    struct BandSplitPoint {
        ParamRamp cutoff_hz;
        CrossoverFilterType filter_type = CrossoverFilterType::LINKWITZ_RILEY;
        int order = 2; // LR2=12dB/oct (1x 2nd order Butterworth), LR4=24dB/oct (2x 2nd order Butterworth)
        // Internally, this will generate low-pass and high-pass sections
        std::vector<std::array<float,3>> lp_b_coeffs, lp_a_coeffs;
        std::vector<std::array<float,3>> hp_b_coeffs, hp_a_coeffs;
        std::vector<std::array<float,2>> lp_z_state, hp_z_state; // State for each stage, per channel (if stereo)
    };

    CrossoverDSP(std::string_view name, uint8_t num_ways = 2);
    void process(std::span<float> buffer) override; // This node is complex. How to handle multiple outputs?
                                                    // Assume it outputs ONE band, configured by `selected_band_output_`.
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    uint8_t num_output_ways_; // e.g., 2 for 2-way, 3 for 3-way
    std::array<BandSplitPoint, MAX_CROSSOVER_BANDS_OUTPUT - 1> split_points_;
    uint8_t selected_band_output_idx_ = 0; // 0 to num_output_ways-1
    std::vector<float> temp_processing_buffer_; // For intermediate stages

    void update_split_filter_coeffs(size_t split_idx);
};

class PhaseCorrectionDSP : public DSPNode {
public:
    PhaseCorrectionDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    ParamRamp target_phase_degrees_;
    ParamRamp target_frequency_hz_;
    float allpass_coeff_a1_ = 0.0f; // For 1st order allpass: y[n] = a*x[n] + x[n-1] - a*y[n-1]
    float z_xn1_ = 0.0f; // x[n-1] state
    float z_yn1_ = 0.0f; // y[n-1] state
    void update_filter_coeffs();
};

class SRCDSP : public DSPNode { // Sample Rate Converter
public:
    static constexpr size_t SRC_FILTER_TAPS = 32;
    SRCDSP(std::string_view name, uint32_t initial_input_rate = 48000, uint32_t initial_output_rate = 44100);
    void process(std::span<float> buffer) override; // Complex: input/output sizes differ
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
private:
    uint32_t input_rate_hz_;
    uint32_t output_rate_hz_;
    float conversion_ratio_ = 1.0f;
    float fractional_time_step_ = 0.0f; // For interpolator
    std::vector<float> history_buffer_;  // Size based on filter taps
    std::vector<float> fir_filter_coeffs_; // Polyphase filter bank or windowed sinc
    void regenerate_fir_filter();
};

class FIRFilterDSP : public DSPNode {
public:
    static constexpr size_t MAX_FIR_TAPS = 64;
    FIRFilterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
private:
    std::array<float, MAX_FIR_TAPS> fir_taps_;
    size_t num_fir_taps_ = 0;
    std::array<float, MAX_FIR_TAPS> input_history_;
    size_t history_write_index_ = 0;
};

class IIRFilterDSP : public DSPNode { // Single biquad section
public:
    IIRFilterDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void reset() override;
private:
    std::array<float, 3> b_coeffs_ = {1.0f, 0.0f, 0.0f}; // b0, b1, b2
    std::array<float, 3> a_coeffs_ = {1.0f, 0.0f, 0.0f}; // a0 (1), a1, a2
    std::array<float, 2> z_state_ = {0.0f, 0.0f};      // w[n-1], w[n-2] for DF2T
};

class FFTEqualizerDSP : public DSPNode { // Conceptual
public:
    static constexpr size_t MAX_FFT_EQ_BANDS = 8;
    FFTEqualizerDSP(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
    void ramp_params() override;
    void reset() override;
private:
    std::array<ParamRamp, MAX_FFT_EQ_BANDS> band_gains_db_;
    size_t num_active_bands_ = 0;
    // FFT-related state would go here (FFT objects, overlap buffers, etc.)
};

class NetworkAudioSinkSource : public DSPNode {
public:
    NetworkAudioSinkSource(std::string_view name);
    void process(std::span<float> buffer) override;
    void configure(const char* args, kernel::hal::UARTDriverOps* uart_ops) override;
private:
    int udp_socket_idx_ = -1;
    bool is_socket_valid_ = false;
    bool is_configured_as_sink_ = false;
    net::IPv4Addr remote_target_ip_; // For sink mode
    uint16_t remote_target_port_ = 0; // For sink mode
    uint8_t num_audio_channels_ = 2;
};


// --- DSP Graph ---
/**
 * @brief Manages a chain or graph of DSPNode objects.
 */
class DSPGraph {
public:
    explicit DSPGraph(std::string_view graph_name);
    /**
     * @brief Adds a DSP node to the graph. Node ownership is transferred to the graph.
     * @param node A unique_ptr to the DSPNode to add.
     * @return True if added successfully, false if a node with the same name exists or other error.
     */
    bool add_node(std::unique_ptr<DSPNode> node);
    /**
     * @brief Removes a DSP node from the graph by name.
     * @param node_name The name of the node to remove.
     * @return True if removed successfully, false if not found.
     */
    bool remove_node(std::string_view node_name);
    /**
     * @brief Retrieves a pointer to a DSP node by its name.
     * @param node_name The name of the node to find.
     * @return Pointer to the DSPNode if found, nullptr otherwise. Caller does not own the pointer.
     */
    DSPNode* get_node(std::string_view node_name);
    /**
     * @brief Configures a specific node within the graph.
     * @param node_name The name of the node to configure.
     * @param args Configuration arguments string for the node.
     * @param uart_ops UART operations for status messages.
     * @return True if configuration was attempted (node found), false if node not found.
     */
    bool configure_node(std::string_view node_name, const char* args, kernel::hal::UARTDriverOps* uart_ops);
    /**
     * @brief Processes an audio buffer through all nodes in the graph sequentially.
     * @param audio_buffer The interleaved audio data to process.
     */
    void process(std::span<float> audio_buffer);
    /**
     * @brief Resets all nodes in the graph to their initial states.
     */
    void reset();

private:
    std::string name_storage_; // Name of the graph itself
    std::vector<std::unique_ptr<DSPNode>> dsp_nodes_; // Owned nodes in processing order
    kernel::Spinlock graph_modification_lock_; // To protect nodes_ vector during add/remove/configure
};

} // namespace dsp
} // namespace kernel

#endif // DSP_HPP