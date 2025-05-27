// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio subsystem header for miniOS v1.7.
 * @details
 * Defines a lock-free audio processing pipeline interfacing with I2S hardware via HAL, supporting
 * mono/stereo, 16/24-bit depths, and sample rates up to 192kHz. Integrates with DSP graph for
 * real-time audio processing and network streaming. Updated in v1.7 with improved error handling,
 * clearer documentation, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 *
 * @version 1.7
 * @see audio.cpp, miniOS.hpp, dsp.hpp, util.hpp
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "miniOS.hpp"
#include "dsp.hpp"
#include <span>
#include <atomic>
#include <memory>

namespace audio {

constexpr size_t MAX_SAMPLES_PER_BLOCK = 1024;
constexpr size_t MAX_I2S_BUFFERS = 8;

/**
 * @brief Audio format configuration.
 */
struct Format {
    enum class BitDepth { BITS_16, BITS_24 };
    uint32_t sample_rate_hz = 48000;
    uint8_t channels = 2; ///< Mono (1) or stereo (2)
    BitDepth bit_depth = BitDepth::BITS_16;
    size_t get_bytes_per_frame() const noexcept {
        return channels * (bit_depth == BitDepth::BITS_16 ? 2 : 3);
    }
};

/**
 * @brief Audio buffer for I2S and DSP processing.
 */
struct AudioBuffer {
    void* data_raw_i2s = nullptr; ///< Raw I2S data (hardware format)
    float* data_dsp_canonical = nullptr; ///< DSP data (floating-point, [-1.0, 1.0])
    size_t size_bytes_raw_buffer = 0; ///< Size of raw I2S buffer
    size_t samples_per_channel = 0; ///< Samples per channel
    bool pool_allocated_raw = false; ///< True if raw buffer is pool-allocated
    bool pool_allocated_dsp = false; ///< True if DSP buffer is pool-allocated
};

/**
 * @brief Audio subsystem configuration.
 */
struct AudioConfig {
    uint32_t sample_rate_hz = 48000; ///< Sample rate (Hz)
    size_t samples_per_block = 256; ///< Samples per block
    uint8_t num_i2s_bufs = 4; ///< Number of I2S buffers
};

/**
 * @brief Audio subsystem class.
 */
class AudioSystem {
public:
    AudioSystem() : initialized_(false) {}
    ~AudioSystem() = default;

    /**
     * @brief Initializes the audio subsystem.
     * @param cfg Audio configuration
     * @return True if initialized, false otherwise
     */
    bool init(const AudioConfig& cfg);

    /**
     * @brief Starts audio processing.
     * @return True if started, false otherwise
     */
    bool start();

    /**
     * @brief Stops audio processing.
     * @return True if stopped, false otherwise
     */
    bool stop();

    /**
     * @brief Gets a transmit buffer for an I2S instance.
     * @param instance_id I2S instance ID (0 for RX, 1 for TX)
     * @return Audio buffer pointer, or nullptr if unavailable
     */
    AudioBuffer* get_tx_buffer(uint32_t instance_id);

    /**
     * @brief Submits a filled transmit buffer to hardware.
     * @param instance_id I2S instance ID
     * @param buffer Audio buffer to submit
     * @return True if submitted, false otherwise
     */
    bool submit_tx_buffer(uint32_t instance_id, AudioBuffer* buffer);

    /**
     * @brief Releases a processed receive buffer.
     * @param instance_id I2S instance ID
     * @param buffer Audio buffer to release
     */
    void release_rx_buffer(uint32_t instance_id, AudioBuffer* buffer);

    dsp::DSPGraph dsp_graph{"main_graph"}; ///< DSP processing graph

private:
    AudioConfig config_;
    std::atomic<bool> initialized_;
    kernel::hal::i2s::I2SDriverOps* i2s_ops_ = nullptr;
    std::unique_ptr<kernel::FixedMemoryPool> raw_pool_;
    std::unique_ptr<kernel::FixedMemoryPool> dsp_pool_;
    void i2s_callback(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode);
    static void i2s_callback_trampoline(uint32_t instance_id, AudioBuffer* buffer,
                                       kernel::hal::i2s::Mode mode, void* user_data);
};

extern AudioSystem g_audio_system; ///< Global audio system instance

} // namespace audio

#endif // AUDIO_HPP