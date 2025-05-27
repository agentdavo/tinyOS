// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio subsystem header for miniOS v1.7.
 * @details
 * Defines the audio processing pipeline including I2S hardware interface, DMA buffer management,
 * DSP graph integration, and memory pools for audio data. Supports real-time audio input/output
 * with configurable parameters like sample rate, block size, and buffer count. Integrates with
 * the DSP subsystem for effects processing and the HAL for hardware abstraction.
 *
 * New in v1.7:
 * - Renamed from audio_v1.6.hpp
 * - Enhanced error handling and diagnostics
 * - Improved Doxygen comments and code clarity
 * - Consistent use of std::span for buffer handling
 *
 * @version 1.7
 * @see audio.cpp, miniOS.hpp, dsp.hpp, util.hpp
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "miniOS.hpp" // Includes kernel types, SPSCQueue, FixedMemoryPool, hal::i2s
#include "dsp.hpp"    // Includes kernel::dsp::DSPGraph
#include <span>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <thread>   // For std::thread

namespace kernel { namespace hal { namespace i2s { struct I2SDriverOps; } } }

namespace audio {

// Max audio buffer size based on common DSP block sizes and sample rates
constexpr size_t MAX_AUDIO_SAMPLES_PER_BLOCK = 1024;
constexpr size_t MAX_AUDIO_CHANNELS = 2; // Stereo default

/**
 * @brief Represents a block of audio data.
 */
struct AudioBuffer {
    std::array<float, MAX_AUDIO_SAMPLES_PER_BLOCK * MAX_AUDIO_CHANNELS> data;
    size_t num_samples = 0;
    uint8_t channels = 0;
    uint32_t sample_rate_hz = 0;
    uint64_t timestamp_us = 0; // Timestamp of the first sample in the buffer
    // Add any other relevant metadata, e.g., buffer ID, status flags
};

/**
 * @brief Configuration for the audio system.
 */
struct AudioConfig {
    uint32_t sample_rate_hz = 48000;
    size_t samples_per_block = 256; // Samples per channel per block
    uint8_t num_channels = 2;
    uint8_t num_i2s_bufs = 4; // Number of I2S DMA buffers
    size_t num_raw_pool_bufs = 8; // Buffers for raw I2S data before DSP
    size_t num_dsp_pool_bufs = 8; // Buffers for DSP processed data before TX
};

/**
 * @brief Manages the audio processing pipeline.
 */
class AudioSystem {
public:
    AudioSystem();
    ~AudioSystem();

    bool init(const AudioConfig& config);
    bool start();
    void stop();

    // For application to send audio data (e.g., from a file or generator)
    AudioBuffer* get_buffer_for_app_tx();
    bool submit_filled_buffer_to_dsp_tx(AudioBuffer* buffer);

    // For application to receive audio data (e.g., for recording or analysis)
    AudioBuffer* get_filled_buffer_from_dsp_rx();
    void release_processed_buffer_to_raw_rx(AudioBuffer* buffer);

private:
    void i2s_thread_entry(); // For managing I2S data flow (if needed beyond callbacks)
    void dsp_thread_entry(); // For processing audio through the DSP graph

    void i2s_rx_callback_static(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode);
    void i2s_tx_callback_static(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode);
    
    // Static wrapper for I2S callback to pass to HAL
    static void i2s_hal_callback_wrapper(uint32_t instance_id, AudioBuffer* buffer,
                                         kernel::hal::i2s::Mode mode, void* user_data);


    AudioConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> i2s_initialized_{false};

    // Queues for data transfer between I2S, DSP, and application
    kernel::SPSCQueue_AudioBuffer* i2s_to_dsp_queue_ = nullptr;   // From I2S RX to DSP
    kernel::SPSCQueue_AudioBuffer* dsp_to_app_queue_ = nullptr;   // From DSP to App RX (recording)
    kernel::SPSCQueue_AudioBuffer* app_to_dsp_queue_ = nullptr;   // From App TX to DSP (playback)
    kernel::SPSCQueue_AudioBuffer* dsp_to_i2s_queue_ = nullptr;   // From DSP to I2S TX

    kernel::dsp::DSPGraph dsp_graph{"main_graph"}; ///< DSP processing graph
    std::thread dsp_thread_handle_;
    // std::thread i2s_thread_handle_; // If a dedicated I2S management thread is used

    kernel::hal::i2s::I2SDriverOps* i2s_ops_ = nullptr;
    std::unique_ptr<kernel::FixedMemoryPool> raw_pool_; // Pool for I2S hardware buffers
    std::unique_ptr<kernel::FixedMemoryPool> dsp_pool_; // Pool for buffers used by DSP graph and app
    uint32_t i2s_instance_id_ = 0; // Assuming one I2S instance for now

    // Helper to manage I2S callback logic
    void i2s_callback_internal(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode);

};

} // namespace audio

#endif // AUDIO_HPP