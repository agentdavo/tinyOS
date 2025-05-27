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
#include <atomic>   // For std::atomic

// Forward declaration for robustness, though miniOS.hpp should provide it.
namespace kernel { namespace hal { namespace i2s { struct I2SDriverOps; } } }

namespace audio {

// Max audio buffer size based on common DSP block sizes and sample rates
constexpr size_t MAX_AUDIO_SAMPLES_PER_BLOCK = 1024; // Samples per channel
constexpr size_t MAX_AUDIO_CHANNELS = 2; // Stereo default

/**
 * @brief Represents a block of audio data.
 * @details This structure holds interleaved audio samples in floating-point format,
 * along with metadata about the audio data.
 */
struct AudioBuffer {
    /**
     * @brief Buffer for interleaved audio samples (LRLRLR...).
     * Uses float for canonical DSP processing format.
     */
    std::array<float, MAX_AUDIO_SAMPLES_PER_BLOCK * MAX_AUDIO_CHANNELS> data;
    size_t num_samples = 0;    ///< Number of samples *per channel* in this buffer.
    uint8_t channels = 0;      ///< Number of active audio channels in this buffer.
    uint32_t sample_rate_hz = 0; ///< Sample rate of the audio data in Hz.
    uint64_t timestamp_us = 0; ///< Timestamp of the first sample in the buffer (microseconds).
};

/**
 * @brief Configuration for the audio system.
 */
struct AudioConfig {
    uint32_t sample_rate_hz = 48000;        ///< Desired sample rate in Hz.
    size_t samples_per_block = 256;         ///< Number of samples per channel in each processing block.
    uint8_t num_channels = 2;               ///< Number of audio channels (e.g., 1 for mono, 2 for stereo).
    uint8_t num_i2s_dma_buffers = 4;        ///< Number of DMA buffers for the I2S hardware (per direction if applicable).
    size_t num_audio_pool_buffers = 8;      ///< Number of AudioBuffer structures to pool for DSP/application use.
    uint32_t i2s_rx_instance_id = 0;        ///< Platform-specific I2S instance ID for reception.
    uint32_t i2s_tx_instance_id = 1;        ///< Platform-specific I2S instance ID for transmission.
};

/**
 * @brief Manages the audio processing pipeline.
 */
class AudioSystem {
public:
    AudioSystem();
    ~AudioSystem();

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) = delete;
    AudioSystem& operator=(AudioSystem&&) = delete;

    bool init(const AudioConfig& config);
    bool start();
    void stop();

    AudioBuffer* get_buffer_for_app_tx();
    bool submit_filled_buffer_to_dsp_tx(AudioBuffer* buffer);
    AudioBuffer* get_filled_buffer_from_dsp_rx();
    void release_buffer_to_pool(AudioBuffer* buffer);

    /**
     * @brief Provides access to the DSP graph for configuration.
     * @return A reference to the internal DSPGraph object.
     */
    kernel::dsp::DSPGraph& get_dsp_graph() { return dsp_graph_; }

    /**
     * @brief Provides const access to the DSP graph.
     * @return A const reference to the internal DSPGraph object.
     */
    const kernel::dsp::DSPGraph& get_dsp_graph() const { return dsp_graph_; }

private:
    void dsp_thread_entry();
    static void i2s_hal_callback_wrapper(uint32_t instance_id, 
                                         audio::AudioBuffer* buffer,
                                         kernel::hal::i2s::Mode mode, 
                                         void* user_data);
    void i2s_callback_internal(uint32_t instance_id, audio::AudioBuffer* buffer, kernel::hal::i2s::Mode mode);

    AudioConfig config_;                        
    std::atomic<bool> running_{false};          
    std::atomic<bool> initialized_{false};      

    std::unique_ptr<kernel::FixedMemoryPool> audio_buffer_pool_; 
    std::vector<uint8_t> pool_storage_for_audio_buffers_;

    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> i2s_rx_to_dsp_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> dsp_to_app_rx_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> app_tx_to_dsp_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> dsp_to_i2s_tx_queue_;

    kernel::dsp::DSPGraph dsp_graph_;        
    kernel::TCB* dsp_thread_tcb_ = nullptr; 

    kernel::hal::i2s::I2SDriverOps* i2s_ops_ = nullptr; 
};

} // namespace audio

#endif // AUDIO_HPP