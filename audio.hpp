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
    // void* hw_buffer_handle = nullptr; // Optional: handle to an underlying hardware/HAL buffer if separate
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
 * @details Handles I2S input/output, buffer management using memory pools,
 * and routing audio data through a DSP graph. It operates its own thread
 * for DSP processing to decouple it from audio hardware interrupts.
 */
class AudioSystem {
public:
    /**
     * @brief Default constructor.
     */
    AudioSystem();
    /**
     * @brief Destructor. Stops the audio system and cleans up resources.
     */
    ~AudioSystem();

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) = delete;
    AudioSystem& operator=(AudioSystem&&) = delete;

    /**
     * @brief Initializes the audio system with the given configuration.
     * @param config The audio configuration parameters.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(const AudioConfig& config);

    /**
     * @brief Starts the audio processing pipeline.
     * @details Starts I2S hardware, DSP processing thread, and enables data flow.
     * @return True if successfully started, false otherwise.
     */
    bool start();

    /**
     * @brief Stops the audio processing pipeline.
     * @details Stops I2S hardware, DSP thread, and flushes queues.
     */
    void stop();

    /**
     * @brief Gets an empty audio buffer for an application to fill for transmission (playback).
     * @details The application should fill this buffer and then submit it using `submit_filled_buffer_to_dsp_tx`.
     * @return Pointer to an AudioBuffer, or nullptr if no buffers are available. The caller does NOT own the buffer.
     */
    AudioBuffer* get_buffer_for_app_tx();

    /**
     * @brief Submits an audio buffer filled by the application to the DSP graph for transmission.
     * @param buffer Pointer to the AudioBuffer filled by the application. Must be a buffer obtained from `get_buffer_for_app_tx`.
     * @return True if the buffer was successfully enqueued for DSP processing, false otherwise.
     */
    bool submit_filled_buffer_to_dsp_tx(AudioBuffer* buffer);

    /**
     * @brief Gets a filled audio buffer that has been processed by the DSP graph from the input (recording).
     * @details The application can process this data (e.g., save to file, analyze).
     * After processing, the buffer must be released using `release_buffer_to_pool`.
     * @return Pointer to an AudioBuffer, or nullptr if no processed buffers are available. The caller does NOT own the buffer.
     */
    AudioBuffer* get_filled_buffer_from_dsp_rx();
    
    /**
     * @brief Releases a processed audio buffer back to the system's pool.
     * @details This should be called after the application is done with a buffer obtained from
     * `get_filled_buffer_from_dsp_rx` or if it decides not to use a buffer from `get_buffer_for_app_tx`.
     * @param buffer Pointer to the AudioBuffer to release.
     */
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
    /**
     * @brief Main function for the DSP processing thread.
     * @details Dequeues buffers from I2S RX and application TX, processes them through dsp_graph_,
     * and enqueues them for I2S TX or application RX.
     */
    void dsp_thread_entry();

    /**
     * @brief Static trampoline function for I2S HAL callbacks.
     * @details Forwards the callback to the appropriate AudioSystem instance method.
     * @param instance_id The I2S instance ID that triggered the callback.
     * @param buffer Pointer to the AudioBuffer involved in the I2S operation (HAL might provide its own buffer type to be mapped).
     * @param mode Indicates if the operation was RX or TX.
     * @param user_data Pointer to the AudioSystem instance.
     */
    static void i2s_hal_callback_wrapper(uint32_t instance_id, 
                                         audio::AudioBuffer* buffer, // Assuming HAL can use our AudioBuffer directly or we map it
                                         kernel::hal::i2s::Mode mode, 
                                         void* user_data);

    /**
     * @brief Internal handler for I2S callbacks.
     * @details Called by `i2s_hal_callback_wrapper`. Handles buffer swapping and queueing.
     */
    void i2s_callback_internal(uint32_t instance_id, audio::AudioBuffer* buffer, kernel::hal::i2s::Mode mode);

    AudioConfig config_;                        ///< Current audio system configuration.
    std::atomic<bool> running_{false};          ///< True if the audio system is running.
    std::atomic<bool> initialized_{false};      ///< True if init() has been successfully called.

    // Memory pools for AudioBuffer objects themselves (not their internal data array, which is std::array)
    // Or, if AudioBuffer::data was dynamically allocated, pools for that.
    // Given AudioBuffer::data is std::array, we pool AudioBuffer objects.
    std::unique_ptr<kernel::FixedMemoryPool> audio_buffer_pool_; 
    // Memory for the pool of AudioBuffer objects
    std::vector<uint8_t> pool_storage_for_audio_buffers_;


    // Queues for data transfer between I2S, DSP, and application
    // These queues will hold pointers to AudioBuffer objects obtained from audio_buffer_pool_
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> i2s_rx_to_dsp_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> dsp_to_app_rx_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> app_tx_to_dsp_queue_;
    std::unique_ptr<kernel::SPSCQueue<audio::AudioBuffer, 16>> dsp_to_i2s_tx_queue_;

    kernel::dsp::DSPGraph dsp_graph_;        ///< The DSP processing graph.
    std::thread dsp_thread_handle_;             ///< Handle for the DSP processing thread.

    kernel::hal::i2s::I2SDriverOps* i2s_ops_ = nullptr; ///< Pointer to I2S HAL operations.
    
    // Buffers pre-allocated by HAL for I2S driver (if HAL manages them)
    // Or, AudioSystem allocates AudioBuffers and gives their data pointers to HAL.
    // The current audio.hpp AudioBuffer has its own std::array.
    // We will need to manage a set of these for the I2S driver.
    std::vector<AudioBuffer> i2s_dma_buffers_rx_;
    std::vector<AudioBuffer> i2s_dma_buffers_tx_;
    // To track which DMA buffers are free for HAL
    std::vector<bool> i2s_dma_buffer_free_rx_;
    std::vector<bool> i2s_dma_buffer_free_tx_;
    kernel::Spinlock i2s_rx_lock_; // If needed for callback buffer management
    kernel::Spinlock i2s_tx_lock_; // If needed for callback buffer management
};

// The global instance is declared in miniOS.hpp as extern, and defined in miniOS.cpp
// extern AudioSystem g_audio_system;

} // namespace audio

#endif // AUDIO_HPP