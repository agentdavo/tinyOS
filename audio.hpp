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

#include "miniOS.hpp" // Includes kernel types, SPSCQueue, FixedMemoryPool, hal::i2s, FORWARD DECL of DSPGraph
// dsp.hpp is NOT included here to break the circular dependency.
// kernel::dsp::DSPGraph is forward-declared in miniOS.hpp
#include <span>
#include <vector>
#include <memory>   // For std::unique_ptr
#include <thread>   // For std::thread
#include <atomic>   // For std::atomic

// Forward declaration for robustness, though miniOS.hpp should provide it.
namespace kernel { namespace hal { namespace i2s { struct I2SDriverOps; } } }
// kernel::dsp::DSPGraph is forward declared in miniOS.hpp

namespace audio {

constexpr size_t MAX_AUDIO_SAMPLES_PER_BLOCK = 1024; 
constexpr size_t MAX_AUDIO_CHANNELS = 2; 

struct AudioBuffer {
    std::array<float, MAX_AUDIO_SAMPLES_PER_BLOCK * MAX_AUDIO_CHANNELS> data;
    size_t num_samples = 0;    
    uint8_t channels = 0;      
    uint32_t sample_rate_hz = 0; 
    uint64_t timestamp_us = 0; 
};

struct AudioConfig {
    uint32_t sample_rate_hz = 48000;        
    size_t samples_per_block = 256;         
    uint8_t num_channels = 2;               
    uint8_t num_i2s_dma_buffers = 4;        
    size_t num_audio_pool_buffers = 8;      
    uint32_t i2s_rx_instance_id = 0;        
    uint32_t i2s_tx_instance_id = 1;        
};

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
     * @details Dereferences the unique_ptr. Caller must check if underlying graph is valid if it can be null.
     * @return A reference to the internal DSPGraph object. Throws if dsp_graph_ is null.
     */
    kernel::dsp::DSPGraph& get_dsp_graph(); // Implementation will dereference unique_ptr

    /**
     * @brief Provides const access to the DSP graph.
     * @details Dereferences the unique_ptr. Caller must check if underlying graph is valid if it can be null.
     * @return A const reference to the internal DSPGraph object. Throws if dsp_graph_ is null.
     */
    const kernel::dsp::DSPGraph& get_dsp_graph() const; // Implementation will dereference unique_ptr


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

    std::unique_ptr<kernel::dsp::DSPGraph> dsp_graph_; // Changed to unique_ptr
    kernel::TCB* dsp_thread_tcb_ = nullptr; 

    kernel::hal::i2s::I2SDriverOps* i2s_ops_ = nullptr; 
};

} // namespace audio

#endif // AUDIO_HPP