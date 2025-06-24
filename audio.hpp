// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio subsystem interfaces for miniOS v1.7.
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "core.hpp"
#include "hal.hpp"
#include <memory>

namespace dsp { class DSPGraph; }

namespace kernel {
namespace audio {

struct AudioBuffer {
    void*  data_raw_i2s = nullptr;
    float* data_dsp_canonical = nullptr;
    size_t size_bytes_raw_buffer = 0;
    size_t samples_per_channel = 0;
    uint8_t channels = 2;
};

struct AudioConfig {
    uint32_t sample_rate_hz;
    size_t   samples_per_block;
    uint8_t  num_channels;
    uint8_t  num_i2s_dma_buffers;
    size_t   num_audio_pool_buffers; // buffers from global pool
    uint32_t i2s_rx_instance_id;
    uint32_t i2s_tx_instance_id;
};

class AudioSystem {
public:
    AudioSystem() = default;
    ~AudioSystem() = default;

    bool init(const AudioConfig& config);
    bool start();
    void stop();

    dsp::DSPGraph&       get_dsp_graph();
    const dsp::DSPGraph& get_dsp_graph() const;

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;

private:
    static void i2s_rx_callback(uint32_t instance_id, AudioBuffer* buffer,
                                kernel::hal::i2s::Mode mode, void* user_data);
    static void i2s_tx_callback(uint32_t instance_id, AudioBuffer* buffer,
                                kernel::hal::i2s::Mode mode, void* user_data);
    static void dsp_thread(void* arg);

    AudioConfig config_{};
    bool is_running_ = false;
    kernel::core::TCB* dsp_thread_tcb_ = nullptr;
    std::unique_ptr<dsp::DSPGraph> dsp_graph_;
    std::unique_ptr<kernel::core::FixedMemoryPool> audio_buffer_pool_;
    std::unique_ptr<kernel::core::SPSCQueue<AudioBuffer, 16>> i2s_rx_to_dsp_queue_;
    std::unique_ptr<kernel::core::SPSCQueue<AudioBuffer, 16>> dsp_to_app_rx_queue_;
    std::unique_ptr<kernel::core::SPSCQueue<AudioBuffer, 16>> app_tx_to_dsp_queue_;
    std::unique_ptr<kernel::core::SPSCQueue<AudioBuffer, 16>> dsp_to_i2s_tx_queue_;
};

} // namespace audio
} // namespace kernel

#endif // AUDIO_HPP
