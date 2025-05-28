// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio subsystem header for miniOS v1.7.
 * @details
 * Defines the audio processing subsystem, including buffer management, I2S interface,
 * and DSP integration. Supports multi-channel audio with DMA-driven I2S transfers.
 * Updated in v1.7 with improved error handling, clearer documentation, and modern
 * C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::unique_ptr for resource management
 * - std::string_view for string operations
 *
 * @version 1.7
 * @see audio.cpp, core.hpp, hal.hpp
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "core.hpp"
#include "hal.hpp"
#include <memory>
#include <vector>
#include <array>
#include <string_view>

// Forward declaration
namespace dsp { class DSPGraph; }

namespace audio {

constexpr size_t MAX_AUDIO_SAMPLES_PER_BLOCK = 1024;

struct AudioBuffer {
    std::span<float> get_data() noexcept {
        return std::span<float>(data.data(), data.size());
    }
    std::span<const float> get_data() const noexcept {
        return std::span<const float>(data.data(), data.size());
    }
    std::array<float, MAX_AUDIO_SAMPLES_PER_BLOCK * kernel::core::MAX_AUDIO_CHANNELS> data;
    uint32_t num_samples_per_channel = 0;
    uint8_t num_channels = 0;
    uint32_t sample_rate_hz = 0;
};

struct AudioConfig {
    uint32_t sample_rate_hz;
    size_t samples_per_block;
    uint8_t num_channels;
    uint8_t num_i2s_dma_buffers;
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

    dsp::DSPGraph& get_dsp_graph();
    const dsp::DSPGraph& get_dsp_graph() const;

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;

private:
    static void i2s_rx_callback(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode, void* user_data);
    static void i2s_tx_callback(uint32_t instance_id, AudioBuffer* buffer, kernel::hal::i2s::Mode mode, void* user_data);
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

#endif // AUDIO_HPP