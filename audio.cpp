// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.cpp
 * @brief Audio subsystem implementation for miniOS v1.7.
 */

#include "audio.hpp"
#include "dsp.hpp"
#include "miniOS.hpp"
#include "util.hpp"

namespace kernel {
namespace audio {

bool AudioSystem::init(const AudioConfig& cfg) {
    if (!g_platform || !g_platform->get_i2s_ops() || is_running_) {
        return false;
    }

    config_ = cfg;

    hal::i2s::Format fmt{cfg.sample_rate_hz, cfg.num_channels,
                         hal::i2s::BitDepth::BITS_16};
    auto* i2s = g_platform->get_i2s_ops();

    if (!i2s->init(cfg.i2s_rx_instance_id, hal::i2s::Mode::MASTER_RX, fmt,
                   cfg.samples_per_block, cfg.num_i2s_dma_buffers,
                   &AudioSystem::i2s_rx_callback, this)) {
        return false;
    }
    if (!i2s->init(cfg.i2s_tx_instance_id, hal::i2s::Mode::MASTER_TX, fmt,
                   cfg.samples_per_block, cfg.num_i2s_dma_buffers,
                   &AudioSystem::i2s_tx_callback, this)) {
        return false;
    }

    dsp_graph_ = std::make_unique<dsp::DSPGraph>();
    i2s_rx_to_dsp_queue_ = std::make_unique<core::SPSCQueue<AudioBuffer, 16>>();
    dsp_to_app_rx_queue_ = std::make_unique<core::SPSCQueue<AudioBuffer, 16>>();
    app_tx_to_dsp_queue_ = std::make_unique<core::SPSCQueue<AudioBuffer, 16>>();
    dsp_to_i2s_tx_queue_ = std::make_unique<core::SPSCQueue<AudioBuffer, 16>>();

    return true;
}

bool AudioSystem::start() {
    if (is_running_ || !g_platform || !g_platform->get_i2s_ops() || !dsp_graph_) {
        return false;
    }
    auto* i2s = g_platform->get_i2s_ops();
    if (!i2s->start(config_.i2s_rx_instance_id) ||
        !i2s->start(config_.i2s_tx_instance_id)) {
        return false;
    }

    if (!g_scheduler_ptr) return false;
    dsp_thread_tcb_ = g_scheduler_ptr->create_thread(&AudioSystem::dsp_thread,
                                                     this, 3, 0, "DSP");
    if (!dsp_thread_tcb_) return false;

    is_running_ = true;
    return true;
}

void AudioSystem::stop() {
    is_running_ = false;
    if (g_platform && g_platform->get_i2s_ops()) {
        auto* i2s = g_platform->get_i2s_ops();
        i2s->stop(config_.i2s_rx_instance_id);
        i2s->stop(config_.i2s_tx_instance_id);
    }
}

dsp::DSPGraph& AudioSystem::get_dsp_graph() { return *dsp_graph_; }
const dsp::DSPGraph& AudioSystem::get_dsp_graph() const { return *dsp_graph_; }

void AudioSystem::i2s_rx_callback(uint32_t instance_id, AudioBuffer* buf,
                                  hal::i2s::Mode, void* user_data) {
    auto* self = static_cast<AudioSystem*>(user_data);
    if (!self || !buf) return;
    if (!self->i2s_rx_to_dsp_queue_->enqueue(buf)) {
        if (g_platform && g_platform->get_i2s_ops()) {
            g_platform->get_i2s_ops()->release_processed_buffer_to_hw_rx(instance_id, buf);
        }
    }
}

void AudioSystem::i2s_tx_callback(uint32_t instance_id, AudioBuffer* buf,
                                  hal::i2s::Mode, void*) {
    if (g_platform && g_platform->get_i2s_ops()) {
        g_platform->get_i2s_ops()->release_processed_buffer_to_hw_rx(instance_id, buf);
    }
}

void AudioSystem::dsp_thread(void* arg) {
    auto* self = static_cast<AudioSystem*>(arg);
    if (!self) return;

    auto* i2s = g_platform ? g_platform->get_i2s_ops() : nullptr;
    hal::i2s::Format fmt{self->config_.sample_rate_hz, self->config_.num_channels,
                         hal::i2s::BitDepth::BITS_16};

    while (self->is_running_) {
        AudioBuffer* buf = self->i2s_rx_to_dsp_queue_->dequeue();
        if (!buf) {
            if (g_scheduler_ptr && g_platform)
                g_scheduler_ptr->yield(g_platform->get_core_id());
            continue;
        }
        if (i2s) {
            i2s->convert_hw_format_to_dsp_format(buf, fmt);
        }
        std::span<float> span(buf->data_dsp_canonical,
                              buf->samples_per_channel * buf->channels);
        self->dsp_graph_->process(span);
        if (i2s) {
            i2s->convert_dsp_format_to_hw_format(buf, fmt);
            i2s->submit_filled_buffer_to_hw_tx(self->config_.i2s_tx_instance_id, buf);
        } else {
            if (g_platform && g_platform->get_i2s_ops())
                g_platform->get_i2s_ops()->release_processed_buffer_to_hw_rx(self->config_.i2s_rx_instance_id, buf);
        }
    }
}

} // namespace audio
} // namespace kernel

