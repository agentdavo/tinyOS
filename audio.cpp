// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.cpp
 * @brief Audio subsystem implementation for miniOS v1.7.
 * @details
 * Implements a lock-free audio processing pipeline with I2S hardware integration via HAL, supporting
 * mono/stereo, 16/24-bit depths, and sample rates up to 192kHz. Manages audio buffers, interfaces
 * with DSP graph, and handles network audio streaming. Generated for v1.7 to match v1.6 requirements,
 * with improved error handling, clearer documentation, and modern C++20 practices.
 *
 * C++20 features:
 * - std::span for buffer handling
 * - std::atomic for thread-safe state
 * - std::unique_ptr for resource management
 *
 * @version 1.7
 * @see audio.hpp, miniOS.hpp, dsp.hpp, util.hpp
 */

#include "audio.hpp"
#include "dsp.hpp" // Now needed for std::make_unique<kernel::dsp::DSPGraph>
#include "util.hpp" 
#include <cstring>   
#include <algorithm> 
#include <vector>    

// The global instance kernel::g_audio_system is defined in miniOS.cpp

namespace audio {

AudioSystem::AudioSystem() 
    : i2s_ops_(nullptr), dsp_thread_tcb_(nullptr) {
    // dsp_graph_ is unique_ptr, will be default constructed to nullptr.
    // It will be initialized in init().
}

AudioSystem::~AudioSystem() {
    stop(); 
    // If dsp_thread_tcb_ was managed by std::thread, join here.
    // For kernel TCBs, ensure they exit cleanly.
}

bool AudioSystem::init(const AudioConfig& cfg) {
    // Use a lock if init can be called from multiple contexts,
    // or ensure it's called from a single-threaded context during setup.
    // kernel::ScopedLock lock(audio_system_lock_); // Example, if audio_system_lock_ is defined and needed.

    if (initialized_.load(std::memory_order_acquire)) {
        return true; 
    }
    if (!kernel::g_platform) {
        return false; 
    }

    config_ = cfg;
    if (config_.sample_rate_hz < 8000 || config_.sample_rate_hz > 192000 ||
        config_.samples_per_block == 0 || config_.samples_per_block > MAX_AUDIO_SAMPLES_PER_BLOCK ||
        config_.num_channels == 0 || config_.num_channels > MAX_AUDIO_CHANNELS ||
        config_.num_i2s_dma_buffers == 0 || config_.num_i2s_dma_buffers > 16 || 
        config_.num_audio_pool_buffers == 0 || config_.num_audio_pool_buffers > 32) { 
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Invalid configuration", __FILE__, __LINE__);
        return false;
    }

    i2s_ops_ = kernel::g_platform->get_i2s_ops();
    if (!i2s_ops_) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: I2S HAL ops not available", __FILE__, __LINE__);
        return false;
    }

    // Initialize DSPGraph
    try {
        dsp_graph_ = std::make_unique<kernel::dsp::DSPGraph>("main_audio_dsp");
    } catch (const std::bad_alloc& e) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to allocate DSPGraph", __FILE__, __LINE__);
        return false;
    }
    if (!dsp_graph_) { // Should not happen if make_unique doesn't throw and returns null on failure (it throws)
         if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: DSPGraph is null after make_unique", __FILE__, __LINE__);
        return false;
    }


    size_t single_audio_buffer_obj_size = sizeof(AudioBuffer);
    try {
        pool_storage_for_audio_buffers_.resize(single_audio_buffer_obj_size * config_.num_audio_pool_buffers);
    } catch (const std::bad_alloc& e) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to allocate memory for AudioBuffer pool storage", __FILE__, __LINE__);
        return false;
    }
    
    audio_buffer_pool_ = std::make_unique<kernel::FixedMemoryPool>();
    if (!audio_buffer_pool_->init(pool_storage_for_audio_buffers_.data(), 
                                 config_.num_audio_pool_buffers, 
                                 single_audio_buffer_obj_size, 
                                 alignof(AudioBuffer))) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init AudioBuffer object pool", __FILE__, __LINE__);
        audio_buffer_pool_.reset(); 
        return false;
    }

    constexpr size_t queue_capacity = 16; 
    try {
        i2s_rx_to_dsp_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
        dsp_to_app_rx_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
        app_tx_to_dsp_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
        dsp_to_i2s_tx_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
    } catch (const std::bad_alloc& e) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to allocate SPSC queues", __FILE__, __LINE__);
        return false; // Other unique_ptrs will clean up
    }


    kernel::hal::i2s::Format i2s_format;
    i2s_format.sample_rate_hz = config_.sample_rate_hz;
    i2s_format.num_channels = config_.num_channels; 
    i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; 

    if (!i2s_ops_->init(config_.i2s_rx_instance_id, kernel::hal::i2s::Mode::MASTER_RX, i2s_format, 
                        config_.samples_per_block, config_.num_i2s_dma_buffers, 
                        AudioSystem::i2s_hal_callback_wrapper, this)) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init I2S RX", __FILE__, __LINE__);
        return false; 
    }

    if (!i2s_ops_->init(config_.i2s_tx_instance_id, kernel::hal::i2s::Mode::MASTER_TX, i2s_format, 
                        config_.samples_per_block, config_.num_i2s_dma_buffers, 
                        AudioSystem::i2s_hal_callback_wrapper, this)) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init I2S TX", __FILE__, __LINE__);
        return false;
    }
    
    initialized_.store(true, std::memory_order_release);
    return true;
}

bool AudioSystem::start() {
    // kernel::ScopedLock lock(audio_system_lock_); // If needed
    if (!initialized_.load(std::memory_order_acquire) || running_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!i2s_ops_ || !audio_buffer_pool_ || !dsp_graph_) return false;


    if (!i2s_ops_->start(config_.i2s_rx_instance_id)) { 
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to start I2S RX", __FILE__, __LINE__);
        return false;
    }
    if (!i2s_ops_->start(config_.i2s_tx_instance_id)) { 
        i2s_ops_->stop(config_.i2s_rx_instance_id); 
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to start I2S TX", __FILE__, __LINE__);
        return false;
    }
    
    for (size_t i = 0; i < config_.num_i2s_dma_buffers / 2; ++i) { 
        AudioBuffer* tx_buf = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
        if (tx_buf) {
            new (tx_buf) AudioBuffer(); 
            tx_buf->num_samples = config_.samples_per_block;
            tx_buf->channels = config_.num_channels;
            tx_buf->sample_rate_hz = config_.sample_rate_hz;
            std::fill(tx_buf->data.begin(), tx_buf->data.begin() + (config_.samples_per_block * config_.num_channels), 0.0f); 
            
             if(!dsp_to_i2s_tx_queue_->enqueue(tx_buf)) {
                audio_buffer_pool_->free_block(tx_buf); 
             }
        }
    }

    running_.store(true, std::memory_order_release);

    if (kernel::g_scheduler_ptr) {
         dsp_thread_tcb_ = kernel::g_scheduler_ptr->create_thread(
             [](void* arg) { static_cast<AudioSystem*>(arg)->dsp_thread_entry(); },
             this,
             10, 
             -1, 
             "AudioDSP",
             false, 
             static_cast<uint64_t>( (static_cast<double>(config_.samples_per_block) / config_.sample_rate_hz) * 1000000.0 ) // Deadline based on block rate
         );
         if (!dsp_thread_tcb_) {
            running_.store(false, std::memory_order_relaxed);
            i2s_ops_->stop(config_.i2s_rx_instance_id);
            i2s_ops_->stop(config_.i2s_tx_instance_id);
            if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to create DSP thread", __FILE__, __LINE__);
            return false;
         }
    } else {
        running_.store(false, std::memory_order_relaxed);
        i2s_ops_->stop(config_.i2s_rx_instance_id);
        i2s_ops_->stop(config_.i2s_tx_instance_id);
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Scheduler not available for DSP thread", __FILE__, __LINE__);
        return false;
    }

    return true;
}

void AudioSystem::stop() {
    // kernel::ScopedLock lock(audio_system_lock_); // If needed
    if (!initialized_.load(std::memory_order_acquire)) { // No need to check running_ if not initialized
        return;
    }
    
    bool expected_running = true;
    if (!running_.compare_exchange_strong(expected_running, false, std::memory_order_acq_rel)) {
        return; // Already stopped or stopping
    }

    // Signal DSP thread to terminate (it checks running_ flag)
    // If DSP thread is waiting on a queue, it might need an explicit wakeup/poison pill.
    // For now, assume it yields and checks running_.

    if (i2s_ops_) {
        i2s_ops_->stop(config_.i2s_rx_instance_id);
        i2s_ops_->stop(config_.i2s_tx_instance_id);
    }

    // For kernel TCBs, the thread should exit based on running_ flag.
    // Actual TCB reclamation is responsibility of the scheduler or a reaper.
    // We don't explicitly "join" a kernel TCB in the same way as std::thread.
    // If dsp_thread_tcb_ was stored, we might signal it or set its state if direct control is needed.
    dsp_thread_tcb_ = nullptr; // Clear TCB pointer

    AudioBuffer* buf;
    if(i2s_rx_to_dsp_queue_) while((buf = i2s_rx_to_dsp_queue_->dequeue()) != nullptr) release_buffer_to_pool(buf);
    if(dsp_to_app_rx_queue_) while((buf = dsp_to_app_rx_queue_->dequeue()) != nullptr) release_buffer_to_pool(buf);
    if(app_tx_to_dsp_queue_) while((buf = app_tx_to_dsp_queue_->dequeue()) != nullptr) release_buffer_to_pool(buf);
    if(dsp_to_i2s_tx_queue_) while((buf = dsp_to_i2s_tx_queue_->dequeue()) != nullptr) release_buffer_to_pool(buf);

    // Consider if pools should be reset/cleared. FixedMemoryPool itself doesn't clear memory.
    // Objects from pool are constructed on allocation and potentially destructed on release.
}

AudioBuffer* AudioSystem::get_buffer_for_app_tx() {
    if (!running_.load(std::memory_order_acquire) || !audio_buffer_pool_) return nullptr;
    AudioBuffer* buffer = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
    if (buffer) {
        new (buffer) AudioBuffer(); 
        buffer->num_samples = config_.samples_per_block;
        buffer->channels = config_.num_channels;
        buffer->sample_rate_hz = config_.sample_rate_hz;
    }
    return buffer;
}

bool AudioSystem::submit_filled_buffer_to_dsp_tx(AudioBuffer* buffer) {
    if (!running_.load(std::memory_order_acquire) || !buffer || !app_tx_to_dsp_queue_) {
        if (buffer) release_buffer_to_pool(buffer); 
        return false;
    }
    if (!app_tx_to_dsp_queue_->enqueue(buffer)) {
        release_buffer_to_pool(buffer); 
        return false;
    }
    return true;
}

AudioBuffer* AudioSystem::get_filled_buffer_from_dsp_rx() {
    if (!running_.load(std::memory_order_acquire) || !dsp_to_app_rx_queue_) return nullptr;
    return dsp_to_app_rx_queue_->dequeue();
}

void AudioSystem::release_buffer_to_pool(AudioBuffer* buffer) {
    if (buffer && audio_buffer_pool_) {
        // If placement new was used, and AudioBuffer has a non-trivial destructor, call it.
        // buffer->~AudioBuffer(); // std::array<float> has trivial destructor.
        audio_buffer_pool_->free_block(buffer);
    }
}

kernel::dsp::DSPGraph& AudioSystem::get_dsp_graph() {
    if (!dsp_graph_) {
        // This should ideally not happen if init was successful.
        // Handle error: either panic or return a dummy static graph.
        if(kernel::g_platform) kernel::g_platform->panic("AudioSystem: DSPGraph not initialized in get_dsp_graph", __FILE__, __LINE__);
        // Fallback to a static dummy if panic is not desired, but this indicates a severe issue.
        static kernel::dsp::DSPGraph dummy_graph("dummy_null_graph");
        return dummy_graph;
    }
    return *dsp_graph_;
}

const kernel::dsp::DSPGraph& AudioSystem::get_dsp_graph() const {
    if (!dsp_graph_) {
        if(kernel::g_platform) kernel::g_platform->panic("AudioSystem: DSPGraph not initialized in get_dsp_graph const", __FILE__, __LINE__);
        static kernel::dsp::DSPGraph dummy_graph("dummy_null_graph_const");
        return dummy_graph;
    }
    return *dsp_graph_;
}


// --- Thread Entries and Callbacks ---

void AudioSystem::dsp_thread_entry() {
    if (!kernel::g_platform || !dsp_graph_) return; 

    while (running_.load(std::memory_order_acquire)) {
        bool processed_something = false;

        AudioBuffer* rx_buf = i2s_rx_to_dsp_queue_->dequeue();
        if (rx_buf) {
            std::span<float> dsp_span(rx_buf->data.data(), rx_buf->num_samples * rx_buf->channels);
            dsp_graph_->process(dsp_span); 

            if (!dsp_to_app_rx_queue_->enqueue(rx_buf)) {
                release_buffer_to_pool(rx_buf); 
            }
            processed_something = true;
        }

        AudioBuffer* app_tx_buf = app_tx_to_dsp_queue_->dequeue();
        if (app_tx_buf) {
            std::span<float> dsp_span(app_tx_buf->data.data(), app_tx_buf->num_samples * app_tx_buf->channels);
            dsp_graph_->process(dsp_span); 

            if (!dsp_to_i2s_tx_queue_->enqueue(app_tx_buf)) {
                release_buffer_to_pool(app_tx_buf); 
            }
            processed_something = true;
        }
        
        AudioBuffer* dsp_tx_buf = dsp_to_i2s_tx_queue_->dequeue();
        if (dsp_tx_buf) {
            if (i2s_ops_) {
                kernel::hal::i2s::Format i2s_format;
                i2s_format.sample_rate_hz = dsp_tx_buf->sample_rate_hz;
                i2s_format.num_channels = dsp_tx_buf->channels;
                i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; 
                
                i2s_ops_->convert_dsp_format_to_hw_format(dsp_tx_buf, i2s_format); 
                
                if (!i2s_ops_->submit_filled_buffer_to_hw_tx(config_.i2s_tx_instance_id, dsp_tx_buf)) {
                    release_buffer_to_pool(dsp_tx_buf);
                }
            } else {
                release_buffer_to_pool(dsp_tx_buf); 
            }
            processed_something = true;
        }

        if (!processed_something) {
            if(kernel::g_scheduler_ptr && kernel::g_platform) {
                 kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
            } else {
                // Minimal busy wait or platform sleep if no scheduler yield
                for(volatile int i=0; i < 1000; ++i); // Placeholder for actual sleep/yield
            }
        }
    }
}

void AudioSystem::i2s_hal_callback_wrapper(uint32_t instance_id, 
                                           audio::AudioBuffer* buffer, 
                                           kernel::hal::i2s::Mode mode, 
                                           void* user_data) {
    if (user_data) {
        static_cast<AudioSystem*>(user_data)->i2s_callback_internal(instance_id, buffer, mode);
    } else if (kernel::g_platform) {
        kernel::g_platform->panic("I2S callback with null user_data", __FILE__, __LINE__);
    }
}

void AudioSystem::i2s_callback_internal(uint32_t instance_id, audio::AudioBuffer* buffer_from_hal, kernel::hal::i2s::Mode mode) {
    if (!buffer_from_hal || !initialized_.load(std::memory_order_acquire)) {
        // If this buffer was from our pool and HAL is just returning it, we must free it
        // This is complex if we don't know its origin. Let's assume HAL gives back what we gave it.
        if (buffer_from_hal && audio_buffer_pool_ && mode == kernel::hal::i2s::Mode::MASTER_TX) {
            release_buffer_to_pool(buffer_from_hal);
        }
        return;
    }
    if(!running_.load(std::memory_order_acquire)) { // If system is stopping
        if (buffer_from_hal && audio_buffer_pool_) release_buffer_to_pool(buffer_from_hal);
        return;
    }


    if (mode == kernel::hal::i2s::Mode::MASTER_RX && instance_id == config_.i2s_rx_instance_id) {
        if (!i2s_ops_ || !kernel::g_platform || !kernel::g_platform->get_timer_ops() || !i2s_rx_to_dsp_queue_) {
            release_buffer_to_pool(buffer_from_hal); // Cannot process
            return;
        }
        
        // Assuming buffer_from_hal is one of our pooled AudioBuffers that HAL has filled.
        // Its metadata (num_samples, channels, sample_rate_hz) should be pre-set or set by HAL.
        // For safety, let's ensure they are consistent:
        buffer_from_hal->num_samples = config_.samples_per_block;
        buffer_from_hal->channels = config_.num_channels;
        buffer_from_hal->sample_rate_hz = config_.sample_rate_hz;
        
        kernel::hal::i2s::Format i2s_format;
        i2s_format.sample_rate_hz = buffer_from_hal->sample_rate_hz; 
        i2s_format.num_channels = buffer_from_hal->channels;             
        i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; 
        
        i2s_ops_->convert_hw_format_to_dsp_format(buffer_from_hal, i2s_format);
        buffer_from_hal->timestamp_us = kernel::g_platform->get_timer_ops()->get_system_time_us();

        if (!i2s_rx_to_dsp_queue_->enqueue(buffer_from_hal)) {
            release_buffer_to_pool(buffer_from_hal);
            // TODO: Log RX overflow
        }
        // The HAL might require an explicit call to requeue this (or another) buffer for further RX DMA.
        // This detail is very HAL specific. If the HAL reuses the same set of buffers, nothing more here.
        // If it needs new ones:
        // AudioBuffer* next_empty_for_rx = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
        // if(next_empty_for_rx) { /* init it */ i2s_ops_->submit_empty_buffer_to_hal_rx(instance_id, next_empty_for_rx); }

    } else if (mode == kernel::hal::i2s::Mode::MASTER_TX && instance_id == config_.i2s_tx_instance_id) {
        release_buffer_to_pool(buffer_from_hal);
    }
}

} // namespace audio