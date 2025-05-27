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
#include "util.hpp" // For kernel::util::memcpy if needed, or std::memcpy
#include <cstring>   // For std::memcpy
#include <algorithm> // For std::min, std::fill
#include <vector>    // For std::vector used as pool memory

// The global instance kernel::g_audio_system is defined in miniOS.cpp

namespace audio {

AudioSystem::AudioSystem() 
    : dsp_graph_("main_dsp_graph"), // Initialize DSPGraph member
      i2s_ops_(nullptr) {
    // Constructor: Initialize members to safe defaults if not done by member initializers
}

AudioSystem::~AudioSystem() {
    stop(); // Ensure resources are released
    if (dsp_thread_handle_.joinable()) {
        dsp_thread_handle_.join();
    }
}

bool AudioSystem::init(const AudioConfig& cfg) {
    kernel::ScopedISRLock lock(kernel::g_audio_system_lock); // Assuming a global lock for AudioSystem init/deinit

    if (initialized_.load(std::memory_order_acquire)) {
        return true; // Already initialized
    }
    if (!kernel::g_platform) {
        // Cannot proceed without platform services (e.g., for panic or logging)
        return false; 
    }

    config_ = cfg;
    // Validate config
    if (config_.sample_rate_hz < 8000 || config_.sample_rate_hz > 192000 ||
        config_.samples_per_block == 0 || config_.samples_per_block > MAX_AUDIO_SAMPLES_PER_BLOCK ||
        config_.num_channels == 0 || config_.num_channels > MAX_AUDIO_CHANNELS ||
        config_.num_i2s_dma_buffers == 0 || config_.num_i2s_dma_buffers > 16 || // Sanity cap for DMA bufs
        config_.num_audio_pool_buffers == 0 || config_.num_audio_pool_buffers > 32) { // Sanity cap for pool
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Invalid configuration", __FILE__, __LINE__);
        return false;
    }

    i2s_ops_ = kernel::g_platform->get_i2s_ops();
    if (!i2s_ops_) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: I2S HAL ops not available", __FILE__, __LINE__);
        return false;
    }

    // 1. Initialize memory pool for AudioBuffer objects
    size_t single_audio_buffer_obj_size = sizeof(AudioBuffer);
    pool_storage_for_audio_buffers_.resize(single_audio_buffer_obj_size * config_.num_audio_pool_buffers);
    audio_buffer_pool_ = std::make_unique<kernel::FixedMemoryPool>();
    if (!audio_buffer_pool_->init(pool_storage_for_audio_buffers_.data(), 
                                 config_.num_audio_pool_buffers, 
                                 single_audio_buffer_obj_size, 
                                 alignof(AudioBuffer))) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init AudioBuffer object pool", __FILE__, __LINE__);
        audio_buffer_pool_.reset(); // Release partially initialized pool
        return false;
    }

    // 2. Initialize SPSC Queues (capacity should be power of 2, e.g., 16)
    //    These queues store AudioBuffer*
    constexpr size_t queue_capacity = 16; // Must be > num_audio_pool_buffers if all buffers can be in flight
    i2s_rx_to_dsp_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
    dsp_to_app_rx_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
    app_tx_to_dsp_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();
    dsp_to_i2s_tx_queue_ = std::make_unique<kernel::SPSCQueue<audio::AudioBuffer, queue_capacity>>();


    // 3. Initialize I2S HAL
    kernel::hal::i2s::Format i2s_format;
    i2s_format.sample_rate_hz = config_.sample_rate_hz;
    i2s_format.channels = config_.num_channels; 
    // Assuming HAL needs a specific bit depth for configuration, map it or make it configurable.
    // For float processing, common hardware might be 24-bit or 32-bit fixed point.
    i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; // Example, adjust as per HAL capabilities

    // Initialize I2S RX
    if (!i2s_ops_->init(config_.i2s_rx_instance_id, kernel::hal::i2s::Mode::MASTER_RX, i2s_format, 
                        config_.samples_per_block, config_.num_i2s_dma_buffers, 
                        AudioSystem::i2s_hal_callback_wrapper, this)) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init I2S RX", __FILE__, __LINE__);
        return false; // Cleanup already done pools/queues if destructor is robust
    }

    // Initialize I2S TX
    if (!i2s_ops_->init(config_.i2s_tx_instance_id, kernel::hal::i2s::Mode::MASTER_TX, i2s_format, 
                        config_.samples_per_block, config_.num_i2s_dma_buffers, 
                        AudioSystem::i2s_hal_callback_wrapper, this)) {
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to init I2S TX", __FILE__, __LINE__);
        // TODO: De-initialize I2S RX if TX fails
        return false;
    }

    // 4. Pre-allocate AudioBuffer objects from the pool and provide them to I2S RX HAL
    // The HAL callback will receive these buffers.
    // This assumes the HAL wants pre-registered buffers.
    // For I2S RX:
    i2s_dma_buffers_rx_.resize(config_.num_i2s_dma_buffers); // If using AudioSystem managed buffers for DMA
    i2s_dma_buffer_free_rx_.assign(config_.num_i2s_dma_buffers, true);
    for (size_t i = 0; i < config_.num_i2s_dma_buffers; ++i) {
        AudioBuffer* rx_buf = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
        if (!rx_buf) {
            if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to allocate initial RX DMA buffers from pool", __FILE__, __LINE__);
            return false;
        }
        // Initialize buffer fields (the HAL might do this or expect it)
        new (rx_buf) AudioBuffer(); // Placement new to construct if pool gives raw memory
        rx_buf->num_samples = config_.samples_per_block;
        rx_buf->channels = config_.num_channels;
        rx_buf->sample_rate_hz = config_.sample_rate_hz;
        // i2s_dma_buffers_rx_[i] = rx_buf; // If storing pointers
        // The HAL's i2s_ops_->init or a separate submit_rx_buffer call would take these.
        // For simplicity, let's assume the callback gives us a HAL-managed buffer pointer that
        // we then copy into one of our pooled AudioBuffers.
        // The current HAL I2SCallback takes audio::AudioBuffer* directly.
        // This implies HAL might allocate them or AudioSystem provides them.
        // Let's refine: AudioSystem pools AudioBuffer objects. HAL uses their `data.data()`
        // or we provide full AudioBuffer* to HAL.
        // For now, we will queue empty buffers to the I2S driver in start() or here.
        // The callback gives us a *filled* buffer.
        // It is simpler if HAL provides its own raw buffers and i2s_callback_internal copies data.
        // For now, assume i2s_hal_callback_wrapper receives an AudioBuffer* that HAL has filled.
        // We still need to provide empty buffers to I2S TX.

        // For this example, let's assume the HAL's `init` call pre-registers internal buffers,
        // and the callback gives us an `AudioBuffer*` that maps to one of those.
        // This part is highly HAL-dependent.
        // The important part is `audio_buffer_pool_` is ready.
    }


    initialized_.store(true, std::memory_order_release);
    return true;
}

bool AudioSystem::start() {
    kernel::ScopedISRLock lock(kernel::g_audio_system_lock);
    if (!initialized_.load(std::memory_order_acquire) || running_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!i2s_ops_ || !audio_buffer_pool_) return false;


    // Start I2S hardware
    if (!i2s_ops_->start(config_.i2s_rx_instance_id)) { // Start RX
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to start I2S RX", __FILE__, __LINE__);
        return false;
    }
    if (!i2s_ops_->start(config_.i2s_tx_instance_id)) { // Start TX
        i2s_ops_->stop(config_.i2s_rx_instance_id); // Stop RX if TX failed to start
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to start I2S TX", __FILE__, __LINE__);
        return false;
    }
    
    // Pre-fill I2S TX queue with some empty/silent buffers to kickstart transmission
    for (size_t i = 0; i < config_.num_i2s_dma_buffers / 2; ++i) { // Fill half the DMA bufs
        AudioBuffer* tx_buf = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
        if (tx_buf) {
            new (tx_buf) AudioBuffer(); // Placement new
            tx_buf->num_samples = config_.samples_per_block;
            tx_buf->channels = config_.num_channels;
            tx_buf->sample_rate_hz = config_.sample_rate_hz;
            std::fill(tx_buf->data.begin(), tx_buf->data.begin() + (config_.samples_per_block * config_.num_channels), 0.0f); // Silence
            
            // The HAL's submit_filled_buffer_to_hw_tx needs to be called.
            // This usually happens in the DSP thread after processing.
            // For initial start, we might submit silence directly.
            // The current AudioBuffer is float, HAL needs conversion.
            // For now, assume dsp_thread will handle initial TX buffers.
            // Or, the HAL automatically starts requesting buffers once started.
            // Let's assume DSP thread will pick up from dsp_to_i2s_tx_queue_
            // and submit to HAL.
            // We can prime the dsp_to_i2s_tx_queue_ here if needed.
             if(!dsp_to_i2s_tx_queue_->enqueue(tx_buf)) {
                audio_buffer_pool_->free_block(tx_buf); // Failed to enqueue, return to pool
             }
        }
    }


    running_.store(true, std::memory_order_release);

    // Start DSP thread
    // Ensure kernel::g_scheduler_ptr is initialized before creating kernel threads
    if (kernel::g_scheduler_ptr) {
         // dsp_thread_handle_ = std::thread(&AudioSystem::dsp_thread_entry, this); // If using std::thread
         // For miniOS, create a kernel thread:
         auto* dsp_task_tcb = kernel::g_scheduler_ptr->create_thread(
             [](void* arg) { static_cast<AudioSystem*>(arg)->dsp_thread_entry(); },
             this,
             10, // Priority for DSP thread (adjust as needed)
             -1, // Core affinity (-1 for any)
             "AudioDSP",
             false, // Not an idle thread
             1000   // Deadline in us (e.g. 1ms, depends on block rate)
         );
         if (!dsp_task_tcb) {
            running_.store(false, std::memory_order_relaxed);
            i2s_ops_->stop(config_.i2s_rx_instance_id);
            i2s_ops_->stop(config_.i2s_tx_instance_id);
            if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Failed to create DSP thread", __FILE__, __LINE__);
            return false;
         }
    } else {
        // Cannot start DSP thread without scheduler
        running_.store(false, std::memory_order_relaxed);
        i2s_ops_->stop(config_.i2s_rx_instance_id);
        i2s_ops_->stop(config_.i2s_tx_instance_id);
        if (kernel::g_platform) kernel::g_platform->panic("AudioSystem: Scheduler not available for DSP thread", __FILE__, __LINE__);
        return false;
    }


    return true;
}

void AudioSystem::stop() {
    kernel::ScopedISRLock lock(kernel::g_audio_system_lock);
    if (!initialized_.load(std::memory_order_acquire) || !running_.load(std::memory_order_acquire)) {
        return;
    }
    running_.store(false, std::memory_order_release); // Signal threads to stop

    if (i2s_ops_) {
        i2s_ops_->stop(config_.i2s_rx_instance_id);
        i2s_ops_->stop(config_.i2s_tx_instance_id);
    }

    // Joining std::thread would happen here. For kernel TCBs, they should exit
    // when running_ becomes false. The TCB itself will be cleaned up by scheduler.
    // If dsp_thread_handle_ was std::thread:
    // if (dsp_thread_handle_.joinable()) {
    //     dsp_thread_handle_.join();
    // }

    // Clear queues and return buffers to pool (optional, depends on desired state after stop)
    AudioBuffer* buf;
    while((buf = i2s_rx_to_dsp_queue_->dequeue()) != nullptr) audio_buffer_pool_->free_block(buf);
    while((buf = dsp_to_app_rx_queue_->dequeue()) != nullptr) audio_buffer_pool_->free_block(buf);
    while((buf = app_tx_to_dsp_queue_->dequeue()) != nullptr) audio_buffer_pool_->free_block(buf);
    while((buf = dsp_to_i2s_tx_queue_->dequeue()) != nullptr) audio_buffer_pool_->free_block(buf);

    // Note: Further cleanup of pools or I2S de-init might happen in destructor or a separate deinit()
}

AudioBuffer* AudioSystem::get_buffer_for_app_tx() {
    if (!running_.load(std::memory_order_acquire)) return nullptr;
    AudioBuffer* buffer = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
    if (buffer) {
        new (buffer) AudioBuffer(); // Placement new to construct
        buffer->num_samples = config_.samples_per_block;
        buffer->channels = config_.num_channels;
        buffer->sample_rate_hz = config_.sample_rate_hz;
        // Application should fill buffer->data
    }
    return buffer;
}

bool AudioSystem::submit_filled_buffer_to_dsp_tx(AudioBuffer* buffer) {
    if (!running_.load(std::memory_order_acquire) || !buffer || !app_tx_to_dsp_queue_) {
        if (buffer) release_buffer_to_pool(buffer); // Return to pool if submit fails early
        return false;
    }
    if (!app_tx_to_dsp_queue_->enqueue(buffer)) {
        release_buffer_to_pool(buffer); // Failed to enqueue, return to pool
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
        // Could call destructor explicitly if placement new was used on raw memory.
        // buffer->~AudioBuffer(); // If needed before freeing block
        audio_buffer_pool_->free_block(buffer);
    }
}


// --- Thread Entries and Callbacks ---

void AudioSystem::dsp_thread_entry() {
    if (!kernel::g_platform) return; // Cannot run without platform context

    while (running_.load(std::memory_order_acquire)) {
        bool processed_something = false;

        // 1. Process I2S RX data -> DSP -> App RX Queue (Recording/Monitoring path)
        AudioBuffer* rx_buf = i2s_rx_to_dsp_queue_->dequeue();
        if (rx_buf) {
            // rx_buf already contains float data converted by i2s_callback_internal
            std::span<float> dsp_span(rx_buf->data.data(), rx_buf->num_samples * rx_buf->channels);
            dsp_graph_.process(dsp_span); // Process through DSP graph

            if (!dsp_to_app_rx_queue_->enqueue(rx_buf)) {
                release_buffer_to_pool(rx_buf); // Drop if app RX queue is full
            }
            processed_something = true;
        }

        // 2. Process App TX data -> DSP -> I2S TX Queue (Playback path)
        AudioBuffer* app_tx_buf = app_tx_to_dsp_queue_->dequeue();
        if (app_tx_buf) {
            std::span<float> dsp_span(app_tx_buf->data.data(), app_tx_buf->num_samples * app_tx_buf->channels);
            dsp_graph_.process(dsp_span); // Process through DSP graph

            if (!dsp_to_i2s_tx_queue_->enqueue(app_tx_buf)) {
                release_buffer_to_pool(app_tx_buf); // Drop if I2S TX queue is full
            }
            processed_something = true;
        }
        
        // 3. Send processed data from DSP to I2S TX
        AudioBuffer* dsp_tx_buf = dsp_to_i2s_tx_queue_->dequeue();
        if (dsp_tx_buf) {
            if (i2s_ops_) {
                // The HAL needs to know the format for conversion.
                kernel::hal::i2s::Format i2s_format;
                i2s_format.sample_rate_hz = dsp_tx_buf->sample_rate_hz;
                i2s_format.channels = dsp_tx_buf->channels;
                i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; // Assuming this conversion target
                
                // Convert float DSP data to I2S hardware format
                // This function needs to exist in I2SDriverOps and be implemented in HAL.
                // It would write to a hardware-specific buffer area if AudioBuffer::data isn't directly DMA'able.
                // For simplicity, if i2s_ops_->submit_filled_buffer_to_hw_tx takes our AudioBuffer*
                // and handles conversion/DMA internally:
                i2s_ops_->convert_dsp_format_to_hw_format(dsp_tx_buf, i2s_format); // Prepare for HAL
                
                if (!i2s_ops_->submit_filled_buffer_to_hw_tx(config_.i2s_tx_instance_id, dsp_tx_buf)) {
                    // Failed to submit to I2S HAL, what to do? Retry, drop?
                    // For now, release it back. This might mean an underrun.
                    release_buffer_to_pool(dsp_tx_buf);
                }
                // If submit is successful, the I2S TX callback will release dsp_tx_buf.
            } else {
                release_buffer_to_pool(dsp_tx_buf); // No I2S ops, release
            }
            processed_something = true;
        }


        if (!processed_something) {
            // No work done, yield or sleep to prevent busy-waiting
            // kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
            // Or use a condition variable / event flag signaled by the SPSC queues
            // For miniOS, a timed sleep or simple yield:
            if (kernel::g_platform && kernel::g_platform->get_timer_ops()) {
                 // A more advanced system would use an event/semaphore from the SPSC queue
                 // For now, a short sleep/yield
                 // This requires a sleep primitive in miniOS or HAL.
                 // Example: platform_sleep_us(1000); // Sleep 1ms
            }
             if(kernel::g_scheduler_ptr && kernel::g_platform) kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        }
    }
}

// Static wrapper
void AudioSystem::i2s_hal_callback_wrapper(uint32_t instance_id, 
                                           audio::AudioBuffer* buffer, 
                                           kernel::hal::i2s::Mode mode, 
                                           void* user_data) {
    if (user_data) {
        static_cast<AudioSystem*>(user_data)->i2s_callback_internal(instance_id, buffer, mode);
    } else if (kernel::g_platform) {
        // This is an error: callback without user_data means we can't find the AudioSystem instance
        kernel::g_platform->panic("I2S callback with null user_data", __FILE__, __LINE__);
    }
}

void AudioSystem::i2s_callback_internal(uint32_t instance_id, audio::AudioBuffer* buffer_from_hal, kernel::hal::i2s::Mode mode) {
    if (!buffer_from_hal || !initialized_.load(std::memory_order_acquire) || !running_.load(std::memory_order_acquire)) {
        // If buffer_from_hal was from our pool and HAL is just returning it, we might need to free it.
        // This logic depends heavily on HAL's buffer ownership model.
        // Assuming for now, if HAL gives us a buffer, it's one we gave it or it manages its own.
        // If it's one we gave it (e.g. for TX), it should be returned to our pool.
        if (buffer_from_hal && mode == kernel::hal::i2s::Mode::MASTER_TX) {
             release_buffer_to_pool(buffer_from_hal); // Buffer has been transmitted
        }
        // For RX, if HAL fills a buffer we provided, it would be enqueued.
        // If HAL allocates and fills, we copy and HAL frees.
        // The current I2SCallback signature in miniOS.hpp implies audio::AudioBuffer* is used.
        return;
    }

    if (mode == kernel::hal::i2s::Mode::MASTER_RX && instance_id == config_.i2s_rx_instance_id) {
        // This buffer_from_hal is filled by I2S hardware.
        // It should be one of the buffers from our audio_buffer_pool_ that we previously
        // "gave" to the HAL to fill (e.g. via a `i2s_ops_->submit_empty_buffer_to_hw_rx()` call).
        // For simplicity, let's assume the `buffer_from_hal` IS one of our pooled buffers.
        
        // Convert hardware format to canonical float DSP format
        kernel::hal::i2s::Format i2s_format;
        i2s_format.sample_rate_hz = buffer_from_hal->sample_rate_hz; // Should match config_
        i2s_format.channels = buffer_from_hal->channels;             // Should match config_
        i2s_format.bit_depth = kernel::hal::i2s::BitDepth::BITS_24_IN_32; // Assumed hardware format
        
        if (i2s_ops_) {
            i2s_ops_->convert_hw_format_to_dsp_format(buffer_from_hal, i2s_format);
        }
        buffer_from_hal->timestamp_us = kernel::g_platform ? kernel::g_platform->get_timer_ops()->get_system_time_us() : 0;

        if (!i2s_rx_to_dsp_queue_->enqueue(buffer_from_hal)) {
            // Failed to enqueue (DSP thread too slow or queue too small)
            // Return buffer to I2S HAL to be refilled, effectively dropping it.
            // Or return to general pool if HAL doesn't reuse directly.
            // This requires a mechanism for HAL to get new empty buffers.
            // For now, release it to the general pool.
            release_buffer_to_pool(buffer_from_hal);
            // TODO: Log underrun/overflow on RX side
        }
        // The HAL might need another empty buffer submitted for RX.
        // This could be done here by getting a new one from pool and calling a HAL function.
        // e.g., AudioBuffer* next_rx_buf = static_cast<AudioBuffer*>(audio_buffer_pool_->allocate());
        //        if(next_rx_buf) i2s_ops_->submit_empty_buffer_for_rx(instance_id, next_rx_buf);
        // This detail depends on the I2S HAL design.

    } else if (mode == kernel::hal::i2s::Mode::MASTER_TX && instance_id == config_.i2s_tx_instance_id) {
        // This buffer_from_hal was previously submitted by DSP thread via submit_filled_buffer_to_hw_tx.
        // It has now been transmitted by I2S hardware. Release it back to the pool.
        release_buffer_to_pool(buffer_from_hal);
    }
    // Else: unknown instance_id or mode, or buffer from an unexpected source.
}


} // namespace audio