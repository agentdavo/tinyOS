// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio subsystem interfaces for miniOS v1.7.
 * @details
 * Defines interfaces for audio buffer management and processing, supporting low-latency
 * audio for network audio products. Updated in v1.7 for modularity and C++20 compatibility.
 *
 * @version 1.7
 * @see audio.cpp, core.hpp, hal.hpp
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "hal.hpp"
#include <cstdint>

namespace kernel {
namespace audio {

struct AudioBuffer {
    void* data_raw_i2s = nullptr; // Raw I2S data
    float* data_dsp_canonical = nullptr; // DSP-processed data
    size_t size_bytes_raw_buffer = 0;
    size_t samples_per_channel = 0;
    uint8_t channels = 2;
};

} // namespace audio
} // namespace kernel

#endif // AUDIO_HPP