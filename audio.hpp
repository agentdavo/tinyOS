// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file audio.hpp
 * @brief Audio buffer POD used by the arm64 I2S HAL driver (the rest of the
 *        audio subsystem was removed; see CLAUDE.md).
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include <cstddef>
#include <cstdint>

namespace kernel {
namespace audio {

struct AudioBuffer {
    void*  data_raw_i2s = nullptr;
    float* data_dsp_canonical = nullptr;
    size_t size_bytes_raw_buffer = 0;
    size_t samples_per_channel = 0;
    uint8_t channels = 2;
};

} // namespace audio
} // namespace kernel

#endif // AUDIO_HPP
