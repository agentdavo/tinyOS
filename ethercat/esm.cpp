// SPDX-License-Identifier: MIT OR Apache-2.0
// ESM helpers (state-byte to name). Actual transition logic lives in
// Master::step_esm() in master.cpp so the datagram build path sits next to the
// cyclic thread.

#include "esm.hpp"

namespace ethercat {

const char* al_state_name(uint8_t s) noexcept {
    switch (s & 0x0F) {
        case AL_INIT:   return "INIT";
        case AL_PREOP:  return "PRE-OP";
        case AL_SAFEOP: return "SAFE-OP";
        case AL_OP:     return "OP";
        default:        return "?";
    }
}

} // namespace ethercat
