// SPDX-License-Identifier: MIT OR Apache-2.0

#pragma once

#include <cstdint>

namespace kernel::ui {

bool init_display_backend();
bool present_display_backend();
// Damage-rect variant. Falls back to full-screen present when the display
// backend doesn't override present_rect or when the rect collapses to
// w==0/h==0 (typical for the heartbeat-only path).
bool present_display_backend_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h);

} // namespace kernel::ui
