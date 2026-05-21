// SPDX-License-Identifier: MIT OR Apache-2.0

#include "display.hpp"
#include "fb.hpp"
#include "../miniOS.hpp"

namespace kernel::ui {

bool init_display_backend() {
    auto* plat = kernel::g_platform;
    if (!plat) return false;
    auto* display = plat->get_display_ops();
    if (!display) return false;
    return display->init(framebuffer().data(), FB_WIDTH, FB_HEIGHT, FB_STRIDE);
}

bool present_display_backend() {
    auto* plat = kernel::g_platform;
    if (!plat) return false;
    auto* display = plat->get_display_ops();
    if (!display) return false;
    return display->present();
}

bool present_display_backend_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    auto* plat = kernel::g_platform;
    if (!plat) return false;
    auto* display = plat->get_display_ops();
    if (!display) return false;
    if (w == 0 || h == 0) return display->present();
    return display->present_rect(x, y, w, h);
}

} // namespace kernel::ui
