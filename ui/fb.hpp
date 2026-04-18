// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file fb.hpp
 * @brief Simple framebuffer driver for virtual display.
 *
 * Provides a simple in-memory framebuffer for UI rendering.
 * In a real system, this would interface with virtio-gpu or hardware framebuffer.
 * Resolution: 1080x1920 portrait (1080 width, 1920 height)
 * Format: RGBA8888 (32-bit per pixel)
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace kernel {
namespace ui {

constexpr uint32_t FB_WIDTH = 1080;
constexpr uint32_t FB_HEIGHT = 1920;
constexpr uint32_t FB_BPP = 32;
constexpr uint32_t FB_STRIDE = FB_WIDTH * 4;
constexpr uint32_t FB_SIZE = FB_WIDTH * FB_HEIGHT * 4;

struct Color {
    uint8_t r, g, b, a;
    
    constexpr Color(uint8_t r_ = 0, uint8_t g_ = 0, uint8_t b_ = 0, uint8_t a_ = 255)
        : r(r_), g(g_), b(b_), a(a_) {}
    
    static constexpr Color Black() { return Color(0, 0, 0, 255); }
    static constexpr Color White() { return Color(255, 255, 255, 255); }
    static constexpr Color Red() { return Color(255, 0, 0, 255); }
    static constexpr Color Green() { return Color(0, 255, 0, 255); }
    static constexpr Color Blue() { return Color(0, 0, 255, 255); }
    static constexpr Color Yellow() { return Color(255, 255, 0, 255); }
    static constexpr Color Cyan() { return Color(0, 255, 255, 255); }
    static constexpr Color Magenta() { return Color(255, 0, 255, 255); }
    static constexpr Color Gray() { return Color(128, 128, 128, 255); }
    static constexpr Color DarkGray() { return Color(64, 64, 64, 255); }
    static constexpr Color LightGray() { return Color(192, 192, 192, 255); }
    static constexpr Color Orange() { return Color(255, 165, 0, 255); }
};

class Framebuffer {
public:
    Framebuffer();
    ~Framebuffer();
    
    // Prevent copying
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;
    
    // Get raw framebuffer pointer (for DMA or hardware)
    uint32_t* data() { return buffer_; }
    const uint32_t* data() const { return buffer_; }
    
    // Dimensions
    uint32_t width() const { return FB_WIDTH; }
    uint32_t height() const { return FB_HEIGHT; }
    uint32_t stride() const { return FB_STRIDE; }
    uint32_t size() const { return FB_SIZE; }
    
    // Clear to color
    void clear(Color c = Color::Black());
    
    // Draw pixel
    void set_pixel(int32_t x, int32_t y, Color c);
    Color get_pixel(int32_t x, int32_t y) const;
    
    // Draw filled rectangle
    void fill_rect(int32_t x, int32_t y, uint32_t w, uint32_t h, Color c);
    
    // Draw outline rectangle
    void draw_rect(int32_t x, int32_t y, uint32_t w, uint32_t h, Color c, uint32_t line_width = 1);
    
    // Draw line (Bresenham)
    void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, Color c);
    
    // Draw circle
    void draw_circle(int32_t cx, int32_t cy, int32_t radius, Color c, bool filled = false);
    
    // Draw text (simple bitmap font, 8x16 characters)
    // Returns width of drawn text
    uint32_t draw_text(int32_t x, int32_t y, const char* text, Color fg, Color bg = Color::Black());

    // Draw text with integer pixel scaling (scale=1 matches draw_text).
    // Each glyph pixel becomes a scale×scale block; advance width is 8*scale.
    uint32_t draw_text_scaled(int32_t x, int32_t y, const char* text, Color fg,
                              Color bg = Color::Black(), uint32_t scale = 1);
    
    // Copy region
    void blit(int32_t dx, int32_t dy, const uint32_t* src, uint32_t sw, uint32_t sh, uint32_t src_stride);
    
    // Mark dirty (for hardware sync)
    void mark_dirty() { dirty_ = true; }
    bool is_dirty() const { return dirty_; }
    void clear_dirty() { dirty_ = false; }
    
    // Get/Set pixel fast (inline for performance)
    inline uint32_t pixel_index(int32_t x, int32_t y) const {
        if (x < 0 || x >= (int32_t)FB_WIDTH || y < 0 || y >= (int32_t)FB_HEIGHT) return 0;
        return static_cast<uint32_t>(y * FB_WIDTH + x);
    }

    // Screenshot: copy framebuffer to provided buffer (must be at least FB_SIZE bytes)
    void screenshot(uint32_t* out_buffer) const;
    
    // Save screenshot in PPM format (for debugging/viewing)
    // Returns number of bytes written, or 0 on error
    size_t save_ppm(const char* filename) const;
    
    // Get raw pointer for DMA/display (legacy)
    uint32_t* raw_buffer() { return buffer_; }
    uint32_t* buffer_;
    bool dirty_;
    
    // Simple 8x16 bitmap font (ASCII 32-126)
    static const uint8_t font_8x16[95][16];
};

// Runtime-created framebuffer singleton. Avoids relying on global constructor
// dispatch during early arm64 boot.
Framebuffer& framebuffer();

} // namespace ui
} // namespace kernel
