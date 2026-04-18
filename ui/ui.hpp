// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file ui.hpp
 * @brief Simple C++ UI framework for miniOS display.
 *
 * Provides widgets, layout, events, and display management.
 * Designed for 1080x1920 portrait display.
 */

#pragma once

#include "fb.hpp"
#include <array>

namespace kernel {
namespace ui {

// Event types for UI interaction
enum class EventType {
    None,
    TouchDown,
    TouchUp,
    TouchMove,
    KeyDown,
    KeyUp,
    Timer,
};

// Simple touch/click event
struct TouchEvent {
    int32_t x;
    int32_t y;
    bool pressed;
};

// Keyboard event
struct KeyEvent {
    uint32_t keycode;
    uint32_t modifiers;  // shift, ctrl, etc.
    bool pressed;
};

// Generic UI event
struct UIEvent {
    EventType type;
    union {
        TouchEvent touch;
        KeyEvent key;
    };
};

// Base widget class
class Widget {
public:
    Widget(int32_t x, int32_t y, uint32_t w, uint32_t h)
        : x_(x), y_(y), width_(w), height_(h), visible_(true), needs_redraw_(true) {}
    virtual ~Widget() = default;
    
    // Position
    int32_t x() const { return x_; }
    int32_t y() const { return y_; }
    void set_position(int32_t x, int32_t y) { x_ = x; y_ = y; mark_dirty(); }
    
    // Size
    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }
    void set_size(uint32_t w, uint32_t h) { width_ = w; height_ = h; mark_dirty(); }
    
    // Visibility
    bool visible() const { return visible_; }
    void show() { visible_ = true; mark_dirty(); }
    void hide() { visible_ = false; mark_dirty(); }
    
    // Redraw
    bool needs_redraw() const { return needs_redraw_; }
    void clear_redraw() { needs_redraw_ = false; }
    
    // Hit test
    bool contains(int32_t px, int32_t py) const {
        return px >= x_ && px < x_ + (int32_t)width_ && 
               py >= y_ && py < y_ + (int32_t)height_;
    }
    
    // Virtual render (to be overridden)
    virtual void render(Framebuffer&) {}
    
    // Event handlers (return true if handled)
    virtual bool on_event(const UIEvent& event) { (void)event; return false; }
    
    // Dirty flag
    void mark_dirty() { needs_redraw_ = true; }
    virtual void mark_subtree_dirty() { mark_dirty(); }

protected:
    int32_t x_, y_;
    uint32_t width_, height_;
    bool visible_;
    bool needs_redraw_;
};

// Container widget (holds children)
class Container : public Widget {
public:
    Container(int32_t x, int32_t y, uint32_t w, uint32_t h) 
        : Widget(x, y, w, h), child_count_(0) {}
    
    void add_child(Widget* child) {
        if (child && child_count_ < children_.size()) {
            children_[child_count_++] = child;
            mark_dirty();
        }
    }
    
    void remove_child(Widget* child) {
        for (size_t i = 0; i < child_count_; ++i) {
            if (children_[i] == child) {
                // Shift remaining children
                for (size_t j = i; j < child_count_ - 1; ++j) {
                    children_[j] = children_[j + 1];
                }
                --child_count_;
                mark_dirty();
                return;
            }
        }
    }
    
    void clear_children() {
        child_count_ = 0;
        mark_dirty();
    }
    
    void render(Framebuffer& fb) override {
        for (size_t i = 0; i < child_count_; ++i) {
            Widget* child = children_[i];
            if (child->visible() && child->needs_redraw()) {
                child->render(fb);
                child->clear_redraw();
            }
        }
    }
    
    bool on_event(const UIEvent& event) override {
        // Handle event for children (reverse order for z-order)
        for (size_t i = child_count_; i > 0; --i) {
            Widget* child = children_[i - 1];
            if (child->visible() && child->on_event(event)) {
                return true;
            }
        }
        return Widget::on_event(event);
    }

    void mark_subtree_dirty() override {
        mark_dirty();
        for (size_t i = 0; i < child_count_; ++i) {
            Widget* child = children_[i];
            if (child) child->mark_subtree_dirty();
        }
    }

protected:
    std::array<Widget*, 32> children_;
    size_t child_count_;
};

// Label widget (simple text)
class Label : public Widget {
public:
    Label(int32_t x, int32_t y, const char* text = "", Color fg = Color::White(), Color bg = Color::Black())
        : Widget(x, y, 100, 24), text_(text), fg_(fg), bg_(bg) {}
    
    void set_text(const char* text) { text_ = text; mark_dirty(); }
    const char* text() const { return text_; }
    
    void set_colors(Color fg, Color bg) { fg_ = fg; bg_ = bg; mark_dirty(); }
    
    void render(Framebuffer& fb) override {
        if (!visible_) return;
        fb.fill_rect(x_, y_, width_, height_, bg_);
        uint32_t tw = fb.draw_text(x_, y_, text_, fg_, bg_);
        if (tw > width_) width_ = tw;
    }

private:
    const char* text_;
    Color fg_, bg_;
};

// Button widget
class Button : public Widget {
public:
    using ClickCallback = void(*)();
    
    Button(int32_t x, int32_t y, uint32_t w, uint32_t h, const char* label = "Button")
        : Widget(x, y, w, h), label_(label), 
          normal_bg_(Color(64, 64, 64)), hover_bg_(Color(96, 96, 96)),
          pressed_bg_(Color(48, 48, 48)), fg_(Color::White()),
          hovered_(false), pressed_(false), callback_(nullptr) {}
    
    void set_label(const char* label) { label_ = label; mark_dirty(); }
    void set_callback(ClickCallback cb) { callback_ = cb; }
    void set_colors(Color normal_bg, Color hover_bg, Color pressed_bg, Color fg) {
        normal_bg_ = normal_bg;
        hover_bg_ = hover_bg;
        pressed_bg_ = pressed_bg;
        fg_ = fg;
        mark_dirty();
    }
    
    void render(Framebuffer& fb) override {
        if (!visible_) return;
        
        Color bg = normal_bg_;
        if (pressed_) bg = pressed_bg_;
        else if (hovered_) bg = hover_bg_;
        
        fb.fill_rect(x_, y_, width_, height_, bg);
        fb.draw_rect(x_, y_, width_, height_, fg_, 3);
        fb.fill_rect(x_ + 8, y_ + 8, width_ - 16, height_ - 16, bg);
        
        // Center text
        fb.draw_text(x_ + 16, y_ + static_cast<int32_t>(height_ / 2) - 8, label_, fg_, bg);
    }
    
    bool on_event(const UIEvent& event) override {
        if (event.type == EventType::TouchDown) {
            if (contains(event.touch.x, event.touch.y)) {
                pressed_ = true;
                hovered_ = true;
                mark_dirty();
                return true;
            }
        } else if (event.type == EventType::TouchUp) {
            if (pressed_) {
                pressed_ = false;
                if (contains(event.touch.x, event.touch.y)) {
                    hovered_ = true;
                    if (callback_) callback_();
                } else {
                    hovered_ = false;
                }
                mark_dirty();
                return true;
            }
        } else if (event.type == EventType::TouchMove) {
            bool was_hovered = hovered_;
            hovered_ = contains(event.touch.x, event.touch.y);
            if (hovered_ != was_hovered) mark_dirty();
        }
        return false;
    }

private:
    const char* label_;
    Color normal_bg_, hover_bg_, pressed_bg_, fg_;
    bool hovered_;
    bool pressed_;
    ClickCallback callback_;
};

class SegmentedControl : public Widget {
public:
    using SelectCallback = void(*)();
    struct Item {
        const char* label;
        SelectCallback callback;
    };

    SegmentedControl(int32_t x, int32_t y, uint32_t w, uint32_t h)
        : Widget(x, y, w, h), item_count_(0), active_index_(0),
          bg_(Color::White()), border_(Color::Black()),
          active_bg_(Color::Black()), active_fg_(Color::White()),
          inactive_fg_(Color::Black()) {}

    void set_colors(Color bg, Color border, Color active_bg, Color active_fg, Color inactive_fg) {
        bg_ = bg;
        border_ = border;
        active_bg_ = active_bg;
        active_fg_ = active_fg;
        inactive_fg_ = inactive_fg;
        mark_dirty();
    }

    void add_item(const char* label, SelectCallback cb) {
        if (item_count_ < items_.size()) {
            items_[item_count_++] = Item{label, cb};
            mark_dirty();
        }
    }

    void set_active(size_t index) {
        if (index < item_count_) {
            active_index_ = index;
            mark_dirty();
        }
    }

    size_t active_index() const { return active_index_; }

    void render(Framebuffer& fb) override {
        if (!visible_ || item_count_ == 0) return;
        fb.fill_rect(x_, y_, width_, height_, bg_);
        fb.draw_rect(x_, y_, width_, height_, border_, 3);
        const uint32_t seg_w = width_ / static_cast<uint32_t>(item_count_);
        for (size_t i = 0; i < item_count_; ++i) {
            const int32_t seg_x = x_ + static_cast<int32_t>(i * seg_w);
            const uint32_t draw_w = (i + 1 == item_count_) ? (width_ - static_cast<uint32_t>(i) * seg_w) : seg_w;
            const bool active = i == active_index_;
            const Color fill = active ? active_bg_ : bg_;
            const Color fg = active ? active_fg_ : inactive_fg_;
            fb.fill_rect(seg_x + 3, y_ + 3, draw_w - 6, height_ - 6, fill);
            if (i > 0) {
                fb.draw_line(seg_x, y_ + 4, seg_x, y_ + static_cast<int32_t>(height_) - 4, border_);
            }
            fb.draw_text(seg_x + 16, y_ + static_cast<int32_t>(height_ / 2) - 8, items_[i].label, fg, fill);
        }
    }

    bool on_event(const UIEvent& event) override {
        if (event.type != EventType::TouchUp || !contains(event.touch.x, event.touch.y) || item_count_ == 0) {
            return false;
        }
        const uint32_t seg_w = width_ / static_cast<uint32_t>(item_count_);
        const int32_t rel_x = event.touch.x - x_;
        size_t idx = static_cast<size_t>(rel_x >= 0 ? static_cast<uint32_t>(rel_x) / (seg_w ? seg_w : 1) : 0);
        if (idx >= item_count_) idx = item_count_ - 1;
        active_index_ = idx;
        if (items_[idx].callback) items_[idx].callback();
        mark_dirty();
        return true;
    }

private:
    std::array<Item, 6> items_{};
    size_t item_count_;
    size_t active_index_;
    Color bg_;
    Color border_;
    Color active_bg_;
    Color active_fg_;
    Color inactive_fg_;
};

class Panel : public Container {
public:
    Panel(int32_t x, int32_t y, uint32_t w, uint32_t h,
          const char* title = nullptr,
          Color bg = Color::White(),
          Color border = Color::Black(),
          Color title_fg = Color::Black())
        : Container(x, y, w, h), title_(title), bg_(bg), border_(border), title_fg_(title_fg) {}

    void set_title(const char* title) { title_ = title; mark_dirty(); }
    void set_colors(Color bg, Color border, Color title_fg) {
        bg_ = bg;
        border_ = border;
        title_fg_ = title_fg;
        mark_dirty();
    }

    void render(Framebuffer& fb) override {
        if (!visible_) return;
        fb.fill_rect(x_, y_, width_, height_, bg_);
        fb.draw_rect(x_, y_, width_, height_, border_, 3);
        if (title_ && title_[0]) {
            fb.fill_rect(x_ + 16, y_ - 2, 8 * 20, 20, bg_);
            fb.draw_text(x_ + 20, y_ + 10, title_, title_fg_, bg_);
        }
        Container::render(fb);
    }

private:
    const char* title_;
    Color bg_;
    Color border_;
    Color title_fg_;
};

// Progress bar widget
class ProgressBar : public Widget {
public:
    ProgressBar(int32_t x, int32_t y, uint32_t w, uint32_t h)
        : Widget(x, y, w, h), value_(0), max_(100), 
          bar_color_(Color::Green()), bg_color_(Color::DarkGray()) {}
    
    void set_value(uint32_t v) { value_ = v; mark_dirty(); }
    void set_max(uint32_t m) { max_ = m; mark_dirty(); }
    void set_colors(Color bar, Color bg) { bar_color_ = bar; bg_color_ = bg; mark_dirty(); }
    
    void render(Framebuffer& fb) override {
        if (!visible_) return;
        
        fb.fill_rect(x_, y_, width_, height_, bg_color_);
        uint32_t fill_w = (value_ * width_) / max_;
        if (fill_w > 0) {
            fb.fill_rect(x_, y_, fill_w, height_, bar_color_);
        }
    }

private:
    uint32_t value_, max_;
    Color bar_color_, bg_color_;
};

// Simple bar graph for displaying values
class BarGraph : public Widget {
public:
    BarGraph(int32_t x, int32_t y, uint32_t w, uint32_t h, uint32_t bars = 8)
        : Widget(x, y, w, h), bars_(bars), bar_width_((w - (bars-1)*2) / bars),
          values_(new int32_t[bars]()), max_val_(100) {
        for (uint32_t i = 0; i < bars; ++i) values_[i] = 0;
    }
    ~BarGraph() { delete[] values_; }
    
    void set_value(uint32_t idx, int32_t val) { 
        if (idx < bars_) { values_[idx] = val; mark_dirty(); } 
    }
    void set_max(uint32_t m) { max_val_ = m; mark_dirty(); }
    
    void render(Framebuffer& fb) override {
        if (!visible_) return;
        
        fb.fill_rect(x_, y_, width_, height_, Color::DarkGray());
        
        uint32_t gap = 2;
        uint32_t h = height_ - gap * 2;
        
        for (uint32_t i = 0; i < bars_; ++i) {
            uint32_t bar_h = (values_[i] * h) / max_val_;
            if (bar_h > h) bar_h = h;
            Color c = values_[i] > max_val_ * 0.9 ? Color::Red() : Color::Green();
            fb.fill_rect(x_ + i * (bar_width_ + gap), y_ + h - bar_h, bar_width_, bar_h, c);
        }
    }

private:
    uint32_t bars_;
    uint32_t bar_width_;
    int32_t* values_;
    uint32_t max_val_;
};

// Screen manager - holds multiple screens/pages
class ScreenManager {
public:
    ScreenManager(Framebuffer& fb) : fb_(fb), current_screen_(nullptr) {}
    
    void set_screen(Widget* screen) { 
        current_screen_ = screen; 
        if (screen) screen->mark_subtree_dirty();
    }
    
    Widget* current_screen() { return current_screen_; }
    
    void render() {
        if (current_screen_ && current_screen_->visible()) {
            if (current_screen_->needs_redraw()) {
                fb_.clear(Color::Black());
                current_screen_->render(fb_);
                current_screen_->clear_redraw();
            }
        }
    }
    
    bool handle_event(const UIEvent& event) {
        if (current_screen_) {
            return current_screen_->on_event(event);
        }
        return false;
    }

private:
    Framebuffer& fb_;
    Widget* current_screen_;
};

} // namespace ui
} // namespace kernel
