// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file splash.cpp
 * @brief TSV-only UI bootstrap for miniOS.
 */

#include "fb.hpp"
#include "display.hpp"
#include "operator_api.hpp"
#include "splash.hpp"
#include "ui.hpp"

#include "../miniOS.hpp"
#include "../ui/ui_builder_tsv.hpp"
#include "../util.hpp"

#include <atomic>
#include <new>

namespace kernel::ui {

namespace {

std::atomic<unsigned> g_ui_loop_iterations{0};

constexpr uint32_t kKeyModShift = 1u << 0;
constexpr uint32_t kKeyModCtrl  = 1u << 1;
constexpr uint32_t kKeyModAlt   = 1u << 2;

constexpr uint8_t KEY_ESC = 1;
constexpr uint8_t KEY_1 = 2;
constexpr uint8_t KEY_0 = 11;
constexpr uint8_t KEY_MINUS = 12;
constexpr uint8_t KEY_EQUAL = 13;
constexpr uint8_t KEY_BACKSPACE = 14;
constexpr uint8_t KEY_TAB = 15;
constexpr uint8_t KEY_Q = 16;
constexpr uint8_t KEY_W = 17;
constexpr uint8_t KEY_E = 18;
constexpr uint8_t KEY_R = 19;
constexpr uint8_t KEY_T = 20;
constexpr uint8_t KEY_Y = 21;
constexpr uint8_t KEY_U = 22;
constexpr uint8_t KEY_I = 23;
constexpr uint8_t KEY_O = 24;
constexpr uint8_t KEY_P = 25;
constexpr uint8_t KEY_LEFTBRACE = 26;
constexpr uint8_t KEY_RIGHTBRACE = 27;
constexpr uint8_t KEY_ENTER = 28;
constexpr uint8_t KEY_LEFTCTRL = 29;
constexpr uint8_t KEY_A = 30;
constexpr uint8_t KEY_S = 31;
constexpr uint8_t KEY_D = 32;
constexpr uint8_t KEY_F = 33;
constexpr uint8_t KEY_G = 34;
constexpr uint8_t KEY_H = 35;
constexpr uint8_t KEY_J = 36;
constexpr uint8_t KEY_K = 37;
constexpr uint8_t KEY_L = 38;
constexpr uint8_t KEY_SEMICOLON = 39;
constexpr uint8_t KEY_APOSTROPHE = 40;
constexpr uint8_t KEY_GRAVE = 41;
constexpr uint8_t KEY_LEFTSHIFT = 42;
constexpr uint8_t KEY_BACKSLASH = 43;
constexpr uint8_t KEY_Z = 44;
constexpr uint8_t KEY_X = 45;
constexpr uint8_t KEY_C = 46;
constexpr uint8_t KEY_V = 47;
constexpr uint8_t KEY_B = 48;
constexpr uint8_t KEY_N = 49;
constexpr uint8_t KEY_M = 50;
constexpr uint8_t KEY_COMMA = 51;
constexpr uint8_t KEY_DOT = 52;
constexpr uint8_t KEY_SLASH = 53;
constexpr uint8_t KEY_RIGHTSHIFT = 54;
constexpr uint8_t KEY_KPASTERISK = 55;
constexpr uint8_t KEY_LEFTALT = 56;
constexpr uint8_t KEY_SPACE = 57;
constexpr uint8_t KEY_RIGHTCTRL = 97;
constexpr uint8_t KEY_KPENTER = 96;
constexpr uint8_t KEY_KPSLASH = 98;
constexpr uint8_t KEY_RIGHTALT = 100;
constexpr uint8_t KEY_HOME = 102;
constexpr uint8_t KEY_UP = 103;
constexpr uint8_t KEY_PAGEUP = 104;
constexpr uint8_t KEY_LEFT = 105;
constexpr uint8_t KEY_RIGHT = 106;
constexpr uint8_t KEY_END = 107;
constexpr uint8_t KEY_DOWN = 108;
constexpr uint8_t KEY_PAGEDOWN = 109;
constexpr uint8_t KEY_INSERT = 110;
constexpr uint8_t KEY_DELETE = 111;
constexpr uint8_t KEY_KPMINUS = 74;
constexpr uint8_t KEY_KPPLUS = 78;
constexpr uint8_t KEY_KP1 = 79;
constexpr uint8_t KEY_KP2 = 80;
constexpr uint8_t KEY_KP3 = 81;
constexpr uint8_t KEY_KP4 = 75;
constexpr uint8_t KEY_KP5 = 76;
constexpr uint8_t KEY_KP6 = 77;
constexpr uint8_t KEY_KP7 = 71;
constexpr uint8_t KEY_KP8 = 72;
constexpr uint8_t KEY_KP9 = 73;
constexpr uint8_t KEY_KP0 = 82;
constexpr uint8_t KEY_KPDOT = 83;

ScreenManager& screen_manager(Framebuffer& fb) {
    alignas(ScreenManager) static unsigned char storage[sizeof(ScreenManager)];
    static bool constructed = false;
    if (!constructed) {
        new (storage) ScreenManager(fb);
        constructed = true;
    }
    return *reinterpret_cast<ScreenManager*>(storage);
}

inline void wait_until_ns(uint64_t target_ns) {
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (timer) timer->wait_until_ns(target_ns);
}

void ui_log_once(const char* msg) {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (!uart || !msg) return;
    static const char* seen[16]{};
    for (const char* entry : seen) {
        if (entry == msg) return;
    }
    for (auto& entry : seen) {
        if (!entry) {
            entry = msg;
            break;
        }
    }
    uart->puts(msg);
}

int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// `abs_range` comes from InputOps::pointer_abs_range(): the device's
// normalised full scale for absolute pointers (tablet / touch), or 0 when the
// value is already in pixels (relative mouse). This used to guess the range
// from each value and treated anything below the screen size as pixels, so a
// tablet touch at raw 5000 (~15% across) landed at x=658 instead of 164.
int32_t normalize_touch_axis(int32_t value, int32_t extent, uint32_t abs_range) {
    if (extent <= 1) return 0;
    if (abs_range == 0) return clamp_i32(value, 0, extent - 1);
    if (value <= 0) return 0;
    const uint64_t scaled = (static_cast<uint64_t>(value) * static_cast<uint64_t>(extent - 1)) / abs_range;
    return clamp_i32(static_cast<int32_t>(scaled), 0, extent - 1);
}

void dispatch_touch(ScreenManager& screen, EventType type, int32_t x, int32_t y, bool pressed) {
    UIEvent event{};
    event.type = type;
    event.touch = TouchEvent{x, y, pressed};
    screen.handle_event(event);
}

bool key_is_keyboard_pointer_move(uint8_t key) {
    switch (key) {
        case KEY_UP:
        case KEY_DOWN:
        case KEY_LEFT:
        case KEY_RIGHT:
        case KEY_W:
        case KEY_A:
        case KEY_S:
        case KEY_D:
        case KEY_H:
        case KEY_J:
        case KEY_K:
        case KEY_L:
        case KEY_KP2:
        case KEY_KP4:
        case KEY_KP6:
        case KEY_KP8:
        case KEY_HOME:
        case KEY_END:
        case KEY_PAGEUP:
        case KEY_PAGEDOWN:
            return true;
        default:
            return false;
    }
}

bool adjust_keyboard_pointer(uint8_t key, uint32_t modifiers, int32_t& x, int32_t& y) {
    const int32_t slow_step = 8;
    const int32_t base_step = 36;
    const int32_t fast_step = 120;
    int32_t step = base_step;
    if ((modifiers & kKeyModCtrl) != 0) step = slow_step;
    if ((modifiers & kKeyModShift) != 0) step = fast_step;

    int32_t dx = 0;
    int32_t dy = 0;
    switch (key) {
        case KEY_UP:
        case KEY_W:
        case KEY_K:
        case KEY_KP8:
            dy = -step;
            break;
        case KEY_DOWN:
        case KEY_S:
        case KEY_J:
        case KEY_KP2:
            dy = step;
            break;
        case KEY_LEFT:
        case KEY_A:
        case KEY_H:
        case KEY_KP4:
            dx = -step;
            break;
        case KEY_RIGHT:
        case KEY_D:
        case KEY_L:
        case KEY_KP6:
            dx = step;
            break;
        case KEY_HOME:
            dx = -step;
            dy = -step;
            break;
        case KEY_END:
            dx = -step;
            dy = step;
            break;
        case KEY_PAGEUP:
            dx = step;
            dy = -step;
            break;
        case KEY_PAGEDOWN:
            dx = step;
            dy = step;
            break;
        default:
            return false;
    }

    x = clamp_i32(x + dx, 0, static_cast<int32_t>(FB_WIDTH - 1));
    y = clamp_i32(y + dy, 0, static_cast<int32_t>(FB_HEIGHT - 1));
    return true;
}

uint32_t map_input_keycode(uint8_t key, uint32_t modifiers) {
    switch (key) {
        case KEY_ESC: return 27;
        case KEY_BACKSPACE: return 8;
        case KEY_TAB: return '\t';
        case KEY_ENTER:
        case KEY_KPENTER:
            return '\r';
        case KEY_KPMINUS: return '-';
        case KEY_KPPLUS: return '+';
        case KEY_KPDOT: return '.';
        case KEY_KPSLASH: return '/';
        case KEY_KPASTERISK: return '*';
        case KEY_INSERT: return 0x1005;
        case KEY_DELETE: return 0x1006;
        case KEY_HOME: return 0x1007;
        case KEY_END: return 0x1008;
        case KEY_PAGEUP: return 0x1009;
        case KEY_PAGEDOWN: return 0x100A;
        case KEY_UP: return 0x1001;
        case KEY_LEFT: return 0x1002;
        case KEY_RIGHT: return 0x1003;
        case KEY_DOWN: return 0x1004;
        case KEY_KP1: return '1';
        case KEY_KP2: return '2';
        case KEY_KP3: return '3';
        case KEY_KP4: return '4';
        case KEY_KP5: return '5';
        case KEY_KP6: return '6';
        case KEY_KP7: return '7';
        case KEY_KP8: return '8';
        case KEY_KP9: return '9';
        case KEY_KP0: return '0';
        default: {
            // US layout, evdev codes 0..57. The old arithmetic assumed each
            // keyboard row was alphabetical (W typed 'r', G typed 'e', X
            // typed '{') and ignored Shift, so MDI / program-name entry
            // produced garbage.
            static constexpr char kPlain[] =
                "\0\0" "1234567890-=" "\0\0" "qwertyuiop[]" "\0\0" "asdfghjkl;'`" "\0" "\\"
                "zxcvbnm,./" "\0\0\0" " ";
            static constexpr char kShift[] =
                "\0\0" "!@#$%^&*()_+" "\0\0" "QWERTYUIOP{}" "\0\0" "ASDFGHJKL:\"~" "\0" "|"
                "ZXCVBNM<>?" "\0\0\0" " ";
            static_assert(sizeof(kPlain) == 59 && sizeof(kShift) == 59, "keymap covers evdev 0..57");
            if (key < 58) {
                const char c = (modifiers & kKeyModShift) ? kShift[key] : kPlain[key];
                if (c) return static_cast<uint32_t>(static_cast<unsigned char>(c));
            }
            return key;
        }
    }
}

void pump_ui_input(ScreenManager& screen) {
    if (!kernel::g_platform) return;
    auto* input = kernel::g_platform->get_input_ops();
    if (!input) return;

    input->poll();

    static bool prev_pressed = false;
    static int32_t prev_x = 0;
    static int32_t prev_y = 0;
    static bool virtual_pointer_initialized = false;
    static bool keyboard_press_active = false;
    static int32_t virtual_x = 0;
    static int32_t virtual_y = 0;
    static uint64_t next_repeat_ns[128] = {};
    static bool prev_keys[128] = {};
    static uint32_t pointer_trace_count = 0;

    if (!virtual_pointer_initialized) {
        virtual_x = static_cast<int32_t>(FB_WIDTH / 2);
        virtual_y = static_cast<int32_t>(FB_HEIGHT / 2);
        prev_x = virtual_x;
        prev_y = virtual_y;
        virtual_pointer_initialized = true;
    }

    const uint32_t abs_range = input->pointer_abs_range();
    const uint64_t now_ns = (kernel::g_platform && kernel::g_platform->get_timer_ops())
                                ? kernel::g_platform->get_timer_ops()->get_system_time_ns()
                                : 0;

    // Modifiers as of the key state replayed so far, so a queued "Shift
    // down, 5, Shift up" types '%' even though Shift is up again by now.
    auto modifiers_now = [&]() {
        uint32_t m = 0;
        if (prev_keys[KEY_LEFTSHIFT] || prev_keys[KEY_RIGHTSHIFT]) m |= kKeyModShift;
        if (prev_keys[KEY_LEFTCTRL] || prev_keys[KEY_RIGHTCTRL]) m |= kKeyModCtrl;
        if (prev_keys[KEY_LEFTALT] || prev_keys[KEY_RIGHTALT]) m |= kKeyModAlt;
        return m;
    };

    // One key edge, from the driver queue or from the held-state diff below.
    auto key_edge = [&](uint8_t key, bool down) {
        prev_keys[key] = down;
        const uint32_t modifiers = modifiers_now();
        // While a control holds keyboard focus the keyboard types into /
        // activates that control only. Otherwise WASD/HJKL moved an
        // invisible pointer while the operator typed, and Enter/Space both
        // "clicked" at that pointer and activated the focused button.
        const bool focus_mode = ui_builder::keyboard_focus_active();
        if (!focus_mode && down && adjust_keyboard_pointer(key, modifiers, virtual_x, virtual_y)) {
            dispatch_touch(screen, EventType::TouchMove, virtual_x, virtual_y, keyboard_press_active);
            next_repeat_ns[key] = now_ns + 300000000ULL;
        }
        if (down && focus_mode && key == KEY_ESC) {
            ui_builder::clear_keyboard_focus();   // back to pointer mode
        } else if (down && !focus_mode) {
            if (key == KEY_ENTER || key == KEY_KPENTER || key == KEY_SPACE) {
                if (!keyboard_press_active) {
                    keyboard_press_active = true;
                    dispatch_touch(screen, EventType::TouchDown, virtual_x, virtual_y, true);
                }
            } else if (key == KEY_ESC && keyboard_press_active) {
                keyboard_press_active = false;
                dispatch_touch(screen, EventType::TouchUp, virtual_x, virtual_y, false);
            }
        } else if (!down) {
            next_repeat_ns[key] = 0;
            if ((key == KEY_ENTER || key == KEY_KPENTER || key == KEY_SPACE) && keyboard_press_active) {
                keyboard_press_active = false;
                dispatch_touch(screen, EventType::TouchUp, virtual_x, virtual_y, false);
            }
        }
        UIEvent event{};
        event.type = down ? EventType::KeyDown : EventType::KeyUp;
        event.key = KeyEvent{map_input_keycode(key, modifiers), modifiers, down};
        screen.handle_event(event);
    };

    // Replay every queued edge in order first. The UI samples input once
    // per 100 ms tick (about once a second while the machine view renders),
    // so a key tapped or a touch lifted between two samples used to vanish.
    kernel::hal::InputEvent qev{};
    while (input->next_event(qev)) {
        if (qev.kind == kernel::hal::InputEvent::Kind::Key) {
            if (qev.key < 128 && prev_keys[qev.key] != qev.down) key_edge(qev.key, qev.down);
            continue;
        }
        const int32_t ex = clamp_i32(normalize_touch_axis(qev.x, static_cast<int32_t>(FB_WIDTH), abs_range),
                                     0, static_cast<int32_t>(FB_WIDTH - 1));
        const int32_t ey = clamp_i32(normalize_touch_axis(qev.y, static_cast<int32_t>(FB_HEIGHT), abs_range),
                                     0, static_cast<int32_t>(FB_HEIGHT - 1));
        if (qev.down && !prev_pressed) {
            if (ex != prev_x || ey != prev_y) dispatch_touch(screen, EventType::TouchMove, ex, ey, false);
            dispatch_touch(screen, EventType::TouchDown, ex, ey, true);
        } else if (!qev.down && prev_pressed) {
            if (ex != prev_x || ey != prev_y) dispatch_touch(screen, EventType::TouchMove, ex, ey, true);
            dispatch_touch(screen, EventType::TouchUp, ex, ey, false);
        } else {
            continue;
        }
        prev_pressed = qev.down;
        prev_x = virtual_x = ex;
        prev_y = virtual_y = ey;
    }

    int32_t x = 0;
    int32_t y = 0;
    bool pressed = false;
    input->get_touch_position(x, y, pressed);
    x = normalize_touch_axis(x, static_cast<int32_t>(FB_WIDTH), abs_range);
    y = normalize_touch_axis(y, static_cast<int32_t>(FB_HEIGHT), abs_range);

    int32_t mouse_x = 0;
    int32_t mouse_y = 0;
    uint8_t mouse_buttons = 0;
    input->get_mouse_position(mouse_x, mouse_y, mouse_buttons);
    mouse_x = normalize_touch_axis(mouse_x, static_cast<int32_t>(FB_WIDTH), abs_range);
    mouse_y = normalize_touch_axis(mouse_y, static_cast<int32_t>(FB_HEIGHT), abs_range);
    mouse_x = clamp_i32(mouse_x, 0, static_cast<int32_t>(FB_WIDTH - 1));
    mouse_y = clamp_i32(mouse_y, 0, static_cast<int32_t>(FB_HEIGHT - 1));

    if (pressed) {
        virtual_x = x;
        virtual_y = y;
    } else if (mouse_x != prev_x || mouse_y != prev_y || (mouse_buttons & 0x1u) != 0) {
        virtual_x = mouse_x;
        virtual_y = mouse_y;
    }

    if (!pressed && (mouse_buttons & 0x1u)) {
        x = virtual_x;
        y = virtual_y;
        pressed = true;
    }

    if (pointer_trace_count < 12 &&
        ((pressed != prev_pressed) || (pressed && (x != prev_x || y != prev_y)))) {
        if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
            char buf[160];
            kernel::util::k_snprintf(buf, sizeof(buf),
                                     "[ui-click] pointer pressed=%lu mouse=%lu x=%ld y=%ld\n",
                                     static_cast<unsigned long>(pressed ? 1 : 0),
                                     static_cast<unsigned long>((mouse_buttons & 0x1u) ? 1 : 0),
                                     static_cast<long>(pressed ? x : virtual_x),
                                     static_cast<long>(pressed ? y : virtual_y));
            uart->puts(buf);
            ++pointer_trace_count;
        }
    }

    if (!pressed && (virtual_x != prev_x || virtual_y != prev_y)) {
        dispatch_touch(screen, EventType::TouchMove, virtual_x, virtual_y, false);
    }

    if (pressed && !prev_pressed) {
        dispatch_touch(screen, EventType::TouchDown, x, y, true);
    } else if (pressed && prev_pressed && (x != prev_x || y != prev_y)) {
        dispatch_touch(screen, EventType::TouchMove, x, y, true);
    } else if (!pressed && prev_pressed) {
        dispatch_touch(screen, EventType::TouchUp, prev_x, prev_y, false);
    }

    prev_pressed = pressed;
    prev_x = pressed ? x : virtual_x;
    prev_y = pressed ? y : virtual_y;

    // Held-state diff: catches edges from drivers that don't queue (the
    // xHCI keyboard), plus pointer-key autorepeat. Queued edges already
    // updated prev_keys, so they are not seen twice.
    for (uint8_t key = 0; key < 128; ++key) {
        const bool down = input->get_key_state(key);
        if (down != prev_keys[key]) {
            key_edge(key, down);
        } else if (down && !ui_builder::keyboard_focus_active() &&
                   key_is_keyboard_pointer_move(key) && now_ns >= next_repeat_ns[key] &&
                   adjust_keyboard_pointer(key, modifiers_now(), virtual_x, virtual_y)) {
            dispatch_touch(screen, EventType::TouchMove, virtual_x, virtual_y, keyboard_press_active);
            next_repeat_ns[key] = now_ns + 80000000ULL;
        } else if (!down) {
            next_repeat_ns[key] = 0;
        }
    }
}

void draw_boot_notice(Framebuffer& fb) {
    fb.clear(Color::Black());
    fb.draw_text(72, 72, "miniOS", Color::White(), Color::Black());
    fb.draw_text(72, 104, "loading TSV UI...", Color(160, 160, 160), Color::Black());
}

// Full repaint of the current tree. Caller holds ui_builder::lock_state():
// the clear used to run outside it and could wipe the framebuffer under a
// render in progress on another thread.
void show_main_page_locked(Framebuffer& fb) {
    fb.clear(Color::Black());
    auto& screen = screen_manager(fb);
    auto* built = ui_builder::root_widget();
    screen.set_screen(built);
    if (built) {
        screen.render();
        return;
    }
    fb.draw_text(72, 72, "TSV UI NOT_LOADED", Color::Red(), Color::Black());
    fb.draw_text(72, 104, "embedded_ui.tsv required at boot.", Color::White(), Color::Black());
}

// Push the damaged rectangle to the display. Caller holds lock_state():
// the UI loop and render_ui_once (CLI / HMI threads) used to present
// concurrently, sharing virtio-gpu's single control queue and response
// buffer — one side zeroed the other's reply ("bad response type=0x0",
// "RESOURCE_FLUSH failed") — and clear_dirty() could drop the other
// thread's damage. `heartbeat` with nothing dirty flushes a single pixel
// rather than retransferring the whole 8 MB frame every second.
void present_locked(Framebuffer& fb, bool heartbeat) {
    if (fb.is_dirty()) {
        (void)present_display_backend_rect(fb.damage_x(), fb.damage_y(),
                                           fb.damage_w(), fb.damage_h());
        fb.clear_dirty();
    } else if (heartbeat) {
        (void)present_display_backend_rect(0, 0, 1, 1);
    }
}

[[gnu::noinline]] void run_ui_main_loop(Framebuffer& fb, kernel::hal::TimerDriverOps* timer) {
    constexpr uint64_t UI_PERIOD_NS = 100000000ULL;
    uint64_t next_ns = timer ? timer->get_system_time_ns() + 750000000ULL : 0;
    if (timer) wait_until_ns(next_ns);
    else for (uint32_t i = 0; i < 1000000; ++i) kernel::util::cpu_relax();

    ui_builder::lock_state();
    show_main_page_locked(fb);
    present_locked(fb, true);
    ui_builder::unlock_state();
    ui_log_once("[ui] TSV main page active\n");

    // Present only when something was drawn, plus a 1 Hz heartbeat for
    // backends that drop the scanout without periodic flushes. The
    // heartbeat flushes one pixel when nothing is dirty; it used to
    // retransfer the full 8 MB frame every second on an idle page.
    constexpr uint32_t HEARTBEAT_TICKS = 10;  // 10 × 100 ms = 1 s
    uint32_t ticks_since_present = HEARTBEAT_TICKS;  // force first present

    for (;;) {
        if (timer) next_ns += UI_PERIOD_NS;

        auto& screen = screen_manager(fb);
        operator_api::step_demo_tick();
        ui_builder::tick();
        pump_ui_input(screen);
        // Same lock render_ui_once() takes: the CLI (ui_page, ui_dump,
        // test ui) and HMI render this tree from their own threads, and two
        // concurrent renders race on the shared GLES preview state (the
        // machine-view mesh import ran twice at once and corrupted meshes).
        ui_builder::lock_state();
        screen.render();
        ++ticks_since_present;
        const bool heartbeat = ticks_since_present >= HEARTBEAT_TICKS;
        if (fb.is_dirty() || heartbeat) {
            // Damage-rect present: only the changed rectangle is
            // transferred (a DRO digit is ~13 KB, not the 8 MB frame).
            present_locked(fb, heartbeat);
            ticks_since_present = 0;
        }
        ui_builder::unlock_state();
        g_ui_loop_iterations.fetch_add(1, std::memory_order_relaxed);

        if (timer) {
            const uint64_t now = timer->get_system_time_ns();
            if (now > next_ns) {
                // Render took longer than UI_PERIOD_NS. Still sleep for one
                // full period before the next iteration so lower-priority
                // threads on this core (cli prio=3, uart_io prio=3 vs ui
                // prio=10) actually get scheduled. The old "next_ns = now"
                // path skipped the wait entirely; on machine_view, where
                // the 1040 × 1240 software GLES1 render takes ~1 s on
                // QEMU TCG, that pegged the UI thread continuously and
                // the cli thread never woke to dispatch the queued
                // `ui_dump 6` command — `UI_DUMP_BEGIN` never reached
                // the wire and CI screenshots aborted at machine_view
                // with `timeout waiting for UI_DUMP_BEGIN`.
                next_ns = now + UI_PERIOD_NS;
            }
            wait_until_ns(next_ns);
        } else {
            kernel::util::cpu_relax();
        }
    }
}

} // namespace

void init_ui_backends() {
    ui_log_once("[ui] init_ui_backends entry\n");
    if (init_display_backend()) ui_log_once("[ui] display backend initialized\n");
    else ui_log_once("[ui] display backend FAILED - using framebuffer only\n");

    if (auto* input = kernel::g_platform ? kernel::g_platform->get_input_ops() : nullptr) {
        if (input->init()) ui_log_once("[ui] input backend initialized\n");
        else ui_log_once("[ui] input backend FAILED\n");
    } else {
        ui_log_once("[ui] input backend unavailable\n");
    }
}

void boot_ui_once() {
    auto& fb = framebuffer();
    draw_boot_notice(fb);
    ui_log_once("[ui] boot notice rendered\n");
    render_ui_once();
}

void render_ui_once() {
    auto& fb = framebuffer();
    ui_builder::lock_state();
    show_main_page_locked(fb);
    present_locked(fb, false);
    ui_builder::unlock_state();
    ui_log_once("[ui] TSV main page active\n");
}

unsigned ui_loop_iterations() { return g_ui_loop_iterations.load(std::memory_order_relaxed); }

void boot_ui_thread_entry(void* /*arg*/) {
    // Bypass ui_log_once so we can see every scheduling quantum of the UI
    // thread during bring-up. ui_log_once dedups by message-pointer and is
    // cache-sized to 16 entries; too easy to accidentally suppress the
    // first line of the thread if an earlier path has already fired it.
    if (auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr) {
        uart->puts("[ui] boot_ui_thread_entry started\n");
    }
    auto* timer = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    auto& fb = framebuffer();
    run_ui_main_loop(fb, timer);
}

} // namespace kernel::ui
