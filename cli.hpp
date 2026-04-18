// SPDX-License-Identifier: MIT OR Apache-2.0
// CLI subsystem for miniOS. Two threads on core 0 communicating via lock-free
// SPSC queues:
//   - uart_io thread (priority 15): owns the UART. Spins on getc_blocking,
//     pushes single bytes onto the input queue. Drains output chunks and
//     writes them via putc.
//   - cli thread (priority 3): pulls bytes from the input queue, runs the
//     line editor / history / tab completion, dispatches commands. Output
//     is buffered into Chunk objects and pushed onto the output queue.

#ifndef CLI_HPP
#define CLI_HPP

#include "core.hpp"
#include "hal.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cli {

// ---- Lock-free SPSC byte queue (single producer, single consumer) ----
// Capacity must be a power of 2. Indexed by head/tail which wrap modulo
// capacity. A byte-by-byte queue is the simplest mapping for keystroke flow,
// avoiding any per-byte allocation.
template <size_t Capacity>
class ByteQueue {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");
    std::array<uint8_t, Capacity> buf_{};
    alignas(64) std::atomic<size_t> head_{0}; // consumer
    alignas(64) std::atomic<size_t> tail_{0}; // producer
public:
    bool try_push(uint8_t b) noexcept {
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t n = (t + 1) & (Capacity - 1);
        if (n == head_.load(std::memory_order_acquire)) return false; // full
        buf_[t] = b;
        tail_.store(n, std::memory_order_release);
        return true;
    }
    bool try_pop(uint8_t& out) noexcept {
        size_t h = head_.load(std::memory_order_relaxed);
        if (h == tail_.load(std::memory_order_acquire)) return false; // empty
        out = buf_[h];
        head_.store((h + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }
    bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }
};

// ---- Output chunk + chunk pointer queue ----
struct Chunk {
    static constexpr size_t CAPACITY = 128;
    char data[CAPACITY];
    uint16_t len = 0;
};

template <size_t Capacity>
class ChunkQueue {
    static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");
    std::array<Chunk*, Capacity> items_{};
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
public:
    bool try_push(Chunk* c) noexcept {
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t n = (t + 1) & (Capacity - 1);
        if (n == head_.load(std::memory_order_acquire)) return false;
        items_[t] = c;
        tail_.store(n, std::memory_order_release);
        return true;
    }
    Chunk* try_pop() noexcept {
        size_t h = head_.load(std::memory_order_relaxed);
        if (h == tail_.load(std::memory_order_acquire)) return nullptr;
        Chunk* c = items_[h];
        head_.store((h + 1) & (Capacity - 1), std::memory_order_release);
        return c;
    }
    bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }
};

namespace io {

constexpr size_t INPUT_CAPACITY  = 256;
constexpr size_t CHUNK_POOL_SIZE = 64;   // total chunks; same as out queue cap
constexpr size_t OUTPUT_CAPACITY = 64;

// Bind the uart driver used by the uart_io thread.
void init(kernel::hal::UARTDriverOps* uart) noexcept;

// CLI-side helpers — push bytes onto the output queue. Block (yield) if the
// pool/queue is momentarily exhausted.
void put(const char* s) noexcept;
void put_n(const char* s, size_t n) noexcept;
void putc_one(char c) noexcept;

// CLI-side helper — pop one input byte, blocking via yield until available.
uint8_t get_blocking() noexcept;

// Non-blocking input check for commands that want to drive their own poll loop
// (e.g. `ttop` exits on any keystroke). Returns true and writes the byte to
// `out` if one was available; returns false otherwise. Drains the byte.
bool try_get(uint8_t& out) noexcept;

// Thread entry points. Both pinned to core 0.
[[noreturn]] void uart_io_entry(void* arg);

} // namespace io

using CommandHandler = int (*)(const char* args, kernel::hal::UARTDriverOps* uart_ops);

struct Command {
    const char* name = nullptr;
    CommandHandler handler = nullptr;
    const char* help = nullptr;
};

class CLI {
public:
    static constexpr size_t MAX_COMMANDS = 96;
    static constexpr size_t MAX_LINE_LEN = 128;
    static constexpr size_t HISTORY_SIZE = 8;

    CLI();

    bool register_command(const char* name, CommandHandler handler, const char* help_text) noexcept;

    // Reads bytes from the input SPSC queue, runs the line editor, dispatches.
    // Output is buffered through cli::io::put. Never returns.
    [[noreturn]] void run() noexcept;

    // Scheduler entry-point adapter. arg is unused; the CLI gets its UART
    // sink via cli::io::put (the uart_io thread owns the UART).
    static void thread_entry(void* arg);

    void print_help(kernel::hal::UARTDriverOps* uart) const noexcept;

    // Adapter that lets command handlers (which take a UARTDriverOps*) write
    // into the queued sink. The "uart_ops" argument is ignored by the
    // adapter's puts/putc; it just funnels into cli::io::put.
    static kernel::hal::UARTDriverOps* sink_uart() noexcept;

private:
    struct HistoryEntry { char buf[MAX_LINE_LEN]; size_t len = 0; };
    enum class EscState { NORMAL, ESC1, CSI };

    std::array<Command, MAX_COMMANDS> commands_{};
    size_t num_commands_ = 0;

    char line_[MAX_LINE_LEN] = {};
    size_t line_len_ = 0;
    size_t cursor_ = 0;

    std::array<HistoryEntry, HISTORY_SIZE> history_{};
    size_t history_count_ = 0;
    size_t history_write_ = 0;
    int history_browse_ = -1;
    HistoryEntry saved_line_{};

    EscState esc_ = EscState::NORMAL;

    // Output via queued sink (uart_io drains).
    void put(char c) noexcept;
    void write(const char* s) noexcept;
    void write_n(const char* s, size_t n) noexcept;
    void write_uint(size_t v) noexcept;
    void cursor_left(size_t n) noexcept;
    void erase_eol() noexcept;
    void prompt_line() noexcept;
    void redraw() noexcept;

    void replace_line(const char* src, size_t len) noexcept;
    void insert_char(char c) noexcept;
    void backspace() noexcept;
    void move_left() noexcept;
    void move_right() noexcept;

    void tab_complete() noexcept;
    void history_prev() noexcept;
    void history_next() noexcept;
    void submit_line() noexcept;
    void dispatch(char* line) noexcept;

    void handle_csi(char c) noexcept;
    void handle_char(char c) noexcept;

    static size_t first_token_len(const char* s, size_t len) noexcept;
    static bool starts_with(const char* s, size_t slen, const char* prefix, size_t plen) noexcept;
};

extern CLI g_cli;

} // namespace cli

#endif // CLI_HPP
