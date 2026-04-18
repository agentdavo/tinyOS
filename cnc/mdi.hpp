// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "../core.hpp"
#include <cstddef>
#include <cstdint>

namespace cnc::mdi {

constexpr size_t MAX_LINE = 96;
constexpr size_t HISTORY_SIZE = 32;
constexpr size_t PENDING_SIZE = 8;
constexpr size_t MDI_CHANNEL = 1;  // channel reserved for MDI execution

enum class Status : uint8_t {
    Idle = 0,
    Queued,
    Running,
    Error,
};

struct Entry {
    char text[MAX_LINE] = {};
    uint32_t tick = 0;
};

struct Snapshot {
    char input[MAX_LINE] = {};
    char last[MAX_LINE] = {};
    char message[MAX_LINE] = {};
    Status status = Status::Idle;
    uint32_t depth = 0;
    uint32_t history_count = 0;
    Entry history[HISTORY_SIZE]{};
};

class Service {
public:
    void set_input(const char* text);
    const char* input() const;

    bool submit(const char* line);
    bool clear();
    bool abort();

    Snapshot snapshot();

    // Advances the execution state machine: pops the head of the pending
    // FIFO into channel 1 when that channel is idle, and transitions
    // Running -> Idle when execution completes. Call periodically from
    // ui_builder::tick().
    void tick();

private:
    bool start_next_locked();  // lock_ must be held

    mutable kernel::core::Spinlock lock_;
    char input_[MAX_LINE] = {};
    char last_[MAX_LINE] = {};
    char message_[MAX_LINE] = {};
    Status status_ = Status::Idle;

    // History ring (surfaced as newest-first view in snapshot())
    Entry ring_[HISTORY_SIZE]{};
    uint32_t ring_head_ = 0;
    uint32_t ring_size_ = 0;
    uint32_t tick_ = 0;

    // Execution FIFO
    char pending_[PENDING_SIZE][MAX_LINE]{};
    uint32_t pending_head_ = 0;
    uint32_t pending_tail_ = 0;
    uint32_t pending_count_ = 0;

    bool running_ = false;
};

extern Service g_service;

}  // namespace cnc::mdi
