// SPDX-License-Identifier: MIT OR Apache-2.0
//
// MDI line queue. submit() enqueues a G-code line into a FIFO; tick() pumps
// the head into channel 1 of the interpreter by writing a throwaway program
// named "__mdi__" (line + "\nM30\n") and calling Runtime::start(). This reuses
// the full G-code parser / motion pipeline without duplicating tick_channel.

#include "mdi.hpp"
#include "programs.hpp"
#include "interpreter.hpp"

namespace cnc::mdi {

Service g_service;

namespace {

constexpr const char* kMdiProgramName = "__mdi__";

void copy_clamped(char* dst, size_t dst_size, const char* src) {
    if (!dst || dst_size == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    size_t n = 0;
    while (src[n] != '\0' && n + 1 < dst_size) {
        dst[n] = src[n];
        ++n;
    }
    dst[n] = '\0';
}

bool is_blank(const char* s) {
    if (!s) return true;
    while (*s) {
        if (*s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') return false;
        ++s;
    }
    return true;
}

void build_program_text(const char* line, char* out, size_t out_size) {
    // Append a program-end M30 so the interpreter transitions ch1 to
    // Complete after executing the line.
    size_t i = 0;
    while (line[i] != '\0' && i + 8 < out_size) {
        out[i] = line[i];
        ++i;
    }
    const char suffix[] = "\nM30\n";
    for (size_t j = 0; suffix[j] != '\0' && i + 1 < out_size; ++j, ++i) {
        out[i] = suffix[j];
    }
    out[i] = '\0';
}

bool channel_idle_for_mdi() {
    const auto snap = cnc::interp::g_runtime.snapshot(MDI_CHANNEL);
    return snap.state == cnc::interp::State::Idle ||
           snap.state == cnc::interp::State::Ready ||
           snap.state == cnc::interp::State::Complete ||
           snap.state == cnc::interp::State::Fault;
}

}  // namespace

void Service::set_input(const char* text) {
    kernel::core::ScopedLock lock(lock_);
    copy_clamped(input_, sizeof(input_), text);
}

const char* Service::input() const {
    kernel::core::ScopedLock lock(lock_);
    return input_;
}

bool Service::submit(const char* line) {
    kernel::core::ScopedLock lock(lock_);
    if (is_blank(line)) {
        copy_clamped(message_, sizeof(message_), "empty line");
        status_ = Status::Error;
        return false;
    }
    if (pending_count_ >= PENDING_SIZE) {
        copy_clamped(message_, sizeof(message_), "mdi queue full");
        status_ = Status::Error;
        return false;
    }
    copy_clamped(last_, sizeof(last_), line);
    ring_[ring_head_].tick = ++tick_;
    copy_clamped(ring_[ring_head_].text, sizeof(ring_[ring_head_].text), line);
    ring_head_ = (ring_head_ + 1u) % HISTORY_SIZE;
    if (ring_size_ < HISTORY_SIZE) ++ring_size_;

    copy_clamped(pending_[pending_tail_], MAX_LINE, line);
    pending_tail_ = (pending_tail_ + 1u) % PENDING_SIZE;
    ++pending_count_;
    status_ = running_ ? Status::Running : Status::Queued;
    copy_clamped(message_, sizeof(message_), "queued");
    input_[0] = '\0';
    return true;
}

bool Service::clear() {
    kernel::core::ScopedLock lock(lock_);
    input_[0] = '\0';
    return true;
}

bool Service::abort() {
    kernel::core::ScopedLock lock(lock_);
    pending_head_ = 0;
    pending_tail_ = 0;
    pending_count_ = 0;
    (void)cnc::interp::g_runtime.stop(MDI_CHANNEL);
    running_ = false;
    status_ = Status::Idle;
    copy_clamped(message_, sizeof(message_), "aborted");
    return true;
}

Snapshot Service::snapshot() {
    kernel::core::ScopedLock lock(lock_);
    Snapshot snap{};
    copy_clamped(snap.input, sizeof(snap.input), input_);
    copy_clamped(snap.last, sizeof(snap.last), last_);
    copy_clamped(snap.message, sizeof(snap.message), message_);
    snap.status = status_;
    snap.depth = pending_count_;
    snap.history_count = ring_size_;
    for (uint32_t i = 0; i < ring_size_; ++i) {
        const uint32_t idx = (ring_head_ + HISTORY_SIZE - 1u - i) % HISTORY_SIZE;
        snap.history[i] = ring_[idx];
    }
    return snap;
}

bool Service::start_next_locked() {
    if (pending_count_ == 0) return false;

    char program_text[MAX_LINE + 8];
    build_program_text(pending_[pending_head_], program_text, sizeof(program_text));

    if (!cnc::programs::g_store.write_program(kMdiProgramName, program_text)) {
        copy_clamped(message_, sizeof(message_), "program slot full");
        status_ = Status::Error;
        pending_head_ = (pending_head_ + 1u) % PENDING_SIZE;
        --pending_count_;
        return false;
    }

    size_t idx = 0;
    if (!cnc::programs::g_store.find_by_name(kMdiProgramName, idx)) {
        copy_clamped(message_, sizeof(message_), "mdi program not found");
        status_ = Status::Error;
        pending_head_ = (pending_head_ + 1u) % PENDING_SIZE;
        --pending_count_;
        return false;
    }
    if (!cnc::programs::g_store.select(MDI_CHANNEL, idx) ||
        !cnc::programs::g_store.open_selected(MDI_CHANNEL)) {
        copy_clamped(message_, sizeof(message_), "mdi select failed");
        status_ = Status::Error;
        pending_head_ = (pending_head_ + 1u) % PENDING_SIZE;
        --pending_count_;
        return false;
    }
    if (!cnc::interp::g_runtime.start(MDI_CHANNEL)) {
        copy_clamped(message_, sizeof(message_), "mdi start refused");
        status_ = Status::Error;
        pending_head_ = (pending_head_ + 1u) % PENDING_SIZE;
        --pending_count_;
        return false;
    }
    copy_clamped(message_, sizeof(message_), "executing");
    status_ = Status::Running;
    running_ = true;
    pending_head_ = (pending_head_ + 1u) % PENDING_SIZE;
    --pending_count_;
    return true;
}

void Service::tick() {
    kernel::core::ScopedLock lock(lock_);

    if (running_) {
        if (channel_idle_for_mdi()) {
            const auto snap = cnc::interp::g_runtime.snapshot(MDI_CHANNEL);
            if (snap.state == cnc::interp::State::Fault) {
                copy_clamped(message_, sizeof(message_), "fault");
                status_ = Status::Error;
            } else {
                copy_clamped(message_, sizeof(message_), pending_count_ > 0 ? "next" : "done");
                status_ = pending_count_ > 0 ? Status::Queued : Status::Idle;
            }
            running_ = false;
        } else {
            return;
        }
    }

    if (!running_ && pending_count_ > 0 && channel_idle_for_mdi()) {
        start_next_locked();
    }
}

}  // namespace cnc::mdi
