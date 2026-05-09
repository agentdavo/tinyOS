// SPDX-License-Identifier: MIT OR Apache-2.0

#include "jobs.hpp"

#include "interpreter.hpp"
#include "programs.hpp"
#include "offsets.hpp"
#include "../machine/pallet.hpp"
#include "../util.hpp"
#include "../miniOS.hpp"

namespace cnc::jobs {

Runtime g_runtime;

namespace {

// Same lightweight TSV parser the rest of the kernel uses. Lines start
// with a record kind token followed by tab-separated key=value fields.
const char* next_line(const char* p, const char* end, char* out, size_t out_size) {
    size_t i = 0;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    while (p < end && *p != '\r' && *p != '\n' && i + 1 < out_size) out[i++] = *p++;
    out[i] = '\0';
    while (p < end && *p != '\r' && *p != '\n') ++p;
    while (p < end && (*p == '\r' || *p == '\n')) ++p;
    return p;
}

const char* field_value(const char* line, const char* key, char* scratch, size_t scratch_size) {
    if (!line || !key || !scratch || scratch_size == 0) return nullptr;
    const char* p = line;
    while (*p && *p != '\t') ++p;
    while (*p == '\t') {
        ++p;
        const char* field = p;
        while (*p && *p != '\t') ++p;
        const char* eq = field;
        while (eq < p && *eq != '=') ++eq;
        if (eq >= p) continue;
        const size_t key_len = static_cast<size_t>(eq - field);
        if (kernel::util::kstrlen(key) == key_len &&
            kernel::util::kmemcmp(field, key, key_len) == 0) {
            const char* value = eq + 1;
            const size_t value_len = static_cast<size_t>(p - value);
            const size_t copy = value_len + 1 < scratch_size ? value_len : scratch_size - 1;
            for (size_t i = 0; i < copy; ++i) scratch[i] = value[i];
            scratch[copy] = '\0';
            return scratch;
        }
    }
    return nullptr;
}

long parse_long(const char* s, long fallback = 0) {
    if (!s || !*s) return fallback;
    bool neg = false;
    if (*s == '-') { neg = true; ++s; }
    long v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); ++s; }
    return neg ? -v : v;
}

void copy_string(char* dst, size_t cap, const char* src) {
    if (!dst || cap == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    kernel::util::k_snprintf(dst, cap, "%s", src);
}

}  // namespace

bool Runtime::load_tsv(const char* buf, size_t len) noexcept {
    if (!buf || len == 0) return false;
    kernel::core::ScopedLock lock(lock_);
    for (auto& j : jobs_) j = Job{};
    job_count_ = 0;
    active_    = SIZE_MAX;
    state_     = SchedulerState::Idle;
    const char* p = buf;
    const char* end = buf + len;
    char line[256];
    while (p < end) {
        p = next_line(p, end, line, sizeof(line));
        if (line[0] == '\0' || line[0] == '#') continue;
        char* rest = line;
        while (*rest && *rest != '\t') ++rest;
        if (*rest == '\t') *rest++ = '\0';
        if (kernel::util::kstrcmp(line, "job") == 0) {
            if (job_count_ >= MAX_JOBS) continue;
            auto& j = jobs_[job_count_++];
            j = Job{};
            j.used = true;
            char a[64], b[64], c[64], d[64], e[64], f[64];
            copy_string(j.id,        sizeof(j.id),        field_value(rest, "id",      a, sizeof(a)));
            copy_string(j.pallet_id, sizeof(j.pallet_id), field_value(rest, "pallet",  b, sizeof(b)));
            const char* prog = field_value(rest, "program", c, sizeof(c));
            if (prog) copy_string(j.program, sizeof(j.program), prog);
            j.work_offset_index = static_cast<int>(parse_long(field_value(rest, "wcs",     d, sizeof(d)), -1));
            j.repeat            = static_cast<uint32_t>(parse_long(field_value(rest, "repeat",   e, sizeof(e)), 1));
            j.priority          = static_cast<uint8_t>(parse_long(field_value(rest, "priority", f, sizeof(f)), 100));
        }
    }
    return true;
}

size_t Runtime::job_count() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return job_count_;
}

const Job* Runtime::job(size_t idx) const noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= job_count_ || !jobs_[idx].used) return nullptr;
    return &jobs_[idx];
}

bool Runtime::find_job(const char* id, size_t& idx_out) const noexcept {
    if (!id) return false;
    kernel::core::ScopedLock lock(lock_);
    for (size_t i = 0; i < job_count_; ++i) {
        if (jobs_[i].used && kernel::util::kstrcmp(jobs_[i].id, id) == 0) {
            idx_out = i;
            return true;
        }
    }
    return false;
}

SchedulerState Runtime::state() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return state_;
}

bool Runtime::start_scheduler() noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (state_ == SchedulerState::Stopped) return false;
    state_ = SchedulerState::Running;
    return true;
}

bool Runtime::pause_scheduler() noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (state_ != SchedulerState::Running) return false;
    state_ = SchedulerState::Holding;
    return true;
}

bool Runtime::resume_scheduler() noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (state_ != SchedulerState::Holding) return false;
    state_ = SchedulerState::Running;
    return true;
}

bool Runtime::stop_scheduler() noexcept {
    kernel::core::ScopedLock lock(lock_);
    state_ = SchedulerState::Stopped;
    return true;
}

bool Runtime::skip_current() noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (active_ >= job_count_) return false;
    jobs_[active_].state = JobState::Skipped;
    copy_message(active_, "skipped by operator");
    active_ = SIZE_MAX;
    return true;
}

bool Runtime::abort_current() noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (active_ >= job_count_) return false;
    // Stop the interpreter on channel 0 so the program halts mid-flight.
    // The underlying motion is left to settle naturally; callers wanting
    // an instant stop should hit the operator E-stop.
    (void)cnc::interp::g_runtime.stop(0);
    jobs_[active_].state = JobState::Faulted;
    copy_message(active_, "aborted by operator");
    active_ = SIZE_MAX;
    state_  = SchedulerState::Holding;
    return true;
}

size_t Runtime::active_job() const noexcept {
    kernel::core::ScopedLock lock(lock_);
    return active_;
}

size_t Runtime::enqueue(const Job& tmpl) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (job_count_ >= MAX_JOBS) return SIZE_MAX;
    const size_t idx = job_count_++;
    jobs_[idx] = tmpl;
    jobs_[idx].used = true;
    if (jobs_[idx].state == JobState::Done ||
        jobs_[idx].state == JobState::Skipped) {
        // Operator probably wants the new job to actually run.
        jobs_[idx].state = JobState::Pending;
    }
    return idx;
}

bool Runtime::remove(size_t idx) noexcept {
    kernel::core::ScopedLock lock(lock_);
    if (idx >= job_count_ || !jobs_[idx].used) return false;
    if (idx == active_) return false; // can't remove the running job
    jobs_[idx] = Job{};
    return true;
}

void Runtime::copy_message(size_t idx, const char* msg) noexcept {
    if (idx >= job_count_) return;
    if (!msg) { jobs_[idx].last_message[0] = '\0'; return; }
    kernel::util::k_snprintf(jobs_[idx].last_message,
                             sizeof(jobs_[idx].last_message), "%s", msg);
}

bool Runtime::start_active(size_t idx) noexcept {
    if (idx >= job_count_) return false;
    auto& j = jobs_[idx];
    if (!j.used) return false;
    // Resolve the pallet (optional but recommended).
    size_t pallet_idx = SIZE_MAX;
    if (j.pallet_id[0] != '\0' &&
        !machine::pallet::g_service.find_pallet(j.pallet_id, pallet_idx)) {
        copy_message(idx, "pallet not in roster");
        j.state = JobState::Faulted;
        return false;
    }
    // Apply WCS override (job > pallet > leave alone). G-code dispatch
    // already handles G54..G59 modally; we just promote it now so the
    // first move uses the right offset.
    int wcs = j.work_offset_index;
    if (wcs < 0 && pallet_idx != SIZE_MAX) {
        const auto* pl = machine::pallet::g_service.pallet(pallet_idx);
        if (pl) wcs = pl->work_offset_index;
    }
    if (wcs >= 0 && wcs < static_cast<int>(cnc::offsets::WORK_OFFSET_COUNT)) {
        (void)cnc::offsets::g_service.select_work(static_cast<size_t>(wcs));
    }
    // Promote the pallet.
    if (pallet_idx != SIZE_MAX) {
        (void)machine::pallet::g_service.set_active_pallet(pallet_idx);
        (void)machine::pallet::g_service.set_status(pallet_idx,
            machine::pallet::PalletStatus::Cutting);
    }
    // Pick the program in the store and start the interpreter on ch0.
    size_t prog_idx = 0;
    if (!cnc::programs::g_store.find_by_name(j.program, prog_idx)) {
        copy_message(idx, "program not in store");
        j.state = JobState::Faulted;
        if (pallet_idx != SIZE_MAX) {
            (void)machine::pallet::g_service.set_status(pallet_idx,
                machine::pallet::PalletStatus::Fault);
        }
        return false;
    }
    if (!cnc::programs::g_store.select(0, prog_idx) ||
        !cnc::programs::g_store.open_selected(0) ||
        !cnc::interp::g_runtime.start(0)) {
        copy_message(idx, "interp start failed");
        j.state = JobState::Faulted;
        return false;
    }
    j.state = JobState::Running;
    copy_message(idx, "running");
    active_ = idx;
    return true;
}

void Runtime::on_interp_complete() noexcept {
    if (active_ >= job_count_) return;
    auto& j = jobs_[active_];
    ++j.completed;
    // Bump the pallet's cycle counter too — the lights-out dashboard
    // lives off this number.
    size_t pallet_idx = 0;
    const bool have_pallet =
        j.pallet_id[0] != '\0' &&
        machine::pallet::g_service.find_pallet(j.pallet_id, pallet_idx);
    if (have_pallet) (void)machine::pallet::g_service.bump_cycles_completed(pallet_idx);

    if (j.repeat == 0 || j.completed < j.repeat) {
        // More to make from the same fixture — re-arm on the same channel.
        // The program's swap macro (M310 etc.) may have moved the part out
        // and reloaded; we just re-run the same .ngc.
        size_t prog_idx = 0;
        if (cnc::programs::g_store.find_by_name(j.program, prog_idx) &&
            cnc::programs::g_store.select(0, prog_idx) &&
            cnc::programs::g_store.open_selected(0) &&
            cnc::interp::g_runtime.start(0)) {
            copy_message(active_, "re-armed for next cycle");
            return;
        }
        // Re-arm failed → fall through to the Done path so the scheduler
        // picks something else next tick.
        copy_message(active_, "re-arm failed; advancing");
    }
    j.state = JobState::Done;
    if (have_pallet) {
        (void)machine::pallet::g_service.set_status(pallet_idx,
            machine::pallet::PalletStatus::Done);
        (void)machine::pallet::g_service.clear_active_pallet();
    }
    active_ = SIZE_MAX;
}

void Runtime::on_interp_fault() noexcept {
    if (active_ >= job_count_) return;
    auto& j = jobs_[active_];
    j.state = JobState::Faulted;
    copy_message(active_, "interpreter fault");
    size_t pallet_idx = 0;
    if (j.pallet_id[0] != '\0' &&
        machine::pallet::g_service.find_pallet(j.pallet_id, pallet_idx)) {
        (void)machine::pallet::g_service.set_status(pallet_idx,
            machine::pallet::PalletStatus::Fault);
        (void)machine::pallet::g_service.clear_active_pallet();
    }
    active_ = SIZE_MAX;
    state_  = SchedulerState::Holding;
}

void Runtime::tick() noexcept {
    kernel::core::ScopedLock lock(lock_);

    // First — if a job is active, watch its interpreter state.
    if (active_ < job_count_) {
        const auto snap = cnc::interp::g_runtime.snapshot(0);
        if (snap.state == cnc::interp::State::Complete) {
            on_interp_complete();
        } else if (snap.state == cnc::interp::State::Fault) {
            on_interp_fault();
        }
        // Every other state — Running / WaitingBarrier / WaitingMacro /
        // Dwell — means we keep waiting. Returning here keeps the pallet
        // marked Cutting and the job Running.
        return;
    }

    // No job active — if we're allowed to pick, do so.
    if (state_ != SchedulerState::Running) {
        if (state_ == SchedulerState::Holding) {
            // Holding finished its current job (active_ is clear); settle
            // to Idle so the operator sees the queue is paused.
            state_ = SchedulerState::Idle;
        }
        return;
    }

    // Find the highest-priority Pending job that's not blocked. Lower
    // priority value runs first (matches POSIX nice convention).
    size_t pick = SIZE_MAX;
    uint8_t best_prio = 0xFF;
    for (size_t i = 0; i < job_count_; ++i) {
        const auto& j = jobs_[i];
        if (!j.used || j.state != JobState::Pending) continue;
        if (j.priority < best_prio) {
            best_prio = j.priority;
            pick = i;
        }
    }
    if (pick == SIZE_MAX) {
        // Drained — settle.
        state_ = SchedulerState::Idle;
        return;
    }
    if (!start_active(pick)) {
        // Job already promoted to Faulted by start_active. Stay Running
        // so the next tick picks the next-best Pending job.
    }
}

void Runtime::thread_entry(void* /*arg*/) {
    // Cooperative 250 ms tick — the scheduler is decision-making, not
    // real-time. Slower than the macro runtime's 1 ms tick because
    // pallet / program transitions happen per-second at most.
    for (;;) {
        g_runtime.tick();
        auto* tm = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
        if (tm) tm->wait_until_ns((tm->get_system_time_us() + 250'000ULL) * 1000ULL);
    }
}

}  // namespace cnc::jobs
