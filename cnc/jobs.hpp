// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Lights-out CNC job scheduler. Walks a fixed-capacity job table,
// picks the next Pending job in priority order, opens its program on
// channel 0, starts the interpreter, and waits for Complete / Fault
// before advancing. Pallet handoff (clamp release, robot pick/place,
// re-clamp) is left to the program itself — typically the program's
// last line is `M310` which is a TSV-defined macro that drives the
// robot interlock signals and waits for ack. The scheduler just
// promotes pallet status as jobs finish so the next run picks up the
// fresh fixture.
//
// Designed to run in a low-priority cooperative loop; the interpreter
// + motion threads do all the time-critical work. The scheduler just
// makes per-second decisions.

#pragma once

#include "../core.hpp"

#include <cstddef>
#include <cstdint>

namespace cnc::jobs {

constexpr size_t MAX_JOBS = 16;
constexpr size_t MAX_NAME_LEN = 32;
constexpr size_t MAX_PROGRAM_LEN = 64;

enum class JobState : uint8_t {
    Pending = 0,    // queued, awaiting scheduler pick-up
    Running,        // interpreter started; awaiting Complete / Fault
    Done,           // repeat count reached; pallet promoted to Done
    Faulted,        // interpreter Fault'd; scheduler transitioned to Holding
    Skipped,        // operator skipped manually; not retried
};

enum class SchedulerState : uint8_t {
    Idle = 0,       // no work to do (or never started)
    Running,        // actively picking + running
    Holding,        // pause-after-current; finish then settle to Idle
    Stopped,        // manual stop; current job runs to natural end
};

struct Job {
    bool     used      = false;
    char     id[MAX_NAME_LEN]{};        // "J001", "fixture_a_run"
    char     pallet_id[MAX_NAME_LEN]{}; // pallet to mount before running
    char     program[MAX_PROGRAM_LEN]{};
    int      work_offset_index = -1;    // overrides pallet's WCS when ≥0
    uint32_t repeat    = 1;
    uint32_t completed = 0;
    uint8_t  priority  = 100;           // lower = run sooner
    JobState state     = JobState::Pending;
    char     last_message[64]{};
};

class Runtime {
public:
    bool   load_tsv(const char* buf, size_t len) noexcept;
    size_t job_count() const noexcept;
    const Job* job(size_t idx) const noexcept;
    bool   find_job(const char* id, size_t& idx_out) const noexcept;

    SchedulerState state() const noexcept;
    bool start_scheduler() noexcept;     // Idle -> Running (picks up where it left off)
    bool pause_scheduler() noexcept;     // Running -> Holding (finishes current, then Idle)
    bool resume_scheduler() noexcept;    // Holding -> Running
    bool stop_scheduler() noexcept;      // -> Stopped (current keeps running)

    bool skip_current() noexcept;        // marks active job Skipped, clears active
    bool abort_current() noexcept;       // same but stops the interpreter immediately

    // Add a job at runtime. Returns the slot's index, or SIZE_MAX on full
    // queue. Used by `job_add` CLI verb and the persistence layer.
    size_t enqueue(const Job& tmpl) noexcept;
    bool   remove(size_t idx) noexcept;

    size_t active_job() const noexcept;

    // One-shot tick. The scheduler thread (created in kernel::boot::
    // create_runtime_services) calls this from a low-priority cooperative
    // loop. tick() is non-blocking — it polls interpreter state, advances
    // pallet/job status when the interpreter completes or faults, and
    // picks the next job when the channel is idle.
    void tick() noexcept;

    static void thread_entry(void* arg);

private:
    bool start_active(size_t idx) noexcept;        // private — caller holds lock_
    void on_interp_complete() noexcept;            // private — caller holds lock_
    void on_interp_fault() noexcept;               // private — caller holds lock_
    void copy_message(size_t idx, const char* msg) noexcept;

    Job             jobs_[MAX_JOBS]{};
    size_t          job_count_ = 0;
    size_t          active_    = SIZE_MAX;
    SchedulerState  state_     = SchedulerState::Idle;
    mutable kernel::core::Spinlock lock_;
};

extern Runtime g_runtime;

}  // namespace cnc::jobs
