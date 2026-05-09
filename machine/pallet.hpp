// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Pallet table service for lights-out CNC. A pallet is a fixture-bearing
// platform that gets swapped under the cutting envelope; each pallet
// carries a different setup so the machine can run program A on pallet 1
// then program B on pallet 2 without operator intervention.
//
// This module owns the pallet roster (TSV-loaded, like toolpods/macros)
// and the per-pallet status. It does not drive a physical changer —
// that's the robot's job, mediated by an M-code macro that handshakes
// signals (request_swap, robot_clear, swap_done, ...). The macro reads
// the active pallet through this service's API; the operator and the
// HMI read its snapshot for status.
//
// Persisted via the existing setup snapshot (see cnc/setup.cpp); each
// pallet's `status` and `assigned_program` survive a reboot so a
// lights-out cycle interrupted at 02:00 picks up where it left off.

#pragma once

#include "../core.hpp"

#include <cstddef>
#include <cstdint>

namespace machine::pallet {

constexpr size_t MAX_PALLETS = 8;
constexpr size_t MAX_NAME_LEN = 32;
constexpr size_t MAX_PROGRAM_LEN = 64;

// Pallet lifecycle. The scheduler walks Loaded → Cutting → Done; the
// operator (or a fault) can flip any pallet to Hold. Robot-mediated
// swap only happens between Done (just finished) and Loaded (next part
// arriving) — Cutting pallets are always under the spindle and must
// not move.
enum class PalletStatus : uint8_t {
    Empty = 0,    // physical platform present, no fixture / part
    Loaded,       // fixture + raw stock present, ready to cut
    Cutting,      // currently under the spindle
    Done,         // finished cycle, awaiting unload
    Fault,        // last cycle tripped — operator inspection required
    Hold,         // operator-paused; scheduler skips
};

struct Pallet {
    bool     used = false;
    char     id[MAX_NAME_LEN]{};            // "P01", "P02", ...
    char     fixture_id[MAX_NAME_LEN]{};    // operator's fixture label
    uint8_t  station_index = 0;             // physical position (0..MAX_PALLETS-1)
    int      work_offset_index = -1;        // 0..5 → G54..G59; -1 = uncommitted
    char     assigned_program[MAX_PROGRAM_LEN]{};  // .ngc file in the program store
    uint32_t cycles_completed = 0;          // bumped on Done transitions
    uint32_t target_cycles = 0;             // 0 = run forever
    PalletStatus status = PalletStatus::Empty;
    char     last_message[64]{};            // operator-visible note
};

class Service {
public:
    bool   load_tsv(const char* buf, size_t len) noexcept;
    size_t pallet_count() const noexcept;
    const Pallet* pallet(size_t idx) const noexcept;
    const Pallet* pallet_by_id(const char* id) const noexcept;
    bool   find_pallet(const char* id, size_t& idx_out) const noexcept;

    // Mutations. All thread-safe via lock_.
    bool set_status(size_t idx, PalletStatus status) noexcept;
    bool set_status(const char* id, PalletStatus status) noexcept;
    bool assign_program(size_t idx, const char* program, int work_offset_index) noexcept;
    bool set_message(size_t idx, const char* msg) noexcept;
    bool bump_cycles_completed(size_t idx) noexcept;
    bool set_target_cycles(size_t idx, uint32_t target) noexcept;

    // Active pallet — the one currently under the spindle. The scheduler
    // sets this when transitioning a pallet to Cutting; the macro handshake
    // (M310 pallet_swap) reads it to know which pallet to unload. Returns
    // SIZE_MAX when no pallet is active.
    size_t active_pallet() const noexcept;
    bool   set_active_pallet(size_t idx) noexcept;
    bool   clear_active_pallet() noexcept;

    // Iterate pending work — first pallet in Loaded state past `start_idx`,
    // honouring Hold (skipped) and Fault (skipped). Returns SIZE_MAX when
    // no eligible pallet exists. The scheduler thread drives this to find
    // the next job; the CLI / HMI use it for status.
    size_t next_loaded_after(size_t start_idx) const noexcept;

    static const char* status_name(PalletStatus s) noexcept;
    static bool        parse_status(const char* token, PalletStatus& out) noexcept;

private:
    void reset() noexcept;

    Pallet pallets_[MAX_PALLETS]{};
    size_t count_ = 0;
    size_t active_ = SIZE_MAX;
    mutable kernel::core::Spinlock lock_;
};

extern Service g_service;

}  // namespace machine::pallet
