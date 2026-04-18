// SPDX-License-Identifier: MIT OR Apache-2.0
// EtherCAT State Machine (ESM) — per-slave and per-master helpers.
//
// AL-control register  = 0x0120 (2 B) — write requested state.
// AL-status  register  = 0x0130 (2 B) — read actual state.
// AL-status code       = 0x0134 (2 B) — read on ERROR_IND.
//
// State byte: 1=Init, 2=PreOp, 4=SafeOp, 8=Op, 0x10=error-ack flag.

#ifndef ETHERCAT_ESM_HPP
#define ETHERCAT_ESM_HPP

#include "cia402.hpp"
#include "pdo.hpp"

#include <array>
#include <cstdint>

namespace ethercat {

constexpr uint16_t REG_AL_CTRL       = 0x0120;
constexpr uint16_t REG_AL_STATUS     = 0x0130;
constexpr uint16_t REG_AL_STATUSCODE = 0x0134;
constexpr uint16_t REG_DL_STATUS     = 0x0110;

constexpr uint8_t AL_INIT   = 0x01;
constexpr uint8_t AL_PREOP  = 0x02;
constexpr uint8_t AL_SAFEOP = 0x04;
constexpr uint8_t AL_OP     = 0x08;
constexpr uint8_t AL_ERRACK = 0x10;

struct SlaveInfo {
    uint16_t station_addr   = 0;
    uint32_t vendor_id      = 0;
    uint32_t product_code   = 0;
    uint8_t  current_state  = AL_INIT;
    uint8_t  target_state   = AL_INIT;
    bool     present        = false;

    // PLAN 1.3 result — set by the bus_config helper if the slave's
    // {vid, pid, rev} read via 0x1018 didn't match the expected TSV
    // fingerprint. When set, the master's ESM gate holds the bus at
    // PreOp and `ec` surfaces the reason.
    bool     identity_mismatch = false;
    uint32_t observed_vid      = 0;
    uint32_t observed_pid      = 0;
    uint32_t observed_rev      = 0;

    // Per-slave CiA-402 application model. The master copies
    // `drive.controlword`/`drive.target_position` into the TX PDO each cycle
    // and copies the returned `statusword`/`actual_position` back, then calls
    // drive.step() to recompute `state` + next `controlword`.
    cia402::Drive drive{};

    // Task 1.8 — byte offsets of each mapped PDO entry within the per-slave
    // LRW slot. Populated by the master from the TSV device entry once
    // identity is known, or left at default_csp_layout() for slaves without
    // a TSV profile (e.g. fake_slave). Stored here rather than in `drive`
    // because the master owns the framing; `drive` only holds application
    // state.
    //
    // Kept as a small POD we own by value — 16 bytes, no pointer chasing
    // from inside the LRW pack/unpack hot path.
    PdoLayout pdo_layout{};

    // Raw process-data mirrors for non-CiA slaves or for PDO entries we
    // don't currently decode into `drive`. Updated in cycle_lrw()/RX.
    std::array<uint8_t, PDO_SLOT_BYTES> rx_process_data{};
    std::array<uint8_t, PDO_SLOT_BYTES> tx_process_data{};
};

// ESM step phases used by the master cyclic loop.
enum class EsmPhase : uint8_t {
    Idle,           // No transition in progress / steady in current state.
    RequestWrite,   // Build+send FPWR to AL_CTRL requesting next state.
    PollStatus,     // Build+send FPRD to AL_STATUS, await match or timeout.
    Settled,        // Transition completed for all slaves; advance master state.
    Error,          // At least one slave in ERR_IND; stay put, read code.
};

// Convert AL byte to short name for logging.
const char* al_state_name(uint8_t s) noexcept;

} // namespace ethercat

#endif
