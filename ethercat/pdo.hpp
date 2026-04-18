// SPDX-License-Identifier: MIT OR Apache-2.0
// PDO mapping configuration via CoE-SDO writes through the mailbox.
//
// Mailbox flow (EtherCAT spec, single-segment SDO):
//
//   Mailbox header (6 B):
//     len (2, LE)     — length of body that follows (CoE hdr + SDO body)
//     addr (2, LE)    — station addr for mailbox request (usually 0)
//     ch_prio (1)     — channel(low 6 bits) | priority(bits 6..7)
//     type_cnt (1)    — type (low 4 bits; 0x3 = CoE) | counter (upper 4 bits)
//
//   CoE header (2 B):
//     number:9 | reserved:3 | service:4  — service 2 = SDO Request
//
//   SDO download expedited (≤4 B) body (8 B):
//     cmd (1)  = 0x23 | 0x27 | 0x2B | 0x2F (expedited download with size)
//     index (2, LE)
//     subindex (1)
//     data (4, LE padded)
//
// CiA-402 PDO mapping we install for a drive at `station_addr`:
//   RxPDO 0x1600 entries = { 0x6040:00 (ctrl_word, 16b), 0x607A:00 (target_pos, 32b) }
//   TxPDO 0x1A00 entries = { 0x6041:00 (status_word,16b), 0x6064:00 (actual_pos,32b) }
//   SM2 assignment 0x1C12 = 0x1600
//   SM3 assignment 0x1C13 = 0x1A00

#ifndef ETHERCAT_PDO_HPP
#define ETHERCAT_PDO_HPP

#include <cstddef>
#include <cstdint>

#include "cia402.hpp"

namespace devices { struct DeviceId; }

namespace ethercat {

constexpr size_t PDO_SLOT_BYTES = 32; // bytes of process image per slave

class Master;

// Rx/Tx PDO pair the master programs into 0x1C12 / 0x1C13. The ClearPath-EC
// ESI exposes six alternate mappings (0x1600..0x1605 / 0x1A00..0x1A05), one
// per CiA-402 mode. The master picks the right pair for whatever mode the
// drive is about to operate in — CSP alone doesn't need the PP-specific
// profile acceleration entries in 0x1604, for example, and shipping a
// pair that carries both wastes LRW bytes every cycle.
struct PdoSet {
    uint16_t rx_index;   // 0x1600..0x1605
    uint16_t tx_index;   // 0x1A00..0x1A05
};

// Map a CiA-402 operating mode to the Rx/Tx PDO pair to assign. Values are
// the ClearPath-EC defaults from the ESI TwinCAT AlternativeSmMapping table
// (see devices/clearpath_ec.tsv header). Unknown / unmappable modes fall
// back to the "combined" 0x1600/0x1A00 pair, which carries every cyclic
// entry and works for any cyclic mode at the cost of extra bytes per LRW.
constexpr PdoSet select_pdo_for_mode(cia402::Mode m) noexcept {
    switch (m) {
        case cia402::Mode::CyclicSyncPosition: return {0x1601, 0x1A01};
        case cia402::Mode::CyclicSyncVelocity: return {0x1602, 0x1A02};
        case cia402::Mode::CyclicSyncTorque:
        case cia402::Mode::ProfileTorque:      return {0x1603, 0x1A03};
        case cia402::Mode::ProfilePosition:    return {0x1604, 0x1A04};
        case cia402::Mode::ProfileVelocity:
        case cia402::Mode::Velocity:           return {0x1605, 0x1A05};
        default:                               return {0x1600, 0x1A00};
    }
}

// CiA-402 object indices we use.
constexpr uint16_t OD_CONTROL_WORD     = 0x6040; // RxPDO, 16-bit
constexpr uint16_t OD_TARGET_POSITION  = 0x607A; // RxPDO, 32-bit
constexpr uint16_t OD_STATUS_WORD      = 0x6041; // TxPDO, 16-bit
constexpr uint16_t OD_ACTUAL_POSITION  = 0x6064; // TxPDO, 32-bit
constexpr uint16_t OD_RXPDO_MAP_1      = 0x1600;
constexpr uint16_t OD_TXPDO_MAP_1      = 0x1A00;
constexpr uint16_t OD_RXPDO_ASSIGN     = 0x1C12;
constexpr uint16_t OD_TXPDO_ASSIGN     = 0x1C13;

// A mapping entry as stored inside 0x1600/0x1A00:
//   bits 31..16 = object index, bits 15..8 = subindex, bits 7..0 = bit length.
constexpr uint32_t pdo_map_entry(uint16_t index, uint8_t sub, uint8_t bits) noexcept {
    return (static_cast<uint32_t>(index) << 16)
         | (static_cast<uint32_t>(sub) << 8)
         | static_cast<uint32_t>(bits);
}

// Build an SDO-expedited download frame from scratch into `buf` (which must be
// at least 60 B). Returns the finalised frame length, or 0 on failure.
//
// `data_len_bytes` must be 1, 2, or 4.
[[nodiscard]] size_t build_sdo_download(uint8_t* buf, size_t cap,
                          const uint8_t dst_mac[6], const uint8_t src_mac[6],
                          uint16_t station_addr,
                          uint16_t od_index, uint8_t od_subindex,
                          const uint8_t* data, uint8_t data_len_bytes,
                          uint8_t& counter) noexcept;

// Build an SDO "initiate upload request" (master -> slave, SM0 mailbox).
// The slave replies with the "initiate upload response" in SM1; the master
// fetches it via FPRD to the mailbox-in address (see SM1_MAILBOX_IN_ADDR
// in pdo.cpp).
[[nodiscard]] size_t build_sdo_upload_request(uint8_t* buf, size_t cap,
                                const uint8_t dst_mac[6], const uint8_t src_mac[6],
                                uint16_t station_addr,
                                uint16_t od_index, uint8_t od_subindex,
                                uint8_t& counter) noexcept;

// Parse an SDO "initiate upload response" sitting at the start of a mailbox-in
// payload (buf). Returns true on a well-formed expedited upload; fills
// `out[0..4]` with up to 4 payload bytes and sets `*out_bytes` to the actual
// payload width. Non-expedited / segmented responses return false (not
// supported yet). An SDO abort response also returns false and writes the
// 32-bit abort code to `*abort_code` when non-null.
[[nodiscard]] bool parse_sdo_upload_response(const uint8_t* buf, size_t len,
                                             uint16_t expected_index, uint8_t expected_sub,
                                             uint8_t out[4], uint8_t* out_bytes,
                                             uint32_t* abort_code) noexcept;

// Extended upload-response classifier. `kind` is filled with one of:
//   0 = Expedited (fills out[0..3], *out_bytes = 1..4)
//   1 = SegmentedInitiate (fills *total_bytes from the 32-bit size field;
//       caller must follow up with segment requests)
//   2 = Abort (fills *abort_code)
//   3 = Invalid / not-our-transfer (mailbox still empty / stale)
[[nodiscard]] bool classify_sdo_upload_response(const uint8_t* buf, size_t len,
                                  uint16_t expected_index,
                                  uint8_t  expected_sub,
                                  uint8_t* kind,
                                  uint8_t  out[4],
                                  uint8_t* out_bytes,
                                  uint32_t* total_bytes,
                                  uint32_t* abort_code) noexcept;

// Build an SDO "upload segment request". `toggle` alternates 0/1 on each
// segment per CANopen §7.2.2.2; the first segment request after the
// initiate response must use toggle=0.
[[nodiscard]] size_t build_sdo_upload_segment_request(uint8_t* buf, size_t cap,
                                        const uint8_t dst_mac[6],
                                        const uint8_t src_mac[6],
                                        uint16_t station_addr,
                                        bool toggle,
                                        uint8_t& counter) noexcept;

// Parse an SDO "upload segment response". Writes up to 7 bytes of payload
// into `out_data`, sets `*out_len` to the count, `*complete` = true if
// the c-bit is set (last segment). Returns false on SDO abort (abort
// code written to `*abort_code`) or on a malformed / empty reply.
[[nodiscard]] bool parse_sdo_upload_segment_response(const uint8_t* buf, size_t len,
                                       uint8_t  out_data[7],
                                       uint8_t* out_len,
                                       bool*    complete,
                                       uint32_t* abort_code) noexcept;

// Write the standard CiA-402 RxPDO/TxPDO mapping and SM2/SM3 assignments to the
// drive at `station_addr`. Issues ~8 SDO downloads on the master's NIC. Returns
// the number of frames transmitted (purely for logging — we don't block on ACKs
// here; the master's cyclic loop handles retry/timeout elsewhere).
//
// Legacy helper — installs the hard-coded minimal CSP pair (0x1600/0x1A00,
// controlword+target_pos / statusword+actual_pos). Retained for fallback
// when no TSV entry exists; prefer configure_cia402_axis_for_mode.
size_t configure_cia402_axis(Master& master, uint16_t station_addr) noexcept;

// Look up a device by VID/PID in g_device_db and issue its sdo_init sequence
// via m.send_sdo_download(). Returns the number of SDO frames transmitted
// (0 if no matching device entry exists). This supersedes the hardcoded
// configure_cia402_axis when a TSV entry is present.
size_t configure_from_db(Master& master, uint16_t station_addr,
                         devices::DeviceId id) noexcept;

// Install the TSV-defined PDO pair for the given CiA-402 `mode` on the drive
// at `station_addr`. Walks the device's `pdo[]` table, filters entries by the
// selected pair (select_pdo_for_mode), and issues the clear / map / assign
// sequence for both directions. Returns the number of SDO downloads sent
// (0 if the device is not in g_device_db or the pair is empty, in which
// case the caller should fall back to configure_cia402_axis).
size_t configure_cia402_axis_for_mode(Master& master, uint16_t station_addr,
                                      devices::DeviceId id,
                                      cia402::Mode mode) noexcept;

// Task 1.8 — derive byte offsets for each known CiA-402 PDO entry within the
// Rx and Tx process-data slots, based on the TSV mapping table for the
// device and the active mode. All offsets are relative to the start of the
// per-slave slot (not the whole LRW payload). `0xFF` means "not mapped in
// this PDO pair". Sizes roll up the total Rx/Tx byte counts so the master
// knows how much of the slot is live.
struct PdoLayout {
    // Rx (master -> slave) — drive inputs.
    uint8_t rx_controlword_off     = 0xFF;
    uint8_t rx_mode_op_off         = 0xFF;
    uint8_t rx_target_position_off = 0xFF;
    uint8_t rx_target_velocity_off = 0xFF;
    uint8_t rx_target_torque_off   = 0xFF;
    uint8_t rx_digital_outputs_off = 0xFF;
    uint8_t rx_size_bytes          = 0;

    // Tx (slave -> master) — drive outputs.
    uint8_t tx_statusword_off       = 0xFF;
    uint8_t tx_mode_op_display_off  = 0xFF;
    uint8_t tx_position_actual_off  = 0xFF;
    uint8_t tx_velocity_actual_off  = 0xFF;
    uint8_t tx_torque_actual_off    = 0xFF;
    uint8_t tx_error_code_off       = 0xFF;
    uint8_t tx_digital_inputs_off   = 0xFF;
    uint8_t tx_size_bytes           = 0;

    bool is_empty() const noexcept {
        return rx_size_bytes == 0 && tx_size_bytes == 0;
    }
};

// Populate `out` from the TSV device's pdo[] entries filtered by the Rx/Tx
// pair that `select_pdo_for_mode(mode)` picks. Returns false and clears `out`
// if the device is absent from g_device_db or the selected pair has no
// entries. Callers that need a safe fallback for ad-hoc slaves (e.g.
// fake_slave) use `default_csp_layout()` below.
[[nodiscard]] bool compute_pdo_layout(devices::DeviceId id, cia402::Mode mode,
                                      PdoLayout& out) noexcept;

// The 16-byte layout this master emitted before 1.8 landed — matches what
// fake_slave packs/unpacks today. Callers should prefer compute_pdo_layout
// when the slave has a TSV entry; this is the fallback.
PdoLayout default_csp_layout() noexcept;

} // namespace ethercat

#endif
