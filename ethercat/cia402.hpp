// SPDX-License-Identifier: MIT OR Apache-2.0
// CiA-402 Drive Profile — master-side data model + Finite State Automaton (FSA)
// decoder. No I/O, no EtherCAT coupling: this header is the common contract
// between the EtherCAT master (which moves bytes between the wire and a
// `Drive`) and any test harness / fake-slave that has to reflect the same
// bit-level semantics.
//
// References (bit values and transition table were cross-checked against):
//   CiA-402-1 §6.3 (statusword / controlword bitmap)
//   Synapticon "CiA-402 drive profile state machine" (2024)
//   KEBA "Comprehensive guide to the CiA-402 drive profile"
//   Myostat CM1-E user guide — CiA-402 controlword/statusword decode
//   TI AM243x industrial comms SDK — EtherCAT subdevice example 2
//
// All numeric values are `constexpr` so the compiler folds them; nothing here
// needs a separate TU.

#ifndef ETHERCAT_CIA402_HPP
#define ETHERCAT_CIA402_HPP

#include <cstdint>

namespace cia402 {

// ---------------------------------------------------------------------------
// Modes of Operation (object 0x6060 / 0x6061). Values are fixed by CiA-402.
// ---------------------------------------------------------------------------
enum class Mode : int8_t {
    NoMode               = 0,
    ProfilePosition      = 1,
    Velocity             = 2,
    ProfileVelocity      = 3,
    ProfileTorque        = 4,
    Homing               = 6,
    InterpolatedPosition = 7,
    CyclicSyncPosition   = 8,
    CyclicSyncVelocity   = 9,
    CyclicSyncTorque     = 10,
};

// ---------------------------------------------------------------------------
// Drive Finite-State Automaton (FSA). Names match CiA-402 §6.3.
// ---------------------------------------------------------------------------
enum class State : uint8_t {
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled   = 1,
    ReadyToSwitchOn    = 2,
    SwitchedOn         = 3,
    OperationEnabled   = 4,
    QuickStopActive    = 5,
    FaultReactionActive= 6,
    Fault              = 7,
};

// ---------------------------------------------------------------------------
// Controlword (OD 0x6040) bits. Bits 4,5,6 and 8 are mode-specific;
// we expose them by their generic names (the per-mode alias is the caller's
// concern — e.g. in PP mode bit 4 = "new_setpoint", in HM mode bit 4 =
// "homing_operation_start").
// ---------------------------------------------------------------------------
constexpr uint16_t CW_SWITCH_ON         = 0x0001; // bit 0
constexpr uint16_t CW_ENABLE_VOLTAGE    = 0x0002; // bit 1
constexpr uint16_t CW_QUICK_STOP        = 0x0004; // bit 2 (active LOW)
constexpr uint16_t CW_ENABLE_OPERATION  = 0x0008; // bit 3
constexpr uint16_t CW_OP_MODE_SPECIFIC4 = 0x0010; // bit 4 (e.g. new_setpoint)
constexpr uint16_t CW_OP_MODE_SPECIFIC5 = 0x0020; // bit 5 (e.g. change_set_immediately)
constexpr uint16_t CW_OP_MODE_SPECIFIC6 = 0x0040; // bit 6 (e.g. abs/rel)
constexpr uint16_t CW_FAULT_RESET       = 0x0080; // bit 7 (0 -> 1 edge)
constexpr uint16_t CW_HALT              = 0x0100; // bit 8

// Mask of the "standard" controlword command bits — bits 0..3 (the four FSA
// command bits) plus bit 7 (fault-reset). A slave decodes transitions by
// matching `cw & CW_MASK_STD` against the CW_CMD_* patterns below.
constexpr uint16_t CW_MASK_STD          = 0x008F;

// Canonical controlword patterns that drive the state machine.
// Bits 1 (enable_voltage) and 2 (quick_stop, active-low) are set in every
// "ready" pattern — clearing them requests DisableVoltage or QuickStop.
constexpr uint16_t CW_CMD_SHUTDOWN          = 0x0006; // -> ReadyToSwitchOn
constexpr uint16_t CW_CMD_SWITCH_ON         = 0x0007; // -> SwitchedOn
constexpr uint16_t CW_CMD_ENABLE_OPERATION  = 0x000F; // -> OperationEnabled
constexpr uint16_t CW_CMD_DISABLE_VOLTAGE   = 0x0000; // -> SwitchOnDisabled
constexpr uint16_t CW_CMD_QUICK_STOP        = 0x0002; // -> QuickStopActive
constexpr uint16_t CW_CMD_DISABLE_OPERATION = 0x0007; // -> SwitchedOn (from OE)
constexpr uint16_t CW_CMD_FAULT_RESET       = 0x0080; // -> SwitchOnDisabled

// ---------------------------------------------------------------------------
// Statusword (OD 0x6041) bits.
// ---------------------------------------------------------------------------
constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0001; // bit 0
constexpr uint16_t SW_SWITCHED_ON        = 0x0002; // bit 1
constexpr uint16_t SW_OP_ENABLED         = 0x0004; // bit 2
constexpr uint16_t SW_FAULT              = 0x0008; // bit 3
constexpr uint16_t SW_VOLTAGE_ENABLED    = 0x0010; // bit 4
constexpr uint16_t SW_QUICK_STOP         = 0x0020; // bit 5 (active LOW in FSA decode)
constexpr uint16_t SW_SWITCH_ON_DISABLED = 0x0040; // bit 6
constexpr uint16_t SW_WARNING            = 0x0080; // bit 7
constexpr uint16_t SW_REMOTE             = 0x0200; // bit 9
constexpr uint16_t SW_TARGET_REACHED     = 0x0400; // bit 10
constexpr uint16_t SW_INTERNAL_LIMIT     = 0x0800; // bit 11
// Bits 12-13 are mode-specific (setpoint_ack / following_error etc.).
constexpr uint16_t SW_OP_MODE_SPECIFIC12 = 0x1000;
constexpr uint16_t SW_OP_MODE_SPECIFIC13 = 0x2000;

// Mask of the bits that participate in the FSA decode (0,1,2,3,5,6).
// Bit 4 (voltage_enabled) is informational only.
constexpr uint16_t FSA_MASK = 0x006F;

// ---------------------------------------------------------------------------
// Object Dictionary index constants used by a CiA-402 drive.
// ---------------------------------------------------------------------------
constexpr uint16_t OD_ERROR_CODE         = 0x603F;
constexpr uint16_t OD_CONTROLWORD        = 0x6040;
constexpr uint16_t OD_STATUSWORD         = 0x6041;
constexpr uint16_t OD_MODES_OP           = 0x6060;
constexpr uint16_t OD_MODES_OP_DISPLAY   = 0x6061;
constexpr uint16_t OD_ACTUAL_POS         = 0x6064;
constexpr uint16_t OD_ACTUAL_VEL         = 0x606C;
constexpr uint16_t OD_TARGET_TORQUE      = 0x6071;
constexpr uint16_t OD_ACTUAL_TORQUE      = 0x6077;
constexpr uint16_t OD_TARGET_POS         = 0x607A;
constexpr uint16_t OD_HOMING_METHOD      = 0x6098;
constexpr uint16_t OD_TARGET_VEL         = 0x60FF;

// ---------------------------------------------------------------------------
// decode_state — extract the FSA state from the statusword per CiA-402 §6.3.
//
// Relevant bits after masking with FSA_MASK (0x006F):
//
//   xx0x 0000  NotReadyToSwitchOn
//   x10x 0000  SwitchOnDisabled
//   x01x 0001  ReadyToSwitchOn
//   x01x 0011  SwitchedOn
//   x01x 0111  OperationEnabled
//   x00x 0111  QuickStopActive
//   x0xx 1111  FaultReactionActive
//   x0xx 1000  Fault
//
// (x = don't care; bit 4 isn't in the mask.)
// ---------------------------------------------------------------------------
inline State decode_state(uint16_t statusword) noexcept {
    const uint16_t s = statusword & FSA_MASK;
    // Fault / FaultReactionActive first — bit 3 is set in both.
    // FaultReactionActive = bits 0..3 all set (0x000F).
    if ((s & 0x000F) == 0x000F) return State::FaultReactionActive;
    if (s & SW_FAULT)          return State::Fault;

    // Remaining decisions on bits 0,1,2,5,6.
    if (s & SW_SWITCH_ON_DISABLED) return State::SwitchOnDisabled;

    const bool rtso = (s & SW_READY_TO_SWITCH_ON) != 0;
    const bool so   = (s & SW_SWITCHED_ON) != 0;
    const bool oe   = (s & SW_OP_ENABLED) != 0;
    const bool qs   = (s & SW_QUICK_STOP) != 0; // active HIGH in statusword

    if (rtso && so && oe &&  qs) return State::OperationEnabled;
    if (rtso && so && oe && !qs) return State::QuickStopActive;
    if (rtso && so && !oe)       return State::SwitchedOn;
    if (rtso && !so && !oe)      return State::ReadyToSwitchOn;
    return State::NotReadyToSwitchOn;
}

// ---------------------------------------------------------------------------
// controlword_for_transition — canonical chain:
//
//   SwitchOnDisabled   --0x06 (shutdown)-->         ReadyToSwitchOn
//   ReadyToSwitchOn    --0x07 (switch_on)-->        SwitchedOn
//   SwitchedOn         --0x0F (enable_operation)--> OperationEnabled
//   OperationEnabled   --0x07 (disable_operation)-->SwitchedOn
//   SwitchedOn         --0x06 (shutdown)-->         ReadyToSwitchOn
//   ReadyToSwitchOn    --0x00 (disable_voltage)-->  SwitchOnDisabled
//   {any_enabled}      --0x02 (quick_stop)-->       QuickStopActive
//   Fault              --0x80 (fault_reset 0->1)--> SwitchOnDisabled
//
// When we need >1 transition to reach the target we emit the controlword for
// the *next* step; the caller re-invokes us after the drive's statusword
// catches up. That keeps the helper stateless.
// ---------------------------------------------------------------------------
inline uint16_t controlword_for_transition(State current, State target) noexcept {
    // Fault recovery always goes via SwitchOnDisabled first.
    if (current == State::Fault || current == State::FaultReactionActive) {
        return CW_CMD_FAULT_RESET;
    }
    // Requesting Fault isn't a thing; ignore it.
    if (target == State::Fault || target == State::FaultReactionActive) {
        return CW_CMD_DISABLE_VOLTAGE;
    }

    if (target == State::QuickStopActive) return CW_CMD_QUICK_STOP;

    // Walk toward the target along the canonical chain. Levels 0..3:
    //   SwitchOnDisabled(0) → ReadyToSwitchOn(1) → SwitchedOn(2) → OperationEnabled(3)
    auto level = [](State s) -> int {
        switch (s) {
            case State::SwitchOnDisabled:   return 0;
            case State::ReadyToSwitchOn:    return 1;
            case State::SwitchedOn:         return 2;
            case State::OperationEnabled:   return 3;
            default:                        return 0;
        }
    };
    const int cur_lvl = level(current);
    const int tgt_lvl = level(target);

    if (tgt_lvl > cur_lvl) {
        // Step up one notch.
        switch (cur_lvl) {
            case 0: return CW_CMD_SHUTDOWN;          // → ReadyToSwitchOn
            case 1: return CW_CMD_SWITCH_ON;         // → SwitchedOn
            case 2: return CW_CMD_ENABLE_OPERATION;  // → OperationEnabled
            default: break;
        }
    } else if (tgt_lvl < cur_lvl) {
        // Step down one notch.
        switch (cur_lvl) {
            case 3: return CW_CMD_DISABLE_OPERATION; // → SwitchedOn
            case 2: return CW_CMD_SHUTDOWN;          // → ReadyToSwitchOn
            case 1: return CW_CMD_DISABLE_VOLTAGE;   // → SwitchOnDisabled
            default: break;
        }
    }
    // Already at target level — hold the corresponding controlword.
    switch (tgt_lvl) {
        case 0: return CW_CMD_DISABLE_VOLTAGE;
        case 1: return CW_CMD_SHUTDOWN;
        case 2: return CW_CMD_SWITCH_ON;
        case 3: return CW_CMD_ENABLE_OPERATION;
    }
    return 0;
}

// Textual name for a state — convenient for CLI dumps.
inline const char* state_name(State s) noexcept {
    switch (s) {
        case State::NotReadyToSwitchOn:  return "NotReadyToSwitchOn";
        case State::SwitchOnDisabled:    return "SwitchOnDisabled";
        case State::ReadyToSwitchOn:     return "ReadyToSwitchOn";
        case State::SwitchedOn:          return "SwitchedOn";
        case State::OperationEnabled:    return "OperationEnabled";
        case State::QuickStopActive:     return "QuickStopActive";
        case State::FaultReactionActive: return "FaultReactionActive";
        case State::Fault:               return "Fault";
    }
    return "?";
}

// error_text — decodes the ClearPath-EC-specific error codes surfaced via
// 0x603F (Error Code) when statusword bit 3 (Fault) rises. Table taken from
// ClearPath-EC Software Reference Appendix B; vendor-extension codes in the
// 0xFFxx range come from the same doc's last page. Returns "?" for anything
// we haven't catalogued so callers still print the raw hex.
inline const char* error_text(uint16_t code) noexcept {
    switch (code) {
        case 0x21D2: return "Motor phase wiring fault";
        case 0x3220: return "Bus over-voltage";
        case 0x3274: return "Regen resistor over-temperature";
        case 0x32D8: return "Bus under-voltage during motion";
        case 0x4310: return "Drive over-temperature";
        case 0x619D: return "Non-volatile memory (EEPROM) fault";
        case 0x62A3: return "Parameter out of range";
        case 0x62A9: return "Parameter read-only";
        case 0x62AA: return "Parameter not supported";
        case 0x80BD: return "Distributed-clock sync lost";
        case 0x80C5: return "PDO mapping invalid";
        case 0x8321: return "Following error (position error window exceeded)";
        case 0xFFCC: return "Hardstop homing failed — torque threshold not reached";
        case 0xFFCD: return "Hardstop homing failed — backoff exceeded travel";
        case 0xFFCE: return "Hardstop homing failed — drive faulted mid-sequence";
        case 0xFFDA: return "Vendor-specific internal error";
        default:     return "?";
    }
}

// diag_text — ClearPath-EC DiagHistory TextId lookup (task 5.2 cosmetic
// follow-up). 52 entries auto-extracted from
// `ethercat/esi/Teknic_ClearPathEC_ESI.xml` DiagMessages block. Used by
// the `ec_diag` CLI when it pulls records from 0x10F3. Text is shortened
// to the first phrase before " - " so each line fits a terminal; the
// full human-facing text lives in the ESI if needed.
inline const char* diag_text(uint16_t text_id) noexcept {
    switch (text_id) {
        case 0x6380: return "Invalid EtherCAT Cycle Time";
        case 0x6381: return "SDO write failed";
        case 0xA000: return "Transition 'Pre-Op' to 'Safe-Op' failed";
        case 0xA001: return "Transition 'Pre-Op' to 'Safe-Op' failed";
        case 0xA002: return "Transition 'Pre-Op' to 'Safe-Op' failed";
        case 0xA003: return "Transition 'Pre-Op' to 'Safe-Op' failed";
        case 0xA011: return "PDO mapping error";
        case 0xA013: return "PDO mapping error";
        case 0xA0FF: return "Entered OP mode while the drive is in fault";
        case 0xAB02: return "FW Problem: type 0";
        case 0xAB03: return "FW Problem: type 1";
        case 0xAB04: return "Stack Overflow Error";
        case 0xAB06: return "Move Generator Problem";
        case 0xAB08: return "ESC EEPROM version mismatch";
        case 0xAB0B: return "Internal Power Supply Problem";
        case 0xAB0D: return "Non-Volatile memory dead";
        case 0xAB0E: return "Flash has been corrupted";
        case 0xAB0F: return "Firmware ROMsum error";
        case 0xAB12: return "ADC values saturated";
        case 0xAB1C: return "E-Stop";
        case 0xAB1D: return "Old config file version";
        case 0xAB1E: return "CPU 2 failed to communicate";
        case 0xAB23: return "Move canceled (Range)";
        case 0xAB24: return "RAS change rejected";
        case 0xAB26: return "Speed too high for RAS";
        case 0xAB28: return "Current sensor HW fault";
        case 0xAB29: return "Move canceled (Limit Switch)";
        case 0xAB2A: return "Move canceled (Soft Limit)";
        case 0xAB36: return "D-Current Heater Error";
        case 0xAB3A: return "Vector Refine Error";
        case 0xAB3C: return "Encoder noise";
        case 0xAB40: return "Motor phase overload";
        case 0xAB42: return "Motor parameter error";
        case 0xAB43: return "Hard Stop gave way";
        case 0xAB45: return "Following error limit exceeded";
        case 0xAB47: return "RMS torque limit exceeded";
        case 0xAB49: return "Step input timing error";
        case 0xAB4C: return "MagAlign distance error";
        case 0xAB4D: return "MagAlign direction error";
        case 0xAB4E: return "Sensorless startup error";
        case 0xAB50: return "Excessive motor temp";
        case 0xAB52: return "Excessive bus current";
        case 0xAB53: return "Max bus voltage exceeded";
        case 0xAB54: return "Bus voltage was lost or was too low";
        case 0xAB55: return "Excessive bus current";
        case 0xAB56: return "Power Mismatched";
        case 0xAB57: return "Encoder Index Problem";
        case 0xAB58: return "Bus under Operating Voltage";
        case 0xAB59: return "Controller Disconnected";
        case 0xAB5A: return "Interlock Triggered";
        case 0xAB5D: return "Encoder Index Problem";
        case 0xAC00: return "Errors have been cleared";
        default:     return "?";
    }
}

inline const char* mode_name(Mode m) noexcept {
    switch (m) {
        case Mode::NoMode:               return "NoMode";
        case Mode::ProfilePosition:      return "PP";
        case Mode::Velocity:             return "V";
        case Mode::ProfileVelocity:      return "PV";
        case Mode::ProfileTorque:        return "PT";
        case Mode::Homing:               return "HM";
        case Mode::InterpolatedPosition: return "IP";
        case Mode::CyclicSyncPosition:   return "CSP";
        case Mode::CyclicSyncVelocity:   return "CSV";
        case Mode::CyclicSyncTorque:     return "CST";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Drive — plain data holder with one behaviour: step() refreshes the decoded
// state from the latest statusword and recomputes controlword toward the
// currently-requested target state. The EtherCAT master is expected to
// DMA-copy `statusword`/`actual_*` in from the TxPDO and `controlword`/
// `target_*` out via the RxPDO each cycle. No locking here — the master owns
// the Drive for the duration of a cycle.
// ---------------------------------------------------------------------------
struct Drive {
    // Mode selection (0x6060 / 0x6061).
    Mode     mode_op         = Mode::NoMode;
    Mode     mode_op_display = Mode::NoMode;

    // FSA.
    State    state           = State::NotReadyToSwitchOn;
    State    target_state    = State::OperationEnabled; // what step() drives toward

    // Cyclic PDO data.
    uint16_t controlword     = 0;
    uint16_t statusword      = 0;
    int32_t  target_position = 0;
    int32_t  actual_position = 0;
    int32_t  target_velocity = 0;
    int32_t  actual_velocity = 0;
    int16_t  target_torque   = 0;
    int16_t  actual_torque   = 0;

    // 0x603F — last emergency / error code reported.
    uint16_t error_code      = 0;

    // 0x60FE:1 (digital outputs) / 0x60FD (digital inputs). Plumbed through
    // the LRW pack/unpack when the active PDO mapping includes them (per
    // TSV). Task 1.8.
    uint32_t digital_outputs = 0;
    uint32_t digital_inputs  = 0;

    // Decode the current statusword and emit the next controlword.
    void step() noexcept {
        state = decode_state(statusword);
        controlword = controlword_for_transition(state, target_state);
    }
};

} // namespace cia402

#endif // ETHERCAT_CIA402_HPP
