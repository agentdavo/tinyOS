// SPDX-License-Identifier: MIT OR Apache-2.0
// FakeIO slave — CiA-402 CSP servo-drive simulator.
//
// This slave behaves like a real CiA-402 drive in Cyclic Synchronous Position
// (mode 8): it maintains the Finite-State Automaton per CiA-402 §6.3 (driven
// by the controlword) and, when OperationEnabled + mode_op==CSP, echoes the
// master's target_position into actual_position with a one-cycle lag.
//
// Process-data mapping (16-byte LRW slot, master's PDO_SLOT_BYTES):
//
//   TX (master → slave, in the LRW payload we *consume*):
//     [0..1]  controlword     (u16 LE, object 0x6040)
//     [2]     mode_op         (i8,    object 0x6060)
//     [3..6]  target_position (i32 LE, object 0x607A)
//     [7..8]  DO bitmap       (u16 LE, legacy FakeIO outputs)
//     [9..15] reserved (zero)
//
//   RX (slave → master, we *write back* into the LRW payload):
//     [0..1]  statusword      (u16 LE, object 0x6041)
//     [2]     mode_op_display (i8,    object 0x6061)
//     [3..6]  actual_position (i32 LE, object 0x6064)
//     [7..8]  DI bitmap       (u16 LE, legacy FakeIO inputs)
//     [9..15] reserved (zero)
//
// The 66-byte FPRD snapshot at REG_PDI_TX_BASE (0x1000) still serves the
// original DI/uni/bip image; only the LRW exchange carries CiA-402 data.
//
// Other modes (1,2,3,4,6,7,9,10) acknowledge in mode_op_display but are
// behaviourally no-ops — CSP is the only mode with working setpoint echo.

#include "fake_slave.hpp"
#include "cia402.hpp"
#include "frame.hpp"
#include "miniOS.hpp"
#include "util.hpp"
#include "diag/jitter.hpp"

#include <cstring>

extern "C" void early_uart_puts(const char*);

namespace ethercat {

// Default fake slave NIC matches the default EtherCAT-B NIC assignment.
FakeSlave g_fake_slave(2);

namespace {

inline void wait_until_us(uint64_t target_us) {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!t) return;
    while (t->get_system_time_us() < target_us) kernel::util::cpu_relax();
}

// Address layout helpers. FPRD/FPWR carry a 32-bit address whose upper 16 bits
// are the configured (station) address and lower 16 bits are the register
// offset. BRD/BWR ignore the station portion.
inline uint16_t station_of(uint32_t a) noexcept { return static_cast<uint16_t>(a >> 16); }
inline uint16_t offset_of (uint32_t a) noexcept { return static_cast<uint16_t>(a & 0xFFFF); }

constexpr uint32_t LOG_BASE        = 0x00000000u; // first slave's logical slot
constexpr uint32_t LOG_SLOT_BYTES  = 16;          // mirrors master's PDO_SLOT_BYTES

} // namespace

void FakeSlave::rx_trampoline(int if_idx, const uint8_t* data, size_t len,
                              void* ctx) noexcept {
    (void)if_idx;
    auto* self = static_cast<FakeSlave*>(ctx);
    if (!self || !data || len == 0 || len > SCRATCH_BYTES) return;
    // Copy into the slave's writable scratch — NIC RX buffers will be re-posted
    // back to the device once poll_rx returns, so we must take a private copy
    // before the rewrite-and-retransmit step.
    std::memcpy(self->scratch_, data, len);
    self->scratch_len_ = len;
    self->frames_in_.fetch_add(1, std::memory_order_relaxed);
    self->process_frame(self->scratch_, self->scratch_len_);
}

bool FakeSlave::snapshot_txpdo(uint8_t out[TXPDO_BYTES]) noexcept {
    // Layout: DI(2) | UNI[0..15](2 each) | BIP[0..15](2 each) — all little-endian.
    put_u16_le(&out[0], dio_inputs_.load(std::memory_order_relaxed));
    for (size_t i = 0; i < 16; ++i) {
        const int16_t v = adc_unipolar_[i].load(std::memory_order_relaxed);
        put_u16_le(&out[2 + i * 2], static_cast<uint16_t>(v));
    }
    for (size_t i = 0; i < 16; ++i) {
        const int16_t v = adc_bipolar_[i].load(std::memory_order_relaxed);
        put_u16_le(&out[34 + i * 2], static_cast<uint16_t>(v));
    }
    return true;
}

// CoE SDO canned upload table (task 1.2). Only the objects the master
// currently reads via upload_sdo() need entries; everything else returns
// an empty response so callers timeout or (future) receive an SDO abort.
bool FakeSlave::build_sdo_upload_response(uint16_t index, uint8_t sub,
                                          uint8_t counter,
                                          uint8_t out[SDO_MAILBOX_LEN]) noexcept {
    uint32_t value = 0;
    uint8_t  bytes = 0;

    // 0x1018 Identity object: 1=VID, 2=PID, 3=revision, 4=serial.
    if (index == 0x1018) {
        switch (sub) {
            case 0x01: value = VENDOR_ID;    bytes = 4; break;
            case 0x02: value = PRODUCT_CODE; bytes = 4; break;
            case 0x03: value = 2;            bytes = 4; break; // Rev 2 per ESI
            case 0x04: value = 0x12345678;   bytes = 4; break; // fake serial
            default: return false;
        }
    }
    // 0x608F Position encoder resolution: 1=increments, 2=motor revs.
    // 51200 cnts/rev reflects the -Exxx ClearPath variant, which is the
    // miniOS default target (DIN 5/6 gear-quality requires the higher-
    // resolution encoder at the motor shaft even when a Renishaw BiSS-C
    // is closing the outer loop). -Rxxx at 12800 is still supported but
    // tests and fake_slave now assume -Exxx.
    else if (index == 0x608F) {
        switch (sub) {
            case 0x01: value = 51200; bytes = 4; break;
            case 0x02: value = 1;     bytes = 4; break;
            default: return false;
        }
    }
    else if (index == 0x6041 && sub == 0) {
        value = statusword_.load(std::memory_order_relaxed);
        bytes = 2;
    }
    // Phase 5 touch-probe (0x60B8/B9/BA-BD, 0x60D5-D8). Pure stub values so
    // the CLI surface round-trips; no edge emulation today.
    else if (index == 0x60B8 && sub == 0) { value = 0x1713; bytes = 2; } // default mask
    else if (index == 0x60B9 && sub == 0) { value = 0;      bytes = 2; } // status
    else if ((index == 0x60BA || index == 0x60BB ||
              index == 0x60BC || index == 0x60BD) && sub == 0) {
        value = 0; bytes = 4;                                             // captured position
    }
    else if ((index == 0x60D5 || index == 0x60D6 ||
              index == 0x60D7 || index == 0x60D8) && sub == 0) {
        value = 0; bytes = 2;                                             // edge counters
    }
    // Phase 5 tuning block — representative default magnitudes so the
    // read surface shows sensible numbers. Real drives keep per-axis
    // calibrated values in NVM.
    else if (index == 0x2143 && sub == 0) { value =  500; bytes = 2; }
    else if (index == 0x2146 && sub == 0) { value = 2000; bytes = 2; }
    else if (index == 0x2147 && sub == 0) { value = 1500; bytes = 2; }
    else if (index == 0x2148 && sub == 0) { value =  200; bytes = 2; }
    else if (index == 0x2149 && sub == 0) { value =    0; bytes = 2; }
    else if (index == 0x214A && sub == 0) { value =    0; bytes = 2; }
    else if (index == 0x214B && sub == 0) { value =    0; bytes = 2; }
    else if (index == 0x214D && sub == 0) { value =    0; bytes = 2; }
    else if (index == 0x214F && sub == 0) { value =    0; bytes = 2; }
    else if (index == 0x215D && sub == 0) { value =   50; bytes = 2; }
    else if (index == 0x2039 && sub == 0) { value =    0; bytes = 2; }
    // Phase 4 fault/stop/safety objects — values mirror the TSV defaults
    // that `configure_from_db` writes at PREOP so `ec_safety` reads back
    // what we expect end-to-end. On real hardware these come from the
    // drive's own NVM / last-applied sdo_init.
    else if (index == 0x605E && sub == 0) { value = 0xFFFFu;     bytes = 2; } // -1 dyn-brake
    else if (index == 0x6065 && sub == 0) { value = 0xFFFFFFFFu; bytes = 4; } // window disabled
    else if (index == 0x6066 && sub == 0) { value = 0;           bytes = 2; } // timeout 0 ms
    else if (index == 0x231A && sub == 0) { value = 10;          bytes = 2; } // overspeed 10 ms
    else if (index == 0x2170 && sub == 0) { value = 0;           bytes = 2; } // brake 0 ms
    // 0x60E3 Supported Homing Methods (task 2.1). Sub 0 is the count,
    // subs 1..N each publish one CiA-402 method number. Advertise a
    // reasonable ClearPath-inspired set: standard 1, 2, 17, 18, 33, 34,
    // 37 plus Teknic's hardstop -1 and -2. Exactly 9 entries.
    else if (index == 0x60E3) {
        static const int8_t methods[] = { 1, 2, 17, 18, 33, 34, 37, -1, -2 };
        constexpr uint8_t count = sizeof(methods) / sizeof(methods[0]);
        if (sub == 0) { value = count; bytes = 1; }
        else if (sub >= 1 && sub <= count) {
            value = static_cast<uint32_t>(static_cast<uint8_t>(methods[sub - 1]));
            bytes = 1;
        } else {
            return false;
        }
    }
    // 0x1C32 / 0x1C33 SM Output / Input parameters (task 3.1). Values mirror
    // the ClearPath-EC ESI defaults so CLI / CI can confirm the slave is
    // reachable and the numbers line up (440 / 62500 / 15000 / 5200 ns).
    else if (index == 0x1C32 || index == 0x1C33) {
        switch (sub) {
            case 0x03: value = 0;      bytes = 4; break; // Sync0 cycle time (ns)
            case 0x05: value = 250000; bytes = 4; break; // Min cycle (250 µs)
            case 0x06: value = 15000;  bytes = 4; break; // Calc+copy time
            case 0x09: value = 5200;   bytes = 4; break; // Delay time
            default: return false;
        }
    }
    // Try segmented payload for the objects we stage that way.
    else if (stage_sdo_segmented_payload(index, sub)) {
        // Initiate upload response (segmented): cmd=0x41 (scs=2, e=0, s=1),
        // data payload = total size u32.
        std::memset(out, 0, SDO_MAILBOX_LEN);
        constexpr uint8_t COE_TYPE            = 0x03;
        constexpr uint8_t COE_SERVICE_SDO_RSP = 0x03;
        put_u16_le(&out[0], 10);
        out[4] = 0;
        out[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);
        put_u16_le(&out[6],
            static_cast<uint16_t>((COE_SERVICE_SDO_RSP & 0xF) << 12));
        out[8]  = 0x41;
        put_u16_le(&out[9], index);
        out[11] = sub;
        put_u32_le(&out[12], sdo_seg_total_);
        return true;
    }
    else {
        return false;
    }

    std::memset(out, 0, SDO_MAILBOX_LEN);

    // Mailbox header [0..5]: body_len=10, station=0, ch/prio=0, cnt|type.
    constexpr uint8_t COE_TYPE            = 0x03;
    constexpr uint8_t COE_SERVICE_SDO_RSP = 0x03;
    put_u16_le(&out[0], 10);
    put_u16_le(&out[2], 0x0000);
    out[4] = 0x00;
    out[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);

    // CoE header [6..7]: service=3 (SDO Response).
    uint16_t coe = static_cast<uint16_t>((COE_SERVICE_SDO_RSP & 0xF) << 12);
    put_u16_le(&out[6], coe);

    // SDO [8..15]: expedited upload response.
    //   cmd = 0x43 | (n<<2) where n = 4 - bytes (0x4F = 1B, 0x4B = 2B, 0x43 = 4B).
    const uint8_t n = static_cast<uint8_t>(4 - bytes);
    out[8]  = static_cast<uint8_t>(0x43 | (n << 2));
    put_u16_le(&out[9], index);
    out[11] = sub;
    put_u32_le(&out[12], value);
    return true;
}

// Segmented-SDO support for fake_slave (task 1.2 follow-up). Covers the two
// objects that exist on a real ClearPath but don't fit expedited:
//   0x1008 device name — string "miniOS-fake-servo"
//   0x10F3:2 diag-history newest-pointer (u32) — expedited so not here
//   0x10F3:6 diag-history entry 0 — 20-byte record (id/flags/port/text)
bool FakeSlave::stage_sdo_segmented_payload(uint16_t index, uint8_t sub) noexcept {
    if (sdo_seg_active_) return false; // one in flight

    if (index == 0x1008 && sub == 0) {
        static const char name[] = "miniOS-fake-servo";
        constexpr uint16_t len = sizeof(name) - 1;
        std::memcpy(sdo_seg_payload_, name, len);
        sdo_seg_total_ = len;
    } else if (index == 0x10F3 && sub == 6) {
        // Canned diag entry — ETG.1020 DiagHistory format: id(2) + flags(2)
        // + port(2) + TextId(2) + params(0) + text ascii NUL-padded.
        static const uint8_t entry[] = {
            0x01, 0x00,                     // id
            0x01, 0x00,                     // flags: Error, newest
            0x00, 0x00,                     // port
            0x58, 0xAB,                     // TextId=0xAB58 (Bus under Operating Voltage)
            // text (matches diag_text() lookup)
            'B','u','s',' ','u','n','d','e','r',' ','V',0, 0,
        };
        std::memcpy(sdo_seg_payload_, entry, sizeof(entry));
        sdo_seg_total_ = static_cast<uint16_t>(sizeof(entry));
    } else {
        return false;
    }

    sdo_seg_sent_   = 0;
    sdo_seg_toggle_ = false;
    sdo_seg_active_ = true;
    return true;
}

bool FakeSlave::build_sdo_segment_response(uint8_t counter,
                                           uint8_t out[SDO_MAILBOX_LEN]) noexcept {
    if (!sdo_seg_active_) return false;

    const uint16_t remaining = static_cast<uint16_t>(sdo_seg_total_ - sdo_seg_sent_);
    const uint8_t  take      = static_cast<uint8_t>(remaining > 7 ? 7 : remaining);
    const bool     complete  = take == remaining;

    std::memset(out, 0, SDO_MAILBOX_LEN);
    constexpr uint8_t COE_TYPE            = 0x03;
    constexpr uint8_t COE_SERVICE_SDO_RSP = 0x03;
    put_u16_le(&out[0], 10);
    out[4] = 0;
    out[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);
    put_u16_le(&out[6],
        static_cast<uint16_t>((COE_SERVICE_SDO_RSP & 0xF) << 12));

    // Segment response: scs=0 (0x00..0x1F), n = 7-take, c = 1 if last,
    // t = toggle bit.
    const uint8_t n = static_cast<uint8_t>(7 - take);
    uint8_t cmd = static_cast<uint8_t>((n & 0x07) << 1);
    if (complete) cmd |= 0x10;
    if (sdo_seg_toggle_) cmd |= 0x01;
    out[8] = cmd;
    std::memcpy(&out[9], &sdo_seg_payload_[sdo_seg_sent_], take);

    sdo_seg_sent_   = static_cast<uint16_t>(sdo_seg_sent_ + take);
    sdo_seg_toggle_ = !sdo_seg_toggle_;
    if (complete) sdo_seg_active_ = false;
    return true;
}

void FakeSlave::apply_rxpdo(const uint8_t* in, size_t len) noexcept {
    // Legacy FPWR RX-PDO slot at 0x0F00 — the RXPDO_BYTES (=2) image is just
    // the DO bitmap. The CiA-402 LRW slot has its own 16-byte layout decoded
    // inside process_frame()/Cmd::LRW — do not touch CiA-402 state here.
    if (!in || len < RXPDO_BYTES) return;
    dio_outputs_.store(get_u16_le(in), std::memory_order_relaxed);
}

// Decode a full 16-byte CiA-402 TX PDO slot (master → slave direction) and
// update the pending FSA inputs. Called once per LRW datagram.
static inline int32_t i32_le(const uint8_t* p) noexcept {
    return static_cast<int32_t>(get_u32_le(p));
}
static inline void put_i32_le(uint8_t* p, int32_t v) noexcept {
    put_u32_le(p, static_cast<uint32_t>(v));
}

// Advance the CiA-402 FSA one cycle. Called after a new controlword has been
// latched (either from LRW or from cia_inject_controlword). Pure function of
// the latched inputs — no frame/IO side effects.
void FakeSlave::tick_cia402() noexcept {
    using cia402::State;

    const uint16_t cw = controlword_.load(std::memory_order_relaxed);
    State st = static_cast<State>(cia_state_.load(std::memory_order_relaxed));

    // Test hook: `fault_inject` from the CLI latches a force-fault counter.
    // While the counter is >0 we drop the FSA into Fault each tick and
    // decrement, suppressing CW_FAULT_RESET pulses from the master. Once
    // the counter hits zero the drive resumes normal behaviour (and will
    // respond to CW_FAULT_RESET on the next master cycle).
    uint16_t ff = force_fault_ticks_.load(std::memory_order_acquire);
    const bool forcing_fault = ff > 0;
    if (forcing_fault) {
        st = State::Fault;
        force_fault_ticks_.store(static_cast<uint16_t>(ff - 1),
                                 std::memory_order_release);
    }

    // Auto NotReadyToSwitchOn → SwitchOnDisabled on first cycle.
    if (st == State::NotReadyToSwitchOn) {
        st = State::SwitchOnDisabled;
    }

    // Fault reset — bit 7 returns Fault to SwitchOnDisabled. Suppressed
    // while force_fault_ticks_ is still active, so motion can observe the
    // latched fault state across its 250 µs cycle.
    if (!forcing_fault && st == State::Fault
        && (cw & cia402::CW_FAULT_RESET)) {
        st = State::SwitchOnDisabled;
        homing_attained_.store(0, std::memory_order_relaxed);
    }

    // Canonical state-machine transitions driven by CW bits 0..3 + bit 7.
    // Table per CiA-402 §6.3 / Synapticon academy diagram. Bit 7 (fault_reset)
    // handled above; the remaining transitions gate on bits 0..3 only.
    const uint16_t cmd = cw & 0x000Fu;
    switch (st) {
        case State::SwitchOnDisabled:
            // 0x06 (EnableVoltage + QuickStop) → ReadyToSwitchOn.
            if ((cmd & 0x0007) == 0x0006) st = State::ReadyToSwitchOn;
            break;
        case State::ReadyToSwitchOn:
            // 0x07 → SwitchedOn.
            if ((cmd & 0x000F) == 0x0007) st = State::SwitchedOn;
            // 0x00 (disable_voltage) → SwitchOnDisabled.
            else if ((cmd & 0x0002) == 0) st = State::SwitchOnDisabled;
            break;
        case State::SwitchedOn:
            // 0x0F → OperationEnabled.
            if ((cmd & 0x000F) == 0x000F) st = State::OperationEnabled;
            else if ((cmd & 0x000F) == 0x0006) st = State::ReadyToSwitchOn;
            else if ((cmd & 0x0002) == 0)     st = State::SwitchOnDisabled;
            break;
        case State::OperationEnabled:
            if ((cmd & 0x000F) == 0x0007)      st = State::SwitchedOn;
            else if ((cmd & 0x000F) == 0x0006) st = State::ReadyToSwitchOn;
            else if ((cmd & 0x0004) == 0)      st = State::QuickStopActive;
            else if ((cmd & 0x0002) == 0)      st = State::SwitchOnDisabled;
            break;
        case State::QuickStopActive:
            // Only way out is disable_voltage → SwitchOnDisabled (per §6.3).
            if ((cmd & 0x0002) == 0) st = State::SwitchOnDisabled;
            break;
        default: break;
    }
    cia_state_.store(static_cast<uint8_t>(st), std::memory_order_relaxed);

    if (st == State::OperationEnabled
        && static_cast<int8_t>(mode_op_.load(std::memory_order_relaxed))
               == static_cast<int8_t>(cia402::Mode::Homing)
        && (cw & cia402::CW_OP_MODE_SPECIFIC4)) {
        homing_attained_.store(1, std::memory_order_relaxed);
    }

    // Build the statusword that reflects this state.
    // VOLTAGE_ENABLED is always 1 in the simulator — there's no physical DC
    // bus we could "lose" in QEMU, and CiA-402 §6.3.2 allows a drive to hold
    // it high unconditionally when powered.
    uint16_t sw = cia402::SW_VOLTAGE_ENABLED;
    switch (st) {
        case State::NotReadyToSwitchOn:
            sw |= 0; break;
        case State::SwitchOnDisabled:
            sw |= cia402::SW_SWITCH_ON_DISABLED;
            break;
        case State::ReadyToSwitchOn:
            sw |= cia402::SW_READY_TO_SWITCH_ON | cia402::SW_QUICK_STOP;
            break;
        case State::SwitchedOn:
            sw |= cia402::SW_READY_TO_SWITCH_ON | cia402::SW_SWITCHED_ON
                | cia402::SW_QUICK_STOP;
            break;
        case State::OperationEnabled:
            sw |= cia402::SW_READY_TO_SWITCH_ON | cia402::SW_SWITCHED_ON
                | cia402::SW_OP_ENABLED | cia402::SW_QUICK_STOP;
            break;
        case State::QuickStopActive:
            sw |= cia402::SW_READY_TO_SWITCH_ON | cia402::SW_SWITCHED_ON
                | cia402::SW_OP_ENABLED; // QS bit cleared => quick-stop active
            break;
        case State::FaultReactionActive:
            sw |= cia402::SW_READY_TO_SWITCH_ON | cia402::SW_SWITCHED_ON
                | cia402::SW_OP_ENABLED | cia402::SW_FAULT;
            break;
        case State::Fault:
            sw |= cia402::SW_FAULT;
            break;
    }
    if (homing_attained_.load(std::memory_order_relaxed)) {
        sw |= cia402::SW_OP_MODE_SPECIFIC12;
    }
    statusword_.store(sw, std::memory_order_relaxed);

    // Mode-of-operation display echoes the requested mode unconditionally;
    // only MODE_CSP (=8) has full behavioural support.
    mode_op_display_.store(mode_op_.load(std::memory_order_relaxed),
                           std::memory_order_relaxed);

    // CSP setpoint echo with one-cycle lag.
    const int8_t mode = static_cast<int8_t>(mode_op_.load(std::memory_order_relaxed));
    if (st == State::OperationEnabled && mode == 8 /* MODE_CSP */) {
        // Commit the target staged on the previous cycle, then stage the
        // newly-latched target for the next cycle.
        const uint32_t pending = csp_pending_.load(std::memory_order_relaxed);
        actual_pos_.store(pending, std::memory_order_relaxed);
        csp_pending_.store(target_pos_.load(std::memory_order_relaxed),
                           std::memory_order_relaxed);
    }
    // When not OP-enabled, actual_position is frozen (we simply don't write).
}

void FakeSlave::handle_al_ctrl_write(const uint8_t* d, uint16_t len) noexcept {
    if (!d || len < 1) return;
    const uint8_t tgt = d[0] & 0x0F;
    al_target_.store(tgt, std::memory_order_relaxed);
    // Real slaves would gate transitions on init/PDO mapping; for FakeIO we
    // accept every requested transition immediately.
    al_state_.store(tgt, std::memory_order_relaxed);
}

void FakeSlave::process_frame(uint8_t* buf, size_t len) noexcept {
    MutableFrameParser fp(buf, len);
    if (!fp.valid()) return;

    bool any_serviced = false;
    MutableFrameParser::Datagram dg;
    while (fp.next(dg)) {
        switch (dg.cmd) {
        case Cmd::BRD: {
            // Broadcast read — fill in our value if it's a known register.
            if (offset_of(dg.address) == REG_DL_STATUS && dg.data_len >= 2) {
                // Bit 0 = link up; bit 8 = port 0 link up. Just say "up".
                dg.data[0] = 0x01;
                dg.data[1] = 0x00;
            } else if (offset_of(dg.address) == REG_AL_STATUS && dg.data_len >= 2) {
                dg.data[0] = al_state_.load(std::memory_order_relaxed);
                dg.data[1] = 0x00;
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::BWR: {
            // Broadcast write — accept and bump WKC.
            if (offset_of(dg.address) == REG_AL_CTRL) {
                handle_al_ctrl_write(dg.data, dg.data_len);
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::FPRD: {
            if (station_of(dg.address) != STATION_ADDR) break;
            const uint16_t off = offset_of(dg.address);
            if (off == REG_AL_STATUS && dg.data_len >= 2) {
                dg.data[0] = al_state_.load(std::memory_order_relaxed);
                dg.data[1] = 0x00;
            } else if (off == REG_AL_CTRL && dg.data_len >= 2) {
                dg.data[0] = al_target_.load(std::memory_order_relaxed);
                dg.data[1] = 0x00;
            } else if (off == REG_AL_STATUSCODE && dg.data_len >= 2) {
                dg.data[0] = 0x00;
                dg.data[1] = 0x00;
            } else if (off == 0x0910 && dg.data_len >= 8) {
                uint64_t now_ns = 0;
                auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
                if (t) now_ns = t->get_system_time_ns();
                for (uint8_t i = 0; i < 8; ++i) {
                    dg.data[i] = static_cast<uint8_t>((now_ns >> (8u * i)) & 0xFFu);
                }
            } else if (off == 0x1400 && dg.data_len >= SDO_MAILBOX_LEN) {
                // SM1 mailbox-in read (task 1.2). Drain the staged upload
                // response if we have one; otherwise return an all-zero body
                // and the master keeps polling.
                if (sdo_response_ready_) {
                    std::memcpy(dg.data, sdo_response_buf_, SDO_MAILBOX_LEN);
                    sdo_response_ready_ = false;
                } else {
                    std::memset(dg.data, 0, SDO_MAILBOX_LEN);
                }
            } else if (off >= REG_PDI_TX_BASE && off < REG_PDI_TX_BASE + TXPDO_BYTES) {
                const uint16_t skip = static_cast<uint16_t>(off - REG_PDI_TX_BASE);
                uint8_t tx[TXPDO_BYTES];
                snapshot_txpdo(tx);
                const uint16_t avail = static_cast<uint16_t>(TXPDO_BYTES - skip);
                const uint16_t cpy   = dg.data_len < avail ? dg.data_len : avail;
                std::memcpy(dg.data, &tx[skip], cpy);
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::FPWR: {
            if (station_of(dg.address) != STATION_ADDR) break;
            const uint16_t off = offset_of(dg.address);
            if (off == REG_AL_CTRL) {
                handle_al_ctrl_write(dg.data, dg.data_len);
            } else if (off == 0x1000 && dg.data_len >= SDO_MAILBOX_LEN) {
                // SM0 mailbox-out write (task 1.2). If it's an SDO upload
                // request (CoE service=2, SDO cmd byte 0x40..0x4F with the
                // low 5 bits of 0x40), stage a canned response in SM1. The
                // master will FPRD SM1 on its next service_sdo_upload pass.
                const uint8_t* mb  = dg.data;
                const uint16_t coe = get_u16_le(&mb[6]);
                const uint8_t  svc = static_cast<uint8_t>((coe >> 12) & 0xF);
                const uint8_t  cmd = mb[8];
                if (svc == /*SDO Request*/ 0x02 && (cmd & 0xE0) == 0x40) {
                    const uint16_t idx = get_u16_le(&mb[9]);
                    const uint8_t  sub = mb[11];
                    const uint8_t  cnt = static_cast<uint8_t>((mb[5] >> 4) & 0x7);
                    if (build_sdo_upload_response(idx, sub, cnt, sdo_response_buf_)) {
                        sdo_response_ready_ = true;
                    }
                } else if (svc == 0x02 && (cmd & 0xE0) == 0x60 && sdo_seg_active_) {
                    // Upload segment request — stage the next segment.
                    const uint8_t cnt = static_cast<uint8_t>((mb[5] >> 4) & 0x7);
                    if (build_sdo_segment_response(cnt, sdo_response_buf_)) {
                        sdo_response_ready_ = true;
                    }
                }
            } else if (off >= REG_PDI_RX_BASE && off < REG_PDI_RX_BASE + RXPDO_BYTES) {
                apply_rxpdo(dg.data, dg.data_len);
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::LRD: {
            // Master reads our slot — copy DI bitmap into the first 2 bytes
            // of the addressed window. We're slave 0 (logical addr 0).
            if (dg.data_len >= 2 && dg.address == LOG_BASE) {
                const uint16_t di = dio_inputs_.load(std::memory_order_relaxed);
                put_u16_le(dg.data, di);
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::LWR: {
            // Master writes our DO slot.
            if (dg.data_len >= 2 && dg.address == LOG_BASE) {
                apply_rxpdo(dg.data, dg.data_len);
            }
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 1);
            any_serviced = true;
            break;
        }
        case Cmd::LRW: {
            // CiA-402 16-byte process-data exchange. Parse the TX PDO layout
            // from the datagram payload, advance the FSA one tick, then
            // overwrite the same bytes with the RX PDO layout.
            if (dg.address == LOG_BASE && dg.data_len >= 11) {
                // ----- TX PDO in (master → slave) -----
                // Task 1.8 — DO is a u32 at offset 7 to match the master's
                // pack (CiA-402 0x60FE:1 is 32-bit). We keep the low 16 bits
                // as our internal DO bitmap since the fake slave only models
                // 16 output channels.
                const uint16_t cw     = get_u16_le(&dg.data[0]);
                const int8_t   mode   = static_cast<int8_t>(dg.data[2]);
                const int32_t  tpos   = i32_le(&dg.data[3]);
                const uint32_t do_bm  = get_u32_le(&dg.data[7]);
                controlword_.store(cw,   std::memory_order_relaxed);
                mode_op_.store(static_cast<uint8_t>(mode), std::memory_order_relaxed);
                target_pos_.store(static_cast<uint32_t>(tpos), std::memory_order_relaxed);
                dio_outputs_.store(static_cast<uint16_t>(do_bm & 0xFFFFu),
                                   std::memory_order_relaxed);

                // Advance the FSA & CSP pipeline using the just-latched inputs.
                tick_cia402();

                // ----- RX PDO out (slave → master) -----
                put_u16_le(&dg.data[0], statusword_.load(std::memory_order_relaxed));
                dg.data[2] = static_cast<uint8_t>(mode_op_display_.load(std::memory_order_relaxed));
                put_i32_le(&dg.data[3],
                           static_cast<int32_t>(actual_pos_.load(std::memory_order_relaxed)));
                // Task 1.8 — digital inputs as u32 at offset 7 (per CiA-402
                // 0x60FD width). DI bitmap is 16-bit; zero-extend into the
                // upper two bytes so the master reads a clean u32.
                put_u32_le(&dg.data[7],
                           static_cast<uint32_t>(dio_inputs_.load(std::memory_order_relaxed)));
                // Task 1.5 — publish 0x603F (error code) at offset 11.
                // When the FSA is in Fault (or was forced into Fault via
                // cia_inject_fault), advertise 0x8321 = "Following Error";
                // it's a plausible CiA-402-encoded value that decodes to a
                // recognisable string via cia402::error_text. Anything else
                // reports 0 so the master sees "healthy".
                using cia402::State;
                const auto st_now = static_cast<State>(
                    cia_state_.load(std::memory_order_relaxed));
                const uint16_t err_code =
                    (st_now == State::Fault ||
                     st_now == State::FaultReactionActive) ? 0x8321 : 0x0000;
                put_u16_le(&dg.data[11], err_code);
                // Bytes [13..data_len) are reserved — zero them so stale master
                // image data can't masquerade as slave output.
                const uint16_t tail_off = 13;
                if (dg.data_len > tail_off) {
                    std::memset(&dg.data[tail_off], 0, dg.data_len - tail_off);
                }
            } else if (dg.data_len >= 2 && dg.address == LOG_BASE) {
                // Short slot (<9 B): fall back to the legacy DIO-only path so
                // we stay backwards-compatible with any master that still
                // allocates a 2-byte slot.
                apply_rxpdo(dg.data, dg.data_len);
                const uint16_t di = dio_inputs_.load(std::memory_order_relaxed);
                put_u16_le(dg.data, di);
            }
            // LRW counts as one read + one write; bump WKC by 3 (2 = read+write
            // both serviced, 1 per direction). The master only checks "non-zero
            // means something happened" so anything > 0 works; we follow the
            // ETG.1000 convention of 3.
            MutableFrameParser::inc_wkc(dg.wkc_ptr, 3);
            any_serviced = true;
            break;
        }
        default:
            // NOP / APRD / APWR / APRW / FPRW / BRW — pass through unchanged.
            break;
        }
        if (!dg.more) break;
    }

    if (any_serviced) {
        wkc_serviced_.fetch_add(1, std::memory_order_relaxed);
    }

    if (net_ && net_->send_packet(nic_idx_, buf, len)) {
        frames_out_.fetch_add(1, std::memory_order_relaxed);
    }
}

void FakeSlave::run_stimulus() noexcept {
    // Generates moving values so a passive `fake` snapshot shows life. Called
    // every loop iteration (100 µs); we throttle on tick count so the visible
    // changes happen at sub-Hz.
    ++stim_tick_;
    // Every 1000 iterations (~100 ms): bump the unipolar counter, walk the
    // DI bit pattern, and step the bipolar triangle wave one notch.
    if ((stim_tick_ % 1000) != 0) return;

    const uint16_t prev_uni = static_cast<uint16_t>(adc_unipolar_[0].load(std::memory_order_relaxed));
    adc_unipolar_[0].store(static_cast<int16_t>(prev_uni + 100), std::memory_order_relaxed);

    static int16_t bip0 = 0;
    static int16_t bip_dir = 256;
    bip0 = static_cast<int16_t>(bip0 + bip_dir);
    if (bip0 >  20000) bip_dir = -256;
    if (bip0 < -20000) bip_dir =  256;
    adc_bipolar_[0].store(bip0, std::memory_order_relaxed);

    static uint16_t di_walk = 1;
    di_walk = static_cast<uint16_t>((di_walk << 1) | (di_walk >> 15));
    if (di_walk == 0) di_walk = 1;
    dio_inputs_.store(di_walk, std::memory_order_relaxed);
}

void FakeSlave::run_loop() {
    auto* t = kernel::g_platform->get_timer_ops();
    uint64_t next_us = t->get_system_time_us();

    for (;;) {
        // Reuse ecat_b jitter tracker — same 200 µs nominal period; our 100 µs
        // cadence will show slack rather than overruns and that's the intent.
        diag::rt::ecat_b.sample(t->get_system_time_ns());

        next_us += PERIOD_US;

        // Drain whatever the NIC has queued. process_frame handles each.
        if (net_) {
            net_->poll_rx(&FakeSlave::rx_trampoline, this, 8);
        }

        // Guarantee the CiA-402 FSA advances at least once per period. This
        // covers the auto NotReady→SwitchOnDisabled on boot and any CLI
        // injection paths that don't feed through an LRW datagram.
        if (pending_cw_.exchange(0, std::memory_order_acq_rel)) {
            tick_cia402();
        }

        run_stimulus();

        const uint64_t now = t->get_system_time_us();
        if (now > next_us) {
            next_us = now;
        } else {
            wait_until_us(next_us);
        }
    }
}

void FakeSlave::thread_entry(void* arg) {
    auto* self = static_cast<FakeSlave*>(arg);
    if (!self || !kernel::g_platform) for (;;) asm volatile("wfi");
    self->net_ = kernel::g_platform->get_net_ops(self->nic_idx_);
    if (!self->net_) {
        early_uart_puts("[FakeSlave] no NIC available — halting thread\n");
        for (;;) asm volatile("wfi");
    }
    self->run_loop();
}

} // namespace ethercat
