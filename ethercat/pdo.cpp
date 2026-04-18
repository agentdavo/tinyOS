// SPDX-License-Identifier: MIT OR Apache-2.0
// PDO mapping — CoE SDO expedited downloads built on top of the frame builder.

#include "pdo.hpp"
#include "frame.hpp"
#include "master.hpp"
#include "devices/device_db.hpp"

#include <cstring>

namespace ethercat {

namespace {

// SM0 / mailbox-out: host writes SDO requests here. Per ClearPath ESI
// (SyncManager layout in devices/clearpath_ec.tsv, SM0 MBoxOut 0x1000). An
// earlier revision of this file set this to 0x1800 — that's SM2 / PDO out,
// which left SDO requests landing in the process-data window and never
// reaching the slave's CoE handler. Fixed as part of task 1.2.
constexpr uint16_t SM_MAILBOX_OUT_ADDR = 0x1000;
constexpr uint8_t  COE_TYPE            = 0x03;   // mailbox type 3 = CoE
constexpr uint8_t  COE_SERVICE_SDO_REQ = 0x02;   // service 2 = SDO Request
constexpr uint8_t  COE_SERVICE_SDO_RSP = 0x03;   // service 3 = SDO Response

// Build the mailbox + CoE + SDO-expedited-download body.
// Layout: [mb hdr 6][coe hdr 2][sdo 8] = 16 bytes.
size_t build_mailbox_sdo_body(uint8_t* out, size_t cap,
                              uint16_t od_index, uint8_t od_subindex,
                              const uint8_t* data, uint8_t data_len_bytes,
                              uint8_t counter) noexcept {
    constexpr size_t BODY = 6 + 2 + 8;
    if (cap < BODY) return 0;
    if (data_len_bytes != 1 && data_len_bytes != 2 && data_len_bytes != 4) return 0;

    std::memset(out, 0, BODY);

    // Mailbox header.
    uint16_t body_len = 2 + 8; // CoE hdr + SDO
    put_u16_le(&out[0], body_len);
    put_u16_le(&out[2], 0x0000); // station addr = 0 (no routing)
    out[4] = 0x00;               // channel 0, priority 0
    out[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);

    // CoE header: number:9 | rsv:3 | service:4.
    uint16_t coe = static_cast<uint16_t>((COE_SERVICE_SDO_REQ & 0xF) << 12);
    put_u16_le(&out[6], coe);

    // SDO — expedited download with size indication.
    // cmd byte: ccs=1 (0b001xxxxx), n = 4-data_len_bytes, e=1, s=1.
    //   0x2F = 1 byte, 0x2B = 2 bytes, 0x27 = 3 bytes, 0x23 = 4 bytes.
    uint8_t n = static_cast<uint8_t>(4 - data_len_bytes);
    uint8_t cmd = static_cast<uint8_t>(0x23 | (n << 2));
    out[8]  = cmd;
    put_u16_le(&out[9], od_index);
    out[11] = od_subindex;
    std::memcpy(&out[12], data, data_len_bytes);

    return BODY;
}

} // namespace

size_t build_sdo_download(uint8_t* buf, size_t cap,
                          const uint8_t dst_mac[6], const uint8_t src_mac[6],
                          uint16_t station_addr,
                          uint16_t od_index, uint8_t od_subindex,
                          const uint8_t* data, uint8_t data_len_bytes,
                          uint8_t& counter) noexcept {
    FrameBuilder fb(buf, cap, dst_mac, src_mac);

    uint8_t body[16];
    size_t body_len = build_mailbox_sdo_body(body, sizeof(body),
                                             od_index, od_subindex,
                                             data, data_len_bytes, counter);
    if (!body_len) return 0;
    counter = static_cast<uint8_t>((counter + 1) & 0x7); // 3-bit rolling

    // Address = (station_addr << 16) | SM_MAILBOX_OUT_ADDR.
    uint32_t addr = (static_cast<uint32_t>(station_addr) << 16) | SM_MAILBOX_OUT_ADDR;
    if (!fb.add_datagram(Cmd::FPWR, 0, addr, body, static_cast<uint16_t>(body_len))) {
        return 0;
    }
    return fb.finalize();
}

size_t configure_cia402_axis(Master& master, uint16_t station_addr) noexcept {
    // Sequence of SDO writes to install RxPDO 0x1600 / TxPDO 0x1A00 with the
    // CiA-402 minimum set. We don't block on ACKs — a higher layer will retry.
    //
    //   1. 0x1C12:00 = 0         (clear RxPDO assign)
    //   2. 0x1600:00 = 0         (clear RxPDO map entry count)
    //   3. 0x1600:01 = 0x60400010 (control_word 16b)
    //   4. 0x1600:02 = 0x607A0020 (target_position 32b)
    //   5. 0x1600:00 = 2         (entry count)
    //   6. 0x1C12:01 = 0x1600    (assign)
    //   7. 0x1C12:00 = 1         (count)
    //   ... same for 0x1A00 / 0x1C13 (TxPDO).
    struct Step {
        uint16_t idx;
        uint8_t  sub;
        uint32_t val;
        uint8_t  bytes;
    };
    const uint32_t m_ctrl = pdo_map_entry(OD_CONTROL_WORD, 0, 16);
    const uint32_t m_tpos = pdo_map_entry(OD_TARGET_POSITION, 0, 32);
    const uint32_t m_stat = pdo_map_entry(OD_STATUS_WORD, 0, 16);
    const uint32_t m_apos = pdo_map_entry(OD_ACTUAL_POSITION, 0, 32);

    const Step steps[] = {
        { OD_RXPDO_ASSIGN, 0x00, 0,          1 },
        { OD_RXPDO_MAP_1,  0x00, 0,          1 },
        { OD_RXPDO_MAP_1,  0x01, m_ctrl,     4 },
        { OD_RXPDO_MAP_1,  0x02, m_tpos,     4 },
        { OD_RXPDO_MAP_1,  0x00, 2,          1 },
        { OD_RXPDO_ASSIGN, 0x01, OD_RXPDO_MAP_1, 2 },
        { OD_RXPDO_ASSIGN, 0x00, 1,          1 },

        { OD_TXPDO_ASSIGN, 0x00, 0,          1 },
        { OD_TXPDO_MAP_1,  0x00, 0,          1 },
        { OD_TXPDO_MAP_1,  0x01, m_stat,     4 },
        { OD_TXPDO_MAP_1,  0x02, m_apos,     4 },
        { OD_TXPDO_MAP_1,  0x00, 2,          1 },
        { OD_TXPDO_ASSIGN, 0x01, OD_TXPDO_MAP_1, 2 },
        { OD_TXPDO_ASSIGN, 0x00, 1,          1 },
    };

    size_t sent = 0;
    for (const auto& s : steps) {
        uint8_t data[4] = {};
        put_u32_le(data, s.val);
        if (master.send_sdo_download(station_addr, s.idx, s.sub, data, s.bytes)) {
            ++sent;
        }
    }
    return sent;
}

// Build the mailbox + CoE + SDO "initiate upload request" body.
// Layout: [mb hdr 6][coe hdr 2][sdo 8] = 16 bytes. The SDO body for upload
// initiate is: cmd=0x40, index(2), sub(1), reserved(4). The slave ignores
// the 4 reserved bytes and replies with an upload *response* in SM1.
static size_t build_mailbox_sdo_upload_body(uint8_t* out, size_t cap,
                                            uint16_t od_index, uint8_t od_subindex,
                                            uint8_t counter) noexcept {
    constexpr size_t BODY = 6 + 2 + 8;
    if (cap < BODY) return 0;
    std::memset(out, 0, BODY);

    // Mailbox header (same shape as the download path).
    uint16_t body_len = 2 + 8;
    put_u16_le(&out[0], body_len);
    put_u16_le(&out[2], 0x0000);
    out[4] = 0x00;
    out[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);

    // CoE header: service=2 (SDO Request).
    uint16_t coe = static_cast<uint16_t>((COE_SERVICE_SDO_REQ & 0xF) << 12);
    put_u16_le(&out[6], coe);

    // SDO command 0x40 = ccs=2 (initiate upload), no size flags.
    out[8]  = 0x40;
    put_u16_le(&out[9], od_index);
    out[11] = od_subindex;
    // out[12..15] are reserved / zero.

    return BODY;
}

size_t build_sdo_upload_request(uint8_t* buf, size_t cap,
                                const uint8_t dst_mac[6], const uint8_t src_mac[6],
                                uint16_t station_addr,
                                uint16_t od_index, uint8_t od_subindex,
                                uint8_t& counter) noexcept {
    FrameBuilder fb(buf, cap, dst_mac, src_mac);
    uint8_t body[16];
    size_t body_len = build_mailbox_sdo_upload_body(body, sizeof(body),
                                                    od_index, od_subindex,
                                                    counter);
    if (!body_len) return 0;
    counter = static_cast<uint8_t>((counter + 1) & 0x7);

    uint32_t addr = (static_cast<uint32_t>(station_addr) << 16) | SM_MAILBOX_OUT_ADDR;
    if (!fb.add_datagram(Cmd::FPWR, 0, addr, body, static_cast<uint16_t>(body_len))) {
        return 0;
    }
    return fb.finalize();
}

// Build an SDO upload segment request. Layout identical to
// build_sdo_upload_request but with cmd byte 0x60 (toggle=0) or 0x70
// (toggle=1) and no index/sub in the SDO body — the slave knows which
// transfer this belongs to because mailbox sessions are serialised.
size_t build_sdo_upload_segment_request(uint8_t* buf, size_t cap,
                                        const uint8_t dst_mac[6],
                                        const uint8_t src_mac[6],
                                        uint16_t station_addr,
                                        bool toggle,
                                        uint8_t& counter) noexcept {
    FrameBuilder fb(buf, cap, dst_mac, src_mac);
    uint8_t body[16];
    std::memset(body, 0, sizeof(body));
    put_u16_le(&body[0], 10);          // mailbox body length
    put_u16_le(&body[2], 0);
    body[4] = 0;
    body[5] = static_cast<uint8_t>((counter << 4) | COE_TYPE);
    put_u16_le(&body[6],
        static_cast<uint16_t>((COE_SERVICE_SDO_REQ & 0xF) << 12));
    body[8] = static_cast<uint8_t>(0x60 | (toggle ? 0x10 : 0x00));
    // body[9..15] reserved

    counter = static_cast<uint8_t>((counter + 1) & 0x7);
    const uint32_t addr = (static_cast<uint32_t>(station_addr) << 16)
                        | SM_MAILBOX_OUT_ADDR;
    if (!fb.add_datagram(Cmd::FPWR, 0, addr, body, 16)) return 0;
    return fb.finalize();
}

bool parse_sdo_upload_segment_response(const uint8_t* buf, size_t len,
                                       uint8_t  out_data[7],
                                       uint8_t* out_len,
                                       bool*    complete,
                                       uint32_t* abort_code) noexcept {
    if (len < 16 || !buf || !out_data || !out_len || !complete) return false;
    const uint16_t body_len = get_u16_le(&buf[0]);
    if (body_len < 10) return false;
    const uint16_t coe = get_u16_le(&buf[6]);
    const uint8_t svc = static_cast<uint8_t>((coe >> 12) & 0xF);
    if (svc != COE_SERVICE_SDO_RSP) return false;
    const uint8_t cmd = buf[8];
    if ((cmd & 0xE0) == 0x80) {
        if (abort_code) *abort_code = get_u32_le(&buf[12]);
        return false;
    }
    // Segment response: scs=0, bit 0 = toggle, bit 4 = c, bits 1..3 = n.
    if ((cmd & 0xE0) != 0x00) return false;
    const uint8_t n = static_cast<uint8_t>((cmd >> 1) & 0x07);
    const uint8_t seg_bytes = static_cast<uint8_t>(7 - n);
    std::memcpy(out_data, &buf[9], seg_bytes);
    *out_len   = seg_bytes;
    *complete  = (cmd & 0x10) != 0;
    if (abort_code) *abort_code = 0;
    return true;
}

bool classify_sdo_upload_response(const uint8_t* buf, size_t len,
                                  uint16_t expected_index,
                                  uint8_t  expected_sub,
                                  uint8_t* kind,
                                  uint8_t  out[4],
                                  uint8_t* out_bytes,
                                  uint32_t* total_bytes,
                                  uint32_t* abort_code) noexcept {
    if (!kind || len < 16 || !buf) return false;
    *kind = 3; // Invalid by default
    const uint16_t body_len = get_u16_le(&buf[0]);
    if (body_len < 10) return false;
    const uint16_t coe = get_u16_le(&buf[6]);
    const uint8_t svc = static_cast<uint8_t>((coe >> 12) & 0xF);
    if (svc != COE_SERVICE_SDO_RSP) return false;

    const uint8_t cmd    = buf[8];
    const uint16_t idx   = get_u16_le(&buf[9]);
    const uint8_t  sub   = buf[11];

    if ((cmd & 0xE0) == 0x80) {
        if (abort_code) *abort_code = get_u32_le(&buf[12]);
        *kind = 2;
        return false;
    }

    if (idx != expected_index || sub != expected_sub) return false;

    // Expedited: (cmd & 0x03) == 0x03, i.e. e=1, s=1.
    if ((cmd & 0x03) == 0x03) {
        const uint8_t n = static_cast<uint8_t>((cmd >> 2) & 0x03);
        const uint8_t bytes = static_cast<uint8_t>(4 - n);
        if (out && out_bytes) {
            std::memcpy(out, &buf[12], bytes);
            *out_bytes = bytes;
        }
        *kind = 0;
        return true;
    }

    // Segmented initiate: e=0, s=1 → total length in the 4-byte payload.
    if ((cmd & 0x03) == 0x01) {
        if (total_bytes) *total_bytes = get_u32_le(&buf[12]);
        *kind = 1;
        return true;
    }

    // e=0, s=0 → size not indicated; we don't handle.
    return false;
}

bool parse_sdo_upload_response(const uint8_t* buf, size_t len,
                               uint16_t expected_index, uint8_t expected_sub,
                               uint8_t out[4], uint8_t* out_bytes,
                               uint32_t* abort_code) noexcept {
    // Mailbox payload layout mirrors the request: [mb hdr 6][coe hdr 2][sdo 8].
    // If the mailbox is empty (slave hasn't written yet) every byte is zero —
    // reject that so callers know to keep polling.
    if (len < 16 || !buf || !out || !out_bytes) return false;

    // Mailbox body length at [0..1] must equal at least 2+8; otherwise the
    // frame is either stale or truncated.
    const uint16_t body_len = get_u16_le(&buf[0]);
    if (body_len < 10) return false;

    // CoE header service (bits 12..15).
    const uint16_t coe = get_u16_le(&buf[6]);
    const uint8_t  svc = static_cast<uint8_t>((coe >> 12) & 0xF);
    if (svc != COE_SERVICE_SDO_RSP) return false;

    const uint8_t cmd    = buf[8];
    const uint16_t idx   = get_u16_le(&buf[9]);
    const uint8_t  sub   = buf[11];

    // Abort? cmd = 0x80 (ccs=4). Payload is the 32-bit abort code.
    if ((cmd & 0xE0) == 0x80) {
        if (abort_code) *abort_code = get_u32_le(&buf[12]);
        *out_bytes = 0;
        return false;
    }

    if (idx != expected_index || sub != expected_sub) return false;

    // Expedited upload: ccs=2, e=1, s=1, cmd = 0x43 | (n<<2) where n = 4 - len.
    //   0x4F = 1 byte, 0x4B = 2 bytes, 0x47 = 3 bytes, 0x43 = 4 bytes.
    if ((cmd & 0x03) == 0x03) {
        const uint8_t n = static_cast<uint8_t>((cmd >> 2) & 0x03);
        const uint8_t bytes = static_cast<uint8_t>(4 - n);
        std::memcpy(out, &buf[12], bytes);
        *out_bytes = bytes;
        if (abort_code) *abort_code = 0;
        return true;
    }

    // Non-expedited (segmented) — not supported yet, task 1.2 follow-up.
    return false;
}

// Task 1.8 — PDO layout computation. The TSV `pdo` lines list entries in the
// order the slave packs them; each entry carries a bit width, and the byte
// offset for entry `i` is `sum(bits[0..i-1]) / 8`. Only handles byte-aligned
// entries (all CiA-402 indices we care about are multiples of 8 bits). For
// unknown entry indices we still advance the offset so later entries stay
// correctly aligned.
namespace {

// Map a CiA-402 OD index (+ subindex for 0x60FE) to the PdoLayout slot, if
// we recognise it. Returns true and fills `*out_off_slot_ptr` with a pointer
// into `layout` when the entry is one we track.
bool match_rx_field(PdoLayout& layout, uint16_t idx, uint8_t sub,
                    uint8_t offset, uint8_t** out_slot) noexcept {
    uint8_t* slot = nullptr;
    if (idx == 0x6040 && sub == 0)                  slot = &layout.rx_controlword_off;
    else if (idx == 0x6060 && sub == 0)             slot = &layout.rx_mode_op_off;
    else if (idx == 0x607A && sub == 0)             slot = &layout.rx_target_position_off;
    else if (idx == 0x60FF && sub == 0)             slot = &layout.rx_target_velocity_off;
    else if (idx == 0x6071 && sub == 0)             slot = &layout.rx_target_torque_off;
    else if (idx == 0x60FE && sub == 1)             slot = &layout.rx_digital_outputs_off;
    if (!slot) return false;
    *slot = offset;
    *out_slot = slot;
    return true;
}

bool match_tx_field(PdoLayout& layout, uint16_t idx, uint8_t sub,
                    uint8_t offset, uint8_t** out_slot) noexcept {
    uint8_t* slot = nullptr;
    if (idx == 0x6041 && sub == 0)                  slot = &layout.tx_statusword_off;
    else if (idx == 0x6061 && sub == 0)             slot = &layout.tx_mode_op_display_off;
    else if (idx == 0x6064 && sub == 0)             slot = &layout.tx_position_actual_off;
    else if (idx == 0x606C && sub == 0)             slot = &layout.tx_velocity_actual_off;
    else if (idx == 0x6077 && sub == 0)             slot = &layout.tx_torque_actual_off;
    else if (idx == 0x603F && sub == 0)             slot = &layout.tx_error_code_off;
    else if (idx == 0x60FD && sub == 0)             slot = &layout.tx_digital_inputs_off;
    if (!slot) return false;
    *slot = offset;
    *out_slot = slot;
    return true;
}

} // namespace

PdoLayout default_csp_layout() noexcept {
    // Lives alongside fake_slave's LRW packing. Rx:
    //   [cw u16 @0][mode i8 @2][tpos i32 @3][DO u32 @7]
    // Tx:
    //   [sw u16 @0][mode_disp i8 @2][apos i32 @3][DI u32 @7][err u16 @11]
    // DI/DO are u32 per CiA-402 (0x60FD / 0x60FE:1). fake_slave's LRW path
    // writes them at the same offsets so both sides agree without needing
    // a TSV device entry for the fake vendor id.
    PdoLayout l;
    l.rx_controlword_off      = 0;
    l.rx_mode_op_off          = 2;
    l.rx_target_position_off  = 3;
    l.rx_digital_outputs_off  = 7;
    l.rx_size_bytes           = 11; // 2 + 1 + 4 + 4

    l.tx_statusword_off       = 0;
    l.tx_mode_op_display_off  = 2;
    l.tx_position_actual_off  = 3;
    l.tx_digital_inputs_off   = 7;
    l.tx_error_code_off       = 11;
    l.tx_size_bytes           = 13; // 2 + 1 + 4 + 4 + 2
    return l;
}

bool compute_pdo_layout(devices::DeviceId id, cia402::Mode mode,
                        PdoLayout& out) noexcept {
    out = PdoLayout{};

    const auto* dev = devices::g_device_db.find(id);
    if (!dev) return false;

    const PdoSet pair = select_pdo_for_mode(mode);

    auto walk = [&](uint16_t pdo_idx, uint8_t dir, bool is_tx,
                    uint8_t& size_out) noexcept {
        uint16_t bit_offset = 0;
        for (size_t i = 0; i < dev->num_pdo; ++i) {
            const auto& e = dev->pdo[i];
            if (e.pdo_idx != pdo_idx || e.dir != dir) continue;
            const uint8_t byte_off = static_cast<uint8_t>(bit_offset / 8);
            if ((e.bits & 0x07) == 0) {
                uint8_t* slot = nullptr;
                if (is_tx) (void)match_tx_field(out, e.entry_idx, e.sub, byte_off, &slot);
                else       (void)match_rx_field(out, e.entry_idx, e.sub, byte_off, &slot);
                (void)slot; // unknown entries still consume the slot space
            }
            bit_offset = static_cast<uint16_t>(bit_offset + e.bits);
        }
        const uint16_t bytes = static_cast<uint16_t>((bit_offset + 7u) / 8u);
        if (bytes > PDO_SLOT_BYTES) return false;
        size_out = static_cast<uint8_t>(bytes);
        return true;
    };

    bool ok = true;
    ok &= walk(pair.rx_index, /*dir=out=*/0, /*is_tx=*/false, out.rx_size_bytes);
    ok &= walk(pair.tx_index, /*dir=in=*/ 1, /*is_tx=*/true,  out.tx_size_bytes);
    if (!ok) { out = PdoLayout{}; return false; }
    if (out.rx_size_bytes == 0 && out.tx_size_bytes == 0) return false;
    return true;
}

size_t configure_cia402_axis_for_mode(Master& master, uint16_t station_addr,
                                      devices::DeviceId id,
                                      cia402::Mode mode) noexcept {
    const auto* dev = devices::g_device_db.find(id);
    if (!dev) return 0;

    const PdoSet pair = select_pdo_for_mode(mode);

    // Walk the TSV pdo table twice — once for Rx, once for Tx. For each
    // direction we:
    //   1. Clear the assignment count (0x1C12:00 = 0 / 0x1C13:00 = 0).
    //   2. Clear the mapping-object entry count (0x160N:00 = 0).
    //   3. Write each entry (0x160N:01.. = pdo_map_entry(...)).
    //   4. Write the entry count back.
    //   5. Assign the mapping object into the SyncManager slot.
    //   6. Set the assignment count to 1.
    //
    // We don't block on ACKs — send_sdo_download returns as soon as the
    // frame is queued. If a step's TX succeeds we bump `sent`; if it fails
    // (NIC backpressure etc.) we give up on that leg and let the caller
    // observe the short count.
    auto program = [&](uint8_t sm, uint16_t pdo_idx, uint16_t assign_idx,
                       uint8_t dir_wanted, size_t& sent_out) noexcept {
        uint8_t data[4] = {};

        // Step 1: clear assignment count.
        data[0] = 0;
        if (!master.send_sdo_download(station_addr, assign_idx, 0x00, data, 1)) return;
        ++sent_out;

        // Step 2: clear mapping entry count.
        if (!master.send_sdo_download(station_addr, pdo_idx, 0x00, data, 1)) return;
        ++sent_out;

        // Step 3: write each matching TSV entry as a pdo_map_entry.
        uint8_t entry_sub = 1;
        for (size_t i = 0; i < dev->num_pdo; ++i) {
            const auto& e = dev->pdo[i];
            if (e.sm != sm || e.dir != dir_wanted || e.pdo_idx != pdo_idx) continue;
            const uint32_t v = pdo_map_entry(e.entry_idx, e.sub, e.bits);
            put_u32_le(data, v);
            if (!master.send_sdo_download(station_addr, pdo_idx, entry_sub, data, 4)) return;
            ++sent_out;
            if (entry_sub == 0xFF) break; // defensive: 0x160N is u8-subindexed
            ++entry_sub;
        }
        const uint8_t entry_count = static_cast<uint8_t>(entry_sub - 1);
        if (entry_count == 0) return; // nothing mapped — nothing to assign

        // Step 4: write entry count back to subindex 0.
        data[0] = entry_count;
        if (!master.send_sdo_download(station_addr, pdo_idx, 0x00, data, 1)) return;
        ++sent_out;

        // Step 5: assign the mapping object into subindex 1.
        put_u32_le(data, pdo_idx);
        if (!master.send_sdo_download(station_addr, assign_idx, 0x01, data, 2)) return;
        ++sent_out;

        // Step 6: set assignment count to 1.
        data[0] = 1;
        if (!master.send_sdo_download(station_addr, assign_idx, 0x00, data, 1)) return;
        ++sent_out;
    };

    size_t sent = 0;
    program(2, pair.rx_index, OD_RXPDO_ASSIGN, /*dir=out*/0, sent);
    program(3, pair.tx_index, OD_TXPDO_ASSIGN, /*dir=in*/ 1, sent);
    return sent;
}

size_t configure_from_db(Master& master, uint16_t station_addr,
                         devices::DeviceId id) noexcept {
    const auto* dev = devices::g_device_db.find(id);
    if (!dev) return 0;

    size_t sent = 0;
    for (size_t i = 0; i < dev->num_sdo_init; ++i) {
        const auto& s = dev->sdo_init[i];
        uint8_t bytes = 0;
        if      (s.bits <= 8)  bytes = 1;
        else if (s.bits <= 16) bytes = 2;
        else if (s.bits <= 32) bytes = 4;
        else continue; // wider SDO writes need segmented transfer — skip.

        uint8_t data[4] = {};
        // Little-endian pack, masked to `bytes`.
        uint64_t v = s.value;
        for (uint8_t b = 0; b < bytes; ++b) {
            data[b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
        }
        if (master.send_sdo_download(station_addr, s.index, s.sub, data, bytes)) {
            ++sent;
        }
    }
    return sent;
}

} // namespace ethercat
