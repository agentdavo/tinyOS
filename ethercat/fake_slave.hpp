// SPDX-License-Identifier: MIT OR Apache-2.0
// In-kernel fake EtherCAT slave ("FakeIO") for loopback testing without an
// external simulator. Owns the configured EtherCAT-B NIC, runs on core 3 in place of ec_b when
// MINIOS_FAKE_SLAVE=1. Parses transit frames, rewrites WKC + read data in
// place, and re-transmits the frame back out the same NIC — exactly what a
// real EtherCAT slave does electrically as the frame passes through it.
//
// Capabilities (per spec doc):
//   - Station address 0x1001, vendor 0xDEADBEEF, product 0x00000001.
//   - 16 DI / 16 DO bits.
//   - 16 unipolar (0..10 V) and 16 bipolar (-10..+10 V) analogue channels.
//   - TxPDO 66 B (DI bitmap + 16×u16 uni + 16×u16 bip).
//   - RxPDO  2 B (DO bitmap).
//
// Process-data image (slave-side register addresses):
//   0x0F00..0x0F01   RxPDO (DO bitmap, master writes)
//   0x1000..0x1041   TxPDO (66 B, master reads)
//
// Logical-address LRW handling: the master allocates one PDO_SLOT_BYTES (=16)
// slot per slave starting at logical address 0. We treat that slot as our
// LRW exchange — the master pulls back our DI bitmap (first 2 B) and pushes
// the DO bitmap (also first 2 B). The remaining 14 B are reserved/zero; the
// full 66 B TxPDO is reachable via FPRD on 0x1000.

#ifndef ETHERCAT_FAKE_SLAVE_HPP
#define ETHERCAT_FAKE_SLAVE_HPP

#include "cia402.hpp"
#include "esm.hpp"
#include "hal.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace ethercat {

class FakeSlave {
public:
    static constexpr uint16_t STATION_ADDR     = 0x1001;
    // Identity matches devices/clearpath_ec.tsv so the 1.3 bus_config
    // identity check passes end-to-end in QEMU. vid=0x0C96 = decimal 3222
    // (Teknic), pid=0x01 (ClearPath-EC Product Code per OD 0x1018:2).
    static constexpr uint32_t VENDOR_ID        = 0x00000C96u;
    static constexpr uint32_t PRODUCT_CODE     = 0x00000001u;

    static constexpr uint16_t REG_PDI_RX_BASE  = 0x0F00; // RxPDO image
    static constexpr uint16_t REG_PDI_TX_BASE  = 0x1000; // TxPDO image
    static constexpr uint16_t TXPDO_BYTES      = 66;     // 2 + 16*2 + 16*2
    static constexpr uint16_t RXPDO_BYTES      = 2;      // DO bitmap
    static constexpr uint32_t PERIOD_US        = 100;    // poll cadence

    FakeSlave(int nic_idx) noexcept : nic_idx_(nic_idx) {}

    int nic_idx() const noexcept { return nic_idx_; }
    void set_nic_idx(int nic_idx) noexcept { nic_idx_ = nic_idx; }

    // Scheduler entry — `arg` is `FakeSlave*`. Never returns.
    static void thread_entry(void* arg);

    // CLI helpers (read-only snapshot accessors).
    uint8_t  al_state()       const noexcept { return al_state_.load(std::memory_order_relaxed); }
    uint64_t wkc_serviced()   const noexcept { return wkc_serviced_.load(std::memory_order_relaxed); }
    uint64_t frames_in()      const noexcept { return frames_in_.load(std::memory_order_relaxed); }
    uint64_t frames_out()     const noexcept { return frames_out_.load(std::memory_order_relaxed); }
    uint16_t dio_in()         const noexcept { return dio_inputs_.load(std::memory_order_relaxed); }
    uint16_t dio_out()        const noexcept { return dio_outputs_.load(std::memory_order_relaxed); }
    void set_do(uint16_t bitmap) noexcept { dio_outputs_.store(bitmap, std::memory_order_relaxed); }
    int16_t  uni(size_t ch)   const noexcept {
        return ch < adc_unipolar_.size() ? adc_unipolar_[ch].load(std::memory_order_relaxed) : 0;
    }
    int16_t  bip(size_t ch)   const noexcept {
        return ch < adc_bipolar_.size() ? adc_bipolar_[ch].load(std::memory_order_relaxed) : 0;
    }

    // Test/CLI hooks for forcing input values (units = raw register values).
    void set_di(uint16_t bitmap)             noexcept { dio_inputs_.store(bitmap, std::memory_order_relaxed); }
    void set_uni(size_t ch, int16_t raw)     noexcept {
        if (ch < adc_unipolar_.size()) adc_unipolar_[ch].store(raw, std::memory_order_relaxed);
    }
    void set_bip(size_t ch, int16_t raw)     noexcept {
        if (ch < adc_bipolar_.size()) adc_bipolar_[ch].store(raw, std::memory_order_relaxed);
    }

    // CiA-402 read-only snapshot accessors. Safe to call from any core; the
    // cyclic loop mutates the underlying atomics.
    uint16_t cia_controlword()     const noexcept { return controlword_.load(std::memory_order_relaxed); }
    uint16_t cia_statusword()      const noexcept { return statusword_.load(std::memory_order_relaxed); }
    int8_t   cia_mode_op()         const noexcept { return static_cast<int8_t>(mode_op_.load(std::memory_order_relaxed)); }
    int8_t   cia_mode_op_display() const noexcept { return static_cast<int8_t>(mode_op_display_.load(std::memory_order_relaxed)); }
    int32_t  cia_target_pos()      const noexcept { return static_cast<int32_t>(target_pos_.load(std::memory_order_relaxed)); }
    int32_t  cia_actual_pos()      const noexcept { return static_cast<int32_t>(actual_pos_.load(std::memory_order_relaxed)); }
    cia402::State cia_state()      const noexcept { return static_cast<cia402::State>(cia_state_.load(std::memory_order_relaxed)); }

    // Test hook: drive the controlword directly (simulates master writing the
    // TX PDO). Advances the FSA on the next cycle.
    void cia_inject_controlword(uint16_t cw)  noexcept { controlword_.store(cw, std::memory_order_relaxed); pending_cw_.store(1, std::memory_order_release); }
    void cia_inject_mode_op(int8_t m)         noexcept { mode_op_.store(static_cast<uint8_t>(m), std::memory_order_relaxed); pending_cw_.store(1, std::memory_order_release); }
    void cia_inject_target_pos(int32_t p)     noexcept { target_pos_.store(static_cast<uint32_t>(p), std::memory_order_relaxed); pending_cw_.store(1, std::memory_order_release); }

    // Force the FSA into Fault for `cycles` consecutive ticks (each tick is
    // the fake_slave period, ~100 µs). Clears on the usual CW_FAULT_RESET
    // edge via tick_cia402() once the counter reaches zero. Default of 20
    // ticks ≈ 2 ms, which is several master LRW round-trips — enough for
    // motion's 250 µs cycle to observe the fault state reliably.
    void cia_inject_fault(uint16_t cycles = 20) noexcept {
        force_fault_ticks_.store(cycles, std::memory_order_release);
    }

private:
    int nic_idx_;
    kernel::hal::net::NetworkDriverOps* net_ = nullptr;

    // EtherCAT-side state.
    std::atomic<uint8_t> al_state_{AL_INIT};
    std::atomic<uint8_t> al_target_{AL_INIT};
    std::atomic<uint64_t> frames_in_{0};
    std::atomic<uint64_t> frames_out_{0};
    std::atomic<uint64_t> wkc_serviced_{0};

    // I/O state. Atomics so the CLI (core 0) can mutate concurrently with the
    // slave thread (core 3) reading them into the TxPDO snapshot.
    std::atomic<uint16_t> dio_inputs_{0};   // master will see these as inputs
    std::atomic<uint16_t> dio_outputs_{0};  // master writes these
    std::array<std::atomic<int16_t>, 16> adc_unipolar_{};
    std::array<std::atomic<int16_t>, 16> adc_bipolar_{};

    // Stimulus generator — drives a slow triangle on bipolar[0], a counter on
    // unipolar[0], and a 16-bit walking-1 on the DI bitmap so the master sees
    // moving values without external test fixtures.
    uint64_t stim_tick_ = 0;
    void run_stimulus() noexcept;

    // ---------- CiA-402 CSP drive simulator ----------
    // All CiA-402 PDO fields are atomics so the CLI (core 0) can peek while
    // the cyclic loop (core 3) updates them. The FSA / actual_pos update runs
    // once per process_frame() call on the LRW datagram path.
    std::atomic<uint16_t> controlword_{0};
    std::atomic<uint16_t> statusword_{0};
    std::atomic<uint8_t>  mode_op_{0};          // 0x6060 target mode
    std::atomic<uint8_t>  mode_op_display_{0};  // 0x6061 reported mode
    std::atomic<uint32_t> target_pos_{0};       // stored unsigned, read as i32
    std::atomic<uint32_t> actual_pos_{0};
    std::atomic<uint8_t>  cia_state_{static_cast<uint8_t>(cia402::State::NotReadyToSwitchOn)};
    // One-cycle lag pipeline for CSP. When OP-enabled and mode_op==8 we copy
    // target → pending at end of cycle N, pending → actual at end of cycle N+1.
    std::atomic<uint32_t> csp_pending_{0};
    std::atomic<uint8_t>  pending_cw_{1}; // wake FSA on first cycle
    std::atomic<uint16_t> force_fault_ticks_{0}; // test-hook: hold Fault for N ticks
    std::atomic<uint8_t>  homing_attained_{0};
    void tick_cia402() noexcept;

    // --- CoE SDO mailbox emulation (task 1.2) ------------------------------
    // When the master writes an SDO upload request into SM0 (FPWR to 0x1000,
    // 16-byte body), we parse it, look up a canned value for the requested
    // OD entry, and stage the 16-byte upload response in `sdo_response_buf_`.
    // The next FPRD to SM1 (0x1400) drains the buffer back to the master.
    // Single-slot — one outstanding request at a time, matching the master's
    // constraint. Not atomics because all mailbox state is owned by the
    // process_frame() path which runs single-threaded on core 3.
    static constexpr uint16_t SDO_MAILBOX_LEN = 16;
    uint8_t  sdo_response_buf_[SDO_MAILBOX_LEN]{};
    bool     sdo_response_ready_ = false;

    // Segmented-SDO transfer state. When the master issues an initiate
    // upload request for an object we serve as segmented, we stash the
    // payload in `sdo_seg_payload_` and feed it out 7 bytes at a time
    // in response to subsequent segment requests. Single outstanding
    // transfer — matches the master-side constraint.
    static constexpr size_t SDO_SEG_CAP = 128;
    uint8_t  sdo_seg_payload_[SDO_SEG_CAP]{};
    uint16_t sdo_seg_total_     = 0;
    uint16_t sdo_seg_sent_      = 0;
    bool     sdo_seg_toggle_    = false;
    bool     sdo_seg_active_    = false;

    bool build_sdo_upload_response(uint16_t index, uint8_t sub, uint8_t counter,
                                   uint8_t out[SDO_MAILBOX_LEN]) noexcept;
    // If `{index, sub}` matches a segmented entry, populate
    // `sdo_seg_payload_` with the canned data and return true. The
    // caller then emits an initiate-segmented response in the mailbox.
    bool stage_sdo_segmented_payload(uint16_t index, uint8_t sub) noexcept;
    // Build the next segment response consuming from `sdo_seg_payload_`.
    bool build_sdo_segment_response(uint8_t counter,
                                    uint8_t out[SDO_MAILBOX_LEN]) noexcept;

    // Frame-handling pipeline.
    void run_loop();
    void process_frame(uint8_t* buf, size_t len) noexcept;
    bool snapshot_txpdo(uint8_t out[TXPDO_BYTES]) noexcept;
    void apply_rxpdo(const uint8_t* in, size_t len) noexcept;

    // Translate AL-control writes from master into our state.
    void handle_al_ctrl_write(const uint8_t* d, uint16_t len) noexcept;

    // RX trampoline: sized to fit a max EtherCAT frame; we copy each NIC frame
    // into this scratch so the parser/rewriter has a writable, contiguous
    // working buffer (the NIC ring buffer is read-only for our purposes).
    static constexpr size_t SCRATCH_BYTES = 1536;
    static void rx_trampoline(int if_idx, const uint8_t* data, size_t len, void* ctx) noexcept;

    // Single-frame scratch — used inside process_frame() context only (the
    // poll loop is single-threaded so a member buffer is safe).
    alignas(8) uint8_t scratch_[SCRATCH_BYTES]{};
    size_t scratch_len_ = 0;
};

extern FakeSlave g_fake_slave;

} // namespace ethercat

#endif
