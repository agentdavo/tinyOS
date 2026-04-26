// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded TSV blobs. The actual bytes are pulled in via `.incbin` in
// embedded_blob.S — we just provide symbols + the loader.

#include "embedded.hpp"
#include "device_db.hpp"
#include "hal.hpp"
#include "miniOS.hpp"
#include "util.hpp"

#include <cstdint>

namespace devices {

// Phase B: binary ESI device descriptor blob (vendor catalog). Linked into
// every arch — there's no per-arch divergence below the HAL boundary for
// device data. Symbols come from devices/embedded_esi.S.
extern "C" {
    extern const uint8_t _binary_esi_payload_start[];
    extern const uint8_t _binary_esi_payload_end[];
}

namespace {

// Boot-log helper: keep the blob result on a single UART line so the SMP
// boot stream stays grep-friendly.
void log_esi_result(size_t loaded, size_t skipped, bool ok) noexcept {
    auto* uart = kernel::g_platform ? kernel::g_platform->get_uart_ops() : nullptr;
    if (!uart) return;
    char line[96];
    if (!ok) {
        uart->puts("[devices] ESI blob: invalid (bad magic / version / bounds)\n");
        return;
    }
    kernel::util::k_snprintf(line, sizeof(line),
                             "[devices] ESI blob: %zu loaded, %zu skipped (TSV duplicate)\n",
                             loaded, skipped);
    uart->puts(line);
}

} // namespace

} // namespace devices

#if defined(__aarch64__)

namespace devices {

// Defined in embedded_blob.S via .incbin.

// Defined in embedded_blob.S via .incbin.
extern "C" {
    extern const char _binary_clearpath_ec_tsv_start[];
    extern const char _binary_clearpath_ec_tsv_end[];
    extern const char _binary_beckhoff_ek_tsv_start[];
    extern const char _binary_beckhoff_ek_tsv_end[];
    extern const char _binary_el5042_biss_tsv_start[];
    extern const char _binary_el5042_biss_tsv_end[];
    extern const char _binary_el1809_din_tsv_start[];
    extern const char _binary_el1809_din_tsv_end[];
    extern const char _binary_el2809_dout_tsv_start[];
    extern const char _binary_el2809_dout_tsv_end[];
    extern const char _binary_el3162_ain_tsv_start[];
    extern const char _binary_el3162_ain_tsv_end[];
    extern const char _binary_el7201_9014_servo_tsv_start[];
    extern const char _binary_el7201_9014_servo_tsv_end[];
    extern const char _binary_el7211_9014_servo_tsv_start[];
    extern const char _binary_el7211_9014_servo_tsv_end[];
    extern const char _binary_el7221_9014_servo_tsv_start[];
    extern const char _binary_el7221_9014_servo_tsv_end[];
}

const char* const clearpath_ec_tsv = _binary_clearpath_ec_tsv_start;
const size_t clearpath_ec_tsv_len =
    static_cast<size_t>(_binary_clearpath_ec_tsv_end - _binary_clearpath_ec_tsv_start);

const char* const beckhoff_ek_tsv = _binary_beckhoff_ek_tsv_start;
const size_t beckhoff_ek_tsv_len =
    static_cast<size_t>(_binary_beckhoff_ek_tsv_end - _binary_beckhoff_ek_tsv_start);

const char* const el5042_biss_tsv = _binary_el5042_biss_tsv_start;
const size_t el5042_biss_tsv_len =
    static_cast<size_t>(_binary_el5042_biss_tsv_end - _binary_el5042_biss_tsv_start);

const char* const el1809_din_tsv = _binary_el1809_din_tsv_start;
const size_t el1809_din_tsv_len =
    static_cast<size_t>(_binary_el1809_din_tsv_end - _binary_el1809_din_tsv_start);

const char* const el2809_dout_tsv = _binary_el2809_dout_tsv_start;
const size_t el2809_dout_tsv_len =
    static_cast<size_t>(_binary_el2809_dout_tsv_end - _binary_el2809_dout_tsv_start);

const char* const el3162_ain_tsv = _binary_el3162_ain_tsv_start;
const size_t el3162_ain_tsv_len =
    static_cast<size_t>(_binary_el3162_ain_tsv_end - _binary_el3162_ain_tsv_start);

const char* const el7201_9014_servo_tsv = _binary_el7201_9014_servo_tsv_start;
const size_t el7201_9014_servo_tsv_len =
    static_cast<size_t>(_binary_el7201_9014_servo_tsv_end - _binary_el7201_9014_servo_tsv_start);

const char* const el7211_9014_servo_tsv = _binary_el7211_9014_servo_tsv_start;
const size_t el7211_9014_servo_tsv_len =
    static_cast<size_t>(_binary_el7211_9014_servo_tsv_end - _binary_el7211_9014_servo_tsv_start);

const char* const el7221_9014_servo_tsv = _binary_el7221_9014_servo_tsv_start;
const size_t el7221_9014_servo_tsv_len =
    static_cast<size_t>(_binary_el7221_9014_servo_tsv_end - _binary_el7221_9014_servo_tsv_start);

bool load_all_embedded() noexcept {
    // TSVs are operator-curated and authoritative — they go in first so the
    // ESI blob loader can defer to them on duplicate {VID,PID}.
    bool ok = g_device_db.load_tsv(clearpath_ec_tsv, clearpath_ec_tsv_len);
    ok &= g_device_db.load_tsv(beckhoff_ek_tsv, beckhoff_ek_tsv_len);
    ok &= g_device_db.load_tsv(el5042_biss_tsv, el5042_biss_tsv_len);
    ok &= g_device_db.load_tsv(el1809_din_tsv, el1809_din_tsv_len);
    ok &= g_device_db.load_tsv(el2809_dout_tsv, el2809_dout_tsv_len);
    ok &= g_device_db.load_tsv(el3162_ain_tsv, el3162_ain_tsv_len);
    ok &= g_device_db.load_tsv(el7201_9014_servo_tsv, el7201_9014_servo_tsv_len);
    ok &= g_device_db.load_tsv(el7211_9014_servo_tsv, el7211_9014_servo_tsv_len);
    ok &= g_device_db.load_tsv(el7221_9014_servo_tsv, el7221_9014_servo_tsv_len);

    size_t esi_loaded = 0, esi_skipped = 0;
    const bool esi_ok = g_device_db.load_esi_blob(_binary_esi_payload_start,
                                                  _binary_esi_payload_end,
                                                  &esi_loaded, &esi_skipped);
    log_esi_result(esi_loaded, esi_skipped, esi_ok);
    return ok && esi_ok;
}

} // namespace devices

#else

namespace devices {

bool load_all_embedded() noexcept {
    // No per-arch TSV blobs on rv64 yet — only the ESI vendor catalog.
    size_t esi_loaded = 0, esi_skipped = 0;
    const bool esi_ok = g_device_db.load_esi_blob(_binary_esi_payload_start,
                                                  _binary_esi_payload_end,
                                                  &esi_loaded, &esi_skipped);
    log_esi_result(esi_loaded, esi_skipped, esi_ok);
    return esi_ok;
}

} // namespace devices

#endif
