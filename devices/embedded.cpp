// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded TSV blobs. The actual bytes are pulled in via `.incbin` in
// embedded_blob.S — we just provide symbols + the loader.

#include "embedded.hpp"
#include "device_db.hpp"

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
    bool ok = g_device_db.load_tsv(clearpath_ec_tsv, clearpath_ec_tsv_len);
    ok &= g_device_db.load_tsv(beckhoff_ek_tsv, beckhoff_ek_tsv_len);
    ok &= g_device_db.load_tsv(el5042_biss_tsv, el5042_biss_tsv_len);
    ok &= g_device_db.load_tsv(el1809_din_tsv, el1809_din_tsv_len);
    ok &= g_device_db.load_tsv(el2809_dout_tsv, el2809_dout_tsv_len);
    ok &= g_device_db.load_tsv(el3162_ain_tsv, el3162_ain_tsv_len);
    ok &= g_device_db.load_tsv(el7201_9014_servo_tsv, el7201_9014_servo_tsv_len);
    ok &= g_device_db.load_tsv(el7211_9014_servo_tsv, el7211_9014_servo_tsv_len);
    ok &= g_device_db.load_tsv(el7221_9014_servo_tsv, el7221_9014_servo_tsv_len);
    return ok;
}

} // namespace devices

#else

namespace devices {

bool load_all_embedded() {
    return true;
}

} // namespace devices

#endif
