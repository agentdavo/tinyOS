// SPDX-License-Identifier: MIT OR Apache-2.0
// Runtime device database. Populated from TSV records (see config/tsv.{hpp,cpp})
// at boot. The database holds static, fixed-size arrays — no heap.
//
// The EtherCAT master looks up a slave by its {VID, PID} after reading
// 0x1018:1 / 0x1018:2, and retrieves its SDO pre-op init sequence, PDO mapping
// entries, and OD defaults.

#ifndef DEVICES_DEVICE_DB_HPP
#define DEVICES_DEVICE_DB_HPP

#include <cstddef>
#include <cstdint>

namespace kernel { namespace hal { struct UARTDriverOps; } }

namespace devices {

struct DeviceId {
    uint32_t vid;
    uint32_t pid;
};

struct SdoInit {
    uint16_t index;
    uint8_t  sub;
    uint8_t  bits;    // 8, 16, 32
    uint64_t value;   // raw integer, sign-extended to bits if negative
};

struct PdoEntry {
    uint16_t pdo_idx;    // e.g. 0x1600 / 0x1A00
    uint16_t entry_idx;  // e.g. 0x607A
    uint8_t  sub;
    uint8_t  bits;
    uint8_t  sm;         // 0..3
    uint8_t  dir;        // 0 = out (RxPDO), 1 = in (TxPDO)
    char     pin[32]{};  // symbolic field name from TSV (optional)
};

struct OdParam {
    uint16_t index;
    uint8_t  sub;
    uint8_t  bits;
    uint8_t  access;     // 0 = ro, 1 = rw
    uint64_t default_value;
    bool     has_default;
};

// Known device classes. `Servo` (CiA-402 drive) is the default; `Coupler`
// marks passive devices (EK1100/EK1110/EK1122 etc.) that have no mailbox
// and no user PDOs — bus_config skips SDO/PDO/DC configuration for them.
// `EncoderInput` (EL5042 BiSS-C) has mailbox SDOs for bit-width config but
// no CiA-402 FSA.
//
// Extension point: add DigitalIO / AnalogIO / Thermocouple / Stepper / ...
// as additional arms here + matching arms in bus_config.cpp's type switch.
// The `Unknown` sentinel is used when the TSV carries a type= key we don't
// recognise — bus_config rejects those so a typo in TSV is caught at boot
// rather than silently falling back to Servo behaviour.
enum class DeviceType : uint8_t {
    Servo          = 0,
    Coupler        = 1,
    EncoderInput   = 2,
    // Beckhoff-style I/O terminals. DigitalInput / DigitalOutput have no
    // CoE mailbox on the simplest variants (EL1809/EL2809), so bus_config
    // skips the identity probe + SDO init. AnalogInput devices typically
    // have a mailbox and take per-channel filter/range SDO-init at PreOp.
    DigitalInput   = 3,
    DigitalOutput  = 4,
    AnalogInput    = 5,
    AnalogOutput   = 6,
    Unknown        = 255,
};

struct DeviceEntry {
    DeviceId id;
    char     name[32];
    DeviceType type = DeviceType::Servo;   // set from TSV `type=` key
    uint16_t dc_assign_activate;  // 0 if TODO / unset
    uint32_t cycle_min_us;
    uint32_t cycle_quant_us;

    // Task 8.1 — E-bus current draw in mA (from ESI `Current` field for
    // Beckhoff terminals). bus_config sums this across every slice
    // downstream of an EK1100 and refuses to bring the bus to SafeOp if
    // the total exceeds 2000 mA. 0 means "unknown" and contributes
    // nothing — couplers default to 0, terminals carry a real number.
    uint16_t ebus_current_ma = 0;

    static constexpr size_t MAX_SDO_INIT = 32;
    static constexpr size_t MAX_PDO      = 96;
    static constexpr size_t MAX_OD       = 160;

    SdoInit  sdo_init[MAX_SDO_INIT];
    size_t   num_sdo_init;

    PdoEntry pdo[MAX_PDO];
    size_t   num_pdo;

    OdParam  od[MAX_OD];
    size_t   num_od;
};

constexpr size_t MAX_DEVICES = 16;

class DeviceDB {
public:
    // Parses `buf` of `len` bytes as TSV. Each `device` record starts a new
    // entry; `sdo_init`, `pdo`, `od` records are appended to the current entry.
    // Returns false only on malformed TSV; record-level overflow sets the
    // truncated() flag but returns true.
    [[nodiscard]] bool load_tsv(const char* buf, size_t len) noexcept;

    const DeviceEntry* find(DeviceId id) const noexcept;

    // Look up the default value of an OD entry for a given device. Returns
    // true iff the device exists *and* the entry carries `has_default`.
    // `out_value` is the raw integer default as parsed from TSV.
    [[nodiscard]] bool get_od_default(DeviceId id, uint16_t index, uint8_t sub,
                                      uint64_t& out_value) const noexcept;
    size_t device_count() const noexcept { return num_devices_; }
    const DeviceEntry* at(size_t i) const noexcept {
        return (i < num_devices_) ? &devices_[i] : nullptr;
    }

    void dump(kernel::hal::UARTDriverOps* uart) const;

    bool truncated() const noexcept { return truncated_; }

    // Internal-use: allocate a fresh (zeroed) device slot for the loader.
    // Returns nullptr if the table is full.
    DeviceEntry* allocate_slot() noexcept;
    void mark_truncated() noexcept { truncated_ = true; }

private:
    DeviceEntry devices_[MAX_DEVICES]{};
    size_t      num_devices_ = 0;
    bool        truncated_   = false;
};

extern DeviceDB g_device_db;

} // namespace devices

#endif
