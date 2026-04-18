// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "core.hpp"

#include <cstddef>
#include <cstdint>

namespace machine {

enum class SymbolType : uint8_t {
    Bool = 0,
    Int,
    Float,
};

struct SymbolValue {
    bool bool_value = false;
    int32_t int_value = 0;
    float float_value = 0.0f;
};

class Registry {
public:
    static constexpr size_t MAX_SYMBOLS = 128;
    static constexpr size_t MAX_NAME_LEN = 32;
    static constexpr size_t MAX_SIGNAL_BINDINGS = 64;

    enum class SignalSource : uint8_t {
        None = 0,
        EcDi,
        EcDo,
        EcAiUni,
        EcAiBip,
        EcTxBit,
        EcRxBit,
        EcTxS16,
        EcTxU16,
        EcTxS32,
        EcTxU32,
        EcRxS16,
        EcRxU16,
        EcRxS32,
        EcRxU32,
        EcTxPin,
        EcRxPin,
        EcStatusWord,
        EcControlWord,
        EcPositionActual,
        EcVelocityActual,
        EcErrorCode,
        EcDriveMode,
        EcDigitalInputs,
        EcDigitalOutputs,
    };

    struct Entry {
        char name[MAX_NAME_LEN]{};
        SymbolType type = SymbolType::Bool;
        bool writable = false;
        bool builtin = false;
        bool used = false;
        SymbolValue value{};
    };

    struct SignalBinding {
        bool used = false;
        char name[MAX_NAME_LEN]{};
        SignalSource source = SignalSource::None;
        SymbolType type = SymbolType::Bool;
        uint8_t master = 0xFF;
        uint8_t slave = 0xFF;
        uint8_t bit = 0;
        uint8_t channel = 0;
        uint16_t offset = 0;
        uint8_t width_bits = 0;
        bool writable = false;
        char pin[MAX_NAME_LEN]{};
    };

    Registry() noexcept;

    bool define_symbol(const char* name, SymbolType type, SymbolValue initial,
                       bool writable, bool builtin = false) noexcept;
    size_t count() const noexcept;
    const Entry* entry(size_t idx) const noexcept;
    bool find(const char* name, size_t& idx_out) const noexcept;

    bool get_bool(const char* name, bool& out) const noexcept;
    bool get_int(const char* name, int32_t& out) const noexcept;
    bool get_float(const char* name, float& out) const noexcept;
    bool set_bool(const char* name, bool value) noexcept;
    bool set_int(const char* name, int32_t value) noexcept;
    bool set_float(const char* name, float value) noexcept;
    bool load_signal_tsv(const char* buf, size_t len) noexcept;
    size_t signal_binding_count() const noexcept;
    const SignalBinding* signal_binding(size_t idx) const noexcept;

    void sync_from_runtime() noexcept;

private:
    bool set_locked(size_t idx, SymbolValue value) noexcept;
    bool find_locked(const char* name, size_t& idx_out) const noexcept;
    void bind_builtins() noexcept;
    void update_io_locked() noexcept;
    void update_motion_locked() noexcept;
    void update_interpreter_locked() noexcept;
    void update_toolpods_locked() noexcept;
    void update_signal_inputs_locked() noexcept;
    void update_signal_outputs_locked() noexcept;
    void update_derived_locked() noexcept;

    Entry entries_[MAX_SYMBOLS]{};
    SignalBinding signal_bindings_[MAX_SIGNAL_BINDINGS]{};
    size_t count_ = 0;
    mutable kernel::core::Spinlock lock_;
};

extern Registry g_registry;

} // namespace machine
