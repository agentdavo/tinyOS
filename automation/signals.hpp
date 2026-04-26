// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>

namespace automation::signals {

// Named-signal write helper for M-code dispatch and other operator-driven
// digital-output paths. Looks the signal up in machine::g_registry by name
// (case-sensitive) and asserts/clears the underlying writable bool symbol;
// the registry's per-cycle update_signal_outputs_locked() pushes that bit
// onto the bound EtherCAT slave's RxPDO.
//
// Returns false in three distinct cases (caller may ignore — none are faults):
//   - either EtherCAT master is deadline-faulted (gated, do nothing)
//   - signal name not present in the registry / not writable
//   - registry write rejected (e.g. wrong symbol type)
// Per spec: an unbound signal (slave==0xFF / not enrolled) must NOT fault;
// the registry already silently skips the slave write in that case, so this
// helper just toggles the symbol value and returns true.
bool set_named_signal_bool(const char* name, bool value) noexcept;

// Read-back accessor for CLI/diagnostic surfaces. Returns false if the name
// is unknown or not a bool symbol. Does not gate on deadline-fault.
bool get_named_signal_bool(const char* name, bool& out) noexcept;

// Hot-path read for the per-cycle safety poller. Returns the symbol's
// current value if defined; falls back to `default_value` (typically false
// = "inactive") when the name is missing. Distinguishes "signal not bound
// to a slave" (the bit was pumped from EcDi as 0, returns false) from
// "name not in the registry at all" (returns default_value). The safety
// path uses `default = false` so an unconfigured machine never trips a
// phantom limit / E-stop on its own.
bool get_named_signal_bool_or(const char* name, bool default_value) noexcept;

// Build the canonical aux-DOUT signal name ("aux_dout_<idx>") for the
// LinuxCNC M62/M63/M64/M65 P-word index. Writes into out_buf; returns false
// if idx is out of range (0..MAX_AUX_DOUT-1) or buf is too small.
constexpr size_t MAX_AUX_DOUT = 8;
bool aux_dout_signal_name(uint32_t idx, char* out_buf, size_t out_buf_len) noexcept;

} // namespace automation::signals
