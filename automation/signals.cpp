// SPDX-License-Identifier: MIT OR Apache-2.0

#include "signals.hpp"

#include "ethercat/master.hpp"
#include "machine/machine_registry.hpp"
#include "util.hpp"

namespace automation::signals {

namespace {

bool any_master_deadline_faulted() noexcept {
    // Mirror ui::operator_api::any_master_deadline_faulted so coolant/spindle
    // writes follow the same gating contract as the operator buttons. Already-
    // asserted outputs stay asserted: we only inhibit *new* writes — the
    // registry's per-cycle output push naturally re-asserts the held state.
    return ethercat::g_master_a.is_deadline_faulted() ||
           ethercat::g_master_b.is_deadline_faulted();
}

} // namespace

bool set_named_signal_bool(const char* name, bool value) noexcept {
    if (!name || !*name) return false;
    if (any_master_deadline_faulted()) return false;
    // set_bool checks the writable flag and the symbol type; an unbound
    // signal still has a registry symbol (TSV loader auto-defines one), so
    // the bit toggle succeeds and the next sync_from_runtime() write is just
    // a no-op when binding.master/slave == 0xFF.
    return machine::g_registry.set_bool(name, value);
}

bool get_named_signal_bool(const char* name, bool& out) noexcept {
    if (!name || !*name) return false;
    return machine::g_registry.get_bool(name, out);
}

bool aux_dout_signal_name(uint32_t idx, char* out_buf, size_t out_buf_len) noexcept {
    if (!out_buf || out_buf_len < 12) return false;
    if (idx >= MAX_AUX_DOUT) return false;
    kernel::util::k_snprintf(out_buf, out_buf_len, "aux_dout_%u",
                             static_cast<unsigned>(idx));
    return true;
}

} // namespace automation::signals
