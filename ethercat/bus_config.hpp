// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Bus-config helper (PLAN 1.1 / 1.3 / 1.4-followup).
//
// One-shot worker that sits between the master reaching PreOp and the
// transition to SafeOp. For each discovered slave it:
//   1. Probes identity (OD 0x1018:1/2/3) and matches against the device DB.
//   2. On match: probes encoder resolution (0x608F), pushes into motion.
//   3. On match: programs the per-mode PDO map (configure_cia402_axis_for_mode).
//   4. When every slave is good, flips `Master::allow_safeop(true)` so the
//      ESM leaves PreOp.
// On any slave mismatch: sets `SlaveInfo::identity_mismatch` and leaves the
// gate closed — the bus stays at PreOp until the operator intervenes. Run
// on core 0 (same as CLI) so the blocking `upload_sdo` calls work as
// intended (yield to the ec_a thread on core 2 for SDO service).

#ifndef ETHERCAT_BUS_CONFIG_HPP
#define ETHERCAT_BUS_CONFIG_HPP

namespace ethercat {

// Scheduler entry point. `arg` may be null (defaults to g_master_a) or a
// `Master*` selecting which EtherCAT segment to configure.
void bus_config_entry(void* arg);

} // namespace ethercat

#endif // ETHERCAT_BUS_CONFIG_HPP
