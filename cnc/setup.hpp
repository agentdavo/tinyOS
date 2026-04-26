// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Operator-edited machine setup persistence.
//
// Snapshots the live machine state that an operator can edit at the
// console (work offsets, tool table, calibration tables, geometry,
// active selections) into a small text file so it survives reboot.
// Vendor-neutral text format keyed [section].key=value, parsed
// forgivingly: unknown sections / keys are skipped with a warning so
// future versions stay backward-compatible with older saves.
//
// The actual storage backend is `kernel::vfs::write_blob`, which tries
// the HAL FileSystemOps writer first (real persistence on a writable
// FS) and falls back to an in-RAM shadow that holds the bytes for the
// current kernel session. Today's FAT32 reader is read-only, so a save
// survives `load_setup` but not a power cycle — the format is fixed
// now so the durability upgrade is a backend swap, not a re-design.

#ifndef CNC_SETUP_HPP
#define CNC_SETUP_HPP

namespace cnc::setup {

// Atomically write the live machine setup snapshot to `path` (e.g.
// "setup.cfg"). The persistence layer attempts a write-then-rename via
// the HAL FS backend; if that backend doesn't support write/rename, the
// bytes land in the VFS in-memory shadow instead. Returns true if the
// snapshot was successfully serialized + handed to the storage layer.
bool save_to(const char* path) noexcept;

// Read the setup file at `path` and apply each recognized field to the
// live subsystems via their existing setter APIs (cnc::offsets::g_service
// setters, motion::g_motion cal/geometry/sphere setters, channel
// overrides). Missing file is not a failure (logs an info line, returns
// false). Returns true when bytes were retrieved and at least one
// section was applied.
bool load_from(const char* path) noexcept;

}  // namespace cnc::setup

#endif  // CNC_SETUP_HPP
