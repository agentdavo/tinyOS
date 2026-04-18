// SPDX-License-Identifier: MIT OR Apache-2.0
// Embedded TSV device descriptions, linked into the kernel image.
//
// There's no filesystem yet, so every TSV lives as a C string in the binary.
// Today that's just the Teknic ClearPath-EC servo drive.

#ifndef DEVICES_EMBEDDED_HPP
#define DEVICES_EMBEDDED_HPP

#include <cstddef>

namespace devices {

extern const char* const clearpath_ec_tsv;
extern const size_t      clearpath_ec_tsv_len;

// Parses every embedded TSV into g_device_db. Call once from kernel_main
// after platform init. Returns false on malformed TSV.
bool load_all_embedded() noexcept;

} // namespace devices

#endif
