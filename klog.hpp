// SPDX-License-Identifier: MIT OR Apache-2.0
//
// klog — small in-RAM ring of recent kernel log output, dumpable via the
// `klog` CLI command. Survives any soft fault that doesn't corrupt the
// buffer's RAM, giving a "what was the kernel doing right before things
// went bad" view that's been missing on-target. Mirrors the role the
// host-side screenshots/kernel.log artifact plays in CI: every line that
// goes through the platform UART driver's puts() is captured, so trap
// dumps, [ec*] / [hmi] / [virtio-gpu] / [sched] / [boot] etc. all land
// here automatically without per-call-site changes.
//
// Capacity: 8 KB — small enough to leave for postmortem inspection,
// large enough to hold ~80 typical 100-byte log lines (the boot
// surface) or several hundred shorter lines.
//
// Ordering: the platform UART's puts() holds the early-uart lock when
// it calls record(), so a single producer is in flight at a time and
// the ring's content order matches the wire order.

#pragma once

#include <cstddef>

namespace kernel { namespace hal { struct UARTDriverOps; } }

namespace kernel { namespace klog {

constexpr size_t RING_SIZE = 8192;

// Append `n` bytes to the ring. Old content is overwritten when the
// ring wraps. Safe to call from any thread that holds the same lock
// the rest of UART output uses (the platform UART's puts already
// serialises against itself).
void record(const char* data, size_t n) noexcept;

// Dump the newest content to `uart` via uart->puts(), in chunks. If the
// ring has wrapped, output starts from the byte after the write head
// (oldest still-resident byte) and walks forward. Output ends with
// a "[klog: <bytes> bytes, <total> total]\n" footer.
void dump(kernel::hal::UARTDriverOps* uart) noexcept;

}} // namespace kernel::klog
