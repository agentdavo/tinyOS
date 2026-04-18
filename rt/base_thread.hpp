// SPDX-License-Identifier: MIT OR Apache-2.0
// Base thread: 25 µs cyclic. Mirrors LinuxCNC's "base thread" — step-pulse
// generation and fast I/O. Pinned to core 0 at priority 15, preempting CLI.

#ifndef RT_BASE_THREAD_HPP
#define RT_BASE_THREAD_HPP

#include <cstdint>

namespace rt {

constexpr uint32_t BASE_PERIOD_US = 25;

// Scheduler entry-point — `arg` ignored. Never returns.
void base_thread_entry(void* arg);

} // namespace rt

#endif
