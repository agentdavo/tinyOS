// SPDX-License-Identifier: MIT OR Apache-2.0
// Definition for the per-core activity counters declared in cpu_load.hpp.

#include "diag/cpu_load.hpp"

namespace diag {

std::array<CoreCounters, kernel::core::MAX_CORES> g_core_counters{};

} // namespace diag
