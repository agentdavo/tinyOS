// SPDX-License-Identifier: MIT OR Apache-2.0
#include "base_thread.hpp"
#include "miniOS.hpp"
#include "diag/jitter.hpp"

namespace rt {

namespace {

inline void wait_until_ns(uint64_t target_ns) {
    auto* t = kernel::g_platform ? kernel::g_platform->get_timer_ops() : nullptr;
    if (!t) return;
    t->wait_until_ns(target_ns);
}

} // namespace

void base_thread_entry(void* /*arg*/) {
    if (!kernel::g_platform) for (;;) asm volatile("wfi");
    auto* t = kernel::g_platform->get_timer_ops();
    uint64_t next_ns = t->get_system_time_ns();
    constexpr uint64_t PERIOD_NS = BASE_PERIOD_US * 1000ULL;

    for (;;) {
        diag::rt::base.sample(t->get_system_time_ns());
        next_ns += PERIOD_NS;

        // TODO: step pulse generation, fast GPIO. For now the thread is pure
        // jitter-tracking telemetry so we can watch the 25 µs cadence under
        // load from EtherCAT + motion on the other cores.

        if (t->get_system_time_ns() > next_ns) {
            next_ns = t->get_system_time_ns();  // slipped — realign
        } else {
            wait_until_ns(next_ns);
        }
    }
}

} // namespace rt
