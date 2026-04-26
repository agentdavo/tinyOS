// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Empty TU. The rv64 port now uses the shared kernel::core::Scheduler +
// EDFPolicy from core.cpp, the same scheduler arm64 has used since v1.7.
// The arch boundary is just the context-switch primitive
// (cpu_context_switch_rv64 in cpu_rv64.S, called via cpu_context_switch_impl
// in rv64_stubs.cpp) plus the trap dispatcher's preemptive_tick hand-off
// (in hal_qemu_rv64.cpp). This file is kept so existing build-system
// references to rv64_sched.cpp don't break; remove it from the Makefile
// HAL_CPP list to drop it entirely.
