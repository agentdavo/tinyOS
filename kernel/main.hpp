// SPDX-License-Identifier: MIT OR Apache-2.0
#ifndef KERNEL_MAIN_HPP
#define KERNEL_MAIN_HPP

#include <cstdint>

extern "C" void kernel_main();
extern "C" void kernel_secondary_main(uint32_t core_id);

namespace kernel {

struct BootHooks {
    using SecondaryReleaseFn = void (*)(uint32_t core_id, void* entry_fn, uint32_t arg);
    using SecondaryEntryFn  = void (*)(uint32_t core_id);

    bool (*supports_multicore)();
    void (*init_secondary_core)(uint32_t core_id);
    void (*post_platform_init)();
};

void register_boot_hooks(const BootHooks* hooks);
bool has_multicore_support();

namespace boot {

// Unified create-thread signature used by the shared boot helpers below.
// arm64 forwards straight to kernel::g_scheduler_ptr->create_thread. rv64
// wraps sched_create_thread (FIFO, no priority/deadline) and ignores prio /
// deadline_us for now — those merge away once task #30 folds the two
// schedulers together.
using CreateThreadFn = bool (*)(void (*fn)(void*), void* arg, int prio,
                                int affinity, const char* name, bool is_idle,
                                uint64_t deadline_us);

// Register embedded defaults in the VFS, probe the block device (virtio-blk),
// mount FAT32 if present, and shadow-register on-disk files over embedded
// defaults. Safe to call even when no block device is attached — the VFS
// then runs in embedded-defaults-only mode.
void init_vfs();

// Load every embedded TSV blob (UI template, macros, ladder, toolpods,
// signals, topology, placement, hmi) into its owning service. Safe to call
// once during boot; both arches used to duplicate this block.
void load_runtime_tsvs();

// Log each net-role (hmi / ec_a / fake_slave or ec_b) and route its NIC IRQ
// to the placement-selected core. Reads placement via the shared service.
void log_and_route_net_roles();

// Create the boot-time services (cli, uart_io, ui). Must run after
// `load_runtime_tsvs` so placement.ui_core / cli_core are correct.
void create_boot_services(CreateThreadFn create_thread);

// Create the runtime workers (motion, gcode, macro, ladder, probe, hmi,
// ec_a, ec_b or fake_slave, bus_cfg). Must run after `load_runtime_tsvs`.
void create_runtime_services(CreateThreadFn create_thread);

} // namespace boot

}

#endif // KERNEL_MAIN_HPP
