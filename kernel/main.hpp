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

}

#endif // KERNEL_MAIN_HPP
