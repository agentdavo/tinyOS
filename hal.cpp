// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file hal.cpp
 * @brief Hardware abstraction layer entry point for miniOS v1.7.
 */

#include "hal.hpp"
#include "hal_qemu_arm64.hpp" 
#include "core.hpp"           
#include "miniOS.hpp"         
#include "util.hpp"
#include <cstdint>    

extern "C" {
    void cpu_context_switch_impl(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb);
}

namespace kernel {
namespace hal {

static ::hal::qemu_virt_arm64::PlatformQEMUVirtARM64 g_platform_instance;
Platform* get_platform() { return &g_platform_instance; }

void cpu_context_switch(kernel::core::TCB* old_tcb, kernel::core::TCB* new_tcb) {
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
         char b[128];
         uint32_t core_id_of_new = new_tcb ? new_tcb->cpu_id_running_on : static_cast<uint32_t>(-1);
         // Using kernel::util::k_snprintf now
         kernel::util::k_snprintf(b, sizeof(b), "HAL_CTXSW: OLD TCB:%p(%s) -> NEW TCB:%p(%s) on Core:%u\n",
            (void*)old_tcb, old_tcb ? old_tcb->name : "NULL",
            (void*)new_tcb, new_tcb ? new_tcb->name : "NULL",
            core_id_of_new
         );
         kernel::g_platform->get_uart_ops()->puts(b);
    }
    if (!new_tcb) {
        if (kernel::g_platform) {
            kernel::g_platform->panic("cpu_context_switch called with NULL new_tcb", __FILE__, __LINE__);
        } else { for(;;); }
    }
    cpu_context_switch_impl(old_tcb, new_tcb);
}

extern "C" void hal_irq_handler(uint32_t core_id) {
    core::ScopedISRLock lock(kernel::g_irq_handler_lock); 
    if (core_id >= core::MAX_CORES || !kernel::g_platform || !kernel::g_platform->get_irq_ops() ||
        !kernel::g_platform->get_timer_ops()) {
        for (;;) asm volatile("nop"); 
        return;
    }
    IRQControllerOps* irq_ops = kernel::g_platform->get_irq_ops();
    TimerDriverOps* timer_ops = kernel::g_platform->get_timer_ops();
    uint32_t irq_id = irq_ops->ack_irq(core_id);

    if (irq_id < 1020) { 
        if (irq_id == kernel::hal::SYSTEM_TIMER_IRQ) { 
            timer_ops->ack_core_timer_interrupt(core_id); 
            if (kernel::g_scheduler_ptr) {
                kernel::g_scheduler_ptr->preemptive_tick(core_id);
            }
            timer_ops->hardware_timer_irq_fired(core_id);
        } else if (irq_id == ::hal::qemu_virt_arm64::IRQ_UART0) { 
            // UART specific handling if interrupt-driven
        } else if (irq_id == ::hal::qemu_virt_arm64::IRQ_VIRTIO_NET) { 
            // VirtIO Net specific handling
        }
        irq_ops->end_irq(core_id, irq_id);
    } else if (irq_id == 1023) {
        // Spurious interrupt
    }
}

namespace sync {
    void barrier_dmb() { asm volatile("dmb sy" ::: "memory"); }
    void barrier_dsb() { asm volatile("dsb sy" ::: "memory"); }
    void barrier_isb() { asm volatile("isb" ::: "memory"); }
}

} // namespace hal
} // namespace kernel