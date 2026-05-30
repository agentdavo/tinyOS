// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cpp_runtime_stubs.cpp
 * @brief Provides minimal stubs for C++ runtime functions needed in freestanding environment.
 */
#include <cstddef>     
#include <cstdint>     
#include <new>         // For std::nothrow_t declaration

#include "miniOS.hpp"  // For kernel::g_platform
#include "core.hpp"    // For kernel::core::Spinlock, kernel::core::ScopedLock

extern "C" {
    void* __dso_handle = nullptr;
}

// Basic panic function if platform isn't available or if panic is called too early
[[noreturn]] static void basic_halt_loop() {
    for (;;) {
        asm volatile("wfi");
    }
}

// --- Minimal new/delete ---
// Bump allocator backing every kernel `operator new` — never freed, sized for
// the full boot + first-render working set. The biggest consumers:
//   - ui_builder::load_tsv constructs ~1113 BuilderXxx widgets (~220 KB)
//   - render/machine_model.cpp imports several OBJ meshes (~32 KB scratch
//     + per-mesh vertex/index blocks)
//   - each BuilderImage running gles1::Renderer lazily allocates a
//     widget-sized depth buffer (machine_view's preview pane is the
//     largest at ~700 KB)
//   - misc kernel bring-up paths (a few tens of KB)
// The previous 256 KB cap OOM-panicked mid-tsv_load (every page rendered
// blank, screenshots committed 336-byte placeholders); 2 MB still OOM'd at
// machine_view's first render. 8 MB carries the full UI page-walk with
// headroom. Costs 7.75 MB of .bss against the 128 MB QEMU image.
#define KERNEL_SIMPLE_HEAP_SIZE (1024 * 1024 * 8)
[[gnu::aligned(16)]] static char simple_kernel_heap[KERNEL_SIMPLE_HEAP_SIZE];
static size_t simple_kernel_heap_ptr = 0;
// This global Spinlock relies on its own constructor being called by call_constructors.
// If new/delete is used by another static constructor *before* this lock is initialized,
// it's unsafe. For truly safe early new/delete, the lock itself needs to be simpler
// (e.g., a raw atomic_flag if used before full Spinlock construction).
// Assuming for now that static constructors using new/delete run after g_simple_heap_lock's constructor.
static kernel::core::Spinlock g_simple_heap_lock; 

// Best-effort emergency safe-stop hook. Strong definition lives in the motion
// subsystem (motion.cpp), which drops every axis to a commanded stop + brake
// engage and trips the EtherCAT masters to QuickStop. Weak here so the kernel
// still links if motion is configured out; the weak no-op just falls through
// to the halt. Declared C-linkage so the motion side can provide it plainly.
extern "C" void minios_emergency_safe_stop() __attribute__((weak));
extern "C" void minios_emergency_safe_stop() {}

[[noreturn]] static void heap_panic_oom() {
    // Drive a safe stop BEFORE halting — an OOM on a machine controller must
    // not leave motors energised under an uncontrolled (now-frozen) kernel.
    minios_emergency_safe_stop();
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        kernel::g_platform->get_uart_ops()->puts("PANIC: operator new - Out Of Memory!\n");
    }
    basic_halt_loop();
}

// Define the std::nothrow object.
// The type std::nothrow_t should be in <new>.
// Explicitly call the constructor.
namespace std {
    const nothrow_t nothrow = nothrow_t(); // Explicit constructor call
}

namespace std {
    [[noreturn]] void terminate() noexcept {
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
            kernel::g_platform->get_uart_ops()->puts("PANIC: std::terminate() called!\n");
        }
        for (;;) asm volatile("wfi");
    }
}

extern "C" [[noreturn]] void _ZSt24__throw_out_of_range_fmtPKcz(const char*, ...) {
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        kernel::g_platform->get_uart_ops()->puts("PANIC: std::out_of_range thrown!\n");
    }
    for (;;) asm volatile("wfi");
}

extern "C" void __gxx_personality_v0() { for (;;) asm volatile("wfi"); }
extern "C" void __cxa_deleted_virtual() { for (;;) asm volatile("wfi"); }
extern "C" unsigned long __getauxval(unsigned long type) { (void)type; return 0; }

void* operator new(size_t size) noexcept { 
    kernel::core::ScopedLock lock(g_simple_heap_lock); 
    size = (size + 15) & ~static_cast<size_t>(15); 
    if (simple_kernel_heap_ptr + size > KERNEL_SIMPLE_HEAP_SIZE) {
        heap_panic_oom(); 
    }
    void* p = &simple_kernel_heap[simple_kernel_heap_ptr];
    simple_kernel_heap_ptr += size;
    return p;
}

void* operator new[](size_t size) noexcept { 
    return ::operator new(size); 
}

// Use ::std::nothrow to ensure we are referring to the global one we defined.
void* operator new(size_t size, const ::std::nothrow_t&) noexcept {
    kernel::core::ScopedLock lock(g_simple_heap_lock);
    size = (size + 15) & ~static_cast<size_t>(15);
    if (simple_kernel_heap_ptr + size > KERNEL_SIMPLE_HEAP_SIZE) {
        return nullptr; 
    }
    void* p = &simple_kernel_heap[simple_kernel_heap_ptr];
    simple_kernel_heap_ptr += size;
    return p;
}

void* operator new[](size_t size, const ::std::nothrow_t&) noexcept {
    return ::operator new(size, std::nothrow); 
}

void operator delete(void* ptr) noexcept { (void)ptr; }
void operator delete[](void* ptr) noexcept { (void)ptr; }
void operator delete(void* ptr, size_t size) noexcept { (void)ptr; (void)size; }
void operator delete[](void* ptr, size_t size) noexcept { (void)ptr; (void)size; }
void operator delete(void* ptr, size_t size, std::align_val_t) noexcept { (void)ptr; (void)size; }
void operator delete[](void* ptr, size_t size, std::align_val_t) noexcept { (void)ptr; (void)size; }
void operator delete(void* ptr, std::align_val_t) noexcept { (void)ptr; }
void operator delete[](void* ptr, std::align_val_t) noexcept { (void)ptr; }

// --- C++ ABI Stubs ---
extern "C" {
    [[noreturn]] void __cxa_pure_virtual() { 
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
             kernel::g_platform->get_uart_ops()->puts("PANIC: Pure virtual function call!\n");
        }
        basic_halt_loop();
    }

    int __cxa_atexit(void (*func)(void*), void* arg, void* dso_handle) {
        (void)func; (void)arg; (void)dso_handle; return 0; 
    }
    
    static kernel::core::Spinlock g_cxa_guard_lock;

    // Itanium C++ ABI guard for function-local statics. The old implementation
    // dropped the lock before returning 1, so two cores hitting the same
    // uninitialised static both saw byte0==0 and both ran the constructor —
    // a double-construction race on SMP. Use byte0 as the "initialised" flag
    // (what the compiler tests inline) and byte1 as an "in progress" claim so
    // exactly one core runs the ctor while the others spin until it finishes.
    // The ctor runs *outside* the lock (between acquire returning 1 and
    // release), so the critical sections stay short.
    int __cxa_guard_acquire(uint64_t* guard_object) {
        volatile uint8_t* gb = reinterpret_cast<volatile uint8_t*>(guard_object);
        for (;;) {
            {
                kernel::core::ScopedLock lock(g_cxa_guard_lock);
                if (gb[0]) return 0;          // already initialised
                if (gb[1] == 0) {             // claim the initialisation
                    gb[1] = 1;
                    return 1;
                }
            }
            // Another core is mid-initialisation. Spin until it commits
            // (byte0 set) or aborts (byte1 cleared), then re-evaluate.
            for (;;) {
                kernel::core::ScopedLock lock(g_cxa_guard_lock);
                if (gb[0]) return 0;          // peer finished
                if (gb[1] == 0) break;        // peer aborted; retry the claim
            }
        }
    }

    void __cxa_guard_release(uint64_t* guard_object) {
        kernel::core::ScopedLock lock(g_cxa_guard_lock);
        volatile uint8_t* gb = reinterpret_cast<volatile uint8_t*>(guard_object);
        gb[0] = 1;   // mark initialised (the flag the compiler checks)
        gb[1] = 0;   // clear the in-progress claim
    }

    void __cxa_guard_abort(uint64_t* guard_object) {
        kernel::core::ScopedLock lock(g_cxa_guard_lock);
        volatile uint8_t* gb = reinterpret_cast<volatile uint8_t*>(guard_object);
        gb[1] = 0;   // clear the claim so a retry can re-attempt init
    }
    
    [[noreturn]] void _Unwind_Resume() { 
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
             kernel::g_platform->get_uart_ops()->puts("PANIC: _Unwind_Resume called!\n");
        }
        basic_halt_loop();
    }
    // void __gxx_personality_v0() {} // Add if linker asks
}
