// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cpp_runtime_stubs.cpp
 * @brief Provides minimal stubs for C++ runtime functions needed in freestanding environment.
 */
#include <cstddef>     
#include <cstdint>     
#include <new>         

// miniOS.hpp provides kernel::g_platform and kernel::core::Spinlock / ScopedLock
#include "miniOS.hpp" 

// Basic panic function if platform isn't available or if panic is called too early
[[noreturn]] static void basic_halt_loop() {
    // Minimalistic halt
    for (;;) {
        asm volatile("wfi");
    }
}

// --- Minimal new/delete ---
// Using a simple bump allocator from a static array.
// THIS IS NOT THREAD-SAFE without the lock and VERY LIMITED.
#define KERNEL_SIMPLE_HEAP_SIZE (1024 * 64) // 64KB tiny heap
[[gnu::aligned(16)]] static char simple_kernel_heap[KERNEL_SIMPLE_HEAP_SIZE];
static size_t simple_kernel_heap_ptr = 0;
static kernel::core::Spinlock g_simple_heap_lock; 

// This global object needs its constructor called by `call_constructors` in assembly.
// Itself uses `kernel::core::Spinlock()` constructor.
// This assumes `call_constructors` runs before any `new` is called by other static initializers.

[[noreturn]] static void heap_panic_oom() {
    // Attempt to print, then halt.
    // This is tricky because UART itself might not be ready if panic occurs during very early static init.
    if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
        kernel::g_platform->get_uart_ops()->puts("PANIC: operator new - Out Of Memory!\n");
    }
    basic_halt_loop();
}

void* operator new(size_t size) noexcept { 
    kernel::core::ScopedLock lock(g_simple_heap_lock); // Assumes g_simple_heap_lock is initialized
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

// std::nothrow_t is a tag type. std::nothrow is an object of this type.
// The <new> header in a freestanding environment should provide these.
// If linker complains about `_ZSt7nothrow` (mangled name for std::nothrow),
// uncommenting this might help, but it's better if the toolchain's <new> provides it.
/*
namespace std {
    const nothrow_t nothrow = {};
}
*/

void* operator new(size_t size, const std::nothrow_t&) noexcept {
    kernel::core::ScopedLock lock(g_simple_heap_lock);
    size = (size + 15) & ~static_cast<size_t>(15);
    if (simple_kernel_heap_ptr + size > KERNEL_SIMPLE_HEAP_SIZE) {
        return nullptr; 
    }
    void* p = &simple_kernel_heap[simple_kernel_heap_ptr];
    simple_kernel_heap_ptr += size;
    return p;
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
    return ::operator new(size, std::nothrow); 
}

void operator delete(void* ptr) noexcept { (void)ptr; }
void operator delete[](void* ptr) noexcept { (void)ptr; }
void operator delete(void* ptr, size_t size) noexcept { (void)ptr; (void)size; }
void operator delete[](void* ptr, size_t size) noexcept { (void)ptr; (void)size; }

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
    
    // Guard variable for static local initialization. This needs a lock.
    static kernel::core::Spinlock g_cxa_guard_lock; 
    
    int __cxa_guard_acquire(uint64_t* guard_object) {
        // This assumes guard_object is 64-bit and first byte indicates init status
        kernel::core::ScopedLock lock(g_cxa_guard_lock);
        if (*reinterpret_cast<volatile uint8_t*>(guard_object) == 0) { 
            return 1; 
        }
        return 0; 
    }

    void __cxa_guard_release(uint64_t* guard_object) {
        kernel::core::ScopedLock lock(g_cxa_guard_lock);
        *reinterpret_cast<volatile uint8_t*>(guard_object) = 1; 
    }

    void __cxa_guard_abort(uint64_t* guard_object) { 
        // If initialization fails, the lock taken in __cxa_guard_acquire will be released.
        // The guard_object itself might need to be reset if retries are possible,
        // but typically they are not for static locals.
        (void)guard_object; 
    }

    // Minimal stack unwinding stub if absolutely required by libgcc for some reason
    // (even with -fno-exceptions).
    [[noreturn]] void _Unwind_Resume() { basic_halt_loop(); }
    // You might also need stubs for:
    // _Unwind_RaiseException, _Unwind_DeleteException, etc. if they appear as undefined.
    // For AArch64, personality routines are often __gxx_personality_v0 or __aeabi_unwind_cpp_pr0/1.
    // void __gxx_personality_v0() {} // Empty stub example
}