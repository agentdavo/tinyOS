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

// Basic panic function if platform isn't available or if panic is called too early
[[noreturn]] static void basic_halt_loop() {
    for (;;) {
        asm volatile("wfi");
    }
}

// --- Minimal new/delete ---
#define KERNEL_SIMPLE_HEAP_SIZE (1024 * 64) 
[[gnu::aligned(16)]] static char simple_kernel_heap[KERNEL_SIMPLE_HEAP_SIZE];
static size_t simple_kernel_heap_ptr = 0;
// This global Spinlock relies on its own constructor being called by call_constructors.
// If new/delete is used by another static constructor *before* this lock is initialized,
// it's unsafe. For truly safe early new/delete, the lock itself needs to be simpler
// (e.g., a raw atomic_flag if used before full Spinlock construction).
// Assuming for now that static constructors using new/delete run after g_simple_heap_lock's constructor.
static kernel::core::Spinlock g_simple_heap_lock; 

[[noreturn]] static void heap_panic_oom() {
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
    
    int __cxa_guard_acquire(uint64_t* guard_object) {
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

    void __cxa_guard_abort(uint64_t* guard_object) { (void)guard_object; }
    
    [[noreturn]] void _Unwind_Resume() { 
        if (kernel::g_platform && kernel::g_platform->get_uart_ops()) {
             kernel::g_platform->get_uart_ops()->puts("PANIC: _Unwind_Resume called!\n");
        }
        basic_halt_loop();
    }
    // void __gxx_personality_v0() {} // Add if linker asks
}