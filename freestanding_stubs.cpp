// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file freestanding_stubs.cpp
 * @brief Provides freestanding implementations for common C library functions and atomic builtins.
 */

#include <cstddef> // For size_t
#include <cstdint> // For integer types

// extern "C" functions
extern "C" {

// --- Memory and String Functions ---
void* memcpy(void* dest_ptr, const void* src_ptr, size_t count) {
    auto* dest = static_cast<unsigned char*>(dest_ptr);
    const auto* src = static_cast<const unsigned char*>(src_ptr);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = src[i];
    }
    return dest_ptr;
}

void* memset(void* dest_ptr, int ch_int, size_t count) {
    auto* dest = static_cast<unsigned char*>(dest_ptr);
    unsigned char ch = static_cast<unsigned char>(ch_int);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = ch;
    }
    return dest_ptr;
}

int memcmp(const void* ptr1, const void* ptr2, size_t count) {
    const auto* p1 = static_cast<const unsigned char*>(ptr1);
    const auto* p2 = static_cast<const unsigned char*>(ptr2);
    for (size_t i = 0; i < count; ++i) {
        if (p1[i] != p2[i]) {
            return (p1[i] < p2[i]) ? -1 : 1;
        }
    }
    return 0;
}

size_t strlen(const char* str) {
    if (!str) return 0; 
    size_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

char* strcpy(char* dest, const char* src) {
    if (!dest || !src) return dest; 
    char* orig_dest = dest;
    while ((*dest++ = *src++)) {}
    return orig_dest;
}

char* strncpy(char* dest, const char* src, size_t count) {
    if (!dest || !src) return dest;
    char* orig_dest = dest;
    size_t i;
    for (i = 0; i < count && src[i] != '\0'; ++i) {
        dest[i] = src[i];
    }
    for (; i < count; ++i) { 
        dest[i] = '\0';
    }
    return orig_dest;
}

int strcmp(const char* lhs, const char* rhs) {
    if (!lhs && !rhs) return 0;
    if (!lhs) return -1; 
    if (!rhs) return 1;
    while (*lhs && (*lhs == *rhs)) {
        lhs++;
        rhs++;
    }
    return static_cast<int>(static_cast<unsigned char>(*lhs)) - 
           static_cast<int>(static_cast<unsigned char>(*rhs));
}

int strncmp(const char* lhs, const char* rhs, size_t count) {
    if (count == 0) return 0;
    if (!lhs && !rhs) return 0; 
    if (!lhs) return -1; 
    if (!rhs) return 1;
    
    size_t i = 0;
    while (i < count && lhs[i] && rhs[i] && (lhs[i] == rhs[i])) {
        if (lhs[i] == '\0') { 
            return 0;
        }
        i++;
    }
    if (i == count) return 0; 
    
    return static_cast<int>(static_cast<unsigned char>(lhs[i])) - 
           static_cast<int>(static_cast<unsigned char>(rhs[i]));
}

// --- Atomic Builtin Implementations for AArch64 ---
// These use inline assembly with Load-Exclusive (LDXR) and Store-Exclusive (STXR)
// instructions, which are part of ARMv8.0-A.

// unsigned char __aarch64_cas1_acq(unsigned char*, unsigned char, unsigned char)
// Atomic Compare-And-Swap byte with acquire semantics.
// Arguments: x0: memory address (ptr)
//            w1: expected value (oldval)
//            w2: desired value (newval)
// Return: old value from memory (in w0)
// Note: The __atomic_compare_exchange_n builtin expects the 'expected' value to be passed by pointer
// and updated if the CAS fails. The __aarch64_cas* builtins often just return the old value.
// We will implement the behavior of __sync_val_compare_and_swap which is similar.
// The signature GCC expects for __aarch64_cas1_acq might be slightly different in what it returns
// (e.g., a status or the old value). Let's implement a common CAS pattern that returns the old value.
// If the signature is different (e.g., returns bool success, updates expected by ref), this needs adjustment.
// Based on common __sync_val_compare_and_swap, it returns the *original* value at *ptr.
// The std::atomic version returns bool and updates expected.
// The __aarch64_cas<N> builtins usually return the value read from memory before the store.
// The prototype for __atomic_compare_exchange_N from GCC docs is:
//   bool __atomic_compare_exchange_N (type *ptr, type *expected, type desired, bool weak, int success_memorder, int failure_memorder)
// The __aarch64_cas<size>_<memorder> are lower-level and might be:
//   type __aarch64_cas<size>_<memorder> (type *mem, type oldval, type newval); returning original *mem
// Let's try implementing that:
uint8_t __aarch64_cas1_acq(volatile uint8_t* mem, uint8_t oldval, uint8_t newval) {
    uint8_t read_val;
    uint32_t success;
    asm volatile (
        "1: ldaxrb   %w[read_val], [%[mem]]\n"      // Load-Acquire Exclusive Byte
        "   cmp     %w[read_val], %w[oldval]\n"    // Compare with expected old value
        "   b.ne    2f\n"                          // If not equal, CAS fails, store read_val in oldval
        "   stlxrb   %w[success], %w[newval], [%[mem]]\n" // Store-Release Exclusive Byte
        "   cbnz    %w[success], 1b\n"            // If store failed (contention), retry from load
        "2:"
        // Output operands: read_val will contain value from memory, success will be 0 if STLRXB succeeded
        : [read_val] "=&r"(read_val), [success] "=&r"(success), "+Q"(*mem) // +Q means memory operand
        // Input operands
        : [mem] "r"(mem), [oldval] "r"(oldval), [newval] "r"(newval)
        // Clobbers
        : "cc", "memory" 
    );
    return read_val; // Return the value read from memory (which is oldval if CAS succeeded)
}


// uint64_t __aarch64_ldadd8_relax(uint64_t *mem, uint64_t val)
// Atomically: tmp = *mem; *mem += val; return tmp; (Relaxed semantics)
// Arguments: x0=mem, x1=val. Returns old value in x0.
uint64_t __aarch64_ldadd8_relax(volatile uint64_t* mem, uint64_t val) {
    uint64_t old_val;
    uint64_t new_val;
    uint32_t success; // For STXR status
    asm volatile (
        "1: ldxr    %[old_val], [%[mem]]\n"            // Load-Exclusive Register
        "   add     %[new_val], %[old_val], %[val]\n"  // Add
        "   stxr    %w[success], %[new_val], [%[mem]]\n"// Store-Exclusive Register
        "   cbnz    %w[success], 1b\n"                 // If failed (somebody else wrote), retry
        // Output operands: old_val will contain original *mem, new_val and success are scratch
        : [old_val] "=&r"(old_val), [new_val] "=&r"(new_val), [success] "=&r"(success), "+Q"(*mem)
        // Input operands
        : [mem] "r"(mem), [val] "r"(val)
        // Clobbers
        : "cc", "memory"
    );
    return old_val;
}

// uint8_t __aarch64_swp1_rel(uint8_t* mem, uint8_t val)
// Atomically: tmp = *mem; *mem = val; return tmp; (Release semantics for the store)
// Arguments: x0=mem, w1=val. Returns old value in w0.
uint8_t __aarch64_swp1_rel(volatile uint8_t* mem, uint8_t val) {
    uint8_t old_val;
    uint32_t success;
    asm volatile (
        // LDAXR provides acquire semantics for the load part of the RMW.
        // STLXR provides release semantics for the store part of the RMW.
        "1: ldaxrb  %w[old_val], [%[mem]]\n"             // Load-Acquire Exclusive Byte
        "   stlxrb  %w[success], %w[val], [%[mem]]\n"    // Store-Release Exclusive Byte
        "   cbnz    %w[success], 1b\n"                   // Retry if store failed
        : [old_val] "=&r"(old_val), [success] "=&r"(success), "+Q"(*mem)
        : [mem] "r"(mem), [val] "r"(val)
        : "cc", "memory"
    );
    return old_val;
}

} // extern "C"