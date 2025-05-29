// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file freestanding_stubs.cpp
 * @brief Provides freestanding implementations for common C library functions and atomic builtins.
 */

#include <cstddef> // For size_t
#include <cstdint> // For integer types

// extern "C" functions from previous version (memcpy, memset, etc.)
extern "C" {

void* memcpy(void* dest_ptr, const void* src_ptr, size_t count) {
    unsigned char* dest = static_cast<unsigned char*>(dest_ptr);
    const unsigned char* src = static_cast<const unsigned char*>(src_ptr);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = src[i];
    }
    return dest_ptr;
}

void* memset(void* dest_ptr, int ch_int, size_t count) {
    unsigned char* dest = static_cast<unsigned char*>(dest_ptr);
    unsigned char ch = static_cast<unsigned char>(ch_int);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = ch;
    }
    return dest_ptr;
}

int memcmp(const void* ptr1, const void* ptr2, size_t count) {
    const unsigned char* p1 = static_cast<const unsigned char*>(ptr1);
    const unsigned char* p2 = static_cast<const unsigned char*>(ptr2);
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


// Atomic builtins for AArch64
// These are simplified and might not cover all memory orderings perfectly
// or might not be optimal, but aim to satisfy the linker.

// bool __aarch64_cas1_acq(uint8_t *mem, uint8_t *oldval, uint8_t newval)
// CAS: Compare And Swap. Atomically performs:
//   if (*mem == *oldval) { *mem = newval; return true (or 0 for success by some ABI); }
//   else { *oldval = *mem; return false (or 1 for failure by some ABI); }
// The `acq` means acquire semantics.
// GCC builtins usually return 0 if comparison fails, 1 if succeeds.
// The __aarch64_cas<size>_<memorder> builtins often return the *old* value from memory before the CAS.
// Let's assume a common pattern for CAS: it returns true if successful.
// Arguments: x0 = mem, x1 = oldval_ptr, w2 = newval (for 1 byte)
// We need to return a boolean (int 0 or 1).
// Using LDXRB/STXRB for exclusive load/store for 1-byte CAS.
bool __aarch64_cas1_acq(volatile uint8_t* ptr, uint8_t* expected_val_ptr, uint8_t desired_val) {
    uint8_t current_val;
    uint32_t success; // STXRB status, 0 for success, 1 for failure
    asm volatile (
        "1: ldaxrb   %w[current_val], [%[ptr]]\n"          // Load-acquire exclusive byte
        "   cmp     %w[current_val], %w[expected_val]\n"  // Compare with expected
        "   b.ne    2f\n"                                 // If not equal, skip store
        "   stlxrb   %w[success], %w[desired_val], [%[ptr]]\n" // Store-release exclusive byte
        "   cbnz    %w[success], 1b\n"                   // If store failed (contention), retry
        "2:"
        : [current_val] "=&r"(current_val), [success] "=&r"(success), "+m" (*ptr)
        : [ptr] "r"(ptr), [expected_val] "r"(*expected_val_ptr), [desired_val] "r"(desired_val)
        : "cc", "memory"
    );
    if (current_val != *expected_val_ptr) { // If comparison failed initially
        *expected_val_ptr = current_val; // Update expected with current
        return false;                   // CAS failed
    }
    return success == 0; // CAS succeeded if store exclusive succeeded
}


// uint64_t __aarch64_ldadd8_relax(uint64_t* mem, uint64_t val)
// Atomically performs: *mem += val; returns OLD value of *mem.
// Relaxed memory order.
// LDADDAL on ARMv8.1-LSE, or LDXR/STXR loop for ARMv8.0
// Arguments: x0 = mem_ptr, x1 = val_to_add
// Returns old value in x0.
uint64_t __aarch64_ldadd8_relax(volatile uint64_t* ptr, uint64_t val) {
    uint64_t old_val;
    uint64_t new_val;
    uint32_t success;
    asm volatile (
        "1: ldxr    %[old_val], [%[ptr]]\n"            // Load exclusive
        "   add     %[new_val], %[old_val], %[val]\n"  // Calculate new value
        "   stxr    %w[success], %[new_val], [%[ptr]]\n"// Store exclusive
        "   cbnz    %w[success], 1b\n"                 // Retry if store failed
        : [old_val] "=&r"(old_val), [new_val] "=&r"(new_val), [success] "=&r"(success), "+m" (*ptr)
        : [ptr] "r"(ptr), [val] "r"(val)
        : "cc", "memory"
    );
    return old_val;
}


// uint8_t __aarch64_swp1_rel(uint8_t *mem, uint8_t val)
// Atomically performs: temp = *mem; *mem = val; return temp;
// Release semantics.
// SWPALB on ARMv8.1-LSE, or LDXRB/STXRB loop.
// Arguments: x0 = mem_ptr, w1 = val_to_store
// Returns old value in w0.
uint8_t __aarch64_swp1_rel(volatile uint8_t* ptr, uint8_t val) {
    uint8_t old_val;
    uint32_t success;
    asm volatile (
        "1: ldaxrb  %w[old_val], [%[ptr]]\n"         // Load-acquire exclusive (acquire for the store-release pair)
        "   stlxrb  %w[success], %w[val], [%[ptr]]\n"// Store-release exclusive
        "   cbnz   %w[success], 1b\n"               // Retry if store failed
        : [old_val] "=&r"(old_val), [success] "=&r"(success), "+m" (*ptr)
        : [ptr] "r"(ptr), [val] "r"(val)
        : "cc", "memory"
    );
    return old_val;
}


} // extern "C"