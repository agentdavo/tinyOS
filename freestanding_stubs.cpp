// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file freestanding_stubs.cpp
 * @brief Provides freestanding implementations for common C library functions.
 */

#include <cstddef> // For size_t
#include <cstdint> // For integer types

// Provide implementations for functions that might be called by compiler/libgcc
// or were previously provided by the standard C library.

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
    // Loop while characters match, count is positive, and lhs hasn't ended
    while (i < count && lhs[i] && (lhs[i] == rhs[i])) {
        i++;
    }
    // If we've compared 'count' characters or one string ended while matching
    if (i == count) return 0; 
    
    // Otherwise, they differ at lhs[i] and rhs[i] or one string ended.
    return static_cast<int>(static_cast<unsigned char>(lhs[i])) - 
           static_cast<int>(static_cast<unsigned char>(rhs[i]));
}

// Add other stubs as needed by libgcc:
// For AArch64, division is usually hardware. Atomics are the main concern.
// The __aarch64_* atomics should come from libgcc or libatomic.
// If not, you'd need to provide them or use compiler flags like -mno-outline-atomics
// or ensure your target architecture features (like LSE for some atomics) are enabled
// and supported by QEMU if GCC emits those instructions directly.

// Weak alias for __aeabi_memcpy, common for ARM toolchains if ::memcpy isn't picked up for it.
// void __aeabi_memcpy(void *dest, const void *source, size_t n) __attribute__((alias("memcpy")));
// void __aeabi_memset(void *s, size_t n, int c) __attribute__((alias("memset")));
// These aliases can sometimes help if libgcc tries to call the __aeabi_ prefixed versions.
// For aarch64-linux-gnu, it usually expects standard names.

} // extern "C"