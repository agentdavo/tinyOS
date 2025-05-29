// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file freestanding_stubs.cpp
 * @brief Provides freestanding implementations for common C library functions with C linkage.
 */

#include <cstddef> // For size_t
#include <cstdint> // For integer types

// These are the raw C-linkage implementations.
extern "C" {

void* memcpy_c(void* dest_ptr, const void* src_ptr, size_t count) {
    unsigned char* dest = static_cast<unsigned char*>(dest_ptr);
    const unsigned char* src = static_cast<const unsigned char*>(src_ptr);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = src[i];
    }
    return dest_ptr;
}

void* memset_c(void* dest_ptr, int ch_int, size_t count) {
    unsigned char* dest = static_cast<unsigned char*>(dest_ptr);
    unsigned char ch = static_cast<unsigned char>(ch_int);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = ch;
    }
    return dest_ptr;
}

int memcmp_c(const void* ptr1, const void* ptr2, size_t count) {
    const unsigned char* p1 = static_cast<const unsigned char*>(ptr1);
    const unsigned char* p2 = static_cast<const unsigned char*>(ptr2);
    for (size_t i = 0; i < count; ++i) {
        if (p1[i] != p2[i]) {
            return (p1[i] < p2[i]) ? -1 : 1;
        }
    }
    return 0;
}

size_t strlen_c(const char* str) {
    if (!str) return 0;
    size_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

char* strcpy_c(char* dest, const char* src) {
    if (!dest || !src) return dest; 
    char* orig_dest = dest;
    while ((*dest++ = *src++)) {}
    return orig_dest;
}

char* strncpy_c(char* dest, const char* src, size_t count) {
    if (!dest || !src) return dest;
    char* orig_dest = dest;
    size_t i;
    // Copy up to 'count' characters or until null terminator in src
    for (i = 0; i < count && src[i] != '\0'; ++i) {
        dest[i] = src[i];
    }
    // If src was shorter than count, pad dest with nulls up to count
    for (; i < count; ++i) { 
        dest[i] = '\0';
    }
    // Note: if src_len >= count, dest might not be null-terminated by this logic alone
    // if null terminator wasn't copied in the first loop.
    // However, standard strncpy behaves this way. safe_strcpy handles guaranteed termination.
    return orig_dest;
}

int strcmp_c(const char* lhs, const char* rhs) {
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

int strncmp_c(const char* lhs, const char* rhs, size_t count) {
    if (count == 0) return 0;
    if (!lhs && !rhs) return 0;
    if (!lhs) return -1;
    if (!rhs) return 1;
    
    size_t i = 0;
    while (i < count && lhs[i] && rhs[i] && (lhs[i] == rhs[i])) {
        i++;
    }
    if (i == count) return 0; // Compared 'count' characters and all matched
    
    // They differ at lhs[i] and rhs[i], or one string ended.
    return static_cast<int>(static_cast<unsigned char>(lhs[i])) - 
           static_cast<int>(static_cast<unsigned char>(rhs[i]));
}

} // extern "C"