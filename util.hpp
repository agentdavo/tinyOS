// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.hpp
 * @brief Freestanding utility functions header for miniOS v1.7.
 */

#ifndef UTIL_HPP
#define UTIL_HPP

#include <string_view> 
#include <cstddef>     
#include <cstdint>     
#include <limits>      
#include <span>        
#include <cstdarg>      

// Declare the C-linkage functions that are defined in freestanding_stubs.cpp
// This makes them visible to C++ code in the global namespace.
extern "C" {
    void* memcpy(void* dest, const void* src, size_t count);
    void* memset(void* dest, int ch, size_t count);
    int memcmp(const void* ptr1, const void* ptr2, size_t count);
    size_t strlen(const char* str);
    int strcmp(const char* lhs, const char* rhs);
    int strncmp(const char* lhs, const char* rhs, size_t count);
    char* strcpy(char* dest, const char* src);
    char* strncpy(char* dest, const char* src, size_t count);
}


namespace kernel {
namespace util {

// Inline wrappers in the kernel::util namespace calling global extern "C" versions
// These provide a namespaced API for the rest of the kernel.
inline void* kmemcpy(void* dest, const void* src, size_t count) noexcept {
    return ::memcpy(dest, src, count); // Calls global C memcpy
}

inline void* kmemset(void* dest, int ch, size_t count) noexcept {
    return ::memset(dest, ch, count); // Calls global C memset
}

inline int kmemcmp(const void* ptr1, const void* ptr2, size_t count) noexcept {
    return ::memcmp(ptr1, ptr2, count); 
}

inline size_t kstrlen(const char* str) noexcept {
    // The global ::strlen already handles null, but an extra check here is harmless.
    if (!str) return 0; 
    return ::strlen(str); 
}

inline int kstrcmp(const char* lhs, const char* rhs) noexcept {
    return ::strcmp(lhs, rhs);
}

inline int kstrncmp(const char* lhs, const char* rhs, size_t count) noexcept {
    return ::strncmp(lhs, rhs, count); 
}

inline char* kstrcpy(char* dest, const char* src) noexcept {
    return ::strcpy(dest, src); 
}

inline char* kstrncpy(char* dest, const char* src, size_t count) noexcept {
    return ::strncpy(dest, src, count); 
}

// Declarations for functions defined in util.cpp
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept; 
char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept; 

// Character functions (can be inline as they are simple)
inline bool isspace(char c) noexcept { 
    return (c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r');
}
inline bool isdigit(char c) noexcept { 
    return (c >= '0' && c <= '9');
}
bool isalpha(char c) noexcept; 
bool isalnum(char c) noexcept; 
int toupper(int c) noexcept;   
int tolower(int c) noexcept;   

// String to number conversion
bool str_to_int32(std::string_view input, int32_t& out_val) noexcept;
bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept;
bool str_to_float(std::string_view input, float& out_val) noexcept; 

// IP address conversion
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept;

// Number to string conversion helpers (definitions in util.cpp)
int int_to_str(int32_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint_to_str(uint32_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint64_to_str(uint64_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint64_to_hex_str(uint64_t value, char* buffer, size_t buffer_size, bool leading_0x = true) noexcept;

void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept; 

std::string_view get_next_token(std::string_view& input, char delimiter) noexcept;

// Simplified snprintf-like functions (definitions in util.cpp)
int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept;
int k_snprintf(char* buffer, size_t bufsz, const char* format, ...) noexcept __attribute__((format(printf, 3, 4)));


template <typename T>
constexpr const T& min(const T& a, const T& b) { return (b < a) ? b : a; }
template <typename T>
constexpr const T& max(const T& a, const T& b) { return (a < b) ? b : a; }

} // namespace util
} // namespace kernel

#endif // UTIL_HPP