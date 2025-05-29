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

// These C-linkage functions will be defined in freestanding_stubs.cpp
// They are the raw implementations.
extern "C" {
    void* memcpy_c(void* dest, const void* src, size_t count);
    void* memset_c(void* dest, int ch, size_t count);
    int memcmp_c(const void* ptr1, const void* ptr2, size_t count);
    size_t strlen_c(const char* str);
    int strcmp_c(const char* lhs, const char* rhs);
    int strncmp_c(const char* lhs, const char* rhs, size_t count);
    char* strcpy_c(char* dest, const char* src);
    char* strncpy_c(char* dest, const char* src, size_t count);
}


namespace kernel {
namespace util {

// These are the functions our kernel code will call.
// Their implementations will be in util.cpp and will call the _c versions.

void* kmemcpy(void* dest, const void* src, size_t count) noexcept;
void* kmemset(void* dest, int ch, size_t count) noexcept;
int kmemcmp(const void* ptr1, const void* ptr2, size_t count) noexcept;
size_t kstrlen(const char* str) noexcept;
int kstrcmp(const char* lhs, const char* rhs) noexcept;
int kstrncmp(const char* lhs, const char* rhs, size_t count) noexcept;
char* kstrcpy(char* dest, const char* src) noexcept;
char* kstrncpy(char* dest, const char* src, size_t count) noexcept;

bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept; 
char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept; 

// Character functions
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

// Simplified snprintf-like functions
int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept;
int k_snprintf(char* buffer, size_t bufsz, const char* format, ...) noexcept __attribute__((format(printf, 3, 4)));


template <typename T>
constexpr const T& min(const T& a, const T& b) { return (b < a) ? b : a; }
template <typename T>
constexpr const T& max(const T& a, const T& b) { return (a < b) ? b : a; }

} // namespace util
} // namespace kernel

#endif // UTIL_HPP