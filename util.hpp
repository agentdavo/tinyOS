// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.hpp
 * @brief Utility functions header for miniOS v1.7.
 * @details
 * Declares common utility functions like string manipulation, memory operations,
 * and parsing, used across various subsystems of miniOS.
 *
 * @version 1.7
 * @see util.cpp, miniOS.hpp
 */

#ifndef UTIL_HPP
#define UTIL_HPP

#include <string_view>
#include <cstddef>
#include <cstdint>
#include <cstring>   // For std::memcpy, std::memset, std::strlen, std::strcmp, std::strncpy
#include <limits>    // For std::numeric_limits
#include <cstdlib>   // For std::strtol, std::strtoul, std::strtof
#include <span>      // For std::span

namespace kernel {
namespace util {

// Use std versions directly or wrap them if specific behavior/portability is needed.
// The provided implementations are mostly wrappers around std functions.
// For a bare-metal OS, you might implement these from scratch or use a freestanding library.

inline void* memcpy(void* dest, const void* src, size_t count) noexcept {
    return std::memcpy(dest, src, count);
}

inline void* memset(void* dest, int ch, size_t count) noexcept { // Changed src to dest to match std::memset
    return std::memset(dest, ch, count);
}

inline size_t strlen(const char* str) noexcept {
    if (!str) return 0;
    return std::strlen(str);
}

inline int strcmp(const char* lhs, const char* rhs) noexcept {
    if (!lhs && !rhs) return 0;
    if (!lhs) return -1; // lhs is null, rhs is not
    if (!rhs) return 1;  // rhs is null, lhs is not
    return std::strcmp(lhs, rhs);
}

/**
 * @brief Safely copies a C-string. Ensures null termination.
 * @param dest Destination buffer.
 * @param src Source string.
 * @param dest_size Size of the destination buffer.
 * @return True if the entire string was copied without truncation, false otherwise.
 */
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept;

bool str_to_int32(std::string_view input, int32_t& out_val) noexcept;
bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept;
bool str_to_float(std::string_view input, float& out_val) noexcept;
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept;
void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept;

/**
 * @brief Extracts the next token from a string_view, advancing the view.
 * @param input The string_view to parse. Modified to point past the extracted token and delimiter.
 * @param delimiter The character delimiting tokens.
 * @return A string_view of the token. Empty if no token found or input is empty.
 *         Leading/trailing whitespace around the token is trimmed.
 */
std::string_view get_next_token(std::string_view& input, char delimiter) noexcept;

} // namespace util
} // namespace kernel

#endif // UTIL_HPP