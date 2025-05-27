// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.hpp
 * @brief Utility functions for miniOS v1.7.
 * @details
 * Provides string manipulation, numeric conversion, and memory operations for miniOS subsystems
 * (CLI, DSP, networking, file system, GPIO). Optimized for embedded systems with minimal
 * dependencies, thread-safe operations, and C++20 features. Updated in v1.7 with improved
 * documentation, error handling, and modern C++ practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::string_view for efficient string processing
 * - std::span for safe buffer handling
 * - noexcept for exception-free guarantees
 *
 * @version 1.7
 * @see util.cpp, miniOS.hpp, cli.hpp, dsp.hpp, net.hpp, fs.hpp
 */

#ifndef UTIL_HPP
#define UTIL_HPP

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace util {

/**
 * @brief Splits a string into tokens based on a delimiter.
 * @param input Input string view
 * @param delim Delimiter character
 * @param tokens Output vector of string views
 * @return Number of tokens found
 */
size_t split_string(std::string_view input, char delim, std::vector<std::string_view>& tokens) noexcept;

/**
 * @brief Trims leading and trailing whitespace from a string view.
 * @param input Input string view
 * @return Trimmed string view
 */
std::string_view trim(std::string_view input) noexcept;

/**
 * @brief Converts a string view to a 32-bit integer.
 * @param input String view containing a number
 * @param value Output integer value
 * @return True if conversion succeeded, false otherwise
 */
bool str_to_int32(std::string_view input, int32_t& value) noexcept;

/**
 * @brief Converts a string view to a 32-bit unsigned integer.
 * @param input String view containing a number
 * @param value Output unsigned integer value
 * @return True if conversion succeeded, false otherwise
 */
bool str_to_uint32(std::string_view input, uint32_t& value) noexcept;

/**
 * @brief Converts a string view to a float.
 * @param input String view containing a number
 * @param value Output float value
 * @return True if conversion succeeded, false otherwise
 */
bool str_to_float(std::string_view input, float& value) noexcept;

/**
 * @brief Converts an IPv4 address string (e.g., "192.168.1.1") to a 32-bit integer.
 * @param input String view containing the IP address
 * @param addr Output 32-bit address (network byte order)
 * @return True if conversion succeeded, false otherwise
 */
bool ipv4_to_uint32(std::string_view input, uint32_t& addr) noexcept;

/**
 * @brief Safely copies a null-terminated string to a fixed-size buffer.
 * @param dest Destination buffer
 * @param src Source string
 * @param dest_size Size of destination buffer
 * @return True if copy succeeded, false if buffer too small or invalid
 */
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept;

/**
 * @brief Computes the length of a null-terminated string.
 * @param str Input string
 * @return Length of string (excluding null terminator)
 */
size_t strlen(const char* str) noexcept;

/**
 * @brief Compares two null-terminated strings.
 * @param str1 First string
 * @param str2 Second string
 * @return 0 if equal, negative if str1 < str2, positive if str1 > str2
 */
int strcmp(const char* str1, const char* str2) noexcept;

/**
 * @brief Copies memory from source to destination.
 * @param dest Destination buffer
 * @param src Source buffer
 * @param size Number of bytes to copy
 */
void memcpy(void* dest, const void* src, size_t size) noexcept;

/**
 * @brief Sets a memory buffer to a specified value.
 * @param dest Destination buffer
 * @param value Value to set
 * @param size Number of bytes to set
 */
void memset(void* dest, uint8_t value, size_t size) noexcept;

} // namespace util

#endif // UTIL_HPP