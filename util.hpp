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
#include <cstddef> // For size_t
#include <cstdint> // For uintX_t types
#include <cstring> // For C-string function declarations (though often in std in C++)
#include <limits>  // For std::numeric_limits
#include <cstdlib> // For std::strtol, std::strtoul, std::strtof
#include <span>    // For std::span (used in uint32_to_ipv4_str)


namespace kernel { // <<<<< WRAP IN NAMESPACE KERNEL
namespace util {

/**
 * @brief Copies memory area.
 * @param dest Pointer to the destination array where the content is to be copied.
 * @param src Pointer to the source of data to be copied.
 * @param count Number of bytes to copy.
 * @return Pointer to dest.
 */
void* memcpy(void* dest, const void* src, size_t count) noexcept;

/**
 * @brief Fills a block of memory.
 * @param dest Pointer to the block of memory to fill.
 * @param ch Value to be set. The value is passed as an int, but the function fills the block of memory using the unsigned char conversion of this value.
 * @param count Number of bytes to be set to the value.
 * @return Pointer to the memory area dest.
 */
void* memset(void* dest, int ch, size_t count) noexcept;

/**
 * @brief Gets the length of a C-style string.
 * @param str Pointer to the null-terminated string.
 * @return The length of the string. Returns 0 if str is nullptr.
 */
size_t strlen(const char* str) noexcept;

/**
 * @brief Compares two C-style strings.
 * @param lhs Pointer to the first string.
 * @param rhs Pointer to the second string.
 * @return An integral value indicating the relationship between the strings:
 *         <0, if lhs is less than rhs.
 *         0, if lhs is equal to rhs.
 *         >0, if lhs is greater than rhs.
 *         Behavior is defined if one is nullptr and other is not.
 */
int strcmp(const char* lhs, const char* rhs) noexcept;

/**
 * @brief Safely copies a C-style string.
 * @details Ensures null-termination even if truncation occurs.
 * @param dest Pointer to the destination buffer.
 * @param src Pointer to the source string.
 * @param dest_size Size of the destination buffer.
 * @return True if the string was copied without truncation, false otherwise (or if input invalid).
 */
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept;

/**
 * @brief Converts a string_view to a 32-bit signed integer.
 * @param input The string_view to convert.
 * @param[out] out_val Reference to store the converted integer.
 * @return True if conversion was successful and complete, false otherwise.
 */
bool str_to_int32(std::string_view input, int32_t& out_val) noexcept;

/**
 * @brief Converts a string_view to a 32-bit unsigned integer.
 * @param input The string_view to convert.
 * @param[out] out_val Reference to store the converted integer.
 * @return True if conversion was successful and complete, false otherwise.
 */
bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept;

/**
 * @brief Converts a string_view to a float.
 * @param input The string_view to convert.
 * @param[out] out_val Reference to store the converted float.
 * @return True if conversion was successful and complete, false otherwise.
 */
bool str_to_float(std::string_view input, float& out_val) noexcept;

/**
 * @brief Converts an IPv4 address string (e.g., "192.168.1.1") to its uint32_t representation.
 * @param ip_str The string_view containing the IPv4 address.
 * @param[out] ip_addr Reference to store the converted uint32_t address (network byte order).
 * @return True if conversion was successful, false otherwise.
 */
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept;

/**
 * @brief Converts a uint32_t IPv4 address (network byte order) to its string representation.
 * @param ip_addr The uint32_t IP address.
 * @param out_buffer A span of characters to write the string representation into. Must be large enough (e.g., 16 chars for "xxx.xxx.xxx.xxx\0").
 *                   The output is null-terminated if space permits.
 */
void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept;

/**
 * @brief Extracts the next token from an input string_view, delimited by a specified character.
 * @details Modifies the input string_view to remove the extracted token and delimiter.
 * Trims whitespace from the extracted token.
 * @param[in,out] input The string_view to parse. It will be modified.
 * @param delimiter The character separating tokens.
 * @return A string_view of the extracted token. Returns an empty string_view if no token is found or if the token is all whitespace.
 */
std::string_view get_next_token(std::string_view& input, char delimiter) noexcept;

} // namespace util
} // namespace kernel

#endif // UTIL_HPP