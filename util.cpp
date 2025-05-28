// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.cpp
 * @brief Utility functions implementation for miniOS v1.7.
 * @details
 * Provides implementations for common utility functions like string manipulation, memory operations,
 * and parsing, used across various subsystems of miniOS. Ensures safe and efficient operations.
 *
 * New in v1.7:
 * - Renamed from util_v1.6.cpp
 * - Enhanced error handling and safety in string/memory functions
 * - Improved Doxygen comments
 *
 * @version 1.7
 * @see util.hpp, miniOS.hpp
 */

#include "util.hpp"
#include <cstring>   // For C string functions if not fully relying on std:: version from header
#include <cctype>    // For std::isspace
#include <cstdio>    // For std::sscanf, std::snprintf
#include <cstdlib>   // For std::strtol, std::strtoul, std::strtof
#include <cerrno>    // For errno
#include <limits>    // For std::numeric_limits
#include <algorithm> // For std::min

namespace kernel {
namespace util {

// memcpy, memset, strlen, strcmp are inline in util.hpp if using std:: versions.
// If custom implementations are needed, they would go here.

bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) {
        if (dest && dest_size > 0) dest[0] = '\0'; // Ensure null termination on error if possible
        return false;
    }
    // strncpy might not null-terminate if src is too long.
    // size_t src_len = std::strlen(src); // Calculate src length first
    // if (src_len < dest_size) {
    //     std::memcpy(dest, src, src_len + 1); // Copy including null terminator
    //     return true;
    // } else {
    //     std::memcpy(dest, src, dest_size - 1);
    //     dest[dest_size - 1] = '\0'; // Ensure null termination
    //     return false; // Truncation occurred
    // }
    // More robust:
    std::strncpy(dest, src, dest_size); // strncpy copies at most dest_size chars
    if (dest_size > 0) {
        dest[dest_size - 1] = '\0'; // Ensure null termination
    }
    // Check if truncation occurred by seeing if original src (up to dest_size-1) had a null terminator
    return (std::strlen(src) < dest_size);
}

bool str_to_int32(std::string_view input, int32_t& out_val) noexcept {
    if (input.empty()) {
        return false;
    }
    // Need a null-terminated string for strtol
    char buffer[24]; // Sufficient for int32_t string representation + null
    size_t len_to_copy = std::min(input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0; // Reset errno before calling strtol
    long result = std::strtol(buffer, &endptr, 10); // Base 10

    if (endptr == buffer) { // No digits were found
        return false;
    }
    // Check if the entire string was consumed (ignoring trailing whitespace)
    while (*endptr != '\0' && std::isspace(static_cast<unsigned char>(*endptr))) {
        endptr++;
    }
    if (*endptr != '\0') { // Non-whitespace characters remain
        return false;
    }

    if (errno == ERANGE || result < std::numeric_limits<int32_t>::min() || result > std::numeric_limits<int32_t>::max()) {
        return false; // Out of range for int32_t
    }

    out_val = static_cast<int32_t>(result);
    return true;
}

bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept {
    if (input.empty()) return false;

    // Trim leading whitespace from input string_view before creating buffer
    size_t start_pos = input.find_first_not_of(" \t\r\n");
    if (start_pos == std::string_view::npos) return false; // String is all whitespace
    std::string_view relevant_input = input.substr(start_pos);
    
    if (relevant_input.empty() || relevant_input[0] == '-') { // strtoul handles leading '+', but not '-' for unsigned
        return false;
    }

    char buffer[24]; // Sufficient for uint32_t string representation + null
    size_t len_to_copy = std::min(relevant_input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, relevant_input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0; // Reset errno
    unsigned long result = std::strtoul(buffer, &endptr, 10); // Base 10

    if (endptr == buffer) { // No digits
        return false;
    }
    // Check for trailing characters
    while (*endptr != '\0' && std::isspace(static_cast<unsigned char>(*endptr))) {
        endptr++;
    }
    if (*endptr != '\0') { // Non-whitespace characters remain
        return false;
    }

    if (errno == ERANGE || result > std::numeric_limits<uint32_t>::max()) {
        return false; // Out of range
    }

    out_val = static_cast<uint32_t>(result);
    return true;
}

bool str_to_float(std::string_view input, float& out_val) noexcept {
    if (input.empty()) {
        return false;
    }
    char buffer[64]; // Sufficient for float string representation + null
    size_t len_to_copy = std::min(input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0; // Reset errno
    out_val = std::strtof(buffer, &endptr);

    if (endptr == buffer) { // No conversion performed
        return false;
    }
    // Check for trailing characters
    while (*endptr != '\0' && std::isspace(static_cast<unsigned char>(*endptr))) {
        endptr++;
    }
    if (*endptr != '\0') { // Non-whitespace characters remain
        return false;
    }
    
    if (errno == ERANGE) { // Value out of range for float
        return false;
    }
    // Note: strtof can return 0.0 on error and set errno, or if "0.0" is input.
    // The endptr check is crucial.
    return true;
}

bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept {
    unsigned int b1 = 0, b2 = 0, b3 = 0, b4 = 0; // sscanf expects unsigned int*

    // sscanf needs a null-terminated string.
    if (ip_str.length() > 15 || ip_str.length() < 7) return false; // Basic length check "x.x.x.x" to "xxx.xxx.xxx.xxx"
    
    char buffer[17]; // Max "xxx.xxx.xxx.xxx" (15) + null
    std::memcpy(buffer, ip_str.data(), ip_str.length());
    buffer[ip_str.length()] = '\0';

    if (std::sscanf(buffer, "%u.%u.%u.%u", &b1, &b2, &b3, &b4) == 4) {
        // Check if sscanf consumed the whole string (not just a prefix)
        // This can be tricky with sscanf. A more robust parser might be better.
        // For now, assume valid if 4 numbers are parsed and they are in range.
        if (b1 <= 255 && b2 <= 255 && b3 <= 255 && b4 <= 255) {
            ip_addr = (static_cast<uint32_t>(b1) << 24) | 
                      (static_cast<uint32_t>(b2) << 16) | 
                      (static_cast<uint32_t>(b3) << 8)  | 
                       static_cast<uint32_t>(b4);
            return true;
        }
    }
    return false;
}

void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept {
    if (out_buffer.size() < 16) { // Minimum size for "xxx.xxx.xxx.xxx\0"
        if (!out_buffer.empty()) out_buffer[0] = '\0'; // Null-terminate if possible
        return;
    }
    // snprintf is safer and handles null termination.
    std::snprintf(out_buffer.data(), out_buffer.size(), "%u.%u.%u.%u",
                  (ip_addr >> 24) & 0xFF,
                  (ip_addr >> 16) & 0xFF,
                  (ip_addr >> 8) & 0xFF,
                  ip_addr & 0xFF);
}

std::string_view get_next_token(std::string_view& input, char delimiter) noexcept {
    // Trim leading whitespace from the main input string_view itself
    size_t start = input.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) { // Input is empty or all whitespace
        input.remove_prefix(input.size()); // Consume all of input
        return {}; // Return empty token
    }
    input.remove_prefix(start);

    size_t pos = input.find(delimiter);
    std::string_view token;
    if (pos == std::string_view::npos) { // Delimiter not found, token is the rest of the string
        token = input;
        input.remove_prefix(input.size()); // Consume all of input
    } else {
        token = input.substr(0, pos);
        input.remove_prefix(pos + 1); // Consume token and delimiter
    }

    // Trim trailing whitespace from the extracted token
    size_t token_end = token.find_last_not_of(" \t\r\n");
    if (token_end == std::string_view::npos) { // Token was all whitespace (shouldn't happen if leading trim worked)
        return {}; // Return empty token
    }
    return token.substr(0, token_end + 1);
}

} // namespace util
} // namespace kernel