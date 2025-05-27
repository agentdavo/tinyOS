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
#include <cstring> // For std::memcpy, std::memset, std::strlen, std::strcmp, std::strncpy
#include <cctype>  // For std::isspace, std::isdigit for get_next_token
#include <cstdio>  // For std::sscanf (used in ipv4_to_uint32), std::snprintf
#include <cstdlib> // For std::strtol, std::strtoul, std::strtof
#include <cerrno>  // For errno and ERANGE
#include <limits>  // For std::numeric_limits
#include <algorithm> // For std::min

namespace kernel { // <<<<< WRAP IN NAMESPACE KERNEL
namespace util {

void* memcpy(void* dest, const void* src, size_t count) noexcept {
    return std::memcpy(dest, src, count);
}

void* memset(void* dest, int ch, size_t count) noexcept {
    return std::memset(dest, ch, count);
}

size_t strlen(const char* str) noexcept {
    if (!str) return 0;
    return std::strlen(str);
}

int strcmp(const char* lhs, const char* rhs) noexcept {
    if (!lhs && !rhs) return 0;
    if (!lhs) return -1; 
    if (!rhs) return 1;
    return std::strcmp(lhs, rhs);
}

bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) {
        return false;
    }
    std::strncpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = '\0'; 
    return std::strlen(src) < dest_size; 
}

bool str_to_int32(std::string_view input, int32_t& out_val) noexcept {
    if (input.empty()) {
        return false;
    }
    char buffer[24]; 
    size_t len_to_copy = std::min(input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0; 
    long result = std::strtol(buffer, &endptr, 10);

    if (endptr == buffer) {
        return false; 
    }
    if (*endptr != '\0') {
        // Allow trailing whitespace
        while (std::isspace(static_cast<unsigned char>(*endptr))) {
            endptr++;
        }
        if (*endptr != '\0') return false; // Still has non-whitespace chars
    }
    if (errno == ERANGE || result < std::numeric_limits<int32_t>::min() || result > std::numeric_limits<int32_t>::max()) {
        return false; 
    }

    out_val = static_cast<int32_t>(result);
    return true;
}

bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept {
    if (input.empty()) return false;
    
    // Skip leading whitespace for strtoul
    size_t start_pos = input.find_first_not_of(" \t\r\n");
    if (start_pos == std::string_view::npos) return false; // All whitespace
    std::string_view relevant_input = input.substr(start_pos);

    if (relevant_input.empty() || relevant_input[0] == '-') { 
        return false;
    }
    char buffer[24];
    size_t len_to_copy = std::min(relevant_input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, relevant_input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0;
    unsigned long result = std::strtoul(buffer, &endptr, 10);

    if (endptr == buffer) return false;
    if (*endptr != '\0') {
        while (std::isspace(static_cast<unsigned char>(*endptr))) {
            endptr++;
        }
        if (*endptr != '\0') return false;
    }
    if (errno == ERANGE || result > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    out_val = static_cast<uint32_t>(result);
    return true;
}

bool str_to_float(std::string_view input, float& out_val) noexcept {
    if (input.empty()) {
        return false;
    }
    char buffer[64]; 
    size_t len_to_copy = std::min(input.length(), sizeof(buffer) - 1);
    std::memcpy(buffer, input.data(), len_to_copy);
    buffer[len_to_copy] = '\0';

    char* endptr;
    errno = 0;
    out_val = std::strtof(buffer, &endptr);

    if (endptr == buffer) return false; 
    if (*endptr != '\0') {
         while (std::isspace(static_cast<unsigned char>(*endptr))) {
            endptr++;
        }
        if (*endptr != '\0') return false;
    }
    if (errno == ERANGE) {
        return false; 
    }
    return true;
}


bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept {
    unsigned int b1 = 0, b2 = 0, b3 = 0, b4 = 0;
    
    char buffer[17]; 
    if (ip_str.length() > 15 || ip_str.length() < 7) return false; 
    std::memcpy(buffer, ip_str.data(), ip_str.length());
    buffer[ip_str.length()] = '\0';

    if (std::sscanf(buffer, "%u.%u.%u.%u", &b1, &b2, &b3, &b4) == 4) {
        if (b1 <= 255 && b2 <= 255 && b3 <= 255 && b4 <= 255) {
            ip_addr = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4;
            return true;
        }
    }
    return false;
}

void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept {
    if (out_buffer.size() < 16) { 
        if (!out_buffer.empty()) out_buffer[0] = '\0';
        return;
    }
    // Use snprintf for safety.
    std::snprintf(out_buffer.data(), out_buffer.size(), "%u.%u.%u.%u",
                  (ip_addr >> 24) & 0xFF,
                  (ip_addr >> 16) & 0xFF,
                  (ip_addr >> 8) & 0xFF,
                  ip_addr & 0xFF);
}

std::string_view get_next_token(std::string_view& input, char delimiter) noexcept {
    size_t start = input.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) {
        input.remove_prefix(input.size()); // Consume all whitespace
        return {};
    }
    input.remove_prefix(start);

    size_t pos = input.find(delimiter);
    std::string_view token;
    if (pos == std::string_view::npos) {
        token = input;
        input.remove_prefix(input.size()); 
    } else {
        token = input.substr(0, pos);
        input.remove_prefix(pos + 1); 
    }
    
    size_t token_end = token.find_last_not_of(" \t\r\n");
    if (token_end == std::string_view::npos) return {}; // Token was all whitespace after initial trim
    
    return token.substr(0, token_end + 1);
}

} // namespace util
} // namespace kernel