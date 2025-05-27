// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.cpp
 * @brief Utility function implementations for miniOS v1.7.
 * @details
 * Implements string manipulation, numeric conversion, and memory operations for miniOS subsystems.
 * Optimized for embedded systems with thread-safe, exception-free code. Updated in v1.7 with
 * enhanced error handling, clearer diagnostics, and modern C++20 practices, retaining all v1.6
 * functionality.
 *
 * C++20 features:
 * - std::string_view for efficient string processing
 * - std::span for safe buffer handling
 * - noexcept for exception-free guarantees
 *
 * @version 1.7
 * @see util.hpp, miniOS.hpp, cli.hpp, dsp.hpp, net.hpp, fs.hpp
 */

#include "util.hpp"
#include <cctype>
#include <cstdlib>
#include <cstring>

namespace util {

size_t split_string(std::string_view input, char delim, std::vector<std::string_view>& tokens) noexcept {
    tokens.clear();
    if (input.empty()) return 0;
    size_t start = 0;
    size_t count = 0;
    for (size_t i = 0; i <= input.size(); ++i) {
        if (i == input.size() || input[i] == delim) {
            if (i > start) {
                tokens.emplace_back(input.substr(start, i - start));
                count++;
            }
            start = i + 1;
        }
    }
    return count;
}

std::string_view trim(std::string_view input) noexcept {
    size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start]))) {
        ++start;
    }
    size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(start, end - start);
}

bool str_to_int32(std::string_view input, int32_t& value) noexcept {
    input = trim(input);
    if (input.empty()) return false;
    char* endptr;
    errno = 0;
    long result = std::strtol(input.data(), &endptr, 10);
    if (endptr == input.data() || endptr != input.data() + input.size() ||
        errno == ERANGE || result < INT32_MIN || result > INT32_MAX) {
        return false;
    }
    value = static_cast<int32_t>(result);
    return true;
}

bool str_to_uint32(std::string_view input, uint32_t& value) noexcept {
    input = trim(input);
    if (input.empty()) return false;
    char* endptr;
    errno = 0;
    unsigned long result = std::strtoul(input.data(), &endptr, 10);
    if (endptr == input.data() || endptr != input.data() + input.size() ||
        errno == ERANGE || result > UINT32_MAX) {
        return false;
    }
    value = static_cast<uint32_t>(result);
    return true;
}

bool str_to_float(std::string_view input, float& value) noexcept {
    input = trim(input);
    if (input.empty()) return false;
    char* endptr;
    errno = 0;
    float result = std::strtof(input.data(), &endptr);
    if (endptr == input.data() || endptr != input.data() + input.size() || errno == ERANGE) {
        return false;
    }
    value = result;
    return true;
}

bool ipv4_to_uint32(std::string_view input, uint32_t& addr) noexcept {
    std::vector<std::string_view> octets;
    if (split_string(input, '.', octets) != 4) return false;
    uint32_t result = 0;
    for (size_t i = 0; i < 4; ++i) {
        uint32_t octet;
        if (!str_to_uint32(octets[i], octet) || octet > 255) return false;
        result = (result << 8) | octet;
    }
    addr = result;
    return true;
}

bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) return false;
    size_t src_len = strlen(src);
    if (src_len >= dest_size) return false;
    std::memcpy(dest, src, src_len);
    dest[src_len] = '\0';
    return true;
}

size_t strlen(const char* str) noexcept {
    if (!str) return 0;
    size_t len = 0;
    while (str[len]) ++len;
    return len;
}

int strcmp(const char* str1, const char* str2) noexcept {
    if (!str1 || !str2) return str1 == str2 ? 0 : (str1 ? 1 : -1);
    while (*str1 && *str1 == *str2) {
        ++str1;
        ++str2;
    }
    return static_cast<unsigned char>(*str1) - static_cast<unsigned char>(*str2);
}

void memcpy(void* dest, const void* src, size_t size) noexcept {
    if (!dest || !src || size == 0) return;
    char* d = static_cast<char*>(dest);
    const char* s = static_cast<const char*>(src);
    for (size_t i = 0; i < size; ++i) {
        d[i] = s[i];
    }
}

void memset(void* dest, uint8_t value, size_t size) noexcept {
    if (!dest || size == 0) return;
    char* d = static_cast<char*>(dest);
    for (size_t i = 0; i < size; ++i) {
        d[i] = static_cast<char>(value);
    }
}

} // namespace util