// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.cpp
 * @brief Freestanding utility functions implementation for miniOS v1.7.
 */

#include "util.hpp" 
#include <cstdarg> // For va_list in k_vsnprintf

namespace kernel {
namespace util {

// safe_strcpy now uses kstrlen and kmemcpy (which call the global extern "C" versions)
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) {
        if (dest && dest_size > 0) dest[0] = '\0'; 
        return false;
    }
    size_t src_len = kstrlen(src); // Uses kernel::util::kstrlen -> ::strlen
    if (src_len < dest_size) {
        kmemcpy(dest, src, src_len + 1); // Uses kernel::util::kmemcpy -> ::memcpy
        return true;
    } else {
        kmemcpy(dest, src, dest_size - 1);
        dest[dest_size - 1] = '\0'; 
        return false; 
    }
}

char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept {
    if (!dest || !src || dest_max_len == 0) return dest;
    size_t dest_len = kstrlen(dest); 
    if (dest_len >= dest_max_len -1) return dest; 

    size_t remaining_space = dest_max_len - dest_len - 1; 
    char* p = dest + dest_len;
    const char* s = src;
    while (*s && remaining_space > 0) {
        *p++ = *s++;
        remaining_space--;
    }
    *p = '\0';
    return dest;
}

// Character functions moved from inline in header to .cpp
bool isalpha(char c) noexcept {
    return ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'));
}

bool isalnum(char c) noexcept {
    return isalpha(c) || isdigit(c); // Uses kernel::util::isalpha and inline kernel::util::isdigit
}

int toupper(int c_int) noexcept {
    char c = static_cast<char>(c_int);
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 'A';
    }
    return c;
}

int tolower(int c_int) noexcept {
    char c = static_cast<char>(c_int);
    if (c >= 'A' && c <= 'Z') {
        return c - 'A' + 'a';
    }
    return c;
}

bool str_to_int32(std::string_view input, int32_t& out_val) noexcept {
    if (input.empty()) return false;
    long long acc = 0; 
    size_t i = 0;
    bool negative = false;
    const long long INT32_MIN_ABS_VAL = 2147483648LL; 

    while (i < input.length() && isspace(input[i])) { i++; } 
    if (i < input.length()) {
        if (input[i] == '-') { negative = true; i++; }
        else if (input[i] == '+') { i++; }
    }
    bool found_digits = false;
    while (i < input.length() && isdigit(input[i])) { 
        found_digits = true;
        int digit = input[i] - '0';
        if (negative) {
             if (acc > (INT32_MIN_ABS_VAL - digit) / 10) return false; 
             if (acc == (INT32_MIN_ABS_VAL - digit) / 10 && static_cast<long long>(digit) > (INT32_MIN_ABS_VAL % 10) ) return false;
        } else { 
             if (acc > (static_cast<long long>(std::numeric_limits<int32_t>::max()) - digit) / 10) return false; 
        }
        acc = acc * 10 + digit;
        i++;
    }
    if (!found_digits) return false;
    while (i < input.length() && isspace(input[i])) { i++; }
    if (i != input.length()) return false; 

    if (negative) acc = -acc;
    if (acc < std::numeric_limits<int32_t>::min() || acc > std::numeric_limits<int32_t>::max()) return false;
    out_val = static_cast<int32_t>(acc);
    return true;
}

bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept {
    if (input.empty()) return false;
    unsigned long long acc = 0; 
    size_t i = 0;
    while (i < input.length() && isspace(input[i])) { i++; }
    if (i < input.length() && input[i] == '+') { i++; }
    bool found_digits = false;
    while (i < input.length() && isdigit(input[i])) {
        found_digits = true;
        int digit = input[i] - '0';
        if (acc > (std::numeric_limits<uint32_t>::max() - static_cast<unsigned long long>(digit)) / 10ULL) return false; 
        acc = acc * 10ULL + static_cast<unsigned long long>(digit);
        i++;
    }
    if (!found_digits) return false;
    while (i < input.length() && isspace(input[i])) { i++; }
    if (i != input.length()) return false;
    if (acc > std::numeric_limits<uint32_t>::max()) return false;
    out_val = static_cast<uint32_t>(acc);
    return true;
}

bool str_to_float(std::string_view input, float& out_val) noexcept {
    (void)input; 
    out_val = 0.0f; 
    return false; 
}

static char* reverse_str(char* str, int length) {
    int start = 0; int end = length - 1;
    while (start < end) { char temp = str[start]; str[start] = str[end]; str[end] = temp; start++; end--; }
    return str;
}

static int num_to_str_base_internal(uint64_t value, char* buffer, size_t buffer_size, int base, bool handle_sign_for_base10) {
    if (buffer_size == 0) return -1; 
    if (base < 2 || base > 36) { 
        if (buffer_size > 0) buffer[0] = '\0'; 
        return -1; 
    }
    char* ptr = buffer;
    int chars_written = 0; 
    if (handle_sign_for_base10 && base == 10 && static_cast<int64_t>(value) < 0) {
        if (static_cast<size_t>(chars_written + 1) >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '-';
        chars_written++;
        if (value == static_cast<uint64_t>(std::numeric_limits<int64_t>::min())) {
            const char* min_int_mag = "9223372036854775808"; size_t min_len = kstrlen(min_int_mag); 
            if (static_cast<size_t>(chars_written) + min_len >= buffer_size) { 
                if(chars_written > 0) buffer[chars_written-1] = '\0'; else buffer[0] = '\0';
                return -1; 
            }
            kmemcpy(ptr, min_int_mag, min_len + 1); 
            return static_cast<int>(static_cast<size_t>(chars_written) + min_len);
        }
        value = static_cast<uint64_t>(-static_cast<int64_t>(value));
        // is_negative was unused, removed
    }
    if (value == 0) {
        if (static_cast<size_t>(chars_written + 1) >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '0';
        chars_written++;
        *ptr = '\0';
        return chars_written;
    }
    char* start_digits = ptr; 
    int num_digits = 0;
    while (value > 0) {
        if (static_cast<size_t>(chars_written + num_digits + 1) >= buffer_size) { 
            buffer[min(static_cast<size_t>(chars_written + num_digits), buffer_size-1)] = '\0'; 
            return -1; 
        }
        int remainder = value % static_cast<unsigned int>(base); 
        *ptr++ = (remainder > 9) ? static_cast<char>((remainder - 10) + 'a') : static_cast<char>(remainder + '0');
        value /= static_cast<unsigned int>(base);
        num_digits++;
    }
    *ptr = '\0'; 
    reverse_str(start_digits, num_digits); 
    return chars_written + num_digits;
}

int int_to_str(int32_t value, char* buffer, size_t buffer_size, int base) noexcept {
    return num_to_str_base_internal(static_cast<uint64_t>(static_cast<int64_t>(value)), buffer, buffer_size, base, true);
}
int uint_to_str(uint32_t value, char* buffer, size_t buffer_size, int base) noexcept {
    return num_to_str_base_internal(value, buffer, buffer_size, base, false);
}
int uint64_to_str(uint64_t value, char* buffer, size_t buffer_size, int base) noexcept {
    return num_to_str_base_internal(value, buffer, buffer_size, base, false);
}
int uint64_to_hex_str(uint64_t value, char* buffer, size_t buffer_size, bool leading_0x) noexcept {
    if (buffer_size == 0) return -1;
    char* ptr = buffer;
    size_t current_written = 0;
    if (leading_0x) {
        if (buffer_size < 3) { buffer[0] = '\0'; return -1; } 
        *ptr++ = '0'; *ptr++ = 'x'; current_written += 2;
    }
    int digits_len = num_to_str_base_internal(value, ptr, buffer_size - current_written, 16, false);
    if (digits_len < 0) { if (buffer_size > 0) buffer[0] = '\0'; return -1; }
    return static_cast<int>(current_written + static_cast<size_t>(digits_len));
}

void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept {
    if (out_buffer.empty()) return;
    char* p = out_buffer.data(); size_t remaining = out_buffer.size(); int written_this_segment;
    for (int i = 0; i < 4; ++i) {
        if (remaining == 0) { 
            if(p != out_buffer.data()) { *(p-1) = '\0'; } 
            else if (!out_buffer.empty()) { out_buffer[0] = '\0'; }
            return; 
        }
        uint8_t octet = (ip_addr >> (24 - i * 8)) & 0xFF;
        written_this_segment = uint_to_str(octet, p, remaining, 10);
        if (written_this_segment < 0 || static_cast<size_t>(written_this_segment) >= remaining) {
            if(!out_buffer.empty()) { out_buffer[0] = '\0'; }
            return;
        }
        p += written_this_segment; remaining -= static_cast<size_t>(written_this_segment);
        if (i < 3) {
            if (remaining <= 1) { *p = '\0'; return; } 
            *p++ = '.'; remaining--;
        }
    }
    if (remaining > 0) { *p = '\0'; }
    else if (!out_buffer.empty()) { out_buffer.back() = '\0'; }
}

bool ipv4_to_uint32(std::string_view ip_str_in, uint32_t& ip_addr) noexcept {
    uint32_t parts[4] = {0}; int current_part_idx = 0;
    uint32_t current_octet_val = 0; bool digits_in_octet = false; bool expect_dot = false; 
    std::string_view ip_str = ip_str_in; 
    size_t first_char = ip_str.find_first_not_of(" \t\r\n");
    if (first_char == std::string_view::npos) return false;
    ip_str.remove_prefix(first_char);
    size_t last_char = ip_str.find_last_not_of(" \t\r\n");
    if (last_char == std::string_view::npos) return false;
    ip_str = ip_str.substr(0, last_char + 1);
    for (char c : ip_str) {
        if (isdigit(c)) {
            if (expect_dot) return false; 
            current_octet_val = current_octet_val * 10 + (c - '0');
            if (current_octet_val > 255) return false; 
            digits_in_octet = true;
        } else if (c == '.') {
            if (!digits_in_octet || current_part_idx >= 3) return false; 
            parts[current_part_idx++] = current_octet_val;
            current_octet_val = 0; digits_in_octet = false; expect_dot = false; 
        } else if (isspace(c)) { 
            if (digits_in_octet && current_part_idx < 3) expect_dot = true; 
            continue; 
        } else { return false; }
    }
    if (!digits_in_octet || current_part_idx != 3) return false; 
    parts[current_part_idx] = current_octet_val;
    ip_addr = (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
    return true;
}

std::string_view get_next_token(std::string_view& input_ref, char delimiter) noexcept {
    std::string_view current_input = input_ref; 
    size_t start = 0;
    while(start < current_input.length() && isspace(current_input[start])) { start++; } 
    current_input.remove_prefix(start);
    if (current_input.empty()) { input_ref.remove_prefix(input_ref.length()); return {}; }
    size_t pos = 0;
    while(pos < current_input.length() && current_input[pos] != delimiter) { pos++; }
    std::string_view token = current_input.substr(0, pos);
    if (pos < current_input.length()) { current_input.remove_prefix(pos + 1); } 
    else { current_input.remove_prefix(pos); }
    input_ref = current_input; 
    size_t token_end = token.length();
    while(token_end > 0 && isspace(token[token_end - 1])) { token_end--; } 
    return token.substr(0, token_end);
}

int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept {
    if (!buffer || bufsz == 0 || !format) { if (bufsz > 0) buffer[0] = '\0'; return 0; }
    char* buf_ptr = buffer;
    char* const buf_write_end = buffer + bufsz -1; 
    int total_written_chars = 0; 
    char temp_num_buf[24]; 

    while (*format && buf_ptr < buf_write_end) { 
        if (*format == '%') {
            format++; 
            bool is_long_long = false;
            if (format[0] == 'l' && format[1] == 'l') {
                is_long_long = true; format += 2;
            }
            int current_segment_len = 0;
            const char* str_to_copy_from = temp_num_buf; 

            switch (*format) {
                case 's': { 
                    const char* s_arg = va_arg(args, const char*);
                    if (!s_arg) s_arg = "(null)";
                    str_to_copy_from = s_arg;
                    current_segment_len = static_cast<int>(kstrlen(s_arg)); 
                    break;
                }
                case 'c': { 
                    temp_num_buf[0] = static_cast<char>(va_arg(args, int)); 
                    temp_num_buf[1] = '\0'; current_segment_len = 1;
                    break;
                }
                case 'd': case 'i': { 
                    if (is_long_long) current_segment_len = int_to_str(static_cast<int32_t>(va_arg(args, long long)), temp_num_buf, sizeof(temp_num_buf));
                    else current_segment_len = int_to_str(va_arg(args, int), temp_num_buf, sizeof(temp_num_buf));
                    break;
                }
                case 'u': { 
                    if (is_long_long) current_segment_len = uint64_to_str(va_arg(args, unsigned long long), temp_num_buf, sizeof(temp_num_buf));
                    else current_segment_len = uint_to_str(va_arg(args, unsigned int), temp_num_buf, sizeof(temp_num_buf));
                    break;
                }
                case 'x': case 'X': case 'p': { 
                    uint64_t hex_val;
                    if (*format == 'p') hex_val = reinterpret_cast<uint64_t>(va_arg(args, void*));
                    else if (is_long_long) hex_val = va_arg(args, unsigned long long);
                    else hex_val = va_arg(args, unsigned int);
                    current_segment_len = uint64_to_hex_str(hex_val, temp_num_buf, sizeof(temp_num_buf), (*format == 'p'));
                    break;
                }
                case '%': { 
                    temp_num_buf[0] = '%'; temp_num_buf[1] = '\0'; current_segment_len = 1;
                    break;
                }
                default: { 
                    if (buf_ptr < buf_write_end) { *buf_ptr++ = '%'; total_written_chars++; }
                    // Ensure not to read past end of format string if it ends with %
                    if (*format && buf_ptr < buf_write_end) { *buf_ptr++ = *format; total_written_chars++; } 
                    str_to_copy_from = nullptr; 
                    break;
                }
            }

            if (str_to_copy_from && current_segment_len > 0) {
                int copy_actual_len = 0;
                for(int k=0; k < current_segment_len && buf_ptr < buf_write_end; ++k) {
                    *buf_ptr++ = str_to_copy_from[k];
                    copy_actual_len++;
                }
                total_written_chars += copy_actual_len;
            } else if (str_to_copy_from && current_segment_len < 0) { /* conversion error */ }
             else if (!str_to_copy_from && (*format == '\0' || *(format+1) == '\0') ) { /* Format string ended after % or %? */ break; }


        } else { 
            *buf_ptr++ = *format;
            total_written_chars++;
        }
        if (*format == '\0') break; 
        format++;
    }
    *buf_ptr = '\0'; 
    return total_written_chars;
}

int k_snprintf(char* buffer, size_t bufsz, const char* format, ...) noexcept {
    va_list args;
    va_start(args, format);
    int result = k_vsnprintf(buffer, bufsz, format, args);
    va_end(args);
    return result;
}

} // namespace util
} // namespace kernel