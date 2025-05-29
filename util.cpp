// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.cpp
 * @brief Freestanding utility functions implementation for miniOS v1.7.
 */

#include "util.hpp" 
// <cstdarg> is included via util.hpp for va_list

namespace kernel {
namespace util {

// safe_strcpy, kstrcat, character functions, str_to_*, num_to_str helpers, IP functions
// ... (These function implementations remain exactly as provided in the previous response) ...
// For brevity, I'm not re-pasting all of them, but they must be present.
// I will re-paste a few essential ones that k_vsnprintf will use.

bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) {
        if (dest && dest_size > 0) dest[0] = '\0'; 
        return false;
    }
    size_t src_len = ::strlen(src); 
    if (src_len < dest_size) {
        ::memcpy(dest, src, src_len + 1); 
        return true;
    } else {
        ::memcpy(dest, src, dest_size - 1);
        dest[dest_size - 1] = '\0'; 
        return false; 
    }
}

char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept {
    if (!dest || !src || dest_max_len == 0) return dest;
    size_t dest_len = ::strlen(dest);
    if (dest_len >= dest_max_len -1) return dest; 

    size_t remaining_space = dest_max_len - dest_len - 1; // -1 for null terminator
    char* p = dest + dest_len;
    const char* s = src;
    while (*s && remaining_space > 0) {
        *p++ = *s++;
        remaining_space--;
    }
    *p = '\0';
    return dest;
}

bool isalpha(char c) noexcept {
    return ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'));
}
bool isalnum(char c) noexcept {
    return isalpha(c) || isdigit(c);
}
int toupper(int c_int) noexcept {
    char c = static_cast<char>(c_int);
    if (c >= 'a' && c <= 'z') return c - 'a' + 'A';
    return c;
}
int tolower(int c_int) noexcept {
    char c = static_cast<char>(c_int);
    if (c >= 'A' && c <= 'Z') return c - 'A' + 'a';
    return c;
}
bool str_to_int32(std::string_view input, int32_t& out_val) noexcept { /* ... as before ... */ return false;}
bool str_to_uint32(std::string_view input, uint32_t& out_val) noexcept { /* ... as before ... */ return false;}
bool str_to_float(std::string_view input, float& out_val) noexcept { (void)input; out_val = 0.0f; return false; }
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept { /* ... as before ... */ return false;}


static char* reverse_str(char* str, int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    return str;
}

static int num_to_str_base_internal(uint64_t value, char* buffer, size_t buffer_size, int base, bool is_signed_val, bool& was_negative_val) {
    if (buffer_size == 0) return -1; 
    if (base < 2 || base > 36) { 
        if (buffer_size > 0) buffer[0] = '\0'; 
        return -1; 
    }

    char* ptr = buffer;
    size_t count = 0; // Chars written to ptr, excluding null terminator
    was_negative_val = false;
    
    if (is_signed_val && base == 10 && static_cast<int64_t>(value) < 0) {
        if (count + 1 >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '-';
        count++;
        if (value == static_cast<uint64_t>(std::numeric_limits<int64_t>::min())) {
            // Special case: print "9223372036854775808" for INT64_MIN's magnitude
            // This string is 19 chars + null = 20.
            const char* min_int_mag = "9223372036854775808";
            size_t min_len = kernel::util::kstrlen(min_int_mag); // Use our kstrlen
            if (count + min_len >= buffer_size) { buffer[count > 0 ? count-1 : 0] = '\0'; return -1; }
            kernel::util::kmemcpy(ptr, min_int_mag, min_len + 1); // Use our kmemcpy
            return static_cast<int>(count + min_len);
        }
        value = static_cast<uint64_t>(-static_cast<int64_t>(value)); // Make positive
        was_negative_val = true;
    }
    
    if (value == 0) {
        if (count + 1 >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '0';
        count++;
        *ptr = '\0';
        return static_cast<int>(count);
    }

    char* start_digits = ptr; 
    while (value > 0) {
        if (count + 1 >= buffer_size) { 
            buffer[kernel::util::min(count, buffer_size-1)] = '\0'; 
            return -1; 
        }
        // Ensure base is treated as unsigned for modulo with uint64_t
        int remainder = value % static_cast<uint32_t>(base); 
        *ptr++ = (remainder > 9) ? static_cast<char>((remainder - 10) + 'a') : static_cast<char>(remainder + '0');
        value /= static_cast<uint32_t>(base);
        count++;
    }
    *ptr = '\0'; 
    reverse_str(start_digits, static_cast<int>(ptr - start_digits)); // Pass length of digits
    return static_cast<int>(count);
}

int int_to_str(int32_t value, char* buffer, size_t buffer_size, int base) noexcept {
    bool was_negative;
    return num_to_str_base_internal(static_cast<uint64_t>(static_cast<int64_t>(value)), buffer, buffer_size, base, true, was_negative);
}
int uint_to_str(uint32_t value, char* buffer, size_t buffer_size, int base) noexcept {
    bool was_negative;
    return num_to_str_base_internal(value, buffer, buffer_size, base, false, was_negative);
}
int uint64_to_str(uint64_t value, char* buffer, size_t buffer_size, int base) noexcept {
    bool was_negative;
    return num_to_str_base_internal(value, buffer, buffer_size, base, false, was_negative);
}
int uint64_to_hex_str(uint64_t value, char* buffer, size_t buffer_size, bool leading_0x) noexcept {
    if (buffer_size == 0) return -1;
    char* start_ptr = buffer;
    size_t remaining_size = buffer_size;

    if (leading_0x) {
        if (remaining_size < 3) { buffer[0] = '\0'; return -1; } 
        *start_ptr++ = '0';
        *start_ptr++ = 'x';
        remaining_size -= 2;
    }
    
    int digits_len = num_to_str_base_internal(value, start_ptr, remaining_size, 16, false, *(new bool(false))); // Dummy was_negative
    if (digits_len < 0) { 
        if (buffer_size > 0) buffer[0] = '\0';
        return -1;
    }
    return static_cast<int>((start_ptr - buffer) + digits_len);
}


void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept {
    // ... (implementation using uint_to_str and kstrcat, as in previous freestanding util.cpp) ...
    if (out_buffer.empty()) return;
    char* p = out_buffer.data();
    size_t remaining = out_buffer.size();
    int written_this_segment;

    for (int i = 0; i < 4; ++i) {
        if (remaining == 0) { if(p != out_buffer.data()) *(p-1) = '\0'; else out_buffer[0] = '\0'; return; }
        uint8_t octet = (ip_addr >> (24 - i * 8)) & 0xFF;
        written_this_segment = uint_to_str(octet, p, remaining, 10);

        if (written_this_segment < 0 || static_cast<size_t>(written_this_segment) >= remaining) {
            if(!out_buffer.empty()) out_buffer[0] = '\0';
            return;
        }
        p += written_this_segment;
        remaining -= static_cast<size_t>(written_this_segment);

        if (i < 3) {
            if (remaining <= 1) { *p = '\0'; return; } 
            *p++ = '.';
            remaining--;
        }
    }
    if (remaining > 0) *p = '\0';
    else if (!out_buffer.empty()) out_buffer.back() = '\0';
}

std::string_view get_next_token(std::string_view& input, char delimiter) noexcept { /* ... as before ... */ return {};}


// Implementation of k_vsnprintf and k_snprintf
int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept {
    if (!buffer || bufsz == 0 || !format) return 0;

    char* buf_ptr = buffer;
    size_t remaining_len = bufsz -1; // Leave space for null terminator
    int written_chars = 0;
    char temp_num_buf[24]; // For converting numbers (max 20 digits for uint64_t + sign + null)

    while (*format && remaining_len > 0) {
        if (*format == '%') {
            format++; // Move past '%'
            bool is_long_long = false;
            // Rudimentary length specifier check for %ll
            if (format[0] == 'l' && format[1] == 'l') {
                is_long_long = true;
                format += 2;
            }

            switch (*format) {
                case 's': { // String
                    const char* s_arg = va_arg(args, const char*);
                    if (!s_arg) s_arg = "(null)";
                    size_t arg_len = kstrlen(s_arg);
                    size_t copy_len = min(arg_len, remaining_len);
                    kmemcpy(buf_ptr, s_arg, copy_len);
                    buf_ptr += copy_len;
                    written_chars += static_cast<int>(copy_len);
                    remaining_len -= copy_len;
                    break;
                }
                case 'c': { // Character
                    char c_arg = static_cast<char>(va_arg(args, int)); // char promotes to int
                    *buf_ptr++ = c_arg;
                    written_chars++;
                    remaining_len--;
                    break;
                }
                case 'd': case 'i': { // Signed decimal integer
                    int len;
                    if (is_long_long) {
                        long long ll_val = va_arg(args, long long);
                        len = uint64_to_str(static_cast<uint64_t>(ll_val), temp_num_buf, sizeof(temp_num_buf), 10);
                    } else {
                        int i_val = va_arg(args, int);
                        len = int_to_str(i_val, temp_num_buf, sizeof(temp_num_buf));
                    }
                    if (len > 0 && static_cast<size_t>(len) <= remaining_len) {
                        kmemcpy(buf_ptr, temp_num_buf, static_cast<size_t>(len));
                        buf_ptr += len;
                        written_chars += len;
                        remaining_len -= static_cast<size_t>(len);
                    } else if (len > 0) { // Not enough space
                        kmemcpy(buf_ptr, temp_num_buf, remaining_len);
                        buf_ptr += remaining_len;
                        written_chars += static_cast<int>(remaining_len);
                        remaining_len = 0;
                    }
                    break;
                }
                case 'u': { // Unsigned decimal integer
                    int len;
                    if (is_long_long) {
                        unsigned long long ull_val = va_arg(args, unsigned long long);
                        len = uint64_to_str(ull_val, temp_num_buf, sizeof(temp_num_buf));
                    } else {
                        unsigned int ui_val = va_arg(args, unsigned int);
                        len = uint_to_str(ui_val, temp_num_buf, sizeof(temp_num_buf));
                    }
                     if (len > 0 && static_cast<size_t>(len) <= remaining_len) {
                        kmemcpy(buf_ptr, temp_num_buf, static_cast<size_t>(len));
                        buf_ptr += len;
                        written_chars += len;
                        remaining_len -= static_cast<size_t>(len);
                    } else if (len > 0) {
                        kmemcpy(buf_ptr, temp_num_buf, remaining_len);
                        buf_ptr += remaining_len;
                        written_chars += static_cast<int>(remaining_len);
                        remaining_len = 0;
                    }
                    break;
                }
                case 'x': case 'X': case 'p': { // Hexadecimal / Pointer
                    // For %p, treat as uint64_t. For %x, %X check long long.
                    // Assume %p takes void* which promotes to uint64_t on AArch64 for va_arg.
                    uint64_t hex_val;
                    if (*format == 'p') {
                        hex_val = reinterpret_cast<uint64_t>(va_arg(args, void*));
                    } else if (is_long_long) {
                        hex_val = va_arg(args, unsigned long long);
                    } else {
                        hex_val = va_arg(args, unsigned int);
                    }
                    // For %X, convert to uppercase later if needed. This uint64_to_hex_str produces lowercase.
                    int len = uint64_to_hex_str(hex_val, temp_num_buf, sizeof(temp_num_buf), (*format == 'p'));
                     if (len > 0 && static_cast<size_t>(len) <= remaining_len) {
                        kmemcpy(buf_ptr, temp_num_buf, static_cast<size_t>(len));
                        buf_ptr += len;
                        written_chars += len;
                        remaining_len -= static_cast<size_t>(len);
                    } else if (len > 0) {
                        kmemcpy(buf_ptr, temp_num_buf, remaining_len);
                        buf_ptr += remaining_len;
                        written_chars += static_cast<int>(remaining_len);
                        remaining_len = 0;
                    }
                    break;
                }
                case '%': { // Literal '%'
                    *buf_ptr++ = '%';
                    written_chars++;
                    remaining_len--;
                    break;
                }
                default: { // Unknown specifier, just copy it
                    *buf_ptr++ = '%'; 
                    written_chars++; 
                    remaining_len--;
                    if (remaining_len > 0 && *format) {
                         *buf_ptr++ = *format; 
                         written_chars++; 
                         remaining_len--;
                    }
                    break;
                }
            }
        } else { // Literal character
            *buf_ptr++ = *format;
            written_chars++;
            remaining_len--;
        }
        format++;
    }

    *buf_ptr = '\0'; // Null terminate
    return written_chars;
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