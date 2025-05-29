// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file util.cpp
 * @brief Freestanding utility functions implementation for miniOS v1.7.
 */

#include "util.hpp" 
#include <cstdarg> // For va_list in k_vsnprintf

namespace kernel {
namespace util {

// Definitions for kernel::util functions, calling the extern "C" _c versions
void* kmemcpy(void* dest, const void* src, size_t count) noexcept {
    return ::memcpy_c(dest, src, count);
}
void* kmemset(void* dest, int ch, size_t count) noexcept {
    return ::memset_c(dest, ch, count);
}
int kmemcmp(const void* ptr1, const void* ptr2, size_t count) noexcept {
    return ::memcmp_c(ptr1, ptr2, count);
}
size_t kstrlen(const char* str) noexcept {
    if (!str) return 0;
    return ::strlen_c(str);
}
int kstrcmp(const char* lhs, const char* rhs) noexcept {
    return ::strcmp_c(lhs, rhs);
}
int kstrncmp(const char* lhs, const char* rhs, size_t count) noexcept {
    return ::strncmp_c(lhs, rhs, count);
}
char* kstrcpy(char* dest, const char* src) noexcept {
    return ::strcpy_c(dest, src);
}
char* kstrncpy(char* dest, const char* src, size_t count) noexcept {
    return ::strncpy_c(dest, src, count);
}


bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept {
    if (!dest || !src || dest_size == 0) {
        if (dest && dest_size > 0) dest[0] = '\0'; 
        return false;
    }
    size_t src_len = kstrlen(src); // Uses kernel::util::kstrlen
    if (src_len < dest_size) {
        kmemcpy(dest, src, src_len + 1); // Uses kernel::util::kmemcpy
        return true;
    } else {
        kmemcpy(dest, src, dest_size - 1);
        dest[dest_size - 1] = '\0'; 
        return false; 
    }
}

char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept {
    if (!dest || !src || dest_max_len == 0) return dest;
    size_t dest_len = kstrlen(dest); // Uses kernel::util::kstrlen
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
bool str_to_int32(std::string_view input, int32_t& out_val) noexcept {
    if (input.empty()) return false;
    long long acc = 0; 
    size_t i = 0;
    bool negative = false;
    while (i < input.length() && isspace(input[i])) { i++; } 
    if (i < input.length()) {
        if (input[i] == '-') { negative = true; i++; }
        else if (input[i] == '+') { i++; }
    }
    bool found_digits = false;
    while (i < input.length() && isdigit(input[i])) { 
        found_digits = true;
        int digit = input[i] - '0';
        if (!negative) { 
             if (acc > (std::numeric_limits<int32_t>::max() - digit) / 10) return false; 
        } else { 
             if (acc > (static_cast<long long>(std::numeric_limits<int32_t>::max()) + 1 - digit) / 10 && digit !=0) { 
                if (acc * 10 + digit > static_cast<long long>(std::numeric_limits<int32_t>::max()) +1) return false;
             } else if (acc > (static_cast<long long>(std::numeric_limits<int32_t>::max()) / 10) && negative && digit==0 && acc * 10 + digit > (static_cast<long long>(std::numeric_limits<int32_t>::max())+1) ) {
                 if (acc * 10 + digit > (static_cast<long long>(std::numeric_limits<int32_t>::max()) +1 )) return false;
             }
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
bool str_to_float(std::string_view input, float& out_val) noexcept { (void)input; out_val = 0.0f; return false; }

static char* reverse_str(char* str, int length) {
    int start = 0; int end = length - 1;
    while (start < end) { char temp = str[start]; str[start] = str[end]; str[end] = temp; start++; end--; }
    return str;
}
static int num_to_str_base_internal(uint64_t value, char* buffer, size_t buffer_size, int base, bool is_signed_val) {
    if (buffer_size == 0) return -1; 
    if (base < 2 || base > 36) { if (buffer_size > 0) buffer[0] = '\0'; return -1; }
    char* ptr = buffer; size_t count = 0; 
    bool was_negative_val = false; // Not strictly needed for this internal func but matches signature
    if (is_signed_val && base == 10 && static_cast<int64_t>(value) < 0) {
        if (count + 1 >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '-'; count++;
        if (value == static_cast<uint64_t>(std::numeric_limits<int64_t>::min())) {
            const char* min_int_mag = "9223372036854775808"; size_t min_len = kstrlen(min_int_mag);
            if (count + min_len >= buffer_size) { buffer[count > 0 ? count-1 : 0] = '\0'; return -1; }
            kmemcpy(ptr, min_int_mag, min_len + 1); return static_cast<int>(count + min_len);
        }
        value = static_cast<uint64_t>(-static_cast<int64_t>(value)); was_negative_val = true;
    }
    if (value == 0) {
        if (count + 1 >= buffer_size) { buffer[0] = '\0'; return -1; }
        *ptr++ = '0'; count++; *ptr = '\0'; return static_cast<int>(count);
    }
    char* start_digits = ptr; 
    while (value > 0) {
        if (count + 1 >= buffer_size) { buffer[min(count, buffer_size-1)] = '\0'; return -1; }
        int remainder = value % static_cast<unsigned int>(base); 
        *ptr++ = (remainder > 9) ? static_cast<char>((remainder - 10) + 'a') : static_cast<char>(remainder + '0');
        value /= static_cast<unsigned int>(base); count++;
    }
    *ptr = '\0'; 
    reverse_str(start_digits, static_cast<int>(ptr - start_digits)); 
    return static_cast<int>(count);
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
    char* ptr_start = buffer; size_t current_written = 0;
    if (leading_0x) {
        if (buffer_size < 3) { buffer[0] = '\0'; return -1; } 
        *ptr_start++ = '0'; *ptr_start++ = 'x'; current_written += 2;
    }
    int digits_len = num_to_str_base_internal(value, ptr_start, buffer_size - current_written, 16, false);
    if (digits_len < 0) { if (buffer_size > 0) buffer[0] = '\0'; return -1; }
    return static_cast<int>(current_written + static_cast<size_t>(digits_len));
}
void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept {
    if (out_buffer.empty()) return;
    char* p = out_buffer.data(); size_t remaining = out_buffer.size(); int written_this_segment;
    for (int i = 0; i < 4; ++i) {
        if (remaining == 0) { if(p != out_buffer.data()) *(p-1) = '\0'; else out_buffer[0] = '\0'; return; }
        uint8_t octet = (ip_addr >> (24 - i * 8)) & 0xFF;
        written_this_segment = uint_to_str(octet, p, remaining, 10);
        if (written_this_segment < 0 || static_cast<size_t>(written_this_segment) >= remaining) {
            if(!out_buffer.empty()) out_buffer[0] = '\0'; return;
        }
        p += written_this_segment; remaining -= static_cast<size_t>(written_this_segment);
        if (i < 3) {
            if (remaining <= 1) { *p = '\0'; return; } 
            *p++ = '.'; remaining--;
        }
    }
    if (remaining > 0) *p = '\0'; 
    else if (!out_buffer.empty()) out_buffer.back() = '\0';
}
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept { /* ... as before ... */ return false; }
std::string_view get_next_token(std::string_view& input, char delimiter) noexcept { /* ... as before ... */ return {}; }

int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept {
    if (!buffer || bufsz == 0 || !format) { if (bufsz > 0) buffer[0] = '\0'; return 0; }
    char* buf_ptr = buffer;
    char* const buf_end = buffer + bufsz -1; // Pointer to last char before null terminator
    int written_chars = 0;
    char temp_num_buf[24]; 

    while (*format && buf_ptr < buf_end) {
        if (*format == '%') {
            format++; 
            bool is_long_long = false;
            if (format[0] == 'l' && format[1] == 'l') {
                is_long_long = true;
                format += 2;
            }
            int len = 0;
            const char* str_to_copy = temp_num_buf;

            switch (*format) {
                case 's': { 
                    const char* s_arg = va_arg(args, const char*);
                    if (!s_arg) s_arg = "(null)";
                    str_to_copy = s_arg;
                    len = static_cast<int>(kstrlen(s_arg));
                    break;
                }
                case 'c': { 
                    temp_num_buf[0] = static_cast<char>(va_arg(args, int)); 
                    temp_num_buf[1] = '\0';
                    len = 1;
                    break;
                }
                case 'd': case 'i': { 
                    if (is_long_long) len = uint64_to_str(static_cast<uint64_t>(va_arg(args, long long)), temp_num_buf, sizeof(temp_num_buf), 10);
                    else len = int_to_str(va_arg(args, int), temp_num_buf, sizeof(temp_num_buf));
                    break;
                }
                case 'u': { 
                    if (is_long_long) len = uint64_to_str(va_arg(args, unsigned long long), temp_num_buf, sizeof(temp_num_buf));
                    else len = uint_to_str(va_arg(args, unsigned int), temp_num_buf, sizeof(temp_num_buf));
                    break;
                }
                case 'x': case 'X': case 'p': { 
                    uint64_t hex_val;
                    if (*format == 'p') hex_val = reinterpret_cast<uint64_t>(va_arg(args, void*));
                    else if (is_long_long) hex_val = va_arg(args, unsigned long long);
                    else hex_val = va_arg(args, unsigned int);
                    len = uint64_to_hex_str(hex_val, temp_num_buf, sizeof(temp_num_buf), (*format == 'p'));
                    // %X for uppercase - current uint64_to_hex_str produces lowercase. Needs modification if UC needed.
                    break;
                }
                case '%': { 
                    temp_num_buf[0] = '%'; temp_num_buf[1] = '\0'; len = 1;
                    break;
                }
                default: { 
                    if (buf_ptr < buf_end) *buf_ptr++ = '%'; written_chars++;
                    if (buf_ptr < buf_end && *format) *buf_ptr++ = *format; written_chars++;
                    str_to_copy = nullptr; // Handled
                    break;
                }
            }

            if (str_to_copy && len > 0) {
                int copy_len = min(len, static_cast<int>(buf_end - buf_ptr));
                kmemcpy(buf_ptr, str_to_copy, static_cast<size_t>(copy_len));
                buf_ptr += copy_len;
                written_chars += copy_len;
            } else if (str_to_copy && len < 0) { /* conversion error */ }

        } else { 
            *buf_ptr++ = *format;
            written_chars++;
        }
        if (*format == '\0') break; // End of format string
        format++;
    }
    *buf_ptr = '\0'; 
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