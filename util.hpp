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

// Arch-portable CPU relax hint. arm64 has a cheap `yield` instruction
// (scheduler-friendly NOP that hints SMT peers); rv64 lacks a direct
// equivalent in the base ISA, so we fall back to `nop`. Spin loops that
// wait for a queue / condition to change should call this to avoid
// burning branch-predictor / commit-queue resources while still being
// a single cycle on both archs.
namespace kernel { namespace util {
static inline void cpu_relax() noexcept {
#if defined(__aarch64__)
    asm volatile("yield");
#elif defined(__riscv)
    asm volatile("nop");
#else
    asm volatile("");
#endif
}
}} // namespace kernel::util

// Declare the C-linkage functions that are defined in freestanding_stubs.cpp
// This makes them visible to C++ code in the global namespace.
extern "C" {
    void* memcpy(void* dest, const void* src, size_t count);
    void* memmove(void* dest, const void* src, size_t count);
    void* memset(void* dest, int ch, size_t count);
    int memcmp(const void* ptr1, const void* ptr2, size_t count);
    size_t strlen(const char* str);
    int strcmp(const char* lhs, const char* rhs);
    int strncmp(const char* lhs, const char* rhs, size_t count);
    char* strcpy(char* dest, const char* src);
    char* strncpy(char* dest, const char* src, size_t count);
    void early_uart_init();
    void early_uart_puts(const char* str);
}


namespace kernel {
namespace util {

// Inline wrappers in the kernel::util namespace calling global extern "C" versions
// These provide a namespaced API for the rest of the kernel.
inline void* kmemcpy(void* dest, const void* src, size_t count) noexcept {
    return ::memcpy(dest, src, count); // Calls global C memcpy
}

inline void* kmemset(void* dest, int ch, size_t count) noexcept {
    return ::memset(dest, ch, count); // Calls global C memset
}

inline int kmemcmp(const void* ptr1, const void* ptr2, size_t count) noexcept {
    return ::memcmp(ptr1, ptr2, count);
}

// Overflow-safe (a * b) / c for 64-bit operands. The naive `a * b` overflows
// uint64_t for timer math: at QEMU's 62.5 MHz arm64 / 10 MHz rv64 generic
// timers, `ticks * 1e9` wraps after only a few minutes of uptime, silently
// corrupting every ns-based deadline (wait_until_ns, motion, EtherCAT DC).
// Promote to 128-bit for the intermediate product so the result is exact for
// the full 64-bit tick range. GCC provides __uint128_t on both aarch64 and
// rv64 (lp64d) targets. Returns 0 if c == 0 rather than trapping.
//
// Implemented without a 128-bit divide: a*b/c == (a/c)*b + ((a%c)*b)/c
// exactly, and (a%c)*b < c*b fits in 64 bits whenever c*b does — true for
// every tick<->time conversion (c = timer Hz, b = 1e6 or 1e9, or the
// reverse). The old `(__int128)a*b / c` lowered to libgcc's __udivti3 on
// every time read; on rv64, Ubuntu's RVA23-built libgcc version of it traps
// on plain rv64 CPUs. Only if c*b itself overflows do we fall back to a
// shift-subtract long division of the full 128-bit product (inline, no
// libgcc). Result is truncated to 64 bits, as before.
inline uint64_t mul_div_u64(uint64_t a, uint64_t b, uint64_t c) noexcept {
    if (c == 0) return 0;
    if (b == 0 || c <= ~0ULL / b) return (a / c) * b + ((a % c) * b) / c;
    const unsigned __int128 n = static_cast<unsigned __int128>(a) * b;
    unsigned __int128 rem = 0;
    uint64_t q = 0;
    for (int bit = 127; bit >= 0; --bit) {
        rem = (rem << 1) | ((n >> bit) & 1u);
        q <<= 1;
        if (rem >= c) { rem -= c; q |= 1u; }
    }
    return q;
}

inline size_t kstrlen(const char* str) noexcept {
    // The global ::strlen already handles null, but an extra check here is harmless.
    if (!str) return 0; 
    return ::strlen(str); 
}

inline int kstrcmp(const char* lhs, const char* rhs) noexcept {
    return ::strcmp(lhs, rhs);
}

inline int kstrncmp(const char* lhs, const char* rhs, size_t count) noexcept {
    return ::strncmp(lhs, rhs, count); 
}

// Declarations for functions defined in util.cpp
bool safe_strcpy(char* dest, const char* src, size_t dest_size) noexcept; 
char* kstrcat(char* dest, const char* src, size_t dest_max_len) noexcept; 

// Character functions (can be inline as they are simple)
inline bool isspace(char c) noexcept { 
    return (c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r');
}
inline bool isdigit(char c) noexcept { 
    return (c >= '0' && c <= '9');
}

// Decimal float scanner shared by every text-config / G-code parser (there
// used to be seven hand-rolled copies). Reads [+-]digits[.digits] starting at
// s[idx] and advances idx past what it consumed. With allow_exponent, an
// e/E followed by [+-]digits scales the result; G-code callers leave it off
// so a following word letter is never swallowed. Stops at the first
// character that doesn't fit, like strtof without locale or hex.
inline float parse_float_at(const char* s, size_t& idx, bool allow_exponent = false) noexcept {
    bool neg = false;
    if (s[idx] == '-') { neg = true; ++idx; }
    else if (s[idx] == '+') { ++idx; }
    float value = 0.0f;
    while (isdigit(s[idx])) {
        value = value * 10.0f + static_cast<float>(s[idx] - '0');
        ++idx;
    }
    if (s[idx] == '.') {
        ++idx;
        float place = 0.1f;
        while (isdigit(s[idx])) {
            value += static_cast<float>(s[idx] - '0') * place;
            place *= 0.1f;
            ++idx;
        }
    }
    if (allow_exponent && (s[idx] == 'e' || s[idx] == 'E')) {
        size_t j = idx + 1;
        bool eneg = false;
        if (s[j] == '-') { eneg = true; ++j; }
        else if (s[j] == '+') { ++j; }
        if (isdigit(s[j])) {
            int exp = 0;
            while (isdigit(s[j])) {
                if (exp < 100) exp = exp * 10 + (s[j] - '0');
                ++j;
            }
            for (int k = 0; k < exp; ++k) value = eneg ? value * 0.1f : value * 10.0f;
            idx = j;
        }
    }
    return neg ? -value : value;
}

// Whole-string convenience: null or empty returns `fallback`.
inline float parse_float(const char* s, float fallback = 0.0f) noexcept {
    if (!s || !*s) return fallback;
    size_t idx = 0;
    return parse_float_at(s, idx, true);
}

// IP address conversion
bool ipv4_to_uint32(std::string_view ip_str, uint32_t& ip_addr) noexcept;

// Number to string conversion helpers (definitions in util.cpp)
int int_to_str(int32_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int int64_to_str(int64_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint_to_str(uint32_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint64_to_str(uint64_t value, char* buffer, size_t buffer_size, int base = 10) noexcept;
int uint64_to_hex_str(uint64_t value, char* buffer, size_t buffer_size, bool leading_0x = true) noexcept;

void uint32_to_ipv4_str(uint32_t ip_addr, std::span<char> out_buffer) noexcept; 

// Kernel bump-heap usage (cpp_runtime_stubs.cpp). operator delete is a no-op,
// so `used` is monotonic.
size_t kernel_heap_used() noexcept;
size_t kernel_heap_capacity() noexcept;

// Simplified snprintf-like functions (definitions in util.cpp)
int k_vsnprintf(char* buffer, size_t bufsz, const char* format, va_list args) noexcept;
int k_snprintf(char* buffer, size_t bufsz, const char* format, ...) noexcept __attribute__((format(printf, 3, 4)));

template <typename T>
constexpr const T& min(const T& a, const T& b) { return (b < a) ? b : a; }
template <typename T>
constexpr const T& max(const T& a, const T& b) { return (a < b) ? b : a; }

} // namespace util
} // namespace kernel

#endif // UTIL_HPP