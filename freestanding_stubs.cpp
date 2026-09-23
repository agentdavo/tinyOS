// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file freestanding_stubs.cpp
 * @brief Provides freestanding implementations for common C library functions and atomic builtins.
 */

#include <cstddef> // For size_t
#include <cstdint> // For integer types
#include <atomic>  // For the early_uart_puts serialisation flag
#include "util.hpp" // cpu_relax() — arch-portable yield hint

// extern "C" functions
extern "C" {

// --- Memory and String Functions ---
void* memcpy(void* dest_ptr, const void* src_ptr, size_t count) {
    auto* dest = static_cast<unsigned char*>(dest_ptr);
    const auto* src = static_cast<const unsigned char*>(src_ptr);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = src[i];
    }
    return dest_ptr;
}

void* __memcpy_chk(void* dest_ptr, const void* src_ptr, size_t count, size_t /*dest_len*/) {
    return memcpy(dest_ptr, src_ptr, count);
}

// Overlap-safe copy. The compiler is free to lower an overlapping struct/array
// assignment to a memmove call; without this stub that either fails to link or
// (worse) silently resolves to the forward-only memcpy and corrupts on
// overlap. Copy backward when the regions overlap with dest above src.
void* memmove(void* dest_ptr, const void* src_ptr, size_t count) {
    auto* dest = static_cast<unsigned char*>(dest_ptr);
    const auto* src = static_cast<const unsigned char*>(src_ptr);
    if (dest == src || count == 0) return dest_ptr;
    if (dest < src) {
        for (size_t i = 0; i < count; ++i) dest[i] = src[i];
    } else {
        for (size_t i = count; i != 0; --i) dest[i - 1] = src[i - 1];
    }
    return dest_ptr;
}

void* __memmove_chk(void* dest_ptr, const void* src_ptr, size_t count, size_t /*dest_len*/) {
    return memmove(dest_ptr, src_ptr, count);
}

void* memset(void* dest_ptr, int ch_int, size_t count) {
    auto* dest = static_cast<unsigned char*>(dest_ptr);
    unsigned char ch = static_cast<unsigned char>(ch_int);
    for (size_t i = 0; i < count; ++i) {
        dest[i] = ch;
    }
    return dest_ptr;
}

void* __memset_chk(void* dest_ptr, int ch_int, size_t count, size_t /*dest_len*/) {
    return memset(dest_ptr, ch_int, count);
}

int memcmp(const void* ptr1, const void* ptr2, size_t count) {
    const auto* p1 = static_cast<const unsigned char*>(ptr1);
    const auto* p2 = static_cast<const unsigned char*>(ptr2);
    for (size_t i = 0; i < count; ++i) {
        if (p1[i] != p2[i]) {
            return (p1[i] < p2[i]) ? -1 : 1;
        }
    }
    return 0;
}

void* memchr(const void* s, int c, size_t n) {
    const unsigned char* p = static_cast<const unsigned char*>(s);
    unsigned char ch = static_cast<unsigned char>(c);
    for (size_t i = 0; i < n; ++i) {
        if (p[i] == ch) return (void*)(p + i);
    }
    return nullptr;
}

size_t strlen(const char* str) {
    if (!str) return 0; 
    size_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

char* strcpy(char* dest, const char* src) {
    if (!dest || !src) return dest; 
    char* orig_dest = dest;
    while ((*dest++ = *src++)) {}
    return orig_dest;
}

char* strncpy(char* dest, const char* src, size_t count) {
    if (!dest || !src) return dest;
    char* orig_dest = dest;
    size_t i;
    for (i = 0; i < count && src[i] != '\0'; ++i) {
        dest[i] = src[i];
    }
    for (; i < count; ++i) { 
        dest[i] = '\0';
    }
    return orig_dest;
}

int strcmp(const char* lhs, const char* rhs) {
    if (!lhs && !rhs) return 0;
    if (!lhs) return -1; 
    if (!rhs) return 1;
    while (*lhs && (*lhs == *rhs)) {
        lhs++;
        rhs++;
    }
    return static_cast<int>(static_cast<unsigned char>(*lhs)) - 
           static_cast<int>(static_cast<unsigned char>(*rhs));
}

int strncmp(const char* lhs, const char* rhs, size_t count) {
    if (count == 0) return 0;
    if (!lhs && !rhs) return 0; 
    if (!lhs) return -1; 
    if (!rhs) return 1;
    
    size_t i = 0;
    while (i < count && lhs[i] && rhs[i] && (lhs[i] == rhs[i])) {
        if (lhs[i] == '\0') { 
            return 0;
        }
        i++;
    }
    if (i == count) return 0; 
    
    return static_cast<int>(static_cast<unsigned char>(lhs[i])) - 
           static_cast<int>(static_cast<unsigned char>(rhs[i]));
}

#if defined(__aarch64__)
// QEMU virt arm64 PL011.
constexpr uint64_t EARLY_UART_BASE_ADDR = 0x09000000;
constexpr uint32_t EARLY_UART_DR_REG  = 0x00;
constexpr uint32_t EARLY_UART_FR_REG  = 0x18;
constexpr uint32_t EARLY_UART_IBRD_REG = 0x24;
constexpr uint32_t EARLY_UART_FBRD_REG = 0x28;
constexpr uint32_t EARLY_UART_LCRH_REG = 0x2C;
constexpr uint32_t EARLY_UART_CR_REG  = 0x30;
constexpr uint32_t EARLY_UART_IMSC_REG = 0x38;
constexpr uint32_t EARLY_UART_ICR_REG  = 0x44;
constexpr uint32_t EARLY_UART_TXFF_FLAG = (1 << 5);

extern "C" void early_uart_init() {
    volatile uint32_t* base = reinterpret_cast<volatile uint32_t*>(EARLY_UART_BASE_ADDR);
    base[EARLY_UART_CR_REG / 4] = 0;          // Disable UART
    base[EARLY_UART_ICR_REG / 4] = 0x7FF;     // Clear interrupts
    base[EARLY_UART_IBRD_REG / 4] = 13;       // 115200 baud for 24MHz clock
    base[EARLY_UART_FBRD_REG / 4] = 2;
    base[EARLY_UART_LCRH_REG / 4] = (3 << 5); // 8N1
    base[EARLY_UART_CR_REG / 4] = (1 << 9) | (1 << 8) | 1; // Enable UART, TX, RX
}

static inline void early_uart_putc_unlocked(char c) {
    while ((*reinterpret_cast<volatile uint32_t*>(EARLY_UART_BASE_ADDR + EARLY_UART_FR_REG)) & EARLY_UART_TXFF_FLAG) {}
    *reinterpret_cast<volatile uint32_t*>(EARLY_UART_BASE_ADDR + EARLY_UART_DR_REG) = static_cast<uint32_t>(c);
}

#elif defined(__riscv)
// QEMU virt rv64 NS16550A. THR at offset 0; LSR at offset 5; bit 5 of LSR =
// transmitter holding register empty (TX ready). The kernel-side rv64
// UARTDriver writes THR without polling, but the early-boot path needs to
// be polite to the FIFO, so we wait on LSR.THRE here.
constexpr uint64_t EARLY_UART_BASE_ADDR = 0x10000000;
constexpr uint32_t NS16550_THR = 0x00;
constexpr uint32_t NS16550_LSR = 0x05;
constexpr uint8_t  NS16550_LSR_THRE = 1u << 5;

extern "C" void early_uart_init() {
    // QEMU virt's NS16550 is preconfigured by the bios=none boot; nothing
    // to program here. Stub kept for symmetry with arm64.
}

static inline void early_uart_putc_unlocked(char c) {
    volatile uint8_t* lsr = reinterpret_cast<volatile uint8_t*>(EARLY_UART_BASE_ADDR + NS16550_LSR);
    volatile uint8_t* thr = reinterpret_cast<volatile uint8_t*>(EARLY_UART_BASE_ADDR + NS16550_THR);
    while ((*lsr & NS16550_LSR_THRE) == 0) {}
    *thr = static_cast<uint8_t>(c);
}

#else
extern "C" void early_uart_init() {}
static inline void early_uart_putc_unlocked(char) {}
#endif

// Simple cross-core serialisation for early_uart_puts so debug output from
// multiple cores doesn't interleave mid-string. One atomic flag, test-and-set
// acquire / relaxed-store release. Writers that contend spin with `yield` —
// acceptable overhead for diagnostic output paths.
static std::atomic<bool> g_early_uart_lock{false};

static inline void early_uart_write_unlocked(const char* str) {
    while (*str) {
        if (*str == '\n') early_uart_putc_unlocked('\r');
        early_uart_putc_unlocked(*str++);
    }
}

extern "C" void early_uart_lock_acquire() {
    while (g_early_uart_lock.exchange(true, std::memory_order_acquire)) {
        kernel::util::cpu_relax();
    }
}

extern "C" void early_uart_lock_release() {
    g_early_uart_lock.store(false, std::memory_order_release);
}

void early_uart_puts(const char* str) {
    if (!str) return;

    // Test-and-set acquire. No spurious failures, no CAS-retry pitfall.
    while (g_early_uart_lock.exchange(true, std::memory_order_acquire)) {
        kernel::util::cpu_relax();
    }

    early_uart_write_unlocked(str);

    g_early_uart_lock.store(false, std::memory_order_release);
}

static inline void format_hex16(uint64_t value, char out[17]) {
    const char digits[] = "0123456789abcdef";
    out[16] = '\0';
    for (int i = 15; i >= 0; --i) {
        out[i] = digits[value & 0xF];
        value >>= 4;
    }
}

// Emit an entire "[boot] core <hex16>: <phase>" line atomically — the ASM
// `boot_log_core_phase` used to call early_uart_puts five times, so lines
// from different cores could interleave between segments even though each
// individual puts is locked.
extern "C" void boot_log_core_phase_c(uint64_t core_id, const char* phase) {
    if (!phase) phase = "";
    char hex[17];
    format_hex16(core_id, hex);

    while (g_early_uart_lock.exchange(true, std::memory_order_acquire)) {
        kernel::util::cpu_relax();
    }
    early_uart_write_unlocked("[boot] core ");
    early_uart_write_unlocked(hex);
    early_uart_write_unlocked(": ");
    early_uart_write_unlocked(phase);
    g_early_uart_lock.store(false, std::memory_order_release);
}

// Emit a "<label><hex16>" pair atomically (ASM boot_log_reg64 used two
// separate lock acquisitions and could interleave with other cores).
extern "C" void boot_log_reg64_c(uint64_t value, const char* label) {
    if (!label) label = "";
    char hex[17];
    format_hex16(value, hex);

    while (g_early_uart_lock.exchange(true, std::memory_order_acquire)) {
        kernel::util::cpu_relax();
    }
    early_uart_write_unlocked(label);
    early_uart_write_unlocked(hex);
    g_early_uart_lock.store(false, std::memory_order_release);
}


} // extern "C"
