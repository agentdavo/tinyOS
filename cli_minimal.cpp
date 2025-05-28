// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli_minimal.cpp
 * @brief Minimal CLI subsystem implementation for miniOS v1.7 kernel.
 * @details
 * Implements a minimal command-line interface for the core kernel, supporting
 * trace and stats commands via command registration. Used for the `make kernel` target.
 *
 * @version 1.7
 * @see cli.hpp, core.hpp, hal.hpp, miniOS.hpp
 */

#include "cli.hpp"
#include "miniOS.hpp"
#include <cstring>

namespace cli {

CLI g_cli;

// Command handlers
[[maybe_unused]] static int trace_command([[maybe_unused]] const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    kernel::dump_trace_buffer(uart_ops);
    return 0;
}

[[maybe_unused]] static int stats_command([[maybe_unused]] const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    kernel::get_kernel_stats(uart_ops);
    return 0;
}

bool CLI::register_command([[maybe_unused]] const char* name,
                           [[maybe_unused]] CommandHandler handler,
                           [[maybe_unused]] const char* help_text) {
    // Stub implementation for minimal kernel
    return true;
}

CLI::CLI() {
    // Register core commands
    register_command("trace", trace_command, "Dump trace buffer");
    register_command("stats", stats_command, "Show kernel statistics");
}

} // namespace cli