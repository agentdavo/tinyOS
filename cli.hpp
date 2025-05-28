// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.hpp
 * @brief CLI subsystem header for miniOS v1.7.
 * @details
 * Defines command-line interface for interacting with the miniOS kernel.
 *
 * @version 1.7
 * @see cli.cpp, core.hpp, hal.hpp
 */

#ifndef CLI_HPP
#define CLI_HPP

#include "core.hpp"
#include "hal.hpp"
#include <string_view>

namespace cli {

using CommandHandler = int (*)(const char* args, kernel::hal::UARTDriverOps* uart_ops);

class CLI {
public:
    CLI();

    static bool register_command(const char* name, CommandHandler handler, const char* help_text);
};

extern CLI g_cli;

} // namespace cli

#endif // CLI_HPP