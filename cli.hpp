// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.hpp
 * @brief Command Line Interface (CLI) subsystem header for miniOS v1.7.
 * @details
 * Defines a simple, extensible CLI framework for interacting with miniOS. Supports command
 * registration, argument parsing, command history, and basic line editing. Integrates with
 * various subsystems (FS, Net, GPIO, DSP, Audio, Trace) through specific commands.
 * Updated in v1.7 with improved error handling, clearer documentation, and modern C++20 features.
 *
 * C++20 features:
 * - std::string_view for efficient string handling
 * - std::array for fixed-size buffers
 *
 * @version 1.7
 * @see cli.cpp, miniOS.hpp, util.hpp
 */

#ifndef CLI_HPP
#define CLI_HPP

#include "miniOS.hpp" // For kernel::hal::UARTDriverOps and other kernel types
#include <string_view>
#include <array>
#include <atomic> // For std::atomic with g_num_commands

// Forward declare kernel::hal::UARTDriverOps if miniOS.hpp isn't guaranteed to be included first by all users
// However, cli.cpp includes this, which includes miniOS.hpp.
// And miniOS.hpp includes cli.hpp (after kernel defs with the fix).

namespace cli {

// Constants for CLI behavior
constexpr size_t MAX_COMMAND_LENGTH = 128;
constexpr size_t MAX_COMMANDS = 32;       // Max number of registered commands
constexpr size_t MAX_HISTORY = 16;        // Max number of commands in history

// Type alias for command handler functions
// Takes raw arguments string and UART operations pointer for output
using CommandHandler = int (*)(const char* args, kernel::hal::UARTDriverOps* uart_ops);

/**
 * @brief Structure to define a CLI command.
 */
struct Command {
    const char* name;        ///< Command name (e.g., "help", "ls")
    CommandHandler handler;  ///< Function pointer to the command handler
    const char* help_text;   ///< Brief description of the command
};

/**
 * @brief Manages the Command Line Interface.
 * @details This class is designed to be mostly static as there's typically one system-wide CLI.
 */
class CLI {
public:
    /**
     * @brief Initializes the CLI subsystem.
     * @details Registers built-in commands.
     * @param uart_ops Pointer to the UART driver operations for I/O.
     */
    static void init(kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Registers a new command with the CLI.
     * @param name Name of the command.
     * @param handler Function pointer to the command's handler.
     * @param help_text Help string for the command.
     * @return True if registration was successful, false otherwise (e.g., max commands reached).
     */
    static bool register_command(const char* name, CommandHandler handler, const char* help_text);

    /**
     * @brief Processes a single line of input from the CLI.
     * @param line The input line to process.
     * @param uart_ops Pointer to the UART driver operations for output.
     * @return True if the line was processed (command found or error handled), false for empty line.
     */
    static bool process_line(std::string_view line, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Executes commands from a script file.
     * @param filename Path to the script file.
     * @param uart_ops Pointer to the UART driver operations.
     * @return True if script processed successfully, false on error (e.g., file not found).
     */
    static bool process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Entry point for the CLI thread.
     * @param uart_ops_ptr Pointer to UARTDriverOps, passed as void* from thread creation.
     */
    static void cli_thread_entry(void* uart_ops_ptr);

// Make these public static so free functions like cli_help_command and cli_history_command can access them.
// Alternatively, those commands could become static methods of CLI.
public:
    // Static members for command registration and global CLI state
    static std::array<Command, MAX_COMMANDS> g_commands; ///< Registered commands
    static std::atomic<size_t> g_num_commands;           ///< Number of registered commands
    
    // Made these static to be accessible from static cli_thread_entry and cli_history_command
    static std::array<char, MAX_COMMAND_LENGTH> cmd_buffer_; 
    static size_t cmd_buffer_idx_; 
    static std::array<std::array<char, MAX_COMMAND_LENGTH>, MAX_HISTORY> command_history_; 
    static size_t history_idx_; 
    static size_t history_count_; 
};

} // namespace cli

#endif // CLI_HPP