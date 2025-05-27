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

namespace cli {

// Constants for CLI behavior
constexpr size_t MAX_COMMAND_LENGTH = 128;    ///< Maximum length of a single command line.
constexpr size_t MAX_COMMANDS = 32;           ///< Maximum number of registered commands.
constexpr size_t MAX_HISTORY = 16;            ///< Maximum number of commands stored in history.

/**
 * @brief Type alias for command handler functions.
 * @param args A C-string containing the arguments passed to the command (can be nullptr if no args).
 * @param uart_ops Pointer to UART driver operations for command output.
 * @return Integer status code (typically 0 for success, non-zero for error).
 */
using CommandHandler = int (*)(const char* args, kernel::hal::UARTDriverOps* uart_ops);

/**
 * @brief Structure to define a CLI command.
 */
struct Command {
    const char* name;        ///< Command name (e.g., "help", "ls"). Must be a null-terminated string.
    CommandHandler handler;  ///< Function pointer to the command handler.
    const char* help_text;   ///< Brief description of the command for help display. Must be null-terminated.
};

/**
 * @brief Manages the Command Line Interface.
 * @details This class uses static members and methods as it's designed for a single,
 * system-wide CLI instance. It handles input processing, command dispatching,
 * and history management.
 */
class CLI {
public:
    /**
     * @brief Initializes the CLI subsystem.
     * @details Registers built-in commands and prepares the CLI for operation.
     * This should be called once during system startup.
     * @param uart_ops Pointer to the UART driver operations for I/O.
     */
    static void init(kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Registers a new command with the CLI.
     * @param name Name of the command. Must be a null-terminated string with static or long-lived storage.
     * @param handler Function pointer to the command's handler.
     * @param help_text Help string for the command. Must be a null-terminated string with static or long-lived storage.
     * @return True if registration was successful, false otherwise (e.g., max commands reached or invalid parameters).
     */
    static bool register_command(const char* name, CommandHandler handler, const char* help_text);

    /**
     * @brief Processes a single line of input from the CLI.
     * @details Parses the command name and arguments from the line, then dispatches to the appropriate handler.
     * @param line The input line (as a string_view) to process.
     * @param uart_ops Pointer to the UART driver operations for outputting command results or errors.
     * @return True if the line was processed (e.g., command found and executed, or "unknown command" handled),
     *         false if the line was empty or only whitespace.
     */
    static bool process_line(std::string_view line, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Executes commands from a script file located in the file system.
     * @param filename Absolute path to the script file.
     * @param uart_ops Pointer to the UART driver operations for command I/O during script execution.
     * @return True if the script was processed successfully (all commands executed, though individual commands might have failed),
     *         false on error (e.g., file not found, read error).
     */
    static bool process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Entry point for the CLI thread.
     * @details This function contains the main loop for reading user input, processing commands,
     * and managing command history. It typically runs in its own dedicated kernel thread.
     * @param uart_ops_ptr A void pointer to `kernel::hal::UARTDriverOps`, passed from the thread creation mechanism.
     */
    static void cli_thread_entry(void* uart_ops_ptr);

// Public static data members for CLI state and command registration.
// Made public to be accessible by free functions like cli_help_command and cli_history_command,
// and for direct access from cli_thread_entry.
public:
    static std::array<Command, MAX_COMMANDS> g_commands; ///< Global array of registered commands.
    static std::atomic<size_t> g_num_commands;           ///< Atomic counter for the number of registered commands.
    
    static std::array<char, MAX_COMMAND_LENGTH> cmd_buffer_; ///< Static buffer for the current command line being edited.
    static size_t cmd_buffer_idx_;                          ///< Current index/length within `cmd_buffer_`.
    static std::array<std::array<char, MAX_COMMAND_LENGTH>, MAX_HISTORY> command_history_; ///< Static buffer for command history.
    static size_t history_idx_;                             ///< Index for navigating command history (e.g., with arrow keys).
    static size_t history_count_;                           ///< Number of actual entries currently in `command_history_`.
};

} // namespace cli

#endif // CLI_HPP