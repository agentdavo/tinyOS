// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.hpp
 * @brief Command Line Interface (CLI) subsystem header for miniOS v1.7.
 * @details
 * Defines an interactive CLI for debugging and controlling miniOS, supporting thread-safe command
 * execution, tab completion, command history, and dynamic command registration from subsystems
 * (kernel, DSP, audio, trace, RAMFS, networking, GPIO). Enhanced in v1.7 with improved error
 * handling, clearer documentation, and modern C++20 practices, retaining all v1.6 functionality
 * including script execution and extensive command set.
 *
 * C++20 features:
 * - std::string_view for efficient string handling
 * - std::span for buffer operations
 * - std::atomic for thread safety
 *
 * @version 1.7
 * @see cli.cpp, miniOS.hpp, util.hpp, dsp.hpp, audio.hpp, trace.hpp, fs.hpp, net.hpp, gpio.hpp
 */

#ifndef CLI_HPP
#define CLI_HPP

#include "miniOS.hpp"
#include <string_view>
#include <span>
#include <array>
#include <atomic>

namespace cli {

constexpr size_t MAX_COMMANDS = 32;
constexpr size_t MAX_COMMAND_LENGTH = 128;
constexpr size_t MAX_HISTORY = 10;

/**
 * @brief CLI command handler function signature.
 * @param args Command arguments
 * @param uart_ops UART driver for output
 * @return 0 on success, non-zero on failure
 */
using CommandHandler = int (*)(const char* args, kernel::hal::UARTDriverOps* uart_ops);

/**
 * @brief CLI command structure.
 */
struct Command {
    const char* name; ///< Command name
    CommandHandler handler; ///< Command handler function
    const char* help; ///< Help text
};

/**
 * @brief CLI subsystem class.
 */
class CLI {
public:
    CLI();

    /**
     * @brief Registers a new CLI command.
     * @param name Command name (must be unique)
     * @param handler Command handler function
     * @param help Help text for the command
     * @return True if registered, false if command limit reached or duplicate
     */
    bool register_command(const char* name, CommandHandler handler, const char* help);

    /**
     * @brief Processes a single command line.
     * @param line Command line to process
     * @param uart_ops UART driver for output
     * @return True if command was recognized, false otherwise
     */
    bool process_line(std::string_view line, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Processes a script file from RAMFS.
     * @param filename Script file path
     * @param uart_ops UART driver for output
     * @return True if script executed successfully, false otherwise
     */
    bool process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Performs tab completion for the current input.
     * @param uart_ops UART driver for output
     */
    void complete_command(kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief Navigates command history (up/down).
     * @param up True for up (previous), false for down (next)
     * @param uart_ops UART driver for output
     */
    void navigate_history(bool up, kernel::hal::UARTDriverOps* uart_ops);

    /**
     * @brief CLI thread entry point.
     * @param arg UART driver operations (kernel::hal::UARTDriverOps*)
     */
    static void cli_thread_entry(void* arg);

    /**
     * @brief Registers core CLI commands (help, script, etc.).
     */
    void register_core_commands();

private:
    static std::array<Command, MAX_COMMANDS> g_commands; ///< Registered commands
    static std::atomic<size_t> g_num_commands; ///< Number of registered commands
    std::array<char, MAX_COMMAND_LENGTH> cmd_buffer_; ///< Current command buffer
    size_t cmd_buffer_idx_ = 0; ///< Current buffer index
    std::array<std::array<char, MAX_COMMAND_LENGTH>, MAX_HISTORY> command_history_; ///< Command history
    size_t history_idx_ = 0; ///< Current history index
    size_t history_count_ = 0; ///< Number of history entries
};

} // namespace cli

#endif // CLI_HPP