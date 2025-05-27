// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.cpp
 * @brief Command Line Interface (CLI) subsystem implementation for miniOS v1.7.
 * @details
 * Implements an interactive CLI for debugging and controlling miniOS, with thread-safe command
 * execution, tab completion, command history, and dynamic command registration. Supports commands
 * for kernel, DSP (including 4-way crossover, effects), audio, trace, RAMFS, networking, and GPIO.
 * Enhanced in v1.7 with improved error handling, clearer diagnostics, and modern C++20 practices,
 * retaining all v1.6 functionality including script execution and extensive command set.
 *
 * C++20 features:
 * - std::string_view for efficient string handling
 * - std::span for buffer operations
 * - std::atomic for thread safety
 *
 * @version 1.7
 * @see cli.hpp, miniOS.hpp, util.hpp, dsp.hpp, audio.hpp, trace.hpp, fs.hpp, net.hpp, gpio.hpp
 */

#include "cli.hpp"
#include "util.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "trace.hpp"
#include "fs.hpp"
#include "net.hpp"
#include "gpio.hpp"
#include <cstring>
#include <array>

namespace cli {

std::array<CLI::Command, CLI::MAX_COMMANDS> CLI::g_commands;
std::atomic<size_t> CLI::g_num_commands{0};

CLI::CLI() : cmd_buffer_{}, cmd_buffer_idx_(0), history_idx_(0), history_count_(0) {
    command_history_.fill({});
}

bool CLI::register_command(const char* name, CommandHandler handler, const char* help) {
    if (!name || !handler || !help || g_num_commands.load(std::memory_order_relaxed) >= MAX_COMMANDS) {
        return false;
    }
    for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
        if (util::strcmp(g_commands[i].name, name) == 0) return false;
    }
    size_t idx = g_num_commands.fetch_add(1, std::memory_order_relaxed);
    if (idx >= MAX_COMMANDS) {
        g_num_commands.fetch_sub(1, std::memory_order_relaxed);
        return false;
    }
    g_commands[idx] = {name, handler, help};
    return true;
}

bool CLI::process_line(std::string_view line, kernel::hal::UARTDriverOps* uart_ops) {
    if (line.empty()) return false;
    std::array<char, MAX_COMMAND_LENGTH + 1> cmd_copy;
    size_t copy_len = std::min(line.size(), MAX_COMMAND_LENGTH);
    util::memcpy(cmd_copy.data(), line.data(), copy_len);
    cmd_copy[copy_len] = '\0';

    char* cmd_name = cmd_copy.data();
    char* args = std::strchr(cmd_name, ' ');
    if (args) {
        *args = '\0';
        args++;
        while (*args == ' ') args++;
    }

    for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
        if (util::strcmp(g_commands[i].name, cmd_name) == 0) {
            int result = g_commands[i].handler(args ? args : "", uart_ops);
            if (result != 0 && uart_ops) {
                uart_ops->puts("Command failed with code: ");
                uart_ops->uart_put_uint64_hex(result);
                uart_ops->puts("\n");
            }
            return true;
        }
    }

    if (uart_ops) {
        uart_ops->puts("Unknown command: ");
        uart_ops->puts(cmd_name);
        uart_ops->puts("\nType 'help' for commands\n");
    }
    return false;
}

bool CLI::process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops) {
    if (!filename || !fs::g_file_system.file_exists(filename)) {
        if (uart_ops) uart_ops->puts("Script file not found\n");
        return false;
    }

    std::array<char, 1024> script_data;
    size_t bytes_read = fs::g_file_system.read_file(filename, script_data.data(), script_data.size());
    if (bytes_read == 0) {
        if (uart_ops) uart_ops->puts("Failed to read script\n");
        return false;
    }

    std::string_view script(script_data.data(), bytes_read);
    size_t pos = 0;
    while (pos < script.size()) {
        size_t end = script.find('\n', pos);
        if (end == std::string_view::npos) end = script.size();
        std::string_view line = util::trim(script.substr(pos, end - pos));
        if (!line.empty()) {
            process_line(line, uart_ops);
        }
        pos = end + 1;
    }
    return true;
}

void CLI::complete_command(kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return;
    std::string_view input(cmd_buffer_.data(), cmd_buffer_idx_);
    std::string_view prefix = input;
    size_t space_pos = input.find(' ');
    if (space_pos != std::string_view::npos) prefix = input.substr(0, space_pos);

    const char* match = nullptr;
    size_t match_count = 0;
    for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
        if (std::strncmp(g_commands[i].name, prefix.data(), prefix.size()) == 0) {
            match = g_commands[i].name;
            match_count++;
        }
    }

    if (match_count == 1 && match) {
        size_t new_len = util::strlen(match);
        if (new_len > MAX_COMMAND_LENGTH - 1) new_len = MAX_COMMAND_LENGTH - 1;
        util::memcpy(cmd_buffer_.data(), match, new_len);
        cmd_buffer_[new_len] = '\0';
        cmd_buffer_idx_ = new_len;
        uart_ops->puts("\r> ");
        uart_ops->puts(cmd_buffer_.data());
    } else if (match_count > 1) {
        uart_ops->puts("\n");
        for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
            if (std::strncmp(g_commands[i].name, prefix.data(), prefix.size()) == 0) {
                uart_ops->puts(g_commands[i].name);
                uart_ops->puts("\n");
            }
        }
        uart_ops->puts("> ");
        uart_ops->puts(cmd_buffer_.data());
    }
}

void CLI::navigate_history(bool up, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return;
    if (up && history_idx_ < history_count_) {
        history_idx_++;
    } else if (!up && history_idx_ > 0) {
        history_idx_--;
    } else {
        return;
    }

    cmd_buffer_idx_ = 0;
    if (history_idx_ > 0) {
        const auto& hist_cmd = command_history_[history_count_ - history_idx_];
        while (cmd_buffer_idx_ < MAX_COMMAND_LENGTH - 1 && hist_cmd[cmd_buffer_idx_] != '\0') {
            cmd_buffer_[cmd_buffer_idx_] = hist_cmd[cmd_buffer_idx_];
            cmd_buffer_idx_++;
        }
        cmd_buffer_[cmd_buffer_idx_] = '\0';
    } else {
        cmd_buffer_[0] = '\0';
    }

    uart_ops->puts("\r> ");
    for (size_t i = 0; i < MAX_COMMAND_LENGTH; ++i) uart_ops->putc(' ');
    uart_ops->puts("\r> ");
    uart_ops->puts(cmd_buffer_.data());
}

void CLI::cli_thread_entry(void* arg) {
    auto* uart_ops = static_cast<kernel::hal::UARTDriverOps*>(arg);
    if (!uart_ops) return;
    CLI cli;
    cli.register_core_commands();
    uart_ops->puts("miniOS v1.7 CLI Ready. Type 'help' for commands\n> ");

    while (true) {
        char c = uart_ops->getc_blocking();
        if (c == '\r' || c == '\n') {
            cmd_buffer_[cmd_buffer_idx_] = '\0';
            if (cmd_buffer_idx_ > 0) {
                // Store in history
                if (history_count_ < MAX_HISTORY) {
                    util::memcpy(command_history_[history_count_].data(), cmd_buffer_.data(), cmd_buffer_idx_ + 1);
                    history_count_++;
                } else {
                    for (size_t i = 1; i < MAX_HISTORY; ++i) {
                        command_history_[i - 1] = command_history_[i];
                    }
                    util::memcpy(command_history_[MAX_HISTORY - 1].data(), cmd_buffer_.data(), cmd_buffer_idx_ + 1);
                }
                history_idx_ = 0;

                std::string_view cmd_line(cmd_buffer_.data(), cmd_buffer_idx_);
                cli.process_line(cmd_line, uart_ops);
            }
            cmd_buffer_idx_ = 0;
            uart_ops->puts("> ");
        } else if (c == '\b' && cmd_buffer_idx_ > 0) {
            cmd_buffer_idx_--;
            uart_ops->putc('\b'); uart_ops->putc(' '); uart_ops->putc('\b');
        } else if (c == '\t') {
            cli.complete_command(uart_ops);
        } else if (c == 0x1B) { // Escape sequence for arrow keys
            char seq[2];
            seq[0] = uart_ops->getc_blocking();
            seq[1] = uart_ops->getc_blocking();
            if (seq[0] == '[' && (seq[1] == 'A' || seq[1] == 'B')) {
                cli.navigate_history(seq[1] == 'A', uart_ops);
            }
        } else if (cmd_buffer_idx_ < MAX_COMMAND_LENGTH - 1 && c >= ' ' && c <= '~') {
            cmd_buffer_[cmd_buffer_idx_++] = c;
            uart_ops->putc(c);
        }
        kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
    }
}

int cli_help_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return 1;
    if (args && *args) {
        for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
            if (util::strcmp(g_commands[i].name, args) == 0) {
                uart_ops->puts(g_commands[i].name);
                uart_ops->puts(": ");
                uart_ops->puts(g_commands[i].help);
                uart_ops->puts("\n");
                return 0;
            }
        }
        uart_ops->puts("Command not found: ");
        uart_ops->puts(args);
        uart_ops->puts("\n");
        return 1;
    }
    uart_ops->puts("Available commands:\n");
    for (size_t i = 0; i < g_num_commands.load(std::memory_order_relaxed); ++i) {
        uart_ops->puts("  ");
        uart_ops->puts(g_commands[i].name);
        uart_ops->puts(" - ");
        uart_ops->puts(g_commands[i].help);
        uart_ops->puts("\n");
    }
    return 0;
}

int cli_script_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: script <filename>\n");
        return 1;
    }
    CLI cli;
    return cli.process_script(args, uart_ops) ? 0 : 1;
}

int cli_gpio_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: gpio config <bank> <pin> <input|output>\n");
        return 1;
    }
    int bank, pin;
    char mode[8];
    if (std::sscanf(args, "%d %d %7s", &bank, &pin, mode) != 3) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (bank < 0 || bank >= kernel::GPIO_BANKS || pin < 0 || pin >= kernel::GPIO_PINS_PER_BANK) {
        if (uart_ops) uart_ops->puts("Invalid bank or pin number\n");
        return 1;
    }
    kernel::hal::gpio::PinMode pin_mode = util::strcmp(mode, "output") == 0 ? kernel::hal::gpio::PinMode::OUTPUT : kernel::hal::gpio::PinMode::INPUT;
    if (gpio::g_gpio_manager.configure_pin(bank, pin, pin_mode)) {
        if (uart_ops) uart_ops->puts("GPIO pin configured\n");
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to configure GPIO pin\n");
    return 1;
}

int cli_gpio_read(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: gpio read <bank> <pin>\n");
        return 1;
    }
    int bank, pin;
    if (std::sscanf(args, "%d %d", &bank, &pin) != 2) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (bank < 0 || bank >= kernel::GPIO_BANKS || pin < 0 || pin >= kernel::GPIO_PINS_PER_BANK) {
        if (uart_ops) uart_ops->puts("Invalid bank or pin number\n");
        return 1;
    }
    kernel::hal::gpio::PinState state = gpio::g_gpio_manager.read_pin(bank, pin);
    if (uart_ops) {
        uart_ops->puts("GPIO pin state: ");
        uart_ops->puts(state == kernel::hal::gpio::PinState::HIGH ? "HIGH\n" : "LOW\n");
    }
    return 0;
}

int cli_gpio_write(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: gpio write <bank> <pin> <high|low>\n");
        return 1;
    }
    int bank, pin;
    char state[8];
    if (std::sscanf(args, "%d %d %7s", &bank, &pin, state) != 3) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (bank < 0 || bank >= kernel::GPIO_BANKS || pin < 0 || pin >= kernel::GPIO_PINS_PER_BANK) {
        if (uart_ops) uart_ops->puts("Invalid bank or pin number\n");
        return 1;
    }
    kernel::hal::gpio::PinState pin_state = util::strcmp(state, "high") == 0 ? kernel::hal::gpio::PinState::HIGH : kernel::hal::gpio::PinState::LOW;
    if (gpio::g_gpio_manager.set_pin(bank, pin, pin_state)) {
        if (uart_ops) uart_ops->puts("GPIO pin written\n");
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to write GPIO pin\n");
    return 1;
}

int cli_net_create(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: net create <udp|tcp> <ip> <port>\n");
        return 1;
    }
    char type[8], ip_str[16];
    int port;
    if (std::sscanf(args, "%7s %15s %d", type, ip_str, &port) != 3 || port < 0 || port > 65535) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    uint32_t ip_addr;
    if (!util::ipv4_to_uint32(ip_str, ip_addr)) {
        if (uart_ops) uart_ops->puts("Invalid IP address\n");
        return 1;
    }
    net::IPv4Addr ip{.addr = ip_addr};
    int socket_idx = util::strcmp(type, "tcp") == 0 ?
                     net::g_net_manager.create_tcp_socket(ip, static_cast<uint16_t>(port)) :
                     net::g_net_manager.create_udp_socket(ip, static_cast<uint16_t>(port));
    if (socket_idx < 0) {
        if (uart_ops) uart_ops->puts("Failed to create socket\n");
        return 1;
    }
    if (uart_ops) {
        uart_ops->puts("Socket created: index=");
        uart_ops->uart_put_uint64_hex(socket_idx);
        uart_ops->puts("\n");
    }
    return 0;
}

int cli_net_audio_stream(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: net audio stream <ip> <port> <channels>\n");
        return 1;
    }
    char ip_str[16];
    int port, channels;
    if (std::sscanf(args, "%15s %d %d", ip_str, &port, &channels) != 3 || port < 0 || port > 65535 || channels < 1 || channels > 8) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    uint32_t ip_addr;
    if (!util::ipv4_to_uint32(ip_str, ip_addr)) {
        if (uart_ops) uart_ops->puts("Invalid IP address\n");
        return 1;
    }
    net::IPv4Addr ip{.addr = ip_addr};
    static int socket_idx = 0; // Simplified: reuse socket
    socket_idx = net::g_net_manager.create_udp_socket({0x0A000001}, 1234 + socket_idx); // 10.0.0.1
    if (socket_idx < 0) {
        if (uart_ops) uart_ops->puts("Failed to create audio stream socket\n");
        return 1;
    }
    net::Packet packet{.dst_ip = ip, .dst_port = static_cast<uint16_t>(port), .data_len = 0, .audio_channels = static_cast<uint8_t>(channels)};
    if (net::g_net_manager.send(socket_idx, packet, true)) {
        if (uart_ops) uart_ops->puts("Audio stream started\n");
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to start audio stream\n");
    return 1;
}

int cli_net_priority(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        uart_ops->puts("Usage: net priority <socket_idx> <priority>\n");
        return 1;
    }
    int socket_idx;
    uint32_t priority;
    if (std::sscanf(args, "%d %u", &socket_idx, &priority) != 2 || priority > 15) {
        uart_ops->puts("Invalid arguments: socket_idx or priority (0-15)\n");
        return 1;
    }
    if (net::g_net_manager.set_socket_priority(socket_idx, static_cast<uint8_t>(priority))) {
        uart_ops->puts("Socket priority set\n");
        return 0;
    }
    uart_ops->puts("Failed to set socket priority\n");
    return 1;
}

int cli_net_jitter(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        uart_ops->puts("Usage: net jitter <socket_idx> <size>\n");
        return 1;
    }
    int socket_idx;
    uint32_t size;
    if (std::sscanf(args, "%d %u", &socket_idx, &size) != 2 || size > MAX_JITTER_BUFFER) {
        uart_ops->puts("Invalid arguments: socket_idx or size (0-8)\n");
        return 1;
    }
    if (net::g_net_manager.set_jitter_buffer_size(socket_idx, size)) {
        uart_ops->puts("Jitter buffer size set\n");
        return 0;
    }
    uart_ops->puts("Failed to set jitter buffer size\n");
    return 1;
}

int cli_dsp_add(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: dsp add <gain|mixer|eq|limiter|gate|stereo|waveshaper|env|compressor|delay|reverb|chorus|flanger|pitch|conv|crossover|phase|src|fir|iir|ffteq|netaudio> <name>\n");
        return 1;
    }
    char type[16], name[32];
    if (std::sscanf(args, "%15s %31s", type, name) != 2) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    kernel::DSPNode* node = nullptr;
    if (util::strcmp(type, "gain") == 0) {
        node = new kernel::GainDSP(1.0f, name);
    } else if (util::strcmp(type, "mixer") == 0) {
        node = new kernel::MixerDSP(name, 2);
    } else if (util::strcmp(type, "eq") == 0) {
        node = new kernel::ParametricEQDSP(name);
    } else if (util::strcmp(type, "limiter") == 0) {
        node = new kernel::LimiterDSP(name);
    } else if (util::strcmp(type, "gate") == 0) {
        node = new kernel::NoiseGateDSP(name);
    } else if (util::strcmp(type, "stereo") == 0) {
        node = new kernel::StereoWidthDSP(name);
    } else if (util::strcmp(type, "waveshaper") == 0) {
        node = new kernel::WaveshaperDSP(name);
    } else if (util::strcmp(type, "env") == 0) {
        node = new kernel::EnvelopeFollowerDSP(name);
    } else if (util::strcmp(type, "compressor") == 0) {
        node = new kernel::CompressorDSP(name);
    } else if (util::strcmp(type, "delay") == 0) {
        node = new kernel::DelayDSP(name);
    } else if (util::strcmp(type, "reverb") == 0) {
        node = new kernel::ReverbDSP(name);
    } else if (util::strcmp(type, "chorus") == 0) {
        node = new kernel::ChorusDSP(name);
    } else if (util::strcmp(type, "flanger") == 0) {
        node = new kernel::FlangerDSP(name);
    } else if (util::strcmp(type, "pitch") == 0) {
        node = new kernel::PitchShifterDSP(name);
    } else if (util::strcmp(type, "conv") == 0) {
        node = new kernel::ConvolutionReverbDSP(name);
    } else if (util::strcmp(type, "crossover") == 0) {
        node = new kernel::CrossoverDSP(name);
    } else if (util::strcmp(type, "phase") == 0) {
        node = new kernel::PhaseCorrectionDSP(name);
    } else if (util::strcmp(type, "src") == 0) {
        node = new kernel::SRCDSP(name);
    } else if (util::strcmp(type, "fir") == 0) {
        node = new kernel::FIRFilterDSP(name);
    } else if (util::strcmp(type, "iir") == 0) {
        node = new kernel::IIRFilterDSP(name);
    } else if (util::strcmp(type, "ffteq") == 0) {
        node = new kernel::FFTEqualizerDSP(name);
    } else if (util::strcmp(type, "netaudio") == 0) {
        node = new kernel::NetworkAudioSinkSource(name);
    } else {
        if (uart_ops) uart_ops->puts("Invalid DSP node type\n");
        return 1;
    }
    if (audio::g_audio_system.dsp_graph.add_node(node)) {
        if (uart_ops) {
            uart_ops->puts("Added node ");
            uart_ops->puts(name);
            uart_ops->puts("\n");
        }
        return 0;
    }
    delete node;
    if (uart_ops) uart_ops->puts("Failed to add node\n");
    return 1;
}

int cli_dsp_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: dsp config <name> <args>\n");
        return 1;
    }
    char name[32], config_args[128];
    if (std::sscanf(args, "%31s %127[^\n]", name, config_args) < 1) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (audio::g_audio_system.dsp_graph.configure_node(name, config_args, uart_ops)) {
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to configure node\n");
    return 1;
}

int cli_trace_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: trace <enable|disable|dump|clear>\n");
        return 1;
    }
    if (util::strcmp(args, "enable") == 0) {
        trace::g_trace_manager.set_enabled(true);
        if (uart_ops) uart_ops->puts("Tracing enabled\n");
    } else if (util::strcmp(args, "disable") == 0) {
        trace::g_trace_manager.set_enabled(false);
        if (uart_ops) uart_ops->puts("Tracing disabled\n");
    } else if (util::strcmp(args, "dump") == 0) {
        trace::g_trace_manager.dump_trace(uart_ops);
    } else if (util::strcmp(args, "clear") == 0) {
        trace::g_trace_manager.clear_trace();
        if (uart_ops) uart_ops->puts("Trace buffer cleared\n");
    } else {
        if (uart_ops) uart_ops->puts("Invalid trace command\n");
        return 1;
    }
    return 0;
}

int cli_fs_create(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: fs create <path> [is_dir]\n");
        return 1;
    }
    char path[64];
    int is_dir = 0;
    if (std::sscanf(args, "%63s %d", path, &is_dir) < 1) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (fs::g_file_system.create_file(path, is_dir != 0)) {
        if (uart_ops) uart_ops->puts("File/directory created\n");
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to create file/directory\n");
    return 1;
}

int cli_fs_list(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!args || !*args || !uart_ops) {
        if (uart_ops) uart_ops->puts("Usage: fs ls <path>\n");
        return 1;
    }
    char path[64];
    if (std::sscanf(args, "%63s", path) != 1) {
        if (uart_ops) uart_ops->puts("Invalid arguments\n");
        return 1;
    }
    if (fs::g_file_system.list_files(path, uart_ops)) {
        return 0;
    }
    if (uart_ops) uart_ops->puts("Failed to list files\n");
    return 1;
}

void CLI::register_core_commands() {
    register_command("help", cli_help_command, "List all commands or show help for a specific command");
    register_command("script", cli_script_command, "Execute script from RAMFS file");
    register_command("gpio config", cli_gpio_config, "Configure GPIO pin (input/output)");
    register_command("gpio read", cli_gpio_read, "Read GPIO pin state");
    register_command("gpio write", cli_gpio_write, "Write GPIO pin state");
    register_command("net create", cli_net_create, "Create UDP/TCP socket");
    register_command("net audio stream", cli_net_audio_stream, "Start audio streaming");
	register_command("net priority", cli_net_priority, "Set socket priority (0-15)");
    register_command("net jitter", cli_net_jitter, "Set audio jitter buffer size (0-8)");
    register_command("dsp add", cli_dsp_add, "Add DSP node (gain, mixer, eq, etc.)");
    register_command("dsp config", cli_dsp_config, "Configure DSP node");
    register_command("trace", cli_trace_command, "Control tracing (enable, disable, dump, clear)");
    register_command("fs create", cli_fs_create, "Create file or directory");
    register_command("fs ls", cli_fs_list, "List files in directory");
}

} // namespace cli