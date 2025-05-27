// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.cpp
 * @brief Command Line Interface (CLI) subsystem implementation for miniOS v1.7.
 * @details
 * Implements the CLI functionality, including command parsing, execution, history,
 * and integration with various miniOS subsystems like FS, Net, GPIO, DSP, Audio, and Trace.
 *
 * New in v1.7:
 * - Renamed from cli_v1.6.cpp
 * - Enhanced error handling and argument parsing robustness
 * - Improved Doxygen comments
 * - Refactored for clarity and adherence to C++20 best practices
 *
 * @version 1.7
 * @see cli.hpp, miniOS.hpp, util.hpp
 */

#include "miniOS.hpp" // Include the core kernel header first (defines GPIO_BANKS, MAX_NAME_LENGTH etc. globally)
#include "util.hpp"   // For kernel::util functions

#include "cli.hpp"    // Then its own header
#include "fs.hpp"     // For FileSystem commands
#include "net.hpp"    // For Network commands and net::MAX_JITTER_BUFFER
#include "gpio.hpp"   // For GPIO commands
#include "dsp.hpp"    // For DSP Node types (kernel::dsp::)
#include "audio.hpp"  // For kernel::g_audio_system
#include "trace.hpp"  // For trace commands
#include <cstring>    // For std::sscanf, std::strlen, etc. (often brought by util.hpp)
#include <cstdio>     // For std::snprintf, std::sscanf
#include <memory>     // For std::unique_ptr

// kernel::g_platform, kernel::g_scheduler_ptr etc are available via miniOS.hpp

namespace cli {

// Static member definitions
std::array<Command, MAX_COMMANDS> CLI::g_commands; // Command and MAX_COMMANDS are in 'cli' namespace
std::atomic<size_t> CLI::g_num_commands{0};

// Definitions for the static CLI state members
std::array<char, MAX_COMMAND_LENGTH> CLI::cmd_buffer_;
size_t CLI::cmd_buffer_idx_ = 0;
std::array<std::array<char, MAX_COMMAND_LENGTH>, MAX_HISTORY> CLI::command_history_;
size_t CLI::history_idx_ = 0;
size_t CLI::history_count_ = 0;


// Built-in command handlers (declarations moved to be static within the file)
static int cli_help_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_echo_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_clear_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_history_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_reboot_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_kernel_stats_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_trace_dump_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);

// Filesystem commands
static int cli_fs_ls_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_fs_cat_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_fs_mkfile_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_fs_mkdir_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_fs_rm_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);

// Network commands
static int cli_net_ifconfig_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_ping_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_listen_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_send_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_close_socket_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_sockets_command(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_audio_stream(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_net_jitter(const char* args, kernel::hal::UARTDriverOps* uart_ops);


// GPIO Commands
static int cli_gpio_config(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_gpio_read(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_gpio_write(const char* args, kernel::hal::UARTDriverOps* uart_ops);

// DSP Commands
static int cli_dsp_add(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_dsp_remove(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_dsp_config(const char* args, kernel::hal::UARTDriverOps* uart_ops);
static int cli_dsp_reset(const char* args, kernel::hal::UARTDriverOps* uart_ops);


void CLI::init(kernel::hal::UARTDriverOps* uart_ops) {
    (void)uart_ops; // uart_ops might be used for initial greeting or error reporting
    cmd_buffer_idx_ = 0;
    history_idx_ = 0;
    history_count_ = 0;
    g_num_commands.store(0, std::memory_order_relaxed); // Initialize atomic

    // Register built-in commands
    register_command("help", cli_help_command, "Show this help message or command details. Usage: help [command]");
    register_command("echo", cli_echo_command, "Print arguments back to the console. Usage: echo <text>");
    register_command("clear", cli_clear_command, "Clear the console screen.");
    register_command("history", cli_history_command, "Show command history.");
    register_command("reboot", cli_reboot_command, "Reboot the system (platform dependent).");
    register_command("kstats", cli_kernel_stats_command, "Show kernel statistics.");
    register_command("trace", cli_trace_dump_command, "Dump trace buffer.");

    // FS commands
    register_command("ls", cli_fs_ls_command, "List files in a directory. Usage: ls [path]");
    register_command("cat", cli_fs_cat_command, "Display file content. Usage: cat <filepath>");
    register_command("mkfile", cli_fs_mkfile_command, "Create an empty file. Usage: mkfile <filepath>");
    register_command("mkdir", cli_fs_mkdir_command, "Create a directory. Usage: mkdir <path>");
    register_command("rm", cli_fs_rm_command, "Remove a file or directory. Usage: rm <path>");

    // Network commands
    register_command("ifconfig", cli_net_ifconfig_command, "Show network interface configuration.");
    register_command("ping", cli_net_ping_command, "Send ICMP echo request. Usage: ping <ip_address>");
    register_command("listen_udp", cli_net_listen_udp_command, "Listen on a UDP port. Usage: listen_udp <port>");
    register_command("send_udp", cli_net_send_udp_command, "Send UDP packet. Usage: send_udp <ip> <port> <data>");
    register_command("close_sock", cli_net_close_socket_command, "Close a network socket. Usage: close_sock <socket_idx>");
    register_command("sockets", cli_net_sockets_command, "List active network sockets.");
    register_command("netaudio", cli_net_audio_stream, "Stream audio over network. Usage: netaudio <sink|source> <ip> <port> <channels>");
    register_command("jitter", cli_net_jitter, "Configure jitter buffer. Usage: jitter <socket_idx> <size_pkts>");


    // GPIO commands
    register_command("gpiocfg", cli_gpio_config, "Configure GPIO pin. Usage: gpiocfg <bank> <pin> <input|output>");
    register_command("gpioread", cli_gpio_read, "Read GPIO pin state. Usage: gpioread <bank> <pin>");
    register_command("gpiowrite", cli_gpio_write, "Write GPIO pin state. Usage: gpiowrite <bank> <pin> <high|low>");

    // DSP commands
    register_command("dspadd", cli_dsp_add, "Add DSP node. Usage: dspadd <type> <name>");
    register_command("dsprm", cli_dsp_remove, "Remove DSP node. Usage: dsprm <name>");
    register_command("dspcfg", cli_dsp_config, "Configure DSP node. Usage: dspcfg <name> <args...>");
    register_command("dspreset", cli_dsp_reset, "Reset DSP graph.");

    // Add more commands for other subsystems (Audio, Trace, etc.) as needed
}

bool CLI::register_command(const char* name, CommandHandler handler, const char* help_text) {
    size_t current_num = g_num_commands.load(std::memory_order_relaxed);
    if (current_num >= MAX_COMMANDS) {
        return false; // Max commands reached
    }
    g_commands[current_num] = {name, handler, help_text};
    g_num_commands.store(current_num + 1, std::memory_order_release);
    return true;
}

bool CLI::process_line(std::string_view line, kernel::hal::UARTDriverOps* uart_ops) {
    if (line.empty() || !uart_ops) {
        return false;
    }

    size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return false; 
    size_t last = line.find_last_not_of(" \t\r\n");
    line = line.substr(first, (last - first + 1));

    if (line.empty()) return false;

    size_t space_pos = line.find(' ');
    std::string_view command_name_sv;
    const char* args_ptr = nullptr; 

    if (space_pos == std::string_view::npos) { 
        command_name_sv = line;
    } else {
        command_name_sv = line.substr(0, space_pos);
        if (space_pos + 1 < line.length()) {
             args_ptr = line.data() + space_pos + 1;
        }
    }
    
    std::string command_name_str(command_name_sv); 

    size_t current_num_cmds = g_num_commands.load(std::memory_order_relaxed);
    for (size_t i = 0; i < current_num_cmds; ++i) {
        if (g_commands[i].name && command_name_str == g_commands[i].name) {
            g_commands[i].handler(args_ptr, uart_ops); 
            return true;
        }
    }

    uart_ops->puts("Unknown command: ");
    uart_ops->puts(command_name_str.c_str());
    uart_ops->puts("\n");
    return true; 
}

void CLI::cli_thread_entry(void* uart_ops_ptr) {
    if (!uart_ops_ptr) return; 
    kernel::hal::UARTDriverOps* uart_ops = static_cast<kernel::hal::UARTDriverOps*>(uart_ops_ptr);

    CLI::init(uart_ops); 

    uart_ops->puts("\n[miniOS CLI v1.7]\nType 'help' for available commands.\n");
    
    cmd_buffer_idx_ = 0; 
    cmd_buffer_.fill('\0'); 

    while (true) {
        uart_ops->puts("mOS> ");
        char c = uart_ops->getc_blocking(); 

        if (c == '\r' || c == '\n') { 
            uart_ops->putc('\n'); 
            if (cmd_buffer_idx_ > 0) {
                cmd_buffer_[cmd_buffer_idx_] = '\0'; 

                if (history_count_ < MAX_HISTORY) {
                    kernel::util::memcpy(command_history_[history_count_].data(), cmd_buffer_.data(), cmd_buffer_idx_ + 1);
                    history_count_++;
                } else { 
                    for (size_t i = 1; i < MAX_HISTORY; ++i) {
                        command_history_[i - 1] = command_history_[i];
                    }
                    kernel::util::memcpy(command_history_[MAX_HISTORY - 1].data(), cmd_buffer_.data(), cmd_buffer_idx_ + 1);
                }
                history_idx_ = history_count_; 

                std::string_view cmd_line_sv(cmd_buffer_.data(), cmd_buffer_idx_);
                process_line(cmd_line_sv, uart_ops);
                cmd_buffer_idx_ = 0; 
                cmd_buffer_.fill('\0');
            }
        } else if (c == '\b' || c == 127) { 
            if (cmd_buffer_idx_ > 0) {
                cmd_buffer_idx_--;
                cmd_buffer_[cmd_buffer_idx_] = '\0'; 
                uart_ops->puts("\b \b"); 
            }
        } else if (c == '\t') { 
            // uart_ops->puts("[TAB COMPLETION TODO]");
        } else if (c >= ' ' && c <= '~') { 
            if (cmd_buffer_idx_ < MAX_COMMAND_LENGTH - 1) {
                cmd_buffer_[cmd_buffer_idx_++] = c;
                uart_ops->putc(c); 
            }
        }
    }
}


// --- Built-in Command Implementations ---
static int cli_help_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return -1;
    if (args && kernel::util::strlen(args) > 0) {
        for (size_t i = 0; i < CLI::g_num_commands.load(std::memory_order_relaxed); ++i) {
            if (CLI::g_commands[i].name && kernel::util::strcmp(CLI::g_commands[i].name, args) == 0) {
                uart_ops->puts(args); 
                if (CLI::g_commands[i].help_text) {
                    uart_ops->puts(": ");
                    uart_ops->puts(CLI::g_commands[i].help_text);
                }
                uart_ops->putc('\n');
                return 0;
            }
        }
        uart_ops->puts("Command not found: "); uart_ops->puts(args); uart_ops->putc('\n');
    } else {
        uart_ops->puts("Available commands:\n");
        for (size_t i = 0; i < CLI::g_num_commands.load(std::memory_order_relaxed); ++i) {
            uart_ops->puts("  ");
            if (CLI::g_commands[i].name) uart_ops->puts(CLI::g_commands[i].name);
            if (CLI::g_commands[i].help_text) {
                uart_ops->puts(" - ");
                uart_ops->puts(CLI::g_commands[i].help_text);
            }
            uart_ops->putc('\n');
        }
    }
    return 0;
}

static int cli_echo_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return -1;
    if (args) {
        uart_ops->puts(args);
    }
    uart_ops->putc('\n');
    return 0;
}

static int cli_clear_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args; 
    if (!uart_ops) return -1;
    uart_ops->puts("\033[2J\033[H"); 
    return 0;
}

static int cli_history_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args; 
    if (!uart_ops) return -1;
    uart_ops->puts("Command History:\n");
    for (size_t i = 0; i < CLI::history_count_; ++i) {
        char num_buf[8];
        std::snprintf(num_buf, sizeof(num_buf), "%2zu: ", i + 1);
        uart_ops->puts(num_buf);
        uart_ops->puts(CLI::command_history_[i].data()); 
        uart_ops->putc('\n');
    }
    return 0;
}

// --- Filesystem Command Implementations ---
static int cli_fs_ls_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return -1;
    const char* path_to_list = (args && kernel::util::strlen(args) > 0) ? args : "/";
    if (!kernel::g_file_system.list_files(path_to_list, uart_ops)) {
        uart_ops->puts("Error listing directory.\n");
        return -1;
    }
    return 0;
}

// --- GPIO Command Implementations ---
static int cli_gpio_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) return -1;
    int bank = -1, pin = -1;
    char mode_str[10] = {0};
    kernel::hal::gpio::PinMode mode_val;

    if (std::sscanf(args, "%d %d %9s", &bank, &pin, mode_str) == 3) {
        if (bank < 0 || static_cast<size_t>(bank) >= GPIO_BANKS || pin < 0 || static_cast<size_t>(pin) >= GPIO_PINS_PER_BANK) {
            if (uart_ops) uart_ops->puts("Error: Invalid bank or pin number.\n");
            return -1;
        }
        std::string_view mode_sv(mode_str);
        if (mode_sv == "input") mode_val = kernel::hal::gpio::PinMode::INPUT;
        else if (mode_sv == "output") mode_val = kernel::hal::gpio::PinMode::OUTPUT;
        else {
            if (uart_ops) uart_ops->puts("Error: Invalid mode. Use 'input' or 'output'.\n");
            return -1;
        }

        if (kernel::g_gpio_manager.configure_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin), mode_val)) {
            if (uart_ops) uart_ops->puts("GPIO pin configured.\n");
        } else {
            if (uart_ops) uart_ops->puts("Error configuring GPIO pin.\n");
            return -1;
        }
    } else {
        if (uart_ops) uart_ops->puts("Usage: gpiocfg <bank> <pin> <input|output>\n");
        return -1;
    }
    return 0;
}

static int cli_gpio_read(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) return -1;
    int bank = -1, pin = -1;
    if (std::sscanf(args, "%d %d", &bank, &pin) == 2) {
        if (bank < 0 || static_cast<size_t>(bank) >= GPIO_BANKS || pin < 0 || static_cast<size_t>(pin) >= GPIO_PINS_PER_BANK) {
            if (uart_ops) uart_ops->puts("Error: Invalid bank or pin number.\n");
            return -1;
        }
        kernel::hal::gpio::PinState state = kernel::g_gpio_manager.read_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin));
        uart_ops->puts("GPIO["); uart_ops->puts(args); uart_ops->puts("] state: "); 
        uart_ops->puts(state == kernel::hal::gpio::PinState::HIGH ? "HIGH\n" : "LOW\n");
    } else {
        if (uart_ops) uart_ops->puts("Usage: gpioread <bank> <pin>\n");
        return -1;
    }
    return 0;
}

static int cli_gpio_write(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) return -1;
    int bank = -1, pin = -1;
    char state_str[5] = {0};
    kernel::hal::gpio::PinState state_val;

    if (std::sscanf(args, "%d %d %4s", &bank, &pin, state_str) == 3) {
        if (bank < 0 || static_cast<size_t>(bank) >= GPIO_BANKS || pin < 0 || static_cast<size_t>(pin) >= GPIO_PINS_PER_BANK) {
            if (uart_ops) uart_ops->puts("Error: Invalid bank or pin number.\n");
            return -1;
        }
        std::string_view state_sv(state_str);
        if (state_sv == "high") state_val = kernel::hal::gpio::PinState::HIGH;
        else if (state_sv == "low") state_val = kernel::hal::gpio::PinState::LOW;
        else {
            if (uart_ops) uart_ops->puts("Error: Invalid state. Use 'high' or 'low'.\n");
            return -1;
        }
        if (kernel::g_gpio_manager.set_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin), state_val)) {
            if (uart_ops) uart_ops->puts("GPIO pin set.\n");
        } else {
            if (uart_ops) uart_ops->puts("Error setting GPIO pin.\n");
            return -1;
        }
    } else {
        if (uart_ops) uart_ops->puts("Usage: gpiowrite <bank> <pin> <high|low>\n");
        return -1;
    }
    return 0;
}


// --- Network Command Implementations ---
static int cli_net_ifconfig_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args; 
    if (!uart_ops) return -1;
    kernel::g_net_manager.print_if_config(0, uart_ops); 
    return 0;
}

static int cli_net_ping_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) {
        if (uart_ops) uart_ops->puts("Usage: ping <ip_address>\n");
        return -1;
    }
    uint32_t ip_addr_val;
    if (kernel::util::ipv4_to_uint32(args, ip_addr_val)) {
        net::IPv4Addr target_ip{ip_addr_val};
        kernel::g_net_manager.ping(target_ip, 3, uart_ops); 
    } else {
        uart_ops->puts("Invalid IP address format.\n");
    }
    return 0;
}

static int cli_net_audio_stream(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args) {
        if(uart_ops) uart_ops->puts("Usage: netaudio <sink|source> <ip> <port> <channels>\n");
        return -1;
    }
    char mode[10], ip_str[16];
    int port, channels;
    if (std::sscanf(args, "%9s %15s %d %d", mode, ip_str, &port, &channels) == 4) {
        uart_ops->puts("Network audio streaming configured (simulated): Mode=");
        uart_ops->puts(mode); uart_ops->puts(", IP="); uart_ops->puts(ip_str);
        // ... print port and channels
        uart_ops->puts("\nTo use, add and configure 'netaudio' DSP node.\n");

        if (kernel::util::strcmp(mode, "sink") == 0 && kernel::g_net_manager.is_initialized()) {
            uint32_t ip_val;
            if (kernel::util::ipv4_to_uint32(ip_str, ip_val) && port > 0 && port <= 65535 && channels > 0) {
                net::IPv4Addr ip{ip_val};
                net::Packet packet{.dst_ip = ip, .dst_port = static_cast<uint16_t>(port), .src_ip={}, .src_port=0, .timestamp_us=0, .data={}, .data_len = 0, .priority=0, .audio_channels = static_cast<uint8_t>(channels)};
                packet.data_len = snprintf(reinterpret_cast<char*>(packet.data.data()), packet.data.size(), "TestNetAudio");
                uart_ops->puts("Simulated packet prepared. Use dsp node for actual streaming.\n");
            }
        }

    } else {
        uart_ops->puts("Invalid arguments for netaudio.\n");
    }
    return 0;
}

static int cli_net_jitter(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) return -1;
    int socket_idx = -1;
    unsigned int size = 0; 

    if (args && std::sscanf(args, "%d %u", &socket_idx, &size) == 2 &&
        socket_idx >= 0 && size <= net::MAX_JITTER_BUFFER) { // Allow size 0 to disable
        if (kernel::g_net_manager.set_jitter_buffer_size(socket_idx, size)) {
            if (uart_ops) uart_ops->puts("Jitter buffer size set.\n");
        } else {
            if (uart_ops) uart_ops->puts("Error setting jitter buffer (invalid socket or manager not init).\n");
        }
    } else {
        if (uart_ops) uart_ops->puts("Usage: jitter <socket_idx> <size_packets (0-8)>\n");
    }
    return 0;
}


// --- DSP Command Implementations ---
static int cli_dsp_add(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) {
        if (uart_ops) uart_ops->puts("Usage: dspadd <type> <name>\n");
        return -1;
    }
    char type_str[32] = {0};
    char name_str[MAX_NAME_LENGTH] = {0}; // Use global MAX_NAME_LENGTH

    if (std::sscanf(args, "%31s %31s", type_str, name_str) == 2) {
        std::string_view type_sv(type_str);
        std::string_view name_sv(name_str);
        
        std::unique_ptr<kernel::dsp::DSPNode> node_ptr = nullptr;

        if (type_sv == "gain") node_ptr = std::make_unique<kernel::dsp::GainDSP>(1.0f, name_sv);
        else if (type_sv == "mixer") node_ptr = std::make_unique<kernel::dsp::MixerDSP>(name_sv, 2);
        else if (type_sv == "eq") node_ptr = std::make_unique<kernel::dsp::ParametricEQDSP>(name_sv);
        else if (type_sv == "limiter") node_ptr = std::make_unique<kernel::dsp::LimiterDSP>(name_sv);
        else if (type_sv == "gate") node_ptr = std::make_unique<kernel::dsp::NoiseGateDSP>(name_sv);
        else if (type_sv == "stereo") node_ptr = std::make_unique<kernel::dsp::StereoWidthDSP>(name_sv);
        else if (type_sv == "shaper") node_ptr = std::make_unique<kernel::dsp::WaveshaperDSP>(name_sv);
        else if (type_sv == "envfollower") node_ptr = std::make_unique<kernel::dsp::EnvelopeFollowerDSP>(name_sv);
        else if (type_sv == "compressor") node_ptr = std::make_unique<kernel::dsp::CompressorDSP>(name_sv);
        else if (type_sv == "delay") node_ptr = std::make_unique<kernel::dsp::DelayDSP>(name_sv);
        else if (type_sv == "reverb") node_ptr = std::make_unique<kernel::dsp::ReverbDSP>(name_sv);
        else if (type_sv == "chorus") node_ptr = std::make_unique<kernel::dsp::ChorusDSP>(name_sv);
        else if (type_sv == "flanger") node_ptr = std::make_unique<kernel::dsp::FlangerDSP>(name_sv);
        else if (type_sv == "pitchshift") node_ptr = std::make_unique<kernel::dsp::PitchShifterDSP>(name_sv);
        else if (type_sv == "convreverb") node_ptr = std::make_unique<kernel::dsp::ConvolutionReverbDSP>(name_sv);
        else if (type_sv == "crossover") node_ptr = std::make_unique<kernel::dsp::CrossoverDSP>(name_sv);
        else if (type_sv == "phasecorr") node_ptr = std::make_unique<kernel::dsp::PhaseCorrectionDSP>(name_sv);
        else if (type_sv == "src") node_ptr = std::make_unique<kernel::dsp::SRCDSP>(name_sv);
        else if (type_sv == "fir") node_ptr = std::make_unique<kernel::dsp::FIRFilterDSP>(name_sv);
        else if (type_sv == "iir") node_ptr = std::make_unique<kernel::dsp::IIRFilterDSP>(name_sv);
        else if (type_sv == "ffteq") node_ptr = std::make_unique<kernel::dsp::FFTEqualizerDSP>(name_sv);
        else if (type_sv == "netaudio") node_ptr = std::make_unique<kernel::dsp::NetworkAudioSinkSource>(name_sv);
        else {
            if (uart_ops) {
                uart_ops->puts("Unknown DSP node type: ");
                uart_ops->puts(type_str);
                uart_ops->puts("\n");
            }
            return -1;
        }

        if (node_ptr) {
            if (kernel::g_audio_system.get_dsp_graph().add_node(std::move(node_ptr))) {
                if (uart_ops) {
                    uart_ops->puts("DSP node added: ");
                    uart_ops->puts(name_str);
                    uart_ops->puts("\n");
                }
            } else {
                if (uart_ops) uart_ops->puts("Failed to add DSP node (already exists or error).\n");
            }
        }
    } else {
        if (uart_ops) uart_ops->puts("Usage: dspadd <type> <name>\n");
        return -1;
    }
    return 0;
}

static int cli_dsp_remove(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args || kernel::util::strlen(args) == 0) {
        if (uart_ops) uart_ops->puts("Usage: dsprm <name>\n");
        return -1;
    }
    std::string_view name_sv(args);
    if (kernel::g_audio_system.get_dsp_graph().remove_node(name_sv)) {
        if (uart_ops) {
            uart_ops->puts("DSP node removed: ");
            uart_ops->puts(args);
            uart_ops->puts("\n");
        }
    } else {
        if (uart_ops) {
            uart_ops->puts("DSP node not found: ");
            uart_ops->puts(args);
            uart_ops->puts("\n");
        }
        return -1;
    }
    return 0;
}

static int cli_dsp_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args) {
        if (uart_ops) uart_ops->puts("Usage: dspcfg <name> <node_specific_args...>\n");
        return -1;
    }
    std::string_view args_sv(args);
    size_t first_space = args_sv.find(' ');
    
    std::string_view name_to_config_sv;
    const char* config_args_for_node_ptr = nullptr; 

    if (first_space == std::string_view::npos) { 
        name_to_config_sv = args_sv;
    } else {
        name_to_config_sv = args_sv.substr(0, first_space);
        if (first_space + 1 < args_sv.length()) {
            config_args_for_node_ptr = args + first_space + 1;
        }
    }
    
    std::string name_to_config_cstr(name_to_config_sv); 

    if (kernel::g_audio_system.get_dsp_graph().configure_node(name_to_config_cstr.c_str(), config_args_for_node_ptr, uart_ops)) {
        // Silence, configure_node should provide feedback.
    } else {
        // Silence, configure_node should provide feedback.
    }
    return 0;
}

static int cli_dsp_reset(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args; 
    if (!uart_ops) {
        return -1;
    }
    kernel::g_audio_system.get_dsp_graph().reset();
    if (uart_ops) {
        uart_ops->puts("DSP graph reset.\n");
    }
    return 0;
}

// --- Other Command Stubs ---
static int cli_reboot_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args;
    if (uart_ops) uart_ops->puts("Rebooting system...\n");
    if (kernel::g_platform) {
        // kernel::g_platform->reboot(); // Assuming a reboot method exists in Platform HAL
    }
    return 0;
}
static int cli_kernel_stats_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args;
    kernel::get_kernel_stats(uart_ops); 
    return 0;
}
static int cli_trace_dump_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args;
    kernel::dump_trace_buffer(uart_ops); 
    return 0;
}

static int cli_fs_cat_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops || !args || kernel::util::strlen(args) == 0) {
        if(uart_ops) { uart_ops->puts("Usage: cat <filepath>\n"); }
        return -1;
    }
    std::string content;
    // Assuming FileSystem::read_file(std::string_view, std::string&) overload exists
    if (kernel::g_file_system.read_file(args, content)) {
        uart_ops->puts(content.c_str());
        uart_ops->putc('\n');
    } else {
        uart_ops->puts("Error reading file or file not found.\n");
    }
    return 0;
}

static int cli_fs_mkfile_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args || kernel::util::strlen(args) == 0) { if(uart_ops) { uart_ops->puts("Usage: mkfile <filepath>\n"); } return -1; }
    if(kernel::g_file_system.create_file(args, false)) uart_ops->puts("File created.\n");
    else uart_ops->puts("Error creating file.\n");
    return 0;
}
static int cli_fs_mkdir_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args || kernel::util::strlen(args) == 0) { if(uart_ops) { uart_ops->puts("Usage: mkdir <path>\n"); } return -1; }
    if(kernel::g_file_system.create_file(args, true)) uart_ops->puts("Directory created.\n"); 
    else uart_ops->puts("Error creating directory.\n");
    return 0;
}
static int cli_fs_rm_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args || kernel::util::strlen(args) == 0) { if(uart_ops) { uart_ops->puts("Usage: rm <path>\n"); } return -1; }
    if(kernel::g_file_system.delete_file(args)) uart_ops->puts("File/directory removed.\n");
    else uart_ops->puts("Error removing file/directory.\n");
    return 0;
}

static int cli_net_listen_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args) { if(uart_ops) uart_ops->puts("Usage: listen_udp <port>\n"); return -1; }
    int port;
    if(std::sscanf(args, "%d", &port) == 1 && port > 0 && port <= 65535) {
        int sock_idx = kernel::g_net_manager.create_udp_socket(net::IPv4Addr{0}, static_cast<uint16_t>(port)); 
        if(sock_idx >= 0) { char b[64]; std::snprintf(b,sizeof(b),"Listening on UDP port %d, socket %d\n", port, sock_idx); uart_ops->puts(b); }
        else uart_ops->puts("Error creating UDP socket.\n");
    } else uart_ops->puts("Invalid port.\n");
    return 0;
}
static int cli_net_send_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args) { if(uart_ops) uart_ops->puts("Usage: send_udp <ip> <port> <data>\n"); return -1;}
    char ip_str[16]; int port; char data_str[128]; 
    if(std::sscanf(args, "%15s %d %127s", ip_str, &port, data_str) == 3) {
        uint32_t ip_val;
        if(kernel::util::ipv4_to_uint32(ip_str, ip_val) && port > 0 && port <= 65535) {
            net::IPv4Addr dst_ip{ip_val};
            net::Packet p;
            p.dst_ip = dst_ip;
            p.dst_port = static_cast<uint16_t>(port);
            p.data_len = kernel::util::strlen(data_str);
            if (p.data_len > p.data.size()) p.data_len = p.data.size(); // Prevent overflow
            kernel::util::memcpy(p.data.data(), data_str, p.data_len);
            
            int sock_idx = kernel::g_net_manager.create_udp_socket(net::IPv4Addr{0}, 0); 
            if (sock_idx >= 0) {
                if(kernel::g_net_manager.send(sock_idx, p, true)) uart_ops->puts("UDP packet sent.\n");
                else uart_ops->puts("Error sending UDP packet.\n");
                kernel::g_net_manager.close_socket(sock_idx);
            } else uart_ops->puts("Error creating send socket.\n");
        } else uart_ops->puts("Invalid IP or port.\n");
    } else uart_ops->puts("Usage: send_udp <ip> <port> <data>\n");
    return 0;
}
static int cli_net_close_socket_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if(!uart_ops || !args) { if(uart_ops) uart_ops->puts("Usage: close_sock <socket_idx>\n"); return -1; }
    int sock_idx;
    if(std::sscanf(args, "%d", &sock_idx) == 1 && sock_idx >= 0) {
        if(kernel::g_net_manager.close_socket(sock_idx)) uart_ops->puts("Socket closed.\n");
        else uart_ops->puts("Error closing socket or socket not found.\n");
    } else uart_ops->puts("Usage: close_sock <socket_idx>\n");
    return 0;
}
static int cli_net_sockets_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    (void)args;
    if(!uart_ops) return -1;
    kernel::g_net_manager.list_sockets(uart_ops);
    return 0;
}


bool CLI::process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops) {
    if (!filename || !uart_ops || !kernel::g_platform) return false;
    
    std::string script_content;
    if (!kernel::g_file_system.read_file(filename, script_content)) {
        uart_ops->puts("Error: Could not read script file '");
        uart_ops->puts(filename);
        uart_ops->puts("'.\n");
        return false;
    }

    uart_ops->puts("Executing script: "); uart_ops->puts(filename); uart_ops->puts("\n");

    std::string_view content_sv(script_content);
    size_t line_start = 0;
    while(line_start < content_sv.length()) {
        size_t line_end = content_sv.find('\n', line_start);
        std::string_view line_sv;
        if (line_end == std::string_view::npos) {
            line_sv = content_sv.substr(line_start);
            line_start = content_sv.length(); 
        } else {
            line_sv = content_sv.substr(line_start, line_end - line_start);
            line_start = line_end + 1;
        }

        size_t first_char = line_sv.find_first_not_of(" \t\r");
        if (first_char == std::string_view::npos || line_sv[first_char] == '#') {
            continue; 
        }
        line_sv = line_sv.substr(first_char);
        
        size_t last_char = line_sv.find_last_not_of(" \t\r");
         if (last_char != std::string_view::npos) {
            line_sv = line_sv.substr(0, last_char + 1);
        } else { // line was all whitespace after first_char
            continue;
        }


        if (!line_sv.empty()) {
            uart_ops->puts("scr> ");
            std::string temp_line(line_sv); 
            uart_ops->puts(temp_line.c_str());
            uart_ops->putc('\n');
            
            process_line(line_sv, uart_ops);
        }
    }
    uart_ops->puts("Script execution finished.\n");
    return true;
}


} // namespace cli