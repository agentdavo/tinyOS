// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cli.cpp
 * @brief CLI subsystem implementation for miniOS v1.7 full build.
 * @details
 * Implements command-line interface with subsystem commands (audio, dsp, fs, gpio, net)
 * for the full build (`make all`).
 *
 * @version 1.7
 * @see cli.hpp, core.hpp, hal.hpp, audio.hpp, dsp.hpp, fs.hpp, gpio.hpp, net.hpp
 */

#include "cli.hpp"
#include "core.hpp"
#include "hal.hpp"
#include "miniOS.hpp"
#include "audio.hpp"
#include "dsp.hpp"
#include "fs.hpp"
#include "gpio.hpp"
#include "net.hpp"
#include <cstring>
#include <memory>

namespace cli {

CLI g_cli;

void CLI::register_command(std::string_view name, CommandCallback callback) {
    // Stub implementation for full build
}

void CLI::process_command(std::string_view cmd, kernel::hal::UARTDriverOps* uart_ops) {
    if (!uart_ops) return;

    if (cmd == "trace") {
        kernel::dump_trace_buffer(uart_ops);
    } else if (cmd == "stats") {
        kernel::get_kernel_stats(uart_ops);
    } else if (cmd.starts_with("ls")) {
        cli_fs_ls_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("gpio_config")) {
        cli_gpio_config(cmd.data(), uart_ops);
    } else if (cmd.starts_with("gpio_read")) {
        cli_gpio_read(cmd.data(), uart_ops);
    } else if (cmd.starts_with("gpio_write")) {
        cli_gpio_write(cmd.data(), uart_ops);
    } else if (cmd.starts_with("ifconfig")) {
        cli_net_ifconfig_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("ping")) {
        cli_net_ping_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("stream")) {
        cli_net_audio_stream(cmd.data(), uart_ops);
    } else if (cmd.starts_with("jitter")) {
        cli_net_jitter(cmd.data(), uart_ops);
    } else if (cmd.starts_with("dsp_add")) {
        cli_dsp_add(cmd.data(), uart_ops);
    } else if (cmd.starts_with("dsp_remove")) {
        cli_dsp_remove(cmd.data(), uart_ops);
    } else if (cmd.starts_with("dsp_config")) {
        cli_dsp_config(cmd.data(), uart_ops);
    } else if (cmd.starts_with("dsp_reset")) {
        cli_dsp_reset(cmd.data(), uart_ops);
    } else if (cmd.starts_with("cat")) {
        cli_fs_cat_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("mkfile")) {
        cli_fs_mkfile_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("mkdir")) {
        cli_fs_mkdir_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("rm")) {
        cli_fs_rm_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("listen_udp")) {
        cli_net_listen_udp_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("send_udp")) {
        cli_net_send_udp_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("close_socket")) {
        cli_net_close_socket_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("sockets")) {
        cli_net_sockets_command(cmd.data(), uart_ops);
    } else if (cmd.starts_with("script")) {
        process_script(cmd.data(), uart_ops);
    } else {
        uart_ops->puts("Unknown command\n");
    }
}

int cli_fs_ls_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    std::string_view path_to_list = args + 3;
    if (!kernel::g_file_system.list_files(path_to_list, uart_ops)) {
        uart_ops->puts("Failed to list files\n");
        return -1;
    }
    return 0;
}

int cli_gpio_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int bank, pin;
    char mode[16];
    if (std::sscanf(args, "gpio_config %d %d %15s", &bank, &pin, mode) != 3) {
        uart_ops->puts("Usage: gpio_config <bank> <pin> <mode>\n");
        return -1;
    }

    if (bank < 0 || static_cast<size_t>(bank) >= kernel::core::GPIO_BANKS ||
        pin < 0 || static_cast<size_t>(pin) >= kernel::core::GPIO_PINS_PER_BANK) {
        uart_ops->puts("Invalid bank or pin\n");
        return -1;
    }

    kernel::hal::gpio::PinMode mode_val;
    if (std::strcmp(mode, "input") == 0) mode_val = kernel::hal::gpio::PinMode::INPUT;
    else if (std::strcmp(mode, "output") == 0) mode_val = kernel::hal::gpio::PinMode::OUTPUT;
    else {
        uart_ops->puts("Invalid mode: use input or output\n");
        return -1;
    }

    if (kernel::g_gpio_manager.configure_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin), mode_val)) {
        uart_ops->puts("GPIO configured\n");
        return 0;
    }
    uart_ops->puts("Failed to configure GPIO\n");
    return -1;
}

int cli_gpio_read(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int bank, pin;
    if (std::sscanf(args, "gpio_read %d %d", &bank, &pin) != 2) {
        uart_ops->puts("Usage: gpio_read <bank> <pin>\n");
        return -1;
    }

    if (bank < 0 || static_cast<size_t>(bank) >= kernel::core::GPIO_BANKS ||
        pin < 0 || static_cast<size_t>(pin) >= kernel::core::GPIO_PINS_PER_BANK) {
        uart_ops->puts("Invalid bank or pin\n");
        return -1;
    }

    kernel::hal::gpio::PinState state = kernel::g_gpio_manager.read_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin));
    char buf[32];
    std::snprintf(buf, sizeof(buf), "GPIO %d:%d = %s\n", bank, pin, state == kernel::hal::gpio::PinState::HIGH ? "HIGH" : "LOW");
    uart_ops->puts(buf);
    return 0;
}

int cli_gpio_write(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int bank, pin;
    char state[8];
    if (std::sscanf(args, "gpio_write %d %d %7s", &bank, &pin, state) != 3) {
        uart_ops->puts("Usage: gpio_write <bank> <pin> <state>\n");
        return -1;
    }

    if (bank < 0 || static_cast<size_t>(bank) >= kernel::core::GPIO_BANKS ||
        pin < 0 || static_cast<size_t>(pin) >= kernel::core::GPIO_PINS_PER_BANK) {
        uart_ops->puts("Invalid bank or pin\n");
        return -1;
    }

    kernel::hal::gpio::PinState state_val;
    if (std::strcmp(state, "high") == 0) state_val = kernel::hal::gpio::PinState::HIGH;
    else if (std::strcmp(state, "low") == 0) state_val = kernel::hal::gpio::PinState::LOW;
    else {
        uart_ops->puts("Invalid state: use high or low\n");
        return -1;
    }

    if (kernel::g_gpio_manager.set_pin(static_cast<uint32_t>(bank), static_cast<uint32_t>(pin), state_val)) {
        uart_ops->puts("GPIO set\n");
        return 0;
    }
    uart_ops->puts("Failed to set GPIO\n");
    return -1;
}

int cli_net_ifconfig_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    kernel::g_net_manager.print_if_config(0, uart_ops);
    return 0;
}

int cli_net_ping_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char target_ip[16];
    if (std::sscanf(args, "ping %15s", target_ip) != 1) {
        uart_ops->puts("Usage: ping <ip>\n");
        return -1;
    }
    kernel::g_net_manager.ping(target_ip, 3, uart_ops);
    return 0;
}

int cli_net_audio_stream(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char mode[16];
    if (std::sscanf(args, "stream %15s", mode) != 1) {
        uart_ops->puts("Usage: stream <source|sink>\n");
        return -1;
    }

    if (::kernel::util::strcmp(mode, "sink") == 0 && kernel::g_net_manager.is_initialized()) {
        uart_ops->puts("Network audio sink not implemented\n");
        return -1;
    } else if (::kernel::util::strcmp(mode, "source") == 0) {
        uart_ops->puts("Network audio source not implemented\n");
        return -1;
    } else {
        uart_ops->puts("Invalid mode: use source or sink\n");
        return -1;
    }
    return 0;
}

int cli_net_jitter(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int socket_idx, size;
    if (std::sscanf(args, "jitter %d %d", &socket_idx, &size) != 2) {
        uart_ops->puts("Usage: jitter <socket_idx> <size_ms>\n");
        return -1;
    }
    if (kernel::g_net_manager.set_jitter_buffer_size(socket_idx, size)) {
        uart_ops->puts("Jitter buffer size set\n");
        return 0;
    }
    uart_ops->puts("Failed to set jitter buffer size\n");
    return -1;
}

int cli_dsp_add(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char type_str[kernel::core::MAX_NAME_LENGTH] = {0};
    char name_str[kernel::core::MAX_NAME_LENGTH] = {0};
    if (std::sscanf(args, "dsp_add %31s %31s", type_str, name_str) != 2) {
        uart_ops->puts("Usage: dsp_add <type> <name>\n");
        return -1;
    }

    std::string_view type_sv = type_str;
    std::string_view name_sv = name_str;
    std::unique_ptr<dsp::DSPNode> node_ptr = nullptr;

    if (type_sv == "gain") node_ptr = std::make_unique<dsp::GainDSP>(1.0f, name_sv);
    else if (type_sv == "mixer") node_ptr = std::make_unique<dsp::MixerDSP>(name_sv, 2);
    else if (type_sv == "eq") node_ptr = std::make_unique<dsp::ParametricEQDSP>(name_sv);
    else if (type_sv == "limiter") node_ptr = std::make_unique<dsp::LimiterDSP>(name_sv);
    else if (type_sv == "gate") node_ptr = std::make_unique<dsp::NoiseGateDSP>(name_sv);
    else if (type_sv == "stereo") node_ptr = std::make_unique<dsp::StereoWidthDSP>(name_sv);
    else if (type_sv == "shaper") node_ptr = std::make_unique<dsp::WaveshaperDSP>(name_sv);
    else if (type_sv == "envfollower") node_ptr = std::make_unique<dsp::EnvelopeFollowerDSP>(name_sv);
    else if (type_sv == "compressor") node_ptr = std::make_unique<dsp::CompressorDSP>(name_sv);
    else if (type_sv == "delay") node_ptr = std::make_unique<dsp::DelayDSP>(name_sv);
    else if (type_sv == "reverb") node_ptr = std::make_unique<dsp::ReverbDSP>(name_sv);
    else if (type_sv == "chorus") node_ptr = std::make_unique<dsp::ChorusDSP>(name_sv);
    else if (type_sv == "flanger") node_ptr = std::make_unique<dsp::FlangerDSP>(name_sv);
    else if (type_sv == "pitchshift") node_ptr = std::make_unique<dsp::PitchShifterDSP>(name_sv);
    else if (type_sv == "convreverb") node_ptr = std::make_unique<dsp::ConvolutionReverbDSP>(name_sv);
    else if (type_sv == "crossover") node_ptr = std::make_unique<dsp::CrossoverDSP>(name_sv);
    else if (type_sv == "phasecorr") node_ptr = std::make_unique<dsp::PhaseCorrectionDSP>(name_sv);
    else if (type_sv == "src") node_ptr = std::make_unique<dsp::SRCDSP>(name_sv);
    else if (type_sv == "fir") node_ptr = std::make_unique<dsp::FIRFilterDSP>(name_sv);
    else if (type_sv == "iir") node_ptr = std::make_unique<dsp::IIRFilterDSP>(name_sv);
    else if (type_sv == "ffteq") node_ptr = std::make_unique<dsp::FFTEqualizerDSP>(name_sv);
    else if (type_sv == "netaudio") node_ptr = std::make_unique<dsp::NetworkAudioSinkSource>(name_sv);
    else {
        uart_ops->puts("Unknown DSP type\n");
        return -1;
    }

    if (!node_ptr) {
        uart_ops->puts("Failed to create DSP node\n");
        return -1;
    }

    if (kernel::g_audio_system.get_dsp_graph().add_node(std::move(node_ptr))) {
        uart_ops->puts("DSP node added\n");
        return 0;
    }
    uart_ops->puts("Failed to add DSP node\n");
    return -1;
}

int cli_dsp_remove(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char name[kernel::core::MAX_NAME_LENGTH] = {0};
    if (std::sscanf(args, "dsp_remove %31s", name) != 1) {
        uart_ops->puts("Usage: dsp_remove <name>\n");
        return -1;
    }

    std::string_view name_sv = name;
    if (kernel::g_audio_system.get_dsp_graph().remove_node(name_sv)) {
        uart_ops->puts("DSP node removed\n");
        return 0;
    }
    uart_ops->puts("Failed to remove DSP node\n");
    return -1;
}

int cli_dsp_config(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char name_to_config[32] = {0};
    char config_args_for_node[64] = {0};
    if (std::sscanf(args, "dsp_config %31s %63[^\n]", name_to_config, config_args_for_node) != 2) {
        uart_ops->puts("Usage: dsp_config <name> <params>\n");
        return -1;
    }

    std::string_view name_to_config_args = name_to_config;
    const char* config_args_for_node_ptr = config_args_for_node;
    if (kernel::g_audio_system.get_dsp_graph().configure_node(name_to_config_args.data(), config_args_for_node_ptr, uart_ops)) {
        uart_ops->puts("DSP node configured\n");
        return 0;
    }
    uart_ops->puts("Failed to configure DSP node\n");
    return -1;
}

int cli_dsp_reset(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    kernel::g_audio_system.get_dsp_graph().reset();
    uart_ops->puts("DSP graph reset\n");
    return 0;
}

int cli_fs_cat_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    std::array<char, fs::MAX_CONTENT_SIZE> content;
    if (kernel::g_file_system.read_file(args + 4, content)) {
        uart_ops->puts(content.data());
        uart_ops->puts("\n");
        return 0;
    }
    uart_ops->puts("Failed to read file\n");
    return -1;
}

int cli_fs_mkfile_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (kernel::g_file_system.create_file(args + 7, false)) {
        uart_ops->puts("File created\n");
        return 0;
    }
    uart_ops->puts("Failed to create file\n");
    return -1;
}

int cli_fs_mkdir_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (kernel::g_file_system.create_file(args + 6, true)) {
        uart_ops->puts("Directory created\n");
        return 0;
    }
    uart_ops->puts("Failed to create directory\n");
    return -1;
}

int cli_fs_rm_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    if (kernel::g_file_system.delete_file(args + 3)) {
        uart_ops->puts("File/directory removed\n");
        return 0;
    }
    uart_ops->puts("Failed to remove file/directory\n");
    return -1;
}

int cli_net_listen_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int port;
    if (std::sscanf(args, "listen_udp %d", &port) != 1) {
        uart_ops->puts("Usage: listen_udp <port>\n");
        return -1;
    }
    int sock_idx = kernel::g_net_manager.create_udp_socket(net::IPv4Addr{0}, static_cast<uint16_t>(port));
    if (sock_idx >= 0) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "UDP socket %d listening on port %d\n", sock_idx, port);
        uart_ops->puts(buf);
        return 0;
    }
    uart_ops->puts("Failed to create UDP socket\n");
    return -1;
}

int cli_net_send_udp_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    char ip[16];
    int port;
    char data[64];
    if (std::sscanf(args, "send_udp %15s %d %63[^\n]", ip, &port, data) != 3) {
        uart_ops->puts("Usage: send_udp <ip> <port> <data>\n");
        return -1;
    }

    net::IPv4Addr addr;
    if (!addr.from_string(ip)) {
        uart_ops->puts("Invalid IP address\n");
        return -1;
    }

    net::UDPPacket p;
    p.dest_ip = addr;
    p.dest_port = static_cast<uint16_t>(port);
    p.payload = std::string_view(data);
    int sock_idx = kernel::g_net_manager.create_udp_socket(net::IPv4Addr{0}, 0);
    if (sock_idx >= 0) {
        if (kernel::g_net_manager.send(sock_idx, p, true)) {
            uart_ops->puts("UDP packet sent\n");
        } else {
            uart_ops->puts("Failed to send UDP packet\n");
        }
        kernel::g_net_manager.close_socket(sock_idx);
        return 0;
    }
    uart_ops->puts("Failed to create UDP socket\n");
    return -1;
}

int cli_net_close_socket_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    int sock_idx;
    if (std::sscanf(args, "close_socket %d", &sock_idx) != 1) {
        uart_ops->puts("Usage: close_socket <socket_idx>\n");
        return -1;
    }
    if (kernel::g_net_manager.close_socket(sock_idx)) {
        uart_ops->puts("Socket closed\n");
        return 0;
    }
    uart_ops->puts("Failed to close socket\n");
    return -1;
}

int cli_net_sockets_command(const char* args, kernel::hal::UARTDriverOps* uart_ops) {
    kernel::g_net_manager.list_sockets(uart_ops);
    return 0;
}

bool CLI::process_script(const char* filename, kernel::hal::UARTDriverOps* uart_ops) {
    std::array<char, fs::MAX_CONTENT_SIZE> script_content;
    if (!kernel::g_file_system.read_file(filename, script_content)) {
        uart_ops->puts("Failed to read script file\n");
        return false;
    }
    // Stub implementation for script processing
    return true;
}

} // namespace cli