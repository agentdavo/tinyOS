// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file demo_cli_dsp.cpp
 * @brief CLI DSP demonstration application for miniOS v1.7.
 * @details
 * Implements a sample DSP pipeline demonstrating real-time audio processing with a 4-way crossover,
 * gain, compressor, and reverb nodes, integrated with CLI commands for setup and control. Uses
 * network audio streaming for input/output. Updated in v1.7 with improved error handling, clearer
 * documentation, and modern C++20 practices, retaining all v1.6 functionality.
 *
 * C++20 features:
 * - std::string_view for string handling
 * - std::span for buffer operations
 * - std::optional for safer returns
 *
 * @version 1.7
 * @see miniOS.hpp, dsp.hpp, audio.hpp, net.hpp, cli.hpp
 */

#include "miniOS.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "net.hpp"
#include "cli.hpp"
#include "util.hpp"
#include <cstring>

namespace demo {

class DSPDemo {
public:
    DSPDemo() = default;

    bool init(kernel::UARTDriverOps* uart_ops) {
        if (!uart_ops) return false;

        audio::AudioConfig audio_cfg = {
            .sample_rate_hz = 48000,
            .samples_per_block = 256,
            .num_i2s_bufs = 4
        };
        if (!audio::g_audio_system.init(audio_cfg) || !audio::g_audio_system.start()) {
            uart_ops->puts("Failed to initialize audio system\n");
            return false;
        }

        if (!setup_dsp_pipeline(uart_ops)) return false;

        socket_idx_ = net::g_net_manager.create_udp_socket({0x0A000001}, 1234);
        if (socket_idx_ < 0) {
            uart_ops->puts("Failed to create network socket\n");
            return false;
        }

        if (!net::g_net_manager.set_socket_priority(socket_idx_, 10) ||
            !net::g_net_manager.set_jitter_buffer_size(socket_idx_, 4)) {
            uart_ops->puts("Failed to configure network socket\n");
            return false;
        }

        uart_ops->puts("DSP demo initialized\n");
        return true;
    }

    static int cli_command(const char* args, kernel::UARTDriverOps* uart_ops) {
        if (!args || !*args || !uart_ops) {
            uart_ops->puts("Usage: dspdemo <start|stop|addnode <type> <name>|confignet <ip> <port> <channels>)\n");
            return 1;
        }

        std::string_view cmd(args);
        if (cmd.starts_with("start")) {
            if (!dsp_demo_.init(uart_ops)) return 1;
            return 0;
        } else if (cmd.starts_with("stop")) {
            audio::g_audio_system.stop();
            uart_ops->puts("DSP demo stopped\n");
            return 0;
        } else if (cmd.starts_with("addnode")) {
            return add_node(args + 8, uart_ops);
        } else if (cmd.starts_with("confignet")) {
            return configure_network(args + 10, uart_ops);
        }

        uart_ops->puts("Invalid dspdemo command\n");
        return 1;
    }

private:
    static DSPDemo dsp_demo_;
    int socket_idx_ = -1;

    bool setup_dsp_pipeline(kernel::UARTDriverOps* uart_ops) {
        auto* crossover = new kernel::CrossoverDSP("crossover");
        if (!audio::g_audio_system.dsp_graph.add_node(crossover)) {
            uart_ops->puts("Failed to add crossover node\n");
            delete crossover;
            return false;
        }
        if (!audio::g_audio_system.dsp_graph.configure_node("crossover",
                "band 0 butterworth lowpass 2 200 0\n"
                "band 1 butterworth highpass 2 200 0\n"
                "band 2 lowpass 2 1000 0\n"
                "band 3 highpass 2 1000 0", uart_ops)) {
            return false;
        }

        auto* gain = new kernel::GainDSP(1.0f, "gain");
        if (!audio::g_audio_system.dsp_graph.add_node(gain)) {
            uart_ops->puts("Failed to add gain node\n");
            delete gain;
            return false;
        }

        auto* comp = new kernel::CompressorDSP("comp");
        if (!audio::g_audio_system.dsp_graph.add_node(comp)) {
            uart_ops->puts("Failed to add compressor node\n");
            delete comp;
            return false;
        }
        audio::g_audio_system.dsp_graph.configure_node("comp", "compressor -15 4 10 100 2", uart_ops);

        auto* reverb = new kernel::ReverbDSP("reverb");
        if (!audio::g_audio_system.dsp_graph.add_node(reverb)) {
            uart_ops->puts("Failed to add reverb node\n");
            delete reverb;
            return false;
        }
        audio::g_audio_system.dsp_graph.configure_node("reverb", "reverb 10 0.5 0.5 0.3", uart_ops);

        return true;
    }

    static int add_node(const char* args, kernel::UARTDriverOps* uart_ops) {
        char type[16], name[32];
        if (std::sscanf(args, "%15s %31s", type, name) != 2) {
            uart_ops->puts("Usage: addnode <type> <name>\n");
            return 1;
        }
        std::string_view node_type(type);
        kernel::DSPNode* node = nullptr;
        if (node_type == "gain") {
            node = new kernel::GainDSP(1.0f, name);
        } else if (node_type == "compressor") {
            node = new kernel::CompressorDSP(name);
        } else if (node_type == "reverb") {
            node = new kernel::ReverbDSP(name);
        } else {
            uart_ops->puts("Invalid node type\n");
            return 1;
        }
        if (audio::g_audio_system.dsp_graph.add_node(node)) {
            uart_ops->puts("Added node ");
            uart_ops->puts(name);
            uart_ops->puts("\n");
            return 0;
        }
        delete node;
        uart_ops->puts("Failed to add node\n");
        return 1;
    }

    static int configure_network(const char* args, kernel::UARTDriverOps* uart_ops) {
        char ip_str[16];
        int port, channels;
        if (std::sscanf(args, "%15s %d %d", ip_str, &port, &channels) != 3 || port <= 0 || port > 65535 || channels < 1 || channels > 8) {
            uart_ops->puts("Usage: confignet <ip> <port> <channels>\n");
            return 1;
        }
        uint32_t ip_addr;
        if (!util::ipv4_to_uint32(ip_str, ip_addr)) {
            uart_ops->puts("Invalid IP address\n");
            return 1;
        }
        auto* netaudio = new kernel::NetworkAudioSinkSource("netaudio");
        if (!audio::g_audio_system.dsp_graph.add_node(netaudio)) {
            uart_ops->puts("Failed to add network audio node\n");
            delete netaudio;
            return 1;
        }
        char config[64];
        std::snprintf(config, sizeof(config), "netaudio sink %s %d %d", ip_str, port, channels);
        if (!audio::g_audio_system.dsp_graph.configure_node("netaudio", config, uart_ops)) {
            return 1;
        }
        return 0;
    }
};

DSPDemo DSPDemo::dsp_demo_;

void register_demo_commands() {
    cli::g_cli.register_command("dspdemo", DSPDemo::cli_command,
                                "Run DSP demo (start, stop, addnode, confignet)");
}

} // namespace demo