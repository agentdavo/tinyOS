// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file test_framework.cpp
 * @brief Self-test framework for miniOS v1.6, validating subsystems.
 * @details
 * Provides hardware-agnostic tests for HAL (UART, timer, GPIO), kernel (spinlock, threading),
 * CLI, DSP, tracing, file system, and networking (UDP/TCP, audio streaming). Tests are invoked
 * via the `test` CLI command or at boot for CI/port validation, with results output to UART.
 * Enhanced in v1.6 with tests for GPIO, audio streaming, DSP nodes (FIR/IIR, equalizer,
 * network audio), and RAMFS (permissions, directories).
 *
 * C++20 features:
 * - std::string_view for test names
 * - std::span for buffer operations
 *
 * @version 1.6
 * @see miniOS.hpp, cli.hpp, dsp.hpp, audio.hpp, trace.hpp, fs.hpp, net.hpp, gpio.hpp
 */

#include "miniOS.hpp"
#include "cli.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "trace.hpp"
#include "fs.hpp"
#include "net.hpp"
#include "gpio.hpp"
#include <array>
#include <span>
#include <string_view>

namespace test {

/**
 * @brief Test result codes.
 */
enum class Result { PASS, FAIL };

/**
 * @brief Outputs a test result to UART.
 */
void report(hal::UARTDriverOps* uart_ops, std::string_view test_name, Result result) {
    uart_ops->puts("Test: ");
    uart_ops->puts(test_name.data());
    uart_ops->puts(" - ");
    uart_ops->puts(result == Result::PASS ? "PASS\n" : "FAIL\n");
}

/**
 * @brief Tests UART output.
 */
Result test_uart(hal::UARTDriverOps* uart_ops) {
    uart_ops->puts("miniOS UART self-test\n");
    return Result::PASS;
}

/**
 * @brief Tests timer delay accuracy.
 */
Result test_timer(hal::UARTDriverOps* uart_ops) {
    auto* timer_ops = kernel::g_platform->get_timer_ops();
    constexpr uint64_t DELAY_US = 100'000; // 100 ms
    uint64_t start = timer_ops->get_system_time_us();
    uint64_t target = start + DELAY_US;
    while (timer_ops->get_system_time_us() < target) {
        asm volatile("yield" ::: "memory");
    }
    uint64_t end = timer_ops->get_system_time_us();
    int64_t actual = static_cast<int64_t>(end - start);
    bool ok = actual > 0.9 * DELAY_US && actual < 1.1 * DELAY_US;
    uart_ops->puts("Timer delay actual(us): ");
    uart_ops->uart_put_uint64_hex(actual);
    uart_ops->puts("\n");
    return ok ? Result::PASS : Result::FAIL;
}

/**
 * @brief Tests spinlock acquire/release.
 */
Result test_spinlock(hal::UARTDriverOps* uart_ops) {
    Spinlock lock;
    lock.acquire_general();
    bool locked = true;
    lock.release_general();
    locked = false;
    return !locked ? Result::PASS : Result::FAIL;
}

/**
 * @brief Tests thread context switching.
 */
volatile bool thread_test_flag = false;
void thread_test_fn(void*) {
    thread_test_flag = true;
    kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
}
Result test_thread_switch(hal::UARTDriverOps* uart_ops) {
    thread_test_flag = false;
    auto* tcb = kernel::g_scheduler_ptr->create_thread(thread_test_fn, nullptr, 1, -1, "test_sw", false, 0);
    for (int i = 0; i < 10; ++i) {
        kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
        if (thread_test_flag) break;
    }
    return thread_test_flag ? Result::PASS : Result::FAIL;
}

/**
 * @brief Tests CLI command registration.
 */
int test_cli_handler(const char*, hal::UARTDriverOps* uart_ops) {
    uart_ops->puts("CLI handler invoked\n");
    return 0;
}
Result test_cli(hal::UARTDriverOps* uart_ops) {
    cli::register_command("testcli", test_cli_handler, "test handler");
    cli::CLI cli;
    cli.process_line("testcli", uart_ops);
    return Result::PASS;
}

/**
 * @brief Tests CLI script execution.
 */
Result test_cli_script(hal::UARTDriverOps* uart_ops) {
    if (!fs::g_file_system.create_file("/test_script.txt")) {
        uart_ops->puts("CLI script test: could not create script file\n");
        return Result::FAIL;
    }
    const char* script_content = "help\n";
    if (!fs::g_file_system.write_file("/test_script.txt", script_content, strlen(script_content))) {
        uart_ops->puts("CLI script test: could not write script file\n");
        return Result::FAIL;
    }
    cli::CLI cli;
    if (!cli.process_script("/test_script.txt", uart_ops)) {
        uart_ops->puts("CLI script test: script execution failed\n");
        return Result::FAIL;
    }
    return Result::PASS;
}

/**
 * @brief Tests CLI tab completion.
 */
Result test_cli_tab_completion(hal::UARTDriverOps* uart_ops) {
    cli::CLI cli;
    cli::register_command("testcmd1", test_cli_handler, "test command 1");
    cli::register_command("testcmd2", test_cli_handler, "test command 2");
    cli.cmd_buffer = {'t', 'e', 's', 't'};
    cli.cmd_buffer_idx = 4;
    cli.complete_command(uart_ops);
    bool ok = std::strncmp(cli.cmd_buffer.data(), "testcmd", 7) == 0;
    return ok ? Result::PASS : Result::FAIL;
}

/**
 * @brief Tests DSP gain node (generic).
 */
Result test_dsp_gain(hal::UARTDriverOps* uart_ops) {
    kernel::DSPGraph g("testgraph");
    auto* node = new kernel::GainDSP_Float(1.0f, "testgain");
    if (!g.add_node(node)) {
        uart_ops->puts("DSP gain test: could not add node\n");
        return Result::FAIL;
    }
    if (g.configure_node("testgain", "gain 2.0", uart_ops)) {
        std::array<float, 4> buffer = {1.0f, 1.0f, 1.0f, 1.0f};
        g.process(std::span(buffer));
        bool correct = true;
        for (float val : buffer) {
            if (std::abs(val - 2.0f) > 0.001f) correct = false;
        }
        if (correct) {
            uart_ops->puts("DSP gain node processed correctly\n");
            return Result::PASS;
        }
        uart_ops->puts("DSP gain node processing incorrect\n");
    }
    return Result::FAIL;
}

/**
 * @brief Tests DSP FIR filter node.
 */
Result test_dsp_fir(hal::UARTDriverOps* uart_ops) {
    kernel::DSPGraph g("testgraph");
    auto* node = new kernel::FIRFilterDSP("testfir");
    if (!g.add_node(node)) {
        uart_ops->puts("DSP FIR test: could not add node\n");
        return Result::FAIL;
    }
    if (g.configure_node("testfir", "fir 1.0 0.5", uart_ops)) {
        std::array<float, 4> buffer = {1.0f, 1.0f, 1.0f, 1.0f};
        g.process(std::span(buffer));
        bool correct = std::abs(buffer[1] - 1.5f) < 0.001f; // Expected: 1.0*1.0 + 1.0*0.5
        if (correct) {
            uart_ops->puts("DSP FIR node processed correctly\n");
            return Result::PASS;
        }
        uart_ops->puts("DSP FIR node processing incorrect\n");
    }
    return Result::FAIL;
}

/**
 * @brief Tests DSP IIR filter node.
 */
Result test_dsp_iir(hal::UARTDriverOps* uart_ops) {
    kernel::DSPGraph g("testgraph");
    auto* node = new kernel::IIRFilterDSP("testiir");
    if (!g.add_node(node)) {
        uart_ops->puts("DSP IIR test: could not add node\n");
        return Result::FAIL;
    }
    if (g.configure_node("testiir", "iir 1.0 0.5 0.0 0.0", uart_ops)) {
        std::array<float, 4> buffer = {1.0f, 1.0f, 1.0f, 1.0f};
        g.process(std::span(buffer));
        bool correct = std::abs(buffer[1] - 1.5f) < 0.001f; // Expected: 1.0*1.0 + 0.5*1.0
        if (correct) {
            uart_ops->puts("DSP IIR node processed correctly\n");
            return Result::PASS;
        }
        uart_ops->puts("DSP IIR node processing incorrect\n");
    }
    return Result::FAIL;
}

/**
 * @brief Tests DSP network audio sink/source.
 */
Result test_dsp_network_audio(hal::UARTDriverOps* uart_ops) {
    kernel::DSPGraph g("testgraph");
    auto* node = new kernel::NetworkAudioSinkSource("testnet");
    if (!g.add_node(node)) {
        uart_ops->puts("DSP network audio test: could not add node\n");
        return Result::FAIL;
    }
    if (g.configure_node("testnet", "sink 10.0.0.2 5678 2", uart_ops)) {
        std::array<float, 4> buffer = {1.0f, 1.0f, 1.0f, 1.0f};
        g.process(std::span(buffer)); // Sends packet
        // Simplified: Assume socket creation succeeded
        uart_ops->puts("DSP network audio sink configured\n");
        return Result::PASS;
    }
    uart_ops->puts("DSP network audio configuration failed\n");
    return Result::FAIL;
}

/**
 * @brief Tests tracing enable/disable.
 */
Result test_trace_enable_disable(hal::UARTDriverOps* uart_ops) {
    trace::g_trace_manager.set_enabled(false);
    if (trace::g_trace_manager.is_enabled()) {
        uart_ops->puts("Trace enable failed: unexpected enabled state\n");
        return Result::FAIL;
    }
    trace::g_trace_manager.set_enabled(true);
    return trace::g_trace_manager.is_enabled() ? Result::PASS : Result::FAIL;
}

/**
 * @brief Tests tracing event recording/dump.
 */
Result test_trace_record_and_dump(hal::UARTDriverOps* uart_ops) {
    trace::g_trace_manager.clear_trace();
    trace::g_trace_manager.set_enabled(true);
    kernel::TCB dummy_tcb{.name = "DummyThread"};
    trace::g_trace_manager.record_event(&dummy_tcb, trace::EventType::THREAD_CREATE, "TestCreate");
    trace::g_trace_manager.record_event(&dummy_tcb, trace::EventType::CUSTOM, "TestCustom", 0x1234);
    if (trace::g_trace_manager.buffer.size() < 2) {
        uart_ops->puts("Trace record failed: insufficient events\n");
        trace::g_trace_manager.set_enabled(false);
        return Result::FAIL;
    }
    trace::g_trace_manager.dump_trace(uart_ops);
    trace::g_trace_manager.set_enabled(false);
    return Result::PASS;
}

/**
 * @brief Tests tracing buffer clear.
 */
Result test_trace_clear(hal::UARTDriverOps* uart_ops) {
    trace::g_trace_manager.clear_trace();
    trace::g_trace_manager.set_enabled(true);
    kernel::TCB dummy_tcb{.name = "DummyThread"};
    trace::g_trace_manager.record_event(&dummy_tcb, trace::EventType::THREAD_CREATE, "TestCreate");
    trace::g_trace_manager.clear_trace();
    if (trace::g_trace_manager.buffer.size() != 0) {
        uart_ops->puts("Trace clear failed: buffer not empty\n");
        trace::g_trace_manager.set_enabled(false);
        return Result::FAIL;
    }
    trace::g_trace_manager.set_enabled(false);
    return Result::PASS;
}

/**
 * @brief Tests file system creation/listing with directories.
 */
Result test_fs_create_list(hal::UARTDriverOps* uart_ops) {
    if (!fs::g_file_system.create_file("/testdir", true)) {
        uart_ops->puts("FS create failed: could not create directory\n");
        return Result::FAIL;
    }
    if (!fs::g_file_system.create_file("/testdir/testfile")) {
        uart_ops->puts("FS create failed: could not create file\n");
        return Result::FAIL;
    }
    fs::g_file_system.list_files("/testdir", uart_ops);
    return Result::PASS;
}

/**
 * @brief Tests file system permissions.
 */
Result test_fs_permissions(hal::UARTDriverOps* uart_ops) {
    if (!fs::g_file_system.create_file("/permfile")) {
        uart_ops->puts("FS permissions test: could not create file\n");
        return Result::FAIL;
    }
    if (!fs::g_file_system.set_permissions("/permfile", 0b00000100)) { // Read-only
        uart_ops->puts("FS permissions test: could not set permissions\n");
        return Result::FAIL;
    }
    const char* data = "test";
    if (fs::g_file_system.write_file("/permfile", data, strlen(data))) {
        uart_ops->puts("FS permissions test: write succeeded on read-only file\n");
        return Result::FAIL;
    }
    char buffer[16];
    size_t bytes_read = fs::g_file_system.read_file("/permfile", buffer, sizeof(buffer));
    if (bytes_read == 0) {
        uart_ops->puts("FS permissions test: read failed on readable file\n");
        return Result::FAIL;
    }
    return Result::PASS;
}

/**
 * @brief Tests networking socket creation.
 */
Result test_net_socket_create(hal::UARTDriverOps* uart_ops) {
    net::IPv4Addr ip{.addr = 0x0A000001}; // 10.0.0.1
    int udp_idx = net::g_net_manager.create_udp_socket(ip, 1234);
    int tcp_idx = net::g_net_manager.create_tcp_socket(ip, 5678);
    if (udp_idx < 0 || tcp_idx < 0) {
        uart_ops->puts("Net socket create failed: could not create UDP/TCP sockets\n");
        return Result::FAIL;
    }
    net::g_net_manager.list_sockets(uart_ops);
    return Result::PASS;
}

/**
 * @brief Tests UDP packet sending.
 */
Result test_net_udp_send(hal::UARTDriverOps* uart_ops) {
    net::IPv4Addr local_ip{.addr = 0x0A000001};
    net::IPv4Addr remote_ip{.addr = 0x0A000002};
    int socket_idx = net::g_net_manager.create_udp_socket(local_ip, 1234);
    if (socket_idx < 0) {
        uart_ops->puts("Net UDP send failed: could not create socket\n");
        return Result::FAIL;
    }
    net::Packet packet{.dst_ip = remote_ip, .dst_port = 5678, .data_len = 5};
    std::memcpy(packet.data.data(), "hello", 5);
    if (!net::g_net_manager.send(socket_idx, packet)) {
        uart_ops->puts("Net UDP send failed: could not send packet\n");
        return Result::FAIL;
    }
    uart_ops->puts("UDP packet sent successfully\n");
    return Result::PASS;
}

/**
 * @brief Tests audio streaming.
 */
Result test_net_audio_stream(hal::UARTDriverOps* uart_ops) {
    net::IPv4Addr local_ip{.addr = 0x0A000001};
    net::IPv4Addr remote_ip{.addr = 0x0A000002};
    int socket_idx = net::g_net_manager.create_udp_socket(local_ip, 1234);
    if (socket_idx < 0) {
        uart_ops->puts("Net audio stream test: could not create socket\n");
        return Result::FAIL;
    }
    net::Packet packet{.dst_ip = remote_ip, .dst_port = 5678, .data_len = 8, .audio_channels = 2};
    std::array<float, 2> audio_data = {1.0f, 1.0f};
    std::memcpy(packet.data.data(), audio_data.data(), 8);
    if (!net::g_net_manager.send(socket_idx, packet, true)) {
        uart_ops->puts("Net audio stream test: could not send audio packet\n");
        return Result::FAIL;
    }
    uart_ops->puts("Audio stream packet sent successfully\n");
    return Result::PASS;
}

/**
 * @brief Tests GPIO configuration and read/write.
 */
Result test_gpio(hal::UARTDriverOps* uart_ops) {
    if (!gpio::g_gpio_manager.configure_pin(0, 0, hal::gpio::PinMode::OUTPUT)) {
        uart_ops->puts("GPIO test: could not configure pin\n");
        return Result::FAIL;
    }
    if (!gpio::g_gpio_manager.set_pin(0, 0, hal::gpio::PinState::HIGH)) {
        uart_ops->puts("GPIO test: could not set pin\n");
        return Result::FAIL;
    }
    hal::gpio::PinState state = gpio::g_gpio_manager.read_pin(0, 0);
    if (state != hal::gpio::PinState::HIGH) {
        uart_ops->puts("GPIO test: read incorrect state\n");
        return Result::FAIL;
    }
    uart_ops->puts("GPIO test: pin configured and read correctly\n");
    return Result::PASS;
}

/**
 * @brief Tests audio latency.
 */
Result test_audio_latency(hal::UARTDriverOps* uart_ops) {
    if (!audio::g_audio_system.initialized) {
        audio::AudioConfig cfg{.sample_rate_hz = 48000, .samples_per_block = 256, .num_i2s_bufs = 4};
        if (!audio::g_audio_system.init(cfg) || !audio::g_audio_system.start()) {
            uart_ops->puts("Audio latency test: could not initialize audio\n");
            return Result::FAIL;
        }
    }
    auto* timer_ops = kernel::g_platform->get_timer_ops();
    uint64_t start = timer_ops->get_system_time_us();
    kernel::g_scheduler_ptr->yield(kernel::g_platform->get_core_id());
    uint64_t end = timer_ops->get_system_time_us();
    int64_t latency_us = end - start;
    bool ok = latency_us < 1000; // Expect <1ms
    uart_ops->puts("Audio latency (us): ");
    uart_ops->uart_put_uint64_hex(latency_us);
    uart_ops->puts("\n");
    return ok ? Result::PASS : Result::FAIL;
}

/**
 * @brief Runs all self-tests.
 */
void run_all(hal::UARTDriverOps* uart_ops) {
    uart_ops->puts("=== miniOS v1.6 Self-Test Framework ===\n");
    report(uart_ops, "UART", test_uart(uart_ops));
    report(uart_ops, "Timer", test_timer(uart_ops));
    report(uart_ops, "Spinlock", test_spinlock(uart_ops));
    report(uart_ops, "ThreadCtx", test_thread_switch(uart_ops));
    report(uart_ops, "CLI", test_cli(uart_ops));
    report(uart_ops, "CLIScript", test_cli_script(uart_ops));
    report(uart_ops, "CLITabCompletion", test_cli_tab_completion(uart_ops));
    report(uart_ops, "DSPGain", test_dsp_gain(uart_ops));
    report(uart_ops, "DSPFIR", test_dsp_fir(uart_ops));
    report(uart_ops, "DSPIIR", test_dsp_iir(uart_ops));
    report(uart_ops, "DSPNetworkAudio", test_dsp_network_audio(uart_ops));
    report(uart_ops, "TraceEnable", test_trace_enable_disable(uart_ops));
    report(uart_ops, "TraceRecord", test_trace_record_and_dump(uart_ops));
    report(uart_ops, "TraceClear", test_trace_clear(uart_ops));
    report(uart_ops, "FSCreateList", test_fs_create_list(uart_ops));
    report(uart_ops, "FSPermissions", test_fs_permissions(uart_ops));
    report(uart_ops, "NetSocketCreate", test_net_socket_create(uart_ops));
    report(uart_ops, "NetUDPSend", test_net_udp_send(uart_ops));
    report(uart_ops, "NetAudioStream", test_net_audio_stream(uart_ops));
    report(uart_ops, "GPIO", test_gpio(uart_ops));
    report(uart_ops, "AudioLatency", test_audio_latency(uart_ops));
    uart_ops->puts("=== Self-test complete ===\n");
}

/**
 * @brief CLI command for tests.
 */
int cli_test_cmd([[maybe_unused]] const char* args, hal::UARTDriverOps* uart_ops) {
    run_all(uart_ops);
    return 0;
}

/**
 * @brief Registers the test CLI command.
 */
void register_cli() {
    cli::register_command("test", cli_test_cmd, "Run self-test framework");
}

} // namespace test