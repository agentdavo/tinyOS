// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file test_framework.cpp
 * @brief Unit test framework for miniOS v1.7.
 * @details
 * Implements a lightweight unit test framework for validating miniOS subsystems, including kernel
 * (scheduler, threading), DSP (nodes, pipeline), audio (pipeline, I2S), networking (sockets,
 * streaming), and GPIO (pins, interrupts). Provides CLI integration for running tests and reporting
 * results via UART. Generated for v1.7 to meet v1.6 testing requirements, with improved error
 * handling, clear documentation, and modern C++20 practices.
 *
 * C++20 features:
 * - std::string_view for string handling
 * - std::span for buffer operations
 * - std::optional for safer returns
 *
 * @version 1.7
 * @see miniOS.hpp, dsp.hpp, audio.hpp, net.hpp, gpio.hpp, cli.hpp
 */

#include "miniOS.hpp"
#include "dsp.hpp"
#include "audio.hpp"
#include "net.hpp"
#include "gpio.hpp"
#include "cli.hpp"
#include "util.hpp"
#include <cstring>
#include <vector>
#include <string_view>

namespace test {

class TestFramework {
public:
    struct TestCase {
        std::string_view name;
        bool (*func)(kernel::UARTDriverOps*);
        std::string_view description;
    };

    void register_test(const TestCase& test) {
        tests_.push_back(test);
    }

    int run_all_tests(kernel::UARTDriverOps* uart_ops) {
        if (!uart_ops) return -1;
        int failures = 0;
        uart_ops->puts("\nRunning miniOS Unit Tests\n");
        for (const auto& test : tests_) {
            uart_ops->puts("Test: ");
            uart_ops->puts(test.name.data());
            uart_ops->puts(" - ");
            uart_ops->puts(test.description.data());
            uart_ops->puts("... ");
            bool result = test.func(uart_ops);
            uart_ops->puts(result ? "PASSED" : "FAILED");
            uart_ops->puts("\n");
            if (!result) ++failures;
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "\nTests completed: %u passed, %u failed\n",
                      static_cast<unsigned>(tests_.size() - failures), static_cast<unsigned>(failures));
        uart_ops->puts(buf);
        return failures;
    }

    static int cli_command(const char* args, kernel::UARTDriverOps* uart_ops) {
        if (!args || !*args || !uart_ops) {
            uart_ops->puts("Usage: test run\n");
            return 1;
        }
        if (util::strcmp(args, "run") == 0) {
            return test_framework_.run_all_tests(uart_ops) == 0 ? 0 : 1;
        }
        uart_ops->puts("Invalid test command\n");
        return 1;
    }

private:
    static TestFramework test_framework_;
    std::vector<TestCase> tests_;

    static bool test_scheduler_thread_creation(kernel::UARTDriverOps* uart_ops) {
        if (!kernel::g_scheduler_ptr) {
            uart_ops->puts("Scheduler not initialized\n");
            return false;
        }
        auto* tcb = kernel::g_scheduler_ptr->create_thread([](void*){}, nullptr, 1, -1, "test_thread", false);
        if (!tcb) {
            uart_ops->puts("Failed to create thread\n");
            return false;
        }
        return true;
    }

    static bool test_dsp_crossover(kernel::UARTDriverOps* uart_ops) {
        kernel::CrossoverDSP crossover("test_cross");
        std::array<float, 256> buffer = {};
        buffer[0] = 1.0f;
        crossover.process(std::span<float>(buffer));
        if (buffer[0] == 1.0f) {
            uart_ops->puts("Crossover did not process input\n");
            return false;
        }
        return true;
    }

    static bool test_audio_pipeline(kernel::UARTDriverOps* uart_ops) {
        audio::AudioConfig cfg = {.sample_rate_hz = 48000, .samples_per_block = 256, .num_i2s_bufs = 4};
        if (!audio::g_audio_system.init(cfg)) {
            uart_ops->puts("Failed to initialize audio system\n");
            return false;
        }
        auto* buffer = audio::g_audio_system.get_tx_buffer(1);
        if (!buffer) {
            uart_ops->puts("Failed to get TX buffer\n");
            return false;
        }
        audio::g_audio_system.submit_tx_buffer(1, buffer);
        return true;
    }

    static bool test_network_socket(kernel::UARTDriverOps* uart_ops) {
        int idx = net::g_net_manager.create_udp_socket({0x0A000001}, 1234);
        if (idx < 0) {
            uart_ops->puts("Failed to create UDP socket\n");
            return false;
        }
        return true;
    }

    static bool test_gpio_pin(kernel::UARTDriverOps* uart_ops) {
        if (!gpio::g_gpio_manager.init(kernel::g_platform ? kernel::g_platform->get_gpio_ops() : nullptr)) {
            uart_ops->puts("Failed to initialize GPIO\n");
            return false;
        }
        if (!gpio::g_gpio_manager.configure_pin(0, 1, kernel::gpio::PinMode::OUTPUT)) {
            uart_ops->puts("Failed to configure GPIO pin\n");
            return false;
        }
        if (!gpio::g_gpio_manager.set_pin(0, 1, kernel::gpio::PinState::HIGH)) {
            uart_ops->puts("Failed to set GPIO pin\n");
            return false;
        }
        return true;
    }
};

TestFramework TestFramework::test_framework_;

void register_tests() {
    TestFramework& tf = TestFramework::test_framework_;
    tf.register_test({"scheduler_thread", test_scheduler_thread_creation, "Test thread creation"});
    tf.register_test({"dsp_crossover", test_dsp_crossover, "Test crossover DSP node"});
    tf.register_test({"audio_pipeline", test_audio_pipeline, "Test audio pipeline"});
    tf.register_test({"network_socket", test_network_socket, "Test UDP socket creation"});
    tf.register_test({"gpio_pin", test_gpio_pin, "Test GPIO pin operations"});
}

} // namespace test