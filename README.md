# miniOS - A Lightweight, Portable RTOS in C++17

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue)](https://opensource.org/licenses)
[![C++ Standard: C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/w/cpp/17)

## Overview
miniOS is a modern, lightweight real-time operating system (RTOS) for embedded systems, written in C++17 with C++20 features. It emphasizes strict kernel/HAL separation, modularity, and developer-friendliness, targeting QEMU virt for ARM64 and RISC-V RV64IMA. Key features include:

- **Modular Subsystems**: CLI (tab completion, history), DSP audio (SIMD-optimized, 4-way crossover), task tracing, RAMFS (directories, permissions), UDP/TCP networking (virtio-net, audio streaming), GPIO (4x64 pins).
- **Portable Design**: Hardware-agnostic kernel with HALs for QEMU ARM64/RISC-V.
- **Multi-Core Support**: SMP-aware EDF scheduler for up to 4 cores/harts.
- **Developer-Friendly**: CLI-driven debugging, self-tests, Doxygen docs, CI pipeline.

Ideal for embedded projects, RTOS education, and research, with a clean C++ codebase and minimal dependencies.

## New in v1.6
- GPIO subsystem (4 banks, 64 pins each, interrupt support)
- Low-latency multichannel audio streaming (RTP-like UDP)
- Enhanced RAMFS with directories, permissions, 16KB files
- Advanced DSP nodes: parametric EQ, limiter, noise gate, stereo width, waveshaper, envelope follower, compressor, delay, reverb, chorus, flanger, pitch shifter, convolution reverb, 4-way crossover (Butterworth, Bessel, Linkwitz-Riley, 6/12/18/24 dB/octave), phase correction, SRC, FIR/IIR, FFT equalizer
- HAL-based SIMD (ARM NEON, RISC-V RVV)
- CLI tab completion and command history
- Task tracing for SMP debugging
- Watchdog timers and kernel statistics
- Extended self-tests and CI with static analysis

## Getting Started

### Prerequisites
- **OS**: Linux (e.g., Ubuntu)
- **Tools**:
  ```bash
  sudo apt update
  sudo apt install build-essential gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
                   binutils-aarch64-linux-gnu qemu-system-arm gdb-multiarch \
                   gcc-riscv64-linux-gnu g++-riscv64-linux-gnu binutils-riscv64-linux-gnu \
                   doxygen clang-tidy
  ```

### Directory Structure
```
miniOS/
├── .github/workflows/ci.yml        # CI pipeline (build, test, analyze)
├── hal/
│   ├── hal_qemu_arm64.hpp/.cpp     # ARM64 HAL (NEON, virtio-net, GPIO)
│   ├── cpu_arm64.S                 # ARM64 boot/context-switch
│   ├── hal_qemu_rv64ima.hpp/.cpp   # RISC-V HAL (RVV, virtio-net, GPIO)
│   ├── cpu_rv64.S                  # RISC-V boot/context-switch
├── miniOS_v1.6.hpp/.cpp            # Kernel (scheduler, HAL interfaces)
├── util.hpp/.cpp                   # String utilities
├── cli.hpp/.cpp                    # CLI (tab completion, history)
├── dsp.hpp/.cpp                    # DSP audio (SIMD, 4-way crossover)
├── audio.hpp/.cpp                  # Lock-free audio pipelines
├── trace.hpp/.cpp                  # Thread tracing
├── fs.hpp/.cpp                     # RAMFS (directories, permissions)
├── net.hpp/.cpp                    # UDP/TCP networking, audio streaming
├── gpio.hpp/.cpp                   # GPIO (4x64 pins)
├── demo_cli_dsp.cpp                # CLI/DSP demo
├── test_framework.cpp              # Self-tests
├── linker_arm64.ld                 # ARM64 linker script
├── linker_rv64ima.ld               # RISC-V linker script
├── Doxyfile                        # Doxygen configuration
├── Makefile                        # Build automation
├── README.md                       # This file
├── PORTING.md                      # Porting guide
```

### Build and Run (ARM64)
1. **Clone Repository** (or copy files).
2. **Build**:
   ```bash
   make
   ```
   Produces `miniOS.bin`.
3. **Run in QEMU**:
   ```bash
   make run
   ```
   Output:
   ```
   [miniOS v1.6] QEMU Virt Init
   Audio, File System, Networking, GPIO, and CLI Threads Created
   miniOS v1.6 CLI Ready. Type 'help' for commands
   >
   ```
4. **Interact**:
   - `help`: List commands
   - `test`: Run self-tests
   - `fs create /testdir 1`: Create directory
   - `fs ls /testdir`: List files
   - `trace enable`: Start tracing
   - `trace dump`: View thread traces
   - `gpio config 0 1 output`: Configure GPIO pin
   - `net create udp 10.0.0.1 1234`: Create UDP socket
   - `net audio stream 10.0.0.2 5678 8`: Start 8-channel audio stream
   - `dsp add crossover mycross`: Add crossover node
   - `dsp config mycross band 0 butterworth lowpass 2 200 0`: Configure crossover
   - Use TAB for completion, arrows for history

### Build and Run (RISC-V)
1. **Update Makefile**:
   Uncomment RISC-V configuration:
   ```makefile
   CC = riscv64-linux-gnu-g++
   AS = riscv64-linux-gnu-gcc
   LD = riscv64-linux-gnu-ld
   OBJCOPY = riscv64-linux-gnu-objcopy
   CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -march=rv64imafdv -mabi=lp64 -std=c++20 -I.
   ASFLAGS = -march=rv64imafdv -mabi=lp64
   LDFLAGS = -T linker_rv64ima.ld
   OBJ = miniOS_v1.6.o util_cpp20.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_qemu_rv64ima.o cpu_rv64.o gpio.o
   ```
2. **Update `miniOS_v1.6.cpp`**:
   In `kernel_main`, use:
   ```cpp
   g_platform = new hal::qemu_virt_rv64::PlatformQEMUVirtRV64();
   ```
3. **Build and Run**:
   ```bash
   make
   make run
   ```
   Similar output with RISC-V initialization.

### Debugging
1. **Start QEMU**:
   ```bash
   make debug
   ```
2. **Connect GDB**:
   ```bash
   gdb-multiarch miniOS.elf
   (gdb) target remote localhost:1234
   (gdb) break kernel_main
   (gdb) continue
   ```

### Documentation
Generate Doxygen documentation:
```bash
make docs
```
Output in `docs/html/` (HTML) and `docs/latex/refman.pdf` (PDF).

## Key Features
- **CLI Commands**:
  - `help`: List commands
  - `test`: Run self-tests
  - `script <filename>`: Execute RAMFS script
  - `trace [enable|disable|dump|clear]`: Manage thread tracing
  - `fs create <path> [is_dir]`: Create file/directory
  - `fs ls <path>`: List files
  - `gpio config/read/write`: Manage GPIO pins
  - `dsp add <type> <name>`: Add DSP node (e.g., crossover, reverb)
  - `dsp config <name> <args>`: Configure DSP node
  - `net create <udp|tcp> <ip> <port>`: Create socket
  - `net audio stream <ip> <port> <channels>`: Start audio streaming
  - `net list`: List active sockets
  - `stats`: Show kernel statistics
- **Subsystems**:
  - **DSP/Audio**: Lock-free pipelines, HAL-based SIMD (NEON, RVV), 4-way crossover, effects
  - **Tracing**: Thread execution tracing for SMP debugging
  - **File System**: RAMFS with directories, permissions
  - **Networking**: UDP/TCP with low-latency audio streaming
  - **GPIO**: 4 banks, 64 pins each, interrupt support
- **Portability**: QEMU ARM64/RISC-V; see `PORTING.md`

## Extending miniOS
- **Add CLI Command**:
  ```cpp
  int my_command(const char*, hal::UARTDriverOps* uart_ops) {
      uart_ops->puts("Hello, miniOS!\n");
      return 0;
  }
  void register_my_command() {
      cli::register_command("my_cmd", my_command, "Print greeting");
  }
  ```
  Call `register_my_command()` in `kernel_main`.
- **Add DSP Node**:
  ```cpp
  class MyDSPNode : public kernel::DSPNode {
  public:
      MyDSPNode(const char* name) : DSPNode(name) {}
      void process(std::span<float> buffer) override {}
      void configure(const char* args, hal::UARTDriverOps*) override {}
  };
  ```
- **Add Network Service**:
  ```cpp
  void network_service(void*) {
      net::Packet packet;
      int socket_idx = net::g_net_manager.create_udp_socket({0x0A000001}, 1234);
      while (net::g_net_manager.receive(socket_idx, packet, true)) {
          // Process audio packet
      }
  }
  ```
  Create thread in `kernel_main`.
- **Port to New Hardware**: See `PORTING.md`.

## License
Dual-licensed under MIT or Apache-2.0.