# Porting miniOS v1.6 to New Hardware

## Overview
miniOS v1.6 is designed for portability, with a hardware-agnostic kernel and modular HAL interfaces. This guide outlines porting miniOS to new hardware, focusing on implementing a new HAL. The HAL encapsulates platform-specific logic (e.g., UART, interrupts, timers, GPIO, DSP SIMD), ensuring kernel compatibility.

## Prerequisites
- **Hardware Specs**: Memory map, interrupt controller, UART, timers, DMA, GPIO, networking, audio (I2S), and optional SIMD units.
- **Toolchain**: Cross-compiler (e.g., `gcc-arm-none-eabi` for ARM).
- **QEMU Reference**: Use `hal_qemu_arm64.hpp/cpp` or `hal_qemu_rv64ima.hpp/cpp` as templates.

## Steps to Port

### 1. Create HAL Files
Create `hal_myplatform.hpp/cpp` in `hal/`, modeling after `hal_qemu_arm64.hpp/cpp`.

### 2. Implement HAL Interfaces
Implement interfaces from `miniOS_v1.6.hpp`:
- **Platform**:
  - `get_core_id()`, `early_init()`, `core_early_init()`, `panic()`
  - Subsystem ops pointers: `get_uart_ops()`, `get_irq_ops()`, etc.
- **UARTDriverOps**: Serial I/O (`putc`, `puts`, `uart_put_uint64_hex`, `getc_blocking`).
- **IRQControllerOps**: Interrupt handling (`enable_core_irqs`, `init_distributor`, `ack_irq`).
- **TimerDriverOps**: System timers (`init_system_timer_properties`, `get_system_time_us`, `hardware_timer_irq_fired`).
- **DMAControllerOps**: DMA transfers (`request_channel`, `configure_and_start_transfer`).
- **I2SDriverOps**: Audio I2S (`init`, `start`, `get_buffer_for_app_tx`, format conversions).
- **MemoryOps**: Cache management (`flush_cache_range`, `invalidate_cache_range`).
- **NetworkDriverOps**: Networking (`send_packet`, `receive_packet`).
- **PowerOps**: Power management (`enter_idle_state`, `set_cpu_frequency`).
- **GPIODriverOps**: GPIO pins (`configure_pin`, `set_pin`, `read_pin`, `enable_interrupt`).
- **WatchdogOps**: Watchdog timers (`start_watchdog`, `reset_watchdog`).

Example:
```cpp
struct MyPlatform : Platform {
    MyUARTDriverOps uart_ops;
    // ... other ops
    uint32_t get_core_id() const override { /* Read core ID */ }
    UARTDriverOps* get_uart_ops() override { return &uart_ops; }
    void early_init() override { /* Initialize UART, interrupts */ }
    void core_early_init() override { /* Per-core setup */ }
    void panic(const char* msg, const char* file, int line) override { /* Halt */ }
};
```

### 3. Implement SIMD DSP Nodes
For SIMD-capable platforms (e.g., NEON, RVV), implement a `DSPNodeFactory` in the HAL:
```cpp
struct DSPNodeFactory {
    static kernel::DSPNode* create_gain_node(float gain, const char* name) {
        return new MyGainDSP_SIMD(gain, name);
    }
};

class MyGainDSP_SIMD : public kernel::DSPNode {
public:
    MyGainDSP_SIMD(float g, const char* n) : DSPNode(n), gain(g) {}
    void process(std::span<float> buffer) override { /* SIMD code */ }
    void configure(const char* args, hal::UARTDriverOps*) override;
private:
    float gain;
};
```

### 4. Update Boot Code
Create `cpu_myplatform.S` for boot and context-switching, similar to `cpu_arm64.S` or `cpu_rv64.S`.

### 5. Update Linker Script
Create `linker_myplatform.ld` defining memory layout, based on `linker_arm64.ld` or `linker_rv64ima.ld`.

### 6. Update Makefile
Add platform configuration:
```makefile
# MyPlatform configuration
CC = myplatform-gcc
AS = myplatform-gcc
LD = myplatform-ld
OBJCOPY = myplatform-objcopy
CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -march=myarch -std=c++20 -I.
ASFLAGS = -march=myarch
LDFLAGS = -T linker_myplatform.ld
OBJ = miniOS_v1.6.o util_cpp20.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_myplatform.o cpu_myplatform.o gpio.o
```

### 7. Update Kernel
In `miniOS_v1.6.cpp`, instantiate your platform:
```cpp
g_platform = new hal::myplatform::MyPlatform();
```

### 8. Test and Debug
- Build: `make`
- Run in emulator or hardware.
- Use `test` CLI command to validate subsystems.
- Debug with GDB (`make debug`).

## Key Considerations
- **Memory Map**: Define MMIO addresses (e.g., `UART_BASE`, `GPIO_BASE`) in HAL header.
- **Interrupts**: Map IRQs (e.g., `GPIO_IRQ`) to your controller.
- **SIMD**: Implement platform-specific DSP nodes if supported.
- **GPIO**: Support 4 banks x 64 pins with interrupts.
- **Watchdog**: Implement hardware-specific timeout/reset.
- **Networking**: Ensure low-latency DMA for audio streaming.
- **DSP**: Support 4-way crossover with Butterworth/Bessel/Linkwitz-Riley filters.

## Testing
Run `test` to verify:
- GPIO configuration/read/write
- Audio streaming (requires network)
- DSP nodes (crossover, reverb, etc.)
- RAMFS directories and permissions
- Thread tracing

## Documentation
Update `Doxyfile` INPUT to include your HAL files:
```bash
make docs
```

## Resources
- QEMU HALs: `hal_qemu_arm64.hpp/cpp`, `hal_qemu_rv64ima.hpp/cpp`
- Kernel: `miniOS_v1.6.hpp/cpp`
- Subsystems: `dsp.hpp/cpp`, `net.hpp/cpp`, `gpio.hpp/cpp`, `fs.hpp/cpp`