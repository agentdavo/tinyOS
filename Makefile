# SPDX-License-Identifier: MIT OR Apache-2.0
# Makefile - Build automation for miniOS v1.6 (ARM64 and RISC-V)
# Supports QEMU ARM64 and RISC-V virt machines with GPIO, networking, audio, and scripting
# @version 1.6

# Default to ARM64; override for RISC-V
CC = aarch64-linux-gnu-g++
AS = aarch64-linux-gnu-gcc
LD = aarch64-linux-gnu-ld
OBJCOPY = aarch64-linux-gnu-objcopy
CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -mcpu=cortex-a53 -march=armv8-a+simd -std=c++20 -I.
ASFLAGS = -mcpu=cortex-a53 -march=armv8-a
LDFLAGS = -T linker_arm64.ld
OBJ = miniOS.o util_cpp20.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_qemu_arm64.o cpu_arm64.o gpio.o
TARGET = miniOS

# RISC-V configuration (uncomment to use)
# CC = riscv64-linux-gnu-g++
# AS = riscv64-linux-gnu-gcc
# LD = riscv64-linux-gnu-ld
# OBJCOPY = riscv64-linux-gnu-objcopy
# CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -march=rv64imafdv -mabi=lp64 -std=c++20 -I.
# ASFLAGS = -march=rv64imafdv -mabi=lp64
# LDFLAGS = -T linker_rv64ima.ld
# OBJ = miniOS_v1.6.o util_cpp20.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_qemu_rv64ima.o cpu_rv64.o gpio.o

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(AS) $(ASFLAGS) -c -o $@ $<

clean:
	rm -f *.o $(TARGET).elf $(TARGET).bin

run: $(TARGET).bin
	qemu-system-aarch64 -M virt -cpu cortex-a53 -smp 4 -m 128M -nographic \
		-kernel $(TARGET).bin -netdev user,id=net0 -device virtio-net-device,netdev=net0
	# For RISC-V:
	# qemu-system-riscv64 -M virt -cpu rv64 -smp 4 -m 128M -nographic \
	#	-kernel $(TARGET).bin -netdev user,id=net0 -device virtio-net-device,netdev=net0

debug: $(TARGET).bin
	qemu-system-aarch64 -M virt -cpu cortex-a53 -smp 4 -m 128M -nographic \
		-kernel $(TARGET).bin -S -s -netdev user,id=net0 -device virtio-net-device,netdev=net0 &
	gdb-multiarch $(TARGET).elf -ex "target remote localhost:1234" \
		-ex "break kernel_main" -ex "continue"
	# For RISC-V:
	# qemu-system-riscv64 -M virt -cpu rv64 -smp 4 -m 128M -nographic \
	#	-kernel $(TARGET).bin -S -s -netdev user,id=net0 -device virtio-net-device,netdev=net0 &
	# gdb-multiarch $(TARGET).elf -ex "target remote localhost:1234" \
	#	-ex "break kernel_main" -ex "continue"

docs:
	doxygen Doxyfile