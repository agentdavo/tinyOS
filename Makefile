# SPDX-License-Identifier: MIT OR Apache-2.0
# Makefile - Build automation for miniOS v1.7 (ARM64 and RISC-V)
# Supports QEMU ARM64 and RISC-V virt machines with GPIO, networking, audio, and scripting
# @version 1.7

TARGET ?= arm64
ifeq ($(TARGET),arm64)
	CC = aarch64-linux-gnu-g++
	AS = aarch64-linux-gnu-gcc
	LD = aarch64-linux-gnu-ld
	OBJCOPY = aarch64-linux-gnu-objcopy
	CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -mcpu=cortex-a53 -march=armv8-a+simd -std=c++20 -I.
	ASFLAGS = -mcpu=cortex-a53 -march=armv8-a
	LDFLAGS = -T ld/linker_arm64.ld
	QEMU = qemu-system-aarch64 -M virt -cpu cortex-a53 -smp 4 -m 128M -nographic -kernel
	OBJ = miniOS.o util.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_qemu_arm64.o cpu_arm64.o gpio.o
else ifeq ($(TARGET),riscv64)
	CC = riscv64-linux-gnu-g++
	AS = riscv64-linux-gnu-gcc
	LD = riscv64-linux-gnu-ld
	OBJCOPY = riscv64-linux-gnu-objcopy
	CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -march=rv64imafdc -mabi=lp64 -std=c++20 -I.
	ASFLAGS = -march=rv64imafdc -mabi=lp64
	LDFLAGS = -T ld/linker_rv64ima.ld
	QEMU = qemu-system-riscv64 -M virt -cpu rv64 -smp 4 -m 128M -nographic -kernel
	OBJ = miniOS.o util.o cli.o dsp.o audio.o demo_cli_dsp.o trace.o fs.o net.o test_framework.o hal_qemu_rv64ima.o cpu_rv64.o gpio.o
else
	$(error Invalid TARGET: use 'arm64' or 'riscv64')
endif

TARGET_NAME = miniOS

all: $(TARGET_NAME).bin

$(TARGET_NAME).bin: $(TARGET_NAME).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET_NAME).elf: $(OBJ)
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: hal/%.S
	$(AS) $(ASFLAGS) -c -o $@ $<

clean:
	rm -f *.o $(TARGET_NAME).elf $(TARGET_NAME).bin

run: $(TARGET_NAME).bin
	$(QEMU) $(TARGET_NAME).bin -netdev user,id=net0 -device virtio-net-device,netdev=net0

debug: $(TARGET_NAME).bin
	$(QEMU) $(TARGET_NAME).bin -S -s -netdev user,id=net0 -device virtio-net-device,netdev=net0 &
	gdb-multiarch $(TARGET_NAME).elf -ex "target remote localhost:1234" \
		-ex "break kernel_main" -ex "continue"

docs:
	doxygen Doxyfile

.PHONY: all clean run debug docs