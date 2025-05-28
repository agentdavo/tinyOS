# SPDX-License-Identifier: MIT OR Apache-2.0
# Makefile - Build automation for miniOS v1.7 (ARM64 and RISC-V)
# Supports QEMU ARM64 and RISC-V virt machines with GPIO, networking, audio, and scripting
# @version 1.7

TARGET ?= arm64
DEBUG_PORT ?= 1234 # QEMU will listen on this port

ifeq ($(TARGET),arm64)
	CC = aarch64-linux-gnu-g++
	AS = aarch64-linux-gnu-gcc
	LD = aarch64-linux-gnu-g++
	OBJCOPY = aarch64-linux-gnu-objcopy
	CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -mcpu=cortex-a53 -march=armv8-a+simd -std=c++20 -I. -fno-PIE -Woverloaded-virtual -g3
	ASFLAGS = -mcpu=cortex-a53 -march=armv8-a -g3
	LDFLAGS = -T ld/linker_arm64.ld -no-pie -nostartfiles -Wl,--no-relax -L/usr/aarch64-linux-gnu/lib -L/usr/lib/aarch64-linux-gnu -lc -lgcc -lstdc++
	QEMU_SYSTEM = qemu-system-aarch64
	QEMU_ARGS = -M virt -cpu cortex-a53 -smp 4 -m 128M -nographic -kernel
	KERNEL_OBJ = core.o hal.o util.o trace.o cli_minimal.o kernel_globals.o cpu_arm64.o hal_qemu_arm64.o
	OBJ = $(KERNEL_OBJ) 
else ifeq ($(TARGET),riscv64)
	CC = riscv64-linux-gnu-g++
	AS = riscv64-linux-gnu-gcc
	LD = riscv64-linux-gnu-g++
	OBJCOPY = riscv64-linux-gnu-objcopy
	CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -march=rv64imafdc -mabi=lp64 -std=c++20 -I. -fno-PIE -Woverloaded-virtual -g3
	ASFLAGS = -march=rv64imafdc -mabi=lp64 -g3
	LDFLAGS = -T ld/linker_rv64ima.ld -no-pie -nostartfiles -Wl,--no-relax -L/usr/riscv64-linux-gnu/lib -L/usr/lib/riscv64-linux-gnu -lc -lgcc -lstdc++
	QEMU_SYSTEM = qemu-system-riscv64
	QEMU_ARGS = -M virt -cpu rv64 -smp 4 -m 128M -nographic -kernel
	KERNEL_OBJ = core.o hal.o util.o trace.o cli_minimal.o kernel_globals.o cpu_rv64.o hal_qemu_rv64ima.o
	OBJ = $(KERNEL_OBJ)
else
	$(error Invalid TARGET: use 'arm64' or 'riscv64')
endif

TARGET_ELF_NAME = miniOS_kernel.elf
QEMU_NET_ARGS = -netdev user,id=net0 -device virtio-net-device,netdev=net0
GDB_SCRIPT_FILE = gdb_init.gdb # Define the GDB script file name

all: $(TARGET_ELF_NAME)
	@echo "Build complete. ELF file: $(TARGET_ELF_NAME)"

kernel: $(TARGET_ELF_NAME)

$(TARGET_ELF_NAME): $(KERNEL_OBJ)
	$(LD) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(AS) $(ASFLAGS) -c -o $@ $<

clean:
	@echo "Cleaning up build files..."
	rm -f *.o $(TARGET_ELF_NAME) miniOS.bin miniOS.elf qemu.pid # No gdb_commands.txt to remove
	@echo "Cleanup finished. Manual QEMU cleanup might be needed if instances were running."


run: $(TARGET_ELF_NAME)
	$(QEMU_SYSTEM) $(QEMU_ARGS) $< $(QEMU_NET_ARGS)

# Use .ONESHELL for recipes that need shell variables to persist across lines.
.ONESHELL:
debug: $(TARGET_ELF_NAME)
	@echo "IMPORTANT: Manual QEMU cleanup might be needed if old instances are running on port $(DEBUG_PORT)."
	@echo "You can try: pkill -f \"$(QEMU_SYSTEM)\" or check processes manually."
	
	@echo "Starting QEMU for debugging $(TARGET_ELF_NAME) on port $(DEBUG_PORT)..."
	# Start QEMU in the background and save its PID to a file
	$(QEMU_SYSTEM) $(QEMU_ARGS) $< -S -gdb tcp::$(DEBUG_PORT) $(QEMU_NET_ARGS) &
	echo $$! > qemu.pid # Save PID of QEMU
	
	@echo "QEMU started (PID `cat qemu.pid`). Waiting for GDB connection (2s)..."; 
	sleep 2; # Give QEMU time to start listening
	
	@echo "Starting GDB with script $(GDB_SCRIPT_FILE) and connecting to localhost:$(DEBUG_PORT)..."
	# GDB will take commands from GDB_SCRIPT_FILE. 
	# Ensure GDB_SCRIPT_FILE has "target remote localhost:$(DEBUG_PORT)"
	# If DEBUG_PORT in Makefile needs to propagate to gdb_init.gdb, that's more complex.
	# For now, gdb_init.gdb has port 1234 hardcoded. Ensure they match.
	gdb-multiarch $< -x $(GDB_SCRIPT_FILE)
	
	@echo "GDB session ended."
	if [ -f qemu.pid ]; then \
		QEMU_PID_VAL=`cat qemu.pid`; \
		echo "Attempting to kill QEMU PID $$QEMU_PID_VAL..."; \
		if ps -p $$QEMU_PID_VAL > /dev/null; then \
			kill $$QEMU_PID_VAL; \
			echo "QEMU PID $$QEMU_PID_VAL killed."; \
		else \
			echo "QEMU PID $$QEMU_PID_VAL already exited or was not found."; \
		fi; \
		rm -f qemu.pid; \
	else \
		echo "qemu.pid not found. Manual QEMU cleanup might be needed if it's still running."; \
	fi

docs:
	doxygen Doxyfile

.PHONY: all kernel clean run debug docs