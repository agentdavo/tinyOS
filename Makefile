# SPDX-License-Identifier: MIT OR Apache-2.0
# Makefile - Build automation for miniOS v1.7 (ARM64)
# Bare-metal build with freestanding utilities

TARGET ?= arm64

ifeq ($(TARGET),arm64)
    CC = aarch64-linux-gnu-g++
    AS = aarch64-linux-gnu-gcc
    LD = aarch64-linux-gnu-g++
    OBJCOPY = aarch64-linux-gnu-objcopy

    # -ffreestanding implies that standard includes and libraries are not available
    # Use -O2 and -g3 for debug builds.
    CFLAGS = -O2 -Wall -Wextra -fno-exceptions -fno-rtti -mcpu=cortex-a53 -march=armv8-a+simd -std=c++20 -I. -fno-PIE -Woverloaded-virtual -g3 -ffreestanding
    ASFLAGS = -mcpu=cortex-a53 -march=armv8-a -g3

    # -nostartfiles and -nostdlib prevent linking system startup files and standard C++ library, respectively.
    # -lgcc provides compiler support functions (e.g., for 64-bit division, atomic ops).
    # -latomic is used to ensure all atomic operation functionality exists
    LDFLAGS = -T ld/linker_arm64.ld -nostartfiles -Wl,--no-relax -nostdlib -lgcc -latomic

    QEMU_SYSTEM = qemu-system-aarch64
    QEMU_ARGS = -M virt -cpu cortex-a53 -smp 4 -m 128M -nographic -kernel

    KERNEL_OBJ = core.o hal.o util.o trace.o cli_minimal.o kernel_globals.o cpu_arm64.o hal_qemu_arm64.o cpp_runtime_stubs.o freestanding_stubs.o
    OBJ = $(KERNEL_OBJ)

else
	$(error Invalid TARGET: use 'arm64')
endif

TARGET_ELF_NAME = miniOS_kernel.elf
QEMU_KERNEL_OPT = -kernel

# Chardev/Serial settings - Used to redirect serial output
SERIAL_LOG_FILE = serial_core0.log # File where the serial output will be logged
QEMU_SERIAL_CHARDEV = -chardev file,id=serial_chardev,path=$(SERIAL_LOG_FILE)
QEMU_SERIAL_OPTS = -serial chardev:serial_chardev # Tell QEMU to use the defined chardev for the serial port.
QEMU_DISPLAY_OPTS = -nographic # Sets to headless.
QEMU_NET_ARGS = -netdev user,id=net0 -device virtio-net-device,netdev=net0

# GDB Settings
DEBUG_PORT = 1234
GDB_SCRIPT_FILE = gdb_init.gdb

all: $(TARGET_ELF_NAME)
	@echo "Build complete. ELF file: $(TARGET_ELF_NAME)"

kernel: $(TARGET_ELF_NAME)

$(TARGET_ELF_NAME): $(KERNEL_OBJ)
	$(LD) $(LDFLAGS) -o $@ $(KERNEL_OBJ)

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(AS) $(ASFLAGS) -c -o $@ $<

clean:
	@echo "Cleaning up build files..."
	rm -f *.o $(TARGET_ELF_NAME) miniOS.bin miniOS.elf qemu.pid $(SERIAL_LOG_FILE)
	@echo "Cleanup finished. Manual QEMU cleanup might be needed if instances were running."

run: $(TARGET_ELF_NAME)
	@echo "Running $(TARGET_ELF_NAME). Serial output will be in $(SERIAL_LOG_FILE)."
	$(QEMU_SYSTEM) $(QEMU_ARGS) $(QEMU_KERNEL_OPT) $< \
		$(QEMU_SERIAL_CHARDEV) $(QEMU_SERIAL_OPTS) \
		$(QEMU_DISPLAY_OPTS) $(QEMU_NET_ARGS)

# Use a .ONESHELL:
debug: $(TARGET_ELF_NAME)
	@echo "Cleaning up old QEMU instances for target $(TARGET)..."
	@-pkill -f "$(QEMU_SYSTEM)" 2>/dev/null || true
	@echo "Serial output from kernel will be logged to: $(SERIAL_LOG_FILE)"
	@rm -f $(SERIAL_LOG_FILE)

	@echo "Starting QEMU for debugging $(TARGET_ELF_NAME) on port $(DEBUG_PORT)..."
	$(QEMU_SYSTEM) $(QEMU_ARGS) $(QEMU_KERNEL_OPT) $< \
		-S -gdb tcp::$(DEBUG_PORT) \
		$(QEMU_SERIAL_CHARDEV) $(QEMU_SERIAL_OPTS) \
		$(QEMU_DISPLAY_OPTS) $(QEMU_NET_ARGS) &
	echo $$! > qemu.pid

	@echo "QEMU started (PID `cat qemu.pid`). Waiting for GDB connection (3s)...";
	sleep 3;

	@echo "Starting GDB with script $(GDB_SCRIPT_FILE) and connecting to localhost:$(DEBUG_PORT)..."
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
	@echo "Serial log is in: $(SERIAL_LOG_FILE)"

docs:
	doxygen Doxyfile

.PHONY: all kernel clean run debug docs