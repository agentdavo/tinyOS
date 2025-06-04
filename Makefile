# SPDX-License-Identifier: MIT OR Apache-2.0
# Makefile - Build automation for miniOS v1.7 (ARM64)
# Bare-metal build with freestanding utilities

TARGET ?= arm64

ifeq ($(TARGET),arm64)
    CC = aarch64-linux-gnu-g++
    AS = aarch64-linux-gnu-gcc
    LD = aarch64-linux-gnu-g++
    OBJCOPY = aarch64-linux-gnu-objcopy

    CFLAGS  = -g -O0
    CFLAGS += -Wall -Wextra -fno-exceptions -fno-rtti -mcpu=cortex-a53 -march=armv8-a+simd -std=c++20 -I. -fno-PIE -Woverloaded-virtual -ffreestanding
    CFLAGS += -ffunction-sections -fdata-sections -fno-stack-protector -fno-omit-frame-pointer -Wshadow -Wcast-align -Wdouble-promotion
	
    ASFLAGS = -mcpu=cortex-a53 -march=armv8-a -g3

    LINKER_SCRIPT = ld/linker_arm64.ld
    LDFLAGS_NO_LIBS = -T $(LINKER_SCRIPT) -nostartfiles -nostdlib \
                      -Wl,--no-relax -Wl,-z,separate-code \
					  -Wl,-z,now -Wl,-z,noexecstack
	
    LIBS = -Wl,--start-group -lgcc -Wl,--end-group

    QEMU_SYSTEM = qemu-system-aarch64
    QEMU_ARGS = -M virt -cpu cortex-a53 -smp 4 -m 128M
    KERNEL_OBJ = core.o hal.o util.o trace.o cli_minimal.o kernel_globals.o cpu_arm64.o hal_qemu_arm64.o cpp_runtime_stubs.o freestanding_stubs.o
else
    $(error Invalid TARGET: use 'arm64' or 'riscv64' - RISC-V settings need similar review)
endif

TARGET_ELF_NAME = miniOS_kernel.elf
MAP_FILE_NAME = $(TARGET_ELF_NAME).map # Consistent map file name
QEMU_KERNEL_OPT = -kernel
SERIAL_LOG_FILE = serial_core0.log
QEMU_SERIAL_CHARDEV = -chardev file,id=serial_chardev,path=$(SERIAL_LOG_FILE)
QEMU_SERIAL_OPTS = -serial chardev:serial_chardev
QEMU_DISPLAY_OPTS = -nographic
QEMU_NET_ARGS = -netdev user,id=net0 -device virtio-net-device,netdev=net0
DEBUG_PORT = 1234
GDB_SCRIPT_FILE = gdb_init.gdb

all: $(TARGET_ELF_NAME)
    @echo "Build complete. ELF file: $(TARGET_ELF_NAME)"

kernel: $(TARGET_ELF_NAME)

$(TARGET_ELF_NAME): $(KERNEL_OBJ) $(LINKER_SCRIPT) # Add linker script as a prerequisite
	@echo "--- Linking $(TARGET_ELF_NAME) ---"
	@echo "LD: $(LD)"
	@echo "Linker Script: $(LINKER_SCRIPT)"
	@echo "LDFLAGS_NO_LIBS (base): $(LDFLAGS_NO_LIBS)"
	@echo "KERNEL_OBJ: $(KERNEL_OBJ)"
	@echo "LIBS: $(LIBS)"
	$(LD) $(LDFLAGS_NO_LIBS) -Wl,-Map=$(MAP_FILE_NAME) -o $@ $(KERNEL_OBJ) $(LIBS)
	@echo "--- Linking finished ---"

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(AS) $(ASFLAGS) -x assembler-with-cpp -c -o $@ $<

clean:
	@echo "Cleaning up build files..."
	rm -f *.o $(TARGET_ELF_NAME) $(MAP_FILE_NAME) qemu.pid $(SERIAL_LOG_FILE)

run: $(TARGET_ELF_NAME)
	@echo "Running $(TARGET_ELF_NAME). Serial output will be in $(SERIAL_LOG_FILE)."
	$(QEMU_SYSTEM) $(QEMU_ARGS) $(QEMU_KERNEL_OPT) $< \
		$(QEMU_SERIAL_CHARDEV) $(QEMU_SERIAL_OPTS) \
		$(QEMU_DISPLAY_OPTS) $(QEMU_NET_ARGS)

.ONESHELL:
debug: $(TARGET_ELF_NAME)
	@echo "IMPORTANT: Manual QEMU cleanup might be needed if old instances are running on port $(DEBUG_PORT)."
	@echo "           You can try: pkill -f \"$(QEMU_SYSTEM)\" or check processes manually."
	@echo "           Serial output from kernel will be logged to: $(SERIAL_LOG_FILE)"
	@rm -f $(SERIAL_LOG_FILE) qemu.pid # Also remove qemu.pid if it exists

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