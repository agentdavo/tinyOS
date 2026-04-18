# SPDX-License-Identifier: MIT OR Apache-2.0
# miniOS — top-level Makefile. Selects a HAL under hal/$(TARGET)/.
#
# Usage:
#   make                        # builds arm64 (default)
#   make TARGET=riscv64
#   make run / make debug / make docs / make clean

TARGET ?= arm64
BUILD_DIR := build
HAL_DIR := hal/$(TARGET)
OBJDIR  := $(BUILD_DIR)/$(TARGET)
ELF     := $(OBJDIR)/miniOS_kernel_$(TARGET).elf
MAP     := $(OBJDIR)/miniOS_kernel_$(TARGET).elf.map

# FAKE_SLAVE=1 (default for arm64): builds the in-kernel FakeIO slave on
# NIC 2, replacing ec_b on core 3. Define MINIOS_FAKE_SLAVE=1 in CFLAGS so
# core.cpp picks the alternate thread wiring and cli.cpp registers the
# `fake` / `fake_set` commands. Set FAKE_SLAVE=0 to disable and bring back
# the symmetric two-master layout.
FAKE_SLAVE ?= 1
QEMU_EC_PORT ?= 20001

# Kernel-wide objects (platform-independent).
CORE_CPP = core.cpp hal.cpp util.cpp trace.cpp cli.cpp kernel_globals.cpp kernel/usb/usb.cpp \
           kernel/main.cpp \
           cpp_runtime_stubs.cpp freestanding_stubs.cpp \
           ethercat/master.cpp ethercat/frame.cpp ethercat/esm.cpp \
           ethercat/pdo.cpp ethercat/bus_config.cpp \
           motion/motion.cpp config/tsv.cpp \
           devices/device_db.cpp devices/embedded.cpp \
            diag/histogram.cpp diag/jitter.cpp diag/cpu_load.cpp \
            rt/base_thread.cpp \
ui/fb.cpp ui/splash.cpp ui/display.cpp ui/operator_api.cpp \
             ui/ui_builder_tsv.cpp machine/machine_registry.cpp machine/machine_topology.cpp machine/runtime_placement.cpp machine/motion_wiring.cpp automation/macro_runtime.cpp automation/ladder_runtime.cpp automation/probe_runtime.cpp machine/toolpods.cpp hmi/hmi_service.cpp \
              cnc/offsets.cpp cnc/programs.cpp cnc/interpreter.cpp cnc/mdi.cpp \
             render/gles1.cpp render/machine_model.cpp render/kinematic_model.cpp \
             render/obj_importer.cpp render/obj_registry.cpp render/stl_importer.cpp \
             render/benchmark.cpp fs/vfs.cpp fs/fat32.cpp fs/fs_fat32.cpp

# Binary blobs embedded via .incbin — built as a separate .S file so the raw
# TSV bytes get a predictable start/end symbol pair in .rodata.
CORE_S_EXTRA =
ifeq ($(TARGET),arm64)
    CORE_S_EXTRA += devices/embedded_blob.S devices/embedded_ui.S devices/embedded_kinematics.S devices/embedded_kinematic_obj.S devices/embedded_automation.S devices/embedded_signals.S devices/embedded_topology.S devices/embedded_placement.S devices/embedded_hmi.S
endif

ifeq ($(TARGET),riscv64)
    CORE_S_EXTRA += devices/embedded_ui.S devices/embedded_kinematics.S devices/embedded_kinematic_obj.S devices/embedded_automation.S devices/embedded_signals.S devices/embedded_topology.S devices/embedded_placement.S devices/embedded_hmi.S
endif

ifeq ($(TARGET),arm64)
ifeq ($(FAKE_SLAVE),1)
    CORE_CPP += ethercat/fake_slave.cpp
endif
endif

# ----- arm64 -----
ifeq ($(TARGET),arm64)
    CROSS      = aarch64-linux-gnu-
    # CPU tuning: schedule for cortex-a53 (conservative), but allow ARMv8.1-A
    # ISA extensions — notably LSE atomics (single-instruction CAS, SWPAL,
    # LDADDAL replacing LDXR/STXR retry loops). QEMU `-cpu max` has LSE.
    # `+simd` keeps NEON; `+lse` is the big win for spinlocks + histogram
    # counters. We schedule for cortex-a53 because that's our conservative
    # target for real hardware; upgrading march doesn't break that.
    CPU_FLAGS  = -mcpu=cortex-a53 -march=armv8-a+simd+lse
    CPU_S      = $(HAL_DIR)/cpu_arm64.S
    HAL_CPP    = $(HAL_DIR)/hal_qemu_arm64.cpp hal/shared/virtio_net.cpp \
                 hal/shared/virtio_gpu.cpp hal/shared/virtio_blk.cpp \
                 hal/shared/e1000.cpp hal/shared/pci.cpp hal/shared/xhci.cpp hal/shared/sdcard.cpp \
                 hal/shared/virtio_input.cpp
    LINKER     = $(HAL_DIR)/linker.ld
    # Disable libgcc outline-atomics. Not needed now LSE is mandated by
    # -march, and its discovery ctor reads a nonexistent aux vector.
    EXTRA_CF   = -mno-outline-atomics
    # Bare-metal arm64 must be a fully resolved static ELF. If ld is left to
    # its defaults it emits PIE/dynamic metadata plus RELATIVE relocations for
    # vtables and .init_array, but no loader ever applies them at boot.
    EXTRA_LDFLAGS = -static -no-pie \
                    -Wl,--no-dynamic-linker -Wl,--build-id=none
    QEMU_SYS   = qemu-system-aarch64
    # `virtio-mmio.force-legacy=false` makes QEMU use transport version 2, which
    # our virtio_net driver implements.
    #
    # NIC layout: eth0 is the HMI/user-network NIC, while eth1/eth2 are a raw
    # L2 socket pair reserved for EtherCAT traffic. This matches the default
    # runtime placement profile:
    #   eth0 -> HMI on core0
    #   eth1 -> EtherCAT master A on core2
    #   eth2 -> EtherCAT master B or fake slave on core3
    QEMU_ARGS  = -M virt -cpu max -smp 4 -m 128M \
                 -global virtio-mmio.force-legacy=false \
                 -netdev user,id=n0 \
                 -device virtio-net-device,netdev=n0 \
                 -netdev socket,id=n1,listen=:$(QEMU_EC_PORT) \
                 -device virtio-net-device,netdev=n1 \
                 -netdev socket,id=n2,connect=127.0.0.1:$(QEMU_EC_PORT) \
                 -device virtio-net-device,netdev=n2 \
                  -device virtio-gpu-device \
                  -device virtio-keyboard-device \
                  -device virtio-tablet-device \
                  -drive if=none,file=sdcard.img,format=raw,id=miniosblk \
                  -device virtio-blk-device,drive=miniosblk
# Note: `-device qemu-xhci,id=xhci` was removed from the default arm64 QEMU
# args because it interacts badly with virtio-blk-device under QEMU (the
# USB/PCI probe hangs). USB isn't used by the operator stack; add it back
# explicitly when running a USB-focused test.
endif

# ----- riscv64 -----
ifeq ($(TARGET),riscv64)
    CROSS      = riscv64-linux-gnu-
    CPU_FLAGS  = -march=rv64imafdc -mabi=lp64d -mcmodel=medany
    CPU_S      = $(HAL_DIR)/cpu_rv64.S
    HAL_CPP    = $(HAL_DIR)/hal_qemu_rv64.cpp $(HAL_DIR)/rv64_stubs.cpp \
                 $(HAL_DIR)/rv64_sched.cpp \
                 hal/shared/virtio_net.cpp hal/shared/virtio_gpu.cpp \
                 hal/shared/e1000.cpp hal/shared/pci.cpp hal/shared/xhci.cpp \
                 hal/shared/virtio_input.cpp
    LINKER     = $(HAL_DIR)/linker.ld
    # -fno-pic/-fno-pie stops the compiler from emitting GOT-indirect
    # address-of-symbol sequences. On a freestanding kernel nothing
    # populates the GOT at runtime, so `auipc+ld` from a zero slot would
    # jump to address 0 and hang silently.
    EXTRA_CF   = -fno-pic -fno-pie
    # -static -no-pie + --no-dynamic-linker force a fully-resolved
    # static ELF. Without them ld emits .dynamic/.rela.dyn and the
    # virtual-function tables in globals (e.g. g_platform_instance) get
    # left as runtime relocations that nothing ever applies, so the
    # vptr reads as zero and the first virtual call loads from 0x18.
    EXTRA_LDFLAGS = -static -no-pie \
                    -Wl,--no-dynamic-linker -Wl,--build-id=none
    QEMU_SYS   = qemu-system-riscv64
    # `virtio-mmio.force-legacy=false` makes QEMU expose transport version 2
    # on rv64 virt — same story as arm64 above. Our shared virtio_net driver
    # rejects legacy (v1) transports, so without this flag the scanner finds
    # the device MMIO window but init() bails at the version check and
    # `discover_virtio_net` returns 0. Older QEMU defaults to legacy on
    # rv64 virt even though arm64 virt was flipped a while back.
    QEMU_ARGS  = -M virt -cpu rv64 -smp 4 -m 128M -bios none \
                 -global virtio-mmio.force-legacy=false \
                 -netdev user,id=n0 \
                 -device virtio-net-device,netdev=n0 \
                 -netdev socket,id=n1,listen=:$(QEMU_EC_PORT) \
                 -device virtio-net-device,netdev=n1 \
                 -netdev socket,id=n2,connect=127.0.0.1:$(QEMU_EC_PORT) \
                 -device virtio-net-device,netdev=n2 \
                 -device virtio-gpu-device \
                 -device virtio-keyboard-device \
                 -device virtio-tablet-device \
                 -device qemu-xhci,id=xhci
    # rv64 now links shared kernel components: kernel/main.cpp + core.cpp 
    # (the scheduler). TCB_Rv64 from rv64_sched.cpp is used via the
    # cpu_context_switch_impl shim in rv64_stubs.cpp. The shared scheduler
    # is invoked via kernel_main() called from kernel_main_rv64() after HAL init.
    # rv64: excludes hal.cpp (provides get_platform wrapper - rv64 has its own)
#        also excludes embedded_blob.S (arm64-only TSV blobs)
CORE_CPP  = kernel/main.cpp core.cpp util.cpp trace.cpp cli.cpp kernel_globals.cpp kernel/usb/usb.cpp \
                cpp_runtime_stubs.cpp freestanding_stubs.cpp \
                ethercat/master.cpp ethercat/frame.cpp ethercat/esm.cpp \
                ethercat/pdo.cpp ethercat/bus_config.cpp ethercat/fake_slave.cpp \
                motion/motion.cpp config/tsv.cpp \
                devices/device_db.cpp devices/embedded.cpp \
                diag/histogram.cpp diag/jitter.cpp diag/cpu_load.cpp \
                rt/base_thread.cpp \
                ui/fb.cpp ui/splash.cpp ui/display.cpp ui/operator_api.cpp \
                ui/ui_builder_tsv.cpp machine/machine_registry.cpp machine/machine_topology.cpp machine/runtime_placement.cpp machine/motion_wiring.cpp automation/macro_runtime.cpp automation/ladder_runtime.cpp automation/probe_runtime.cpp machine/toolpods.cpp hmi/hmi_service.cpp \
                cnc/offsets.cpp cnc/programs.cpp cnc/interpreter.cpp cnc/mdi.cpp \
                render/gles1.cpp render/machine_model.cpp render/kinematic_model.cpp \
                render/obj_importer.cpp render/obj_registry.cpp render/stl_importer.cpp render/benchmark.cpp \
                fs/vfs.cpp fs/fat32.cpp fs/fs_fat32.cpp
endif

ifndef CPU_S
    $(error Invalid TARGET: use 'arm64' or 'riscv64')
endif

CC       = $(CROSS)g++
AS       = $(CROSS)gcc
LD       = $(CROSS)g++
OBJCOPY  = $(CROSS)objcopy
READELF  = $(CROSS)readelf

# DEBUG=1 builds with -O0 + symbols for gdb; DEBUG=0 builds with -O2 + LTO for
# real cycle-time measurement. Default: production (DEBUG=0).
DEBUG ?= 0
ifeq ($(DEBUG),1)
    OPT_FLAGS = -g -O0
else
    # -flto is tempting but the freestanding memset/memcpy stubs don't survive
    # cross-TU LTO unless explicitly preserved. Skip it; plain -O2 already
    # gets ~10× vs -O0 and the link is cleaner.
    OPT_FLAGS = -g -O2 -fno-plt
endif

CFLAGS  = $(OPT_FLAGS)
CFLAGS += -Wall -Wextra -fno-exceptions -fno-rtti -fno-PIE -Woverloaded-virtual
CFLAGS += -ffreestanding -ffunction-sections -fdata-sections
CFLAGS += -fno-stack-protector -fno-omit-frame-pointer
CFLAGS += -Wshadow -Wcast-align -Wdouble-promotion
CFLAGS += -std=c++20
CFLAGS += $(CPU_FLAGS) $(EXTRA_CF)
CFLAGS += -I. -I$(HAL_DIR)

# Surface MINIOS_FAKE_SLAVE to all TUs so core.cpp + cli.cpp pick up the
# alternate slave wiring. Always defined (0/1) so #if works without #ifdef.
CFLAGS += -DMINIOS_FAKE_SLAVE=$(FAKE_SLAVE)

# Git hash stamped into the `version` CLI output. Falls back to
# "unknown" when git is unavailable (e.g. tarball build). The `+dirty`
# suffix flags uncommitted changes — useful when debugging what's on
# the metal vs what's on the branch.
GIT_HASH := $(shell (git -C $(abspath $(dir $(lastword $(MAKEFILE_LIST)))) rev-parse --short HEAD 2>/dev/null || echo unknown) | tr -d '\n')
GIT_DIRTY := $(shell git -C $(abspath $(dir $(lastword $(MAKEFILE_LIST)))) diff --quiet 2>/dev/null || echo +dirty)
CFLAGS += -DMINIOS_GIT_HASH=\"$(GIT_HASH)$(GIT_DIRTY)\"

ASFLAGS = $(CPU_FLAGS) -g3

LDFLAGS = $(OPT_FLAGS) -T $(LINKER) -nostartfiles -nostdlib \
          -Wl,--no-relax -Wl,-z,separate-code \
          -Wl,-z,now -Wl,-z,noexecstack \
          -Wl,--gc-sections \
          -Wl,--no-warn-rwx-segments $(EXTRA_LDFLAGS)
LIBS    = -Wl,--start-group -lgcc -Wl,--end-group

# Object list: cpu.S + HAL .cpp + kernel .cpps + extra .S. All flattened to top-level .o.
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(CPU_S:.S=.o)) $(notdir $(HAL_CPP:.cpp=.o)) $(notdir $(CORE_CPP:.cpp=.o)) \
      $(notdir $(CORE_S_EXTRA:.S=.o)))

all: $(ELF)
	@echo "Built $(ELF)"

$(ELF): $(OBJ) $(LINKER)
	@echo "--- Linking $(ELF) ($(TARGET)) ---"
	$(LD) $(LDFLAGS) -Wl,-Map=$(MAP) -o $@ $(OBJ) $(LIBS)

# Pattern rules — match both hal/$arch/ sources and root sources.
$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: %.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: %.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -x assembler-with-cpp -c -o $@ $<

$(OBJDIR)/%.o: $(HAL_DIR)/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: $(HAL_DIR)/%.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -x assembler-with-cpp -c -o $@ $<

# Shared (arch-agnostic) HAL sources live under hal/shared/.
$(OBJDIR)/%.o: hal/shared/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: kernel/usb/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: hmi/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: machine/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: automation/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: ui/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJDIR)/virtio_blk.o: hal/shared/virtio_blk.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/fat32.o: fs/fat32.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/fs_fat32.o: fs/fs_fat32.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/virtio_net.o: hal/shared/virtio_net.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

# Flattened-name rules for subsystems in subdirs (ethercat/, motion/, config/).
$(OBJDIR)/master.o: ethercat/master.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/frame.o: ethercat/frame.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/esm.o: ethercat/esm.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/embedded_ui.o: devices/embedded_ui.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_automation.o: devices/embedded_automation.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_signals.o: devices/embedded_signals.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_topology.o: devices/embedded_topology.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_placement.o: devices/embedded_placement.S | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_hmi.o: devices/embedded_hmi.S devices/embedded_hmi.tsv | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/pdo.o: ethercat/pdo.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/bus_config.o: ethercat/bus_config.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/fake_slave.o: ethercat/fake_slave.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/main.o: kernel/main.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/motion.o: motion/motion.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/tsv.o: config/tsv.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/device_db.o: devices/device_db.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/embedded.o: devices/embedded.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/histogram.o: diag/histogram.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/jitter.o: diag/jitter.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/cpu_load.o: diag/cpu_load.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/base_thread.o: rt/base_thread.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/fb.o: ui/fb.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/display.o: ui/display.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/operator_api.o: ui/operator_api.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/offsets.o: cnc/offsets.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/programs.o: cnc/programs.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/interpreter.o: cnc/interpreter.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/mdi.o: cnc/mdi.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/splash.o: ui/splash.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/gles1.o: render/gles1.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/machine_model.o: render/machine_model.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/kinematic_model.o: render/kinematic_model.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/obj_importer.o: render/obj_importer.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/obj_registry.o: render/obj_registry.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/stl_importer.o: render/stl_importer.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/vfs.o: fs/vfs.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/benchmark.o: render/benchmark.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
# .incbin resolves paths relative to the assembler's include search paths
# (-I). The rule runs from the project root, so `devices/clearpath_ec.tsv`
# referenced inside embedded_blob.S resolves directly.
$(OBJDIR)/embedded_blob.o: devices/embedded_blob.S devices/clearpath_ec.tsv | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_kinematics.o: devices/embedded_kinematics.S machines/kinematic_mill3.tsv machines/kinematic_millturn.tsv machines/kinematic_mx850.tsv | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<
$(OBJDIR)/embedded_kinematic_obj.o: devices/embedded_kinematic_obj.S devices/demo_box.obj devices/demo_part.stl | $(OBJDIR)
	$(AS) $(ASFLAGS) -I. -x assembler-with-cpp -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR) miniOS_kernel_*.elf miniOS_kernel_*.elf.map qemu.pid serial_core0.log

run: $(ELF)
	$(QEMU_SYS) $(QEMU_ARGS) -nographic -kernel $(ELF)

debug: $(ELF)
	@rm -f qemu.pid
	$(QEMU_SYS) $(QEMU_ARGS) -nographic -kernel $(ELF) -S -gdb tcp::1234 &
	@echo $$! > qemu.pid
	@sleep 2
	gdb-multiarch $(ELF) -x gdb_init.gdb || true
	@if [ -f qemu.pid ]; then kill `cat qemu.pid` 2>/dev/null; rm -f qemu.pid; fi

docs:
	doxygen Doxyfile

.PHONY: all clean run debug docs
