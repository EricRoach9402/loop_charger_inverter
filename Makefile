###############################################################################
# Makefile for loop_charger_inverter
#
# Supports cross-compilation for x86 and ARM platforms.
# Build output is placed in x86/ or arm/ respectively.
#
# Usage
# ─────
# 1. Build for x86 (host):
#      make ARCH=x86
#
# 2. Build for ARM (requires aarch64-linux-gnu-gcc on PATH):
#      sudo apt install gcc-aarch64-linux-gnu   # if not installed
#      make ARCH=arm
#
# 3. Debug build:
#      make ARCH=x86 DEBUG=DEBUG
#
# 4. Clean all output:
#      make clean
#
# 5. Rebuild from scratch:
#      make ARCH=<x86|arm> rebuild
#
# Notes
# ─────
# - ARM uses the system aarch64-linux-gnu-gcc cross-compiler.
# - json-c must be installed for the target (libjson-c-dev:arm64 on Debian/Ubuntu).
###############################################################################

# ── Build options ─────────────────────────────────────────────────────────
ARCH  ?= x86
DEBUG ?= RELEASE

ARCH  := $(strip $(ARCH))
DEBUG := $(strip $(DEBUG))

# ── Target binary name ────────────────────────────────────────────────────
TARGET = loop_charger_inverter

# ── Source files ──────────────────────────────────────────────────────────
# Project-specific sources
SRCS = \
	src/main.c \
	src/config_loader.c \
	src/inverter_module.c \
	src/inverter_cmos_bridge.c \
	src/inverter_alarm.c \
	src/inverter_alarm_manager.c \
	devices/inverter/inverter_map.c

# Generic library sources (portable across projects)
SRCS += \
	lib/alarm/alarm_engine.c \
	lib/alarm/alarm_bridge.c \
	lib/cmos/cmos_pub.c \
	lib/cmos/cmos_sub.c \
	lib/modbus/modbus_rtu_client.c \
	lib/device_map/device_register_map.c \
	lib/log/log.c \
	lib/sqlite3/sqlite3.c

OBJS = $(SRCS:.c=.o)

# ── ARM toolchain ─────────────────────────────────────────────────────────
ARM_CC          = aarch64-linux-gnu-gcc
ARM_LIBJSONC_A  = lib/arm64/libjson-c.a

# ── Common flags ──────────────────────────────────────────────────────────
CFLAGS_COMMON  = -Wall -Wextra -std=c11 -D_GNU_SOURCE
CFLAGS_COMMON += -Iinclude -Idevices
CFLAGS_COMMON += -Ilib/alarm -Ilib/cmos -Ilib/modbus -Ilib/device_map -Ilib/log -Ilib/sqlite3

# x86: link against system libjson-c dynamically
LDLIBS_X86 = -lpthread -ljson-c -ldl

ifeq ($(DEBUG), DEBUG)
    CFLAGS_COMMON += -g -O0 -DDEBUG_MODE
else
    CFLAGS_COMMON += -O2
endif

# ── Architecture-specific settings ────────────────────────────────────────
ifeq ($(ARCH), x86)
    CC         = gcc
    CFLAGS     = $(CFLAGS_COMMON)
    LDFLAGS    =
    LDLIBS     = $(LDLIBS_X86)
    OUTPUT_DIR = x86

else ifeq ($(ARCH), arm)
    CC         = $(ARM_CC)
    CFLAGS     = $(CFLAGS_COMMON)
    LDFLAGS    =
    # Link libjson-c statically from the bundled prebuilt; system may not have arm64 package.
    LDLIBS     = -lpthread -ldl $(ARM_LIBJSONC_A)
    OUTPUT_DIR = arm

else
    $(error Unsupported ARCH "$(ARCH)". Use ARCH=x86 or ARCH=arm.)
endif

# ── Build rules ───────────────────────────────────────────────────────────
.PHONY: all clean rebuild

all: $(OUTPUT_DIR)/$(TARGET)

# Link: object files come BEFORE -l flags so the linker resolves symbols correctly.
$(OUTPUT_DIR)/$(TARGET): $(OBJS)
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS)
	rm -rf x86 arm

rebuild: clean all
