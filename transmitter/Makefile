# Makefile for Node A (STM32F103C8T6 Blue Pill)
# Bare-metal transmitter & master hop controller for FHSS radio link

TARGET     ?= firmware
BUILD_DIR   = build
ALT_TARGET  = node_a

# Toolchain definitions
CC          = arm-none-eabi-gcc
OBJCOPY     = arm-none-eabi-objcopy
OBJDUMP     = arm-none-eabi-objdump
SIZE        = arm-none-eabi-size

# Host toolchain for unit tests
HOST_CC     = gcc

# Architecture and MCU compiler flags
MCU_FLAGS   = -mcpu=cortex-m3 -mthumb
WARN_FLAGS  = -Wall -Wextra -Werror
OPT_FLAGS   = -Os -ffunction-sections -fdata-sections
INCLUDES    = -I../common -Inode_a_f103

CFLAGS      = $(MCU_FLAGS) $(WARN_FLAGS) $(OPT_FLAGS) $(INCLUDES)
LDFLAGS     = $(MCU_FLAGS) -Tnode_a_f103/linker_f103.ld -Wl,--gc-sections -nostartfiles -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--no-warn-rwx-segments

# Target objects
OBJS        = $(BUILD_DIR)/status.o \
              $(BUILD_DIR)/config.o \
              $(BUILD_DIR)/log.o \
              $(BUILD_DIR)/sx1278_fsk.o \
              $(BUILD_DIR)/startup_f103.o \
              $(BUILD_DIR)/stm32_baremetal.o \
              $(BUILD_DIR)/node_a_tx.o

# Host test sources
HOST_SRCS   = ../common/status.c \
              ../common/config.c \
              ../common/log.c \
              ../common/sx1278_fsk.c \
              tests/host/mock_hal.c \
              tests/host/test_a0_host.c

HOST_CFLAGS = -Wall -Wextra -Werror -I../common -Itests/host

.PHONY: all clean flash test selftest_a0 selftest_a1 selftest_a2 selftest_spi_loopback selftest_pins selftest_tx_blind selftest_miso_static debug

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@cp -f $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Build complete ---"
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

selftest_a0: CFLAGS += -DSELFTEST_A0
selftest_a0: clean all

selftest_a1: CFLAGS += -DSELFTEST_A0 -DSELFTEST_A1
selftest_a1: clean all

selftest_a2: CFLAGS += -DSELFTEST_A0 -DSELFTEST_A2
selftest_a2: clean all

selftest_spi_loopback: CFLAGS += -DSELFTEST_SPI_LOOPBACK
selftest_spi_loopback: clean all

selftest_pins: CFLAGS += -DSELFTEST_PINS
selftest_pins: clean all

selftest_tx_blind: CFLAGS += -DSELFTEST_TX_BLIND
selftest_tx_blind: clean all

selftest_miso_static: CFLAGS += -DSELFTEST_MISO_STATIC
selftest_miso_static: clean all

$(BUILD_DIR)/%.o: ../common/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: node_a_f103/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# OpenOCD programming and debug targets
OPENOCD_CFG = -f interface/stlink.cfg -f target/stm32f1x.cfg

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd $(OPENOCD_CFG) -c "program $< verify reset exit"

debug: $(BUILD_DIR)/$(TARGET).elf
	openocd $(OPENOCD_CFG)

# Host unit tests
test: $(BUILD_DIR)/test_a0_host
	@echo "--- Executing host unit tests ---"
	./$(BUILD_DIR)/test_a0_host

$(BUILD_DIR)/test_a0_host: $(HOST_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(HOST_CC) $(HOST_CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD_DIR)
