# Makefile for Node B (STM32F401 / STM32F411 Black Pill)
# Bare-metal receiver & responder for FHSS radio link

TARGET     ?= firmware
BUILD_DIR   = build
ALT_TARGET  = node_b

# Toolchain definitions
CC          = arm-none-eabi-gcc
OBJCOPY     = arm-none-eabi-objcopy
OBJDUMP     = arm-none-eabi-objdump
SIZE        = arm-none-eabi-size

# Host toolchain for unit tests
HOST_CC     = gcc

# Architecture and MCU compiler flags
MCU_FLAGS   = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
WARN_FLAGS  = -Wall -Wextra -Werror
OPT_FLAGS   = -Os -ffunction-sections -fdata-sections
INCLUDES    = -I../common -Inode_b_f4

CFLAGS      = $(MCU_FLAGS) $(WARN_FLAGS) $(OPT_FLAGS) $(INCLUDES)
LDFLAGS     = $(MCU_FLAGS) -Tnode_b_f4/linker_f4.ld -Wl,--gc-sections -nostartfiles -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--no-warn-rwx-segments

# Target objects
OBJS        = $(BUILD_DIR)/status.o \
              $(BUILD_DIR)/config.o \
              $(BUILD_DIR)/log.o \
              $(BUILD_DIR)/sx1278_fsk.o \
              $(BUILD_DIR)/startup_f4.o \
              $(BUILD_DIR)/stm32_baremetal.o \
              $(BUILD_DIR)/fhss_protocol.o \
              $(BUILD_DIR)/air_usart2.o \
              $(BUILD_DIR)/demo_node_b.o \
              $(BUILD_DIR)/step1_uart_test.o \
              $(BUILD_DIR)/step2_mock.o \
              $(BUILD_DIR)/step3_fhss_mock.o \
              $(BUILD_DIR)/step4_radio_spi_diag.o \
              $(BUILD_DIR)/node_b_rx.o

# Host test sources
HOST_SRCS   = ../common/status.c \
              ../common/config.c \
              ../common/log.c \
              ../common/sx1278_fsk.c \
              ../common/fhss_protocol.c \
              node_b_f4/step1_uart_test.c \
              node_b_f4/step2_mock.c \
              node_b_f4/step3_fhss_mock.c \
              node_b_f4/step4_radio_spi_diag.c \
              tests/host/mock_hal.c \
              tests/host/test_b0_host.c

HOST_CFLAGS = -Wall -Wextra -Werror -I../common -Itests/host -Inode_b_f4 -DHOST_TEST

.PHONY: all clean flash test selftest_b0 selftest_b1 selftest_b2 selftest_spi_loopback selftest_pins selftest_tx_blind selftest_miso_static demo debug step1-uart-test flash_step1 step2_mock step2-mock flash_step2 flash_step2_mock step3_fhss_mock step3-fhss-mock flash_step3 flash_step3_mock step4_radio_spi_diag step4-radio-spi-diag flash_step4 flash_step4_radio_spi_diag

step1-uart-test: CFLAGS += -DSTEP1_UART_TEST
step1-uart-test: clean
	@$(MAKE) $(BUILD_DIR)/step1_uart_test.elf $(BUILD_DIR)/step1_uart_test.bin $(BUILD_DIR)/step1_uart_test.hex TARGET=step1_uart_test CFLAGS="$(CFLAGS) -DSTEP1_UART_TEST"
	@cp -f $(BUILD_DIR)/step1_uart_test.elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step1_uart_test.bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step1_uart_test.hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Step 1 UART Test Build complete ---"
	$(SIZE) $(BUILD_DIR)/step1_uart_test.elf

flash_step1: step1-uart-test
	openocd $(OPENOCD_CFG) -c "program $(BUILD_DIR)/step1_uart_test.elf verify reset exit"

step2_mock: CFLAGS += -DSTEP2_MOCK
step2_mock: clean
	@$(MAKE) $(BUILD_DIR)/step2_mock.elf $(BUILD_DIR)/step2_mock.bin $(BUILD_DIR)/step2_mock.hex TARGET=step2_mock CFLAGS="$(CFLAGS) -DSTEP2_MOCK"
	@cp -f $(BUILD_DIR)/step2_mock.elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step2_mock.bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step2_mock.hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Step 2 Mock Build complete ---"
	$(SIZE) $(BUILD_DIR)/step2_mock.elf

step2-mock: step2_mock

flash_step2: step2_mock
	openocd $(OPENOCD_CFG) -c "program $(BUILD_DIR)/step2_mock.elf verify reset exit"

flash_step2_mock: flash_step2

step3_fhss_mock: CFLAGS += -DSTEP3_FHSS_MOCK
step3_fhss_mock: clean
	@$(MAKE) $(BUILD_DIR)/step3_fhss_mock.elf $(BUILD_DIR)/step3_fhss_mock.bin $(BUILD_DIR)/step3_fhss_mock.hex TARGET=step3_fhss_mock CFLAGS="$(CFLAGS) -DSTEP3_FHSS_MOCK"
	@cp -f $(BUILD_DIR)/step3_fhss_mock.elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step3_fhss_mock.bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step3_fhss_mock.hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Step 3 FHSS Mock Build complete ---"
	$(SIZE) $(BUILD_DIR)/step3_fhss_mock.elf

step3-fhss-mock: step3_fhss_mock

flash_step3: step3_fhss_mock
	openocd $(OPENOCD_CFG) -c "program $(BUILD_DIR)/step3_fhss_mock.elf verify reset exit"

flash_step3_mock: flash_step3

step4_radio_spi_diag: CFLAGS += -DSTEP4_RADIO_SPI_DIAG
step4_radio_spi_diag: clean
	@$(MAKE) $(BUILD_DIR)/step4_radio_spi_diag.elf $(BUILD_DIR)/step4_radio_spi_diag.bin $(BUILD_DIR)/step4_radio_spi_diag.hex TARGET=step4_radio_spi_diag CFLAGS="$(CFLAGS) -DSTEP4_RADIO_SPI_DIAG"
	@cp -f $(BUILD_DIR)/step4_radio_spi_diag.elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step4_radio_spi_diag.bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/step4_radio_spi_diag.hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Step 4 Radio SPI Diag Build complete ---"
	$(SIZE) $(BUILD_DIR)/step4_radio_spi_diag.elf

step4-radio-spi-diag: step4_radio_spi_diag

flash_step4: step4_radio_spi_diag
	openocd $(OPENOCD_CFG) -c "program $(BUILD_DIR)/step4_radio_spi_diag.elf verify reset exit"

flash_step4_radio_spi_diag: flash_step4

demo: CFLAGS += -DDEMO_USART_LINK -DDEMO_TRACE
ifeq ($(RF_KEYING),1)
demo: CFLAGS += -DDEMO_RF_KEYING
endif
demo: clean all

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@cp -f $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(ALT_TARGET).elf 2>/dev/null || true
	@cp -f $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(ALT_TARGET).bin 2>/dev/null || true
	@cp -f $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(ALT_TARGET).hex 2>/dev/null || true
	@echo "--- Build complete ---"
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

selftest_b0: CFLAGS += -DSELFTEST_B0
selftest_b0: clean all

selftest_b1: CFLAGS += -DSELFTEST_B0 -DSELFTEST_B1
selftest_b1: clean all

selftest_b2: CFLAGS += -DSELFTEST_B0 -DSELFTEST_B2
selftest_b2: clean all

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

$(BUILD_DIR)/%.o: node_b_f4/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.elf: $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

# OpenOCD programming and debug targets
OPENOCD_CFG = -f interface/stlink.cfg -f target/stm32f4x.cfg

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd $(OPENOCD_CFG) -c "program $< verify reset exit"

debug: $(BUILD_DIR)/$(TARGET).elf
	openocd $(OPENOCD_CFG)

# Host unit tests
test: $(BUILD_DIR)/test_b0_host
	@echo "--- Executing host unit tests ---"
	./$(BUILD_DIR)/test_b0_host

$(BUILD_DIR)/test_b0_host: $(HOST_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(HOST_CC) $(HOST_CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD_DIR)
