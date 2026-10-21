# Makefile for Node A (STM32F103C8T6 Bare-Metal FHSS Transmitter)

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

HOST_CC = gcc

BUILD_DIR = build
TARGET    = $(BUILD_DIR)/node_a

# Strict compiler flags per specification:
# -Wall -Wextra -Werror -Os -ffunction-sections -Wl,--gc-sections
CFLAGS  = -mcpu=cortex-m3 -mthumb -Os \
          -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections \
          -g \
          -I. -I.. -I../common -Inode_a_f103 \
          $(EXTRA_CFLAGS)

LDFLAGS = -mcpu=cortex-m3 -mthumb \
          -Tnode_a_f103/linker_f103.ld \
          -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map \
          -nostartfiles \
          -specs=nano.specs -specs=nosys.specs

# Source files for Node A target
SRCS_F103 = node_a_f103/startup_f103.c \
            node_a_f103/stm32_baremetal.c \
            node_a_f103/node_a_tx.c \
            node_a_f103/air_mock_f103.c

OBJS_F103 = $(patsubst node_a_f103/%.c,$(BUILD_DIR)/%.o,$(SRCS_F103)) \
            $(BUILD_DIR)/sx1278_fsk.o \
            $(BUILD_DIR)/fhss_protocol.o \
            $(BUILD_DIR)/air_real.o

# Host test sources
HOST_TEST_SRCS = tests/host/test_main.c \
                 tests/host/mock_hal.c \
                 ../common/sx1278_fsk.c \
                 ../common/fhss_protocol.c
HOST_TEST_BIN  = $(BUILD_DIR)/host_tests

.PHONY: all clean flash test dump_vector selftest_loopback selftest_pins demo step1_test step1 step2_mock step2 step3_fhss_mock step3 step4_radio_spi_diag step4

ifeq ($(RF_KEYING),1)
DEMO_EXTRA = -DDEMO_RF_KEYING
endif

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex
	@echo "--- Build successful. Image size: ---"
	$(SIZE) $(TARGET).elf

step1_test:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DSTEP1_UART_TEST"

step1: step1_test

step2_mock:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DDEMO_USART_LINK -DSTEP2_MOCK_TEST"

step2: step2_mock

step3_fhss_mock:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DDEMO_USART_LINK -DSTEP3_FHSS_MOCK $(DEMO_EXTRA)"

step3: step3_fhss_mock

step4_radio_spi_diag:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DSTEP4_RADIO_SPI_DIAG"

step4: step4_radio_spi_diag

demo:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DDEMO_USART_LINK -DDEMO_TRACE $(DEMO_EXTRA)"

selftest_loopback:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DSELFTEST_SPI_LOOPBACK"

selftest_pins:
	$(MAKE) clean
	$(MAKE) all EXTRA_CFLAGS="-DSELFTEST_PINS"

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).elf: $(OBJS_F103)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS_F103) $(LDFLAGS) -o $@

$(BUILD_DIR)/sx1278_fsk.o: ../common/sx1278_fsk.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/fhss_protocol.o: ../common/fhss_protocol.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/air_real.o: ../common/air_real.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: node_a_f103/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

dump_vector: $(TARGET).bin
	@echo "--- Vector Table (First 8 words / 32 bytes) ---"
	@od -A x -t x4 -N 32 $(TARGET).bin

flash: $(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(TARGET).elf verify reset exit"

test: $(HOST_TEST_BIN)
	./$(HOST_TEST_BIN)

$(HOST_TEST_BIN): $(HOST_TEST_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(HOST_CC) -Wall -Wextra -Werror -O2 -I. -I.. -I../common -Itests/host $(HOST_TEST_SRCS) -o $@

clean:
	rm -rf $(BUILD_DIR)
