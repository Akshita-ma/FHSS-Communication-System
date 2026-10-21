# Makefile for Node B (STM32F401 / STM32F411 Bare-Metal FHSS Receiver)

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

BUILD_DIR = build
TARGET    = $(BUILD_DIR)/node_b

# Strict compiler flags per specification:
# -Wall -Wextra -Werror -Os -ffunction-sections -Wl,--gc-sections
CFLAGS  = -mcpu=cortex-m4 -mthumb -Os \
          -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections \
          -g \
          -I. -I.. -I../common -Inode_b_f4 \
          $(EXTRA_CFLAGS)

LDFLAGS = -mcpu=cortex-m4 -mthumb \
          -Tnode_b_f4/linker_f4.ld \
          -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map \
          -nostartfiles \
          -specs=nano.specs -specs=nosys.specs

# Source files for Node B target
SRCS_F4 = node_b_f4/startup_f4.c \
          node_b_f4/stm32_baremetal.c \
          node_b_f4/node_b_rx.c

OBJS_F4 = $(patsubst node_b_f4/%.c,$(BUILD_DIR)/%.o,$(SRCS_F4)) \
          $(BUILD_DIR)/sx1278_fsk.o

.PHONY: all clean flash test dump_vector selftest_loopback selftest_pins

# Host test sources
HOST_TEST_SRCS = ../transmitter/tests/host/test_main.c \
                 ../transmitter/tests/host/mock_hal.c \
                 ../common/sx1278_fsk.c
HOST_TEST_BIN  = $(BUILD_DIR)/host_tests

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex
	@echo "--- Build successful. Image size: ---"
	$(SIZE) $(TARGET).elf

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

$(TARGET).elf: $(OBJS_F4)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS_F4) $(LDFLAGS) -o $@

$(BUILD_DIR)/sx1278_fsk.o: ../common/sx1278_fsk.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: node_b_f4/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

dump_vector: $(TARGET).bin
	@echo "--- Vector Table (First 8 words / 32 bytes) ---"
	@od -A x -t x4 -N 32 $(TARGET).bin

test: $(HOST_TEST_BIN)
	./$(HOST_TEST_BIN)

$(HOST_TEST_BIN): $(HOST_TEST_SRCS)
	@mkdir -p $(BUILD_DIR)
	gcc -Wall -Wextra -Werror -O2 -I. -I.. -I../common -I../transmitter/tests/host $(HOST_TEST_SRCS) -o $@

flash: $(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(TARGET).elf verify reset exit"

clean:
	rm -rf $(BUILD_DIR)
