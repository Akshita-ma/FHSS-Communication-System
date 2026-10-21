# Forwarding Makefile for root workspace
# Default target forwards directly to reciever/

.PHONY: all clean flash test selftest_spi_loopback selftest_pins selftest_tx_blind selftest_miso_static selftest_b0 selftest_b1 selftest_b2 demo debug

all:
	$(MAKE) -C reciever all

demo:
	$(MAKE) -C reciever demo RF_KEYING=$(RF_KEYING)

clean:
	$(MAKE) -C reciever clean

flash:
	$(MAKE) -C reciever flash

test:
	$(MAKE) -C reciever test

selftest_spi_loopback:
	$(MAKE) -C reciever selftest_spi_loopback

selftest_pins:
	$(MAKE) -C reciever selftest_pins

selftest_tx_blind:
	$(MAKE) -C reciever selftest_tx_blind

selftest_miso_static:
	$(MAKE) -C reciever selftest_miso_static

selftest_b0:
	$(MAKE) -C reciever selftest_b0

selftest_b1:
	$(MAKE) -C reciever selftest_b1

selftest_b2:
	$(MAKE) -C reciever selftest_b2

debug:
	$(MAKE) -C reciever debug

step1-uart-test:
	$(MAKE) -C reciever step1-uart-test

flash_step1:
	$(MAKE) -C reciever flash_step1

step2_mock:
	$(MAKE) -C reciever step2_mock

step2-mock:
	$(MAKE) -C reciever step2-mock

flash_step2:
	$(MAKE) -C reciever flash_step2
