#include "stm32_baremetal.h"
#include "hal.h"
#include "log.h"
#include "status.h"
#include "config.h"
#include "sx1278_fsk.h"
#include <stddef.h>

#include "demo_node_b.h"
#include "step1_uart_test.h"
#include "step2_mock.h"

#if !defined(SELFTEST_SPI_LOOPBACK) && !defined(SELFTEST_PINS) && !defined(SELFTEST_TX_BLIND) && !defined(SELFTEST_MISO_STATIC) && !defined(DEMO_USART_LINK) && !defined(STEP1_UART_TEST) && !defined(STEP2_MOCK)
/* Global test variables for checking .data and .bss initialization */
static volatile uint32_t g_test_data = 0x12345678;
static volatile uint32_t g_test_bss = 0;

static void run_selftests(void) {
    /* Test B0-01: UART1_SendString NULL safety */
    UART1_SendString(NULL);
    Log_Test("B0-01", 1);

    /* Test B0-02: Register configuration check */
    uint8_t b0_02_pass = 1;
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) {
        b0_02_pass = 0;
    }
    if (!(RCC->APB2ENR & RCC_APB2ENR_USART1EN)) {
        b0_02_pass = 0;
    }
    if (USART1->BRR != 0x008B) {
        b0_02_pass = 0;
    }
    if (!(USART1->CR1 & USART_CR1_UE)) {
        b0_02_pass = 0;
    }
    if (!(USART1->CR1 & USART_CR1_TE)) {
        b0_02_pass = 0;
    }
    Log_Test("B0-02", b0_02_pass);

    /* Test B0-03: Memory check (.data initialized, .bss zeroed) */
    uint8_t b0_03_pass = 1;
    if (g_test_data != 0x12345678) {
        b0_03_pass = 0;
    }
    if (g_test_bss != 0) {
        b0_03_pass = 0;
    }
    Log_Test("B0-03", b0_03_pass);
}
#endif

#if !defined(SELFTEST_SPI_LOOPBACK) && !defined(SELFTEST_PINS) && !defined(SELFTEST_TX_BLIND) && !defined(SELFTEST_MISO_STATIC) && !defined(DEMO_USART_LINK) && !defined(STEP1_UART_TEST) && !defined(STEP2_MOCK)
static void log_reg_mismatch(uint8_t ver) {
    if (ver == 0x00) {
        Log_Msg("RADIO", "RegVersion=0x00 mismatch (MISO stuck low)");
    } else if (ver == 0xFF) {
        Log_Msg("RADIO", "RegVersion=0xFF mismatch (MISO stuck high)");
    } else {
        char msg[36];
        const char hex[] = "0123456789ABCDEF";
        msg[0] = 'R'; msg[1] = 'e'; msg[2] = 'g'; msg[3] = 'V';
        msg[4] = 'e'; msg[5] = 'r'; msg[6] = 's'; msg[7] = 'i';
        msg[8] = 'o'; msg[9] = 'n'; msg[10] = '='; msg[11] = '0';
        msg[12] = 'x';
        msg[13] = hex[(ver >> 4) & 0x0F];
        msg[14] = hex[ver & 0x0F];
        msg[15] = ' '; msg[16] = 'm'; msg[17] = 'i'; msg[18] = 's';
        msg[19] = 'm'; msg[20] = 'a'; msg[21] = 't'; msg[22] = 'c';
        msg[23] = 'h'; msg[24] = '\0';
        Log_Msg("RADIO", msg);
    }
}

static void scan_spi_pins(void) {
    const uint8_t perms[6][3] = {
        {5, 6, 7}, /* P0: SCK=5, MISO=6, MOSI=7 */
        {5, 7, 6}, /* P1: SCK=5, MISO=7, MOSI=6 */
        {7, 6, 5}, /* P2: SCK=7, MISO=6, MOSI=5 */
        {7, 5, 6}, /* P3: SCK=7, MISO=5, MOSI=6 */
        {6, 5, 7}, /* P4: SCK=6, MISO=5, MOSI=7 */
        {6, 7, 5}  /* P5: SCK=6, MISO=7, MOSI=5 */
    };

    char res_msg[80];
    const char hex[] = "0123456789ABCDEF";
    int p = 0;
    const char title[] = "SCAN: ";
    for (int i = 0; title[i] != '\0'; i++) res_msg[p++] = title[i];

    for (int k = 0; k < 6; k++) {
        uint8_t sck  = perms[k][0];
        uint8_t miso = perms[k][1];
        uint8_t mosi = perms[k][2];

        /* Reconfigure sck, mosi as out, miso as in with pull-up */
        GPIOA->MODER &= ~((3UL << (sck * 2)) | (3UL << (miso * 2)) | (3UL << (mosi * 2)));
        GPIOA->MODER |=  ((1UL << (sck * 2)) | (0UL << (miso * 2)) | (1UL << (mosi * 2)));
        GPIOA->PUPDR &= ~((3UL << (sck * 2)) | (3UL << (miso * 2)) | (3UL << (mosi * 2)));
        GPIOA->PUPDR |=  (1UL << (miso * 2));

        /* Idle */
        GPIOA->BSRR = (1UL << (sck + 16)) | (1UL << (mosi + 16));
        NSS_HIGH();
        delay_ms(1);
        NSS_LOW();
        delay_us(10);

        /* Send 0x42 */
        uint8_t cmd = 0x42;
        for (int i = 7; i >= 0; i--) {
            if (cmd & (1 << i)) {
                GPIOA->BSRR = (1UL << mosi);
            } else {
                GPIOA->BSRR = (1UL << (mosi + 16));
            }
            delay_us(5);
            GPIOA->BSRR = (1UL << sck);
            delay_us(5);
            GPIOA->BSRR = (1UL << (sck + 16));
            delay_us(5);
        }

        /* Read byte */
        uint8_t val = 0;
        for (int i = 7; i >= 0; i--) {
            GPIOA->BSRR = (1UL << (mosi + 16));
            delay_us(5);
            GPIOA->BSRR = (1UL << sck);
            delay_us(5);
            if (GPIOA->IDR & (1 << miso)) {
                val |= (uint8_t)(1 << i);
            }
            GPIOA->BSRR = (1UL << (sck + 16));
            delay_us(5);
        }

        NSS_HIGH();
        delay_us(10);

        res_msg[p++] = 'P';
        res_msg[p++] = (char)('0' + k);
        res_msg[p++] = '=';
        res_msg[p++] = hex[(val >> 4) & 0x0F];
        res_msg[p++] = hex[val & 0x0F];
        res_msg[p++] = ' ';
    }
    res_msg[p] = '\0';
    Log_Msg("RADIO", res_msg);

    /* Restore all 3 pins to AF5 */
    GPIOA->MODER &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));
    GPIOA->MODER |=  ((2UL << 10) | (2UL << 12) | (2UL << 14));
    GPIOA->PUPDR &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));
    GPIOA->PUPDR |=  (2UL << 10) | (1UL << 12) | (2UL << 14);
}

static void log_radio_diagnostics(void) {
    uint8_t op = 0, frf_m = 0, frf_mid = 0, ver = 0;
    sx1278_read_reg(0x01, &op);
    sx1278_read_reg(0x06, &frf_m);
    sx1278_read_reg(0x07, &frf_mid);
    sx1278_read_reg(0x42, &ver);

    char msg[64];
    const char hex[] = "0123456789ABCDEF";
    const char prefix[] = "REGS: Op=0x";
    int p = 0;
    for (int i = 0; prefix[i] != '\0'; i++) msg[p++] = prefix[i];
    msg[p++] = hex[(op >> 4) & 0x0F];
    msg[p++] = hex[op & 0x0F];
    const char p2[] = " Frf=0x";
    for (int i = 0; p2[i] != '\0'; i++) msg[p++] = p2[i];
    msg[p++] = hex[(frf_m >> 4) & 0x0F];
    msg[p++] = hex[frf_m & 0x0F];
    msg[p++] = hex[(frf_mid >> 4) & 0x0F];
    msg[p++] = hex[frf_mid & 0x0F];
    const char p3[] = " Ver=0x";
    for (int i = 0; p3[i] != '\0'; i++) msg[p++] = p3[i];
    msg[p++] = hex[(ver >> 4) & 0x0F];
    msg[p++] = hex[ver & 0x0F];
    msg[p] = '\0';
    Log_Msg("RADIO", msg);

    scan_spi_pins();
}

static void log_hello_n(uint32_t n) {
    char msg[32];
    const char prefix[] = "node=B hello ";
    int i = 0;
    while (prefix[i] != '\0') {
        msg[i] = prefix[i];
        i++;
    }

    char num[12];
    int k = 0;
    if (n == 0) {
        num[k++] = '0';
    } else {
        uint32_t temp = n;
        char rev[12];
        int r = 0;
        while (temp > 0) {
            rev[r++] = (char)('0' + (temp % 10));
            temp /= 10;
        }
        while (r > 0) {
            num[k++] = rev[--r];
        }
    }

    for (int m = 0; m < k; m++) {
        msg[i++] = num[m];
    }
    msg[i] = '\0';
    Log_Msg("BOOT", msg);
}
#endif

#if defined(SELFTEST_SPI_LOOPBACK)
static void run_selftest_spi_loopback(void) {
    const uint8_t test_bytes[4] = {0xA5, 0x5A, 0x00, 0xFF};
    const char hex[] = "0123456789ABCDEF";

    while (1) {
        /* Print raw SPI1 CR1 and SR register values */
        uint32_t cr1 = SPI1->CR1;
        uint32_t sr = SPI1->SR;
        char reg_msg[48];
        const char cr1_prefix[] = "SPI1 CR1=0x";
        int p = 0;
        for (int i = 0; cr1_prefix[i] != '\0'; i++) reg_msg[p++] = cr1_prefix[i];
        for (int i = 3; i >= 0; i--) reg_msg[p++] = hex[(cr1 >> (i * 4)) & 0x0F];
        const char sr_prefix[] = " SR=0x";
        for (int i = 0; sr_prefix[i] != '\0'; i++) reg_msg[p++] = sr_prefix[i];
        for (int i = 3; i >= 0; i--) reg_msg[p++] = hex[(sr >> (i * 4)) & 0x0F];
        reg_msg[p] = '\0';
        Log_Msg("TEST", reg_msg);

        /* Transfer 0xA5, 0x5A, 0x00, 0xFF */
        uint8_t pass = 1;
        uint8_t rx_bytes[4] = {0};
        for (int i = 0; i < 4; i++) {
            rx_bytes[i] = SPI1_Transfer(test_bytes[i]);
            if (rx_bytes[i] != test_bytes[i]) {
                pass = 0;
            }
        }

        /* Print "[TEST] SPI loopback PASS" only if each byte is echoed */
        if (pass) {
            Log_Msg("TEST", "SPI loopback PASS");
        } else {
            char fail_msg[64];
            const char fail_pfx[] = "SPI loopback FAIL echoed=";
            p = 0;
            for (int i = 0; fail_pfx[i] != '\0'; i++) fail_msg[p++] = fail_pfx[i];
            for (int i = 0; i < 4; i++) {
                fail_msg[p++] = '0';
                fail_msg[p++] = 'x';
                fail_msg[p++] = hex[(rx_bytes[i] >> 4) & 0x0F];
                fail_msg[p++] = hex[rx_bytes[i] & 0x0F];
                if (i < 3) fail_msg[p++] = ' ';
            }
            fail_msg[p] = '\0';
            Log_Msg("TEST", fail_msg);
        }

        delay_ms(2000);
    }
}
#endif

#if defined(SELFTEST_PINS)
static void print_miso_level(void) {
    uint8_t level = (GPIOA->IDR & (1UL << 6)) ? 1 : 0;
    char msg[16];
    msg[0] = 'P'; msg[1] = 'I'; msg[2] = 'N'; msg[3] = 'S';
    msg[4] = ' '; msg[5] = 'M'; msg[6] = 'I'; msg[7] = 'S';
    msg[8] = 'O'; msg[9] = '=';
    msg[10] = (char)('0' + level);
    msg[11] = '\0';
    Log_Msg("TEST", msg);
}

static void log_pins_driving(const char *name, const char *level_str) {
    char msg[48];
    const char prefix[] = "PINS driving ";
    int p = 0;
    for (int i = 0; prefix[i] != '\0'; i++) msg[p++] = prefix[i];
    for (int i = 0; name[i] != '\0'; i++) msg[p++] = name[i];
    msg[p++] = ' ';
    for (int i = 0; level_str[i] != '\0'; i++) msg[p++] = level_str[i];
    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void run_selftest_pins(void) {
    /* 1. Disable SPI1 so AF doesn't take over PA5/PA7 */
    SPI1->CR1 = 0;

    /* 2. Configure NSS (PB0) as plain GPIO output push-pull */
    GPIOB->MODER &= ~(3UL << (0 * 2));
    GPIOB->MODER |=  (1UL << (0 * 2)); /* Output */
    GPIOB->OTYPER &= ~(1UL << 0);      /* Push-pull */
    GPIOB->OSPEEDR |= (3UL << (0 * 2));
    GPIOB->PUPDR &= ~(3UL << (0 * 2));

    /* 3. Configure RESET (PA1), SCK (PA5), MOSI (PA7) as plain GPIO output push-pull */
    GPIOA->MODER &= ~((3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (7 * 2)));
    GPIOA->MODER |=  ((1UL << (1 * 2)) | (1UL << (5 * 2)) | (1UL << (7 * 2))); /* Output */
    GPIOA->OTYPER &= ~((1UL << 1) | (1UL << 5) | (1UL << 7));                  /* Push-pull */
    GPIOA->OSPEEDR |= ((3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (7 * 2)));
    GPIOA->PUPDR &= ~((3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (7 * 2)));

    /* 4. Configure MISO (PA6) as input with pull-up */
    GPIOA->MODER &= ~(3UL << (6 * 2)); /* Input: 00b */
    GPIOA->PUPDR &= ~(3UL << (6 * 2));
    GPIOA->PUPDR |=  (1UL << (6 * 2)); /* Pull-up: 01b */

    /* Initial state: all driven pins LOW */
    GPIOB->BSRR = (1UL << (0 + 16));
    GPIOA->BSRR = (1UL << (1 + 16)) | (1UL << (5 + 16)) | (1UL << (7 + 16));

    while (1) {
        /* NSS (PB0) */
        log_pins_driving("NSS", "HIGH");
        GPIOB->BSRR = (1UL << 0);
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        log_pins_driving("NSS", "LOW");
        GPIOB->BSRR = (1UL << (0 + 16));
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        /* RESET (PA1) */
        log_pins_driving("RESET", "HIGH");
        GPIOA->BSRR = (1UL << 1);
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        log_pins_driving("RESET", "LOW");
        GPIOA->BSRR = (1UL << (1 + 16));
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        /* SCK (PA5) */
        log_pins_driving("SCK", "HIGH");
        GPIOA->BSRR = (1UL << 5);
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        log_pins_driving("SCK", "LOW");
        GPIOA->BSRR = (1UL << (5 + 16));
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        /* MOSI (PA7) */
        log_pins_driving("MOSI", "HIGH");
        GPIOA->BSRR = (1UL << 7);
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }

        log_pins_driving("MOSI", "LOW");
        GPIOA->BSRR = (1UL << (7 + 16));
        for (int i = 0; i < 3; i++) {
            delay_ms(1000);
            print_miso_level();
        }
    }
}
#endif

#if defined(SELFTEST_TX_BLIND)
static void blind_write_fifo(const uint8_t *data, uint8_t len) {
    NSS_LOW();
    delay_us(2);
    SPI1_Transfer(0x00 | 0x80); /* REG_FIFO (0x00) with write bit (0x80) */
    for (uint8_t i = 0; i < len; i++) {
        SPI1_Transfer(data[i]);
    }
    delay_us(2);
    NSS_HIGH();
    delay_us(2);
}

static void log_blind_tx_result(uint8_t yes, uint32_t elapsed_ms) {
    char msg[64];
    int p = 0;
    const char prefix[] = "BLIND TX DIO0 edge: ";
    for (int i = 0; prefix[i] != '\0'; i++) msg[p++] = prefix[i];

    if (yes) {
        msg[p++] = 'Y';
        msg[p++] = 'E';
        msg[p++] = 'S';
    } else {
        msg[p++] = 'N';
        msg[p++] = 'O';
    }

    msg[p++] = ' ';
    msg[p++] = '(';

    char num[12];
    int k = 0;
    if (elapsed_ms == 0) {
        num[k++] = '0';
    } else {
        uint32_t temp = elapsed_ms;
        char rev[12];
        int r = 0;
        while (temp > 0) {
            rev[r++] = (char)('0' + (temp % 10));
            temp /= 10;
        }
        while (r > 0) {
            num[k++] = rev[--r];
        }
    }
    for (int m = 0; m < k; m++) {
        msg[p++] = num[m];
    }

    msg[p++] = ' ';
    msg[p++] = 'm';
    msg[p++] = 's';
    msg[p++] = ')';
    msg[p] = '\0';

    Log_Msg("TEST", msg);
}

static void run_selftest_tx_blind(void) {
    /* Configure PA0 (DIO0) as a plain GPIO input with pull-down */
    GPIOA->MODER &= ~(3UL << (0 * 2));
    GPIOA->PUPDR &= ~(3UL << (0 * 2));
    GPIOA->PUPDR |=  (2UL << (0 * 2)); /* Pull-down */

    while (1) {
        uint32_t cycle_start = millis();

        /* 1. Reset pulse on PA1 and wait 5 ms */
        Radio_Reset_Pin(0);
        delay_ms(1);
        Radio_Reset_Pin(1);
        delay_ms(5);

        /* 2. Using writes only:
         *    - Set RegOpMode (0x01) to FSK standby
         *      (0x00 sleep first ensures clean switch from LoRa if active)
         */
        sx1278_write_reg(0x01, 0x00);
        delay_ms(1);
        sx1278_write_reg(0x01, 0x01);
        delay_ms(1);

        /* Set RegPaConfig (0x09) to 0x8F (PA_BOOST) */
        sx1278_write_reg(0x09, 0x8F);

        /* Write 434.0 MHz default frequency regs: 0x6C, 0x80, 0x00 */
        sx1278_write_reg(0x06, 0x6C);
        sx1278_write_reg(0x07, 0x80);
        sx1278_write_reg(0x08, 0x00);

        /* Ensure variable packet mode and DIO0 mapped to PacketSent */
        sx1278_write_reg(0x30, 0x90); /* RegPacketConfig1: Variable length, CRC ON */
        sx1278_write_reg(0x31, 0x40); /* RegPacketConfig2: Packet mode */
        sx1278_write_reg(0x40, 0x00); /* RegDioMapping1: DIO0 = PacketSent */

        /* Write a FIFO payload (length byte 4, then 4 bytes) */
        const uint8_t fifo_payload[5] = {0x04, 0xDE, 0xAD, 0xBE, 0xEF};
        blind_write_fifo(fifo_payload, 5);

        /* Set RegOpMode to FSK transmit (0x03) */
        sx1278_write_reg(0x01, 0x03);

        /* 3. Poll PA0 (DIO0) as a plain GPIO input for up to 200 ms */
        uint32_t poll_start = millis();
        uint8_t edge_detected = 0;
        uint32_t elapsed_ms = 0;

        while (1) {
            uint32_t now = millis();
            elapsed_ms = now - poll_start;
            if (GPIOA->IDR & (1UL << 0)) {
                edge_detected = 1;
                break;
            }
            if (elapsed_ms >= 200) {
                break;
            }
        }

        /* 4. Print "[TEST] BLIND TX DIO0 edge: YES" or "NO" with the time it took */
        log_blind_tx_result(edge_detected, elapsed_ms);

        /* Return radio to Standby */
        sx1278_write_reg(0x01, 0x01);

        /* Repeat every 2 s */
        uint32_t cycle_elapsed = millis() - cycle_start;
        if (cycle_elapsed < 2000) {
            delay_ms(2000 - cycle_elapsed);
        }
    }
}
#endif

#if defined(SELFTEST_MISO_STATIC)
static void run_selftest_miso_static(void) {
    /* 1. Disable SPI1 so AF does not drive pins */
    SPI1->CR1 = 0;

    /* 2. Configure PB0 (NSS) as plain GPIO output push-pull */
    GPIOB->MODER &= ~(3UL << (0 * 2));
    GPIOB->MODER |=  (1UL << (0 * 2));
    GPIOB->OTYPER &= ~(1UL << 0);
    GPIOB->BSRR = (1UL << 0); /* Initial NSS HIGH */

    /* 3. Configure PA4 as plain GPIO output push-pull, held HIGH to deselect on-board flash */
    GPIOA->MODER &= ~(3UL << (4 * 2));
    GPIOA->MODER |=  (1UL << (4 * 2));
    GPIOA->OTYPER &= ~(1UL << 4);
    GPIOA->BSRR = (1UL << 4); /* PA4 HIGH */

    /* 4. Configure RESET (PA1), SCK (PA5), MOSI (PA7) as plain GPIO outputs push-pull */
    GPIOA->MODER &= ~((3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (7 * 2)));
    GPIOA->MODER |=  ((1UL << (1 * 2)) | (1UL << (5 * 2)) | (1UL << (7 * 2)));
    GPIOA->OTYPER &= ~((1UL << 1) | (1UL << 5) | (1UL << 7));
    GPIOA->BSRR = (1UL << 1); /* RESET HIGH */
    GPIOA->BSRR = (1UL << (5 + 16)) | (1UL << (7 + 16)); /* SCK and MOSI LOW */

    /* 5. Configure MISO (PA6) as plain input with NO PULL */
    GPIOA->MODER &= ~(3UL << (6 * 2)); /* Input (00b) */
    GPIOA->PUPDR &= ~(3UL << (6 * 2)); /* No pull (00b) */

    while (1) {
        /* Reset the radio and wait 5 ms */
        GPIOA->BSRR = (1UL << (1 + 16)); /* PA1 LOW */
        delay_ms(1);
        GPIOA->BSRR = (1UL << 1);        /* PA1 HIGH */
        delay_ms(5);

        /* NSS low */
        GPIOB->BSRR = (1UL << (0 + 16));
        delay_ms(1);

        /* Clock out address 0x42 MSB first, SCK idle low, 1 ms per half-clock */
        uint8_t addr = 0x42;
        for (int i = 7; i >= 0; i--) {
            if (addr & (1 << i)) {
                GPIOA->BSRR = (1UL << 7); /* MOSI HIGH */
            } else {
                GPIOA->BSRR = (1UL << (7 + 16)); /* MOSI LOW */
            }
            delay_ms(1);
            GPIOA->BSRR = (1UL << 5); /* SCK HIGH */
            delay_ms(1);
            GPIOA->BSRR = (1UL << (5 + 16)); /* SCK LOW */
        }

        /* After 8 address bits, sample MISO after each of 8 more full SCK cycles */
        char bits_str[9];
        GPIOA->BSRR = (1UL << (7 + 16)); /* MOSI LOW */
        for (int i = 0; i < 8; i++) {
            delay_ms(1);
            GPIOA->BSRR = (1UL << 5); /* SCK HIGH */
            delay_ms(1);
            uint8_t bit = (GPIOA->IDR & (1UL << 6)) ? 1 : 0;
            bits_str[i] = (char)('0' + bit);
            GPIOA->BSRR = (1UL << (5 + 16)); /* SCK LOW */
        }
        bits_str[8] = '\0';

        /* Release NSS */
        GPIOB->BSRR = (1UL << 0);
        delay_ms(1);

        /* Print "[TEST] MISO bits: 00010010" as the sampled bits */
        char miso_msg[32];
        int p = 0;
        const char pfx[] = "MISO bits: ";
        for (int i = 0; pfx[i] != '\0'; i++) miso_msg[p++] = pfx[i];
        for (int i = 0; i < 8; i++) miso_msg[p++] = bits_str[i];
        miso_msg[p] = '\0';
        Log_Msg("TEST", miso_msg);

        /* Second pass for HOLD:
         * Reset the radio and wait 5 ms, NSS low, clock address 0x42,
         * then after exactly 3 more full cycles past the address (radio output is bit 4 = 1),
         * stop clocking: keep NSS low and SCK low for 20 s.
         */
        delay_ms(10);
        GPIOA->BSRR = (1UL << (1 + 16)); /* PA1 LOW */
        delay_ms(1);
        GPIOA->BSRR = (1UL << 1);        /* PA1 HIGH */
        delay_ms(5);

        GPIOB->BSRR = (1UL << (0 + 16)); /* NSS LOW */
        delay_ms(1);

        /* Clock out address 0x42 */
        for (int i = 7; i >= 0; i--) {
            if (addr & (1 << i)) {
                GPIOA->BSRR = (1UL << 7);
            } else {
                GPIOA->BSRR = (1UL << (7 + 16));
            }
            delay_ms(1);
            GPIOA->BSRR = (1UL << 5);
            delay_ms(1);
            GPIOA->BSRR = (1UL << (5 + 16));
        }

        /* Exactly 3 more full cycles past the address */
        GPIOA->BSRR = (1UL << (7 + 16)); /* MOSI LOW */
        for (int i = 0; i < 3; i++) {
            delay_ms(1);
            GPIOA->BSRR = (1UL << 5); /* SCK HIGH */
            delay_ms(1);
            GPIOA->BSRR = (1UL << (5 + 16)); /* SCK LOW */
        }

        /* Stop clocking: keep NSS low and SCK low for 20 s */
        Log_Msg("TEST", "HOLD: MISO should be 1 now, measure at the module pad");

        for (int s = 0; s < 20; s++) {
            delay_ms(1000);
        }

        /* Release NSS and repeat */
        GPIOB->BSRR = (1UL << 0);
        delay_ms(1000);
    }
}
#endif

int main(void) {
    baremetal_init();

#if defined(STEP2_MOCK)
    step2_mock_run();
#elif defined(STEP1_UART_TEST)
    step1_uart_test_run();
#elif defined(DEMO_USART_LINK)
    demo_node_b_run();
#elif defined(SELFTEST_PINS)
    run_selftest_pins();
#elif defined(SELFTEST_SPI_LOOPBACK)
    run_selftest_spi_loopback();
#elif defined(SELFTEST_TX_BLIND)
    run_selftest_tx_blind();
#elif defined(SELFTEST_MISO_STATIC)
    run_selftest_miso_static();
#else
    /* Execute Step B0 hardware self-tests */
    run_selftests();

    /* Step B2: Radio reset and RegVersion check */
    sx1278_reset();

    uint8_t ver = 0;
    uint8_t version_ok = 0;

    for (int retry = 0; retry < 4; retry++) {
        if (sx1278_read_reg(REG_VERSION, &ver) == OK && ver == 0x12) {
            version_ok = 1;
            break;
        }
        if (retry == 1) {
            sx1278_reset();
        }
        delay_ms(10);
    }

    if (version_ok) {
        Log_Msg("RADIO", "RegVersion=0x12 OK");
        Log_Test("B2-01", 1);
    } else {
        log_reg_mismatch(ver);
        log_radio_diagnostics();
        Log_Test("B2-01", 0);
        NSS_HIGH();
        while (1) {
            /* Safe halt with NSS high and no more SPI traffic */
        }
    }

    uint32_t hello_count = 0;
    uint32_t last_hello_ms = millis();

    /* Output first hello immediately on boot */
    log_hello_n(hello_count++);

    while (1) {
        uint32_t now = millis();
        if ((uint32_t)(now - last_hello_ms) >= 1000) {
            last_hello_ms += 1000;
            log_hello_n(hello_count++);
        }
    }
#endif

    return 0;
}
