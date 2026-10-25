#include "log.h"
#include "hal.h"
#include <stddef.h>

static void uint_to_str(uint32_t val, char *buf) {
    char temp[12];
    int i = 0;
    if (val == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }
    while (val > 0) {
        temp[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    int j = 0;
    while (i > 0) {
        buf[j++] = temp[--i];
    }
    buf[j] = '\0';
}

static void hex_to_str(uint32_t val, char *buf) {
    const char hex_chars[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    int shift = 28;
    int j = 2;
    int started = 0;
    for (int k = 0; k < 8; k++) {
        uint8_t nibble = (uint8_t)((val >> shift) & 0x0F);
        shift -= 4;
        if (nibble != 0 || started || k == 7) {
            started = 1;
            buf[j++] = hex_chars[nibble];
        }
    }
    buf[j] = '\0';
}

void Log_Msg(const char *tag, const char *text) {
    if (tag == NULL) {
        tag = "ERR";
    }
    UART1_SendString("[");
    UART1_SendString(tag);
    UART1_SendString("] ");
    if (text != NULL) {
        UART1_SendString(text);
    }
    UART1_SendString("\r\n");
}

void Log_Test(const char *test_id, uint8_t pass) {
    UART1_SendString("[TEST] ");
    if (test_id != NULL) {
        UART1_SendString(test_id);
    }
    if (pass) {
        UART1_SendString(" PASS\r\n");
    } else {
        UART1_SendString(" FAIL\r\n");
    }
}

void Log_Uint(const char *tag, const char *prefix, uint32_t val) {
    char num_str[12];
    uint_to_str(val, num_str);

    if (tag == NULL) {
        tag = "STAT";
    }
    UART1_SendString("[");
    UART1_SendString(tag);
    UART1_SendString("] ");
    if (prefix != NULL) {
        UART1_SendString(prefix);
    }
    UART1_SendString(num_str);
    UART1_SendString("\r\n");
}

void Log_Hex(const char *tag, const char *prefix, uint32_t val) {
    char num_str[12];
    hex_to_str(val, num_str);

    if (tag == NULL) {
        tag = "STAT";
    }
    UART1_SendString("[");
    UART1_SendString(tag);
    UART1_SendString("] ");
    if (prefix != NULL) {
        UART1_SendString(prefix);
    }
    UART1_SendString(num_str);
    UART1_SendString("\r\n");
}
