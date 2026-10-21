#ifndef LOG_H
#define LOG_H

#include "hal.h"

/*
 * UART Log Format per CONTEXT.md section 7:
 * "[TAG] text\r\n"
 * Tags: BOOT RADIO TX RX HOP ARQ HEALTH SYNC STAT ERR TEST
 */

#define TAG_BOOT    "BOOT"
#define TAG_RADIO   "RADIO"
#define TAG_TX      "TX"
#define TAG_RX      "RX"
#define TAG_HOP     "HOP"
#define TAG_ARQ     "ARQ"
#define TAG_HEALTH  "HEALTH"
#define TAG_SYNC    "SYNC"
#define TAG_STAT    "STAT"
#define TAG_ERR     "ERR"
#define TAG_TEST    "TEST"

static inline void log_line(const char *tag, const char *text) {
    if (tag != (const char *)0) {
        UART1_SendString("[");
        UART1_SendString(tag);
        UART1_SendString("] ");
    }
    if (text != (const char *)0) {
        UART1_SendString(text);
    }
    UART1_SendString("\r\n");
}

#endif /* LOG_H */
