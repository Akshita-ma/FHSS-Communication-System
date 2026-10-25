#ifndef LOG_H
#define LOG_H

#include <stdint.h>

/*
 * UART Log Interface (CONTEXT.md section 7 & Hard Rule 6)
 * Format is strictly: "[TAG] text\r\n"
 * Allowed tags: BOOT, RADIO, TX, RX, HOP, ARQ, HEALTH, SYNC, STAT, ERR, TEST
 */

void Log_Msg(const char *tag, const char *text);
void Log_Test(const char *test_id, uint8_t pass);
void Log_Uint(const char *tag, const char *prefix, uint32_t val);
void Log_Hex(const char *tag, const char *prefix, uint32_t val);

#endif /* LOG_H */
