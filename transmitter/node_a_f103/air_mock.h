#ifndef AIR_MOCK_H
#define AIR_MOCK_H

#include <stdint.h>
#include "common/status.h"

/*
 * Diagnostic telemetry counters for USART2 Mock Air Link
 */
typedef struct {
    uint32_t channel_mismatch_count;
    uint32_t crc_error_count;
    uint8_t  last_mismatch_wire_ch;
    uint8_t  last_mismatch_tuned_ch;
    uint8_t  last_mismatch_len;
} air_mock_diag_t;

void air_mock_init(void);
void air_mock_flush(void);
int  air_mock_read_byte(uint8_t *byte);
int  air_mock_send_raw(const uint8_t *data, uint8_t len);
void air_mock_get_diag(air_mock_diag_t *diag);
void air_mock_clear_diag(void);
void mock_air_notify_locked(void);

#endif /* AIR_MOCK_H */
