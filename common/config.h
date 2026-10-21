#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "status.h"

/*
 * FROZEN CONSTANTS (CONTEXT.md)
 */

/* Radio configuration (section 8) */
#define SX1278_EXPECTED_VERSION    0x12
#define RADIO_BITRATE_BPS          50000UL
#define RADIO_FDEV_HZ              25000UL
#define RADIO_PREAMBLE_BYTES       5
#define RADIO_SYNC_WORD_LEN        2
#define RADIO_SYNC_BYTE1           0x2D
#define RADIO_SYNC_BYTE2           0xD4
#define RADIO_RESET_PULSE_US       150    /* >= 100 us */
#define RADIO_RESET_WAIT_MS        10     /* >= 5 ms */

/* Channel plan (section 9) */
#define FHSS_NUM_CHANNELS          8
#define FHSS_HOME_CHANNEL          0
#define FHSS_FREQ_MIN_HZ           433100000UL
#define FHSS_FREQ_MAX_HZ           434500000UL
#define FHSS_FREQ_STEP_HZ          200000UL

/* Frame format (section 10) */
#define MAX_PAYLOAD_LEN            16
#define HEADER_LEN                 6
#define MAX_FRAME_LEN              (HEADER_LEN + MAX_PAYLOAD_LEN)
#define FHSS_KEY                   0xA5C31F7BUL
#define CRC8_SMBUS_POLY            0x07
#define CRC8_SMBUS_INIT            0x00

#define PKT_TYPE_DATA              1
#define PKT_TYPE_ACK               2
#define PKT_TYPE_HOP               3

#define PKT_VER_PREFIX             0xA0

/* Selective Bitmap ARQ (section 11) */
#define ARQ_MAX_FRAGMENTS          8
#define ARQ_MAX_MSG_LEN            128
#define ARQ_ACK_TIMEOUT_MS         12
#define ARQ_MAX_ROUNDS             5
#define ARQ_SILENCE_TIMEOUT_MS     6

/* Hop schedule & timing (section 12) */
#define HOME_DWELL_MS              120
#define HOP_GUARD_MS               8
#define DWELL_MIN_MS               80
#define DWELL_MAX_MS               235
#define DWELL_STEP_MS              5
#define DWELL_CODE_MAX             31

/* Channel health (section 13) */
#define HEALTH_SCORE_INIT          128
#define HEALTH_SCORE_SUCCESS       8
#define HEALTH_SCORE_FAILURE       32
#define HEALTH_SCORE_BLACKLIST     32
#define HEALTH_SCORE_PROBE         64
#define HEALTH_PROBE_HOP_COUNT     20
#define HEALTH_JAMMED_RSSI_DBM     (-70)

#define SYNC_LOST_HOPS_THRESHOLD   3
#define STATS_REPORT_INTERVAL_MS   5000UL

/* Demo Jam and ARQ switches (defaults on in demo) */
#define DEMO_JAM_MASK              0x24   /* channels 2 and 5 */
#define DEMO_JAM_START_MS          30000UL
#define DEMO_JAM_END_MS            90000UL
#define DEMO_DROP_FIRST_ROUND      0x24   /* fragments 2 and 5 */

/* Channel lookup table structure */
typedef struct {
    uint8_t  channel;
    uint32_t freq_hz;
    uint32_t frf;
    uint8_t  frf_msb;
    uint8_t  frf_mid;
    uint8_t  frf_lsb;
} channel_entry_t;

extern const channel_entry_t g_channel_table[FHSS_NUM_CHANNELS];

status_t config_get_channel_frf(uint8_t ch, uint32_t *frf_out);
status_t config_get_channel_bytes(uint8_t ch, uint8_t *msb, uint8_t *mid, uint8_t *lsb);

#endif /* CONFIG_H */
