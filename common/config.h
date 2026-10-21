#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/*
 * FHSS Protocol and Radio Configuration Constants
 * FROZEN per CONTEXT.md sections 8 - 14.
 */

/* Security / Scrambler */
#define FHSS_KEY                0xA5C31F7BUL

/* Frame sizing */
#define MAX_PAYLOAD             16U
#define HEADER_LEN              6U
#define MAX_FRAME_LEN           (HEADER_LEN + MAX_PAYLOAD)

/* Packet types (VER_TYPE: 0xA0 | TYPE) */
#define PKT_TYPE_MASK           0x0FU
#define PKT_VER_MASK            0xF0U
#define PKT_VERSION_PREFIX      0xA0U

#define PKT_TYPE_DATA           1U
#define PKT_TYPE_ACK            2U
#define PKT_TYPE_HOP            3U

/* ARQ limits */
#define MAX_FRAGMENTS           8U
#define MAX_MESSAGE_LEN         128U
#define MAX_ARQ_ROUNDS          5U
#define ACK_TIMEOUT_MS          12U

/* Hop timing (ms) */
#define HOME_DWELL_MS           120U
#define HOP_GUARD_MS            8U
#define MIN_DWELL_CODE          0U
#define MAX_DWELL_CODE          31U
#define DWELL_BASE_MS           80U
#define DWELL_STEP_MS           5U

/* Channel plan */
#define NUM_CHANNELS            8U
#define CH_FREQ_BASE_HZ         433100000UL
#define CH_FREQ_STEP_HZ         200000UL
#define CH_FREQ_MIN_HZ          433100000UL
#define CH_FREQ_MAX_HZ          434500000UL

/* Channel health defaults */
#define HEALTH_SCORE_INIT       128U
#define HEALTH_SCORE_MIN        0U
#define HEALTH_SCORE_MAX        255U
#define HEALTH_SCORE_SUCCESS    8U
#define HEALTH_SCORE_FAIL       32U
#define HEALTH_SCORE_BLACKLIST  32U
#define HEALTH_PROBE_INTERVAL   20U
#define HEALTH_PROBE_SCORE      64U
#define HEALTH_MIN_ELIGIBLE     2U
#define JAMMED_RSSI_THRESHOLD_DBM (-70)

/* Sync state machine */
#define SYNC_BEACON_INTERVAL_MS 20U
#define SYNC_MAX_MISSED_HOPS    3U
#define STAT_REPORT_INTERVAL_MS 5000U

/* Demo link and channel emulation switches */
#ifndef DEMO_JAM_MASK
#define DEMO_JAM_MASK           0x24U   /* Channels 2 and 5 (1<<2 | 1<<5) */
#endif

#ifndef DEMO_JAM_START_MS
#define DEMO_JAM_START_MS       30000UL /* 30 s from first LOCKED event */
#endif

#ifndef DEMO_JAM_END_MS
#define DEMO_JAM_END_MS         90000UL /* 90 s from first LOCKED event */
#endif

#ifndef DEMO_DROP_FIRST_ROUND
#define DEMO_DROP_FIRST_ROUND   0x24U   /* Discard fragments 2 and 5 on first round */
#endif

#endif /* CONFIG_H */

