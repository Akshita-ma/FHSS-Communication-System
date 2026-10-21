#ifndef STEP4_RADIO_SPI_DIAG_H
#define STEP4_RADIO_SPI_DIAG_H

#include <stdint.h>
#include "status.h"

/*
 * Step 4: Read-Only SX1278 SPI Diagnostic
 *
 * Diagnostic Goal:
 *   Investigate Node B's real SX1278 SPI interface.
 *   Verify RegVersion (address 0x42) reset value (expected: 0x12).
 *
 * Rules:
 *   - Strictly read-only: reads only register 0x42 (REG_VERSION).
 *   - Never enters TX or RX mode.
 *   - Never writes arbitrary radio registers.
 *   - Preserves production driver configuration.
 */

#define STEP4_READ_COUNT 32
#define SX1278_EXPECTED_REG_VERSION 0x12

typedef struct {
    uint8_t total_reads;
    uint8_t pass_count;               /* Count of reads returning 0x12 */
    uint8_t fail_count;               /* Count of reads != 0x12 or SPI error */
    uint8_t first_unexpected;         /* First unexpected value */
    uint8_t first_unexpected_valid;   /* 1 if first_unexpected was recorded */
    uint8_t all_same;                 /* 1 if all 32 reads returned identical value */
    uint8_t reads[STEP4_READ_COUNT];  /* Individual read results */
    status_t statuses[STEP4_READ_COUNT];
} step4_diag_result_t;

/* Host-testable pure diagnostic execution function */
void step4_run_diagnostic(
    step4_diag_result_t *res,
    status_t (*read_fn)(uint8_t addr, uint8_t *val)
);

/* Target firmware entry point */
void step4_radio_spi_diag_run(void);

#endif /* STEP4_RADIO_SPI_DIAG_H */
