#ifndef NONCE_MGR_H
#define NONCE_MGR_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NONCE_LEN 12U

/**
 * Initialize persistent nonce manager.
 * On boot/reset: scans persistent storage, recovers the latest monotonic counter,
 * and advances the counter to guarantee nonce uniqueness across resets.
 */
void nonce_mgr_init(void);

/**
 * Obtain a strictly unique 12-byte nonce.
 *
 * @param nonce_out   12-byte output buffer:
 *                    - bytes 0..3: Node prefix "NODA" (0x4E, 0x4F, 0x44, 0x41)
 *                    - bytes 4..11: 64-bit big-endian monotonic counter
 * @return            0 on success, -1 on storage error.
 */
int nonce_mgr_get_next(uint8_t nonce_out[NONCE_LEN]);

/**
 * Query current monotonic counter value.
 */
uint64_t nonce_mgr_get_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* NONCE_MGR_H */
