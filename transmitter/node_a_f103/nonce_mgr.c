#include "nonce_mgr.h"
#include <string.h>

#define NODE_A_PREFIX_0     0x4EU /* 'N' */
#define NODE_A_PREFIX_1     0x4FU /* 'O' */
#define NODE_A_PREFIX_2     0x44U /* 'D' */
#define NODE_A_PREFIX_3     0x41U /* 'A' */

static uint64_t s_current_counter = 0U;

void nonce_mgr_init(void) {
    if (s_current_counter == 0U) {
        s_current_counter = 1U;
    }
}

int nonce_mgr_get_next(uint8_t nonce_out[NONCE_LEN]) {
    s_current_counter++;

    /* Construct 12-byte nonce: [4 bytes "NODA"][8 bytes 64-bit big endian counter] */
    nonce_out[0] = NODE_A_PREFIX_0;
    nonce_out[1] = NODE_A_PREFIX_1;
    nonce_out[2] = NODE_A_PREFIX_2;
    nonce_out[3] = NODE_A_PREFIX_3;

    nonce_out[4]  = (uint8_t)((s_current_counter >> 56) & 0xFFU);
    nonce_out[5]  = (uint8_t)((s_current_counter >> 48) & 0xFFU);
    nonce_out[6]  = (uint8_t)((s_current_counter >> 40) & 0xFFU);
    nonce_out[7]  = (uint8_t)((s_current_counter >> 32) & 0xFFU);
    nonce_out[8]  = (uint8_t)((s_current_counter >> 24) & 0xFFU);
    nonce_out[9]  = (uint8_t)((s_current_counter >> 16) & 0xFFU);
    nonce_out[10] = (uint8_t)((s_current_counter >> 8)  & 0xFFU);
    nonce_out[11] = (uint8_t)(s_current_counter         & 0xFFU);

    return 0;
}

uint64_t nonce_mgr_get_counter(void) {
    return s_current_counter;
}
