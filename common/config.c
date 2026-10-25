#include "config.h"
#include <stddef.h>

const channel_entry_t g_channel_table[FHSS_NUM_CHANNELS] = {
    {0, 433100000UL, 0x6C4666, 0x6C, 0x46, 0x66},
    {1, 433300000UL, 0x6C5333, 0x6C, 0x53, 0x33},
    {2, 433500000UL, 0x6C6000, 0x6C, 0x60, 0x00},
    {3, 433700000UL, 0x6C6CCD, 0x6C, 0x6C, 0xCD},
    {4, 433900000UL, 0x6C799A, 0x6C, 0x79, 0x9A},
    {5, 434100000UL, 0x6C8666, 0x6C, 0x86, 0x66},
    {6, 434300000UL, 0x6C9333, 0x6C, 0x93, 0x33},
    {7, 434500000UL, 0x6CA000, 0x6C, 0xA0, 0x00}
};

status_t config_get_channel_frf(uint8_t ch, uint32_t *frf_out) {
    if (frf_out == NULL) {
        return ERR_NULL;
    }
    if (ch >= FHSS_NUM_CHANNELS) {
        return ERR_RANGE;
    }
    *frf_out = g_channel_table[ch].frf;
    return OK;
}

status_t config_get_channel_bytes(uint8_t ch, uint8_t *msb, uint8_t *mid, uint8_t *lsb) {
    if (msb == NULL || mid == NULL || lsb == NULL) {
        return ERR_NULL;
    }
    if (ch >= FHSS_NUM_CHANNELS) {
        return ERR_RANGE;
    }
    *msb = g_channel_table[ch].frf_msb;
    *mid = g_channel_table[ch].frf_mid;
    *lsb = g_channel_table[ch].frf_lsb;
    return OK;
}
