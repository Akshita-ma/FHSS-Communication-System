#include "aead_chacha20_poly1305.h"
#include <string.h>

#define ROTL32(v, n) (((v) << (n)) | ((v) >> (32 - (n))))

#define CHACHA20_QR(a, b, c, d) \
    a += b; d ^= a; d = ROTL32(d, 16); \
    c += d; b ^= c; b = ROTL32(b, 12); \
    a += b; d ^= a; d = ROTL32(d, 8);  \
    c += d; b ^= c; b = ROTL32(b, 7);

static uint32_t load32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void store32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void chacha20_block(const uint8_t key[32], uint32_t counter, const uint8_t nonce[12], uint8_t out[64]) {
    uint32_t state[16];
    state[0] = 0x61707865U;
    state[1] = 0x3320646eU;
    state[2] = 0x79622d32U;
    state[3] = 0x6b206574U;

    for (int i = 0; i < 8; i++) {
        state[4 + i] = load32_le(&key[i * 4]);
    }
    state[12] = counter;
    state[13] = load32_le(&nonce[0]);
    state[14] = load32_le(&nonce[4]);
    state[15] = load32_le(&nonce[8]);

    uint32_t working[16];
    for (int i = 0; i < 16; i++) {
        working[i] = state[i];
    }

    for (int i = 0; i < 10; i++) {
        /* Column round */
        CHACHA20_QR(working[0], working[4], working[8],  working[12]);
        CHACHA20_QR(working[1], working[5], working[9],  working[13]);
        CHACHA20_QR(working[2], working[6], working[10], working[14]);
        CHACHA20_QR(working[3], working[7], working[11], working[15]);
        /* Diagonal round */
        CHACHA20_QR(working[0], working[5], working[10], working[15]);
        CHACHA20_QR(working[1], working[6], working[11], working[12]);
        CHACHA20_QR(working[2], working[7], working[8],  working[13]);
        CHACHA20_QR(working[3], working[4], working[9],  working[14]);
    }

    for (int i = 0; i < 16; i++) {
        store32_le(&out[i * 4], working[i] + state[i]);
    }
}

static void chacha20_crypt(const uint8_t key[32], uint32_t counter, const uint8_t nonce[12],
                           const uint8_t *in, size_t in_len, uint8_t *out) {
    uint8_t block[64];
    while (in_len > 0) {
        chacha20_block(key, counter, nonce, block);
        size_t chunk = (in_len < 64) ? in_len : 64;
        for (size_t i = 0; i < chunk; i++) {
            out[i] = in[i] ^ block[i];
        }
        counter++;
        in += chunk;
        out += chunk;
        in_len -= chunk;
    }
}

/* Poly1305 (26-bit limbs arithmetic modulo 2^130 - 5) */
typedef struct {
    uint32_t r0, r1, r2, r3, r4;
    uint32_t s1, s2, s3, s4;
    uint32_t h0, h1, h2, h3, h4;
    uint8_t  pad[16];
    size_t   buf_len;
    uint8_t  buf[16];
} poly1305_ctx_t;

static void poly1305_init(poly1305_ctx_t *ctx, const uint8_t key[32]) {
    /* Clamp r */
    uint32_t t0 = load32_le(&key[0]);
    uint32_t t1 = load32_le(&key[4]);
    uint32_t t2 = load32_le(&key[8]);
    uint32_t t3 = load32_le(&key[12]);

    /* r &= 0x0ffffffc0fffffff0ffffffc0fffffff */
    t0 &= 0x0FFFFFFFU;
    t1 &= 0x0FFFFFFCU;
    t2 &= 0x0FFFFFFCU;
    t3 &= 0x0FFFFFFCU;

    ctx->r0 = t0 & 0x3FFFFFFU;
    ctx->r1 = ((t0 >> 26) | (t1 << 6)) & 0x3FFFFFFU;
    ctx->r2 = ((t1 >> 20) | (t2 << 12)) & 0x3FFFFFFU;
    ctx->r3 = ((t2 >> 14) | (t3 << 18)) & 0x3FFFFFFU;
    ctx->r4 = (t3 >> 8) & 0x3FFFFFFU;

    ctx->s1 = ctx->r1 * 5U;
    ctx->s2 = ctx->r2 * 5U;
    ctx->s3 = ctx->r3 * 5U;
    ctx->s4 = ctx->r4 * 5U;

    ctx->h0 = 0; ctx->h1 = 0; ctx->h2 = 0; ctx->h3 = 0; ctx->h4 = 0;
    memcpy(ctx->pad, &key[16], 16);
    ctx->buf_len = 0;
}

static void poly1305_process_block(poly1305_ctx_t *ctx, const uint8_t *m, size_t bytes) {
    uint8_t temp[17];
    memset(temp, 0, sizeof(temp));
    memcpy(temp, m, bytes);
    temp[bytes] = 0x01; /* 2^(8*bytes) */

    uint32_t t0 = load32_le(&temp[0]);
    uint32_t t1 = load32_le(&temp[4]);
    uint32_t t2 = load32_le(&temp[8]);
    uint32_t t3 = load32_le(&temp[12]);
    uint32_t hibit = (uint32_t)temp[16];

    uint32_t m0 = t0 & 0x3FFFFFFU;
    uint32_t m1 = ((t0 >> 26) | (t1 << 6)) & 0x3FFFFFFU;
    uint32_t m2 = ((t1 >> 20) | (t2 << 12)) & 0x3FFFFFFU;
    uint32_t m3 = ((t2 >> 14) | (t3 << 18)) & 0x3FFFFFFU;
    uint32_t m4 = ((t3 >> 8) | (hibit << 24)) & 0x3FFFFFFU;

    ctx->h0 += m0;
    ctx->h1 += m1;
    ctx->h2 += m2;
    ctx->h3 += m3;
    ctx->h4 += m4;

    uint64_t d0 = ((uint64_t)ctx->h0 * ctx->r0) + ((uint64_t)ctx->h1 * ctx->s4) +
                  ((uint64_t)ctx->h2 * ctx->s3) + ((uint64_t)ctx->h3 * ctx->s2) +
                  ((uint64_t)ctx->h4 * ctx->s1);
    uint64_t d1 = ((uint64_t)ctx->h0 * ctx->r1) + ((uint64_t)ctx->h1 * ctx->r0) +
                  ((uint64_t)ctx->h2 * ctx->s4) + ((uint64_t)ctx->h3 * ctx->s3) +
                  ((uint64_t)ctx->h4 * ctx->s2);
    uint64_t d2 = ((uint64_t)ctx->h0 * ctx->r2) + ((uint64_t)ctx->h1 * ctx->r1) +
                  ((uint64_t)ctx->h2 * ctx->r0) + ((uint64_t)ctx->h3 * ctx->s4) +
                  ((uint64_t)ctx->h4 * ctx->s3);
    uint64_t d3 = ((uint64_t)ctx->h0 * ctx->r3) + ((uint64_t)ctx->h1 * ctx->r2) +
                  ((uint64_t)ctx->h2 * ctx->r1) + ((uint64_t)ctx->h3 * ctx->r0) +
                  ((uint64_t)ctx->h4 * ctx->s4);
    uint64_t d4 = ((uint64_t)ctx->h0 * ctx->r4) + ((uint64_t)ctx->h1 * ctx->r3) +
                  ((uint64_t)ctx->h2 * ctx->r2) + ((uint64_t)ctx->h3 * ctx->r1) +
                  ((uint64_t)ctx->h4 * ctx->r0);

    uint64_t c;
    c = d0 >> 26; ctx->h0 = (uint32_t)d0 & 0x3FFFFFFU; d1 += c;
    c = d1 >> 26; ctx->h1 = (uint32_t)d1 & 0x3FFFFFFU; d2 += c;
    c = d2 >> 26; ctx->h2 = (uint32_t)d2 & 0x3FFFFFFU; d3 += c;
    c = d3 >> 26; ctx->h3 = (uint32_t)d3 & 0x3FFFFFFU; d4 += c;
    c = d4 >> 26; ctx->h4 = (uint32_t)d4 & 0x3FFFFFFU;
    ctx->h0 += (uint32_t)(c * 5U);
    c = ctx->h0 >> 26; ctx->h0 &= 0x3FFFFFFU; ctx->h1 += (uint32_t)c;
}

static void poly1305_update(poly1305_ctx_t *ctx, const uint8_t *data, size_t len) {
    if (ctx->buf_len > 0) {
        size_t need = 16 - ctx->buf_len;
        if (len < need) {
            memcpy(&ctx->buf[ctx->buf_len], data, len);
            ctx->buf_len += len;
            return;
        }
        memcpy(&ctx->buf[ctx->buf_len], data, need);
        poly1305_process_block(ctx, ctx->buf, 16);
        data += need;
        len -= need;
        ctx->buf_len = 0;
    }

    while (len >= 16) {
        poly1305_process_block(ctx, data, 16);
        data += 16;
        len -= 16;
    }

    if (len > 0) {
        memcpy(ctx->buf, data, len);
        ctx->buf_len = len;
    }
}

static void poly1305_finish(poly1305_ctx_t *ctx, uint8_t tag[16]) {
    if (ctx->buf_len > 0) {
        poly1305_process_block(ctx, ctx->buf, ctx->buf_len);
        ctx->buf_len = 0;
    }

    /* Fully reduce h mod 2^130 - 5 */
    uint32_t c = ctx->h0 >> 26; ctx->h0 &= 0x3FFFFFFU; ctx->h1 += c;
    c = ctx->h1 >> 26; ctx->h1 &= 0x3FFFFFFU; ctx->h2 += c;
    c = ctx->h2 >> 26; ctx->h2 &= 0x3FFFFFFU; ctx->h3 += c;
    c = ctx->h3 >> 26; ctx->h3 &= 0x3FFFFFFU; ctx->h4 += c;
    c = ctx->h4 >> 26; ctx->h4 &= 0x3FFFFFFU;
    ctx->h0 += c * 5U;
    c = ctx->h0 >> 26; ctx->h0 &= 0x3FFFFFFU; ctx->h1 += c;

    /* Compute h + 5 */
    uint32_t g0 = ctx->h0 + 5U;
    c = g0 >> 26; g0 &= 0x3FFFFFFU;
    uint32_t g1 = ctx->h1 + c;
    c = g1 >> 26; g1 &= 0x3FFFFFFU;
    uint32_t g2 = ctx->h2 + c;
    c = g2 >> 26; g2 &= 0x3FFFFFFU;
    uint32_t g3 = ctx->h3 + c;
    c = g3 >> 26; g3 &= 0x3FFFFFFU;
    uint32_t g4 = ctx->h4 + c - (1U << 26);

    /* If g4 borrowed (bit 31 is 1), h < 2^130 - 5, so keep h.
     * If g4 did not borrow (bit 31 is 0), h >= 2^130 - 5, so use g.
     */
    uint32_t b = g4 >> 31;
    uint32_t mask = b - 1U;
    g0 &= mask; g1 &= mask; g2 &= mask; g3 &= mask; g4 &= mask;
    mask = ~mask;
    ctx->h0 = (ctx->h0 & mask) | g0;
    ctx->h1 = (ctx->h1 & mask) | g1;
    ctx->h2 = (ctx->h2 & mask) | g2;
    ctx->h3 = (ctx->h3 & mask) | g3;
    ctx->h4 = (ctx->h4 & mask) | g4;

    /* Reconstruct 128-bit integer */
    uint32_t f0 = ctx->h0 | (ctx->h1 << 26);
    uint32_t f1 = (ctx->h1 >> 6) | (ctx->h2 << 20);
    uint32_t f2 = (ctx->h2 >> 12) | (ctx->h3 << 14);
    uint32_t f3 = (ctx->h3 >> 18) | (ctx->h4 << 8);

    /* Add pad (mod 2^128) */
    uint64_t t;
    t = (uint64_t)f0 + load32_le(&ctx->pad[0]);
    store32_le(&tag[0], (uint32_t)t);
    t = (uint64_t)f1 + load32_le(&ctx->pad[4]) + (t >> 32);
    store32_le(&tag[4], (uint32_t)t);
    t = (uint64_t)f2 + load32_le(&ctx->pad[8]) + (t >> 32);
    store32_le(&tag[8], (uint32_t)t);
    t = (uint64_t)f3 + load32_le(&ctx->pad[12]) + (t >> 32);
    store32_le(&tag[12], (uint32_t)t);
}

void aead_chacha20_poly1305_encrypt(
    const uint8_t key[CHACHA20_KEY_LEN],
    const uint8_t nonce[CHACHA20_NONCE_LEN],
    const uint8_t *aad, size_t aad_len,
    const uint8_t *plaintext, size_t plaintext_len,
    uint8_t *ciphertext,
    uint8_t tag[POLY1305_TAG_LEN]
) {
    /* Step 1: Generate Poly1305 one-time key with counter=0 */
    uint8_t otpk[64];
    chacha20_block(key, 0, nonce, otpk);

    /* Step 2: Encrypt plaintext starting at counter=1 */
    if (plaintext_len > 0) {
        chacha20_crypt(key, 1, nonce, plaintext, plaintext_len, ciphertext);
    }

    /* Step 3: Poly1305 MAC */
    poly1305_ctx_t poly;
    poly1305_init(&poly, otpk);

    if (aad_len > 0) {
        poly1305_update(&poly, aad, aad_len);
        size_t pad1_len = (16 - (aad_len % 16)) % 16;
        if (pad1_len > 0) {
            static const uint8_t zeros[16] = {0};
            poly1305_update(&poly, zeros, pad1_len);
        }
    }

    if (plaintext_len > 0) {
        poly1305_update(&poly, ciphertext, plaintext_len);
        size_t pad2_len = (16 - (plaintext_len % 16)) % 16;
        if (pad2_len > 0) {
            static const uint8_t zeros[16] = {0};
            poly1305_update(&poly, zeros, pad2_len);
        }
    }

    uint8_t len_block[16];
    store32_le(&len_block[0], (uint32_t)aad_len);
    store32_le(&len_block[4], 0);
    store32_le(&len_block[8], (uint32_t)plaintext_len);
    store32_le(&len_block[12], 0);
    poly1305_update(&poly, len_block, 16);

    poly1305_finish(&poly, tag);
}

static int constant_time_memcmp(const uint8_t *a, const uint8_t *b, size_t len) {
    uint8_t diff = 0;
    for (size_t i = 0; i < len; i++) {
        diff |= (a[i] ^ b[i]);
    }
    return (diff == 0) ? 0 : -1;
}

int aead_chacha20_poly1305_decrypt(
    const uint8_t key[CHACHA20_KEY_LEN],
    const uint8_t nonce[CHACHA20_NONCE_LEN],
    const uint8_t *aad, size_t aad_len,
    const uint8_t *ciphertext, size_t ciphertext_len,
    const uint8_t tag[POLY1305_TAG_LEN],
    uint8_t *plaintext
) {
    /* Step 1: Generate Poly1305 one-time key with counter=0 */
    uint8_t otpk[64];
    chacha20_block(key, 0, nonce, otpk);

    /* Step 2: Compute expected Poly1305 tag */
    poly1305_ctx_t poly;
    poly1305_init(&poly, otpk);

    if (aad_len > 0) {
        poly1305_update(&poly, aad, aad_len);
        size_t pad1_len = (16 - (aad_len % 16)) % 16;
        if (pad1_len > 0) {
            static const uint8_t zeros[16] = {0};
            poly1305_update(&poly, zeros, pad1_len);
        }
    }

    if (ciphertext_len > 0) {
        poly1305_update(&poly, ciphertext, ciphertext_len);
        size_t pad2_len = (16 - (ciphertext_len % 16)) % 16;
        if (pad2_len > 0) {
            static const uint8_t zeros[16] = {0};
            poly1305_update(&poly, zeros, pad2_len);
        }
    }

    uint8_t len_block[16];
    store32_le(&len_block[0], (uint32_t)aad_len);
    store32_le(&len_block[4], 0);
    store32_le(&len_block[8], (uint32_t)ciphertext_len);
    store32_le(&len_block[12], 0);
    poly1305_update(&poly, len_block, 16);

    uint8_t computed_tag[POLY1305_TAG_LEN];
    poly1305_finish(&poly, computed_tag);

    /* Step 3: Constant-time authentication tag comparison */
    if (constant_time_memcmp(computed_tag, tag, POLY1305_TAG_LEN) != 0) {
        /* Authentication failed! Zero out plaintext destination and abort */
        if (plaintext != NULL && ciphertext_len > 0) {
            memset(plaintext, 0, ciphertext_len);
        }
        return -1;
    }

    /* Step 4: Tag verified - decrypt ciphertext with counter=1 */
    if (ciphertext_len > 0) {
        chacha20_crypt(key, 1, nonce, ciphertext, ciphertext_len, plaintext);
    }

    return 0;
}
