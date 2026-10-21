#ifndef AEAD_CHACHA20_POLY1305_H
#define AEAD_CHACHA20_POLY1305_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHACHA20_KEY_LEN     32U
#define CHACHA20_NONCE_LEN   12U
#define POLY1305_TAG_LEN     16U
#define ENC_VERSION_BYTE     0xE1U

/**
 * Standard RFC 8439 ChaCha20-Poly1305 Authenticated Encryption.
 *
 * @param key           32-byte shared key.
 * @param nonce         12-byte unique nonce.
 * @param aad           Associated data buffer.
 * @param aad_len       Length of associated data in bytes.
 * @param plaintext     Input plaintext buffer.
 * @param plaintext_len Length of plaintext in bytes.
 * @param ciphertext    Output ciphertext buffer (must hold at least plaintext_len bytes).
 * @param tag           Output 16-byte Poly1305 authentication tag.
 */
void aead_chacha20_poly1305_encrypt(
    const uint8_t key[CHACHA20_KEY_LEN],
    const uint8_t nonce[CHACHA20_NONCE_LEN],
    const uint8_t *aad, size_t aad_len,
    const uint8_t *plaintext, size_t plaintext_len,
    uint8_t *ciphertext,
    uint8_t tag[POLY1305_TAG_LEN]
);

/**
 * Standard RFC 8439 ChaCha20-Poly1305 Authenticated Decryption.
 *
 * @param key           32-byte shared key.
 * @param nonce         12-byte nonce.
 * @param aad           Associated data buffer.
 * @param aad_len       Length of associated data in bytes.
 * @param ciphertext    Input ciphertext buffer.
 * @param ciphertext_len Length of ciphertext in bytes.
 * @param tag           Input 16-byte Poly1305 authentication tag to verify.
 * @param plaintext     Output plaintext buffer (must hold at least ciphertext_len bytes).
 * @return              0 on success (authenticated and decrypted), -1 on authentication failure.
 */
int aead_chacha20_poly1305_decrypt(
    const uint8_t key[CHACHA20_KEY_LEN],
    const uint8_t nonce[CHACHA20_NONCE_LEN],
    const uint8_t *aad, size_t aad_len,
    const uint8_t *ciphertext, size_t ciphertext_len,
    const uint8_t tag[POLY1305_TAG_LEN],
    uint8_t *plaintext
);

#ifdef __cplusplus
}
#endif

#endif /* AEAD_CHACHA20_POLY1305_H */
