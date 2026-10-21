#ifndef STATUS_H
#define STATUS_H

/*
 * Status codes (FROZEN per CONTEXT.md section 6)
 * Every public function validates inputs and returns one of these.
 */
typedef enum {
    OK = 0,
    ERR_NULL,
    ERR_LEN,
    ERR_RANGE,
    ERR_TIMEOUT,
    ERR_CRC,
    ERR_TAG,
    ERR_REPLAY,
    ERR_NOACK,
    ERR_STATE,
    ERR_HW
} status_t;

#endif /* STATUS_H */
