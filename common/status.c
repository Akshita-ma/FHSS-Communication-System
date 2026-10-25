#include "status.h"

const char *status_to_string(status_t st) {
    switch (st) {
        case OK:          return "OK";
        case ERR_NULL:    return "ERR_NULL";
        case ERR_LEN:     return "ERR_LEN";
        case ERR_RANGE:   return "ERR_RANGE";
        case ERR_TIMEOUT: return "ERR_TIMEOUT";
        case ERR_CRC:     return "ERR_CRC";
        case ERR_TAG:     return "ERR_TAG";
        case ERR_REPLAY:  return "ERR_REPLAY";
        case ERR_NOACK:   return "ERR_NOACK";
        case ERR_STATE:   return "ERR_STATE";
        case ERR_HW:      return "ERR_HW";
        default:          return "ERR_UNKNOWN";
    }
}
