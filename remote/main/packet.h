#pragma once

#include <stdint.h>

#define PKT_MAGIC 0xA5

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    int16_t  throttle; /* -1000 .. 1000 */
    int16_t  steer;    /* -1000 .. 1000 */
    uint8_t  seq;
} rc_pkt_t;
