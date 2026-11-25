#pragma once

#include <stdint.h>
#include "esp_err.h"

/* GPIO at init. LEDC low-speed TIMER_1 CH2, 50 Hz, 14-bit.
 * Pulse clamped 1000–2000 us; center 1500 us + trim; |steer|=1000 → ±range_us. */

esp_err_t servo_init(int gpio, int trim_us, int range_us);
void servo_set(int16_t steer);
