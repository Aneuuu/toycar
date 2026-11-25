#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Pins: PWMA=GPIO0, AIN1=GPIO1, AIN2=GPIO3, PWMB=GPIO4, BIN1=GPIO5, BIN2=GPIO6, STBY=GPIO7.
 * LEDC low-speed TIMER_0 CH0/CH1, 20 kHz, 10-bit. Servo must use another timer/channel. */

esp_err_t tb6612_init(bool invert_a, bool invert_b);
void tb6612_set_throttle(int16_t thr);
