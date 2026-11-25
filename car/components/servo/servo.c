#include "servo.h"

#include "driver/ledc.h"

#define SERVO_FREQ_HZ   50
#define SERVO_RES       LEDC_TIMER_14_BIT
#define SERVO_CENTER_US 1500
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000

static int s_trim_us;
static int s_range_us;

static void servo_pulse_us(int us)
{
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    } else if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    /* 50 Hz period is 20000 us; 14-bit max duty is 16383. */
    uint32_t duty = (uint32_t)us * ((1 << 14) - 1) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

esp_err_t servo_init(int gpio, int trim_us, int range_us)
{
    s_trim_us = trim_us;
    s_range_us = range_us;

    ledc_timer_config_t t1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = SERVO_RES,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&t1);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
        .gpio_num = gpio,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        return err;
    }
    servo_pulse_us(SERVO_CENTER_US + s_trim_us);
    return ESP_OK;
}

void servo_set(int16_t steer)
{
    servo_pulse_us(SERVO_CENTER_US + s_trim_us + (int)steer * s_range_us / 1000);
}
