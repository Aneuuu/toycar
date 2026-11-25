#include "tb6612.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#define PIN_PWMA 0
#define PIN_AIN1 1
#define PIN_AIN2 3
#define PIN_PWMB 4
#define PIN_BIN1 5
#define PIN_BIN2 6
#define PIN_STBY 7

#define PWM_FREQ_HZ 20000
#define PWM_RES     LEDC_TIMER_10_BIT
#define PWM_MAX     ((1 << 10) - 1)
#define DEADBAND    40

static bool s_invert_a;
static bool s_invert_b;

esp_err_t tb6612_init(bool invert_a, bool invert_b)
{
    s_invert_a = invert_a;
    s_invert_b = invert_b;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) |
                        (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2) |
                        (1ULL << PIN_STBY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(PIN_STBY, 0);

    ledc_timer_config_t t0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&t0);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t ch[] = {
        { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
          .timer_sel = LEDC_TIMER_0, .gpio_num = PIN_PWMA, .duty = 0,
          .intr_type = LEDC_INTR_DISABLE },
        { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
          .timer_sel = LEDC_TIMER_0, .gpio_num = PIN_PWMB, .duty = 0,
          .intr_type = LEDC_INTR_DISABLE },
    };
    for (int i = 0; i < 2; i++) {
        err = ledc_channel_config(&ch[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

void tb6612_set_throttle(int16_t thr)
{
    if (thr > -DEADBAND && thr < DEADBAND) {
        thr = 0;
    }
    if (thr == 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        gpio_set_level(PIN_AIN1, 0);
        gpio_set_level(PIN_AIN2, 0);
        gpio_set_level(PIN_BIN1, 0);
        gpio_set_level(PIN_BIN2, 0);
        gpio_set_level(PIN_STBY, 0);
        return;
    }

    gpio_set_level(PIN_STBY, 1);

    int fwd = thr > 0;
    int a_fwd = s_invert_a ? !fwd : fwd;
    int b_fwd = s_invert_b ? !fwd : fwd;
    gpio_set_level(PIN_AIN1, a_fwd);
    gpio_set_level(PIN_AIN2, !a_fwd);
    gpio_set_level(PIN_BIN1, b_fwd);
    gpio_set_level(PIN_BIN2, !b_fwd);

    int mag = thr < 0 ? -thr : thr;
    uint32_t duty = (uint32_t)mag * PWM_MAX / 1000;
    if (duty > PWM_MAX) {
        duty = PWM_MAX;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
