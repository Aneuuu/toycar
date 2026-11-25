#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "packet.h"

static const char *TAG = "car";

#define PIN_PWMA   0
#define PIN_AIN1   1
#define PIN_AIN2   3
#define PIN_PWMB   4
#define PIN_BIN1   5
#define PIN_BIN2   6
#define PIN_STBY   7
#define PIN_SERVO 10

/* Flip if a rear wheel runs backward. */
#define MOTOR_A_INVERT 0
#define MOTOR_B_INVERT 0

#define PWM_FREQ_HZ    20000
#define PWM_RES        LEDC_TIMER_10_BIT
#define PWM_MAX        ((1 << 10) - 1)

#define SERVO_FREQ_HZ  50
#define SERVO_RES      LEDC_TIMER_14_BIT
#define SERVO_TRIM     0
#define SERVO_RANGE_US 400
#define SERVO_CENTER_US 1500
#define SERVO_MIN_US   1000
#define SERVO_MAX_US   2000

#define FAILSAFE_US    300000
#define DEADBAND       40
#define WIFI_CH        1

static volatile int16_t g_thr;
static volatile int16_t g_steer;
static volatile int64_t g_last_us;

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

static void motor_set(int16_t thr)
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
    int a_fwd = MOTOR_A_INVERT ? !fwd : fwd;
    int b_fwd = MOTOR_B_INVERT ? !fwd : fwd;
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

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    if (len != (int)sizeof(rc_pkt_t)) {
        return;
    }
    const rc_pkt_t *p = (const rc_pkt_t *)data;
    if (p->magic != PKT_MAGIC) {
        return;
    }
    int16_t t = p->throttle;
    int16_t s = p->steer;
    if (t > 1000) {
        t = 1000;
    } else if (t < -1000) {
        t = -1000;
    }
    if (s > 1000) {
        s = 1000;
    } else if (s < -1000) {
        s = -1000;
    }
    g_thr = t;
    g_steer = s;
    g_last_us = esp_timer_get_time();
}

static void drive_task(void *arg)
{
    (void)arg;
    for (;;) {
        int16_t thr = g_thr;
        int16_t st = g_steer;
        if (esp_timer_get_time() - g_last_us > FAILSAFE_US) {
            thr = 0;
            st = 0;
        }
        motor_set(thr);
        servo_pulse_us(SERVO_CENTER_US + SERVO_TRIM + (int)st * SERVO_RANGE_US / 1000);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void wifi_now_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "MAC %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
}

static void io_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) |
                        (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2) |
                        (1ULL << PIN_STBY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_STBY, 0);

    ledc_timer_config_t t0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t0));

    ledc_timer_config_t t1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = SERVO_RES,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t1));

    ledc_channel_config_t ch[] = {
        { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
          .timer_sel = LEDC_TIMER_0, .gpio_num = PIN_PWMA, .duty = 0,
          .intr_type = LEDC_INTR_DISABLE },
        { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
          .timer_sel = LEDC_TIMER_0, .gpio_num = PIN_PWMB, .duty = 0,
          .intr_type = LEDC_INTR_DISABLE },
        { .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2,
          .timer_sel = LEDC_TIMER_1, .gpio_num = PIN_SERVO, .duty = 0,
          .intr_type = LEDC_INTR_DISABLE },
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ch[i]));
    }
    servo_pulse_us(SERVO_CENTER_US + SERVO_TRIM);
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    io_init();
    wifi_now_init();
    xTaskCreate(drive_task, "drive", 2048, NULL, 5, NULL);
}
