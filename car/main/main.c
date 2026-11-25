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
#include "packet.h"
#include "servo.h"
#include "tb6612.h"

static const char *TAG = "car";

#define PIN_SERVO 10

/* Flip if a rear wheel runs backward. */
#define MOTOR_A_INVERT 0
#define MOTOR_B_INVERT 0

#define SERVO_TRIM     0
#define SERVO_RANGE_US 400

#define FAILSAFE_US    300000
#define WIFI_CH        1

static volatile int16_t g_thr;
static volatile int16_t g_steer;
static volatile int64_t g_last_us;

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
        tb6612_set_throttle(thr);
        servo_set(st);
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

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    ESP_ERROR_CHECK(tb6612_init(MOTOR_A_INVERT, MOTOR_B_INVERT));
    ESP_ERROR_CHECK(servo_init(PIN_SERVO, SERVO_TRIM, SERVO_RANGE_US));
    wifi_now_init();
    xTaskCreate(drive_task, "drive", 2048, NULL, 5, NULL);
}
