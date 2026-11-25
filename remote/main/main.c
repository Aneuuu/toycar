#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "packet.h"

static const char *TAG = "remote";

#define ADC_Y_CH     ADC_CHANNEL_0 /* GPIO0, throttle */
#define ADC_X_CH     ADC_CHANNEL_1 /* GPIO1, steer */
#define INVERT_THR   0
#define INVERT_STR   0
#define DEADBAND     40
#define WIFI_CH      1
#define SEND_MS      20
#define CAL_SAMPLES  64
#define ADC_MAX      4095

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static int map_axis(int raw, int center, int invert)
{
    int v = raw - center;
    int span = center > (ADC_MAX - center) ? center : (ADC_MAX - center);
    if (span < 1) {
        span = 1;
    }
    int out = v * 1000 / span;
    if (out > 1000) {
        out = 1000;
    } else if (out < -1000) {
        out = -1000;
    }
    if (out > -DEADBAND && out < DEADBAND) {
        out = 0;
    }
    return invert ? -out : out;
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

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = WIFI_CH;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    wifi_now_init();

    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &adc));

    adc_oneshot_chan_cfg_t ccfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_Y_CH, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_X_CH, &ccfg));

    int sumy = 0, sumx = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        int y, x;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_Y_CH, &y));
        ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_X_CH, &x));
        sumy += y;
        sumx += x;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    int cy = sumy / CAL_SAMPLES;
    int cx = sumx / CAL_SAMPLES;
    ESP_LOGI(TAG, "center Y=%d X=%d (leave stick alone at boot)", cy, cx);

    uint8_t seq = 0;
    for (;;) {
        int y, x;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_Y_CH, &y));
        ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_X_CH, &x));
        rc_pkt_t p = {
            .magic = PKT_MAGIC,
            .throttle = (int16_t)map_axis(y, cy, INVERT_THR),
            .steer = (int16_t)map_axis(x, cx, INVERT_STR),
            .seq = seq++,
        };
        esp_now_send(BCAST, (const uint8_t *)&p, sizeof(p));
        vTaskDelay(pdMS_TO_TICKS(SEND_MS));
    }
}
