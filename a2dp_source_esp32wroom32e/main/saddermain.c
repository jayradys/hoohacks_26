#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "ultrasonic.h" // Your ultrasonic library

#define TAG "A2DP_ULTRASONIC"

/* ---------- Ultrasonic config ---------- */
#define MAX_DISTANCE_CM 500
#define THRESHOLD_CM 50
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

/* ---------- A2DP audio config ---------- */
#define SAMPLE_RATE 44100
#define TONE_FREQ 2000 // 2 kHz
#define CHANNELS 2
#define BITS_PER_SAMPLE 16
#define PI 3.14159265f

/* Global flag: whether we should send tone data or silence */
static bool g_play_tone = false;

/* Sine wave generation state */
static int16_t sine_table[256];
static int sine_idx = 0;

/* ---------- Forward declarations ---------- */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
static void ultrasonic_task(void *pvParameters);

/* ---------- Helper: build a sine table ---------- */
static void build_sine_table(void)
{
    for (int i = 0; i < 256; i++)
    {
        float theta = 2.0f * PI * ((float)i / 256.0f);
        sine_table[i] = (int16_t)(32767.0f * sinf(theta));
    }
}

/* ---------- A2DP data callback ---------- */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    /* This callback is called by the A2DP stack when it needs more PCM data.
       We ignore the incoming "data" pointer (not used here) and generate our own.
       IMPORTANT: This callback runs in BT context, do NOT block for long. */

    uint16_t *pcm = (uint16_t *)data;
    uint32_t samples = len / (BITS_PER_SAMPLE / 8) / CHANNELS; // stereo 16-bit

    for (uint32_t i = 0; i < samples; i++)
    {
        int16_t sample_val = 0;

        if (g_play_tone)
        {
            // Generate 2 kHz tone using sine_table.
            // To get the right frequency, step through the sine table accordingly.
            // Example: step size = TONE_FREQ * 256 / SAMPLE_RATE
            static int step = 0;
            if (step == 0)
            {
                step = (int)((float)TONE_FREQ * 256.0f / (float)SAMPLE_RATE);
                if (step <= 0)
                    step = 1;
            }

            sample_val = sine_table[sine_idx];
            sine_idx = (sine_idx + step) & 0xFF; // wrap around 256 entries
        }
        else
        {
            // Silence when not playing tone
            sample_val = 0;
        }

        // Stereo: write same sample to L and R
        pcm[2 * i] = sample_val;
        pcm[2 * i + 1] = sample_val;
    }
}

/* ---------- Ultrasonic task ---------- */
static void ultrasonic_task(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);

    while (1)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            ESP_LOGW(TAG, "Ultrasonic error: %d", res);
            // On error, do not play tone
            g_play_tone = false;
        }
        else
        {
            float distance_cm = distance * 100.0f;
            ESP_LOGI(TAG, "Distance: %.2f cm", distance_cm);

            if (distance_cm < THRESHOLD_CM)
            {
                ESP_LOGI(TAG, "Object in range (< %d cm) -> play tone", THRESHOLD_CM);
                g_play_tone = true;
            }
            else
            {
                g_play_tone = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ---------- Bluetooth / A2DP init logic ---------- */
/* This part must be consistent with Espressif's a2dp_source example */

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;
    default:
        break;
    }
}

static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    // Implement AVRCP remote control callbacks if needed
    (void)event;
    (void)param;
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Init sine table");
    build_sine_table();

    ESP_LOGI(TAG, "Init BT controller");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Init Bluedroid");
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* Set ESP32 device name (this is what AirPods will see) */
    esp_bt_dev_set_device_name("ESP32_Akshaj_A2DP");

    /* Register A2DP source callbacks */
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);

    /* Initialize A2DP source */
    esp_a2d_source_init();

    /* Optional: AVRCP controller for play/pause etc */
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_app_avrc_ct_cb);

    /* Start device discoverability / connect to a known sink
       – In the original example, they use esp_bt_gap_xxx calls to discover
         and connect to a chosen device (by address). For AirPods, you’ll have
         to pair once and then possibly hardcode or store their BT address.
    */

    ESP_LOGI(TAG, "Bluetooth A2DP source initialized, start scanning/pairing from example code");

    /* Start ultrasonic task */
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);
}