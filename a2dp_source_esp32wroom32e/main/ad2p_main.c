#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#include "bt_app_core.h"
#include "ultrasonic.h"

#define TAG "MAIN_ULTRA_A2DP"

/* ---------------- Ultrasonic + Buzzer config ---------------- */

#define MAX_DISTANCE_CM 500
#define THRESHOLD_CM 50

#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

// Passive buzzer on GPIO 4 using LEDC PWM
#define BUZZER_GPIO 4
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER LEDC_TIMER_0
#define BUZZER_FREQ_HZ 2000 // 2 kHz tone
#define BUZZER_DUTY 512     // 50% duty for 10-bit

/* ---------------- A2DP config ---------------- */

// Replace with your actual AirPods name for logs (name matching is optional here)
#define TARGET_DEVICE_NAME "Akshaj's AirPods Pro"

// You MUST replace this by your AirPods MAC address from a BT scan/log
static esp_bd_addr_t s_peer_bd_addr = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

// Audio tone parameters
#define TONE_SAMPLE_RATE 44100
#define TONE_FREQ_HZ 1000
#define TONE_CHANNELS 2
#define A2DP_PCM_SAMPLES_PER_CH 512 // per channel in each chunk

static bool s_bt_initialized = false;
static bool s_a2dp_connected = false;
static bool s_a2dp_playing = false;
static float s_phase = 0.0f;

/* ---------------- Buzzer helpers ---------------- */

static void buzzer_pwm_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BUZZER_TIMER,
        .freq_hz = BUZZER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BUZZER_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_TIMER,
        .duty = 0, // start OFF
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, BUZZER_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

/* ---------------- A2DP tone generation ---------------- */

static void generate_tone(int16_t *samples, size_t sample_count)
{
    float phase_inc = 2.0f * (float)M_PI * TONE_FREQ_HZ / (float)TONE_SAMPLE_RATE;

    for (size_t i = 0; i < sample_count; i++)
    {
        float value = sinf(s_phase);
        s_phase += phase_inc;
        if (s_phase >= 2.0f * (float)M_PI)
        {
            s_phase -= 2.0f * (float)M_PI;
        }

        int16_t v = (int16_t)(value * 16000); // amplitude

        // stereo: L and R
        samples[2 * i] = v;
        samples[2 * i + 1] = v;
    }
}

/* A2DP data callback: fill PCM buffer with tone */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    // 'data' is a buffer we must fill with PCM (16‑bit, stereo).
    int16_t *samples = (int16_t *)data;
    size_t sample_count = len / (sizeof(int16_t) * TONE_CHANNELS);
    generate_tone(samples, sample_count);
}

/* Forward declarations for GAP/A2DP callbacks dispatched via bt_app_core */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/* Dispatch wrappers so callbacks run in BT app task context */
static void bt_app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    bt_app_msg_t msg = {0};
    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = (bt_app_cb_t)bt_app_a2d_cb;

    if (param)
    {
        msg.param = malloc(sizeof(esp_a2d_cb_param_t));
        memcpy(msg.param, param, sizeof(esp_a2d_cb_param_t));
    }

    bt_app_work_dispatch(msg.cb, msg.event, msg.param,
                         sizeof(esp_a2d_cb_param_t), NULL);
}

static void bt_app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    bt_app_msg_t msg = {0};
    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = (bt_app_cb_t)bt_app_gap_cb;

    if (param)
    {
        msg.param = malloc(sizeof(esp_bt_gap_cb_param_t));
        memcpy(msg.param, param, sizeof(esp_bt_gap_cb_param_t));
    }

    bt_app_work_dispatch(msg.cb, msg.event, msg.param,
                         sizeof(esp_bt_gap_cb_param_t), NULL);
}

/* Actual GAP event handler (runs in BT app task context) */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Auth success: %s", param->auth_cmpl.device_name);
        }
        else
        {
            ESP_LOGE(TAG, "Auth failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    default:
        break;
    }
}

/* Actual A2DP event handler (runs in BT app task context) */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
        {
            s_a2dp_connected = true;
        }
        else
        {
            s_a2dp_connected = false;
            s_a2dp_playing = false;
        }
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED)
        {
            s_a2dp_playing = true;
        }
        else
        {
            s_a2dp_playing = false;
        }
        break;

    default:
        break;
    }
}

/* ---------------- A2DP helper functions ---------------- */

static void bt_init_a2dp_source(void)
{
    if (s_bt_initialized)
        return;

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Start the BT app dispatcher task
    bt_app_task_start_up();

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_callback));

    const char *dev_name = "ESP32_A2DP_ULTRA";
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(dev_name));

    // Register A2DP callbacks
    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_app_a2d_callback));
    ESP_ERROR_CHECK(esp_a2d_source_register_data_callback(bt_app_a2d_data_cb));
    ESP_ERROR_CHECK(esp_a2d_source_init());

    // Make device discoverable + connectable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    s_bt_initialized = true;
    ESP_LOGI(TAG, "Bluetooth A2DP source initialized");
}

static void bt_connect_to_peer(void)
{
    if (!s_bt_initialized)
        return;

    ESP_LOGI(TAG,
             "Trying to connect to %s at %02x:%02x:%02x:%02x:%02x:%02x",
             TARGET_DEVICE_NAME,
             s_peer_bd_addr[0], s_peer_bd_addr[1], s_peer_bd_addr[2],
             s_peer_bd_addr[3], s_peer_bd_addr[4], s_peer_bd_addr[5]);

    esp_err_t err = esp_a2d_source_connect(s_peer_bd_addr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "A2DP connect failed: %s", esp_err_to_name(err));
    }
}

static void bt_start_audio(void)
{
    if (!s_a2dp_connected)
    {
        bt_connect_to_peer();
        return;
    }

    if (!s_a2dp_playing)
    {
        ESP_LOGI(TAG, "Starting A2DP audio");
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
    }
}

static void bt_stop_audio(void)
{
    if (s_a2dp_playing)
    {
        ESP_LOGI(TAG, "Stopping A2DP audio");
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
        s_a2dp_playing = false;
    }
}

/* ---------------- Ultrasonic task ---------------- */

static void ultrasonic_task(void *arg)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);
    buzzer_pwm_init();
    bt_init_a2dp_source();

    while (1)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);

        if (res != ESP_OK)
        {
            ESP_LOGW(TAG, "Ultrasonic error: %s", esp_err_to_name(res));
            buzzer_off();
            bt_stop_audio();
        }
        else
        {
            float distance_cm = distance * 100.0f;
            ESP_LOGI(TAG, "Distance: %.2f cm", distance_cm);

            if (distance_cm < THRESHOLD_CM)
            {
                // Close: local buzzer + AirPods tone
                buzzer_on();
                bt_start_audio();
            }
            else
            {
                // Far: turn off both
                buzzer_off();
                bt_stop_audio();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------------- app_main ---------------- */

void app_main(void)
{
    // Just start the ultrasonic + BT control task;
    // all BT init is done inside it.
    xTaskCreate(ultrasonic_task,
                "ultrasonic_task",
                4096,
                NULL,
                5,
                NULL);
}