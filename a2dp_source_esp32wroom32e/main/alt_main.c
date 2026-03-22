/*#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <ultrasonic.h>
#include <esp_err.h>

#define MAX_DISTANCE_CM 500  // 5m max
#define THRESHOLD_CM    50

#define TRIGGER_GPIO    5
#define ECHO_GPIO       18

// Buzzer pin (change to your wiring)
#define BUZZER_GPIO     4

static void buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Start with buzzer off
    gpio_set_level(BUZZER_GPIO, 0);
}

static void buzzer_on(void)
{
    gpio_set_level(BUZZER_GPIO, 1);
}

static void buzzer_off(void)
{
    gpio_set_level(BUZZER_GPIO, 0);
}

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);
    buzzer_init();

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                printf("Cannot ping (device is in invalid state)\n");
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                printf("Ping timeout (no device found)\n");
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                printf("Echo timeout (i.e. distance too big)\n");
                break;
            default:
                printf("%s\n", esp_err_to_name(res));
            }
            // On error, ensure buzzer is off
            buzzer_off();
        }
        else
        {
            float distance_cm = distance * 100.0f;
            printf("Distance: %0.04f cm\n", distance_cm);

            if (distance_cm < THRESHOLD_CM)
            {
                printf("Object detected within %d cm!\n", THRESHOLD_CM);
                // Turn buzzer ON when object is close
                buzzer_on();
            }
            else
            {
                // Turn buzzer OFF when object is far
                buzzer_off();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test",
                configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}*/
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include <driver/ledc.h>

#define BUZZER_GPIO 4
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER LEDC_TIMER_0

#define MAX_DISTANCE_CM 500 // 5m max
#define THRESHOLD_CM 50

#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

static void buzzer_pwm_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BUZZER_TIMER,
        .freq_hz = 2000, // 2 kHz tone
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BUZZER_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel_conf);
}

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);
    buzzer_pwm_init();

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                printf("Cannot ping (device is in invalid state)\n");
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                printf("Ping timeout (no device found)\n");
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                printf("Echo timeout (i.e. distance too big)\n");
                break;
            default:
                printf("%s\n", esp_err_to_name(res));
            }
            // On error, ensure buzzer is off
            buzzer_off();
        }
        else
        {
            float distance_cm = distance * 100.0f;
            printf("Distance: %0.04f cm\n", distance_cm);

            if (distance_cm < THRESHOLD_CM)
            {
                printf("Object detected within %d cm!\n", THRESHOLD_CM);
                // Turn buzzer ON when object is close
                buzzer_on();
            }
            else
            {
                // Turn buzzer OFF when object is far
                buzzer_off();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test",
                configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}