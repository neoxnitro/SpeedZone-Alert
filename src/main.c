/* Minimal ESP-IDF app_main to start gamecube TX and push test controller data */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "gamecube_controller.h"
#include "driver/gpio.h"

static const char *TAG = "main";

static QueueHandle_t g_queue = NULL;

static void test_sender_task(void *arg)
{
    controller_data msg;
    memset(&msg, 0, sizeof(msg));

    while (1)
    {
        // Example: press A for one packet, then release
        msg.a_button = 1;
        // center analog values
        msg.joystick_x = 128;
        msg.joystick_y = 128;
        msg.c_stick_x = 128;
        msg.c_stick_y = 128;
        msg.l_bumper = 0;
        msg.r_bumper = 0;

        xQueueSend(g_queue, &msg, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Sent A=1");

        vTaskDelay(pdMS_TO_TICKS(200));

        msg.a_button = 0;
        xQueueSend(g_queue, &msg, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Sent A=0");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting app_main");

    // Create queue for controller data
    g_queue = xQueueCreate(8, sizeof(controller_data));
    if (g_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    // Configure gamecube component
    gamecube_rx_config cfg;
    cfg.input_pin = GPIO_NUM_4; // change if needed
    cfg.gamecube_data_queue = g_queue;
    cfg.ring_buffer_size = 2048;

    esp_err_t err = gamecube_tx_start(cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gamecube_tx_start failed: %d", err);
        return;
    }

    xTaskCreate(test_sender_task, "test_sender", 4096, NULL, 5, NULL);
}
