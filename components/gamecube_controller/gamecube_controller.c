#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gamecube_controller.h"
#include <string.h>
#include <sys/time.h>
#include "driver/rmt.h"

#define RMT_CLOCK_SPEED 80000000
#define RMT_CLOCK_DIVIDER 80          // 1 RMT tick = 1 microsecond
#define RMT_RX_IDLE_THRESHOLD_US 9500 // Idle threshold for the remote receiver

static gamecube_rx_config *rx_config = NULL;

static const rmt_item32_t zero_bit = {
    .duration0 = 3, // 3 microseconds low
    .level0 = 0,
    .duration1 = 1, // 1 microsecond high
    .level1 = 1,
};

static const rmt_item32_t one_bit = {
    .duration0 = 1, // 1 microsecond low
    .level0 = 0,
    .duration1 = 3, // 3 microseconds high
    .level1 = 1,
};

// -- helper routines (read/write bytes, convert pulses <-> controller_data) --
static inline uint8_t IRAM_ATTR read_byte(rmt_item32_t *items)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
    {
        bool is_one = items[i].duration0 < items[i].duration1;
        val |= (is_one << (7 - i));
    }
    return val;
}

static inline void IRAM_ATTR write_byte(uint8_t b, rmt_item32_t *items)
{
    for (int i = 0; i < 8; i++)
    {
        bool is_one = ((b >> (7 - i)) & 1) == 1;
        items[i] = is_one ? one_bit : zero_bit;
    }
}

void write_controller_bytes(controller_data *controller, uint8_t *dst)
{
    uint8_t byte_0 = 0;
    uint8_t bit_counter = 0;

    byte_0 |= (controller->start_button << bit_counter++);
    byte_0 |= (controller->y_button << bit_counter++);
    byte_0 |= (controller->x_button << bit_counter++);
    byte_0 |= (controller->b_button << bit_counter++);
    byte_0 |= (controller->a_button << bit_counter++);
    byte_0 |= (controller->l_button << bit_counter++);
    byte_0 |= (controller->r_button << bit_counter++);
    byte_0 |= (controller->z_button << bit_counter++);

    uint8_t byte_1 = 0;
    bit_counter = 0;
    byte_1 |= (controller->dpad_up_button << bit_counter++);
    byte_1 |= (controller->dpad_down_button << bit_counter++);
    byte_1 |= (controller->dpad_right_button << bit_counter++);
    byte_1 |= (controller->dpad_left_button << bit_counter++);

    dst[0] = byte_0;
    dst[1] = byte_1;
    dst[2] = controller->joystick_x;
    dst[3] = controller->joystick_y;
    dst[4] = controller->c_stick_x;
    dst[5] = controller->c_stick_y;
    dst[6] = controller->l_bumper;
    dst[7] = controller->r_bumper;
}

void controller_from_bytes(uint8_t *src, controller_data *controller)
{
    uint8_t bit_counter = 0;
    controller->start_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->y_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->x_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->b_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->a_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->l_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->r_button = ((1 << bit_counter++) & src[0]) != 0;
    controller->z_button = ((1 << bit_counter++) & src[0]) != 0;

    bit_counter = 0;
    controller->dpad_up_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_down_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_right_button = ((1 << bit_counter++) & src[1]) != 0;
    controller->dpad_left_button = ((1 << bit_counter++) & src[1]) != 0;

    controller->joystick_x = src[2];
    controller->joystick_y = src[3];
    controller->c_stick_x = src[4];
    controller->c_stick_y = src[5];
    controller->l_bumper = src[6];
    controller->r_bumper = src[7];
}

void controller_from_pulses(rmt_item32_t *pulses, controller_data *controller)
{
    controller->start_button = pulses[28].duration0 < pulses[28].duration1;
    controller->y_button = pulses[29].duration0 < pulses[29].duration1;
    controller->x_button = pulses[30].duration0 < pulses[30].duration1;
    controller->b_button = pulses[31].duration0 < pulses[31].duration1;
    controller->a_button = pulses[32].duration0 < pulses[32].duration1;

    controller->l_button = pulses[34].duration0 < pulses[34].duration1;
    controller->r_button = pulses[35].duration0 < pulses[35].duration1;
    controller->z_button = pulses[36].duration0 < pulses[36].duration1;
    controller->dpad_up_button = pulses[37].duration0 < pulses[37].duration1;
    controller->dpad_down_button = pulses[38].duration0 < pulses[38].duration1;
    controller->dpad_right_button = pulses[39].duration0 < pulses[39].duration1;
    controller->dpad_left_button = pulses[40].duration0 < pulses[40].duration1;

    controller->joystick_x = read_byte(&pulses[41]);
    controller->joystick_y = read_byte(&pulses[49]);
    controller->c_stick_x = read_byte(&pulses[57]);
    controller->c_stick_y = read_byte(&pulses[65]);
    controller->l_bumper = read_byte(&pulses[73]);
    controller->r_bumper = read_byte(&pulses[81]);
}

void IRAM_ATTR controller_to_pulses(controller_data *controller, rmt_item32_t *pulses)
{
    pulses[0] = zero_bit;
    pulses[1] = zero_bit;
    pulses[2] = zero_bit;
    pulses[3] = controller->start_button ? one_bit : zero_bit;
    pulses[4] = controller->y_button ? one_bit : zero_bit;
    pulses[5] = controller->x_button ? one_bit : zero_bit;
    pulses[6] = controller->b_button ? one_bit : zero_bit;
    pulses[7] = controller->a_button ? one_bit : zero_bit;
    pulses[8] = one_bit;
    pulses[9] = controller->l_button ? one_bit : zero_bit;
    pulses[10] = controller->r_button ? one_bit : zero_bit;
    pulses[11] = controller->z_button ? one_bit : zero_bit;
    pulses[12] = controller->dpad_up_button ? one_bit : zero_bit;
    pulses[13] = controller->dpad_down_button ? one_bit : zero_bit;
    pulses[14] = controller->dpad_right_button ? one_bit : zero_bit;
    pulses[15] = controller->dpad_left_button ? one_bit : zero_bit;

    write_byte(controller->joystick_x, &pulses[16]);
    write_byte(controller->joystick_y, &pulses[24]);
    write_byte(controller->c_stick_x, &pulses[32]);
    write_byte(controller->c_stick_y, &pulses[40]);
    write_byte(controller->l_bumper, &pulses[48]);
    write_byte(controller->r_bumper, &pulses[56]);
    pulses[64] = one_bit;
}

void print_controller_data(controller_data *controller)
{
    printf("S: %u, Y: %u, X: %u, B: %u, A: %u, L: %u, R: %u, Z: %u, UP: %u, DOWN: %u, RIGHT: %u, LEFT: %u, Joystick: (%u, %u), C-Stick: (%u, %u), Bumps: (%u, %u)\n",
           controller->start_button, controller->y_button, controller->x_button, controller->b_button,
           controller->a_button, controller->l_button, controller->r_button, controller->z_button,
           controller->dpad_up_button, controller->dpad_down_button, controller->dpad_right_button, controller->dpad_left_button,
           controller->joystick_x, controller->joystick_y, controller->c_stick_x, controller->c_stick_y, controller->l_bumper, controller->r_bumper);
}

// Minimal start functions that create tasks - using original repo logic is more involved.
static void gamecube_rx_task(void *arg)
{
    // Placeholder: full implementation in original repo
    vTaskDelete(NULL);
}

static void gamecube_tx_task(void *arg)
{
    // Placeholder: full implementation in original repo
    vTaskDelete(NULL);
}

esp_err_t gamecube_rx_start(gamecube_rx_config config)
{
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));
    xTaskCreate(gamecube_rx_task, "gamecube_rx_task", 1024 * 2, NULL, 10, NULL);
    return ESP_OK;
}

esp_err_t gamecube_tx_start(gamecube_rx_config config)
{
    rx_config = malloc(sizeof(gamecube_rx_config));
    memcpy(rx_config, &config, sizeof(gamecube_rx_config));
    xTaskCreate(gamecube_tx_task, "gamecube_tx_task", 1024 * 4, NULL, 10, NULL);
    return ESP_OK;
}
