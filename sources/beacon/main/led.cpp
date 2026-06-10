#include "led.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_PIN     GPIO_NUM_2
#define BLINK_DELAY 120 // ms

static led_state actual_state = E_LED_OFF;

void led_blink(uint8_t nb_blink)
{
	led_set_state(E_LED_OFF);
	vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));

	for (int i = 0; i < (nb_blink * 2); i++)
	{
		led_set_state((i % 2) == 0 ? E_LED_ON : E_LED_OFF);
		vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
	}
}

void led_init(void)
{
	gpio_reset_pin(LED_PIN);
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	led_set_state(E_LED_OFF);
	actual_state = E_LED_OFF;
}

void led_set_state(led_state new_state)
{
	switch (new_state)
	{
		case E_LED_OFF:
			gpio_set_level(LED_PIN, 1); // active-low LED
			actual_state = new_state;
			break;
		case E_LED_ON:
			gpio_set_level(LED_PIN, 0);
			actual_state = new_state;
			break;
		default:
			printf("Error: wrong led state\n");
			break;
	}
}

void led_toggle_state(void)
{
	if (actual_state == E_LED_OFF)
		led_set_state(E_LED_ON);
	else if (actual_state == E_LED_ON)
		led_set_state(E_LED_OFF);
	else
		printf("Error: wrong led state\n");
}
