#include "led.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "log.h"

#define LED_PIN     GPIO_NUM_2
#define BLINK_DELAY 120 // ms

static led_state actual_state = E_LED_OFF;

int led_blink(uint8_t nb_blink)
{
	int ret = 0;

	ret = led_set_state(E_LED_OFF);
	fail_if_negative(ret, -1, "led_set_state(OFF) failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));

	for (int i = 0; i < (nb_blink * 2); i++)
	{
		ret = led_set_state((i % 2) == 0 ? E_LED_ON : E_LED_OFF);
		fail_if_negative(ret, -2, "led_set_state failed, returned: %d\n", ret);
		vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
	}

	return 0;
}

int led_init(void)
{
	int ret = 0;

	ret = gpio_reset_pin(LED_PIN);
	fail_if_not_zero(ret, -1, "gpio_reset_pin failed, returned: %d\n", ret);

	ret = gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	fail_if_not_zero(ret, -2, "gpio_set_direction failed, returned: %d\n", ret);

	ret = led_set_state(E_LED_OFF);
	fail_if_negative(ret, -3, "led_set_state failed, returned: %d\n", ret);

	actual_state = E_LED_OFF;

	return 0;
}

int led_set_state(led_state new_state)
{
	int ret = 0;

	switch (new_state)
	{
		case E_LED_OFF:
			ret = gpio_set_level(LED_PIN, 1); // active-low LED
			fail_if_not_zero(ret, -1, "gpio_set_level failed, returned: %d\n", ret);
			actual_state = new_state;
			break;
		case E_LED_ON:
			ret = gpio_set_level(LED_PIN, 0);
			fail_if_not_zero(ret, -2, "gpio_set_level failed, returned: %d\n", ret);
			actual_state = new_state;
			break;
		default:
			fail(-3, "wrong led state\n");
			break;
	}

	return 0;
}

int led_toggle_state(void)
{
	int ret = 0;

	if (actual_state == E_LED_OFF)
	{
		ret = led_set_state(E_LED_ON);
		fail_if_negative(ret, -1, "led_set_state(ON) failed, returned: %d\n", ret);
	}
	else if (actual_state == E_LED_ON)
	{
		ret = led_set_state(E_LED_OFF);
		fail_if_negative(ret, -2, "led_set_state(OFF) failed, returned: %d\n", ret);
	}
	else
	{
		fail(-3, "wrong led state\n");
	}

	return 0;
}
