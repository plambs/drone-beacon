#include <Arduino.h>
#include "led.h"

#define LED_PIN 2
#define BLINK_DELAY 120 //ms

static led_state actual_state = E_LED_OFF;

void led_blink(uint8_t nb_blink)
{
	led_set_state(E_LED_OFF);
	delay(BLINK_DELAY);

	for(int i = 0; i < (nb_blink * 2); i++)
	{
		if((i % 2) == 0)
		{
			led_set_state(E_LED_ON);
		} else {
			led_set_state(E_LED_OFF);
		}
		delay(BLINK_DELAY);
	}
}

void led_init()
{
    pinMode(LED_PIN, OUTPUT);
	led_set_state(E_LED_OFF);
	actual_state = E_LED_OFF;
}

void led_set_state(led_state new_state)
{
	switch(new_state)
	{
		case E_LED_OFF:
            digitalWrite(LED_PIN, HIGH);
			actual_state = new_state;
			break;
		case E_LED_ON:
            digitalWrite(LED_PIN, LOW);
			actual_state = new_state;
			break;
		default:
            printf("Error: wrong led state");
			break;
	}
}

void led_toggle_state()
{
	if(actual_state == E_LED_OFF)
	{
		led_set_state(E_LED_ON);	
	}
	else if (actual_state == E_LED_ON)
	{
		led_set_state(E_LED_OFF);	
	}
	else
	{
		printf("Error: wrong led state");
	}
}
