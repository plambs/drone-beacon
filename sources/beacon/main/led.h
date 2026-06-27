#ifndef LED_HEADER
#define LED_HEADER

#include <stdint.h>

typedef enum {
	E_LED_OFF = 0,
	E_LED_ON = 1,
} led_state;

int led_init();
int led_set_state(led_state new_state);
int led_toggle_state();
int led_blink(uint8_t nb_blink);

#endif /* LED_HEADER */
