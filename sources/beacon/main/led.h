#ifndef LED_HEADER
#define LED_HEADER

typedef enum {
	E_LED_OFF = 0,
	E_LED_ON = 1,
} led_state;

void led_init();
void led_set_state(led_state new_state);
void led_toggle_state();
void led_blink(uint8_t nb_blink); 

#endif /* LED_HEADER */
