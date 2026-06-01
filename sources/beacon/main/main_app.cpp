/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Used with ESP32 + BN220, modified by Julien Launay 14/09/2020
 *  Using https://github.com/khancyr/TTGO_T_BEAM/tree/master/src. Thanks to Pierre Khancyr the original author of this project.
 *  With https://github.com/f5soh/balise_esp32/blob/droneID_FR_testing/droneID_FR.h
 */

#include <Arduino.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_sleep.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "led.h"
#include "gps.h"
#include "beacon.h"
#include "config.h"
#include "nmea.h"

/* Forward declaration using the extern C so esp-idf can link */
extern "C" {
	void app_main(void);
}

static void _init_minimal_system()
{
	esp_err_t ret;

	// 1. Init NVS
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
			ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	// 2. Init TCP/IP stack
	ESP_ERROR_CHECK(esp_netif_init());

	// 3. Init event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());
}

#if SLEEP_MODE_ENABLED
#define WAKEUP_GPIO GPIO_NUM_4
static void _setup_sleep()
{
	// Configure the chip to wakeup when the wakeup pin in pulled high (PPS froom GPS module).
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << WAKEUP_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

	// Wake when GPIO4 goes HIGH
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1));

	// Also configure the chip to wakeup when the uart received some data.
	// TODO check if uart2 can wake up the chip
}
#endif

#if SLEEP_MODE_ENABLED
static void _flush_logs(void)
{
	fflush(stdout);
	uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
}
#endif

#if SLEEP_MODE_ENABLED
static void _go_to_sleep()
{
	// Prevent immediate wakeup
	while (gpio_get_level(WAKEUP_GPIO))
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// Stop wifi
	esp_wifi_stop();

#if DEBUG_DISPLAY_SLEEP_LOGS
	printf("Entering light sleep...\n");
#endif

	_flush_logs();

#if DEBUG_DISPLAY_WAKE_UP_SOURCE_AND_DURATION
	int64_t t0 = esp_timer_get_time();
#endif

	// Go to sleep, this function will return when we wakeup.
	esp_light_sleep_start();

#if DEBUG_DISPLAY_WAKE_UP_SOURCE_AND_DURATION
	int64_t t1 = esp_timer_get_time();
#endif

#if DEBUG_DISPLAY_SLEEP_LOGS
	printf("Wakeup!\n");
#endif

#if DEBUG_DISPLAY_WAKE_UP_SOURCE_AND_DURATION
	printf("Sleep duration: %.3f ms\n", (t1 - t0) / 1000.0);

	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause == ESP_SLEEP_WAKEUP_EXT0)
	{
		printf("Wakeup from GPIO4 pulse\n");
	}
	else
	{
		printf("Other wake up source: %d\n", cause);
	}
#endif

	// Restart wifi
	esp_wifi_start();
}
#endif

void app_main(void)
{
    Serial.begin(SERIAL_DEFAUT_BAUDRATE);

	_init_minimal_system();

#if SLEEP_MODE_ENABLED
	_setup_sleep();
#endif

	led_init();

	gps_init();

	beacon_init();

	printf("Init phase done, go in mainloop\n");

	while(1)
	{
		/**
		 * Run the gps mainloop, this get the char sended by the gps module,
		 * decode them and get all the usefull information for the beacon.
		 */
		gps_get_data();

		// if gps is dead -> todo reset and reconfigure gps
		if(gps_need_reset())
		{
			// Print info log
			printf("[ERROR] No GPS detected\n");

			// Turn off the led when the beacon is not working.
			led_blink(4);

			// Reset of the gps module
			gps_reset();

			// Wait some time then retry to get gps data
			delay(DELAY_BEFORE_RELOOPING_MS);
			continue;
		}

		// TODO print number satelite, fix, home status and if data must be send
		printf("sat view: %d, sat fix: %d, hdop: %0.2f, home set: %d\n",
			nmea_get_satellites_in_view(),
			gps_get_satellites(),
			gps_get_precision(),
			beacon_is_home_set()
		);

		// if no position -> new try later
		if(!gps_position_detected())
		{
			// Blink the led
			led_blink(3);

			printf("Position unknown\n");

			// Position is not detected yet, we enable sleep mode only if position
			// is knowed, this assure us the gps module is working correctly. The no
			// position state should not exceed 3 min in correct condition so the power
			// usage is limited
			// Wait some time then retry to get gps data
			delay(DELAY_BEFORE_RELOOPING_NO_SLEEP_MS);
			continue;
		}

		// if position and no home -> set home
		if(!beacon_is_home_set())
		{
			// Blink the led
			led_blink(2);

			printf("Position detected but no home set\n");

			// job done, go to sleep;
			goto main_sleep;
		}

		// if home and it time to send beacon -> send beacon data
		if(beacon_data_must_be_send())
		{
			// toggle the LED to see beacon sended
			led_toggle_state();

			printf("Send beacon\n");

			// Send the drone identification frame
			beacon_send_data();
		}

main_sleep:
		/** Wait 10ms before going to the next iteration of the loop so the system can do something else
		 * like kicking the watchdog
		 */
		delay(DELAY_BEFORE_RELOOPING_MS);

#if SLEEP_MODE_ENABLED
		_go_to_sleep();
#else
		delay(DELAY_BEFORE_RELOOPING_NO_SLEEP_MS);
#endif
	}
}
