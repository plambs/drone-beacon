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

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_sleep.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "led.h"
#include "gps.h"
#include "beacon.h"
#include "config.h"
#include "nmea.h"

#include "log.h"

/* Forward declaration using the extern C so esp-idf can link */
extern "C" {
	void app_main(void);
}

static int _init_minimal_system()
{
	int ret = 0;

	// 1. Init NVS
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
			ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ret = nvs_flash_erase();
		fail_if_not_zero(ret, -1, "nvs_flash_erase failed, returned: %d\n", ret);
		ret = nvs_flash_init();
	}

	fail_if_not_zero(ret, -2, "nvs_flash_init failed, returned: %d\n", ret);

	// 2. Init TCP/IP stack
	ret = esp_netif_init();
	fail_if_not_zero(ret, -3, "esp_netif_init failed, returned: %d\n", ret);

	// 3. Init event loop
	ret = esp_event_loop_create_default();
	fail_if_not_zero(ret, -4, "esp_event_loop_create_default failed, returned: %d\n", ret);

	return 0;
}

#if SLEEP_MODE_ENABLED
#define WAKEUP_GPIO GPIO_NUM_4
static int _setup_sleep()
{
	int ret = 0;

	// Configure the chip to wakeup when the wakeup pin in pulled high (PPS froom GPS module).
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << WAKEUP_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    ret = gpio_config(&io_conf);
    fail_if_not_zero(ret, -1, "gpio_config failed, returned: %d\n", ret);

	// Wake when GPIO4 goes HIGH
    ret = esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);
    fail_if_not_zero(ret, -2, "esp_sleep_enable_ext0_wakeup failed, returned: %d\n", ret);

	// Also configure the chip to wakeup when the uart received some data.
	// TODO check if uart2 can wake up the chip

	return 0;
}
#endif

#if SLEEP_MODE_ENABLED
static int _flush_logs(void)
{
	int ret = 0;
	ret = fflush(stdout);
	fail_if_not_zero(ret, -1, "fflush failed, returned: %d\n", ret);
	return 0;
}
#endif

#if SLEEP_MODE_ENABLED
static int _go_to_sleep()
{
	int ret = 0;

	// Prevent immediate wakeup
	while (gpio_get_level(WAKEUP_GPIO))
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	// Stop wifi
	ret = esp_wifi_stop();
	fail_if_not_zero(ret, -1, "esp_wifi_stop failed, returned: %d\n", ret);

#if DEBUG_DISPLAY_SLEEP_LOGS
	printf("Entering light sleep...\n");
#endif

	ret = _flush_logs();
	fail_if_negative(ret, -2, "_flush_logs failed, returned: %d\n", ret);

#if DEBUG_DISPLAY_WAKE_UP_SOURCE_AND_DURATION
	int64_t t0 = esp_timer_get_time();
#endif

	// Go to sleep, this function will return when we wakeup.
	ret = esp_light_sleep_start();
	fail_if_not_zero(ret, -3, "esp_light_sleep_start failed, returned: %d\n", ret);

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
	ret = esp_wifi_start();
	fail_if_not_zero(ret, -4, "esp_wifi_start failed, returned: %d\n", ret);

	return 0;
}
#endif

void app_main(void)
{
	int ret = 0;

	ret = _init_minimal_system();
	log_if_negative(ret, "_init_minimal_system failed, returned: %d\n", ret);

#if SLEEP_MODE_ENABLED
	ret = _setup_sleep();
	log_if_negative(ret, "_setup_sleep failed, returned: %d\n", ret);
#endif

	log("Start main application\n");

	ret = led_init();
	log_if_negative(ret, "led_init failed, returned: %d\n", ret);

	ret = gps_init();
	log_if_negative(ret, "gps_init failed, returned: %d\n", ret);

	ret = beacon_init();
	log_if_negative(ret, "beacon_init failed, returned: %d\n", ret);

	printf("Init phase done, go in mainloop\n");

	while(1)
	{
		/**
		 * Run the gps mainloop, this get the char sended by the gps module,
		 * decode them and get all the usefull information for the beacon.
		 */
		ret = gps_get_data();
		log_if_negative(ret, "gps_get_data failed, returned: %d\n", ret);

		// if gps is dead -> todo reset and reconfigure gps
		if(gps_need_reset())
		{
			// Print info log
			log_error("No GPS detected\n");

			// Turn off the led when the beacon is not working.
			ret = led_blink(4);
			log_if_negative(ret, "led_blink failed, returned: %d\n", ret);

			// Reset of the gps module
			ret = gps_reset();
			log_if_negative(ret, "gps_reset failed, returned: %d\n", ret);

			// Wait some time then retry to get gps data
			vTaskDelay(pdMS_TO_TICKS(DELAY_BEFORE_RELOOPING_MS));
			continue;
		}

		// print number satelite, fix, home status and if data must be send
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
			ret = led_blink(3);
			log_if_negative(ret, "led_blink failed, returned: %d\n", ret);

			printf("Position unknown\n");

			// Position is not detected yet, we enable sleep mode only if position
			// is knowed, this assure us the gps module is working correctly. The no
			// position state should not exceed 3 min in correct condition so the power
			// usage is limited
			// Wait some time then retry to get gps data
			vTaskDelay(pdMS_TO_TICKS(DELAY_BEFORE_RELOOPING_NO_SLEEP_MS));
			continue;
		}

		// if position and no home -> set home
		if(!beacon_is_home_set())
		{
			// Blink the led
			ret = led_blink(2);
			log_if_negative(ret, "led_blink failed, returned: %d\n", ret);

			printf("Position detected but no home set\n");

			// job done, go to sleep;
			goto main_sleep;
		}

		// if home and it time to send beacon -> send beacon data
		if(beacon_data_must_be_send())
		{
			// toggle the LED to see beacon sended
			ret = led_toggle_state();
			log_if_negative(ret, "led_toggle_state failed, returned: %d\n", ret);

			printf("Send beacon\n");

			// Send the drone identification frame
			ret = beacon_send_data();
			log_if_negative(ret, "beacon_send_data failed, returned: %d\n", ret);
		}

main_sleep:
		/** Wait 10ms before going to the next iteration of the loop so the system can do something else
		 * like kicking the watchdog
		 */
		vTaskDelay(pdMS_TO_TICKS(DELAY_BEFORE_RELOOPING_MS));

#if SLEEP_MODE_ENABLED
		ret = _go_to_sleep();
		log_if_negative(ret, "_go_to_sleep failed, returned: %d\n", ret);
#else
		vTaskDelay(pdMS_TO_TICKS(DELAY_BEFORE_RELOOPING_NO_SLEEP_MS));
#endif
	}
}
