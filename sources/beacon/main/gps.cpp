#include <TinyGPS++.h>
#include "gps.h"
#include "config.h"
#include "nmea.h"
#include "beacon.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "log.h"

#define GPS_INIT_TIME_MS             1000
#define GPS_RESET_TRIGGER_DELAY_MS     25
#define GPS_BAUDRATE_CHANGE_DELAY_MS  300
#define GPS_BAUDRATE_DEFAULT         9600
#define GPS_BAUDRATE_115200        115200
#define GPS_RX_PIN                     16
#define GPS_TX_PIN                     17
#define GPS_RESET_PIN           GPIO_NUM_18

static TinyGPSPlus gps;
static bool has_set_home = false;
static uint64_t time_since_last_reset = 0;

static uint64_t _millis(void)
{
	return (uint64_t)(esp_timer_get_time() / 1000LL);
}

static int _uart2_init(int baud_rate)
{
	int ret = 0;

	uart_config_t config;
	config.baud_rate           = baud_rate;
	config.data_bits           = UART_DATA_8_BITS;
	config.parity              = UART_PARITY_DISABLE;
	config.stop_bits           = UART_STOP_BITS_1;
	config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
	config.rx_flow_ctrl_thresh = 0;
	config.source_clk          = UART_SCLK_DEFAULT;

	if (!uart_is_driver_installed(UART_NUM_2)) {
		ret = uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0);
		fail_if_not_zero(ret, -1, "uart_driver_install failed, returned: %d\n", ret);
	}

	ret = uart_param_config(UART_NUM_2, &config);
	if (ret == ESP_ERR_NOT_SUPPORTED) {
		log_warn("uart_param_config: light sleep not supported, continuing\n");
		ret = 0;
	}
	fail_if_not_zero(ret, -2, "uart_param_config failed, returned: %d\n", ret);

	ret = uart_set_pin(UART_NUM_2, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	fail_if_not_zero(ret, -3, "uart_set_pin failed, returned: %d\n", ret);

	return 0;
}

static int _uart2_close(void)
{
	int ret = 0;

	if (!uart_is_driver_installed(UART_NUM_2))
	{
		return 0;
	}

	ret = uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(500));
	fail_if_not_zero(ret, -1, "uart_wait_tx_done failed, returned: %d\n", ret);

	ret = uart_flush(UART_NUM_2);
	fail_if_not_zero(ret, -2, "uart_flush failed, returned: %d\n", ret);

	ret = uart_driver_delete(UART_NUM_2);
	fail_if_not_zero(ret, -3, "uart_driver_delete failed, returned: %d\n", ret);

	return 0;
}

#define NMEA_ACK_INVALID_PACKET  0
#define NMEA_ACK_UNSUPPORTED     1
#define NMEA_ACK_ACTION_FAILED   2
#define NMEA_ACK_SUCCESS         3

static uint8_t _nmea_crc(const char *str)
{
	uint8_t crc = 0;
	const char *p = (*str == '$') ? str + 1 : str;
	while (*p && *p != '*')
		crc ^= (uint8_t)*p++;
	return crc;
}

static int _uart2_send_cmd(const char *str)
{
	int ret = 0;
	fail_if_null(str, -1, "str is NULL\n");

	char suffix[8];

	ret = snprintf(suffix, sizeof(suffix), "*%02X\r\n", _nmea_crc(str));
	fail_if_negative(ret, -2, "snprintf failed, returned: %d\n", ret);

	ret = uart_write_bytes(UART_NUM_2, str, strlen(str));
	fail_if_negative(ret, -3, "uart_write_bytes(cmd) failed, returned: %d\n", ret);

	ret = uart_write_bytes(UART_NUM_2, suffix, strlen(suffix));
	fail_if_negative(ret, -4, "uart_write_bytes(suffix) failed, returned: %d\n", ret);

	return 0;
}

static int _pmtk_cmd_id(const char *cmd)
{
	int ret = 0;
	fail_if_null(cmd, -1, "cmd is NULL\n");

	if (strncmp(cmd, "$PMTK", 5) != 0)
		return -2;

	(void)ret;
	return atoi(cmd + 5);
}

static int _read_nmea_line(char *buf, int buf_len, uint64_t deadline)
{
	int ret = 0;
	fail_if_null(buf, -1, "buf is NULL\n");
	fail_if_inferior_or_equal(buf_len, 0, -2, "buf_len is invalid: %d\n", buf_len);

	int idx = 0;
	while ((uint64_t)esp_timer_get_time() < deadline)
	{
		uint8_t c;
		ret = uart_read_bytes(UART_NUM_2, &c, 1, pdMS_TO_TICKS(1));
		fail_if_negative(ret, -3, "uart_read_bytes failed, returned: %d\n", ret);
		if (ret > 0)
		{
			if (c == '\n')
			{
				buf[idx] = '\0';
				return 0;
			}
			else if (c != '\r' && idx < buf_len - 1)
			{
				buf[idx++] = c;
			}
		}
	}
	return -4;
}

// Returns NMEA_ACK_* on success, -1 on CRC mismatch, -2 on timeout.
static int _wait_gps_ack(int expected_cmd_id)
{
	int ret = 0;
	char line[128];
	uint64_t deadline = (uint64_t)esp_timer_get_time() + 1000000ULL; // 1000 ms

	while (1)
	{
		ret = _read_nmea_line(line, sizeof(line), deadline);
		if (ret < 0)
			return -2;

		if (strncmp(line, "$PMTK001,", 9) != 0)
			continue;

		char *star = strchr(line, '*');
		if (star == NULL)
			continue;

		uint8_t received_crc = (uint8_t)strtol(star + 1, NULL, 16);
		if (_nmea_crc(line) != received_crc)
		{
			printf("ACK CRC mismatch for cmd %d\n", expected_cmd_id);
			return -1;
		}

		int cmd_id, flag;
		if (sscanf(line + 9, "%d,%d", &cmd_id, &flag) != 2)
			continue;

		if (cmd_id != expected_cmd_id)
			continue;

		return flag;
	}
}

static int _send_gps_cmd_and_check_result(const char *description, const char *cmd)
{
	int ret = 0;
	fail_if_null(description, -1, "description is NULL\n");
	fail_if_null(cmd, -2, "cmd is NULL\n");

	ret = _pmtk_cmd_id(cmd);
	fail_if_negative(ret, -3, "cannot extract cmd id from: %s, returned: %d\n", cmd, ret);
	int cmd_id = ret;

	for (int attempt = 1; attempt <= 3; attempt++)
	{
		ret = _uart2_send_cmd(cmd);
		fail_if_negative(ret, -4, "_uart2_send_cmd failed, returned: %d\n", ret);

		ret = _wait_gps_ack(cmd_id);
		if (ret == NMEA_ACK_SUCCESS)
		{
			printf("%s: OK\n", description);
			return 0;
		}

		if (ret < 0)
			printf("%s: attempt %d/3 timeout or CRC error (%d)\n", description, attempt, ret);
		else
			printf("%s: attempt %d/3 failed with status %d\n", description, attempt, ret);
	}

	fail(-5, "%s failed after 3 attempts\n", description);
}

static int _print_gps_firmware_version(void)
{
	int ret = 0;
	char line[128] = {0};

	uint8_t dummy;
	while (uart_read_bytes(UART_NUM_2, &dummy, 1, 0) > 0);

	ret = _uart2_send_cmd("$PMTK605");
	fail_if_negative(ret, -1, "_uart2_send_cmd failed, returned: %d\n", ret);

	uint64_t deadline = (uint64_t)esp_timer_get_time() + 1000000ULL;
	ret = _read_nmea_line(line, sizeof(line), deadline);
	fail_if_negative(ret, -2, "no firmware version response, returned: %d\n", ret);

	printf("Quectel L96-M33 fw version: %s\n", line);
	return 0;
}

#if DEBUG_DISPLAY_HOME_STATUS
static int _display_home_status(void)
{
	int ret = 0;
	static uint64_t home_time = 0;
	uint64_t now = _millis();

	if (now - home_time > LOG_PERIOD_MS)
	{
		printf("Home is set: %s, satellites value: %ld (wanted: %d), hdop: %f (wanted: %f)\n",
				has_set_home ? "YES" : "NO",
				gps.satellites.value(),
				WANTED_SATELLITES,
				gps.hdop.hdop(),
				WANTED_PRECISION);
		home_time = now;
	}

	(void)ret;
	return 0;
}
#endif

#if DEBUG_DISPLAY_GPS_DATA
static int _display_gps_data(void)
{
	int ret = 0;
	static uint64_t gpsSec = 0;
	static uint64_t gpsMap = 0;
	uint64_t now = _millis();

	if (now - gpsMap > LOG_PERIOD_MS) {
		printf("\nPositioning (%llu)\n", gpsSec++);
		printf("satellites with fix:%lu\n", gps.satellites.value());
		printf("UTC:%d:%d:%d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
		printf("LNG:%.4f - LAT:%.4f\n", gps.location.lng(), gps.location.lat());
		gpsMap = now;
	}

	(void)ret;
	return 0;
}
#endif

static int _gps_configure(void)
{
	int ret = 0;

	// Change GPS baudrate to 115200, there is NO ACK for this command
	ret = _uart2_send_cmd("$PMTK251,115200");
	fail_if_negative(ret, -1, "_uart2_send_cmd(baudrate) failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(GPS_BAUDRATE_CHANGE_DELAY_MS));

	ret = _uart2_close();
	fail_if_negative(ret, -2, "_uart2_close failed, returned: %d\n", ret);

	// Restart at new baudrate
	ret = _uart2_init(GPS_BAUDRATE_115200);
	fail_if_negative(ret, -3, "_uart2_init(115200) failed, returned: %d\n", ret);

	ret = _print_gps_firmware_version();
	fail_if_negative(ret, -4, "_print_gps_firmware_version failed, returned: %d\n", ret);

	printf("Configure GPS module:\n");

	ret = _send_gps_cmd_and_check_result("Enable PPS", "$PMTK255,1");
	fail_if_negative(ret, -5, "_send_gps_cmd_and_check_result(PPS) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("PPS always 100ms", "$PMTK285,4,100");
	fail_if_negative(ret, -6, "_send_gps_cmd_and_check_result(PPS 100ms) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Normal navigation mode", "$PMTK886,0");
	fail_if_negative(ret, -7, "_send_gps_cmd_and_check_result(nav mode) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Disable EASY", "$PMTK869,1,0");
	fail_if_negative(ret, -8, "_send_gps_cmd_and_check_result(EASY) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Enable jamming detection", "$PMTK838,1");
	fail_if_negative(ret, -9, "_send_gps_cmd_and_check_result(jamming) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("GPS + Glonass + Galileo", "$PMTK353,1,1,1,0,0");
	fail_if_negative(ret, -10, "_send_gps_cmd_and_check_result(constellations) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Disable QZSS", "$PMTK352,0");
	fail_if_negative(ret, -11, "_send_gps_cmd_and_check_result(QZSS) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Enable RMC, GGA, GSA and GSV", "$PMTK314,0,1,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0");
	fail_if_negative(ret, -12, "_send_gps_cmd_and_check_result(NMEA output) failed, returned: %d\n", ret);

	ret = _send_gps_cmd_and_check_result("Enable AIC", "$PMTK286,1");
	fail_if_negative(ret, -13, "_send_gps_cmd_and_check_result(AIC) failed, returned: %d\n", ret);

	printf("Done\n");

	return 0;
}

static int _trigger_reset_pin(void)
{
	int ret = 0;

	printf("Trigger GPS module reset\n");

	ret = gpio_set_level(GPS_RESET_PIN, 0);
	fail_if_not_zero(ret, -1, "gpio_set_level(0) failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(GPS_RESET_TRIGGER_DELAY_MS));

	ret = gpio_set_level(GPS_RESET_PIN, 1);
	fail_if_not_zero(ret, -2, "gpio_set_level(1) failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(GPS_RESET_TRIGGER_DELAY_MS));

	return 0;
}

static int _init_gps(void)
{
	int ret = 0;

	ret = _uart2_init(GPS_BAUDRATE_DEFAULT);
	fail_if_negative(ret, -1, "_uart2_init failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(GPS_INIT_TIME_MS));

	ret = _gps_configure();
	fail_if_negative(ret, -2, "_gps_configure failed, returned: %d\n", ret);

	time_since_last_reset = _millis();

	return 0;
}

int gps_reset(void)
{
	int ret = 0;

	ret = _uart2_close();
	fail_if_negative(ret, -1, "_uart2_close failed, returned: %d\n", ret);

	ret = _trigger_reset_pin();
	fail_if_negative(ret, -2, "_trigger_reset_pin failed, returned: %d\n", ret);

	ret = _init_gps();
	fail_if_negative(ret, -3, "_init_gps failed, returned: %d\n", ret);

	return 0;
}

int gps_init(void)
{
	int ret = 0;

	// Init the gps module is nothing more than setting the reset
	// gpio as output and launching the gps reset sequence.
	ret = gpio_set_direction(GPS_RESET_PIN, GPIO_MODE_OUTPUT);
	fail_if_not_zero(ret, -1, "gpio_set_direction failed, returned: %d\n", ret);

	ret = gpio_set_level(GPS_RESET_PIN, 1);
	fail_if_not_zero(ret, -2, "gpio_set_level failed, returned: %d\n", ret);

	vTaskDelay(pdMS_TO_TICKS(200));

	ret = gps_reset();
	fail_if_negative(ret, -3, "gps_reset failed, returned: %d\n", ret);

	return 0;
}

bool gps_position_detected(void)
{
	return gps.location.isValid();
}

#define RESET_GPS_AFTER_INACTIVE_TIME_IN_MS  5000
#define RESET_GPS_MINIMUM_CHAR_PROCESSED       10
bool gps_need_reset(void)
{
	if ((_millis() - time_since_last_reset) > RESET_GPS_AFTER_INACTIVE_TIME_IN_MS
		&& gps.charsProcessed() < RESET_GPS_MINIMUM_CHAR_PROCESSED)
	{
		return true;
	}
	return false;
}

uint16_t gps_get_satellites(void)
{
	return (uint16_t)gps.satellites.value();
}

double gps_get_precision(void)
{
	return gps.hdop.hdop();
}

int gps_get_data(void)
{
	int ret = 0;
	bool waiting_first_data = true;
	uint8_t c;
	int len;

	// Read until the UART buffer is drained; block up to 1 ms per byte so the
	// task yields while waiting for the first character after wakeup.
	do
	{
		len = uart_read_bytes(UART_NUM_2, &c, 1, pdMS_TO_TICKS(1));
		if (len > 0)
		{
			waiting_first_data = false;
			gps.encode((char)c);
			ret = nmea_encode((char)c);
			fail_if_negative(ret, -1, "nmea_encode failed, returned: %d\n", ret);
		}
	} while (len > 0 || waiting_first_data);

#if DEBUG_DISPLAY_HOME_STATUS
	ret = _display_home_status();
	fail_if_negative(ret, -2, "_display_home_status failed, returned: %d\n", ret);
#endif

	// Set home once GPS quality meets the threshold
	if (!has_set_home
		&& gps.satellites.value() >= WANTED_SATELLITES
		&& gps.hdop.hdop() <= WANTED_PRECISION)
	{
		ret = beacon_set_home(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
		fail_if_negative(ret, -3, "beacon_set_home failed, returned: %d\n", ret);
		has_set_home = true;
	}

	if (has_set_home)
	{
		ret = beacon_update_data(
				gps.location.lat(),
				gps.location.lng(),
				gps.altitude.meters(),
				gps.course.deg(),
				gps.speed.mps());
		fail_if_negative(ret, -4, "beacon_update_data failed, returned: %d\n", ret);

#if DEBUG_DISPLAY_GPS_DATA
		ret = _display_gps_data();
		fail_if_negative(ret, -5, "_display_gps_data failed, returned: %d\n", ret);
#endif
	}

	return 0;
}
