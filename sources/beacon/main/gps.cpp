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

#define GPS_INIT_TIME_MS             1500
#define GPS_RESET_TRIGGER_DELAY_MS    100
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

static void _uart2_begin(int baud_rate)
{
	uart_config_t config;
	config.baud_rate           = baud_rate;
	config.data_bits           = UART_DATA_8_BITS;
	config.parity              = UART_PARITY_DISABLE;
	config.stop_bits           = UART_STOP_BITS_1;
	config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
	config.rx_flow_ctrl_thresh = 0;
	config.source_clk          = UART_SCLK_DEFAULT;

	if (!uart_is_driver_installed(UART_NUM_2)) {
		uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);
	}
	uart_param_config(UART_NUM_2, &config);
	uart_set_pin(UART_NUM_2, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void _uart2_end(void)
{
	uart_driver_delete(UART_NUM_2);
}

static void _uart2_println(const char *str)
{
	uart_write_bytes(UART_NUM_2, str, strlen(str));
	uart_write_bytes(UART_NUM_2, "\r\n", 2);
}

static void _print_gps_firmware_version(void)
{
	char line[128] = {0};
	int line_len = 0;

	// Flush stale data
	uint8_t dummy;
	while (uart_read_bytes(UART_NUM_2, &dummy, 1, 0) > 0);

	_uart2_println("$PMTK605*31");

	uint64_t deadline = (uint64_t)esp_timer_get_time() + 200000ULL; // 200 ms
	while ((uint64_t)esp_timer_get_time() < deadline)
	{
		uint8_t c;
		if (uart_read_bytes(UART_NUM_2, &c, 1, pdMS_TO_TICKS(1)) > 0)
		{
			if (c == '\n') {
				printf("Quectel L96-M33 fw version: %s\n", line);
				return;
			} else if (c != '\r' && line_len < (int)(sizeof(line) - 1)) {
				line[line_len++] = c;
			}
		}
	}

	printf("no firmware response\n");
}

#if DEBUG_DISPLAY_HOME_STATUS
static void _display_home_status(void)
{
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
}
#endif

#if DEBUG_DISPLAY_GPS_DATA
static void _display_gps_data(void)
{
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
}
#endif

void gps_configure(void)
{
	// Change GPS baudrate to 115200
	_uart2_println("$PMTK251,115200*1F");
	vTaskDelay(pdMS_TO_TICKS(GPS_BAUDRATE_CHANGE_DELAY_MS));
	_uart2_end();

	// Restart at new baudrate
	_uart2_begin(GPS_BAUDRATE_115200);

	_print_gps_firmware_version();

	printf("Configure GPS module: ");
	_uart2_println("$PMTK255,1*2D");                                           // Enable PPS
	_uart2_println("$PMTK285,4,100*38");                                       // PPS always on, 100ms pulse
	_uart2_println("$PMTK886,0*28");                                           // Normal navigation mode
	_uart2_println("$PMTK869,1,0*34");                                         // Disable EASY
	_uart2_println("$PMTK838,1*2C");                                           // Enable jamming detection
	_uart2_println("$PMTK514,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*2F");    // GGA + RMC + GSV only
	_uart2_println("$PMTK353,1,1,1,0,0*2A");                                  // GPS + GLONASS + Galileo
	_uart2_println("$PMTK352,0*2A");                                           // Disable QZSS
	_uart2_println("$PMTK314,0,1,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0*29");    // RMC+GGA+GSA+GSV enabled
	_uart2_println("$PMTK286,1*23");                                           // Enable AIC
	printf("Done\n");
}

static void _trigger_reset_pin(void)
{
	printf("Trigger GPS module reset\n");
	gpio_set_level(GPS_RESET_PIN, 0);
	vTaskDelay(pdMS_TO_TICKS(GPS_RESET_TRIGGER_DELAY_MS));
	gpio_set_level(GPS_RESET_PIN, 1);
}

static void _init_gps(void)
{
	_uart2_begin(GPS_BAUDRATE_DEFAULT);
	vTaskDelay(pdMS_TO_TICKS(GPS_INIT_TIME_MS));
	gps_configure();
	time_since_last_reset = _millis();
}

void gps_reset(void)
{
	_trigger_reset_pin();
	_uart2_end();
	_init_gps();
}

void gps_init(void)
{
	gpio_set_direction(GPS_RESET_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(GPS_RESET_PIN, 1);
	_init_gps();
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

void gps_get_data(void)
{
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
			nmea_encode((char)c);
		}
	} while (len > 0 || waiting_first_data);

#if DEBUG_DISPLAY_HOME_STATUS
	_display_home_status();
#endif

	// Set home once GPS quality meets the threshold
	if (!has_set_home
		&& gps.satellites.value() >= WANTED_SATELLITES
		&& gps.hdop.hdop() <= WANTED_PRECISION)
	{
		beacon_set_home(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
		has_set_home = true;
	}

	if (has_set_home)
	{
		beacon_update_data(
				gps.location.lat(),
				gps.location.lng(),
				gps.altitude.meters(),
				gps.course.deg(),
				gps.speed.mps());

#if DEBUG_DISPLAY_GPS_DATA
		_display_gps_data();
#endif
	}
}
