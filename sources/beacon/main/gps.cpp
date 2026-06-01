#include <TinyGPS++.h>
#include "gps.h"
#include "config.h"
#include "nmea.h"
#include "beacon.h"

#define GPS_INIT_TIME_MS 1500
#define GPS_RESET_TRIGGER_DELAY_MS 100
#define GPS_BAUDRATE_CHANGE_DELAY_MS 300
#define GPS_BAUDRATE_DEFAULT 9600
#define GPS_BAUDRATE_115200 115200
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_RESET_PIN 18

static TinyGPSPlus gps;
static bool has_set_home = false;
static uint64_t time_since_last_reset = 0;

static void _print_gps_firmware_version()
{
	String line = "";
	unsigned long start_time = millis();

	// Flush old data
    while (Serial2.available())
    {
        Serial2.read();
    }

	Serial2.println("$PMTK605*31");

	while (millis() - start_time < 200)
	{
		while (Serial2.available())
		{
			char c = Serial2.read();

			if (c == '\n')
			{
				printf("Quectel L96-M33 fw version: %s\n",line.c_str());
				return;
			}
			else if (c != '\r')
			{
				line += c;
			}
		}
	}

	printf("no firmware response\n");
}

#if DEBUG_DISPLAY_HOME_STATUS
static void _display_home_status()
{
	static uint64_t home_time = 0;

	if (millis() - home_time > LOG_PERIOD_MS)
	{
		printf("Home is set: %s, satellites value: %ld (wanted: %d), hdop: %f (wanted: %f)\n",
				has_set_home ? "YES" : "NO",
				gps.satellites.value(),
				WANTED_SATELLITES,
				gps.hdop.hdop(),
				WANTED_PRECISION);

		home_time = millis();
	}
}
#endif

#if DEBUG_DISPLAY_GPS_DATA
static void _display_gps_data()
{
	static uint64_t gpsSec = 0;
	static uint64_t gpsMap = 0;

	// Display gps data in the serial console for debug purpose
	if (millis() - gpsMap > LOG_PERIOD_MS) {

		printf("\nPositioning (%llu)\n", gpsSec++);
		printf("satellites with fix:%lu\n", gps.satellites.value());
		printf("UTC:%d:%d:%d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
		printf("LNG:%.4f - LAT:%.4f\n", gps.location.lng(), gps.location.lat());

		gpsMap = millis();
	}
}
#endif


void gps_configure()
{
    // Init Quectel L96 gps module.
	// Start communication and change baudrate
	Serial2.println("$PMTK251,115200*1F"); // Set baudrate to 115200bauds
	delay(GPS_BAUDRATE_CHANGE_DELAY_MS); // Let time to the gps module to change its baudrate.
	Serial2.end();

	// Restart communication with gps using the new baudrate
    Serial2.begin(GPS_BAUDRATE_115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

	_print_gps_firmware_version();

	// Send the rest of the gps configuration
    printf("Configure GPS module: ");
	Serial2.println("$PMTK255,1*2D"); // Enable PPS
	Serial2.println("$PMTK285,4,100*38"); // Set pps pulse width to always, and 100ms.
	Serial2.println("$PMTK886,0*28"); // Normal navigation mode
	Serial2.println("$PMTK869,1,0*34"); // Disable EASY message
	Serial2.println("$PMTK838,1*2C"); // Enable jamming detection
	Serial2.println("$PMTK514,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*2F"); // Configure message output, keep only GGA and RMC and GSV, disable VTG, GSA and GLL.
	Serial2.println("$PMTK353,1,1,1,0,0*2A"); // Search for GPS + Glonass + Galileo satellites.
	Serial2.println("$PMTK352,0*2A"); // Stop QZSS regional positioning service.
	Serial2.println("$PMTK314,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0*28");
	Serial2.println("$PMTK286,1*23"); // Enable AIC function.

    printf("Done\n");
}

void _trigger_reset_pin(void)
{
	printf("Trigger GPS module reset\n");

	// Trigger reset by pulling down the reset pin
	digitalWrite(GPS_RESET_PIN, LOW);
	delay(GPS_RESET_TRIGGER_DELAY_MS);
	digitalWrite(GPS_RESET_PIN, HIGH);
}

void _init_gps(void)
{
	// Init the GPS serial to default baudrate
    Serial2.begin(GPS_BAUDRATE_DEFAULT, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

	// Let gps time to init
	delay(GPS_INIT_TIME_MS);

	// configure the gps
	gps_configure();

	// Remember the actual time to check if the reset is needed later
	time_since_last_reset = millis();
}

void gps_reset()
{
	_trigger_reset_pin();

	// Stop the gps serial connection before reinit
	Serial2.end();

	_init_gps();
}

void gps_init()
{
	// Fix the reset pin to HIGH
    pinMode(GPS_RESET_PIN, OUTPUT);
	digitalWrite(GPS_RESET_PIN, HIGH);

	_init_gps();
}

bool gps_position_detected()
{
	return gps.location.isValid();
}

#define RESET_GPS_AFTER_INACTIVE_TIME_IN_MS 5000
#define RESET_GPS_MINIMUM_CHAR_PROCESSED 10
bool gps_need_reset()
{
	// Case where the gps as an issue and doesn't work properly.
	if ((millis() - time_since_last_reset) > RESET_GPS_AFTER_INACTIVE_TIME_IN_MS
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

void gps_get_data()
{
	bool waiting_first_data = true;

	// Read gps data and feed the TinyGPS++ library, display the wanted sentence for debug
	do
	{
		// Data are available, read and encode them
		if(Serial2.available())
		{
			waiting_first_data = false;
			char c = Serial2.read();
			gps.encode(c);
			nmea_encode(c);
		}
		else
		{
			// Give time to the system to do something else
			delay(1);
		}
	} while (Serial2.available() || waiting_first_data);

#if DEBUG_DISPLAY_HOME_STATUS
	_display_home_status();
#endif

	// If we have a position and the precision is high enough we can set the home position of the beacon.
	if (!has_set_home && gps.satellites.value() >= WANTED_SATELLITES && gps.hdop.hdop() <= WANTED_PRECISION) {
		beacon_set_home(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
		has_set_home = true;
	}

	if(has_set_home)
	{
		// Update the beacon data with all new data received from the gps
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
