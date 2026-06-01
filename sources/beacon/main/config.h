#ifndef _CONFIG_HEADER_
#define _CONFIG_HEADER_

#define SERIAL_DEFAUT_BAUDRATE 115200

/*
 * Name of the acces point
 */
#define BEACON_SSID "DroneBeacon"

/*
 * Access point password
 */
#define BEACON_PASSWORD "123456789"

/*
 * Beacon ID, change it by the one received on Alphatango.
 *
 * ID format is like that: 000 AM1 00000000 G MMM XXXXXXXXXXXX
 * 000 is the beacon builder id, for homemade one we must use 000
 * AM1 is the beacon model id (put whatever you want)
 * 00000000 don't change it.
 * XXXXXXXXXXXX will be replaced with the BEACON_ID set in config.h or the MAC address of the ESP32.
 * G will be replaced with the model group
 * MMM will be replaced by the model mass
 * - 002 (group 0)
 * - 004 (group 1)
 * - 025 (group 2)
 * - 150 (group 3)
 */
#define BEACON_BUILDER_ID "000" // Homemade always use 000
#define BEACON_MODEL_ID "DB1" // Put whatever you want, lengh max 3 chars
#define BEACON_ID "" // If empty the system use the MAC addess, if you put a custom one, lengh max 11 chars
#define BEACON_ID_FULL_LENGTH 31

#define WANTED_SATELLITES 5 // Value must be equal or higher to be valid
#define WANTED_PRECISION 3.0 // Value must be lower than that to be valid
#define LOG_PERIOD_MS 1000 // print log every second
#define DELAY_BEFORE_RELOOPING_MS 10 // ms
#define SLEEP_MODE_ENABLED 1
#define DELAY_BEFORE_RELOOPING_NO_SLEEP_MS 500 //ms

// Hardware debug define
#define DEBUG_DISPLAY_SATELLITE_AND_SNR_DETAILS 0
#define DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS 0
#define DEBUG_DISPLAY_GPS_DATA 0
#define DEBUG_DISPLAY_HOME_STATUS 0
#define DEBUG_DISPLAY_BEACON_PACKET 0
#define DEBUG_DISPLAY_BEACON_DATA 1
#define DEBUG_DISPLAY_DETAILS_SAT_SNR 0
#define DEBUG_DISPLAY_WAKE_UP_SOURCE_AND_DURATION 0
#define DEBUG_DISPLAY_SLEEP_LOGS 0

#endif //_CONFIG_HEADER_
