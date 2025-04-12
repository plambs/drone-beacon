#ifndef _CONFIG_HEADER_
#define _CONFIG_HEADER_

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

#endif //_CONFIG_HEADER_
