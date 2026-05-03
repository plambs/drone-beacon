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
#include <TinyGPS++.h>
#include <WiFi.h>

#include "config.h"
#include "droneID_FR.h"

extern "C" {
#include "esp_wifi.h"
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

#define DIP_SWITCH_1_PIN 13
#define DIP_SWITCH_2_PIN 14
#define DIP_SWITCH_3_PIN 27
#define DIP_SWITCH_4_PIN 26
#define LED_PIN 2

#define GPS_BAUDRATE_DEFAULT 9600
#define GPS_BAUDRATE_115200 115200
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define BEACON_ID_FULL_LENGTH 31

#define WANTED_SATELLITES 4 // Value must be equal or higher to be valid
#define WANTED_PRECISION 2.0 // Value must be lower than that to be valid
#define LOG_PERIOD_MS 1000 // print log every second
#define DELAY_BEFORE_RELOOPING_MS 100 // ms

// Hardware debug define
#define DEBUG_DISPLAY_NMEA_SENTENCE 0
#define DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS 0
#define DEBUG_DISPLAY_GPS_DATA 0
#define DEBUG_DISPLAY_HOME_STATUS 0
#define DEBUG_DISPLAY_BEACON_PACKET 0

TinyGPSPlus gps;
droneIDFR drone_idfr;

/* Change the value in config.h */
const char ssid[] = BEACON_SSID;
const char password[] = BEACON_PASSWORD;
char beacon_id[BEACON_ID_FULL_LENGTH] = "BEACON_ID_IS_NOT_SET_000000000";

/*
 * DO NOT CHANGE THE WIFI CONFIGURATION
 */
// Wifi use the channel 6 as asked by the specification.
static constexpr uint8_t wifi_channel = 6;
// Ensure the ssid is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Ensure the beacon_id is max 30 letters
static_assert((sizeof(beacon_id)/sizeof(*beacon_id))<=BEACON_ID_FULL_LENGTH, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination
// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
        0x80, 0x00,							            // 0-1: Frame Control
        0x00, 0x00,							            // 2-3: Duration
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 10-15: Source address FAKE  // TODO should bet set manually
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 16-21: Source address FAKE
        0x00, 0x00,							            // 22-23: Sequence / fragment number (done by the SDK)
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
        0xB8, 0x0B,							            // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
        0x21, 0x04,							            // 34-35: Capability info
        0x03, 0x01, 0x06,						        // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
        0x00, 0x20,                     				// 39-40: SSID parameter set, 0x20:maxlength:content
                                                        // 41-XX: SSID (max 32)
};

typedef enum {
	LED_OFF = 0,
	LED_ON = 1,
} led_state;

/*
Pour les types de modèles, les groupes sont les suivants :
 - Groupe 1 : aérostat captif / aéromodèle de vol circulaire / aéromodèle de vol libre / montgolfière
 - Groupe 2 : planeur, aile (non motorisé) / dirigeable / parachute, parapente / aéronef à ailes battantes
 - Groupe 3 : hélicoptère / multirotors / convertible / combiné / paramoteur / autogire
 - Groupe 4 : avion, aile, planeur (motorisé)
*/
typedef enum {
	MODEL_GROUP1 = 0,
	MODEL_GROUP2 = 1,
	MODEL_GROUP3 = 2,
	MODEL_GROUP4 = 3,
} model_group;

/*
Pour les plages de masse, les groupes sont les suivants :
 - Entre 800 g et 2 kg (model_mass = 0)
 - Entre 2 kg et 4 kg (model_mass = 1)
 - Entre 4 kg et 25 kg (model_mass = 2)
 - Entre 25 kg et 150 kg (model_mass = 3)
 - Plus de 150 kg (not supported)
*/
typedef enum {
	MODEL_MASS_800GR_2KG = 0,
	MODEL_MASS_2KG_4KG = 1,
	MODEL_MASS_4KG_25KG = 2,
	MODEL_MASS_25KG_150KG = 3,
} model_mass;

typedef struct {
	char builder_id[4];
	char version_id[4];
	char mac[13];
	model_group group;
	model_mass mass;
	char mass_str[4];
} beacon_data;

static void _set_led_state(led_state state)
{
	switch(state)
	{
		case LED_OFF:
            digitalWrite(LED_PIN, HIGH);
			break;
		case LED_ON:
            digitalWrite(LED_PIN, LOW);
			break;
		default:
            printf("Error: wrong led state");
			break;
	}
}

static void _blink_led(uint8_t nb_blink, uint16_t delay_ms)
{
	_set_led_state(LED_OFF);
	delay(delay_ms);
	for(int i = 0; i < nb_blink; i++)
	{
		if((i % 2) == 0)
		{
			_set_led_state(LED_ON);
		} else {
			_set_led_state(LED_OFF);
		}
		delay(delay_ms);
	}
}

static model_group _get_model_group(void)
{
	uint8_t group = 0;
    group = group | (uint8_t)(digitalRead(DIP_SWITCH_1_PIN));
    group = group | (uint8_t)(digitalRead(DIP_SWITCH_2_PIN) << 1);

	return (model_group)group;
}

static model_mass _get_model_mass(void)
{
	uint8_t mass = 0;
    mass = mass | (uint8_t)(digitalRead(DIP_SWITCH_3_PIN));
    mass = mass | (uint8_t)(digitalRead(DIP_SWITCH_4_PIN) << 1);

	return (model_mass)mass;
}

static int _get_mass_str(char *str, int max_size)
{
	model_mass mass = _get_model_mass();
	uint16_t mass_value = 999;
	switch(mass)
	{
		case MODEL_MASS_800GR_2KG:
			mass_value = 2;
			break;
		case MODEL_MASS_2KG_4KG:
			mass_value = 4;
			break;
		case MODEL_MASS_4KG_25KG:
			mass_value = 25;
			break;
		case MODEL_MASS_25KG_150KG:
			mass_value = 150;
			break;
		default:
			printf("Error wrong model mass category: %d\n", mass);
			break;
	}

	// Format the mass str
	snprintf(str, max_size, "%03d", mass_value);

	return 0;
}

static int _get_mac_str(char *str, int max_size)
{
	uint8_t mac[6];
	esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
	if (ret == ESP_OK) {

		snprintf(str, max_size, "%02x%02x%02x%02x%02x%02x",
				mac[0], mac[1], mac[2],
				mac[3], mac[4], mac[5]);
	} else {
		printf("Error while getting the MAC address");
		return -1;
	}

	printf("Mac address: %s\n", str);

	return 0;
}

static void _get_beacon_data(beacon_data *data)
{
	snprintf(data->builder_id, 4, "%s", BEACON_BUILDER_ID);
	snprintf(data->version_id, 4, "%s", BEACON_MODEL_ID);
	_get_mac_str(data->mac, 13);
	data->group = _get_model_group();
	data->mass = _get_model_mass();
	_get_mass_str(data->mass_str, 4);
}

static void _format_beacon_id(char *str, uint8_t max_size, beacon_data *data)
{
	// Compute the beacon full id using MAC address
	snprintf(str, max_size, "%s%s%07d%01d%s%s", data->builder_id, data->version_id, 0, data->group, data->mass_str, data->mac);

	// Display the beacon full id
	printf("AlfaTango balise ID: %s  %s  %01d%s%s\n", data->builder_id, data->version_id, data->group, data->mass_str, data->mac);
	printf("Emitted balise ID  : %s\n", str);
}

static void _print_beacon_data(beacon_data *data)
{
	printf("Beacon data\n");
	printf(" - builder id: %s\n", data->builder_id);
	printf(" - version id: %s\n", data->version_id);
	printf(" - mac: %s\n", data->mac);
	printf(" - model group: %d\n", data->group);
	printf(" - model mass: %d (str: %s)\n", data->mass, data->mass_str);
}

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
				printf("Quectel L96-M33 fw version: %s",line.c_str());
				return;
			}
			else if (c != '\r')
			{
				line += c;
			}
		}
	}

	printf("no firmware response");
}


/**
 * Phase de configuration.
 */
void setup()
{
    Serial.begin(115200);

	// Set all gpio
    pinMode(DIP_SWITCH_1_PIN, INPUT_PULLDOWN);
    pinMode(DIP_SWITCH_2_PIN, INPUT_PULLDOWN);
    pinMode(DIP_SWITCH_3_PIN, INPUT_PULLDOWN);
    pinMode(DIP_SWITCH_4_PIN, INPUT_PULLDOWN);
    pinMode(LED_PIN, OUTPUT);

	// Let gps time to init
	delay(1000);

    // Init Quectel L96 gps module.
	// Start communication and change baudrate
    Serial2.begin(GPS_BAUDRATE_DEFAULT, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
	Serial2.println("$PMTK251,115200*1F"); // Set baudrate to 115200bauds
	delay(300); // Let time to the gps module to change its baudrate.
	Serial2.end();

	// Restart communication with gps using the new baudrate
    Serial2.begin(GPS_BAUDRATE_115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

	_print_gps_firmware_version();

	// Send the rest of the gps configuration
    printf("Configure GPS module: ");
	Serial2.println("$PMTK255,1*2D"); // Enable PPS
	Serial2.println("$PMTK886,0*28"); // Normal navigation mode
	Serial2.println("$PMTK869,1,0*34"); // Disable EASY message
	Serial2.println("$PMTK838,1*2C"); // Enable jamming detection
	Serial2.println("$PMTK514,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0*2E"); // Configure message output, keep only GGA and RMC, disable VTG, GSA, GSV and GLL.
	Serial2.println("$PMTK353,1,1,1,0,0*2A"); // Search for GPS + Glonass + Galileo satellites.
	Serial2.println("$PMTK352,0*2A"); // Stop QZSS regional positioning service.
	Serial2.println("$PMTK314,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0*28");
	Serial2.println("$PMTK286,1*23"); // Enable AIC function.
	Serial2.println("$PMTK285,4,100*38"); // Set pps pulse width to always, and 100ms.
    printf("Done\n");

	// Init the wifi, create an access point that do nothing.
    printf("Starting AP");
    WiFi.softAP(ssid, nullptr, wifi_channel);
    IPAddress myIP = WiFi.softAPIP();

	// Print wifi data
	// TODO replace by printf
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.print("AP mac address: ");
    Serial.println(WiFi.macAddress());
    wifi_config_t conf_current;
    esp_wifi_get_config(WIFI_IF_AP, &conf_current);

	// Format beacon ID according to specification for AlphaTango
	beacon_data data;
	_get_beacon_data(&data);
	_print_beacon_data(&data);
	_format_beacon_id(beacon_id, sizeof(beacon_id), &data);

    // Change WIFI AP default beacon interval sending to 1s.
    conf_current.ap.beacon_interval = 1000;
    drone_idfr.set_drone_id(beacon_id);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));

	// TODO usefull ?
    delay(1000);

    //check if Tansmit power is at his max (20 dBm -> 100mW)
	int8_t P1 = 0;
    esp_wifi_get_max_tx_power(&P1);
    printf("Tx power Value (dBm)=%f\n", (P1*0.25));
    if(P1>77) {
		_blink_led(10, 20);
    }
}

#if DEBUG_DISPLAY_NMEA_SENTENCE
#define DEBUG_DISPLAY_DETAILS_SAT_SNR 0
static void _parseGSV(String sentence) {
	int fieldIndex = 0;
	int satellitesInView = 0;
	uint16_t snr_worse = 99;
	uint16_t snr_best = 0;

	char *token;
	char buffer[120];
	sentence.toCharArray(buffer, sizeof(buffer));

	token = strtok(buffer, ",");

	while (token != NULL) {
		if (fieldIndex == 3) {
			satellitesInView = atoi(token);
#if DEBUG_DISPLAY_DETAILS_SAT_SNR
			Serial.print("");
			Serial.println(satellitesInView);
#endif
		}

		// SNR fields are every 4th after index 4
		if (fieldIndex >= 7 && ((fieldIndex - 7) % 4 == 0)) {
			int snr = atoi(token);
			if (snr > 0) {
#if DEBUG_DISPLAY_DETAILS_SAT_SNR
				Serial.print("SNR: ");
				Serial.println(snr);
#endif
				// Remember SNR value if it higher than the best recorded
				if((snr > snr_best) && (snr < 99))
				{
					snr_best = snr;
				}

				// Remember SNR value if it lower than the worst recorded
				if(snr < snr_worse)
				{
					snr_worse = snr;
				}
			}
		}

		token = strtok(NULL, ",");
		fieldIndex++;
	}

	// Print all data
	if(snr_best == 0 || snr_worse == 99)
	{
		printf("Satellites in view: %d\n", satellitesInView);
	}
	else
	{
		printf("Satellites in view: %d, SNR, best: %ddb, worst: %ddb\n", satellitesInView, snr_best, snr_worse);
	}
}

static void _parse_nmea_sentence(String nmea)
{
	// Print the sentence received
#if DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS
	Serial.println(nmea);
#endif

	// Parse GSV (satellites in view and SNR)
	if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GNGSV")) {
		_parseGSV(nmea);
	}

	// Detect jamming
	// Jamming detected == $PMTKSPF,3*58
	if (nmea.startsWith("$PMTKSPF,3*58")) {
		Serial.println("Interference detected!");
	}
}
#endif

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
	uint64_t gpsMap = 0;

	bool has_set_home = false;
	double home_alt = 0.0;
	bool stat_led = false;

#if DEBUG_DISPLAY_GPS_DATA
	uint64_t gpsSec = 0;
#endif

	while(1)
	{
		// Read gps data and feed the TinyGPS++ library, display the wanted sentence for debug
		while (Serial2.available())
		{
			char c = Serial2.read();
			gps.encode(c);

#if DEBUG_DISPLAY_NMEA_SENTENCE
			// Debug path to see all the sentences and manually decode some of them outside TinyGPS++
			static String nmea = "";
			if (c == '\n') {
				_parse_nmea_sentence(nmea);
				nmea = "";
			} else {
				nmea += c;
			}
#endif
		}

		// Case where the gps as an issue and doesn't work properly.
		if (millis() > 5000 && gps.charsProcessed() < 10) {
			// Print info log
			printf("No GPS detected\n");

			// Turn off the led when the beacon is not working.
			_set_led_state(LED_OFF);

			// Wait some time then retry.
			delay(DELAY_BEFORE_RELOOPING_MS);

			// TODO add reset of the gps module

			// Jump to the next interation of the main loop.
			continue;
		}

		// Manage the invalid GPS position case.
		if (!gps.location.isValid()) {
			if (millis() - gpsMap > LOG_PERIOD_MS) {
				// Blink the led once
				_set_led_state(LED_ON);

#if DEBUG_DISPLAY_GPS_DATA
				// Print info logs
				// Print the total number of characters received by the object.
				// Print the number of $GPRMC or $GPGGA sentences that had a fix.
				// Print the number of sentences of all types that failed the checksum test
				// Print the number of sentences of all types that passed the checksum test
				printf("\nPositioning (%llu)\n", gpsSec++);
				printf("Char processed: %ld\n", gps.charsProcessed());
				printf("Sentence with fix: %ld\n", gps.sentencesWithFix());
				printf("Failed checksum: %ld\n", gps.failedChecksum());
				printf("Passed checksum: %ld\n", gps.passedChecksum());
				printf("satellites: %lu\n\n", gps.satellites.value());
#endif

				// Keep track of the elapsed time.
				gpsMap = millis();

				// Add a little delay before turning off the led.
				delay(10);
				_set_led_state(LED_OFF);
			}

			// Jump to the next interation of the main loop but wait 100ms before doing so to let the system doing something else.
			delay(DELAY_BEFORE_RELOOPING_MS);
			continue;
		}

#if DEBUG_DISPLAY_HOME_STATUS
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
#endif

		// GPS is valid, set the home position when the precision is high enough
		if (!has_set_home && gps.satellites.value() >= WANTED_SATELLITES && gps.hdop.hdop() <= WANTED_PRECISION) {
			printf("Setting Home Position");
			has_set_home = true;
			home_alt = gps.altitude.meters();
			printf("Altitude de départ=%s\n", String(home_alt).c_str());
			drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());

			_set_led_state(LED_ON);
		}

		// Send the gps data to the drone_idfr lib to format them.
		drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
		drone_idfr.set_heading(gps.course.deg());
		drone_idfr.set_ground_speed(gps.speed.mps());
		drone_idfr.set_heigth(gps.altitude.meters() - home_alt);

#if DEBUG_DISPLAY_GPS_DATA
		// Display gps data in the serial console for debug purpose
		if (millis() - gpsMap > LOG_PERIOD_MS) {

			printf("\nPositioning (%llu)\n", gpsSec++);
			printf("satellites with fix:%lu\n", gps.satellites.value());
			printf("UTC:%d:%d:%d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
			printf("LNG:%.4f - LAT:%.4f\n", gps.location.lng(), gps.location.lat());

			gpsMap = millis();
		}
#endif

		/**
		 * On regarde s'il temps d'envoyer la trame d'identification drone:
		 *  - soit toutes les 3s,
		 *  - soit si le drone s'est déplacé de 30m en moins de 3s soit 10m/s ou 36km/h,
		 *  - uniquement si la position Home est déjà définie,
		 *  - et dans le cas où les données GPS sont nouvelles.
		 */
		if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
			// Compute elapsed time and save new actual reference time.
			static uint64_t beaconSec = 0;
			float time_elapsed = (float(millis() - beaconSec) / 1000);
			beaconSec = millis();

			// Print the beacon data that we use for the frame
			printf("Send beacon -> last beacon: %fs, send reason: %s, distance travel: %f m, speed: %f km/h\n",
				time_elapsed,
				drone_idfr.has_pass_distance() ? "distance" : "time",
				drone_idfr.get_distance_from_last_position_sent(),
				drone_idfr.get_ground_speed_kmh()
			);

			// toggle the LED to see beacon sended
			if (stat_led) {
				_set_led_state(LED_OFF);
				stat_led = false;
			}
			else {
				_set_led_state(LED_ON);
				stat_led = true;
			}

			// write new SSID into beacon frame
			const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
			beaconPacket[40] = ssid_size;  // set size
			memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
			const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker

			// Generate the identification frame to send over wifi.
			const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination

#if DEBUG_DISPLAY_BEACON_PACKET
			// Debug log to show the id frame in the serial console
			printf("beaconPacket : ");
			for (auto i=0; i<sizeof(beaconPacket);i++) {
				printf("0x%X ", beaconPacket[i]);
			}
			printf("\n");
#endif

			// Send the wifi identification frame
			ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));

			// After sending we reset the send condition.
			drone_idfr.set_last_send();
		}

		// Wait 100ms before going to the next iteration of the loop
		delay(DELAY_BEFORE_RELOOPING_MS);
	}
}
