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
#define GPS_BAUDRATE 115200
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define BEACON_ID_FULL_LENGTH 31

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
            Serial.println("Error: wrong led state");
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
			Serial.printf("Error wrong model mass category: %d\n", mass);
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
		Serial.println("Error while getting the MAC address");
		return -1;
	}

	Serial.printf("Mac address: %s\n", str);

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
	Serial.printf("AlfaTango balise ID: %s  %s  %01d%s%s\n", data->builder_id, data->version_id, data->group, data->mass_str, data->mac);
	Serial.printf("Emitted balise ID  : %s\n", str);
}

static void _print_beacon_data(beacon_data *data)
{
	Serial.printf("Beacon data\n");
	Serial.printf(" - builder id: %s\n", data->builder_id);
	Serial.printf(" - version id: %s\n", data->version_id);
	Serial.printf(" - mac: %s\n", data->mac);
	Serial.printf(" - model group: %d\n", data->group);
	Serial.printf(" - model mass: %d (str: %s)\n", data->mass, data->mass_str);
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
	Serial2.println("$PMTK103*30"); // Cold start the GPS
	Serial2.println("$PMTK251,115200*1F"); // Set baudrate to 115200bauds
	Serial2.end();

	// Restart communication with gps using the new baudrate
    Serial2.begin(GPS_BAUDRATE_DEFAULT, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

	// Send the rest of the gps configuration
	Serial2.println("$PMTK255,1*2D"); // Enable PPS
	Serial2.println("$PMTK886,0*28"); // Normal navigation mode
	Serial2.println("$PMTK869,1,0*34"); // Disable EASY message
	Serial2.println("$PMTK838,0*2D"); // Disable jamming detection
	Serial2.println("$PMTK514,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E"); // Configure message output, keep only GGA and RMC, disable VTG, GSA, GSV and GLL.
	Serial2.println("$PMTK353,1,1,1,0,0*2A"); // Search for GPS + Glonass + Galileo satellites.
	Serial2.println("$PMTK352,0*2A"); // Stop QZSS regional positioning service.
	Serial2.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Send sentence for RMC and GGA each fix.
	Serial2.println("$PMTK286,0*22"); // Disable AIC function.
	Serial2.println("$PMTK285,4,100*38"); // Set pps pulse width to always, and 100ms.

#if 0
	// Debug
	String data = "";
	int8_t byteRead = 0;
	while(1)
	{
		byteRead = Serial2.read();
		if(byteRead == -1)
		{
			if(data != "")
			{
				Serial.println(data);
			}
			data = "";
			delay(250);
		}
		else
		{
			data.concat((char)byteRead);
		}
	}
#endif

	// Init the wifi, create an access point that do nothing.
    Serial.println("Starting AP");
    WiFi.softAP(ssid, nullptr, wifi_channel);
    IPAddress myIP = WiFi.softAPIP();

	// Print wifi data
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

	// TODO usefull 
    delay(1000);
    
    //check if Tansmit power is at his max (20 dBm -> 100mW)
	int8_t P1 = 0;
    esp_wifi_get_max_tx_power(&P1);
    Serial.printf("Tx power Value (dBm)=%f", (P1*0.25));
    if(P1>77) {
		_blink_led(10, 20);
    }
}

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
	uint64_t gpsMap = 0;

	bool has_set_home = false;
	double home_alt = 0.0;
	char buff[5][256];
	uint64_t gpsSec = 0;
	uint64_t beaconSec = 0;
	bool stat_led = false;

	while(1)
	{
		// Read gps data and feed the TinyGPS++ library
		while (Serial2.available())
		{
			gps.encode(Serial2.read());
		}

		// Case where the gps as an issue and doesn't work properly.
		if (millis() > 5000 && gps.charsProcessed() < 10) {
			// Print info log
			snprintf(buff[0], sizeof(buff[0]), "No GPS detected");
			Serial.println(buff[0]);

			// Turn off the led when the beacon is not working.
			_set_led_state(LED_OFF);

			// Wait some time then retry.
			delay(500);

			// Jump to the next interation of the main loop.
			continue;
		}

		// Manage the invalid GPS position case.
		if (!gps.location.isValid()) {
			if (millis() - gpsMap > 1000) {
				// Blink the led once
				_set_led_state(LED_ON);

				// Print info logs
				// Print the total number of characters received by the object.
				// Print the number of $GPRMC or $GPGGA sentences that had a fix.
				// Print the number of sentences of all types that failed the checksum test
				// Print the number of sentences of all types that passed the checksum test
				snprintf(buff[0], sizeof(buff[0]), "Positioning(%llu)", gpsSec++);
    			snprintf(buff[1], sizeof(buff[1]), "Char processed: %ld", gps.charsProcessed());
    			snprintf(buff[2], sizeof(buff[2]), "Sentence with fix: %ld", gps.sentencesWithFix());
    			snprintf(buff[3], sizeof(buff[3]), "Failed checksum: %ld", gps.failedChecksum());
    			snprintf(buff[4], sizeof(buff[4]), "Passed checksum: %ld", gps.passedChecksum());

				Serial.println(buff[0]);
				Serial.println(buff[1]);
				Serial.println(buff[2]);
				Serial.println(buff[3]);
				Serial.println(buff[4]);
				Serial.println("");

				// Keep track of the elapsed time.
				gpsMap = millis();

				// Add a little delay before turning off the led.
				delay(10);
				_set_led_state(LED_OFF);
			}

			// Jump to the next interation of the main loop.
			continue;
		}

		// GPS is valid, set the home position when the precision is high enough
		if (!has_set_home && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
			Serial.println("Setting Home Position");
			has_set_home = true;
			home_alt = gps.altitude.meters();
			Serial.println("Altitude de départ="+String(home_alt));
			drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());

 			// All is ok set the led on.
			digitalWrite(LED_PIN, HIGH);
		}

		// Send the gps data to the drone_idfr lib to format them.
		drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
		drone_idfr.set_heading(gps.course.deg());
		drone_idfr.set_ground_speed(gps.speed.mps());
		drone_idfr.set_heigth(gps.altitude.meters() - home_alt);

#if 1
		// Display gps data in the serial console for debug purpose
		if (millis() - gpsMap > 1000) {

			snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
			snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
			snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
			snprintf(buff[3], sizeof(buff[3]), "satellites:%lu", gps.satellites.value());

			Serial.println(buff[0]);
			Serial.println(buff[1]);
			Serial.println(buff[2]);
			Serial.println(buff[3]);

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
			Serial.println("Send beacon");
			// toggle the LED to see beacon sended
			if (stat_led) {
				_set_led_state(LED_OFF);
				stat_led = false;
			}
			else {
				_set_led_state(LED_ON);
				stat_led = true;
			}

			// Compute elapsed time and save new actual reference time.
			float time_elapsed = (float(millis() - beaconSec) / 1000); 
			beaconSec = millis();

#if 1
			// Print the beacon data that we use for the frame
			Serial.print(time_elapsed,1);
			Serial.print("s Send beacon: ");
			Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
			Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent());
			Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
#endif

			// write new SSID into beacon frame
			const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
			beaconPacket[40] = ssid_size;  // set size
			memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
			const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker

			// Generate the identification frame to send over wifi.
			const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
																								  // Décommenter ce block pour voir la trame entière sur le port usb

#if 0
			// Debug log to show the id frame in the serial console
			Serial.println("beaconPacket : ");
			for (auto i=0; i<sizeof(beaconPacket);i++) {
				Serial.print(beaconPacket[i], HEX);
				Serial.print(" ");
			}
			Serial.println(" ");*/
#endif

			// Send the wifi identification frame
			ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));

			// After sending we reset the send condition.
			drone_idfr.set_last_send();
		}
	}
}
