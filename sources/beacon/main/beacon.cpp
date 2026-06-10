#include "beacon.h"
#include "config.h"
#include "led.h"

// droneID_FR.h was written for Arduino and uses these without including them.
// Provide the missing standard headers and macros before including the library.
#include <cmath>
#include <cstring>
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef radians
#define radians(deg) ((deg) * DEG_TO_RAD)
#endif
#ifndef sq
#define sq(x) ((x) * (x))
#endif

#include "droneID_FR.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define DIP_SWITCH_1_PIN 13
#define DIP_SWITCH_2_PIN 14
#define DIP_SWITCH_3_PIN 27
#define DIP_SWITCH_4_PIN 26

#define WIFI_INIT_DELAY_MS 250

extern "C" {
#include "esp_wifi.h"
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

/* Change the value in config.h */
static const char ssid[] = BEACON_SSID;

// Ensure the ssid is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");

/*
 * DO NOT CHANGE THE WIFI CONFIGURATION
 */
// Wifi use the channel 6 as asked by the specification.
static constexpr uint8_t wifi_channel = 6;

/* Change the value in config.h */
static const char password[] = BEACON_PASSWORD;
static char beacon_id[BEACON_ID_FULL_LENGTH] = "BEACON_ID_IS_NOT_SET_000000000";
static beacon_data data;
static droneIDFR drone_idfr;
static bool home_is_set = false;
static double home_alt = 0;

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

static model_group _get_model_group(void)
{
	uint8_t group = 0;
	group |= (uint8_t)gpio_get_level((gpio_num_t)DIP_SWITCH_1_PIN);
	group |= (uint8_t)(gpio_get_level((gpio_num_t)DIP_SWITCH_2_PIN) << 1);
	return (model_group)group;
}

static model_mass _get_model_mass(void)
{
	uint8_t mass = 0;
	mass |= (uint8_t)gpio_get_level((gpio_num_t)DIP_SWITCH_3_PIN);
	mass |= (uint8_t)(gpio_get_level((gpio_num_t)DIP_SWITCH_4_PIN) << 1);
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
	esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
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

#if DEBUG_DISPLAY_BEACON_DATA
static void _display_beacon_data()
{
	// Compute elapsed time and save new actual reference time.
	static uint64_t beaconSec = 0;
	uint64_t now = (uint64_t)(esp_timer_get_time() / 1000LL);
	float time_elapsed = (float)(now - beaconSec) / 1000.0f;
	beaconSec = now;

	// Print the beacon data that we use for the frame
	printf("Send beacon -> last beacon: %fs, send reason: %s, distance travel: %f m, speed: %f km/h\n",
			time_elapsed,
			drone_idfr.has_pass_distance() ? "distance" : "time",
			drone_idfr.get_distance_from_last_position_sent(),
			drone_idfr.get_ground_speed_kmh()
		  );
}
#endif

void beacon_set_home(double lat, double lng, double alt)
{
	printf("Setting home position, start altitude: %fm\n", alt);
	home_is_set = true;
	home_alt = alt;
	drone_idfr.set_home_position(lat, lng, home_alt);

}

bool beacon_is_home_set()
{
	return home_is_set;
}

void beacon_update_data(double latitude, double longitude, double altitude, double course, double speed)
{
	// Send the gps data to the drone_idfr lib to format them.
	drone_idfr.set_current_position(latitude, longitude, altitude);
	drone_idfr.set_heading(course);
	drone_idfr.set_ground_speed(speed);
	drone_idfr.set_heigth(altitude - home_alt);
}

bool beacon_data_must_be_send()
{
	/**
	 * On regarde s'il temps d'envoyer la trame d'identification drone:
	 *  - soit toutes les 3s,
	 *  - soit si le drone s'est déplacé de 30m en moins de 3s soit 10m/s ou 36km/h,
	 *  - uniquement si la position Home est déjà définie,
	 *  - et dans le cas où les données GPS sont nouvelles.
	 */
	if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
		return true;
	}
	
	return false;
}

void beacon_send_data()
{
	// write new SSID into beacon frame
	const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
	beaconPacket[40] = ssid_size;  // set size
	memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
	const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker

	// Generate the identification frame to send over wifi.
	const uint8_t bytes_to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination

#if DEBUG_DISPLAY_BEACON_PACKET
	// Debug log to show the id frame in the serial console
	printf("bytes to send: %d\n", bytes_to_send);
	printf("beaconPacket : ");
	for (int i = 0; i < bytes_to_send; i++) {
		printf("0x%X ", beaconPacket[i]);
	}
	printf("\n");
#endif

#if  DEBUG_DISPLAY_BEACON_DATA
	_display_beacon_data();
#endif

	// Send the wifi identification frame
	ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, bytes_to_send, true));

	// After sending we reset the send condition.
	drone_idfr.set_last_send();
}

void beacon_init()
{
	// Set all DIP switch GPIOs as inputs with pull-down
	gpio_config_t dip_conf = {};
	dip_conf.pin_bit_mask = (1ULL << DIP_SWITCH_1_PIN) | (1ULL << DIP_SWITCH_2_PIN)
	                      | (1ULL << DIP_SWITCH_3_PIN) | (1ULL << DIP_SWITCH_4_PIN);
	dip_conf.mode         = GPIO_MODE_INPUT;
	dip_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
	dip_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	dip_conf.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&dip_conf);

	// Init the wifi, create an access point that do nothing.
    printf("Starting AP\n");

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    const size_t ssid_size = (sizeof(ssid) / sizeof(*ssid)) - 1;
    wifi_config_t wifi_config = {};
    memcpy(wifi_config.ap.ssid, ssid, ssid_size);
    wifi_config.ap.ssid_len = (uint8_t)ssid_size;
    wifi_config.ap.channel = wifi_channel;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.beacon_interval = 1000;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t ap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
    printf("AP mac address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);

    //check if Tansmit power is at his max (20 dBm -> 100mW)
	int8_t P1 = 0;
    esp_wifi_get_max_tx_power(&P1);
    printf("Tx power Value (dBm)=%f\n", (P1*0.25));
    if(P1>77) {
		led_blink(1);
    }

	// Format beacon ID according to specification for AlphaTango
	_get_beacon_data(&data);
	_print_beacon_data(&data);
	_format_beacon_id(beacon_id, sizeof(beacon_id), &data);

	// Set the beacon id to into the droneIFR lib
    drone_idfr.set_drone_id(beacon_id);
}
