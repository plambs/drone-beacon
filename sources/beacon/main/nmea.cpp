#include <Arduino.h>
#include "nmea.h"
#include "config.h"

static uint16_t satellites_in_view = 0;

static void _parse_gsv(String sentence) {
	int fieldIndex = 0;
	uint16_t snr_worse = 99;
	uint16_t snr_best = 0;

	char *token;
	char buffer[120];
	sentence.toCharArray(buffer, sizeof(buffer));

	token = strtok(buffer, ",");

	while (token != NULL) {
		if (fieldIndex == 3) {
			satellites_in_view = atoi(token);
		}

		// SNR fields are every 4th after index 4
		if (fieldIndex >= 7 && ((fieldIndex - 7) % 4 == 0)) {
			int snr = atoi(token);
			if (snr > 0) {
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

#if DEBUG_DISPLAY_SATELLITE_AND_SNR_DETAILS
	// Print all data
	if(snr_best == 0 || snr_worse == 99)
	{
		printf("Satellites in view: %d\n", satellites_in_view);
	}
	else
	{
		printf("Satellites in view: %d, SNR, best: %ddb, worst: %ddb\n", satellites_in_view, snr_best, snr_worse);
	}
#endif
}

static void _parse_nmea_sentence(String nmea)
{
	// Print the sentence received
#if DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS
	Serial.println(nmea);
#endif

	// Parse GSV (satellites in view and SNR)
	if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GNGSV")) {
		_parse_gsv(nmea);
	}

	// Detect jamming
	// Jamming detected == $PMTKSPF,3*58
	if (nmea.startsWith("$PMTKSPF,3*58")) {
		printf("Interference detected!\n");
	}
}

uint16_t nmea_get_satellites_in_view(void)
{
	return satellites_in_view;
}

void nmea_encode(char c)
{
	// Collect sentences and manually decode some of them outside TinyGPS++
	static String nmea = "";
	if (c == '\n') {
		_parse_nmea_sentence(nmea);
		nmea = "";
	} else {
		nmea += c;
	}
}
