#include <Arduino.h>
#include "nmea.h"
#include "config.h"

static uint16_t _siv_gps = 0;
static uint16_t _siv_glo = 0;
static uint16_t _siv_gal = 0;
static uint16_t _siv_gn  = 0;

static void _parse_gsv(String sentence, uint16_t *siv_bucket) {
	int fieldIndex = 0;
	uint16_t snr_worse = 99;
	uint16_t snr_best = 0;

	char *token;
	char buffer[120];
	sentence.toCharArray(buffer, sizeof(buffer));

	token = strtok(buffer, ",");

	while (token != NULL) {
		if (fieldIndex == 3) {
			*siv_bucket = atoi(token);
		}

		// SNR fields are every 4th after index 7
		if (fieldIndex >= 7 && ((fieldIndex - 7) % 4 == 0)) {
			int snr = atoi(token);
			if (snr > 0) {
				if((snr > snr_best) && (snr < 99))
					snr_best = snr;
				if(snr < snr_worse)
					snr_worse = snr;
			}
		}

		token = strtok(NULL, ",");
		fieldIndex++;
	}

#if DEBUG_DISPLAY_SATELLITE_AND_SNR_DETAILS
	if(snr_best == 0 || snr_worse == 99)
		printf("Satellites in view: %d\n", *siv_bucket);
	else
		printf("Satellites in view: %d, SNR, best: %ddb, worst: %ddb\n", *siv_bucket, snr_best, snr_worse);
#endif
}

static void _parse_nmea_sentence(String nmea)
{
#if DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS
	Serial.println(nmea);
#endif

	// Parse GSV per constellation — each has its own counter to avoid overwriting
	if      (nmea.startsWith("$GPGSV")) _parse_gsv(nmea, &_siv_gps);
	else if (nmea.startsWith("$GLGSV")) _parse_gsv(nmea, &_siv_glo);
	else if (nmea.startsWith("$GAGSV")) _parse_gsv(nmea, &_siv_gal);
	else if (nmea.startsWith("$GNGSV")) _parse_gsv(nmea, &_siv_gn);

	if (nmea.startsWith("$PMTKSPF,3*58")) {
		printf("Interference detected!\n");
	}
}

uint16_t nmea_get_satellites_in_view(void)
{
	// Use combined GNSS sentence if available, otherwise sum per-constellation counts
	if (_siv_gn > 0)
		return _siv_gn;
	return _siv_gps + _siv_glo + _siv_gal;
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
