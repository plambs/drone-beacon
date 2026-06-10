#include "nmea.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Per-constellation satellites-in-view counters.
// Each GSV sentence type carries its own "total in view" count — keeping
// them separate avoids overwriting one constellation's value with another's.
static uint16_t _siv_gps = 0;
static uint16_t _siv_glo = 0;
static uint16_t _siv_gal = 0;
static uint16_t _siv_gn  = 0; // combined GNSS sentence ($GNGSV)

static void _parse_gsv(const char *sentence, uint16_t *siv_bucket)
{
	int fieldIndex = 0;
	uint16_t snr_worse = 99;
	uint16_t snr_best = 0;

	// strtok modifies the string in-place, so work on a local copy
	char buffer[120];
	strncpy(buffer, sentence, sizeof(buffer) - 1);
	buffer[sizeof(buffer) - 1] = '\0';

	char *token = strtok(buffer, ",");

	while (token != NULL) {
		if (fieldIndex == 3) {
			*siv_bucket = (uint16_t)atoi(token);
		}

		// SNR fields appear every 4th field starting at index 7
		if (fieldIndex >= 7 && ((fieldIndex - 7) % 4 == 0)) {
			int snr = atoi(token);
			if (snr > 0) {
				if (snr > snr_best && snr < 99) snr_best = snr;
				if (snr < snr_worse)            snr_worse = snr;
			}
		}

		token = strtok(NULL, ",");
		fieldIndex++;
	}

#if DEBUG_DISPLAY_SATELLITE_AND_SNR_DETAILS
	if (snr_best == 0 || snr_worse == 99)
		printf("Satellites in view: %d\n", *siv_bucket);
	else
		printf("Satellites in view: %d, SNR, best: %ddb, worst: %ddb\n",
		       *siv_bucket, snr_best, snr_worse);
#endif
}

static void _parse_nmea_sentence(const char *nmea)
{
#if DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS
	printf("%s\n", nmea);
#endif

	// Route each GSV sentence type to its own counter
	if      (strncmp(nmea, "$GPGSV", 6) == 0) _parse_gsv(nmea, &_siv_gps);
	else if (strncmp(nmea, "$GLGSV", 6) == 0) _parse_gsv(nmea, &_siv_glo);
	else if (strncmp(nmea, "$GAGSV", 6) == 0) _parse_gsv(nmea, &_siv_gal);
	else if (strncmp(nmea, "$GNGSV", 6) == 0) _parse_gsv(nmea, &_siv_gn);

	if (strncmp(nmea, "$PMTKSPF,3*58", 13) == 0) {
		printf("Interference detected!\n");
	}
}

uint16_t nmea_get_satellites_in_view(void)
{
	// Prefer the combined GNSS sentence when available; otherwise sum per-constellation
	if (_siv_gn > 0)
		return _siv_gn;
	return _siv_gps + _siv_glo + _siv_gal;
}

void nmea_encode(char c)
{
	static char nmea[128];
	static int  nmea_len = 0;

	if (c == '\n') {
		nmea[nmea_len] = '\0';
		_parse_nmea_sentence(nmea);
		nmea_len = 0;
	} else if (c != '\r') {
		if (nmea_len < (int)(sizeof(nmea) - 1))
			nmea[nmea_len++] = c;
	}
}
