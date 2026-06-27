#include "nmea.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "log.h"

// Per-constellation satellites-in-view counters.
// Each GSV sentence type carries its own "total in view" count — keeping
// them separate avoids overwriting one constellation's value with another's.
static uint16_t _siv_gps = 0;
static uint16_t _siv_glo = 0;
static uint16_t _siv_gal = 0;
static uint16_t _siv_gn  = 0; // combined GNSS sentence ($GNGSV)

static int _parse_gsv(const char *sentence, uint16_t *siv_bucket)
{
	int ret = 0;
	fail_if_null(sentence, -1, "sentence is NULL\n");
	fail_if_null(siv_bucket, -2, "siv_bucket is NULL\n");

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
		log_debug("Satellites in view: %d\n", *siv_bucket);
	else
		log_debug("Satellites in view: %d, SNR, best: %ddb, worst: %ddb\n",
		       *siv_bucket, snr_best, snr_worse);
#endif

	(void)ret;
	return 0;
}

static int _parse_nmea_sentence(const char *nmea)
{
	int ret = 0;
	fail_if_null(nmea, -1, "nmea is NULL\n");

#if DEBUG_DISPLAY_RAW_NMEA_RECEIVED_FROM_GPS
	log_debug("%s\n", nmea);
#endif

	// Route each GSV sentence type to its own counter
	if      (strncmp(nmea, "$GPGSV", 6) == 0) {
		ret = _parse_gsv(nmea, &_siv_gps);
		fail_if_negative(ret, -2, "_parse_gsv(GPS) failed, returned: %d\n", ret);
	}
	else if (strncmp(nmea, "$GLGSV", 6) == 0) {
		ret = _parse_gsv(nmea, &_siv_glo);
		fail_if_negative(ret, -3, "_parse_gsv(GLO) failed, returned: %d\n", ret);
	}
	else if (strncmp(nmea, "$GAGSV", 6) == 0) {
		ret = _parse_gsv(nmea, &_siv_gal);
		fail_if_negative(ret, -4, "_parse_gsv(GAL) failed, returned: %d\n", ret);
	}
	else if (strncmp(nmea, "$GNGSV", 6) == 0) {
		ret = _parse_gsv(nmea, &_siv_gn);
		fail_if_negative(ret, -5, "_parse_gsv(GN) failed, returned: %d\n", ret);
	}

	if (strncmp(nmea, "$PMTKSPF,3*58", 13) == 0) {
		log_warn("Interference detected!\n");
	}

	return 0;
}

uint16_t nmea_get_satellites_in_view(void)
{
	// Prefer the combined GNSS sentence when available; otherwise sum per-constellation
	if (_siv_gn > 0)
		return _siv_gn;
	return _siv_gps + _siv_glo + _siv_gal;
}

int nmea_encode(char c)
{
	int ret = 0;
	static char nmea[128];
	static int  nmea_len = 0;

	if (c == '\n') {
		nmea[nmea_len] = '\0';
		ret = _parse_nmea_sentence(nmea);
		fail_if_negative(ret, -1, "_parse_nmea_sentence failed, returned: %d\n", ret);
		nmea_len = 0;
	} else if (c != '\r') {
		if (nmea_len < (int)(sizeof(nmea) - 1))
			nmea[nmea_len++] = c;
	}

	return 0;
}
