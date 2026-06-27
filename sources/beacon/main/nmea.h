#ifndef _NMEA_HEADER_
#define _NMEA_HEADER_

#include <cstdint>

int nmea_encode(char c);
uint16_t nmea_get_satellites_in_view(void);

#endif /* _NMEA_HEADER_ */
