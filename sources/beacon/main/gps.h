#ifndef _GPS_HEADER_
#define _GPS_HEADER_

int gps_init(void);
bool gps_position_detected(void);
bool gps_need_reset(void);
int gps_reset(void);
int gps_get_data(void);
double gps_get_precision(void);
uint16_t gps_get_satellites(void);

#endif /* _GPS_HEADER_ */
