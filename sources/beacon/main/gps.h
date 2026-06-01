#ifndef _GPS_HEADER_
#define _GPS_HEADER_

void gps_init(void);
void gps_configure(void);
bool gps_position_detected(void);
bool gps_need_reset(void);
void gps_reset(void);
void gps_get_data(void);
double gps_get_precision(void);
uint16_t gps_get_satellites(void);

#endif /* _GPS_HEADER_ */
