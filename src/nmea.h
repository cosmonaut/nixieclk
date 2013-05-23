/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   NMEA parser
*/

#ifndef _NMEA_H_
#define _NMEA_H_

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
} gps_rmc_time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint8_t year;
} gps_rmc_date_t;

uint8_t pgtop;
uint8_t pmtk_ack;
uint8_t pmtk_nack;
uint8_t npackets;
uint8_t gprmc;
uint8_t gps_fix;
gps_rmc_time_t gps_time;
gps_rmc_date_t gps_date;
char gps_time_s[20];

void nmea_parse(void);
void nmea_flush(void);

#endif
