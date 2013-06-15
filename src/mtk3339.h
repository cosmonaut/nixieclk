/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   MTK3339 interface

   NOTE: uart_init must be called before using this library.
*/

#ifndef _MTK3339_H_
#define _MTK3339_H_

#define MTK3339_DEFAULT_BAUD 9600

#define PMTK_TEST "$PMTK000*32\r\n"

#define PMTK_SET_NMEA_UPDATERATE_1HZ "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATERATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATERATE_10HZ "$PMTK220,100*2F\r\n"

#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_NMEA_OUTPUT_RMC_GSA "$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define PMTK_SET_NMEA_OUTPUT_DEFAULT "$PMTK314,-1*04\r\n"

#define PMTK_SET_NMEA_BAUDRATE_DEFAULT "$PMTK251,0*28\r\n"
#define PMTK_SET_NMEA_BAUDRATE_38400 "$PMTK251,38400*27\r\n"
#define PMTK_SET_NMEA_BAUDRATE_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_NMEA_BAUDRATE_115200 "$PMTK251,115200*1F\r\n"

void mtk3339_init(void);
void mtk3339_hw_init(void);
void mtk3339_enable_int(void);
void mtk3339_disable_int(void);
void mtk3339_test(void);
void mtk3339_set_output_rmc(void);
void mtk3339_set_output_default(void);
uint8_t mtk3339_send_command(char* cmd, uint8_t ack);

#endif
