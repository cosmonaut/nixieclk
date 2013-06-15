/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   NMEA parser
*/

#include "uart.h"
#include "nmea.h"

#define UTC_ENTRY_LEN 10

uint8_t packet_buf[128];
uint8_t packet_buf_pos = 0;
uint8_t start_flag = 0;
uint8_t stop_flag_r = 0;
uint8_t packet_flag = 0;

void nmea_flush(void) {
    pgtop = 0;
    gprmc = 0;
    gpgsa = 0;
    npackets = 0;
    pmtk_ack = 0;
    pmtk_nack = 0;
    packet_buf_pos = 0;
    start_flag = 0;
    stop_flag_r = 0;
    packet_flag = 0;
    gps_fix = 0;
}

void nmea_parse(void) {
    uint8_t count = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t rx_byte = 0;
    uint8_t utc_place = 0;

    count = RingBuffer_GetCount(&uart_buf);

    if (!start_flag) {
        /* Look for start flag. If there is other junk in the buffer
           before a '$' this will ignore it... */
        for (i = 0; i < count; i++) {
            rx_byte = RingBuffer_Remove(&uart_buf);
            if (rx_byte == '$') {
                packet_buf[0] = rx_byte;
                packet_buf_pos++;
                start_flag = 1;
                break;
            }
        }
    }

    if (!stop_flag_r && start_flag) {
        count = RingBuffer_GetCount(&uart_buf);
        for (i = 0; i < count; i++) {
            rx_byte = RingBuffer_Remove(&uart_buf);
            if (rx_byte == '\r') {
                stop_flag_r = 1;
                packet_buf[packet_buf_pos] = rx_byte;
                packet_buf_pos++;
                break;
            } else {
                packet_buf[packet_buf_pos] = rx_byte;
                packet_buf_pos++;
            }
        }
    }

    if (!packet_flag && start_flag && stop_flag_r) {
        count = RingBuffer_GetCount(&uart_buf);
        if (count > 0) {
            rx_byte = RingBuffer_Remove(&uart_buf);
            if (rx_byte != '\n') {
                /* what?! */
            } else {
                packet_buf[packet_buf_pos] = rx_byte;
                packet_buf_pos++;
                packet_flag = 1;
            }                
        }
    }

    if (packet_flag) {
        /* parse packet... */
        if (strncmp((char*)packet_buf, "$PGTOP", 6) == 0) {
            pgtop++;
            /* add external antenna flag later */
        } else if (strncmp((char*)packet_buf, "$GPGSA", 6) == 0) { 
            i = 5;
            while(packet_buf[i++] != ',');
            /* Mode */

            while(packet_buf[i++] != ',');
            /* Fix status -- PPS line only runs on 3D fix */
            if ((char)packet_buf[i] == '3') {
                gps_fix = 1;
            } else {
                gps_fix = 0;
            }
            
            gpgsa++;
        } else if (strncmp((char*)packet_buf, "$GPRMC", 6) == 0) {
            i = 5;
            while(packet_buf[i++] != ',');
            /* UTC */
            utc_place = i;
            while(packet_buf[i++] != ',');
            /* packet valid/invalid */
            if ((char)packet_buf[i] == 'A') {
                //gps_fix = 1;
                if ((i - utc_place) == (UTC_ENTRY_LEN + 1)) {
                    gps_time.hours = (packet_buf[utc_place] - '0')*10 + (packet_buf[utc_place + 1] - '0');
                    gps_time.minutes = (packet_buf[utc_place + 2] - '0')*10 + (packet_buf[utc_place + 3] - '0');
                    gps_time.seconds = (packet_buf[utc_place + 4] - '0')*10 + (packet_buf[utc_place + 5] - '0');
                    memset(gps_time_s, 0x00, sizeof(gps_time_s));
                    for (j = 0; j < UTC_ENTRY_LEN; j++) {
                        gps_time_s[j] = (char)packet_buf[utc_place + j];
                    }
                }
                while(packet_buf[i++] != ',');
                /* Latitude */
                while(packet_buf[i++] != ',');
                /* N/S */
                while(packet_buf[i++] != ',');
                /* Longitude */
                while(packet_buf[i++] != ',');
                /* E/W */
                while(packet_buf[i++] != ',');
                /* ground speed */
                while(packet_buf[i++] != ',');
                /* ground heading */
                while(packet_buf[i++] != ',');
                /* Date */
                j = i;
                while(packet_buf[j++] != ',');
                if ((j - i) == 7) {
                    gps_date.day = (packet_buf[i] - '0')*10 + (packet_buf[i + 1] - '0');
                    gps_date.month = (packet_buf[i + 2] - '0')*10 + (packet_buf[i + 3] - '0');
                    gps_date.year = (packet_buf[i + 4] - '0')*10 + (packet_buf[i + 5] - '0');
                }
            } else {
                //gps_fix = 0;
            }
            gprmc++;
        } else if (strncmp((char*)packet_buf, "$PMTK001", 8) == 0) {
            i = 7;
            while(packet_buf[i++] != ',');
            while(packet_buf[i++] != ',');
            /* Shows a successful ack */
            if ((char)packet_buf[i] == '3') {
                pmtk_ack = 1;
            } else {
                pmtk_nack = 1;
            }
            //pmtk_ack = 1;

        }


        npackets++;

        start_flag = 0;
        stop_flag_r = 0;
        packet_flag = 0;
        packet_buf_pos = 0;
        
    }

}
