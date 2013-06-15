/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   MTK3339 interface

   NOTE: uart_init must be called before using this library.
*/

#include <util/delay.h>
#include "mtk3339.h"
#include "uart.h"
#include "nmea.h"
#include "main.h"

/* Prepare the correct pins and peripherals on the at90usb1287 to
   control the GPS. */
void mtk3339_hw_init(void) {
    /* Set int 7 on rising edge (GPS 1 pps line) */
    DDRE &= ~(1 << PE7);
    PORTE &= ~(1 << PE7);
    /* Rising edge interrupts */
    EICRB |= (1 << ISC71) | (1 << ISC70);
}

void mtk3339_enable_int(void) {
    /* Enable interrupt */
    EIMSK |= (1 << INT7);
}

void mtk3339_disable_int(void) {
    EIMSK &= ~(1 << INT7);
}

void mtk3339_init(void) {
    /* RMC only output for now */
    //mtk3339_send_command(PMTK_SET_NMEA_OUTPUT_RMC, 1);
    mtk3339_send_command(PMTK_SET_NMEA_OUTPUT_RMC_GSA, 1);

    /* Set higher baud rate so that we can get more packets per
       second. Ignore ack! */
    mtk3339_send_command(PMTK_SET_NMEA_BAUDRATE_57600, 0);

    /* Allow baud change to 'settle' */
    _delay_ms(100);

    /* No ack after this! */
    uart_disable();
    uart_flush_buffer();
    nmea_flush();
    uart_init(57600);

    mtk3339_send_command(PMTK_TEST, 1);
    
    mtk3339_send_command(PMTK_SET_NMEA_UPDATERATE_5HZ, 1);
}

void mtk3339_test(void) {
    mtk3339_send_command(PMTK_TEST, 1);
}

void mtk3339_set_output_rmc(void) {
    mtk3339_send_command(PMTK_SET_NMEA_OUTPUT_RMC, 1);
}

void mtk3339_set_output_default(void) {
    mtk3339_send_command(PMTK_SET_NMEA_OUTPUT_DEFAULT, 1);
}

/* Send a command to GPS:
   cmd: a complete command as a string (use #defines in mtk3339.h
   ack: 1 for must ack, 0 for ignore ack */
uint8_t mtk3339_send_command(char* cmd, uint8_t ack) {
    pmtk_ack = 0;
    pmtk_nack = 0;

    /* send command */
    uart_send_string(cmd);
    if (ack) {
        /* Loop while waiting for ack or nack */
        while(!pmtk_ack){
            uart_task();
            if (pmtk_nack) {
                pmtk_nack = 0;
                return 1;
            }
        }
        pmtk_ack = 0;
    }
    
    return 0;
}

ISR(INT7_vect) {
    increment_time();
    PORTD ^= (1 << PD6);
}

