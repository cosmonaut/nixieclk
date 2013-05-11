/* Author: Nicholas Nell
   Email: nico.nell@gmail.com 

   DS3231 Interface 
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "twi_master.h"
#include "ds3231.h"

static const uint8_t ds3231_init_seq[] PROGMEM = {
    0x00, // seconds
    0x00, // minutes
    0x00, // hours, 12/24 config
    0x01, // Day (1-7)
    0x01, // date (1-31)
    0x01, // month & century
    0x00, // year (00 - 99)
    0x00, // alarm1 seconds
    0x00, // alarm1 minutes
    0x00, // alarm1 hours
    0x01, // alarm1 date
    0x00, // alarm2 minutes
    0x00, // alarm2 hours
    0x01, // alarm2 date
    0x00, // control
    0x00, // control/status
    0x00 // aging offset
};

volatile uint8_t led = 0;

/* Prepare the correct pins and peripherals on the at90usb1287 to
   control the ds3231 */
void ds3231_hw_init(void) {
    /* enable int 6 on rising edge (ds3231 1 pps line) */
    DDRE &= ~(1 << PE6);
    PORTE &= ~(1 << PE6);
    EICRB |= (1 << ISC61) | (1 << ISC60);
    EIMSK |= (1 << INT6);
}

/* Initialize the DS3231 chip. TWI_init() must have been called for
   this to work. */
uint8_t ds3231_init(void) {
    uint8_t i = 0;

    while(TWI_busy){};

    /* word address for config start */
    TWI_buffer_out[0] = 0x00;

    /* call me only after TWI_init has been called */
    for (i = 0; i < sizeof(ds3231_init_seq); i++) {
        TWI_buffer_out[i + 1] = pgm_read_byte(&ds3231_init_seq[i]);
    }

    /* write config to chip */
    TWI_master_start_write(DS3231_ADDR, sizeof(ds3231_init_seq) + 1);

    while(TWI_busy){};
    return 0;
}

// uint8_t ds3231_set_datetime() {

// }

// uint8_t ds3231_get_datetime() {

// }

uint8_t ds3231_get_time_digits(nixie_time_digits_t *nix_digits) {
    uint8_t time_reg_array[DS3231_TIME_NREG];
    
    if (nix_digits == NULL) {
        return 1;
    }

    while(TWI_busy){};

    /* word address for time registers start */
    TWI_buffer_out[0] = 0x00;
    TWI_master_start_write_then_read(DS3231_ADDR, 1, DS3231_TIME_NREG);
    while(TWI_busy) {};
    
    memcpy(time_reg_array, (const void *)TWI_buffer_in, DS3231_TIME_NREG);
    
    nix_digits->seconds = (time_reg_array[0] & 0x0F);
    nix_digits->tens_seconds = ((time_reg_array[0] & 0x70) >> 4);
    nix_digits->minutes = (time_reg_array[1] & 0x0F);
    nix_digits->tens_minutes = ((time_reg_array[1] & 0x70) >> 4);
    nix_digits->hours = (time_reg_array[2] & 0x0F);
    nix_digits->tens_hours = ((time_reg_array[2] & 0x30) >> 4);
                           
    return 0;
}

/* Get all registers from the DS3231 and copy them to reg_array */
uint8_t ds3231_get_registers(uint8_t *reg_array) {
    if (reg_array == NULL) {
        return 1;
    }
    
    while(TWI_busy){};
    /* Start at first word */
    TWI_buffer_out[0] = 0x00;
    TWI_master_start_write_then_read(DS3231_ADDR, 1, DS3231_NREG);
    while(TWI_busy) {};

    memcpy(reg_array, (const void *)TWI_buffer_in, DS3231_NREG);
    
    return 0;
}

/* Convert DS3231 register into regular integer */
uint8_t ds3231_get_reg_as_int(uint8_t s) {
    return((s & 0x0F) + 10*((s & 0xF0) >> 4));
}

/* Convert DS3231 hours register into a number (0 - 23) */
uint8_t ds3231_get_hours(uint8_t h) {
    return((h & 0x0F) + 10*((h & 0x30) >> 4));
}

/* String representation of DS3231 registers */
// TODO: Add alarm registers... do we care?
uint8_t ds3231_print_info(char *print_buf) {
    uint8_t j = 0;
    uint8_t reg_buf[DS3231_NREG];
    uint8_t stat = 0;
    char tbuf[8];

    if (print_buf == NULL) {
        return 1;
    }

    memset(tbuf, 0x00, sizeof(tbuf));

    stat = ds3231_get_registers(reg_buf);
    if (stat) {
        return 1;
    }
    
    j += sprintf(print_buf + j, "Seconds: %i\n", ds3231_get_seconds(reg_buf[0]));
    j += sprintf(print_buf + j, "Minutes: %i\n", ds3231_get_minutes(reg_buf[1]));
    j += sprintf(print_buf + j, "Hours: %i\n", ds3231_get_hours(reg_buf[2]));
    j += sprintf(print_buf + j, "Day: %i\n", reg_buf[3]);
    j += sprintf(print_buf + j, "Date: %i\n", ds3231_get_date(reg_buf[4]));
    j += sprintf(print_buf + j, "Month: %i\n", ds3231_get_hours(reg_buf[5]));
    j += sprintf(print_buf + j, "Year: %i\n", ds3231_get_year(reg_buf[6]));
    j += sprintf(print_buf + j, "Control: 0x%02X\n", reg_buf[14]);
    j += sprintf(print_buf + j, "Ctrl/Stat: 0x%02X\n", reg_buf[15]);
    j += sprintf(print_buf + j, "Aging: 0x%02X\n", reg_buf[16]);
    dtostrf(ds3231_convert_temp(reg_buf[17], reg_buf[18]), 7, 2, tbuf);
    j += sprintf(print_buf + j, "Temp: %s\n", tbuf);
    
    return 0;
}

/* Convert the two temperature registers into a float value. MSB is
   integer temp in degrees celsius. The highest two bits of the LSB
   are fractional temperature in quarters. */
float ds3231_convert_temp(uint8_t msb, uint8_t lsb) {
    /* the LSB of temp is stored at the MSB of the word for some
       reason */
    return((float)msb + (float)(lsb >> 6)/4.0);
}

/* Retrieve temperature from DS3231 and return as a float */
float ds3231_get_temp(void) {
    //float temp = 0.0;
    uint8_t t_msb;
    uint8_t t_lsb;
    while(TWI_busy) {};
    TWI_buffer_out[0] = 0x11;
    TWI_master_start_write_then_read(DS3231_ADDR, 1, 2);
    while(TWI_busy) {};
    t_msb = TWI_buffer_in[0];
    t_lsb = TWI_buffer_in[1];
    /* the LSB of temp is stored at the MSB of the word for unknown
       reasons */
    //t_lsb = (t_lsb >> 6);

    //temp = (float)t_msb + (float)t_lsb/4.0;

    //return temp;
    return(ds3231_convert_temp(t_msb, t_lsb));
}
        
/* blink LED on square wave stuff (for now) 
   Eventually this will be one of the 1 PPS timing interrupts */
ISR(INT6_vect) {
    pps = 1;
    //PORTD ^= (1 << PD6);
    increment_time();
    if (led) {
        PORTD &= ~(1 << PD6);
        led = 0;
    } else {
        PORTD |= (1 << PD6);
        led = 1;
    }
}

