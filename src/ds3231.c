/* Author: Nicholas Nell
   Email: nico.nell@gmail.com 

   DS3231 Interface 
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
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

/* decimal to binary coded decimal helper */
static inline uint8_t dectobcd(uint8_t k) {
    return((k/10)*16 + (k%10));
}

/* Prepare the correct pins and peripherals on the at90usb1287 to
   control the ds3231 */
void ds3231_hw_init(void) {
    /* enable int 6 on rising edge (ds3231 1 pps line) */
    DDRE &= ~(1 << PE6);
    PORTE &= ~(1 << PE6);
    EICRB |= (1 << ISC61) | (1 << ISC60);
    ds3231_enable_int();
}

/* Enable interrupt */
void ds3231_enable_int(void) {
    EIMSK |= (1 << INT6);
}

/* Disable interrupt */
void ds3231_disable_int(void) {
    EIMSK &= ~(1 << INT6);
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

/* Set ds3231 time */
void ds3231_set_time_gps(gps_rmc_time_t time) {
    while(TWI_busy){};

    /* word address for time set start */
    TWI_buffer_out[0] = 0x00;
    TWI_buffer_out[1] = dectobcd(time.seconds);
    TWI_buffer_out[2] = dectobcd(time.minutes);
    /* maintain 24-hour setting */
    TWI_buffer_out[3] = (~(0xc0) & dectobcd(time.hours));

    /* write (UTC) time to chip */
    TWI_master_start_write(DS3231_ADDR, 4);
    
    while(TWI_busy){};
    return;
}

/* Set ds3231 time */
void ds3231_set_time(nixie_time_t time) {
    while(TWI_busy){};

    /* word address for time set start */
    TWI_buffer_out[0] = 0x00;
    TWI_buffer_out[1] = dectobcd(time.seconds);
    TWI_buffer_out[2] = dectobcd(time.minutes);
    /* maintain 24-hour setting */
    TWI_buffer_out[3] = (~(0xc0) & dectobcd(time.hours));

    /* write (UTC) time to chip */
    TWI_master_start_write(DS3231_ADDR, 4);
    
    while(TWI_busy){};
    return;
}

/* Set ds3231 date */
void ds3231_set_date(gps_rmc_date_t date) {
    while(TWI_busy){};

    /* word address for time set start */
    TWI_buffer_out[0] = 0x04;
    /* Day code 1-7 = Sunday - Saturday */
    // todo: add me?
    //TWI_buffer_out[1] = (0x07 & 0x01);
    TWI_buffer_out[1] = (0x3f & dectobcd(date.day));
    TWI_buffer_out[2] = (0x1f & dectobcd(date.month));
    TWI_buffer_out[3] = (dectobcd(date.year));

    /* write (UTC) date to chip */
    TWI_master_start_write(DS3231_ADDR, 4);

    while(TWI_busy){};
    return;
}    

/* Get Date */
void ds3231_get_date(nixie_date_t *date) {
    while(TWI_busy) {};

    TWI_buffer_out[0] = 0x04;
    TWI_master_start_write_then_read(DS3231_ADDR, 1, DS3231_DATE_NREG);
    while(TWI_busy) {};

    date->day = (TWI_buffer_in[0] & 0x0f) + ((TWI_buffer_in[0] & 0x30) >> 4)*10;
    date->month = (TWI_buffer_in[1] & 0x0f) + ((TWI_buffer_in[1] & 0x10) >> 4)*10;
    date->year = (TWI_buffer_in[2] & 0x0f) + ((TWI_buffer_in[2] & 0xf0) >> 4)*10;
}

// uint8_t ds3231_get_datetime() {

// }

/* Get ds3231 time as digits */
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
static uint8_t _ds3231_get_hours(uint8_t h) {
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
    
    j += sprintf(print_buf + j, "Seconds: %i\n", _ds3231_get_seconds(reg_buf[0]));
    j += sprintf(print_buf + j, "Minutes: %i\n", _ds3231_get_minutes(reg_buf[1]));
    j += sprintf(print_buf + j, "Hours: %i\n", _ds3231_get_hours(reg_buf[2]));
    j += sprintf(print_buf + j, "Day: %i\n", reg_buf[3]);
    j += sprintf(print_buf + j, "Date: %i\n", _ds3231_get_date(reg_buf[4]));
    j += sprintf(print_buf + j, "Month: %i\n", _ds3231_get_hours(reg_buf[5]));
    j += sprintf(print_buf + j, "Year: %i\n", _ds3231_get_year(reg_buf[6]));
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
    uint8_t t_msb;
    uint8_t t_lsb;

    while(TWI_busy) {};
    TWI_buffer_out[0] = 0x11;
    TWI_master_start_write_then_read(DS3231_ADDR, 1, 2);
    while(TWI_busy) {};

    t_msb = TWI_buffer_in[0];
    t_lsb = TWI_buffer_in[1];
    return(ds3231_convert_temp(t_msb, t_lsb));
}
        
/* blink LED on square wave stuff (for now) 
   Eventually this will be one of the 1 PPS timing interrupts */
ISR(INT6_vect) {
    increment_time();
    if (led) {
        PORTD &= ~(1 << PD6);
        led = 0;
    } else {
        PORTD |= (1 << PD6);
        led = 1;
    }
}

