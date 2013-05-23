#ifndef _DS3231_H_
#define _DS3231_H_

#include "main.h"

#define DS3231_ADDR 0x68
#define DS3231_NREG 19 // number of registers in memory
#define DS3231_DATETIME_NREG 7 // number of registers for time
#define DS3231_TIME_NREG 3 // number of registers for time

/* Lots of DS3231 registers read out the same way... */
#define ds3231_get_seconds ds3231_get_reg_as_int
#define ds3231_get_minutes ds3231_get_reg_as_int
#define ds3231_get_date ds3231_get_reg_as_int
#define ds3231_get_year ds3231_get_reg_as_int

//struct nixie_time_digits_t;

void ds3231_hw_init(void);
uint8_t ds3231_init(void);
void ds3231_enable_int(void);
void ds3231_disable_int(void);
uint8_t ds3231_get_registers(uint8_t *);
uint8_t ds3231_get_reg_as_int(uint8_t);
uint8_t ds3231_print_info(char *);
float ds3231_convert_temp(uint8_t, uint8_t);
float ds3231_get_temp(void);
uint8_t ds3231_get_time_digits(nixie_time_digits_t *);

#endif

