/* Nixie Clock 
   Author: Nicholas Nell
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "ds3231.h"
#include "twi_master.h"
#include "uart.h"
#include "mtk3339.h"
#include "nmea.h"
#include "spi.h"

/* Clock operation modes */
#define GPS_FIX_NEW 1
#define GPS_FIX_STABLE 2
#define GPS_FIX_NONE 3
#define GPS_FIX_CHECK_TIME 4
#define MAN_SET_TIME 5

/* Tube display modes */
#define NIXIE_TIME_MODE 0
#define NIXIE_WAVE_MODE 1
#define NIXIE_SET_MODE 2
#define NIXIE_DATE_MODE 3
#define NIXIE_TEMP_MODE 4

#define SEC_OS 0
#define TENS_SEC_OS 10
#define MIN_OS 20
#define TENS_MIN_OS 32
#define HR_OS 42
#define TENS_HR_OS 52

/* Switch positions */
#define SW_TSET 0x01
#define SW_NEXT 0x02
#define SW_TUP 0x04
#define SW_TDOWN 0x08
#define SW_SHDN 0x10

static inline uint8_t tmod(uint8_t);
static inline uint8_t dectobcd(uint8_t);
static inline void time_to_nix_digits(nixie_time_t, nixie_time_digits_t *);
static void date_to_nixie_digits(nixie_date_t, volatile uint8_t *);
static void temp_to_nixie_digits(float, volatile uint8_t *);
static inline void nixie_time_to_nixie_digits(nixie_time_digits_t, volatile uint8_t *);
static inline void blank_digit(uint8_t, volatile uint8_t *);
static void set_system_time(nixie_time_digits_t);

/* usb data transmit ready status */
volatile bool dtr_status;
volatile uint8_t seconds_cnt = 0;
/* seconds -> hours from indices 0->8 */
volatile uint8_t nixie_digits[8];
volatile uint8_t nixie_mode = 0;
volatile uint8_t mode = 0;
/* switch time set items */
volatile uint8_t pa = 0x1f;
volatile uint8_t test = 0x00;
volatile uint8_t pa_store = 0x1f;
volatile uint8_t hv = 0x00;
volatile uint8_t set_digit = 5;

nixie_time_t the_time = {.seconds = 0, 
                         .minutes = 0, 
                         .hours = 0}; 
nixie_date_t the_date = {.day = 1,
                         .month = 1,
                         .year = 0};
nixie_time_digits_t nixie_time = {.seconds = 0, 
                                  .tens_seconds = 0, 
                                  .minutes = 0, 
                                  .tens_minutes = 0, 
                                  .hours = 0, 
                                  .tens_hours = 0}; 

nixie_time_digits_t time_setting = {.seconds = 0, 
                                    .tens_seconds = 0, 
                                    .minutes = 0, 
                                    .tens_minutes = 0, 
                                    .hours = 0, 
                                    .tens_hours = 0}; 

/* Sequence of digits that loop through every digit in the tube in
   height order and back */
volatile const uint8_t dig_loop[18] = {1, 0, 2, 6, 9, 5, 7, 8, 4, 3, 4, 8, 7, 5, 9, 6, 2, 0};

/* Hamming Weight Lookup Table for one byte */
static volatile uint8_t pcnt_lktb[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 
    3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 
    3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 
    5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 
    3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 
    4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 
    5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 
    4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 
    3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 
    5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 
    4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 
    6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 
    5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};


/* LUFA CDC Class driver interface configuration and state
 * information. This structure is passed to all CDC Class driver
 * functions, so that multiple instances of the same class within a
 * device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
    .Config = {
        .ControlInterfaceNumber   = 0,
        .DataINEndpoint = {
            .Address          = CDC_TX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .DataOUTEndpoint = {
            .Address          = CDC_RX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .NotificationEndpoint = {
            .Address          = CDC_NOTIFICATION_EPADDR,
            .Size             = CDC_NOTIFICATION_EPSIZE,
            .Banks            = 1,
        },
    },
};

/* Standard file stream for the CDC interface when set up, so that the
 * virtual CDC COM port can be used like any regular character stream
 * in the C APIs
 */
//static FILE USBSerialStream;


/* Main program entry point. This routine contains the overall program
 * flow, including initial setup of all components and the main
 * program loop.
 */
int main(void) {
    uint8_t my_byte = 0;
    uint16_t rx_num_bytes = 0;
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t sw = 0;
    uint8_t cdc_dev_status = 0;
    char sbuf[255];
    //float t = 0.0;
    //uint8_t memwad[19];
    uint8_t e_stat = 0;
    uint8_t gps_fix_state = 0;

    uint16_t k = 0;
    uint8_t p = 0;

    uint8_t sw_cnt = 0;
    uint8_t blink_sw = 0;

    uint8_t date_cnt = 0;
    float temp = 0;
    uint8_t nixie_mode_last = 0;

    /* Let ISP verification work? */
    //_delay_ms(5);

    nixie_mode = NIXIE_TIME_MODE;
    nixie_mode_last = NIXIE_TIME_MODE;

    /* Initialize at90usb1287 peripherals */
    setup_hardware();

    /* Create a regular character stream for the interface so that it
       can be used with the stdio.h functions */
    //CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    memset((void *)nixie_digits, 0x00, sizeof(nixie_digits));

    the_time.hours = 0;
    the_time.minutes = 0;
    the_time.seconds = 0;

    /* enable interrupts */
    sei();

    //pgtop = 0;
    //gprmc = 0;

    /* Wait for a valid GPS packet before initializing it (boot
       time) */
    while(pgtop < 1) {
        uart_task();
        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }

    /* Initialize the DS3231 RTC */
    ds3231_init();

    /* Initialize MTK3339 GPS unit */
    mtk3339_init();

    /* Start by assuming no GPS fix */
    mode = GPS_FIX_NONE;
    gps_fix_state = 0;
    /* default mode ds3231 */
    PORTC |= (1 << PC6);

    /* make the magic happen */
    for (;;) {
        /* Check GPS fix status and switch modes if needed */
        if (gps_fix != gps_fix_state) {
            gps_fix_state = gps_fix;
            if (gps_fix) {
                mode = GPS_FIX_NEW;
                seconds_cnt = 0;
                /* 3D Fix on */
                PORTC |= (1 << PC4);
            } else {
                /* Enable ds3231 pps line */
                mode = GPS_FIX_NONE;
                mtk3339_disable_int();
                ds3231_enable_int();
                /* 3D Fix off */
                PORTC &= ~(1 << PC4);
                /* GPS mode off/ ds3231 mode on */
                PORTC &= ~(1 << PC5);
                PORTC |= (1 << PC6);
            }
        }

        /* Handle mode tasks */
        switch(mode) {
        case GPS_FIX_NEW:
            if (seconds_cnt > 9) {
                mode = GPS_FIX_STABLE;
                ds3231_disable_int();
                the_time.hours = tmod(gps_time.hours - 7);
                the_time.minutes = gps_time.minutes;
                the_time.seconds = gps_time.seconds;
                /* Set ds3231 date/time */
                ds3231_set_time_gps(gps_time);
                ds3231_set_date(gps_date);
                mtk3339_enable_int();
                PORTC &= ~(1 << PC6);
                PORTC |= (1 << PC5);
            }
            break;
        case GPS_FIX_STABLE:
            if (seconds_cnt > 254) {
                mode = GPS_FIX_CHECK_TIME;
            }
            break;
        case GPS_FIX_NONE:
            break;
        case GPS_FIX_CHECK_TIME:
            if ((the_time.seconds != gps_time.seconds) |
                (the_time.minutes != gps_time.minutes) |
                (the_time.hours != tmod(gps_time.hours - 7))) {
                /* Set time */
                the_time.hours = tmod(gps_time.hours - 7);
                the_time.minutes = gps_time.minutes;
                the_time.seconds = gps_time.seconds;
            }
            mode = GPS_FIX_STABLE;
            break;
        case MAN_SET_TIME:
            ds3231_disable_int();
            set_system_time(time_setting);
            ds3231_enable_int();
            mode = GPS_FIX_NONE;
        }
        

        if (nixie_mode != nixie_mode_last) {
            nixie_mode_last = nixie_mode;
            /* Reset all mode-specific vars */
            date_cnt = 0;
            sw_cnt = 0;
        }

        //while(1) {
        if (nixie_mode == NIXIE_WAVE_MODE) {
            memset((void *)nixie_digits, 0x00, 8);
            p = SEC_OS + dig_loop[k%18];
            nixie_digits[p/8] = (1 << p%8);
            p = TENS_SEC_OS + dig_loop[(k + 1)%18];
            nixie_digits[p/8] |= (1 << p%8);
            p = MIN_OS + dig_loop[(k + 2)%18];
            nixie_digits[p/8] |= (1 << p%8);
            p = TENS_MIN_OS + dig_loop[(k + 3)%18];
            nixie_digits[p/8] |= (1 << p%8);
            p = HR_OS + dig_loop[(k + 4)%18];
            nixie_digits[p/8] |= (1 << p%8);
            p = TENS_HR_OS + dig_loop[(k + 5)%18];
            nixie_digits[p/8] |= (1 << p%8);
            k++;

            for (j = sizeof(nixie_digits); j-- > 0; ) {
                spi_master_tx(nixie_digits[j]);
            }
            /* Latch the data */
            PORTC |= (1 << PC0);
            _delay_us(1);
            PORTC &= ~(1 << PC0);
            
            _delay_ms(50);
        } else if (nixie_mode == NIXIE_SET_MODE) {
            memset((void *)nixie_digits, 0x00, 8);
            nixie_time_to_nixie_digits(time_setting, nixie_digits);
            if (blink_sw) {
                blank_digit(set_digit, nixie_digits);
            }
            for (j = sizeof(nixie_digits); j-- > 0; ) {
                spi_master_tx(nixie_digits[j]);
            }
            /* Latch the data */
            PORTC |= (1 << PC0);
            _delay_us(1);
            PORTC &= ~(1 << PC0);

            sw_cnt++;
            if (sw_cnt > 49) {
                sw_cnt = 0;
                blink_sw ^= 0x01;
            }

            _delay_ms(10);
        } else if (nixie_mode == NIXIE_DATE_MODE) {
            ds3231_get_date(&the_date);
            memset((void *)nixie_digits, 0x00, 8);
            date_to_nixie_digits(the_date, nixie_digits);

            for (j = sizeof(nixie_digits); j-- > 0; ) {
                spi_master_tx(nixie_digits[j]);
            }
            /* Latch the data */
            PORTC |= (1 << PC0);
            _delay_us(1);
            PORTC &= ~(1 << PC0);
            
            date_cnt++;
            if (date_cnt > 199) {
                date_cnt = 0;
                nixie_mode = NIXIE_TEMP_MODE;
                //nixie_mode = NIXIE_TIME_MODE;
            }

            _delay_ms(10);
        } else if (nixie_mode == NIXIE_TEMP_MODE) {
            temp = ds3231_get_temp();
            memset((void *)nixie_digits, 0x00, 8);
            temp_to_nixie_digits(temp, nixie_digits);

            for (j = sizeof(nixie_digits); j-- > 0; ) {
                spi_master_tx(nixie_digits[j]);
            }
            /* Latch the data */
            PORTC |= (1 << PC0);
            _delay_us(1);
            PORTC &= ~(1 << PC0);

            date_cnt++;
            if (date_cnt > 199) {
                date_cnt = 0;
                nixie_mode = NIXIE_TIME_MODE;
            }

            _delay_ms(10);            
        }

        /* USB rx commands */
        if (USB_DeviceState == DEVICE_STATE_Configured) {
            /* Must throw away unused bytes from the host, or it will lock
               up while waiting for the device */
            rx_num_bytes = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
            if (rx_num_bytes > 0) {
                for (i = 0; i < rx_num_bytes; i++) {
                    // we'll need a nice buffer for these in the future...
                    my_byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
                }

                if ((char)my_byte == 'p') {
                    e_stat = ds3231_print_info(sbuf);
                    sw = 0;
                    if (!e_stat) {
                        //sw = 0;
                        //memset(sbuf, 0x00, sizeof(sbuf));
                    }
                    my_byte = 0x00;
                } else if ((char)my_byte == 't') {
                    e_stat = ds3231_get_time_digits(&nixie_time);
                    sprintf(sbuf, "%i%i:%i%i:%i%i\n", nixie_time.tens_hours,
                            nixie_time.hours, nixie_time.tens_minutes, nixie_time.minutes,
                            nixie_time.tens_seconds, nixie_time.seconds);
                    sw = 0;
                    my_byte = 0x00;
                } else if ((char)my_byte == 'g') {
                    if (dtr_status) {
                        sprintf(sbuf, "%02i:%02i:%02i\n", the_time.hours, the_time.minutes, the_time.seconds);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "GPS FIX: ");
                        itoa(gps_fix, sbuf, 10);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        //cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(gps_fix, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, gps_time_s);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        sprintf(sbuf, "%02i-%02i-%02i\n", gps_date.year, gps_date.month, gps_date.day);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "GSA packets: ");
                        itoa(gpgsa, sbuf, 10);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                    }
                } else if ((char)my_byte == 'r') {
                    mtk3339_set_output_rmc();
                } else if ((char)my_byte == 'd') {
                    mtk3339_set_output_default();
                } else if ((char)my_byte == 'l') {
                    if (dtr_status) {
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(pgtop, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(npackets, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(uart_rx_overflow, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(RingBuffer_GetCount(&uart_buf), sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(pmtk_ack, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(seconds_cnt, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        memset(sbuf, 0x00, sizeof(sbuf));
                    }
                } else if ((char)my_byte == 'u') {
                    /* mtk3339 debug stuff.. */
                    // uart_disable();
                    // uart_flush_buffer();
                    // nmea_flush();
                    // uart_init(57600);
                } else if ((char)my_byte == 'i') {
                    if (nixie_mode != NIXIE_SET_MODE) {
                        nixie_mode = NIXIE_WAVE_MODE;
                    }
                } else if ((char)my_byte == 'o') {
                    if (nixie_mode != NIXIE_SET_MODE) {
                        nixie_mode = NIXIE_TIME_MODE;
                    }
                } else if ((char)my_byte == 'k') {
                    /* debug for switches */
                    sprintf(sbuf, "PA: %02x\n", pa);
                    cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                    memset(sbuf, 0x00, sizeof(sbuf));
                    sprintf(sbuf, "test: %02x\n", test);
                    cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                    memset(sbuf, 0x00, sizeof(sbuf));
                    sprintf(sbuf, "HV: %02x\n", hv);
                    cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                    memset(sbuf, 0x00, sizeof(sbuf));
                }

            }
            

            /* Only write data if the host is reading */
            if (dtr_status) {
                // we can send stuff to usb.
                if (sw == 0) {
                    //cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "HELLO WORLD!");
                    //cdc_dev_status = CDC_Device_SendData(&VirtualSerial_CDC_Interface, TWI_buffer_in, 19);
                    //cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "MESSAGE TIME\n");
                    cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                    memset(sbuf, 0x00, sizeof(sbuf));
                    //cdc_dev_status = Cget multiple bytesDC_Device_SendData(&VirtualSerial_CDC_Interface, memwad, 19);
                    sw = 1;
                }

            }
        }
        
        // /* Some test stuff for HV5522 current */
        // /* blast some data to HV5522s */
        // for (j = 0; j < 8; j++) {
        //     spi_master_tx(0x00);
        // }
        // /* Latch! */
        // PORTC |= (1 << PC0);
        // _delay_us(1);
        // PORTC &= ~(1 << PC0);

        uart_task();
        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }
}


/* Configures the board hardware and chip peripherals for the demo's
   functionality. */
void setup_hardware(void) {
    uint8_t i = 0;

    /* For initializing tube display */
    memset((void *)nixie_digits, 0x00, 8);


    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    USB_Init();

    DDRC |= (1 << PC0) | (1 << PC1);
    /* Set HV5522 blanking on */
    PORTC &= ~(1 << PC1);
    PORTC &= ~(1 << PC0);

    /* SMD LED to output */
    DDRD = (1 << PD6);
    /* SMD LED on */
    PORTD |= (1 << PD6);

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* LED0 to output */
    DDRC |= (1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7);
    /* LED off! */
    PORTC &= ~((1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7));

    /* INT6 setup for external rising edge */
    ds3231_hw_init();

    /* init spi (for HV5522s) */
    spi_init();

    /* Set all filaments off */
    for (i = sizeof(nixie_digits); i-- > 0; ) {
        spi_master_tx(nixie_digits[i]);
    }
    /* Latch the data */
    PORTC |= (1 << PC0);
    _delay_us(1);
    PORTC &= ~(1 << PC0);


    /* Set HV5522 blanking off */
    PORTC |= (1 << PC1);
    /* Set Latch Enable off */
    PORTC &= ~(1 << PC0);

    /* init TWI */
    TWI_init();

    /* init uart */
    uart_init_buffer();
    uart_init(MTK3339_DEFAULT_BAUD);

    mtk3339_hw_init();

    /* MAX6818 stuff */
    DDRC |= (1 << PC2);

    PORTC |= (1 << PC2);
    _delay_us(1);
    PORTC &= ~(1 << PC2);
    _delay_us(1);
    PORTC |= (1 << PC2);

    /* Set port A as input */
    DDRA = 0x00;
    /* Set pcint5 as input */
    DDRB &= ~(1 << PB5);

    PCICR |= (1 << PCIE0);
    /* Only using max6818 interrupt for now */
    PCMSK0 |= (1 << PCINT5);
}

/* Call to increment by one second */
void increment_time(void) {
    uint8_t j = 0;
    seconds_cnt++;
    the_time.seconds++;
    if (the_time.seconds > 59) {
        the_time.minutes++;
        the_time.seconds = 0;
    }
    
    if (the_time.minutes > 59) {
        the_time.hours++;
        the_time.minutes = 0;
    }
    
    if (the_time.hours > 23) {
        the_time.hours = 0;
    }

    time_to_nix_digits(the_time, &nixie_time);

    /* Display on tubes if we're in time mode */
    if (nixie_mode == NIXIE_TIME_MODE) {
        /* Insert next two lines into display only code */
        memset((void *)nixie_digits, 0x00, sizeof(nixie_digits));
        nixie_time_to_nixie_digits(nixie_time, nixie_digits);

        /* blast some data to HV5522s (reverse order, 10s hours first,
           seconds last */
        for (j = sizeof(nixie_digits); j-- > 0; ) {
            spi_master_tx(nixie_digits[j]);
        }
        /* Latch the data */
        PORTC |= (1 << PC0);
        _delay_us(1);
        PORTC &= ~(1 << PC0);
    }
}

static void set_system_time(nixie_time_digits_t t) {
    if ((mode != GPS_FIX_STABLE) && (mode != GPS_FIX_CHECK_TIME)) {
        the_time.seconds = t.tens_seconds*10 + t.seconds;
        the_time.minutes = t.tens_minutes*10 + t.minutes;
        the_time.hours = t.tens_hours*10 + t.hours;
        
        ds3231_set_time(the_time);
    }
}

/* Function to make sure any hour of day subtraction comes out between
   0 and 23 */
static inline uint8_t tmod(uint8_t t) {
    if (t > 23) {
        t = 24 - (256 - t);
        return t;
    } 
    
    return t;
}

/* decimal to binary coded decimal helper */
static inline uint8_t dectobcd(uint8_t k) {
    return((k/10)*16 + (k%10));
}

static inline void time_to_nix_digits(nixie_time_t t, nixie_time_digits_t *td) {
    uint8_t p = 0;
    p = dectobcd(t.seconds);
    td->seconds = (p & 0x0f);
    td->tens_seconds = ((p >> 4) & 0x0f);
    p = dectobcd(t.minutes);
    td->minutes = (p & 0x0f);
    td->tens_minutes = ((p >> 4) & 0x0f);
    p = dectobcd(t.hours);
    td->hours = (p & 0x0f);
    td->tens_hours = ((p >> 4) & 0x0f);
}

static void date_to_nixie_digits(nixie_date_t d, volatile uint8_t *nd) {
    if (nd != 0) {
        memset((void *)nd, 0x00, 8);
        uint8_t n = 0;
        uint8_t temp = 0x00;

        temp = dectobcd(d.day);
        n = SEC_OS + (temp & 0x0f);
        nd[n/8] = (1 << n%8);
        n = TENS_SEC_OS + ((temp & 0xf0) >> 4);
        nd[n/8] |= (1 << n%8);

        temp = dectobcd(d.month);
        n = MIN_OS + (temp & 0x0f);
        nd[n/8] |= (1 << n%8);
        n = TENS_MIN_OS + ((temp & 0xf0) >> 4);
        nd[n/8] |= (1 << n%8);

        temp = dectobcd(d.year);
        n = HR_OS + (temp & 0x0f);
        nd[n/8] |= (1 << n%8);
        n = TENS_HR_OS + ((temp & 0xf0) >> 4);
        nd[n/8] |= (1 << n%8);
    }
}

static void temp_to_nixie_digits(float temp, volatile uint8_t *nd) {
    if (nd != 0) {
        memset((void *)nd, 0x00, 8);
        uint8_t n = 0;
        
        uint8_t c = 0;
        uint8_t d = 0;

        uint8_t bcd = 0;

        /* Split temperature into two ints */
        c = (uint8_t)temp;
        d = fmod(round(temp*100.0), 100.0);

        bcd = dectobcd(d);
        
        n = MIN_OS + (bcd & 0x0f);
        nd[n/8] |= (1 << n%8);
        n = TENS_MIN_OS + ((bcd & 0xf0) >> 4);
        nd[n/8] |= (1 << n%8);

        bcd = dectobcd(c);

        n = HR_OS + (bcd & 0x0f);
        nd[n/8] |= (1 << n%8);
        n = TENS_HR_OS + ((bcd & 0xf0) >> 4);
        nd[n/8] |= (1 << n%8);
    }
}

/* nd should be an array of bytes of at least size 8 */    
static inline void nixie_time_to_nixie_digits(nixie_time_digits_t t, volatile uint8_t *nd) {
    if (nd != 0) {
        memset((void *)nd, 0x00, 8);
        uint8_t n = 0;
        n = SEC_OS + t.seconds;
        nd[n/8] = (1 << n%8);
        n = TENS_SEC_OS + t.tens_seconds;
        nd[n/8] |= (1 << n%8);
        n = MIN_OS + t.minutes;
        nd[n/8] |= (1 << n%8);
        n = TENS_MIN_OS + t.tens_minutes;
        nd[n/8] |= (1 << n%8);
        n = HR_OS + t.hours;
        nd[n/8] |= (1 << n%8);
        n = TENS_HR_OS + t.tens_hours;
        nd[n/8] |= (1 << n%8);
    }
}

/* nd must be an array of bytes of at least size 8 */
static inline void blank_digit(uint8_t dig, volatile uint8_t *nd) {
    switch (dig) {
    case 0:
        nd[0] = 0x00;
        nd[1] &= ~(0x03);
        break;
    case 1:
        nd[1] &= ~(0xfc);
        nd[2] &= ~(0x0f);
        break;
    case 2:
        nd[2] &= ~(0xf0);
        nd[3] &= ~(0x3f);
        break;
    case 3:
        nd[4] = 0x00;
        nd[5] &= ~(0x03);
        break;
    case 4:
        nd[5] &= ~(0xfc);
        nd[6] &= ~(0x0f);
        break;
    case 5:
        nd[6] &= ~(0xf0);
        nd[7] &= ~(0x3f);
        break;
    default:
        break;
    }
}


/* Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) {
    //LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
    /* Switching PSU seems to put junk in the USB registers on boot --
       this gets rid of it. */
    // if (USB_DeviceState == DEVICE_STATE_Configured) {
    //     CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    //     CDC_Device_SendString(&VirtualSerial_CDC_Interface, "FLUSHED!");
    // }
    //PORTC |= (1 << PC7);
}

/* Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) {
    //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
    //PORTC &= ~(1 << PC7);
}

/* Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/* Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/* Event handler for the CDC Class driver Host-to-Device Line Encoding
 * Changed event.
 *
 *  \param[in] CDCInterfaceInfo Pointer to the CDC class interface
 *  configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
    bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

    /* Let us know what the host is up to (one byte -- no need for
       atomic) */
    dtr_status = CurrentDTRState;
}

ISR(PCINT0_vect) {
    uint8_t temp = 0x00;

    if (!(PINB & 0x20)) {

        /* EN on max6818 */
        PORTC &= ~(1 << PC2);
        _delay_us(1);

        /* Read pins */
        pa = (PINA & 0x1f);
        temp = pa ^ pa_store;
        test = temp;

        if (pcnt_lktb[temp] == 1) {
            switch(temp) {
            case SW_TSET:

                if (!(pa & SW_TSET)) {
                    //PORTC |= (1 << PC7);
                    time_to_nix_digits(the_time, &time_setting);
                    nixie_mode = NIXIE_SET_MODE;
                } else {
                    //PORTC &= ~(1 << PC7);
                    /* Check and adjust all digits to meet time limits */
                    if (time_setting.tens_hours == 2) {
                        if (time_setting.hours > 3) {
                            time_setting.hours = 0;
                        }
                    }

                    set_digit = 5;
                    
                    //set_system_time(time_setting);
                    if (mode == GPS_FIX_NONE) {
                        mode = MAN_SET_TIME;
                    }
                    
                    nixie_mode = NIXIE_TIME_MODE;

                }
                break;
            case SW_NEXT:
                if (nixie_mode == NIXIE_SET_MODE) {
                    if (!(pa & SW_NEXT)) {
                        set_digit--;
                        if (set_digit > 5) {
                            set_digit = 5;
                        }

                        /* Check and adjust all digits to meet time limits */
                        if (time_setting.tens_hours == 2) {
                            if (time_setting.hours > 3) {
                                time_setting.hours = 0;
                            }
                        }
                        

                    }
                } else {
                    if (!(pa & SW_NEXT)) {
                        /* Set date mode, clock will display date/temp */
                        nixie_mode = NIXIE_DATE_MODE;
                    }
                }

                break;
            case SW_TUP:
                if (nixie_mode == NIXIE_SET_MODE) {
                    if (!(pa & SW_TUP)){
                        if (set_digit == 5) {
                            time_setting.tens_hours++;
                            if (time_setting.tens_hours > 2) {
                                time_setting.tens_hours = 0;
                            }
                        } else if (set_digit == 4) {
                            time_setting.hours++;
                            if (time_setting.tens_hours == 2) {
                                if (time_setting.hours > 3) {
                                    time_setting.hours = 0;
                                }
                            } else {
                                if (time_setting.hours > 9) {
                                    time_setting.hours = 0;
                                }
                            }
                        } else if (set_digit == 3) {
                            time_setting.tens_minutes++;
                            if (time_setting.tens_minutes > 5) {
                                time_setting.tens_minutes = 0;
                            }
                        } else if (set_digit == 2) {
                            time_setting.minutes++;
                            if (time_setting.minutes > 9) {
                                time_setting.minutes = 0;
                            }
                        } else if (set_digit == 1) {
                            time_setting.tens_seconds++;
                            if (time_setting.tens_seconds > 5) {
                                time_setting.tens_seconds = 0;
                            }
                        } else if (set_digit == 0) {
                            time_setting.seconds++;
                            if (time_setting.seconds > 9) {
                                time_setting.seconds = 0;
                            }
                        }


                    }
                }
                break;
            case SW_TDOWN:
                if (nixie_mode == NIXIE_SET_MODE) {
                    if (!(pa & SW_TDOWN)){
                        if (set_digit == 5) {
                            time_setting.tens_hours--;
                            if (time_setting.tens_hours > 2) {
                                time_setting.tens_hours = 2;
                            }
                        } else if (set_digit == 4) {
                            time_setting.hours--;
                            if (time_setting.tens_hours == 2) {
                                if (time_setting.hours > 3) {
                                    time_setting.hours = 3;
                                }
                            } else {
                                if (time_setting.hours > 9) {
                                    time_setting.hours = 9;
                                }
                            }
                        } else if (set_digit == 3) {
                            time_setting.tens_minutes--;
                            if (time_setting.tens_minutes > 5) {
                                time_setting.tens_minutes = 5;
                            }
                        } else if (set_digit == 2) {
                            time_setting.minutes--;
                            if (time_setting.minutes > 9) {
                                time_setting.minutes = 9;
                            }
                        } else if (set_digit == 1) {
                            time_setting.tens_seconds--;
                            if (time_setting.tens_seconds > 5) {
                                time_setting.tens_seconds = 5;
                            }
                        } else if (set_digit == 0) {
                            time_setting.seconds--;
                            if (time_setting.seconds > 9) {
                                time_setting.seconds = 9;
                            }
                        }

                    }

                }
                break;
            case SW_SHDN:
                if (!(pa & SW_SHDN)) {
                    hv = 0x01;
                } else {
                    hv = 0x00;
                }
                break;
            default:
                break;
            }
        }
            
        pa_store = pa;

        /* EN back high (resets CH) */
        _delay_us(1);
        PORTC |= (1 << PC2);
    }
}

