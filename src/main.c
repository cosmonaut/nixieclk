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

#define GPS_FIX_NEW 1
#define GPS_FIX_STABLE 2
#define GPS_FIX_NONE 3
#define GPS_FIX_CHECK_TIME 4

static inline uint8_t tmod(uint8_t a);

/* usb data transmit ready status */
volatile bool dtr_status;
volatile uint8_t seconds_cnt = 0;
nixie_time_t the_time = {.seconds = 0, 
                         .minutes = 0, 
                         .hours = 0}; 
nixie_time_digits_t nixie_time = {.seconds = 0, 
                                  .tens_seconds = 0, 
                                  .minutes = 0, 
                                  .tens_minutes = 0, 
                                  .hours = 0, 
                                  .tens_hours = 0}; 

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
    uint8_t mode = 0;

    /* Initialize at90usb1287 peripherals */
    setup_hardware();

    /* Create a regular character stream for the interface so that it
       can be used with the stdio.h functions */
    //CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

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
    }

    /* Initialize the DS3231 RTC */
    ds3231_init();

    /* Initialize MTK3339 GPS unit */
    mtk3339_init();

    /* Start by assuming no GPS fix */
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
                the_time.hours = tmod(gps_time.hours - 6);
                the_time.minutes = gps_time.minutes;
                the_time.seconds = gps_time.seconds;
                /* Set ds3231 date/time */
                ds3231_set_time(gps_time);
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
                (the_time.hours != tmod(gps_time.hours - 6))) {
                /* Set time */
                the_time.hours = tmod(gps_time.hours - 6);
                the_time.minutes = gps_time.minutes;
                the_time.seconds = gps_time.seconds;
            }
            mode = GPS_FIX_STABLE;
            break;
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
                        sprintf(sbuf, "%i:%i:%i\n", the_time.hours, the_time.minutes, the_time.seconds);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        memset(sbuf, 0x00, sizeof(sbuf));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "GPS FIX: ");
                        itoa(gps_fix, sbuf, 10);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
                        //cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, itoa(gps_fix, sbuf, 10));
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, gps_time_s);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n");

                        sprintf(sbuf, "%i-%i-%i\n", gps_date.year, gps_date.month, gps_date.day);
                        cdc_dev_status = CDC_Device_SendString(&VirtualSerial_CDC_Interface, sbuf);
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
                    uart_disable();
                    uart_flush_buffer();
                    nmea_flush();
                    uart_init(57600);

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
        
       
        uart_task();
        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }
}


/* Configures the board hardware and chip peripherals for the demo's
   functionality. */
void setup_hardware(void) {
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* SMD LED to output */
    DDRD = (1 << PD6);
    /* SMD LED on */
    PORTD |= (1 << PD6);

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* LED0 to output */
    DDRC |= (1 << PC4) | (1 << PC5) | (1 << PC6);
    /* LED off! */
    PORTC &= ~((1 << PC4) | (1 << PC5) | (1 << PC6));

    /* INT6 setup for external rising edge */
    ds3231_hw_init();

    /* init TWI */
    TWI_init();

    /* init uart */
    uart_init_buffer();
    uart_init(MTK3339_DEFAULT_BAUD);

    mtk3339_hw_init();

    /* Hardware Initialization */
    USB_Init();
}

/* Call to increment by one second */
void increment_time(void) {
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

/* Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) {
    //LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
    /* Switching PSU seems to put junk in the USB registers on boot --
       this gets rid of it. */
    // if (USB_DeviceState == DEVICE_STATE_Configured) {
    //     CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    //     CDC_Device_SendString(&VirtualSerial_CDC_Interface, "FLUSHED!");
    // }
}

/* Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) {
    //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/* Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

    /* This doesn't always work... ? */
    if (USB_DeviceState == DEVICE_STATE_Configured) {
        CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    }

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

