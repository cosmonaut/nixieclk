/* Nixie Clock 
   Author: Nicholas Nell
*/

#include "main.h"
#include "twi_master.h"
#include "ds3231.h"
#include "uart.h"

/* usb data transmit ready status */
volatile bool dtr_status;
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
    float t = 0.0;
    uint8_t memwad[19];
    uint8_t e_stat = 0;
    uint8_t uart_byte = 0x00;

    /* Initialize at90usb1287 peripherals */
    setup_hardware();

    /* Create a regular character stream for the interface so that it
       can be used with the stdio.h functions */
    //CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    /* enable interrupts */
    sei();

    /* Initialize the DS3231 RTC */
    ds3231_init();

    /* make the magic happen */
    for (;;) {
        /* Must throw away unused bytes from the host, or it will lock
           up while waiting for the device */
        // if (USB_DeviceState == DEVICE_STATE_Configured) {
        //     rx_num_bytes = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
        //     for (i = 0; i < rx_num_bytes; i++) {
        //         // we'll need a nice buffer for these in the future...
        //         my_byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
        //     }
        // }

        if (pps) {
            //sw = 0;
            /* wait for TWI and read the whole register */
            // while (TWI_busy){};
            // TWI_buffer_out[0] = 0x00;
            // TWI_master_start_write(DS3231_ADDR, 1);
            // while (TWI_busy){};
    

            // TWI_master_start_read(DS3231_ADDR, DS3231_NREG);
            // while (TWI_busy){};
            // //sprintf(sbuf, "
            // for (i = 0; i < 19; i++) {
            //     j += sprintf(sbuf + j, "0x%02X ", TWI_buffer_in[i]);
            // }
            // j = 0;
            //sprintf(sbuf, "%i:%i:%i\n", the_time.hours, the_time.minutes, the_time.seconds);
            //e_stat = ds3231_get_registers(memwad);


            // e_stat = ds3231_print_info(sbuf);
            // if (!e_stat) {
            //     sw = 0;
            // }

            // e_stat = ds3231_get_time_digits(&nixie_time);
            // sprintf(sbuf, "%i%i:%i%i:%i%i\n", nixie_time.tens_hours,
            //         nixie_time.hours, nixie_time.tens_minutes, nixie_time.minutes,
            //         nixie_time.tens_seconds, nixie_time.seconds);
            // sw = 0;

            pps = 0;
        }



        //PORTD &= ~(1 << PD6);            
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

                /* Temporary GPS echo from uart -> usb */
                if (UCSR1A & (1 << RXC1)) {
                    while (UCSR1A & (1 << RXC1)) {
                        uart_byte = uart_receive();
                        cdc_dev_status = CDC_Device_SendByte(&VirtualSerial_CDC_Interface, uart_byte);
                    }
                }

            }
        }
        
       

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

    /* LED to output */
    DDRD = (1 << PD6);
    /* LED on */
    PORTD |= (1 << PD6);

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* INT6 setup for external rising edge */
    ds3231_hw_init();

    /* init TWI */
    TWI_init();

    /* init uart */
    uart_init(0x00);

    /* Hardware Initialization */
    USB_Init();
}

/* Call to increment by one second */
void increment_time(void) {
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

