/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   UART interface
*/

#include "uart.h"

/* Initialize interface */
void uart_init(uint8_t baud) {
    //UBRRH = (baud >> 8);
    //UBRRL = baud;
    // 9600 baud
    UBRR1H = 0x00;
    UBRR1L = 103;

    // 8 bit, 1 stop bit, no parity
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);
}

void uart_transmit(uint8_t byte) {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = byte;
}

uint8_t uart_receive(void) {
    while (!(UCSR1A & (1 << RXC1)));
    return UDR1;
}

