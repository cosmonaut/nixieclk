/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   UART interface
*/

#include "uart.h"
#include "nmea.h"

static uint8_t uart_receive(void);

/* Initialize interface */
void uart_init(uint32_t baud) {
    uint16_t ubrr_val = ((F_CPU/(baud*16UL)) - 1);
    UBRR1H = (ubrr_val >> 8);
    UBRR1L = ubrr_val;

    // 8 bit, 1 stop bit, no parity
    UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1);

    uart_rx_overflow = 0;

    /* Enable uart RX complete interrupt */
    UCSR1B |= (1 << RXCIE1);
}

/* Initialize the uart RX buffer. Run before uart_init() */
void uart_init_buffer(void) {
    /* Create ring buffer for uart rx */
    RingBuffer_InitBuffer(&uart_buf, uart_buf_data, sizeof(uart_buf_data));
}

/* Completely clear the uart RX buffer. */
void uart_flush_buffer(void) {
    while(!RingBuffer_IsEmpty(&uart_buf)) {
        RingBuffer_Remove(&uart_buf);
    }
}

/* Disable uart and interrupt */
void uart_disable(void) {
    /* Disable uart RX complete interrupt */
    UCSR1B &= ~(1 << RXCIE1);

    UCSR1B = 0x00;
    UCSR1A = 0x00;
    UCSR1C = 0x00;

    UBRR1H = 0x00;
    UBRR1L = 0x00;
}
    
void uart_send_string(char* str) {
    uint8_t i = 0;

    for (i = 0; str[i] != 0; i++) {
        uart_transmit(str[i]);
    }
}

/* transmit a byte */
void uart_transmit(uint8_t byte) {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = byte;
}

/* don't use... */
uint8_t uart_receive(void) {
    while (!(UCSR1A & (1 << RXC1)));
    return UDR1;
}

/* Must be called frequently in main loop! */
void uart_task(void) {
    if (RingBuffer_IsEmpty(&uart_buf)) {
        return;
    } else {
        /* Try and get NMEA stuff out of the buffer */
        nmea_parse();
    }
}

ISR(USART1_RX_vect) {
    uint8_t rx_byte = 0x00;
    rx_byte = UDR1;
    if (RingBuffer_IsFull(&uart_buf)) {
        /* overflow... no data sent along */
        uart_rx_overflow++;
    } else {
        RingBuffer_Insert(&uart_buf, rx_byte);
    }
}

