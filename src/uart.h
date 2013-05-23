/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   UART interface
*/
#ifndef _UART_H_
#define _UART_H_

#include <LUFA/Drivers/Misc/RingBuffer.h>

#define NIX_UART_BUFFER_SIZE 128

RingBuffer_t uart_buf;
uint8_t uart_buf_data[NIX_UART_BUFFER_SIZE];
volatile uint8_t uart_rx_overflow;

void uart_init(uint32_t baud);
void uart_init_buffer(void);
void uart_flush_buffer(void);
void uart_disable(void);
void uart_transmit(uint8_t byte);
void uart_send_string(char* str);
void uart_task(void);

#endif
