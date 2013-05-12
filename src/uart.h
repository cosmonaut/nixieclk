/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   UART interface
*/
#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>
#include <avr/io.h>

void uart_init(uint8_t baud);
void uart_transmit(uint8_t byte);
uint8_t uart_receive(void);

#endif
