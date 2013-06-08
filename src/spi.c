/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   SPI
*/

#include <avr/io.h>
#include "spi.h"

void spi_init(void) {
    /* SCK and MOSI set as outputs */
    DDRB |= (1 << PB1) | (1 << PB2);
    /* PC0 and PC1 are latch enable and blanking */
    DDRC |= (1 << PC0) | (1 << PC1);

    /* Enable SPI master, set clock ate f/16 (1 MHz), clock phase
       trail */
    //SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << CPHA);
    /* 4 MHz */
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << CPHA);
}

/* Transmit and block until done */
void spi_master_tx(uint8_t payload) {
    SPDR = payload;
    while(!(SPSR & (1 << SPIF)));
}

