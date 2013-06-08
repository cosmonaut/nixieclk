/* Author: Nicholas Nell
   email: nicholas.nell@colorado.edu

   SPI
*/

#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>

void spi_init(void);
void spi_master_tx(uint8_t payload);

#endif

