Author: Nicholas Nell
Email: nicholas.nell@colorado.edu

Updated: 03-21-2013

This is a simple nixie clock board meant to drive 6 tubes. It has
places for three different types of clock sources. These sources
follow:
* 3.2x1.5 mm 32 kHz crystal (connected directly to at90usb1287)
* Maxim DS3231 TCXO RTC (communicates via I2C)
* PA6H (MTK3339) GPS Module (communicates via UART)

Nixie driving is handled via SPI with two daisy-chained HV5530 (or
HV5522) chips.

The board requires one input voltage of ~15V. From this voltage +12V
is created by a zener regulator for the Supertex chips and +5V is
created via a buck regulator. A small linear regulator creates +3.3V
for the GPS module.

Changelog:

version 1:
* First version manufactured
