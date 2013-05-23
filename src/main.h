/*
  LUFA Library
  Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
  www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for VirtualSerial.c.
 */

#ifndef _MAIN_H_
#define _MAIN_H_

/* Includes: */
#include <LUFA/Drivers/USB/USB.h>
#include "descriptors.h"

//volatile uint8_t pps;

typedef struct {
    volatile uint8_t seconds;
    volatile uint8_t minutes;
    volatile uint8_t hours;
} nixie_time_t;

typedef struct {
    volatile uint8_t seconds;
    volatile uint8_t tens_seconds;
    volatile uint8_t minutes;
    volatile uint8_t tens_minutes;
    volatile uint8_t hours;
    volatile uint8_t tens_hours;
} nixie_time_digits_t;

/* Function Prototypes: */
void setup_hardware(void);
void increment_time(void);

/* USB stuff */
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);

#endif

