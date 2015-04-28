
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 */
 
 /**
 * @file RF24_arch_config.h
 * General defines and includes for RF24/Linux
 */

 /**
 * Example of RF24_arch_config.h for RF24 portability
 *
 * @defgroup Porting_General Porting: General
 *
 * 
 * @{
 */
 
 
#ifndef __ARCH_CONFIG_H__
#define __ARCH_CONFIG_H__

#define TI_ARM


#include <stddef.h>
#include "compatibility.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/rom.h> // preprocessor define: TARGET_IS_BLIZZARD_RB1
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/ssi.h>

#include "arch/TI_ARM/spi_gpio.h"


#define _BV(x) (1<<(x))

#define HIGH 1
#define LOW 0
#define true 1
#define false 0
//#define bool uint8_t

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) ({x;})
#else
#define IF_SERIAL_DEBUG(x)
#endif

// Avoid spurious warnings
#if 1
#if ! defined( NATIVE ) && defined( ARDUINO )
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))
#undef PSTR
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
#endif
#endif

typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)
#define printf_P printf
#define strlen_P strlen
#define PROGMEM
#define pgm_read_word(p) (*(p)) 
#define PRIPSTR "%s"
#define pgm_read_byte(p) (*(p))


// Function, constant map as a result of migrating from Arduino
#define INPUT 0
#define OUTPUT 1

//#define _SPI_transfer(ssi_base, data) SPItransfer(ssi_base, data)
//extern void pinMode(uint32_t gpio_port_base, uint8_t pin, uint8_t direction);
//extern void digitalWrite(uint32_t gpio_port_base, uint8_t pin, uint8_t value);
//extern uint32_t SPItransfer(uint32_t ssi_base, uint32_t data);

//#define digitalWrite(gpio_port_base, pin, value)
//#define pinMode(gpio_port_base, pin, direction)

#define delay(milisec) __msleep(milisec)
#define delayMicroseconds(usec) __usleep(usec)
#define millis() __millis()


#endif // __ARCH_CONFIG_H__


/*@}*/	
