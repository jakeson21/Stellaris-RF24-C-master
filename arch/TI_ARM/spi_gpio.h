
/**
 * @file spi.h
 * Class declaration for SPI helper files
 */

 /**
 * Example of spi.h class declaration for SPI portability
 *
 * @defgroup Porting_SPI Porting: SPI
 *
 * 
 * @{
 */

#ifndef RF24_SPI
#define	RF24_SPI

#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/rom.h> // preprocessor define: TARGET_IS_BLIZZARD_RB1
#include <driverlib/ssi.h>
#include <driverlib/pin_map.h>

uint8_t rf24_spiInit(uint32_t ssibase, uint32_t bitrate);
uint32_t SPItransfer(uint32_t ssibase, uint32_t data);
void pinMode(uint32_t gpio_port_base, uint8_t pin, uint8_t direction);
void digitalWrite(uint32_t gpio_port_base, uint8_t pin, uint8_t value);

#endif

/*@}*/
