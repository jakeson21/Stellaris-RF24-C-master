
#include "spi_gpio.h"

uint8_t rf24_spiInit(uint32_t SSIBase, uint32_t bitrate){
	switch (SSIBase){
	case SSI0_BASE:
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	    /* SPI0 */
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	    GPIOPinConfigure(GPIO_PA4_SSI0RX);
	    GPIOPinConfigure(GPIO_PA5_SSI0TX);

	    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 |
	                                    GPIO_PIN_4 | GPIO_PIN_5);
		break;

	case SSI1_BASE:
		break;

	case SSI2_BASE:
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	    /* SSI2 */
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

	    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
	    GPIOPinConfigure(GPIO_PB6_SSI2RX);
	    GPIOPinConfigure(GPIO_PB7_SSI2TX);

	    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 |
	                                    GPIO_PIN_6 | GPIO_PIN_7);
		break;

	case SSI3_BASE:
		break;

	default:
		return 1;
	}

	SSIDisable(SSIBase);
	SSIConfigSetExpClk(SSIBase,
		SysCtlClockGet(),
		SSI_FRF_MOTO_MODE_0,
		SSI_MODE_MASTER, bitrate, 8);
	SSIEnable(SSIBase);
	SysCtlDelay(100000);

	return 0;
}

uint32_t SPItransfer(uint32_t ssi_base, uint32_t data)
{
	uint32_t rxData = 0;
	//while (ROM_SSIBusy(ssi_base)) {
	//}
	SSIDataPut(ssi_base, data);
#if defined(PART_LM4F120H5QR)
	SSIDataGet(ssi_base, (unsigned long *)&rxData);
#else
	SSIDataGet(ssi_base, &rxData);
#endif
	//while (ROM_SSIBusy(ssi_base)) {}
	return rxData;
}


void pinMode(uint32_t gpio_port_base, uint8_t pin, uint8_t direction){
	if (direction)
		GPIOPinTypeGPIOOutput(gpio_port_base, pin);
	else
		GPIOPinTypeGPIOInput(gpio_port_base, pin);
}

void digitalWrite(uint32_t gpio_port_base, uint8_t pin, uint8_t value){
	GPIOPinWrite(gpio_port_base, pin, value);
}

