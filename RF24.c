/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "RF24.h"

/****************************************************************************/

void RF24_csn(tRF24* rf24, bool mode) {

#if defined (RF24_TINY)
	if (rf24->rf24->ce_pin != rf24->csn_pin) {
		digitalWrite(rf24->csn_pin,mode);
	}
	else {
		if (mode == HIGH) {
			PORTB |= (1<<PINB2);  	// SCK->CSN HIGH
			delayMicroseconds(100);// allow csn to settle.
		}
		else {
			PORTB &= ~(1<<PINB2);	// SCK->CSN LOW
			delayMicroseconds(11);// allow csn to settle
		}
	}
	// Return, CSN toggle complete
	return;

#elif defined(ARDUINO)
	// Minimum ideal SPI bus speed is 2x data rate
	// If we assume 2Mbs data rate and 16Mhz clock, a
	// divider of 4 is the minimum we want.
	// CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz

#if !defined (SOFTSPI)
	_SPI.setBitOrder(MSBFIRST);
	_SPI.setDataMode(SPI_MODE0);
	_SPI.setClockDivider(SPI_CLOCK_DIV2);
#endif

#elif defined (RF24_RPi)
	_SPI.setBitOrder(RF24_BIT_ORDER);
	_SPI.setDataMode(RF24_DATA_MODE);
	_SPI.setClockDivider(rf24->spi_speed ? rf24->spi_speed : RF24_CLOCK_DIVIDER);
	_SPI.chipSelect(rf24->csn_pin);
	delayMicroseconds(5);
#endif

#if !defined (RF24_LINUX) && !defined (TI_ARM)
	digitalWrite(rf24->csn_pin, mode);
	delayMicroseconds(5);
#endif

#if defined (TI_ARM)
	if (mode)
		digitalWrite(rf24->csn_pin_port_base, rf24->csn_pin, rf24->csn_pin);
	else
		digitalWrite(rf24->csn_pin_port_base, rf24->csn_pin, 0);

#endif

}

/****************************************************************************/

void RF24_ce(tRF24* rf24, bool level) {
	//Allow for 3-pin use on ATTiny
#if ! defined(TI_ARM)
	if (rf24->ce_pin != rf24->csn_pin)
		digitalWrite(rf24->ce_pin, level);
#else
	if (rf24->ce_pin != rf24->csn_pin)
		if (level)
			digitalWrite(rf24->ce_pin_port_base, rf24->ce_pin, rf24->ce_pin);
		else
			digitalWrite(rf24->ce_pin_port_base, rf24->ce_pin, 0);
#endif
}

/****************************************************************************/

uint8_t RF24_read_register_chunk(tRF24* rf24, uint8_t reg, uint8_t* buf,
		uint8_t len) {
	uint8_t status;

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW); //In this case, calling csn(LOW) configures the spi settings for RPi
	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	uint8_t size = len + 1;// Add register value to transmit buffer

	*ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );

	while (len--) {*ptx++ = NOP;} // Dummy operation, just for reading

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, size);

	status = *prx++;// status is 1st byte of receive buffer

	// decrement before to skip status byte
	while ( --size ) {*buf++ = *prx++;}

#elif defined (RF24_DUE)
	status = _SPI.transfer(rf24->csn_pin, R_REGISTER | ( REGISTER_MASK & reg ), SPI_CONTINUE );
	while ( len-- > 1 ) {
		*buf++ = _SPI.transfer(rf24->csn_pin,0xff, SPI_CONTINUE);
	}
	*buf++ = _SPI.transfer(rf24->csn_pin,0xff);

#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base, R_REGISTER | ( REGISTER_MASK & reg));
	while (len--) {
		*buf++ = SPItransfer(rf24->ssi_base, 0xff);
	}
	RF24_csn(rf24, HIGH);

#else
	RF24_csn(rf24, LOW);
	status = _SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg));
	while (len--) {
		*buf++ = _SPI.transfer(0xff);
	}
	RF24_csn(rf24, HIGH);

#endif

	return status;
}

/****************************************************************************/

uint8_t RF24_read_register(tRF24* rf24, uint8_t reg) {
	uint8_t result;

#if defined (RF24_LINUX)

	RF24_csn(rf24, LOW);

	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	*ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
	*ptx++ = NOP; // Dummy operation, just for reading

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, 2);
	result = *++prx;// result is 2nd byte of receive buffer

#elif defined (RF24_DUE)
	_SPI.transfer(rf24->csn_pin, R_REGISTER | ( REGISTER_MASK & reg ) , SPI_CONTINUE);
	result = _SPI.transfer(rf24->csn_pin,0xff);
#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	SPItransfer(rf24->ssi_base, R_REGISTER | ( REGISTER_MASK & reg));
	result = SPItransfer(rf24->ssi_base, 0xff);
	RF24_csn(rf24, HIGH);

#else
	RF24_csn(rf24, LOW);
	_SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg));
	result = _SPI.transfer(0xff);

	RF24_csn(rf24, HIGH);
#endif

	return result;
}

/****************************************************************************/

uint8_t RF24_write_register_chunk(tRF24* rf24, uint8_t reg, const uint8_t* buf,
		uint8_t len) {
	uint8_t status;

#if defined (RF24_LINUX)

	RF24_csn(rf24, LOW);
	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	uint8_t size = len + 1; // Add register value to transmit buffer

	*ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	while ( len-- )
	*ptx++ = *buf++;

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, size);
	status = *prx;// status is 1st byte of receive buffer

#elif defined (RF24_DUE)
	status = _SPI.transfer(rf24->csn_pin, W_REGISTER | ( REGISTER_MASK & reg ), SPI_CONTINUE );
	while ( --len) {
		_SPI.transfer(rf24->csn_pin,*buf++, SPI_CONTINUE);
	}
	_SPI.transfer(rf24->csn_pin,*buf++);

#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base,  W_REGISTER | ( REGISTER_MASK & reg));
	while (len--)
		SPItransfer(rf24->ssi_base, *buf++);
	RF24_csn(rf24, HIGH);

#else

	RF24_csn(rf24, LOW);
	status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg));
	while (len--)
		_SPI.transfer(*buf++);

	RF24_csn(rf24, HIGH);

#endif

	return status;
}

/****************************************************************************/

uint8_t RF24_write_register(tRF24* rf24, uint8_t reg, uint8_t value) {
	uint8_t status;

	IF_SERIAL_DEBUG(printf_P(PSTR("RF24_write_register(rf24, %02x,%02x)\r\n"),reg,value));

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	*ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	*ptx = value;

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, 2);
	status = *prx++; // status is 1st byte of receive buffer

#elif defined (RF24_DUE)
	status = _SPI.transfer(rf24->csn_pin, W_REGISTER | ( REGISTER_MASK & reg ), SPI_CONTINUE);
	_SPI.transfer(rf24->csn_pin,value);
#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base,  W_REGISTER | ( REGISTER_MASK & reg));
	SPItransfer(rf24->ssi_base, value);
	RF24_csn(rf24, HIGH);

#else

	RF24_csn(rf24, LOW);
	status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg));
	_SPI.transfer(value);
	RF24_csn(rf24, HIGH);

#endif

	return status;
}

/****************************************************************************/

uint8_t RF24_write_payload(tRF24* rf24, const void* buf, uint8_t data_len,
		const uint8_t writeType) {
	uint8_t status;
	const uint8_t
	//* current = reinterpret_cast<const uint8_t*>(buf);
	* current = (const uint8_t*)(buf);

	data_len = rf24_min(data_len, rf24->payload_size);
	uint8_t blank_len =	rf24->dynamic_payloads_enabled ? 0 : rf24->payload_size - data_len;

	//printf("[Writing %u bytes %u blanks]",data_len,blank_len);
	IF_SERIAL_DEBUG( printf("[Writing %u bytes %u blanks]\n",data_len,blank_len); );

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	uint8_t size;
	size = data_len + blank_len + 1; // Add register value to transmit buffer

	*ptx++ = writeType;
	while ( data_len-- )
	*ptx++ = *current++;
	while ( blank_len-- )
	*ptx++ = 0;

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, size);
	status = *prx;// status is 1st byte of receive buffer

#elif defined (RF24_DUE)

	status = _SPI.transfer(rf24->csn_pin, writeType , SPI_CONTINUE);

	if(blank_len) {
		while ( data_len--) {
			_SPI.transfer(rf24->csn_pin,*current++, SPI_CONTINUE);
		}
		while ( --blank_len ) {
			_SPI.transfer(rf24->csn_pin,0, SPI_CONTINUE);
		}
		_SPI.transfer(rf24->csn_pin,0);
	} else {
		while( --data_len ) {
			_SPI.transfer(rf24->csn_pin,*current++, SPI_CONTINUE);
		}
		_SPI.transfer(rf24->csn_pin,*current);
	}

#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base, writeType);
	while (data_len--) {
		SPItransfer(rf24->ssi_base, *current++);
	}
	while (blank_len--) {
		SPItransfer(rf24->ssi_base, 0);
	}
	RF24_csn(rf24, HIGH);

#else

	RF24_csn(rf24, LOW);
	status = _SPI.transfer(writeType);
	while (data_len--) {
		_SPI.transfer(*current++);
	}
	while (blank_len--) {
		_SPI.transfer(0);
	}
	RF24_csn(rf24, HIGH);

#endif

	return status;
}

/****************************************************************************/

uint8_t RF24_read_payload(tRF24* rf24, void* buf, uint8_t data_len) {
	uint8_t status;
	//uint8_t* current = reinterpret_cast<uint8_t*>(buf);
	uint8_t* current = (uint8_t*)(buf);

	if (data_len > rf24->payload_size)
		data_len = rf24->payload_size;
	uint8_t blank_len =
			rf24->dynamic_payloads_enabled ? 0 : rf24->payload_size - data_len;

	//printf("[Reading %u bytes %u blanks]",data_len,blank_len);

	IF_SERIAL_DEBUG( printf("[Reading %u bytes %u blanks]\n",data_len,blank_len); );

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	uint8_t * prx = rf24->spi_rxbuff;
	uint8_t * ptx = rf24->spi_txbuff;
	uint8_t size;
	size = data_len + blank_len + 1; // Add register value to transmit buffer

	*ptx++ = R_RX_PAYLOAD;
	while(size--)
	*ptx++ = NOP;

	size = data_len + blank_len + 1;// Size has been lost during while, re affect

	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, size);

	status = *prx++;// 1st byte is status

	while ( --data_len )// Decrement before to skip 1st status byte
	*current++ = *prx++;

	*current = *prx;

#elif defined (RF24_DUE)

	status = _SPI.transfer(rf24->csn_pin, R_RX_PAYLOAD, SPI_CONTINUE );

	if( blank_len ) {
		while ( data_len-- ) {
			*current++ = _SPI.transfer(rf24->csn_pin,0xFF, SPI_CONTINUE);
		}

		while ( --blank_len ) {
			_SPI.transfer(rf24->csn_pin,0xFF, SPI_CONTINUE);
		}
		_SPI.transfer(rf24->csn_pin,0xFF);
	} else {
		while ( --data_len ) {
			*current++ = _SPI.transfer(rf24->csn_pin,0xFF, SPI_CONTINUE);
		}
		*current = _SPI.transfer(rf24->csn_pin,0xFF);
	}

#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base,  R_RX_PAYLOAD);
	while (data_len--) {
		*current++ = SPItransfer(rf24->ssi_base, 0xFF);
	}
	while (blank_len--) {
		SPItransfer(rf24->ssi_base, 0xff);
	}
	RF24_csn(rf24, HIGH);

#else

	RF24_csn(rf24, LOW);
	status = _SPI.transfer( R_RX_PAYLOAD);
	while (data_len--) {
		*current++ = _SPI.transfer(0xFF);
	}
	while (blank_len--) {
		_SPI.transfer(0xff);
	}
	RF24_csn(rf24, HIGH);

#endif

	return status;
}

/****************************************************************************/

uint8_t RF24_flush_rx(tRF24* rf24) {
	return RF24_spiTrans(rf24,  FLUSH_RX);
}

/****************************************************************************/

uint8_t RF24_flush_tx(tRF24* rf24) {
	return RF24_spiTrans(rf24, FLUSH_TX);
}

/****************************************************************************/

uint8_t RF24_spiTrans(tRF24* rf24, uint8_t cmd) {

	uint8_t status;
#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	status = _SPI.transfer( cmd );
#elif defined (RF24_DUE)
	status = _SPI.transfer(rf24->csn_pin, cmd );
#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	status = SPItransfer(rf24->ssi_base, cmd);
	RF24_csn(rf24, HIGH);

#else

	RF24_csn(rf24, LOW);
	status = _SPI.transfer(cmd);
	RF24_csn(rf24, HIGH);
#endif
	return status;
}

/****************************************************************************/

uint8_t RF24_get_status(tRF24* rf24) {
	return RF24_spiTrans(rf24, NOP);
}

/****************************************************************************/
#if !defined (MINIMAL)
void RF24_print_status(uint8_t status) {
	printf_P(
			PSTR(
					"STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
			status, (status & _BV(RX_DR)) ? 1 : 0,
			(status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0,
			((status >> RX_P_NO) & 0b111), (status & _BV(TX_FULL)) ? 1 : 0);
}

/****************************************************************************/

void RF24_print_observe_tx(tRF24* rf24, uint8_t value) {
	printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"), value,
			(value >> PLOS_CNT) & 0b1111, (value >> ARC_CNT) & 0b1111);
}

/****************************************************************************/

void RF24_print_byte_register(tRF24* rf24, const char* name, uint8_t reg,
		uint8_t qty) {
	//char extra_tab = strlen_P(name) < 8 ? '\t' : '\a';
	//printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
#if defined (RF24_LINUX)
	char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
	printf("%s\t%c =", name, extra_tab);
#else
	char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
	printf_P(PSTR(PRIPSTR"\t%c ="), name, extra_tab);
#endif
	while (qty--)
		printf_P(PSTR(" 0x%02x"), RF24_read_register(rf24, reg++));
	printf_P(PSTR("\r\n"));
}

/****************************************************************************/

void RF24_print_address_register(tRF24* rf24, const char* name, uint8_t reg,
		uint8_t qty) {

#if defined (RF24_LINUX)
	char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
	printf("%s\t%c =",name,extra_tab);
#else
	char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
	printf_P(PSTR(PRIPSTR"\t%c ="), name, extra_tab);
#endif
	while (qty--) {
		uint8_t buffer[rf24->addr_width];
		RF24_read_register_chunk(rf24, reg++, buffer, sizeof buffer);

		printf_P(PSTR(" 0x"));
		uint8_t* bufptr = buffer + sizeof buffer;
		while (--bufptr >= buffer)
			printf_P(PSTR("%02x"), *bufptr);
	}

	printf_P(PSTR("\r\n"));
}
#endif
/****************************************************************************/

void RF24_init(tRF24* rf24, uint8_t _cepin, uint8_t _cspin) {
	rf24->ce_pin = _cepin;
	rf24->csn_pin = _cspin;
	rf24->p_variant = false;
	rf24->payload_size = 32;
	rf24->dynamic_payloads_enabled = false;
	rf24->addr_width = 5;
}

/****************************************************************************/

#if defined (RF24_LINUX) && !defined (MRAA)//RPi constructor
void RF24_init_speed(tRF24* rf24, uint8_t _cepin, uint8_t _cspin, uint32_t _spi_speed) {
	rf24->ce_pin = _cepin;
	rf24->csn_pin = _cspin;
	rf24->spi_speed = _spi_speed;
	rf24->p_variant = false;
	rf24->payload_size = 32;
	rf24->dynamic_payloads_enabled = false;
	rf24->addr_width = 5;  //,rf24->pipe0_reading_address(0)
}
#endif

/****************************************************************************/

void RF24_setChannel(tRF24* rf24, uint8_t channel) {
	const uint8_t max_channel = 127;
	RF24_write_register(rf24, RF_CH, rf24_min(channel, max_channel));
}

/****************************************************************************/

void RF24_setPayloadSize(tRF24* rf24, uint8_t size) {
	rf24->payload_size = rf24_min(size, 32);
}

/****************************************************************************/

uint8_t RF24_getPayloadSize(tRF24* rf24) {
	return rf24->payload_size;
}

/****************************************************************************/

#if !defined (MINIMAL)

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
		rf24_datarate_e_str_0, rf24_datarate_e_str_1, rf24_datarate_e_str_2, };
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = { rf24_model_e_str_0,
		rf24_model_e_str_1, };
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits";
static const char * const rf24_crclength_e_str_P[] PROGMEM
		= { rf24_crclength_e_str_0, rf24_crclength_e_str_1,
				rf24_crclength_e_str_2, };
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = { rf24_pa_dbm_e_str_0,
		rf24_pa_dbm_e_str_1, rf24_pa_dbm_e_str_2, rf24_pa_dbm_e_str_3, };

#if defined (RF24_LINUX)
static const char rf24_csn_e_str_0[] = "CE0 (PI Hardware Driven)";
static const char rf24_csn_e_str_1[] = "CE1 (PI Hardware Driven)";
static const char rf24_csn_e_str_2[] = "CE2 (PI Hardware Driven)";
static const char rf24_csn_e_str_3[] = "Custom GPIO Software Driven";
static const char * const rf24_csn_e_str_P[] = {
	rf24_csn_e_str_0,
	rf24_csn_e_str_1,
	rf24_csn_e_str_2,
	rf24_csn_e_str_3,
};
#endif

void RF24_printDetails(tRF24* rf24) {

#if defined (RF24_RPi)
	printf("================ SPI Configuration ================\n" );
	if (rf24->csn_pin < BCM2835_SPI_CS_NONE ) {
		printf("CSN Pin  \t = %s\n",rf24_csn_e_str_P[rf24->csn_pin]);
	} else {
		printf("CSN Pin  \t = Custom GPIO%d%s\n", rf24->csn_pin,
				rf24->csn_pin==RPI_V2_GPIO_P1_26 ? " (CE1) Software Driven" : "" );
	}
	printf("CE Pin  \t = Custom GPIO%d\n", rf24->ce_pin );
	printf("Clock Speed\t = " );
	switch (rf24->spi_speed)
	{
		case BCM2835_SPI_SPEED_64MHZ : printf("64 Mhz"); break;
		case BCM2835_SPI_SPEED_32MHZ : printf("32 Mhz"); break;
		case BCM2835_SPI_SPEED_16MHZ : printf("16 Mhz"); break;
		case BCM2835_SPI_SPEED_8MHZ : printf("8 Mhz"); break;
		case BCM2835_SPI_SPEED_4MHZ : printf("4 Mhz"); break;
		case BCM2835_SPI_SPEED_2MHZ : printf("2 Mhz"); break;
		case BCM2835_SPI_SPEED_1MHZ : printf("1 Mhz"); break;
		case BCM2835_SPI_SPEED_512KHZ: printf("512 KHz"); break;
		case BCM2835_SPI_SPEED_256KHZ: printf("256 KHz"); break;
		case BCM2835_SPI_SPEED_128KHZ: printf("128 KHz"); break;
		case BCM2835_SPI_SPEED_64KHZ : printf("64 KHz"); break;
		case BCM2835_SPI_SPEED_32KHZ : printf("32 KHz"); break;
		case BCM2835_SPI_SPEED_16KHZ : printf("16 KHz"); break;
		case BCM2835_SPI_SPEED_8KHZ : printf("8 KHz"); break;
		default : printf("8 Mhz"); break;
	}
	printf("\n================ NRF Configuration ================\n");

#endif //Linux

	RF24_print_status(RF24_get_status(rf24));

	RF24_print_address_register(rf24,PSTR("RX_ADDR_P0-1"), RX_ADDR_P0, 2);
	RF24_print_byte_register(rf24,PSTR("RX_ADDR_P2-5"), RX_ADDR_P2, 4);
	RF24_print_address_register(rf24, PSTR("TX_ADDR"), TX_ADDR, 1);

	RF24_print_byte_register(rf24,PSTR("RX_PW_P0-6"), RX_PW_P0, 6);
	RF24_print_byte_register(rf24,PSTR("EN_AA"), EN_AA, 1);
	RF24_print_byte_register(rf24,PSTR("EN_RXADDR"), EN_RXADDR, 1);
	RF24_print_byte_register(rf24,PSTR("RF_CH"), RF_CH, 1);
	RF24_print_byte_register(rf24,PSTR("RF_SETUP"), RF_SETUP, 1);
	RF24_print_byte_register(rf24,PSTR("CONFIG"), CONFIG, 1);
	RF24_print_byte_register(rf24,PSTR("DYNPD/FEATURE"), DYNPD, 2);

#if defined (TI_ARM)
	printf_P(PSTR("Data Rate\t = %s\r\n"),pgm_read_word(&rf24_datarate_e_str_P[RF24_getDataRate(rf24)]));
	printf_P(PSTR("Model\t\t = %s\r\n"),pgm_read_word(&rf24_model_e_str_P[RF24_isPVariant(rf24)]));
	printf_P(PSTR("CRC Length\t = %s\r\n"),pgm_read_word(&rf24_crclength_e_str_P[RF24_getCRCLength(rf24)]));
	printf_P(PSTR("PA Power\t = %s\r\n"), pgm_read_word(&rf24_pa_dbm_e_str_P[RF24_getPALevel(rf24)]));

#elif defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__) || defined(LITTLEWIRE) || defined (RF24_BBB)
	printf_P(PSTR("Data Rate\t = %s\r\n"),pgm_read_word(&rf24_datarate_e_str_P[RF24_getDataRate(rf24)]));
	printf_P(PSTR("Model\t\t = %s\r\n"),pgm_read_word(&rf24_model_e_str_P[RF24_isPVariant(rf24)]));
	printf_P(PSTR("CRC Length\t = %s\r\n"),pgm_read_word(&rf24_crclength_e_str_P[RF24_getCRCLength(rf24)]));
	printf_P(PSTR("PA Power\t = %s\r\n"), pgm_read_word(&rf24_pa_dbm_e_str_P[RF24_getPALevel(rf24)]));

#else
	printf_P(PSTR("Data Rate\t = %S\r\n"),
			pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
	printf_P(PSTR("Model\t\t = %S\r\n"),
			pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
	printf_P(PSTR("CRC Length\t = %S\r\n"),
			pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
	printf_P(PSTR("PA Power\t = %S\r\n"),
			pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
#endif

}

#endif
/****************************************************************************/

void RF24_begin(tRF24* rf24) {

#if defined (RF24_LINUX)

	SPI();

#if defined (MRAA)
	GPIO();
	rf24->gpio.begin(rf24->ce_pin,rf24->csn_pin);
#endif

	switch(rf24->csn_pin) {     //Ensure valid hardware CS pin
		case 0: break;
		case 1: break;
		// Allow BCM2835 enums for RPi
		case 8: rf24->csn_pin = 0; break;
		case 7: rf24->csn_pin = 1; break;
		default: rf24->csn_pin = 0; break;
	}

	_SPI.begin(rf24->csn_pin);

	pinMode(rf24->ce_pin,OUTPUT);
	RF24_ce(rf24, LOW);

	delay(100);

#elif defined(LITTLEWIRE)
	pinMode(rf24->csn_pin,OUTPUT);
	_SPI.begin();
	RF24_csn(rf24, HIGH);
#elif defined(TI_ARM)
	// Initialize ce, csn pins
	pinMode(rf24->csn_pin_port_base, rf24->csn_pin, OUTPUT);
	pinMode(rf24->ce_pin_port_base, rf24->ce_pin, OUTPUT);

	rf24_spiInit(rf24->ssi_base, 1000000);

	RF24_ce(rf24, LOW);
	RF24_csn(rf24, HIGH);

#else
	// Initialize pins
	if (rf24->ce_pin != rf24->csn_pin) pinMode(rf24->ce_pin, OUTPUT);

#if defined (RF24_DUE)
	_SPI.begin(rf24->csn_pin);	// Using the extended SPI features of the DUE
	_SPI.setClockDivider(rf24->csn_pin, 11);// Set the bus speed to just under 8mhz on Due
	_SPI.setBitOrder(rf24->csn_pin,MSBFIRST);// Set the bit order and mode specific to this device
	_SPI.setDataMode(rf24->csn_pin,SPI_MODE0);
	RF24_ce(rf24, LOW);
#else
#if ! defined(LITTLEWIRE)
	if (rf24->ce_pin != rf24->csn_pin)
#endif
	pinMode(rf24->csn_pin,OUTPUT);

	_SPI.begin();
	RF24_ce(rf24, LOW);
	RF24_csn(rf24, HIGH);
#if defined (__ARDUINO_X86__)
	delay(100);
#endif
#endif
#endif //Linux

	// Must allow the radio time to settle else configuration bits will not necessarily stick.
	// This is actually only required following power up but some settling time also appears to
	// be required after resets too. For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	delay(5);

	// Reset CONFIG and enable 16-bit CRC.
	RF24_write_register(rf24,  CONFIG, 0b00001100);

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	RF24_setRetries(rf24, 5, 15);

	// Reset value is MAX
	//setPALevel( RF24_PA_MAX ) ;

	// Determine if this is a p or non-p RF24 module and then
	// reset our data rate back to default value. This works
	// because a non-P variant won't allow the data rate to
	// be set to 250Kbps.
	if (RF24_setDataRate(rf24, RF24_250KBPS)) {
		rf24->p_variant = true;
	}

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	RF24_setDataRate(rf24, RF24_1MBPS);

	// Initialize CRC and request 2-byte (16bit) CRC
	//setCRCLength( RF24_CRC_16 ) ;

	// Disable dynamic payloads, to match rf24->dynamic_payloads_enabled setting - Reset value is 0
	RF24_toggle_features(rf24);
	RF24_write_register(rf24, FEATURE, 0);
	RF24_write_register(rf24, DYNPD, 0);

	// Reset current status
	// Notice reset and flush is the last thing we do
	RF24_write_register(rf24, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	RF24_setChannel(rf24, 76);

	// Flush buffers
	RF24_flush_rx(rf24);
	RF24_flush_tx(rf24);

	RF24_powerUp(rf24); //Power up by default when begin() is called

	// Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
	// PTX should use only 22uA of power
	RF24_write_register(rf24, CONFIG, (RF24_read_register(rf24, CONFIG)) & ~_BV(PRIM_RX));

}

/****************************************************************************/

void RF24_startListening(tRF24* rf24) {
#if !defined (RF24_TINY) && ! defined(LITTLEWIRE)
	RF24_powerUp(rf24);
#endif
	RF24_write_register(rf24, CONFIG, RF24_read_register(rf24, CONFIG) | _BV(PRIM_RX));
	RF24_write_register(rf24, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	RF24_ce(rf24, HIGH);
	// Restore the pipe0 adddress, if exists
	if (rf24->pipe0_reading_address[0] > 0) {
		RF24_write_register_chunk(rf24, RX_ADDR_P0, rf24->pipe0_reading_address,
				rf24->addr_width);
	} else {
		RF24_closeReadingPipe(rf24, 0);
	}

	// Flush buffers
	//RF24_flush_rx(rf24);
	if (RF24_read_register(rf24, FEATURE) & _BV(EN_ACK_PAY)) {
		RF24_flush_tx(rf24);
	}

	// Go!
	//delayMicroseconds(100);
}

/****************************************************************************/
static const uint8_t child_pipe_enable[] PROGMEM = {
ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };

void RF24_stopListening(tRF24* rf24) {
	RF24_ce(rf24, LOW);

	delayMicroseconds(rf24->txRxDelay);

	if (RF24_read_register(rf24,  FEATURE) & _BV(EN_ACK_PAY)) {
		delayMicroseconds(rf24->txRxDelay); //200
		RF24_flush_tx(rf24);
	}
	//RF24_flush_rx(rf24);
	RF24_write_register(rf24, CONFIG, (RF24_read_register(rf24, CONFIG)) & ~_BV(PRIM_RX));

#if defined (RF24_TINY) || defined (LITTLEWIRE)
	// for 3 pins solution TX mode is only left with additonal powerDown/powerUp cycle
	if (rf24->ce_pin == rf24->csn_pin) {
		RF24_powerDown(rf24);
		RF24_powerUp(rf24);
	}
#endif
	RF24_write_register(rf24, EN_RXADDR,
			RF24_read_register(rf24, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0

	//delayMicroseconds(100);

}

/****************************************************************************/

void RF24_powerDown(tRF24* rf24) {
	RF24_ce(rf24, LOW); // Guarantee CE is low on powerDown
	RF24_write_register(rf24, CONFIG, RF24_read_register(rf24, CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24_powerUp(tRF24* rf24) {
	uint8_t cfg = RF24_read_register(rf24, CONFIG);

	// if not powered up then power up and wait for the radio to initialize
	if (!(cfg & _BV(PWR_UP))) {
		RF24_write_register(rf24, CONFIG, RF24_read_register(rf24, CONFIG) | _BV(PWR_UP));

		// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
		// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
		// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
		delay(5);
	}
}

/******************************************************************/
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
void RF24_errNotify(tRF24* rf24) {
#if defined (SERIAL_DEBUG) || defined (RF24_LINUX)
	printf_P(PSTR("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.\r\n"));
#endif
#if defined (FAILURE_HANDLING)
	RF24_failureDetected = 1;
#else
	delay(5000);
#endif
}
#endif
/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool RF24_write_multi(tRF24* rf24, const void* buf, uint8_t len,
		const bool multicast) {
	//Start Writing
	RF24_startFastWrite(rf24, buf, len, multicast, true);

	//Wait until complete or failed
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
	uint32_t timer = millis();
#endif

	while (!(RF24_get_status(rf24) & ( _BV(TX_DS) | _BV(MAX_RT)))) {
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if(millis() - timer > 85) {
			errNotify();
#if defined (FAILURE_HANDLING)
			return 0;
#else
			delay(100);
#endif
		}
#endif
	}

	RF24_ce(rf24, LOW);

	uint8_t status = RF24_write_register(rf24, STATUS,
			_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	//Max retries exceeded
	if (status & _BV(MAX_RT)) {
		RF24_flush_tx(rf24); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return 0;
	}
	//TX OK 1 or 0
	return 1;
}

bool RF24_write(tRF24* rf24, const void* buf, uint8_t len) {
	return RF24_write_multi(rf24, buf, len, 0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool RF24_writeBlocking(tRF24* rf24, const void* buf, uint8_t len,
		uint32_t timeout) {
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//This way the FIFO will fill up and allow blocking until packets go through
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	uint32_t timer = millis();//Get the time that the payload transmission started

	while ((RF24_get_status(rf24) & (_BV(TX_FULL)))) {//Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

		if (RF24_get_status(rf24) & _BV(MAX_RT)) {	//If MAX Retries have been reached
			RF24_reUseTX(rf24);	//Set re-transmit and clear the MAX_RT interrupt flag
			if (millis() - timer > timeout) {
				return 0;
			}//If this payload has exceeded the user-defined timeout, exit and return 0
		}
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if(millis() - timer > (timeout+85) ) {
			errNotify();
#if defined (FAILURE_HANDLING)
			return 0;
#endif
		}
#endif

	}

	//Start Writing
	RF24_startFastWrite(rf24, buf, len, 0, true);//Write the payload if a buffer is clear

	return 1;					//Return 1 to indicate successful transmission
}

/****************************************************************************/

void RF24_reUseTX(tRF24* rf24) {
	RF24_write_register(rf24, STATUS, _BV(MAX_RT));			  //Clear max retry flag
	RF24_spiTrans(rf24,  REUSE_TX_PL);
	RF24_ce(rf24, LOW);										//Re-Transfer packet
	RF24_ce(rf24, HIGH);
}

/****************************************************************************/

bool RF24_writeFast_multi(tRF24* rf24, const void* buf, uint8_t len,
		const bool multicast) {
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//Return 0 so the user can control the retrys and set a timer or failure counter if required
	//The radio will auto-clear everything in the FIFO as long as CE remains high

#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
	uint32_t timer = millis();
#endif

	while ((RF24_get_status(rf24) & (_BV(TX_FULL)))) {//Blocking only if FIFO is full. This will loop and block until TX is successful or fail

		if (RF24_get_status(rf24) & _BV(MAX_RT)) {
			//reUseTX();										  //Set re-transmit
			RF24_write_register(rf24, STATUS, _BV(MAX_RT));		//Clear max retry flag
			return 0;	//Return 0. The previous payload has been retransmitted
						//From the user perspective, if you get a 0, just keep trying to send the same payload
		}
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if(millis() - timer > 85 ) {
			errNotify();
#if defined (FAILURE_HANDLING)
			return 0;
#endif
		}
#endif
	}
	//Start Writing
	RF24_startFastWrite(rf24, buf, len, multicast, true);

	return 1;
}

bool RF24_writeFast(tRF24* rf24, const void* buf, uint8_t len) {
	return RF24_writeFast_multi(rf24, buf, len, 0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data
void RF24_startFastWrite(tRF24* rf24, const void* buf, uint8_t len,
		const bool multicast, bool startTx) { //TMRh20

	//write_payload( buf,len);
	RF24_write_payload(rf24, buf, len,
			multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	if (startTx) {
		RF24_ce(rf24, HIGH);
	}

}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void RF24_startWrite(tRF24* rf24, const void* buf, uint8_t len,
		const bool multicast) {

	// Send the payload

	//write_payload( buf, len );
	RF24_write_payload(rf24, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	RF24_ce(rf24, HIGH);
#if defined(CORE_TEENSY) || !defined(ARDUINO) || defined (RF24_BBB) || defined (RF24_DUE)
	delayMicroseconds(10);
#endif
	RF24_ce(rf24, LOW);

}

/****************************************************************************/

bool RF24_rxFifoFull(tRF24* rf24) {
	return RF24_read_register(rf24, FIFO_STATUS) & _BV(RX_FULL);
}
/****************************************************************************/

bool RF24_txStandBy(tRF24* rf24) {

#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
	uint32_t timeout = millis();
#endif
	while (!(RF24_read_register(rf24, FIFO_STATUS) & _BV(TX_EMPTY))) {
		if (RF24_get_status(rf24) & _BV(MAX_RT)) {
			RF24_write_register(rf24, STATUS, _BV(MAX_RT));
			RF24_ce(rf24, LOW);
			RF24_flush_tx(rf24);    //Non blocking, flush the data
			return 0;
		}
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if( millis() - timeout > 85) {
			errNotify();
#if defined (FAILURE_HANDLING)
			return 0;
#endif
		}
#endif
	}

	RF24_ce(rf24, LOW);			   //Set STANDBY-I mode
	return 1;
}

/****************************************************************************/

bool RF24_txStandBy_timeout(tRF24* rf24, uint32_t timeout, bool startTx) {

	if (startTx) {
		RF24_stopListening(rf24);
		RF24_ce(rf24, HIGH);
	}
	uint32_t start = millis();

	while (!(RF24_read_register(rf24, FIFO_STATUS) & _BV(TX_EMPTY))) {
		if (RF24_get_status(rf24) & _BV(MAX_RT)) {
			RF24_write_register(rf24, STATUS, _BV(MAX_RT));
			RF24_ce(rf24, LOW);								//Set re-transmit
			RF24_ce(rf24, HIGH);
			if (millis() - start >= timeout) {
				RF24_ce(rf24, LOW);
				RF24_flush_tx(rf24);
				return 0;
			}
		}
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		if( millis() - start > (timeout+85)) {
			errNotify();
#if defined (FAILURE_HANDLING)
			return 0;
#endif
		}
#endif
	}

	RF24_ce(rf24, LOW);				   //Set STANDBY-I mode
	return 1;

}

/****************************************************************************/

void RF24_maskIRQ(tRF24* rf24, bool tx, bool fail, bool rx) {

	RF24_write_register(rf24, CONFIG,
			(RF24_read_register(rf24, CONFIG)) | fail << MASK_MAX_RT | tx << MASK_TX_DS
					| rx << MASK_RX_DR);
}

/****************************************************************************/

uint8_t RF24_getDynamicPayloadSize(tRF24* rf24) {
	uint8_t result = 0;

#if defined (RF24_LINUX)
	rf24->spi_txbuff[0] = R_RX_PL_WID;
	rf24->spi_rxbuff[1] = 0xff;
	RF24_csn(rf24, LOW);
	_SPI.transfernb( (char *) rf24->spi_txbuff, (char *) rf24->spi_rxbuff, 2);
	result = rf24->spi_rxbuff[1];
	RF24_csn(rf24, HIGH);
#elif defined (RF24_DUE)
	_SPI.transfer(rf24->csn_pin, R_RX_PL_WID, SPI_CONTINUE );
	result = _SPI.transfer(rf24->csn_pin,0xff);
#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	SPItransfer(rf24->ssi_base,  R_RX_PL_WID);
	result = SPItransfer(rf24->ssi_base, 0xff);
	RF24_csn(rf24, HIGH);

#else
	RF24_csn(rf24, LOW);
	_SPI.transfer( R_RX_PL_WID);
	result = _SPI.transfer(0xff);
	RF24_csn(rf24, HIGH);

#endif

	if (result > 32) {
		RF24_flush_rx(rf24);
		delay(2);
		return 0;
	}
	return result;
}

/****************************************************************************/

bool RF24_available(tRF24* rf24) {
	return RF24_available_FIFO(rf24, NULL);
}

/****************************************************************************/

bool RF24_available_FIFO(tRF24* rf24, uint8_t* pipe_num) {
	if (!(RF24_read_register(rf24, FIFO_STATUS) & _BV(RX_EMPTY))) {

		// If the caller wants the pipe number, include that
		if (pipe_num) {
			uint8_t status = RF24_get_status(rf24);
			*pipe_num = (status >> RX_P_NO) & 0b111;
		}
		return 1;
	}

	return 0;

}

/****************************************************************************/

void RF24_read(tRF24* rf24, void* buf, uint8_t len) {
	// Fetch the payload
	RF24_read_payload(rf24, buf, len);

	//Clear the two possible interrupt flags with one command
	RF24_write_register(rf24, STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));

}

/****************************************************************************/

void RF24_whatHappened(tRF24* rf24, bool * tx_ok, bool * tx_fail,
		bool * rx_ready) {
	// Read the status & reset the status in one easy call
	// Or is that such a good idea?
	uint8_t status = RF24_write_register(rf24, STATUS,
			_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	// Report to the user what happened
	*tx_ok = status & _BV(TX_DS);
	*tx_fail = status & _BV(MAX_RT);
	*rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/
/*
 void RF24_openWritingPipe(tRF24* rf24, uint64_t value)
 {
 // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
 // expects it LSB first too, so we're good.

 RF24_write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), rf24->addr_width);
 RF24_write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), rf24->addr_width);


 //const uint8_t max_payload_size = 32;
 //write_register(RX_PW_P0,rf24_min(rf24->payload_size,max_payload_size));
 RF24_write_register(RX_PW_P0,rf24->payload_size);
 }
 */
/****************************************************************************/
void RF24_openWritingPipe(tRF24* rf24, const uint8_t *address) {
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

	RF24_write_register_chunk(rf24, RX_ADDR_P0, address, rf24->addr_width);
	RF24_write_register_chunk(rf24, TX_ADDR, address, rf24->addr_width);

	//const uint8_t max_payload_size = 32;
	//RF24_write_register(rf24, RX_PW_P0,rf24_min(rf24->payload_size,max_payload_size));
	RF24_write_register(rf24, RX_PW_P0, rf24->payload_size);
}

/****************************************************************************/
static const uint8_t child_pipe[] PROGMEM = {
RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };
static const uint8_t child_payload_size[] PROGMEM = {
RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };

/*
 void RF24_openReadingPipe(tRF24* rf24, uint8_t child, uint64_t address)
 {
 // If this is pipe 0, cache the address.  This is needed because
 // openWritingPipe() will overwrite the pipe 0 address, so
 // startListening() will have to restore it.
 if (child == 0){
 memcpy(rf24->pipe0_reading_address,&address,rf24->addr_width);
 }

 if (child <= 6)
 {
 // For pipes 2-5, only write the LSB
 if ( child < 2 )
 RF24_write_register(rf24, pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), rf24->addr_width);
 else
 RF24_write_register(rf24, pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

 RF24_write_register(rf24, pgm_read_byte(&child_payload_size[child]),rf24->payload_size);

 // Note it would be more efficient to set all of the bits for all open
 // pipes at once.  However, I thought it would make the calling code
 // more simple to do it this way.
 RF24_write_register(rf24, EN_RXADDR,RF24_read_register(rf24, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
 }
 }
 */

/****************************************************************************/
void RF24_setAddressWidth(tRF24* rf24, uint8_t a_width) {

	if (a_width -= 2) {
		RF24_write_register(rf24, SETUP_AW, a_width % 4);
		rf24->addr_width = (a_width % 4) + 2;
	}

}

/****************************************************************************/

void RF24_openReadingPipe(tRF24* rf24, uint8_t child, const uint8_t *address) {
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0) {
		memcpy(rf24->pipe0_reading_address, address, rf24->addr_width);
	}
	if (child <= 6) {
		// For pipes 2-5, only write the LSB
		if (child < 2) {
			RF24_write_register_chunk(rf24, pgm_read_byte(&child_pipe[child]), address,
					rf24->addr_width);
		} else {
			RF24_write_register_chunk(rf24, pgm_read_byte(&child_pipe[child]), address, 1);
		}
		RF24_write_register(rf24, pgm_read_byte(&child_payload_size[child]),
				rf24->payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		RF24_write_register(rf24, EN_RXADDR,
				RF24_read_register(rf24,
						EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

	}
}


/****************************************************************************/
bool RF24_isValid(tRF24* rf24) {
	return rf24->ce_pin != 0xff && rf24->csn_pin != 0xff;
}
/****************************************************************************/


/****************************************************************************/

void RF24_closeReadingPipe(tRF24* rf24, uint8_t pipe) {
	RF24_write_register(rf24, EN_RXADDR,
			RF24_read_register(rf24,
					EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void RF24_toggle_features(tRF24* rf24) {

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	_SPI.transfer( ACTIVATE );
	_SPI.transfer( 0x73 );
	RF24_csn(rf24, HIGH);
#elif defined (RF24_DUE)
	_SPI.transfer(rf24->csn_pin, ACTIVATE, SPI_CONTINUE );
	_SPI.transfer(rf24->csn_pin, 0x73 );
#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	SPItransfer(rf24->ssi_base,  ACTIVATE);
	SPItransfer(rf24->ssi_base, 0x73);
	RF24_csn(rf24, HIGH);

#else
	RF24_csn(rf24, LOW);
	_SPI.transfer( ACTIVATE);
	_SPI.transfer(0x73);
	RF24_csn(rf24, HIGH);
#endif
}

/****************************************************************************/

void RF24_enableDynamicPayloads(tRF24* rf24) {
	// Enable dynamic payload throughout the system

	//RF24_toggle_features(rf24);
	RF24_write_register(rf24, FEATURE, RF24_read_register(rf24, FEATURE) | _BV(EN_DPL));

	IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",RF24_read_register(rf24, FEATURE)));

	// Enable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
	RF24_write_register(rf24, DYNPD,
			RF24_read_register(rf24,
					DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

	rf24->dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24_enableAckPayload(tRF24* rf24) {
	//
	// enable ack payload and dynamic payload features
	//

	//RF24_toggle_features(rf24);
	RF24_write_register(rf24, FEATURE,
			RF24_read_register(rf24, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

	IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",RF24_read_register(rf24, FEATURE)));

	//
	// Enable dynamic payload on pipes 0 & 1
	//

	RF24_write_register(rf24, DYNPD, RF24_read_register(rf24, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
	rf24->dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24_enableDynamicAck(tRF24* rf24) {
	//
	// enable dynamic ack features
	//
	//RF24_toggle_features(rf24);
	RF24_write_register(rf24, FEATURE, RF24_read_register(rf24, FEATURE) | _BV(EN_DYN_ACK));

	IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",RF24_read_register(rf24, FEATURE)));

}

/****************************************************************************/

void RF24_writeAckPayload(tRF24* rf24, uint8_t pipe, const void* buf,
		uint8_t len) {
	const uint8_t
	//* current = reinterpret_cast<const uint8_t*>(buf);
	* current = (const uint8_t*)(buf);

	uint8_t data_len = rf24_min(len, 32);

#if defined (RF24_LINUX)
	RF24_csn(rf24, LOW);
	uint8_t * ptx = rf24->spi_txbuff;
	uint8_t size = data_len + 1; // Add register value to transmit buffer
	*ptx++ = W_ACK_PAYLOAD | ( pipe & 0b111 );
	while ( data_len-- ) {
		*ptx++ = *current++;
	}

	_SPI.transfern( (char *) rf24->spi_txbuff, size);
	RF24_csn(rf24, HIGH);
#elif defined (RF24_DUE)
	_SPI.transfer(rf24->csn_pin, W_ACK_PAYLOAD | ( pipe & 0b111 ), SPI_CONTINUE);
	while ( data_len-- > 1 ) {
		_SPI.transfer(rf24->csn_pin,*current++, SPI_CONTINUE);
	}
	_SPI.transfer(rf24->csn_pin,*current++);

#elif defined (TI_ARM)
	RF24_csn(rf24, LOW);
	SPItransfer(rf24->ssi_base, W_ACK_PAYLOAD | (pipe & 0b111));

	while (data_len--)
		SPItransfer(rf24->ssi_base, *current++);

	RF24_csn(rf24, HIGH);

#else
	RF24_csn(rf24, LOW);
	_SPI.transfer(W_ACK_PAYLOAD | (pipe & 0b111));

	while (data_len--)
		_SPI.transfer(*current++);

	RF24_csn(rf24, HIGH);

#endif
}

/****************************************************************************/

bool RF24_isAckPayloadAvailable(tRF24* rf24) {
	return !(RF24_read_register(rf24, FIFO_STATUS) & _BV(RX_EMPTY));
}

/****************************************************************************/

bool RF24_isPVariant(tRF24* rf24) {
	return rf24->p_variant;
}

/****************************************************************************/

void RF24_setAutoAck(tRF24* rf24, bool enable) {
	if (enable)
		RF24_write_register(rf24, EN_AA, 0b111111);
	else
		RF24_write_register(rf24, EN_AA, 0);
}

/****************************************************************************/

void RF24_setAutoAck_pipe(tRF24* rf24, uint8_t pipe, bool enable) {
	if (pipe <= 6) {
		uint8_t en_aa = RF24_read_register(rf24, EN_AA);
		if (enable) {
			en_aa |= _BV(pipe);
		} else {
			en_aa &= ~_BV(pipe);
		}
		RF24_write_register(rf24, EN_AA, en_aa);
	}
}

/****************************************************************************/

bool RF24_testCarrier(tRF24* rf24) {
	return (RF24_read_register(rf24, CD) & 1);
}

/****************************************************************************/

bool RF24_testRPD(tRF24* rf24) {
	return (RF24_read_register(rf24, RPD) & 1);
}

/****************************************************************************/

void RF24_setPALevel(tRF24* rf24, uint8_t level) {

	uint8_t setup = RF24_read_register(rf24, RF_SETUP) & 0b11111000;

	if (level > 3) {  						// If invalid level, go to max PA
		level = (RF24_PA_MAX << 1) + 1;	// +1 to support the SI24R1 chip extra bit
	} else {
		level = (level << 1) + 1;	 		// Else set level as requested
	}

	RF24_write_register(rf24,  RF_SETUP, setup |= level);	// Write it to the chip
}

/****************************************************************************/

uint8_t RF24_getPALevel(tRF24* rf24) {

	return (RF24_read_register(rf24, RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

bool RF24_setDataRate(tRF24* rf24, rf24_datarate_e speed) {
	bool result = false;
	uint8_t setup = RF24_read_register(rf24, RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

#if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__) || defined(__TI_ARM__)
	rf24->txRxDelay=250;
#else //16Mhz Arduino
	rf24->txRxDelay = 85;
#endif
	if (speed == RF24_250KBPS) {
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= _BV(RF_DR_LOW);
#if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__) || defined(__TI_ARM__)
		rf24->txRxDelay=450;
#else //16Mhz Arduino
		rf24->txRxDelay = 155;
#endif
	} else {
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if (speed == RF24_2MBPS) {
			setup |= _BV(RF_DR_HIGH);
#if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__) || defined(__TI_ARM__)
			rf24->txRxDelay=190;
#else //16Mhz Arduino
			rf24->txRxDelay = 65;
#endif
		}
	}
	RF24_write_register(rf24, RF_SETUP, setup);

	// Verify our result
	if (RF24_read_register(rf24, RF_SETUP) == setup) {
		result = true;
	}
	return result;
}

/****************************************************************************/

rf24_datarate_e RF24_getDataRate(tRF24* rf24) {
	rf24_datarate_e result;
	uint8_t dr = RF24_read_register(rf24, RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	if (dr == _BV(RF_DR_LOW)) {
		// '10' = 250KBPS
		result = RF24_250KBPS;
	} else if (dr == _BV(RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

/****************************************************************************/

void RF24_setCRCLength(tRF24* rf24, rf24_crclength_e length) {
	uint8_t config = RF24_read_register(rf24, CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC));

	// switch uses RAM (evil!)
	if (length == RF24_CRC_DISABLED) {
		// Do nothing, we turned it off above.
	} else if (length == RF24_CRC_8) {
		config |= _BV(EN_CRC);
	} else {
		config |= _BV(EN_CRC);
		config |= _BV(CRCO);
	}
	RF24_write_register(rf24,  CONFIG, config);
}

/****************************************************************************/

rf24_crclength_e RF24_getCRCLength(tRF24* rf24) {
	rf24_crclength_e result = RF24_CRC_DISABLED;

	uint8_t config = RF24_read_register(rf24, CONFIG) & ( _BV(CRCO) | _BV(EN_CRC));
	uint8_t AA = RF24_read_register(rf24, EN_AA);

	if (config & _BV(EN_CRC) || AA) {
		if (config & _BV(CRCO))
			result = RF24_CRC_16;
		else
			result = RF24_CRC_8;
	}

	return result;
}

/****************************************************************************/

void RF24_disableCRC(tRF24* rf24) {
	uint8_t disable = RF24_read_register(rf24, CONFIG) & ~_BV(EN_CRC);
	RF24_write_register(rf24,  CONFIG, disable);
}

/****************************************************************************/
void RF24_setRetries(tRF24* rf24, uint8_t delay, uint8_t count) {
	RF24_write_register(rf24, SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

//ATTiny support code pulled in from https://github.com/jscrane/RF24

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
// see http://gammon.com.au/spi
#	define DI   0  // D0, pin 5  Data In
#	define DO   1  // D1, pin 6  Data Out (this is *not* MOSI)
#	define USCK 2  // D2, pin 7  Universal Serial Interface clock
#	define SS   3  // D3, pin 2  Slave Select
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
// these depend on the core used (check pins_arduino.h)
// this is for jeelabs' one (based on google-code core)
#	define DI   4   // PA6
#	define DO   5   // PA5
#	define USCK 6   // PA4
#	define SS   3   // PA7
#endif

#if defined(RF24_TINY)

void SPIClass_begin(tRF24* rf24) {

	digitalWrite(SS, HIGH);
	pinMode(USCK, OUTPUT);
	pinMode(DO, OUTPUT);
	pinMode(SS, OUTPUT);
	pinMode(DI, INPUT);
	USICR = _BV(USIWM0);

}

byte SPIClass_transfer(tRF24* rf24, byte b) {

	USIDR = b;
	USISR = _BV(USIOIF);
	do
	USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
	while ((USISR & _BV(USIOIF)) == 0);
	return USIDR;

}

void SPIClass_end(tRF24* rf24) {}
void SPIClass_setDataMode(tRF24* rf24, uint8_t mode) {}
void SPIClass_setBitOrder(tRF24* rf24, uint8_t bitOrder) {}
void SPIClass_setClockDivider(tRF24* rf24, uint8_t rate) {}

#endif
