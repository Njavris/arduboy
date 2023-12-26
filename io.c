#include <util/delay.h>
#include "io.h"

void io_init(void) {
	// LEDs
	DDRB |= LED_ALL;
	PORTB |= LED_ALL;

	// Speaker
	DDRC |= SPK_P | SPK_N;
	PORTC |= SPK_P;
	PORTC &= ~SPK_N;

	// Controls
	DDRF &= ~BIT_DIR;
	PORTF |= BIT_DIR;
	DDRE ^= ~BIT_A;
	PORTE |= BIT_A;
	DDRB &= ~BIT_B;
	PORTB |= BIT_B;

}

#define DISP_DC		(1 << PD4)
#define DISP_NRST	(1 << PD7)
#define DISP_NCS	(1 << PD6)

#define SPI_MOSI	(1 << PB2)
#define SPI_MISO	(1 << PB3)
#define SPI_CLK		(1 << PB1)

void spi_init(void) {
	DDRB |= SPI_MOSI | SPI_CLK;
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

#define SPI_WRITE_B(d) {			\
	do {					\
		SPDR = (d);			\
		while(!(SPSR & (1 << SPIF)));	\
	} while(0);				\
}

void disp_cmd(uint8_t cmd) {
	PORTD &= ~DISP_NCS;
	PORTD &= ~DISP_DC;
	SPI_WRITE_B(cmd);
	PORTD |= DISP_NCS;
}

void disp_data(uint8_t data) {
	PORTD &= ~DISP_NCS;
	PORTD |= DISP_DC;
	SPI_WRITE_B(data);
	PORTD |= DISP_NCS;
}

uint8_t disp_init_seq[] = {
	0xae,		// off
	0x81, 0xff,	// contrast
	0xa6,		// 0xa6 - normal 0xa7 - invert color
	0x2e,		// Deactivate scroll
	0x20, 0x00,	// Memory addressing mode 0 - H; 1 - V; 2 - Page;
	0xa1,		// segment remap
	0xa8, 0x3f,	// Multiplex ratio 
	0xc0,		// COM output scan dir
	0xd3, 0x00,	// Display offset
	0xda, 0x12,	// COM HW conf
	0xd5, 0x80,
//	0xd4, 0x80,	// Display clock rate div
	0xd9, 0x22,	// Pre-charge Period
	0xdb, 0x20,	// VCOMH Deselect level
	0x8d, 0x14,	// Charge pump
	0x40, 0xc8,	// Start line
	0xa4,		// entire display on
	0xaf,		// on
};

void disp_init(void) {
	spi_init();
	DDRD |= DISP_DC | DISP_NRST | DISP_NCS;
	PORTD &= ~(DISP_DC | DISP_NRST | DISP_NCS);
	_delay_us(5);
	PORTD |= DISP_NRST;

	for (int i = 0; i < sizeof(disp_init_seq) /
					sizeof(disp_init_seq[0]); i++)
		disp_cmd(disp_init_seq[i]);

	for(int i = 0; i < 1024; i++)
		disp_data(0x00);
	disp_data(0xf);
	disp_data(0xf0);
	disp_data(0xf);
	disp_data(0xf0);
	disp_data(0xf);
	disp_data(0xf0);
	disp_data(0xf);
	disp_data(0xf0);
	disp_data(0xaa);
	for(int i = 0; i < 256; i++)
		disp_data(0xff);
	//for (int i = 0; i < 8; i++)
	//	disp_data(1 << i);
}
