#include <util/delay.h>
#include <avr/interrupt.h>
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

	// disable ADC and I2C
	PRR0 = (1 << PRTWI) | (1 << PRADC);
	// disable USART1
	PRR1 |= (1 << PRUSART1);
}

#define DISP_DC		(1 << PD4)
#define DISP_NRST	(1 << PD7)
#define DISP_NCS	(1 << PD6)

#define SPI_MOSI	(1 << PB2)
#define SPI_MISO	(1 << PB3)
#define SPI_CLK		(1 << PB1)

void spi_init(void) {
	DDRB |= SPI_MOSI | SPI_CLK;
	SPCR = (1 << SPE) | (1 << MSTR);
	SPSR |= (1 << SPI2X);
}

#define SPI_WRITE_B(d) {		\
	SPDR = (d);			\
	while(!(SPSR & (1 << SPIF)));	\
}

#define DISP_CS_ASSERT		{ PORTD &= ~DISP_NCS; }
#define DISP_CS_DEASSERT	{ PORTD |= DISP_NCS; }

void disp_cmd(uint8_t cmd) {
	DISP_CS_ASSERT;
	PORTD &= ~DISP_DC;
	SPI_WRITE_B(cmd);
	DISP_CS_DEASSERT;
	PORTD |= DISP_DC;
}

void disp_data(uint8_t data) {
	DISP_CS_ASSERT;
	SPI_WRITE_B(data);
	DISP_CS_DEASSERT;
}

uint8_t disp_init_seq[] = {
	0xae,			// off
	0xa8, 0x3f,		// Multiplex ratio
	0xd3, 0x00,		// Display offset
	0x40,			// Start line
	0xa1,			// segment remap
	0xc8,			// COM output scan dir
	0x81, 0xcf,		// contrast
	0xa4,			// entire display on
	0xa6,			// 0xa6 - normal 0xa7 - invert color
	0xd5, 0xf0,		// Display clock rate div
	0x8d, 0x14,		// Charge pump
	0x2e,			// Deactivate scroll
	0x20, 0x01,		// Memory addressing mode 0 - H; 1 - V; 2 - Page;
	0xda, 0x12,		// COM HW conf
	0xd9, 0xf1,		// Pre-charge Period
	0xdb, 0x40,		// VCOMH Deselect level
	0xaf,			// on
	0x21, 0x00, 0x7f,	// Col start/end
	0x22, 0x00, 0x07,	// Page start/end
};

void disp_init(void) {
	spi_init();
	DDRD |= DISP_DC | DISP_NRST | DISP_NCS;
	PORTD &= ~(DISP_DC | DISP_NRST | DISP_NCS);
	_delay_us(5);
	PORTD |= DISP_NRST;

	for (int i = 0; i < sizeof(disp_init_seq) / sizeof(disp_init_seq[0]); i++)
		disp_cmd(disp_init_seq[i]);

	DISP_CS_ASSERT;
	for (int i = 0; i < 1024; i++)
		SPI_WRITE_B(0x00);
	DISP_CS_DEASSERT;

	DISP_CS_ASSERT;
	for (int i = 0; i < 768; i++)
		SPI_WRITE_B(0xAA);
	DISP_CS_DEASSERT;
}

#define TICKS		60
#define TIMER_TICK	(16000000 / 1024 / TICKS)

void ticks_init(void) {
	cli();
	TCNT1 = 0;
	OCR1A = TIMER_TICK;
	TCCR1A = 0x0;
	TCCR1B = (1 << WGM12) | (1 << CS12)| (1 << CS10);
	TIMSK1 = (1 << OCIE1A);  
	TIMSK1 = (1 << OCIE1A);
	sei();
}

uint8_t tick_cnt = 0;

ISR(TIMER1_COMPA_vect) {
	if (++tick_cnt == TICKS) {
		LED_TOGGLE(LED_ALL);
		tick_cnt = 0;
	}
}
