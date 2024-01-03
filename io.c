#include <util/delay.h>
#include <avr/interrupt.h>
#include "io.h"

#define BIT_UP		(PINF7)
#define BIT_RIGHT	(PINF6)
#define BIT_LEFT	(PINF5)
#define BIT_DOWN	(PINF4)
#define DIR_MASK	(BIT(BIT_UP) | BIT(BIT_DOWN) | BIT(BIT_RIGHT) | BIT(BIT_LEFT))
#define BIT_A		(PINE6)
#define BIT_B		(PINB4)

uint8_t controls = 0;

void io_init(void) {
	// LEDs
	DDRB |= LED_ALL;
	PORTB |= LED_ALL;

	// Controls
	DDRF &= ~DIR_MASK;
	PORTF |= DIR_MASK;
	DDRE ^= ~BIT(BIT_A);
	PORTE |= BIT(BIT_A);
	DDRB &= ~BIT(BIT_B);
	PORTB |= BIT(BIT_B);

	// disable ADC and I2C
	PRR0 = BIT(PRTWI) | BIT(PRADC);
	// disable USART1
	PRR1 |= BIT(PRUSART1);
}

#define DISP_DC		BIT(PD4)
#define DISP_NRST	BIT(PD7)
#define DISP_NCS	BIT(PD6)

#define SPI_MOSI	BIT(PB2)
#define SPI_MISO	BIT(PB3)
#define SPI_CLK		BIT(PB1)

void spi_init(void) {
	DDRB |= SPI_MOSI | SPI_CLK;
	SPCR = BIT(SPE) | BIT(MSTR);
	SPSR |= BIT(SPI2X);
}

#define SPI_WRITE_B(d) {		\
	SPDR = (d);			\
	while(!(SPSR & BIT(SPIF)));	\
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
	0x20, 0x00,		// Memory addressing mode 0 - H; 1 - V; 2 - Page;
	0xda, 0x12,		// COM HW conf
	0xd9, 0xf1,		// Pre-charge Period
	0xdb, 0x40,		// VCOMH Deselect level
	0xaf,			// on
	0x21, 0x00, 0x7f,	// Col start/end
	0x22, 0x00, 0x07,	// Page start/end
};

static void disp_clear(void) {
	DISP_CS_ASSERT;
	for (int i = 0; i < 1024; i++)
		SPI_WRITE_B(0x00);
	DISP_CS_DEASSERT;
}

void disp_init(void) {
	spi_init();
	DDRD |= DISP_DC | DISP_NRST | DISP_NCS;
	PORTD &= ~(DISP_DC | DISP_NRST | DISP_NCS);
	_delay_us(5);
	PORTD |= DISP_NRST;

	for (int i = 0; i < sizeof(disp_init_seq) / sizeof(disp_init_seq[0]); i++)
		disp_cmd(disp_init_seq[i]);

	disp_clear();
}

#define TICKS		60
#define TIMER_TICK	(16000000 / 1024 / TICKS)

void ticks_init(void) {
	TCNT1 = 0;
	OCR1A = TIMER_TICK;
	TCCR1A = 0x0;
	TCCR1B = BIT(WGM12) | BIT(CS12)| BIT(CS10);
	TIMSK1 = BIT(OCIE1A);
	sei();
}

#define SPK_P		BIT(PC6)
#define SPK_N		BIT(PC7)

#define V1_PRESC	64
#define V2_PRESC	64
#define NOTE(p, f, l)	((((uint16_t)(16000000 / (p) / 2 / (f))) & 0xfff) | ((l) << 12)) 
#define VOICE1(f)	{ OCR3A = (f); TCNT3 = 0; }
#define VOICE2(f)	{ OCR4C = (f); TCNT4 = 0; }

/*
Voice 2 - 10 bit	0-1023
Voice 1 - 16 bit	0-65535
*/

void sound_init(void) {
#ifndef MUTE
	// OC4A OC3A
	DDRC |= SPK_P | SPK_N;
//	PORTC |= SPK_P | SPK_N; 

	TCCR3A = BIT(COM3A0);
	TCCR3B = BIT(WGM32) | BIT(CS31) | BIT(CS30);
	TCNT3 = 0;
	OCR3A = 0;

	TCCR4A = BIT(COM4A0);
	TCCR4B = BIT(PWM4X) | BIT(CS42) | BIT(CS41) | BIT(CS40);
	TCCR4D = BIT(WGM40);
	TCNT4 = 0;
	TC4H = 0;
	OCR4A = 0;
	OCR4C = 0;
#endif
}

#include "notes.h"

uint16_t score_v1[] =  {
	NOTE(V1_PRESC, D4, 4), NOTE(V1_PRESC, E4, 4), NOTE(V1_PRESC, F4, 4), NOTE(V1_PRESC, G4, 4),
	NOTE(V1_PRESC, A4, 4), NOTE(V1_PRESC, F4, 4), NOTE(V1_PRESC, D4, 4), NOTE(V1_PRESC, F4, 4),
	NOTE(V1_PRESC, A4, 2), NOTE(V1_PRESC, B4, 2),
	NOTE(V1_PRESC, A4, 4), NOTE(V1_PRESC, F4, 4), NOTE(V1_PRESC, D4, 2),
	NOTE(V1_PRESC, A4, 4), NOTE(V1_PRESC, A4, 4), NOTE(V1_PRESC, G4, 4), NOTE(V1_PRESC, E4, 4),
	NOTE(V1_PRESC, F4, 4), NOTE(V1_PRESC, F4, 4), NOTE(V1_PRESC, D4, 4), NOTE(V1_PRESC, E4, 4),
	NOTE(V1_PRESC, F4, 2), NOTE(V1_PRESC, E4, 2),
	NOTE(V1_PRESC, D4, 1),
};

uint16_t score_v2[] =  {
	NOTE(V2_PRESC, A0, 1),
	NOTE(V2_PRESC, A0, 1),
	NOTE(V2_PRESC, A0, 2), NOTE(V2_PRESC, D1, 2),
	NOTE(V2_PRESC, A0, 1),
	NOTE(V2_PRESC, E1, 1),
	NOTE(V2_PRESC, A0, 1),
	NOTE(V2_PRESC, E1, 1),
	NOTE(V2_PRESC, A0, 1),
};

static void controls_update(void) {
	// read inputs
	controls = (PINF & DIR_MASK) >> 4; // DIR
	controls |= ((PINE & BIT(BIT_A)) >> BIT_A) << CTRL_BIT_A;
	controls |= ((PINB & BIT(BIT_B)) >> BIT_B) << CTRL_BIT_B;
	controls = ~controls;
}

static uint8_t nv1 = 0, nv2 = 0, cnt1 = 0, cnt2 = 0;
static void sound_tick(void) {
	if (!cnt1) {
		if (nv1 >= sizeof(score_v1)/sizeof(score_v1[0]))
			nv1 = 0;
		uint16_t val = score_v1[nv1++];
		VOICE1(val & 0xfff);
		cnt1 = TICKS / ((val >> 12) & 0xf);
	}
	if (!cnt2) {
		if (nv2 >= sizeof(score_v2)/sizeof(score_v2[0]))
			nv2 = 0;
		uint16_t val = score_v2[nv2++];
		VOICE2(val & 0xfff);
		cnt2 = TICKS / ((val >> 12) & 0xf);
	}
	cnt1 --;
	cnt2 --;
}

static uint16_t ticks = 0;
ISR(TIMER1_COMPA_vect) {
	controls_update();
	if (!(ticks % 4)) 
		sound_tick();
	ticks ++;
}
