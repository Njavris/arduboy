#include "io.h"


void io_init(void) {
	// LEDS
	DDRB |= LED_ALL;
	PORTB |= LED_ALL;
	// SPEAKER
	DDRC |= SPK_P | SPK_N;
	PORTC |= SPK_P;
	PORTC &= ~SPK_N;
	// SWITCHES
	DDRF &= ~BIT_DIR;
	PORTF |= BIT_DIR;
	DDRE ^= ~BIT_A;
	PORTE |= BIT_A;
	DDRB &= ~BIT_B;
	PORTB |= BIT_B;
	// DISPLAY
	DDRD |= DISP_DC | DISP_RST | DISP_CS;
	PORTD |= DISP_DC | DISP_RST | DISP_CS;
	PORTD &= ~(DISP_DC | DISP_RST);
}
