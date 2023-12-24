#include <avr/io.h>


#define LED_RED		(1 << PB6)
#define LED_GREEN	(1 << PB7)
#define LED_BLUE	(1 << PB5)
#define LED_ALL		(LED_RED | LED_GREEN | LED_BLUE)
#define LED_RX		(1 << PB0)
#define LED_TX		(1 << PD5)

#define BIT(b)		(1 << (b))

#define LED_SET(led)	{ PORTB &= ~(led); }
#define LED_UNSET(led)	{ PORTB |= (led); } 
#define LED_TOGGLE(led) { PORTB ^= (led); } 

#define BIT_UP		(1 << PINF7)
#define BIT_RIGHT	(1 << PINF6)
#define BIT_LEFT	(1 << PINF5)
#define BIT_DOWN	(1 << PINF4)
#define BIT_DIR		(BIT_UP | BIT_DOWN | BIT_LEFT | BIT_RIGHT)
#define BIT_A		(1 << PINE6)
#define BIT_B		(1 << PINB4)

#define SW_UP		(!(PINF & BIT_UP))
#define SW_DOWN		(!(PINF & BIT_DOWN))
#define SW_LEFT		(!(PINF & BIT_LEFT))
#define SW_RIGHT	(!(PINF & BIT_RIGHT))
#define SW_A_D		(!(PINE & BIT_A))
#define SW_B_D		(!(PINB & BIT_B))

#define SPK_P		(1 << PC6)
#define SPK_N		(1 << PC7)
#define SPK		{ PORTC ^= SPK_P | SPK_N; }

#define DISP_DC		(1 << PD4)
#define DISP_RST	(1 << PD7)
#define DISP_CS		(1 << PD6)

void io_init(void);
