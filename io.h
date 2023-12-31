#ifndef __IO_H__
#define __IO_H__
#include <avr/io.h>


#define BIT(b)		(1 << (b))

#define LED_RED		BIT(PB6)
#define LED_GREEN	BIT(PB7)
#define LED_BLUE	BIT(PB5)
#define LED_ALL		(LED_RED | LED_GREEN | LED_BLUE)
#define LED_RX		BIT(PB0)
#define LED_TX		BIT(PD5)

#define LED_SET(led)	{ PORTB &= ~(led); }
#define LED_UNSET(led)	{ PORTB |= (led); } 
#define LED_TOGGLE(led) { PORTB ^= (led); } 

extern uint8_t controls;

#define CTRL_BIT_DOWN	0
#define CTRL_BIT_LEFT	1
#define CTRL_BIT_RIGHT	2
#define CTRL_BIT_UP	3
#define CTRL_BIT_A	4	
#define CTRL_BIT_B	5
#define CTRL_MASK_DOWN	BIT(CTRL_BIT_DOWN)
#define CTRL_MASK_LEFT	BIT(CTRL_BIT_LEFT)
#define CTRL_MASK_RIGHT	BIT(CTRL_BIT_RIGHT)
#define CTRL_MASK_UP	BIT(CTRL_BIT_UP)
#define CTRL_MASK_A	BIT(CTRL_BIT_A)
#define CTRL_MASK_B	BIT(CTRL_BIT_B)


void io_init(void);
void disp_init(void);
void ticks_init(void);

#endif
