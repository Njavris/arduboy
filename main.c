#include <avr/io.h>
#include <util/delay.h>

#define LED_RED		PB6
#define LED_GREEN	PB7
#define LED_BLUE	PB5

int main(void) {
    DDRD |= 1 << LED_GREEN;
    while (1) {
	PORTB ^= 1 << LED_GREEN;
        _delay_ms(1000);
    }
    return 0;
}
