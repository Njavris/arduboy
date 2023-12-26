#include <util/delay.h>

#include "io.h"


int main(void) {
	io_init();
	disp_init();

	while (1) {
		if (SW_UP) {
    			LED_SET(LED_ALL);
        		_delay_ms(10);
    			LED_UNSET(LED_ALL);
		}
        	_delay_ms(10);
	}
	return 0;
}
