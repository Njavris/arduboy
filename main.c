#include <util/delay.h>

#include "io.h"


int main(void) {
	io_init();
	disp_init();
	sound_init();
	ticks_init();
	LED_SET(LED_GREEN);

	while (1);
	return 0;
}
