#include <util/delay.h>

#include "io.h"


int main(void) {
	io_init();
	disp_init();
	ticks_init();

	while (1) {
	}
	return 0;
}
