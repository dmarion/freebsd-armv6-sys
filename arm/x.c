#include <stdint.h>
void early_a(uint8_t x);

void aaa() {
	early_a(1);
	early_a(2);
	early_a(3);
}