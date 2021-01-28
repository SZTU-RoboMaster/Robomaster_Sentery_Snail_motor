#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "struct_typedef.h"

#define RED    0xFFFF0000
#define GREEN  0xFF00FF00
#define BLUE   0xFF0000FF
#define YELLOW 0xFFFFFF00

void led_init(void);
void led_show(uint32_t aRGB);
	
#endif
