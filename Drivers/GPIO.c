#include "stm32f4xx.h"                  // Device header
#include "GPIO.h"

void init_gpio(void) {
	
	//set up PA0 as output
	GPIOA->MODER |= (0x01); 
	
}