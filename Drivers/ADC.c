#include "stm32f4xx.h"                  // Device header
#include "ADC.h"

// setup the ADC
void init_adc(void) {
	
	//onfigure the GPIO as input
	
	//PA1 is ADC123_in1 input by default so I'll leave it
	GPIOA->MODER |= 0x3 << 2; // reset the PA1 bit
	
	RCC->APB2ENR |= 1<<8; //en adc1 clock
	ADC1->SQR3 |= 0x1; // channel 1 for adc1
	ADC1->CR2 |= 1; //enable the ADC
	
}


uint16_t read_adc(void) {
	ADC1->CR2 |= 1<<30; //trigger a conversion
	while(!(ADC1->SR && ~(0x2) ));
	return ADC1->DR * v_scale;
	
}