// Library for the dac
#include "DAC.h"
#include "stm32f4xx.h"                  // Device header


// Initialise the DAC
void DAC_init(void) {
	
	//DACOUT1 is PA4
	//enable GPIB clock for PA
	RCC->AHB1ENR |= (0x1); // enable port A clock (for DAC1)
	GPIOA->MODER |= 0x3 << 4; // sets the PA4 as analogue output (for DAC)	
	
	RCC->APB1ENR |= (0x1 << 29); //enable the DAC clock
	
	DAC->CR |= 0x1; //enable dac1
	
}

// write to the DAC reg
void DAC_write8(char data) {
	
	// write to DAC_DHRx register
	DAC->DHR8R1 = data; //write 8bits to the DAC
	
}
