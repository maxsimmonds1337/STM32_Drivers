#include "NCO.h"
#include "timer.h"
#include "stm32f411xe.h"                  // Device header
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_pwr_ex.h"

/*

__PIN ASSIGNMENT__

https://os.mbed.com/platforms/ST-Nucleo-F411RE/


D12 is PA6
CN10 pin1 is PC9
D7 is PA8


*/

/*

#define OFFSET 20 // this is the timer counter compare value that gives our zero point (mid rail)

extern unsigned int sine_full_5[1024]; // LUT
extern unsigned int sine_full_4[1024]; // LUT
extern unsigned int sine_full_3[1024]; // LUT
extern unsigned int sine_full_2[1024]; // LUT
extern unsigned int sine_full_1[1024]; // LUT

extern unsigned int sine_index; // holds the index for the sinewave lookup
extern unsigned int DC; // holds the DC
extern unsigned int temp; // just used for debug
extern short GAIN; // gain which is used to adjust the sinewave
extern short Mode; // this sets the midpoint of the sinewave

*/

/*typedef long T_INT32;
typedef short T_INT16;
typedef unsigned short T_UINT16;
typedef long long T_INT64;

typedef struct {
	T_INT32 real;
	T_INT32 imag;
} T_COMPLEX32;
*/

extern unsigned int DC; // holds the DC
extern unsigned int temp; // just used for debug
extern short GAIN; // gain which is used to adjust the sinewave
extern short Mode; // this sets the midpoint of the sinewave

unsigned short counter = 0; //keeps track of how many times the NCO is called

T_COMPLEX32 phasor = {.real = -2147483647, .imag = 0}; // this holds the phasor 2147483647
T_COMPLEX32 PHASE_ADVANCE_1024 = {.real=2147443220, .imag=13176712};	// this holds the phasor that's used to advance the phasor

//T_COMPLEX32 Complex_MUT(T_COMPLEX32 a, T_COMPLEX32 b);

// this will setup the phase lock loop
void PLL(void) {

		//First step is to set the wait states, for anything less than 64MHz, but greater than 30, 1 wait state is needed
		FLASH->ACR |= FLASH_ACR_LATENCY_1WS; // this adds one Wait State (WS)
		
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // reset the register
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // reset the register
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); // reset the register

		RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_1; // sets PLLM to 8 (2^3 = 8)
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_1 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5; // sets PLLN to 50 (2^1 OR 2^4 OR 2^5, 2+16+32)
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_Msk; // sets PLLP to 2 (2^1)


/* 
 	//This section turns on the GPIO PC9 to be able to see the PPL clock
	RCC->CFGR |= RCC_CFGR_MCO2; // outputs the system clock
	RCC->CFGR |= RCC_CFGR_MCO2PRE_Msk; //sets the prescale to 5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable the GPIO C clock
	GPIOC->MODER |= GPIO_MODER_MODE9_1;// Set pin 9 (PC9, or MCO2) to alternate function. This is for outputting a clock source (2^1 = 2 (0b10))
	GPIOC->AFR[1] |= GPIO_AF0_MCO << GPIO_AFRH_AFSEL9_Pos; // set the pin to AF0, which is MCO (microcontroller output 1)
 
 	//This section turns on the GPIO PA8 to be able to see the PPL clock
	RCC->CFGR |= RCC_CFGR_MCO1_Msk; // outputs the PLL clock (enabled by default)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable the GPIO A clock
	GPIOA->MODER |= GPIO_MODER_MODE8_1;// Set pin 8 (PA8, or MCO1) to alternate function. This is for outputting a clock source (2^1 = 2 (0b10))
	GPIOA->AFR[1] |= GPIO_AF0_MCO << GPIO_AFRH_AFSEL8_Pos; // set the pin to AF0, which is MCO (microcontroller output 1)

	  */

	//turn on PLL
 	RCC->CR |= RCC_CR_PLLON; 
 	while (!(RCC->CR & RCC_CR_PLLRDY)) ;  // wait for the pll ready flag

	//Set the SW state to PLL
	
	RCC->CFGR |= RCC_CFGR_SW_PLL; //sets the SW mux to use PLL
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) ; // wait for the PLL to be ready
	

	SystemCoreClockUpdate();  // needs to be run once the clocks are updated


}

void start_TIM2() {
  RCC->APB2ENR |= 0x1; //RCC_APB2ENR_TIM1EN;
  TIM1->CR1 |= TIM_CR1_CEN;
}

uint16_t read_TIM_3() {
	return TIM3->CNT;
}

uint16_t read_TIM2() {
  return RCC->CR;
}

uint16_t read_flag() {
	return TIM1->SR; //
}

void timer_init(void) {
	
	// enable the TIM1 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable timer 1

	// enable GPIOs clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable the port A clock on GPIO
	TIM1->CR1 &= ~(TIM_CR1_CEN); //ensure clock is off

	// set the counter mode to up (default is up)
	//TIM12->CR1 |= 0x1 << 4; // set the dir of the counter to up
	
	//TIM12->PSC = 0x1; // load the prescaler with 1 (168MHz)
	
	// it's a 16bit timer (tim 8). this is needed for setting the period
	// tim_period = clock_f / pwm_f = (16M / 100k) - 1  = 159
 	 // set the clock to 100KHz
	
	//TIM1->DIER = TIM_DIER_UIE;
	
	//set the value in the output compare register (this is compared with the counter value)
	// CCR1 = ((TIM_Period + 1) * DutyCycle) / 100 - 1
	// = (1679 + 1 * 20) / (100 - 1) = 335
	/* Reset the Output Compare Mode Bits */
  //TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
  //TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM1->ARR = 500; // 500 in decimal gives 100k, or 10us delay
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;	// PWM mode (2^1 + 2^2 = 6, 110b = PWM)
	TIM1->CCR1 = TIM1->ARR/4; // 20% duty (capture compare register)
	// output compare stuff
	TIM1->CCER |= TIM_CCER_CC1E; // set the capture compare enable register, bit 1 for CC output enable

	// set up the GPIOs
	GPIOA->MODER |= GPIO_MODER_MODER8_1; //set PORTA8 as alternate function mode - 2^1 = 2
	GPIOA->AFR[1] = GPIO_AF1_TIM1 << GPIO_AFRH_AFSEL8_Pos; // set 0001 in the AF register, for pin 8 (sets PA8 as TIM1)
	
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->EGR |= TIM_EGR_UG; // update all the registers by triggering an update event
	
	TIM1->CR1 |= 0x1; // set clock en

}

void timer_init_3(void) {

	/**** TIM3 INIT ******/
	
	//set up the clocks, and GPIO registers
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable the timer
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable the port A clock on GPIO
	//GPIOA->MODER |= 0x2 << (2*6); // sets port PA6 as alternate function
	//GPIOA->AFR[0] |= 0010 << (4*6);	// this sets the alternate function mode "0010" (AF2) into port 6

	// set up the timer parameters
	TIM3->CR1 &= ~(TIM_CR1_CEN); //ensure clock is off
	TIM3->PSC = 5 - 1;  // incoming clock is 50mhz, so this will scale it to 1mhz
	TIM3->ARR = 100-1; // 100 * 0.1us = 10us interrupt (100khz)
	TIM3->DIER |= TIM_DIER_UIE; //enable timer interrupts


	NVIC_SetPriority(TIM3_IRQn, 0x01);
	NVIC_EnableIRQ(TIM3_IRQn); // enable the IRQ

	TIM3->CR1 |= TIM_CR1_CEN; //enable the counter

}

//interrupt service routine for the timer
void TIM3_IRQHandler(void) {

	if(counter >= 1023) {
		counter = 0;
		T_COMPLEX32 phasor = {.real = -2147483647, .imag = 0}; // reset the phasor, to stop drifting
	} else {
		counter++; //inc counter
	}

	signed long tmp = 0;

	// toggle on the pin, so we know we're in the ISR
	//GPIOA->ODR ^= 1 << 0x6;
	//GPIOA->ODR |= 1 << 0x6;
	

	phasor = Complex_MUT(phasor, PHASE_ADVANCE_1024);	// perform the complex MUT

	//GPIOA->ODR &= ~(1 << 0x6);


	tmp =(((phasor.real)>>9)*500>>23); //scale the real component by the full scale of the PWM reg
	
	DC = (int)tmp+250; // cast 

	TIM1->CCR1 = (DC); //update the capture control register with the phasor

	TIM3->SR = 0;		// clear the status reg


	// this might be used to reset the phasor at some point, not using for now
	/*
	if(sine_index == 1023) {
		sine_index=0;
	} else {
		sine_index++;// inc the sine wave counter
	}*/

	// turn of the io so we know when we're out of the ISR*/



	//GPIOA->ODR &= ~(0x01 << 6);

}
