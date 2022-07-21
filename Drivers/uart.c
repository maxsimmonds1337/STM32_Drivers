#include "uart.h"
#include "stm32f4xx.h" // this includes the specific stm32f4R11e board because I defined in in here (line 80)
#include <string.h>

extern RxBuffer; // this is a global var, that is written to in the ISR


//setup uart 2

void init_uart2(void) {

    // pin configuration
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable the GPIO clock
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // sets PA2 as alternate function (2^1 = 2 = 10b)
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_2 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL3_2 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_0; // sets ports PA2 and PA3 to AF 7 (b111) which enables the USART2
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR2_0) | GPIO_OSPEEDER_OSPEEDR3_1 | GPIO_OSPEEDER_OSPEEDR3_0;

    //UART config
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // turn on the clock
    USART2->CR1 = 0x00; //clear the config reg.
    USART2->BRR |= (2 << USART_BRR_DIV_Fraction_Pos) | (27 << USART_BRR_DIV_Mantissa_Pos); // this sets up the baud rate clock. This is done by f_APB1 / (16 * BR) = 50M/(16*115200) = 27.128. 27 is the menstissa, 0.128 * 16 = 2, becomes the frctional part
    USART2->CR1 |= USART_CR1_UE; // enable the uart clocks
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // set the transmit enable bit

	USART2->CR1 |= USART_CR1_RXNEIE;

	NVIC_SetPriority(USART2_IRQn, 0x02); // priority 2, less than the phasor
	NVIC_EnableIRQ(USART2_IRQn); // enable the IRQ

}

void UART2_SendChar (char c) {
	/*********** STEPS FOLLOWED *************
	
	1. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
		 for each data to be transmitted in case of single buffer.
	2. After writing the last data into the USART_DR register, wait until TC=1. This indicates
		 that the transmission of the last frame is complete. This is required for instance when
		 the USART is disabled or enters the Halt mode to avoid corrupting the last transmission.
	
	****************************************/

	USART2->DR = c;   // LOad the Data
	while (!(USART2->SR & USART_SR_TC));  // Wait for TC to SET.. This indicates that the data has been transmitted
}

void SendData(char *data) {

	

	int lengthData = strlen(data); //get length of array

	for(int i=0; i<lengthData; i++) {
		UART2_SendChar(data[i]); // send the data
	}

}

char UART2_GetChar (void) {
		/*********** STEPS FOLLOWED *************
	
	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit
	llllkkkl
	****************************************/
	char Temp;
	
	while (!(USART2->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	Temp = USART2->DR;  // Read the data.

	return Temp; 

}

void printIntro(void) {

	char header[] = "\r\n"
					"\r\n"
					"/*********************************************/\r\n"
					"/****** Small Scale Cooler Breadboard FW *****/\r\n"
					"/******     Written by Max Simmonds      *****/\r\n"
					"/******               V1.0.0             *****/\r\n"
					"/*********************************************/\r\n"
					"\r\n"
					"\r\n"
					"Type h for help/list of commands"
					"\r\n"
					"-> ";

	SendData(header);

}

void printHelp(void) {

	char header[] = "\r\n"
					"List of commands:\r\n"
					"\r\n"
					"'Gain X' where 'X' is a number between 0.0 - 1.0\r\n"
					"'h' displays this help message"
					"\r\n"
					"-> ";

	SendData(header);

}

// int strlen(char *myString) {

// 	int i = 0;

// 	while(myString[i] != '\0'){
// 		i++;
// 	}
// }