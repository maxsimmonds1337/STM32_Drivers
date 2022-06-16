# STM32_Drivers

[Reference Manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=106&zoom=100,89,117)

[Nucleo FR411RE Pinout](https://os.mbed.com/platforms/ST-Nucleo-F411RE/)

## Timer Setup

### Timers for interrupts

Sometimes we want an interrupt to occur at a specific frequency. Hardware timers can be used to do this. In this example, I set up an interrupt to fire at 100Khz using timer 3.

By default, the system clock source is the HSI. For the stm32 F411RE, this is a 16 MHz internal RC.
This then goes to the "AHB Prescaler" - by default this is set to 1 (no clock division)
It then goes through another prescaler, the "APB prescaler"
It then goes into an AND gate, which is how the clock is enabled

To set up the clock to interrupt at 100Khz, the following must be done:

Set the AHB prescaler if needed, by default there is no division
Enable the peripheral clock 

```C
(RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable the timer)
```

Search the reference manual for "TIM3EN" to find which clock is used 
Always best to make sure the clock is off

```C
TIM3->CR1 &= ~(TIM_CR1_CEN); //ensure clock is off
```
Set the timers prescaler. By default, the internal HSI clock is used which runs at 16Mhz, so the prescaler divides this down.

```C
TIM3->PSC = 16 - 1;  // incoming clock is 16mhz, so this will scale it to 1mhz
```

You need to subtract one to get the actual division, because a PSC value of 1 divides by 2.
Next set the auto reload register. This is the value at which the counter will reset (so it's max value)

```C
TIM3->ARR = 10; // 10 * 1us = 10us interrupt (100khz)
```

A value of 10, with a clock speed of 1Mhz, gives a 10us interrupt (100Khz)
Now set the update interrupt enable flag in the DMA/Interupt enable register (DIER)

```C
TIM3->DIER |= TIM_DIER_UIE; //enable timer interrupts
```
Now, the interrupt priority needs to be set (1 is the highest), and the interrupt needs to be enabled:

```C 
NVIC_SetPriority(TIM3_IRQn, 0x01);
NVIC_EnableIRQ(TIM3_IRQn); // enable the IRQ
```

Finally, turn on the clock for the timer, so that is starts counting:

```C
TIM3->CR1 |= TIM_CR1_CEN; //enable the counter
```

Once this is all done, you need to make sure you have a function that overwrites the weakly defined interrupt. This is defined in the startup_xxxxxx.s file. Place a function in your code without the weak link. Within this, you need to make sure to clear the timer  Update interrupt flag in status register!

```C
//interrupt service routine for the timer
void TIM3_IRQHandler(void) {

TIM3->SR = 0;		// clear the status reg

}
```

### Phase Locked Loop (PLL)

![image](https://user-images.githubusercontent.com/58208872/173787454-10bbb6bc-025a-4f46-9742-dcd64e5b005b.png)


The PPL on the STM32 can take several inputs, the HSI or HSE. For this application, I've used the HSI, a 16MHz clock. The process for increasing the PLL clock is as follows:

1) Check which speed you would like to obtain, I have chosen 50Mhz.
2) ![image](https://user-images.githubusercontent.com/58208872/173788120-89ac3783-7c0a-490c-a66a-0da2ec4722ee.png)
3) Look at the table in 2), and see how many wait states (WS) are required for the flash memory reading. In the case of a 50Mhz and a voltage range of 2.7 V - 3.6 V, then a single WS is needed.
4) Set the wait state register
5) Configure the M,N, and P registers (these have rules that must be followed)
6) Turn on the PLL clock, and wait for it to come online
7) Change the switch to allow the PLL clock through, wait for it to come online
8) execute SystemCoreClockUpdate()

In order to determine what values of N,M, and P to use, the following must be done (found on page 104 of the ref manual) :

$$ f_{VcoClock} = f_{PllClockIn} \cdot \frac{PLLN}{PLLM} $$

$$ f_{PllGeneralClockOutput} = \frac{f_{VcoClock}}{PLLP} $$

Since I won't be using the USB OTG clock, I will ignore it's equation.

The following must be adheared to when setting the PLL:

$$ 100MHz \ge f_{VcoClock} \le 432MHz $$
$$ 50 \ge PLLN \le 432 $$
$$ 1MHz \ge f_{VcoInputClock} \le 2MHz $$ (2MHz recommended to reduce jitter)
$$ 2 \ge PLLM \le 63 $$
$$ PLLP = 2,4,6,8 $$

Therefore, in order to get 50MHz out (with an input of 16MHz), the following numbers are proposed:

* PLLM = 2
* PLLN = 50
* PLLP = 8

That gives a VCO clock input of 16MHz / 2 = 8MHz, not the recommended frequency but it works. An N value of 50 brings the VCO clock output to 100MHz, within the required range. Finally, a P value of 8 drops the PLL clock output to 50MHz, within the range requried for a single wait state.

These parameters are written to the PLL configuration registers within the Reset and Clock Control register (RCC), namley, RCC->PLLCFGR.  Programmatically, the above looks like:

```C
//First step is to set the wait states, for anything less than 64MHz, but greater than 30, 1 wait state is needed
FLASH->ACR |= FLASH_ACR_LATENCY_1WS; // this adds one Wait State (WS)
	
RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // reset the register
RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); // reset the register
RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); // reset the register

RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_3; // sets PLLM to 8 (2^3 = 8)
RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_1 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5; // sets PLLN to 50 (2^1 OR 2^4 OR 2^5, 2+16+32)
RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_1; // sets PLLP to 2 (2^1)
```

Finally, the PLL clock now needs to be turned on (it cannot be enabled until the PLL configuration is complete). This is done in the RCC->CR register. The PLLON bit is set, and the PLLRDY bit is polled until high:

```C
//turn on PLL
RCC->CR |= RCC_CR_PLLON; 
while (!(RCC->CR & RCC_CR_PLLRDY)) ;  // wait for the pll ready flag
```

Ideally, there should be some error handler within the loop, to count the number of itterations and break out if nothing happens after a while.

Lastly, the switch (labelled SW in the clock tree diagram) needs to be setup up so that the SYSCLOCK becomes the PLL. This is done by setting the SW bits within the RCC configuration register (RCC->CFGR). Again, there should be code to wait for this switch to settle, which is done by polling the SW status bits:

```C
//Set the SW state to PLL
	
RCC->CFGR |= RCC_CFGR_SW_PLL; //sets the SW mux to use PLL
while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) ; // wait for the PLL to be ready
```	

Lastly, everytime the system clock is changed, the following function must be called:

```C
	SystemCoreClockUpdate();  // needs to be run once the clocks are updated
```

### Outputting Clocks to GPIO

Whilst setting up the PLL clock, it was important to see if what I was setting up was indeed working. There are two pins on the STM32f411RE that can be used to output clocks; PA8 and PC9, or MCO1 and MCO2 respectively. To find what pins have this alternate function, look at the datasheet and search for "MCO". Table 9 (Alternate function mapping) in the datasheet for this IC shows that MC0 is on pin PA8 (or D7 on the Nucleo board) and is AF 0000. Likewise, MCO2 is PC9 (or pin 1 of CN10) and is also AF 0000.

The process for setting this up is also follows:

![image](https://user-images.githubusercontent.com/58208872/173802299-b7c32e2d-09e8-476a-ba68-2cc63ec1cef8.png)


1) Set which clock you would like outputting on the MCO pin. Refer to the above image for which pins can be used for which pin. The maximum speed that can be outputted to a IO is 100MHz (the max I/O speed)
2) Set the prescaler should you need to, this is useful when the scope that you are using to probe it (or device you'd like to run) doesn't have the bandwidth. In my example, I use a prescaler of 5 to get a 10MHz output.
3) Turn on the GPIO clock for the relevant port
4) Set the particular pin (PA8 or PC9) as an alternate function, using the MODER register (GPIOx->MODER)
5) Set the function number (in this case, 0000) in the alternate function register.

Page 106 of the reference manual shows what bits the MCOx registers should be set to for different clocks. They are:

| MCOx | Register | Clock Source |
|------|----------|--------------|
|  0   | 00       | System clock |
|  0   | 01       | PLLI2S       |
|  0   | 10       | HSE          |
|  0   | 11       | PLL          |
|      |          |              |
|  1   | 00       | HSI 	 |
|  1   | 01       | LSE		 |
|  1   | 10       | HSE          |
|  1   | 11       | PLL          |
 

Programmatically, this looks like the below:

```C
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
```

### Timers for PWM outputs

The steps are as follows:

![image](https://user-images.githubusercontent.com/58208872/174078586-f59747f1-4a6c-4218-8de0-7fd2f8093e14.png)

* Enable the timer's clock in the APB1ENR - we know that it's on the APB1 bus because of the block diagram (shown above) in the datasheet
* Decide which pin/channel you want to output the PWM to, check the alternate mapping table in the datasheet. In this example, I'm using TIM1 (advanced timer with PWM capabilities) and channel 1, which means I need pin PA8.
* Enable the clock of the GPIO bank that you're using (GPIOA)
* Disable the clock (t's off by default, but just to be sure)
* Set the autoreload register of the timer, this value is chosen knowing the input clock speed (this comes from looking at the clock tree diagram *NOTE: to change the APB prescaler, change the PPREx bits in the RCC->CFGR)
* The autoreload register is set by $$ ARR = \frac{f_{in}}{f_{cl}}. In our case, with a 50MHz clock and a switching frequency of 100K, ARR = 500. 
![image](https://user-images.githubusercontent.com/58208872/174089878-b5411cdf-4d47-4500-8fe0-b80b7c5df2dd.png)
* Referring to the above image, it can be seen that set the output capture compare mode of the timer (this mode is for outputting things, rather than taking an input, like clock counting). To enable this, 110 is written to the OC1M bits in the TIMx_CCMR1 register
* Set the capture compare register (TIM1->CCR1) to a value less than the ARR - this sets the duty cycle
* Now the capture/compare mode needs to be enabled, this is done in the TIM1_CCER. It can be set by TIM1->CCER |= TIM_CCER_CC1E.
* Now set the output pin (PA8) to alternate function
* set the alternate fuction register to the appropriate number (in our case, it's 0001)
* For some reason, TIM1 and and 8 have a Main Output Enable (MOE) pin, this is set in the Break and Deadtime Register BDTR



### Timers for IO output

** This is not complete, just didn't want to lose something I had already written **

* Enable the GPIO bank for the output (if using one) 
  * ```RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable the port A clock on GPIO```
  * Use the datasheet (not the ref manual) to find the "alternate function mapping" table to see what are the alternate functions
  * Set the GPIO to alternate function
  * ```GPIOA->MODER |= 0x2 << (2*6); // sets port PA6 as alternate function```
  * The MODER register is 2 bits wide, so I take the port numner (6) multiply it by 2, and then shift the alternate function code (0x2) by that much
* Make the link between the timer and the GPIO. This is done by writting to the alternate register 
  * ```GPIOA->AFR[0] |= 0010 << (4*6);	// this sets the alternate function mode "0010" (AF2) into port 6```
