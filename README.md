# STM32_Drivers

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
