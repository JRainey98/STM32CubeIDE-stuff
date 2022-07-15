/*
 * 005_Button_Interrupt.c
 *
 *  Created on: May 28, 2022
 *      Author: jordanrainey
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"


#define HIGH	1
#define LOW		0
#include<string.h>
int main(void) {

	GPIO_Handle_t gpioLED, gpioButton;

 	memset(&gpioLED, 0, sizeof(gpioLED));
	memset(&gpioButton, 0, sizeof(gpioButton));

	gpioLED.pGPIOx = GPIOD; // uses the macros

	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	GPIO_PeriClockControl(gpioLED.pGPIOx, ENABLE);
	GPIO_Init(&gpioLED);

	// gpioLED.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0;
	
	// configure the user button on the MCU
	gpioButton.pGPIOx = GPIOA; // uses the macros
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;		// interrupt falling trigger, pushing button connects to GND
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);
	GPIO_Init(&gpioButton);

	/* IRQ configuration */

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI9_5, NVIC_IRQ_PRIO_15);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI9_5, ENABLE);

	while(1);
	return 0;
}

void EXTI9_5_IRQHandler(void) {

	GPIO_IRQHandling(GPIO_PIN_5);		// clears the pending bit from the IRQ trigger.
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
	for (uint32_t i = 0; i < 500000; i++); // this is the delay

}

