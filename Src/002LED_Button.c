/*
 * 001_LEDToggle.c
 *
 *  Created on: May 26, 2022
 *      Author: jordanrainey
 */

//#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

void delay(uint32_t delayAmt) {
	for (uint32_t i = 0; i < delayAmt; i++);
}

int main(void) {

	GPIO_Handle_t gpioLED, gpioButton;

	gpioLED.pGPIOx = GPIOD; // uses the macros
	gpioLED.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioLED.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLED.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpioLED.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioLED.GPIO_pinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;

	GPIO_PeriClockControl(gpioLED.pGPIOx, ENABLE);

	GPIO_Init(&gpioLED);

	gpioButton.pGPIOx = GPIOA; // uses the macros
	gpioButton.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	gpioButton.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpioButton.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	//don't need PUPD because there is already one with VDD and GND.
	GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);

	GPIO_Init(&gpioButton);

	while (1) {
		if (GPIO_ReadFromInputPin(gpioButton.pGPIOx, 0) == HIGH) {
			delay(20000);	//this is to prevent button impedence. 
			GPIO_ToggleOutputPin(gpioLED.pGPIOx, GPIO_PIN_12);

		}

	}
	return 0;
}
