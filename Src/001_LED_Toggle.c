/*
 * 001_LEDToggle.c
 *
 *  Created on: May 26, 2022
 *      Author: jordanrainey
 */

//#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void) {

	GPIO_Handle_t gpioLED;

	gpioLED.pGPIOx = GPIOD; // uses the macros
	gpioLED.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioLED.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	gpioLED.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpioLED.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpioLED.GPIO_pinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;

	GPIO_PeriClockControl(gpioLED.pGPIOx, ENABLE);

	// gpioLED.GPIO_pinConfig.GPIO_PinAltFunMode = GPIO_AF_0;
	GPIO_Init(&gpioLED);
	while (1) {

		GPIO_ToggleOutputPin(gpioLED.pGPIOx, GPIO_PIN_12);
		delay();

	}
	return 0;
}
