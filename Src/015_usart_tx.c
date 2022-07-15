/*
 * 015_usart_tx.c
 *
 *  Created on: Jun 25, 2022
 *      Author: jordanrainey
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_gpio_driver.h"

uint8_t msg[5] = {23,45,56,78,250};

USART_Handle_t usart2_Handle;

void delay(uint32_t delayAmt) {
	for (uint32_t x = 0; x < delayAmt; x++);
}

void USART2_Init(void) {

	usart2_Handle.pUSARTx = USART2;
	usart2_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_PeriClockControl(USART2, ENABLE);

	USART_Init(&usart2_Handle);

}

void USART2_GPIOInit(void) {

	GPIO_Handle_t pUSART_GPIO;

	// configure the Tx pin, PA2
	pUSART_GPIO.pGPIOx = GPIOA;
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7;
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	// Tx line is idle HIGH, START is a falling edge, therefore we use internal pullup.
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;

	GPIO_Init(&pUSART_GPIO);

	// configure the Rx pin. PA3
	pUSART_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&pUSART_GPIO);

}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t gpioButton;

	// PA0 is the user button the board
	gpioButton.pGPIOx = GPIOA; // uses the macros
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	//don't need PUPD because there is already one with VDD and GND.
	GPIO_Init(&gpioButton);

}



int main(void) {

	USART2_GPIOInit(); // this also enables the peripheral inside GPIO_Init();

	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	// configures the PA0 button for triggering
	GPIO_ButtonInit();

	// wait for button to be triggered.
	while ( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
	delay(20000);

	while (1) {
		USART_SendData(&usart2_Handle, (uint8_t*)msg, sizeof(msg)/sizeof(msg[0]));
		delay(10000);

	}


	for (;;);
	return 0;
}
