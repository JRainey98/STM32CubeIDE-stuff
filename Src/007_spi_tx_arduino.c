/*
 * 006_spi_tx_testing.c
 *
 *  Created on: May 30, 2022
 *      Author: jordanrainey
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

void delay(uint32_t delayAmt) {
	for (uint32_t i = 0; i < delayAmt; i++);
}

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;		//not required, so can put whatever
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; 		// this doesn't matter too much

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {



	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		//generates sclk of 2mhz
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_HW;		// hardware slave management enabled for NSS pin. We must configure SSOE

	SPI_Init(&SPI2handle);

	//SPI2handle.pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);

}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t gpioLED, gpioButton;

		gpioLED.pGPIOx = GPIOD; // uses the macros
		gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
		gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
		gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;

		GPIO_PeriClockControl(gpioLED.pGPIOx, ENABLE);

		GPIO_Init(&gpioLED);

		gpioButton.pGPIOx = GPIOA; // uses the macros
		gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
		gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
		gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	//don't need PUPD because there is already one with VDD and GND.
		GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);


}

int main(void) {

	uint8_t myNums[] = {12, 200, 13};

	GPIO_ButtonInit();

	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);

	//SPI2_SSI enable. Make NSS signal interanally HIGH and avoids MODF error

	//SPI_SSIConfig(SPI2, ENABLE); //Not required for Hardware mode slave management. But will use SSOE To enable the NSS output



	// dont' transmit until button is pressed
	while (1) {

		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) {
				// do nothing until the button is pressed. button will work.
		}
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_12); 	// just to test make sure button works.
		delay(500000); // prevents button debouncing
		SPI_PeripheralControl(SPI2, ENABLE);

		//#error "Look at #154. Adruino Uno requires length info be sent first. Add this then debug"

		SPI_SendData(SPI2, myNums, 3); // will keep sending as long as the peripheral can

		// need to make sure the communication is trutly finished before disabling.
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)) {
			// do nothing until it isn't busy anymore. // this prevents an error makes sure the Tx buffer is empty before closing communicaiton and disabling.
		}

		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
