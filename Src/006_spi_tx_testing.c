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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		//generates sclk of 8mhz
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_SW;		// software slave management enabled for NSS pin. The master chooses the slave device.

	SPI_Init(&SPI2handle);

	//SPI2handle.pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);

}

int main(void) {

	uint8_t myNums[] = {12, 200, 13};

	SPI2_GPIOInits();
	SPI2_Inits();

	// SPI2_SSI enable. Make NSS signal interanally HIGH and avoids MODF error

	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	while (1) {

		SPI_SendDataIT(SPI2, myNums, 3); //3 numbers to send

	}

	// need to make sure the communication is trutly finished before disabling.

	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)) {
		// do nothing until it isn't busy anymore. // this prevents an error makes sure the Tx buffer is empty before closing communicaiton and disabling.
	}
	SPI_PeripheralControl(SPI2, DISABLE);



	return 0;
}
