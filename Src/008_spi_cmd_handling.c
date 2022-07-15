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

/*
 * Command code for arduino
 */

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

/*
 * arduino analog pins
 */

#define ANALOG_PIN_0	0
#define ANALOG_PIN_1	1
#define ANALOG_PIN_2	2
#define ANALOG_PIN_3	3
#define ANALOG_PIN_4	4

/*
 * arduino LED
 */

#define LED_PIN_9	9


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

uint8_t SPI_VerifyResponse(uint8_t ackByte) {

	if (ackByte == 0xF5)
		return 1; //ack
	else
		return 0; //nack
}

int main(void) {

	/* before recieve data, must send dummy byte to get ACK or NACK back before SPI communication begins */

	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;
	uint8_t ackByte;
	uint8_t args[2];
	//uint8_t myNums[] = {12, 200, 13};

	GPIO_ButtonInit();

	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);

	//SPI2_SSI enable. Make NSS signal interanally HIGH and avoids MODF error

	//SPI_SSIConfig(SPI2, ENABLE); //Not required for Hardware mode slave management. But will use SSOE To enable the NSS output

	while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) {
			// do nothing until the button is pressed. button will work.
	}

	//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_12); 	// just to test make sure button works.

	delay(500000); // prevents button debouncing

	/* enable the SPI peripheral */
	SPI_PeripheralControl(SPI2, ENABLE);



	/* command #1 CMD_LED_CTRL <pin no(1) value(1)*/
	uint8_t commandCode = COMMAND_LED_CTRL;

	//send command
	SPI_SendData(SPI2, &commandCode, 1);
	SPI_RecieveData(SPI2, &dummyRead, 1); // do dummy read to clear off the RXNE. We recieve garbage values first time.

	SPI_SendData(SPI2, &dummyWrite, 1); // we are sending 1 byte of data so len = 1.
	SPI_RecieveData(SPI2, &ackByte, 1); // recieves ACK/NACK

	if ( SPI_VerifyResponse(ackByte) ) {

`1		//
		args[0] = LED_PIN_9;
		args[1] = LED_ON;
		SPI_SendData(SPI2, args, 2); // sending two bytes of data.

	}

	//end of COMMAND_LED_CTRL

	while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) {
		// do nothing until the button is pressed. button will work.
	}

	commandCode = COMMAND_SENSOR_READ;

	SPI_SendData(SPI2, &commandCode, 1);
	SPI_RecieveData(SPI2, &dummyRead, 1); // do dummy read to clear off the RXNE. We recieve garbage values here

	//send some dummy bits to fetch ACK/NACK from slave
	SPI_SendData(SPI2, &dummyWrite, 1); //
	SPI_RecieveData(SPI2, &ackByte, 1); // recieves ACK/NACK

	if ( SPI_VerifyResponse(ackByte) ) {

		args[0] = ANALOG_PIN_0;

		//send arguments
		SPI_SendData(SPI2, args, 1); // sending two bytes of data.
		// insert some delay so slave can ready with the data. It has to do ADC conversion which is not
		// not immediately ready.
		delay(20000);

		// we transmitted so we have to do a dummyread.
		SPI_RecieveData(SPI2, &dummyRead, 1); // do dummy read to clear off the RXNE. Recieve buffer must be empty before we continue. We clear it by reading from recieve buffer.

		//send some dummy bits fetch response from slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		uint8_t analogRead;
		SPI_RecieveData(SPI2, &analogRead, 1); // read analog data.

	}

	// end of COMMAND_SENSOR_READ

	// confirm that the SPI is not busy
	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)) {
		// do nothing until it isn't busy anymore. // this prevents an error makes sure the Tx buffer is empty before closing communicaiton and disabling.
	}

#warning "IGNORE: the rest of the source code is on their github"

	SPI_PeripheralControl(SPI2, DISABLE);



	return 0;
}
