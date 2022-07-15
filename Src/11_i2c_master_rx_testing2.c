/*
 * 10_i2c_master_tx_testing.c
 *
 *  Created on: Jun 18, 2022
 *      Author: jordanrainey
 */

#include<stdio.h>
#include<string.h>
#include"stm32f407xx.h"
#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#define myAddr	0x61 // 97b10. Certain addresses are already reserved.
#define ARDUINO_SLAVE_ADDR	0x68

void delay(uint32_t delayAmt) {
	for (uint32_t x; x < delayAmt; x++);
}

static I2C_Handle_t I2C1Handle;

// some data to send.

void I2C1_GPIOInits(void) {

	/*
	 * PB6 -> SCL
	 * PB9 -> SDA
	 * Both use AF4 and internal pull up resistor
	 */

	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Internal Pullup
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH; // max is in Mhz and I2C max is 0.40 Mhz

	/* PB6 = SCL */
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2C1Pins);

	/* PB9 = SDA */
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&I2C1Pins);

	// Init turns on the peripheral control for us. No need to implement here.

}

void I2C1_Inits(void) {



	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = myAddr; // 0x61
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_ST; // standard mode < 100khz.

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void) {

	GPIO_Handle_t gpioButton;

		gpioButton.pGPIOx = GPIOA; // uses the macros
		gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
		gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
		gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GPIO_Init(&gpioButton);
		GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);

}

int main(void) {

	//i2c pin inits
	I2C1_GPIOInits();

	I2C1_Inits();

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ACKControl(I2C1, ENABLE); // leave this here. ACK can only be configured after peripheral is enabled (PE = 1)

	GPIO_ButtonInit();

	uint8_t rcv_buf[32];

	uint8_t someData[] = "we are testing I2C protocol\n";
	uint8_t commandCode;
	uint32_t len;
	while (1) {

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay(50000);

		commandCode = 0x51;

		I2C_MasterSendData(&I2C1Handle,&commandCode,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle,&len,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR);

		commandCode = 0x52;
		I2C_MasterSendData(&I2C1Handle,&commandCode,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR);


		I2C_MasterReceiveData(&I2C1Handle,rcv_buf,len,ARDUINO_SLAVE_ADDR,I2C_DISABLE_SR);

		rcv_buf[len+1] = '\0';

				//printf("Data : %s",rcv_buf);


	}
	//wait for button press


	for (;;);

	return 0;
}
