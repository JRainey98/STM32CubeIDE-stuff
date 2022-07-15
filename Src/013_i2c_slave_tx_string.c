/*
 * 10_i2c_master_tx_testing.c
 *
 *  Created on: Jun 18, 2022
 *      Author: jordanrainey
 *      This relates to lecture 234 on I2C slave programming.
 */

#include<stdio.h>
#include<string.h>
#include"stm32f407xx.h"
#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#define ARDUINO_SLAVE_ADDR		0x68
#define MY_ADDR					ARDUINO_SLAVE_ADDR;

uint8_t commandCode;
uint8_t tx_buf[32] = "STM32 slave mode testing\n";


void delay(uint16_t delayAmt) {
	for (uint16_t x; x < delayAmt; x++);
}

static I2C_Handle_t I2C1Handle;

// Flag variable
uint8_t rxComplt = RESET;

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
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Internal Pullup, very important for I2C if not using external pullup.
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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; // 0x68, slave address
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

	// I2C IRQ Configurations, for events (EV) and errors (ER)
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE); // the assocaited IRQ number is not enabled and will be delivered to the processor
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE); // the assocaited IRQ number is not enabled and will be delivered to the processor

	// we need to enable control bits from CR2 to enable and generate the interrupts.

	/* enables the interrupts for the slave in the CR2 register */
	I2C_SlaveEnableOrDisableCallbackEvents(I2C1, ENABLE);

	/*
	 * Study: I'm guessing that because interrupts are enabled that we can alter the callback api to send the data once it is detected.
	 */


	uint8_t someData[] = "we are testing I2C protocol\n";

	uint8_t len;





	for (;;);

	return 0;
}
/*
 * Once the interrupts are triggered the handler functions are executed.
 */

void I2C1_EV_IRQHandler(void) {

	I2C_EV_IRQHandling(&I2C1Handle);

}

void I2C1_ER_IRQHandler(void) {

	I2C_ER_IRQHandling(&I2C1Handle);
}

/* check for I2C application event macros.
 * Uses semihosting for printf functionality.
 * */



void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	// memory for thse variablers won't be in the stack. Will be in the global space.
	// memory for the space won't be deallocated.
	// they are private to this function.
	static uint8_t commandCode = 0x0;
	static uint32_t count = 0;
	static uint32_t w_ptr;

	if (AppEv == I2C_EV_DATA_REQ) {
		// Master wants data, slave needs to send it.
		// send the length information to the master.
		if (commandCode == 0x51) {
			// 0x51 means send the length information
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buf)); // char is 8bytes, typecast is fine
		}
		else if (commandCode == 0x52) {
			// 0x52 means send the contents of tx_buf.
			I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buf[count++]);

		}
	} // above is executed after below. If commandCode matches then we send the info
	else if (AppEv == I2C_EV_DATA_RCV) {
		// data is waiting for the slave to read.

		commandCode = I2C_SlaveRecieveData(pI2CHandle->pI2Cx);

	}
	else if (AppEv == I2C_ERROR_AF) { // AF = "ACK failure". note: I thinking on the master end auot ACK will be disabled when len == 2.
		// Indicates end of slave Tx.
		// only occurs in slave mode.
		//Master sent the NACK so slave should understand that master doesn't need more data

		// if current active code is 0x52 then don't invalidate.

		commandCode = 0xFF; // NACK
		count = 0; // reset count varible.
	}
	else if (AppEv == I2C_EV_STOP) {
		// This happens only during slave reception
		// Master has ended the I2C communication with the slave.
		// Slave does nothing.

	}

}
