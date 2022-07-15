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

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#define myAddr	0x61 // 97b10. Certain addresses are already reserved.
#define ARDUINO_SLAVE_ADDR	0x68

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

	// I2C IRQ Configurations, for events (EV) and errors (ER)
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE); // the assocaited IRQ number is not enabled and will be delivered to the processor
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE); // the assocaited IRQ number is not enabled and will be delivered to the processor
	// no configuration of the priority because we don't have multiple interrupts.

	uint8_t rcv_buf[32];

	uint8_t someData[] = "we are testing I2C protocol\n";
	uint8_t commandCode;
	uint8_t len;

	while (1) {

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay(50000);

		commandCode = 0x51;

		/* in the MasterSendData we need to wait until it is ready since the interrupt version returns the state */
		while (I2C_MasterSendDataIT(&I2C1Handle,&commandCode,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); // send over the command code. Don't continue until I2C is ready again.
		while (I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); // arduino slave sends back the len info we need.



		commandCode = 0x52;
		while (I2C_MasterSendDataIT(&I2C1Handle,&commandCode,1,ARDUINO_SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); // send over the new command code.
		/* Wait until the recieve data is complete */
		while (I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,ARDUINO_SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY);
		/* Wait until RX Completes. */

		rxComplt = RESET; // why are we coding it like this? Why doesn't this cause an infite loop?

		while (rxComplt != SET); // the applicationEventCallback will set this. Or at least should.

		rcv_buf[len+1] = '\0'; // this is for semihosting. Usage of printf statements.

		printf("Data : %s",rcv_buf);

		rxComplt = RESET;


	}
	//wait for button press


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

	// note: error condidtions in section 27.3.4 of the Data Manual
	if (AppEv == I2C_EV_TX_CMPLT) {
		printf("Tx is completed\n");
	}
	else if (AppEv == I2C_EV_RX_CMPLT) {
		printf("Rx is completed\n");
		rxComplt = SET;
	}
	else if (AppEv == I2C_ERROR_AF) {
		printf("Error: ACK failure\n");
		// Master Mode: slave fails to send ACK for the byte. A stop or a repreated start
		// Slave Mode: Lines are then released by hardware.
		I2C_CloseSendData(pI2CHandle); // resets the handle structure
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx); // to release the BUS.
		while(1); // hang in infitie loop. Application specific.
	}

}
