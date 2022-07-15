///*
// * 10_i2c_master_tx_testing.c
// *
// *  Created on: Jun 18, 2022
// *      Author: jordanrainey
// *
// *      Udemy: Sections 208-212, I2C Rx communication
// */
//
//#include<stdio.h>
//#include<string.h>
//#include"stm32f407xx.h"
//#include"stm32f407xx_gpio_driver.h"
//#include"stm32f407xx_spi_driver.h"
//#include "stm32f407xx_i2c_driver.h"
//
//#define myAddr	0x61 // 97b10. Certain addresses are already reserved.
//#define ARDUINO_SLAVE_ADDR	0x68
//
//void delay(uint32_t delayAmt) {
//	for (uint32_t x; x < delayAmt; x++);
//}
//
//static I2C_Handle_t I2C1Handle;
//
//// some data to send.
//
//void I2C1_GPIOInits(void) {
//
//	/*
//	 * PB6 -> SCL
//	 * PB9 -> SDA
//	 * Both use AF4 and internal pull up resistor
//	 */
//
//	GPIO_Handle_t I2C1Pins;
//
//	I2C1Pins.pGPIOx = GPIOB;
//	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
//	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
//	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Internal Pullup
//	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM; // max is in Mhz and I2C max is 0.40 Mhz
//
//	/* PB6 = SCL */
//	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
//	GPIO_Init(&I2C1Pins);
//
//	/* PB7 = SDA */
//	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
//	GPIO_Init(&I2C1Pins);
//
//	// Init turns on the peripheral control for us. No need to implement here.
//
//}
//
//void I2C1_Inits(void) {
//
//
//
//	I2C1Handle.pI2Cx = I2C1;
//	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
//	I2C1Handle.I2C_Config.I2C_DeviceAddress = myAddr; // 0x61
//	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
//	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM2K; // standard mode < 100khz.
//
//	I2C_Init(&I2C1Handle);
//
//}
//
//void GPIO_ButtonInit(void) {
//
//	GPIO_Handle_t gpioButton;
//
//		gpioButton.pGPIOx = GPIOA; // uses the macros
//		gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
//		gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
//		gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
//		gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//		GPIO_Init(&gpioButton);
//		GPIO_PeriClockControl(gpioButton.pGPIOx, ENABLE);
//
//}
//
//int main(void) {
//
//	uint8_t commandCode;
//	uint8_t len; //placeholder for length.
//	char someData[] = "hello world!\n";
//	//i2c pin inits
//	I2C1_GPIOInits();
//
//	I2C1_Inits();
//	I2C_PeripheralControl(I2C1, ENABLE);
//
//	// does this work? Yes it does!!!
//	I2C1Handle.pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
//
//	GPIO_ButtonInit();
//	uint8_t recieveBuff[32];
//
//	while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) {
//		// do nothing.
//	}
//	delay(5000);
//
//	/* In a for-loop so that I can better view it on the logic analyzer */
//	for (int i = 0; i < 200; i++) {
//		I2C_MasterSendData(&I2C1Handle, (uint8_t*)someData, strlen((char*)someData), ARDUINO_SLAVE_ADDR, 0);
//		delay(50); // get a big enough gap inbetween the send datas on the logic analyzer.
//	}
//
//	//wait for button press
//
//
//	for (;;);
//
//	return 0;
//}

/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

#define MY_ADDR 0x61; // a non-reserved address.

#define SLAVE_ADDR  0x68

void delay(uint32_t value)
{
	for(uint32_t i = 0 ; i < value ; i ++);
}

I2C_Handle_t I2C1Handle;

//some data
//uint8_t some_data[] = "We are testing I2C master Tx\n";

uint8_t some_data[] = {0x12, 0x23, 0x45, 0x56, 0xa3};

/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;

	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	// below doesn't really matter what we choose, we aren't in fast mode (FM)
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_ST;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}


int main(void)
{

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin( GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay(200000);

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle,some_data,sizeof(some_data)/sizeof(some_data[0]),SLAVE_ADDR,0);
	}

}

