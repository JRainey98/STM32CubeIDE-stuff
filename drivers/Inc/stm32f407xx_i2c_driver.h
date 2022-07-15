/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 25, 2022
 *      Author: jordanrainey
 */

#ifndef STM32F407XX_I2C_DRIVER_H_
#define STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration Structure for I2Cx peripheral
 */

typedef struct {

	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;	//slave device address. Mentioned by the user. No macros6`
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;;

}I2C_Config_t;

/*
 * Handle Structure
 */
 
typedef struct {

	I2Cx_Registers_t *pI2Cx;
	I2C_Config_t I2C_Config;
	//below are the modifications for the interrupt based operations
	uint8_t 	*pTxBuffer; 	// to store the app. Tx buffer address
	uint8_t 	*pRxBuffer; 	// To store the app. Rx buffer address
	uint32_t 	TxLen;
	uint32_t 	RxLen;
	/* @study: no separate states because I2C can only XOR transmit or receive data */
	uint8_t		TxRxState;		//To store communication state values are @TxRxState_Values
	uint8_t 	DevAddr;		// To store slave/device address
	uint32_t 	RxSize;
	uint8_t 	Sr;				// to store the repeated start value macros

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed values
 * @note: there are 4 modes, standard (100Khz), fast (400khz)
 */
#define I2C_SCL_SPEED_ST	100000	//Standard mode, 100kHz
#define I2C_SCL_SPEED_FM4K	400000 	// fast mode, 400kHz
#define I2C_SCL_SPEED_FM2K	200000	// fast mode, 200kHz

/*
 * @I2C_AckControl
 * @note: enables or disables automatic ACK
 * This bit is set and cleared byu software and cleared by hardware when PE == 0.
 */
#define I2C_ACK_ENABLE		ENABLE
#define I2C_ACK_DISABLE		DISABLE

/*
 * @I2C_FMDutyCycle values
 * Data Manual Section 27.6.8
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * @I2C_TRISE register macros
 * Data Manual Section 27.6.9
 * @note: after expression evaluation, result is in microseconds
 */

#define TRISE_ST_MAX		(1/1000000U)	//Standard Mode (ST) TRISE Max is 1000ns = 1 microsecond. ONLY USE AS DENOMINATOR. 1000e-9 == 1/1000e9 = 1 microsecond
#define TRISE_FM_MAX		(1/300000U)		//Fast mode (FM) TRISE Max according to the I2C specification.

/*
 *	@I2C_SR1_FLAG_VALUES
 * 	@Note: Used for checking flag status of the bit position in conjunction with the '&' operator
 * 	@note: Used in function I2C_GetFlagStatus() defined in stm32f407_i2c_driver.c file.
 */

#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
// Bit [5] reserve
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR			(1 << I2C_SR1_PECERR)
//bit [13] reserved
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT		(1 << I2C_SR1_SMBALERT)

/*
 * Enable or Disable repeated starts in I2C communication
 */
#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET

/*
 * I2C application event macros
 * @I2C_ApplicationEventCallback()
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2

/*
 * I2C_ER_IRQHandling() function macros
 */

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

/*
 * I2C application events macros
 * Used with I2C_ApplicationEventCallback()
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7

#define I2C_EV_DATA_REQ         8 // data request
#define I2C_EV_DATA_RCV         9 // data receive

/*
 * I2C_ExecuteAddressPhase Macros
 */

#define WRITE	0
#define READ	1

/*
 * I2C application states
 * @TxRxState_Values
 */

#define I2C_READY			0
#define	I2C_BUSY_IN_RX		1	//	busy in receiving
#define	I2C_BUSY_IN_TX		2	// 	busy in transmitting

/*START*********APIs Supported By This Driver******************************/

void I2C_Init(I2C_Handle_t *pI2CHandle);

void I2C_DeInit(I2Cx_Registers_t *pI2Cx);    // Resets all peripherals of specified port using RCC reset 

/*
 * Send Data Function, polling-based
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);

/*
 * 	Recieve data function, polling-based
 * 	Returns the applicaiton state @TxRxState_Values
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Send Data Function, interrupt based
 * Returns state @TxRxState_Values
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);

/*
 * 	Recieve data function, interrupt based
 * 	Returns state @TxRxState_Values
 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);


/*
 * peripheral clock setup 
 *
 */

void I2C_PeriClockControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS); //ENorDIS = "Enable or disable"

/*
 *  IRQ Configuration and ISR handling  
 */

void I2C_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENorDI);
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle); // event-based handling
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle); // error-based handling


/*
 *  I2C_SR flag getting function.
 *  @note: Data Manual Section 28.5.3
 */

/*
 *  Other APIs supported by this device
 */

void I2C_PeripheralControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS);

/*
 *  Enable/Disable of automatic ACKing.
 *  Data Manual Section 27.6.1
 */

void I2C_ExecuteAddressPhase(I2Cx_Registers_t *pI2Cx, uint8_t slaveAddr, uint8_t readOrWrite);

void I2C_ACKControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS);



void I2C_SSOEConfig(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS);
uint8_t I2C_GetFlagStatus(I2Cx_Registers_t *pI2Cx, uint32_t flagName);	// also has status register

void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

void I2C_GenerateStopCondition(I2Cx_Registers_t *pI2Cx);


/*
 * Slave specific functions
 */
void I2C_SlaveEnableOrDisableCallbackEvents(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS);
uint8_t I2C_SlaveRecieveData(I2Cx_Registers_t *pI2Cx);
void I2C_SlaveSendData(I2Cx_Registers_t *pI2Cx, uint8_t data);



/*END*********APIs Supported By This Driver******************************/


#endif /* stm32f407xx_i2c_driver.h */
