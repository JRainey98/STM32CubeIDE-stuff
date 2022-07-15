/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 25, 2022
 *      Author: jordanrainey
 */
  
#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx.h"

uint16_t AHB_Prescaler_Values[8] = {2,4,8,16, 64,128,256,512};
uint8_t APB1_Prescaler_Values[4] = {2,4,8,16};

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


/***************************************************************************************
 * @fn          - I2C_SlaveEnableOrDisableCallbackEvents()
 *
 * @brief       - Enables or disables the CR2 bits to generate interrupts.
 *
 * @param       - pointer to I2Cx register. pass in argument as pHandle->pI2Cx form
 *
 * @return      - none
 *
 */
void I2C_SlaveEnableOrDisableCallbackEvents(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS) {

	if (ENorDIS == ENABLE)
	{

		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);

	}
	else
	{
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
	}

}

/***************************************************************************************
 * @fn          - I2C_SlaveRecieveData()
 *
 * @brief       - When device in slave mode receives data it reads from DR (data register)
 *
 * @param       - pointer to I2C_Handle_t structure. To access the DR
 *
 * @return      - the byte read from the data register
 *
 */
uint8_t I2C_SlaveRecieveData(I2Cx_Registers_t *pI2Cx) {

	return (pI2Cx->I2C_DR);

}
void I2C_SlaveSendData(I2Cx_Registers_t *pI2Cx, uint8_t data) {

	pI2Cx->I2C_DR = data;

}

/***************************************************************************************
 * @fn          - I2C_PeriClockControl 
 *
 * @brief       - Enables the clock for the peripherals of the I2C
 *
 * @param       - Pointer to I2Cx_Registers_t struct
 *
 * @return      - none
 *
 * @note        - I2Cx register reset macros in stm32f407xx.h 
 */

void I2C_PeriClockControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDI) {

	if (ENorDI == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}	
	}
	else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}	
	}
}

/***************************************************************************************
 * @fn          - I2C_PeripheralControl()
 * 
 * @brief       - Enables or disables the I2C peripheral using the PE bit in control register 1
 *
 * @param       - Pointer to SPIx registers. 
 * @param       - ENABLE or DISABLE macros.
 *
 * @return      - none
 *
 */

void I2C_PeripheralControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS) {

    if (ENorDIS == ENABLE) {
        pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);   // enables the peripheral
    }
    else {
        pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);  // enables the peripheral
    }

}

/***************************************************************************************
 * @fn          - I2C_ACKControl()
 *
 * @brief       - Enables or disables automatic acking during Master Recieving data.
 *
 * @param       - Pointer to I2C register struct
 * @param		- ENABLE or DISABLE macro
 *
 * @return      - none
 */

void I2C_ACKControl(I2Cx_Registers_t *pI2Cx, uint8_t ENorDIS) {

	if (ENorDIS == I2C_ACK_ENABLE) {
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);   // enables automatic acking
	}
	else {
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);  // disables automatic acking
	}

}

/***************************************************************************************
 * @fn          - I2C_Init() 
 * 
 * @brief       - Intialization function for values from the handle structure.
 *
 * @param       - Pointer to I2C register struct
 * @param		- ENABLE or DISABLE macro
 *
 * @return      - none
 */

void I2C_Init(I2C_Handle_t *pI2CHandle) { // we setup automatic ACKing, specified the device slave address, specified the frequency of the peripheral bus after prescalers, configure the clock speed that we want.

	/* Set automatic ACKing */
	uint32_t tempReg = 0x0;

	/* Enable the peripheral clock */
	// @study: "why doens't this one trigger the SR2 busy flag?"
	// ^^^ I think it is because the pins need to be configured first before the clock can be set?
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	//@change: Below 1 line. perhaps I need to have peripheral enabled before I can set ACK control.
	I2C_PeripheralControl(I2C1, ENABLE);
	//ack control bit IMPORTANT. We first need to enable the peripheral or else ACK will never be set.
	tempReg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 |= tempReg;

	// tempReg = 0;

	/* configure the FREQ field of CR2. Frequency of our peripheral based on  */
	tempReg = RCC_GetPCLK1Value() / 1000000U;	//we just want the number in Mhz
	pI2CHandle->pI2Cx->I2C_CR2 = (tempReg & 0x3F);

	tempReg = 0;

	//program the device own address. Helpful for when the device is acting as a slave device
	tempReg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1); // this is in the OAR1 register of I2C
	tempReg |= (1 << 14);	// Data Manual says that 14th bit must be KEPT at 1 by via software
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;


	/* CCR Calculations produces different serial clock speeds */
	tempReg = 0;
	uint16_t CCR_Value = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_ST) {//Standard Mode (SM)

		//tempReg |= (0 << 15);	// 0 = SM mode, bit position 15
		//note: for standard mode T-High = T-Low = CCR * T-PCLK
		CCR_Value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //@note: Formula for how this was derived is in video #191. Reference Duty Cycle also
		tempReg |= (CCR_Value & 0xFFF);		//mask the values.
	}
	else {	//fast mode (FM)
		tempReg |= (1 << 15);	// 1 = Fast Mode (FM), bit postion 15
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);	// bit position 14 configures the DUTY bit

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == 0) {		// DUTY = 0, T-HIGH + T-LOW = 3 * CCR * T-PCLK1

			CCR_Value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //@note: Formula for how this was derived is in video #191. Reference Duty Cycle also
			tempReg |= (CCR_Value & 0xFFF);

		}
		else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == 1) {

			CCR_Value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //@note: Formula for how this was derived is in video #191. Reference Duty Cycle also
			tempReg |= (CCR_Value & 0xFFF);

		}
		tempReg |= (CCR_Value & 0xFFF);		//mask the values, 12-bit mask.
	}

	pI2CHandle->pI2Cx->I2C_CCR |= tempReg;

	/* TRISE configuration (refer to section 199)*/
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_ST) {	// standard mode

		tempReg = ( RCC_GetPCLK1Value() * TRISE_ST_MAX) + 1;	// +1 is because the Data manual says so

	}
	else {
		tempReg = (RCC_GetPCLK1Value() * TRISE_FM_MAX) + 1;		// +1 is because the data manual says so
	}

	pI2CHandle->pI2Cx->I2C_TRISE = (tempReg & 0x3F);

}

/***************************************************************************************
 * @fn          - I2C_GenerateStartCondition
 *
 * @brief       -
 * @param		- Pointer to I2Cx_Registers_t structure
 *
 * @return      - none
 *
 * @note        - This can only occur AFTER you check the TxE == 1 and BTF == 1.
 * @note		- STOP bit clears once hardware detects a STOP. So only need to set in this function, not clear.
 */

void I2C_GenerateStartCondition(I2Cx_Registers_t *pI2Cx) {

	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);

}
/***************************************************************************************
 * @fn          - I2C_GenerateStopCondition()
 *
 * @brief       - Generates the stop condition after TxE = 1 and BTF = 1 (means transmit buffer emtpy and byte transfer finsihed)
 *
 * @param		- Pointer to I2Cx_Registers_t structure
 *
 * @return      - none
 *
 * @note        - This can only occur AFTER you check the TxE == 1 and BTF == 1.
 * @note		- STOP bit clears once hardware detects a STOP. So only need to set in this function, not clear.
 */

void I2C_GenerateStopCondition(I2Cx_Registers_t *pI2Cx) {

	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);

}

/***************************************************************************************
 * @fn          - I2C_ExecuteAddressPhase()
 *
 * @brief       - loads slave address into data register (DR) along with readOrWrite bit depending on argument
 *
 * @param		- Pointer to I2Cx_Registers_t structure
 * @param		- slaveAddr to be matched.
 * @param		- READ (1) or WRITE (0) macros.
 *
 *
 * @return      - none
 *
 * @note        -
 */

void I2C_ExecuteAddressPhase(I2Cx_Registers_t *pI2Cx, uint8_t slaveAddr, uint8_t readOrWrite) {

	slaveAddr <<= 1;	//because the last bit is the writeOrRead bit

	if (readOrWrite) { //note: READ = 1. So this would be a READ
		slaveAddr |= 0x1;
	}
	else {	//note: WRITE = 0
		slaveAddr &= ~(0x01);
	}
	pI2Cx->I2C_DR = slaveAddr;

}



/***************************************************************************************
 * @fn          - I2C_ClearADDRFlag()
 *
 * @brief       - AFter a successful slave address send and leading into data register the ADDR bit in SR1 gets set to 1 and is cleared by reading SR1 then SR2.
 *
 * @param		- Pointer to I2Cx_Registers_t structure
 *
 * @return      - none
 *
 * @note        - For info about ADDR, refer to data manual 27.6.6
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {

	uint32_t dummyRead;
	(void)dummyRead; // gets rid of warning message "unused variable"

	//check if in master or slave mode.
	if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {

		//device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

			if (pI2CHandle->RxSize == 1) {

				// first disable the ACKing.
				I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);

				// can now clear the ADDR flag (read SR1, then read SR2)
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;



			}

		}
		else {

			// clear the ADDR flag (read SR1, then read SR2 to clear)
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;

		}

	}
	else {

		// device is in slave mode
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;

	}

}

uint8_t I2C_SR1_GetFlagStatus(I2Cx_Registers_t *pI2Cx, uint32_t flagName) {

	uint32_t flag = pI2Cx->I2C_SR1;
	if (flag & flagName) {
		return FLAG_SET;
	}
	else {
		return FLAG_RESET;
	}

}

/***************************************************************************************
 * @fn          - I2C_CloseRecieveData()
 *
 * @brief       - Sends data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 *
 * @note        - Refer to clock tree and RCC registers.
 */

/* we are closing the interrupts because we don't want them anymore to occur. */


void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle) {

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY; // state is reset
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) // only if we want to keep it open
	{
		I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
	}

}

/***************************************************************************************
 * @fn          - I2C_CloseSendData()
 *
 * @brief       - Sends data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 *
 * @note        - Refer to clock tree and RCC registers.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

/***************************************************************************************
 * @fn          - I2C_MasterSendData()
 * 
 * @brief       - Sends data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 * @note        - Refer to clock tree and RCC registers.
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {

	/* Step 1: Generate the start condition */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* Step 2: Confirm that start generation is completed by checking the SB flag in the SR1. Note: Until SB is cleared SCL will be stretched (Pulled to LOW) */
 	while(!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) { // SB = "Start bit" Sets when start condition is generated by the master.
		// do nothing. Wai`t until the START is generated by the master (SB == 1)
	}

	/* Step 3: Send the address of the slave with r/w bit set to WRITE!!! (0) (total 8-bits, 7 + 1) */
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr, WRITE); // sends slave address

	while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) {





		/* Step 4: Confirm that address phase is completed by checking the ADDR flag in the SR1 register */
			// @debug: ADDR flag is only set after an ACK is recieved.





		// confirm address phase is completed by checking the ADDR flag.
	}
	/* Step 5: Clear the ADDR flag by reading from SR1, then SR2 */
	// If we reach below then means ACK was sent back.
	I2C_ClearADDRFlag(pI2CHandle); // clears by reading SR1, then SR2 using dummy values.

	/* Step 6: Send data until the len = 0 */

	// automatic ACKing so we don't have to really check for that.

	while (len > 0) { // acking is done automatically

			while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) { // waits until the TxE buffer is emtpy.
				//do nothing except for wait until TXE buffer is completly empty.
			}

			//load the I2C data register
			pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
			pTxBuffer++;
			len--;
	}
	/* Step 7: before generating STOP need to check TxE and BTF flags to make sure buffer is empty and byte transmission has finished. */
	while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	// btf won't set
	while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	/* Step 8: generate the stop condition */
	if (Sr == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}



}

/***************************************************************************************
 * @fn          - I2C_MasterRecieveData
 *
 * @brief       - Recieves data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 *
 * @note        -
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint32_t dummyRead;
	(void)dummyRead;
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while (! I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{

		//Disable Acking
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while (!I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if (Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

	}

    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while( !I2C_SR1_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// could have a repeated start. check the github file for refernce
				if (Sr == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			//increment the buffer address
			pRxBuffer++;
		}

	}
	//re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/***************************************************************************************
 * @fn          - I2C_MasterSendDataIT()
 *
 * @brief       - Recieves data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 *
 * @note        -
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busyState = pI2CHandle->TxRxState; // gets current state

	if( (busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX)) // checks to make sure not transmitting or recieveing
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr; // Sr = "repeated start macro value"

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx); //this will generate interrupt due to SB flag being set. IRQ num 31 will trigger then we handle it.

		/* Section below, enables all the interrupts. Enable control bits for I2C protocol. */
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
		/* end of section */

	}

	return busyState;
}


/***************************************************************************************
 * @fn          - I2C_MasterRecieveData
 *
 * @brief       - Recieves data along I2C
 *
 * @param		-
 * @param		-
 * @param		-
 * @param		-
 *
 * @return      - none
 *
 * @note        -
 */



uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) // makes sure not currently transmitting or recieving
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);	//will generate an interrupt due to SB being set.

		/* section below enables all the interrupt enable control bits for I2C communication protocol */
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate; // should return I2C_BUSY_IN_RX macro.
}


/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleRXNEInterrupt()
 *
 * @brief             - Helper function to the RXNE interrupt handler I2C_EV_IRQHandling()
 *
 * @param[in]         - pointer to I2C_Handle_t structure.
 *
 * @return            - none
 *
 * @Note              -
 */

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {

	// We have to do the data reception
	if (pI2CHandle->RxSize == 1) {

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;

	}
	if (pI2CHandle->RxSize > 1) {

		// refer to the blocking-api version for more details
		if (pI2CHandle->RxLen == 2) {
			I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE); // disable ACKing.
		}

		// read the data register (DR)
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}

	if (pI2CHandle->RxLen == 0) {

		//close the I2C data reception and notify the application.

		// 1.) gernerate the stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR) {// don't want repeated start (SR = "repeated start").
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2.) Close the I2C Rx
		I2C_CloseRecieveData(pI2CHandle);

		// 3.) Notifiy the applicaiton
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}

}

/*********************************************************************
 * @fn      		  - I2C_MasterHandleRXNEInterrupt()
 *
 * @brief             - Helper function to the RXNE interrupt handler I2C_EV_IRQHandling()
 *
 * @param[in]         - pointer to I2C_Handle_t structure.
 *
 * @return            - none
 *
 * @Note              -
 */

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->TxLen > 0) {
		// 1.) load data into DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		// 2.) decrement the TxLen
		pI2CHandle->TxLen--;

		// 3.) increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}
/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/* study: as the events get triggered it is like a domino effect. In one part of code interrupt is triggered then antoher within that and another within that. */

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	//many kinds of events that can generate an interrupt.
	//Interrupt handling for both master and slave device.

	uint32_t temp1, temp2, temp3;

	/* Checks all the flags in the CR2 and SR1 for event-based interrupts that they actually set */
	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);	// enables event interrupts
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);			// Has the start bit been genreated? If so then we move onto the address phase.

	// 1.) SB event (only applicable in master-mode)
	if (temp1 && temp3) { // occurs for address phase.

		// Interrupt generated because SB flag is set. We are in master mode.
		// this blcok will not execute in slave mode because slave always has SB == 0. (SB = "start bit") 27.6.6
		// in this block lets execute the address phase. this is how the event is handled.

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READ);
		}
	}

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	// 2. Handle for interrupt generated by ADDR event
	// Note: Master mode address is sent, slave mode address matched with own address.
	if (temp1 && temp3) {

		// interrupt has occured because of ADDR flag is set. Clcok is stretched and in wait state.
		// need to clear the flag.
		I2C_ClearADDRFlag(pI2CHandle); // clears the ADDR flag. Finsihed for this ISR


	}

	// 3.) Handle for interupt generated by BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	if (temp1 && temp3) {

		// Interupt generated because BTF (byte transferred finished) flag is set
		//
		// refer to lecture 221 for diagram.
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			// make sure that txe is also set.
			if (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE)) {
				//BTF TXE are set. Indicates we close the transmission.
				if (pI2CHandle->TxLen == 0) { // only if no more to transmist do we close communication.
					// 1.) generate the STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR) //only stop if no repreated start.
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// 2.) reset the memebr elements of the handle structure.
					I2C_CloseSendData(pI2CHandle); // This is the part that sets pHandle TxRxState = READY

					// 3.) notify the applicaiton about transmission complete.
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			// nothing to do here. We don't close I2C reception like this.
		}

	}

	// 4.) handle for interrupt generated by STOPF event. (STOP condition has been detected by slave
	// Note: Stop detection flag is applicable only in slave mode.
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF); // read the flag
	if (temp1 && temp3) {
		// Slave mode (only can occur in slave mode)
		// STOPF flag is set. STOPF == 1 when STOP condition is detected. Section 27.6.6

		// clear the STOPF (read SR1 (already done) then write to CR1 to to clear
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000; // won't get affected but still technically writes.

		// notify the application that STOP is detected.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	// 5.) Handle for interrupt generated by TXE event.

	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE); // TXE, TXE buffer is empty
	if (temp1 && temp2 && temp3) {

		// check for device mode
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
			// we are in master mode.
			//TXE flag is set
			// We have to do the data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) { // we are indeed doing data transmission in master mode.
				I2C_MasterHandleTXEInterrupt(pI2CHandle); // loads the DR (Data register) and decrements the length.
			}

		}
		else { 	// device is in slave mode. If TXE buffer is empty it implies slave device is transmitting to master.

			// double check to make sure slave is in transmit mode.
			if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) {
				// TRA == 1 means "data bytes transmitted.
				// notify the application that the slave has transmitted data and is finished.
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}


	}

	// 6.) Handle for interrupt generated by RXNE event.

	// we are recieving bytes
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3) {

		// check the device mode, master or slave
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) { // this decides that below is a master section.

			//The device is master

			// RXNE flag is set. The recieve shift register is emtpy.
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}

		}
		else {
			// slave mode. Make sure really in reciever mode.
			if (!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}

	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2; // only two temps because ITERREN

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_ERROR_ARLO); // Arbitration loss error flag 27.6.6
		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_ERROR_AF); // AF = "ACK failure" 27.6.6
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);

	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_ERROR_OVR); // Section 27.6.6
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_ERROR_TIMEOUT); // Section 27.6.6
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);

	}

}



// void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
