/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 29, 2022
 *      Author: jordanrainey
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"

/* helper functions. Do not include in the MCU header */
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/***************************************************************************************
 * @fn          - SPI_ApplicationEventCallback
 *
 * @brief       - For the application side to implement. Otherwise this __weak will be called
 * @param       - Pointer to SPI_Handle_t struct.
 * @param       - Macros for what event has occured. @SPI_APPLICATION_EVENTS in SPI header
 *
 * @return      - none
 *
 * @note        - This is to be implemented by the application developer. Else this weak function is called.
 */
/* Below is gcc attricute weak being utilized */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent); // Informs application that Tx is over.

void SPI_PeriClockControl(SPIx_Registers_t *pSPIx, uint8_t ENorDI) { //ENorDI = "Enable or disable"
	if (ENorDI == ENABLE) {

		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5) {
			SPI5_PCLK_EN();
		}
		else if (pSPIx == SPI6) {
			SPI6_PCLK_EN();
		}
	}
	else { //otheriwse means it wants to be DISABLED macro

		if(pSPIx == SPI1) {
        	SPI1_PCLK_DI();
    	}
    	else if (pSPIx == SPI2) {
        	SPI2_PCLK_DI();
    	}
    	else if (pSPIx == SPI3) {
        	SPI3_PCLK_DI();
    	}
    	else if (pSPIx == SPI4) {
        	SPI4_PCLK_DI();
    	}
    	else if (pSPIx == SPI5) {
        	SPI4_PCLK_DI();
    	}
    	else if (pSPIx == SPI6) {
        	SPI5_PCLK_DI();
    	}
	}
}

 /***************************************************************************************
  * @fn          - SPI_Init()
  *
  * @brief       - Sets the Registers from the Handle struct memebrs. 
  *
  * @param       - Pointer to SPI_Handle_t struct
  *
  * @return      - none
  *
  * @note        - SPIx register reset macros in stm32f407xx.h 
  */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);	// Won't forget to initialize the RCC clock

	//configure the SPI_CR1 register, which is a uint32_t. So we can create a temp and initialize.
	uint32_t tempReg = 0;	
	/* Master or Slave Mode */
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	/* BusConfig */
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);	//sets to 0	
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);	// BIDIMODE (bidirectional) should be reset.
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	/* Sclk Speed Config */
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	/* DFF config */
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);	//only 1 values, 0 and 1
	
	/* SPI_CPOL config */
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);	//only 2 values, 0 and 1

	/* SPI_CPHA config */
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);	//only 2 values, 0 and 1

	/* SPI_SSM config */
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);	//only 2 values, 0 and 1

	pSPIHandle->pSPIx->SPI_CR1 = tempReg;

	/* enable the SPI clock */
}
 /***************************************************************************************
  * @fn          - SPI_DeInit()
  * 
  * @brief       - Clears the registers values, then swaps out of reset mode based on which SPIx.
  *
  * @param       - Pointer to SPIx registers. 
  *
  * @return      - none
  *
  * @note        - SPIx register reset macros in stm32f407xx.h 
  */

void SPI_DeInit(SPIx_Registers_t *pSPIx) {// Resets all peripherals of specified port using RCC reset 

	if(pSPIx == SPI1) {
         SPI1_REG_RESET();
     }
     else if (pSPIx == SPI2) {
         SPI2_REG_RESET();
     }
     else if (pSPIx == SPI3) {
         SPI3_REG_RESET();
     }
     else if (pSPIx == SPI4) {
         SPI4_REG_RESET();
     }
     else if (pSPIx == SPI5) {
         SPI5_REG_RESET();
     }
     else if (pSPIx == SPI6) {
         SPI6_REG_RESET();
     }

}

 /***************************************************************************************
  * @fn          - SPI_GetFlagStatus()
  * 
  * @brief       - returns bit value (flag) stored in SPI Status register. 
  *
  * @param       - Pointer to SPIx registers. 
  * @param       - Macro value from stm32f407xx_spi_driver.h, @SPI_SR_FLAG_VALUES
  *
  * @return      - Flag Status, FLAG_RESET == 0, FLAG_SET = 1 
  *
  * @note        - flagName parameter values in SPI driver header, @SPI_SR_FLAG_VALUES
  */

uint8_t SPI_GetFlagStatus(SPIx_Registers_t *pSPIx, uint32_t flagName) {

	if (pSPIx->SPI_SR & (flagName)) {
		return FLAG_SET;
	}	
	return FLAG_RESET;

}

/***************************************************************************************
 * @fn          - SPI_SendData()
 * 
 * @brief       - Loads a pointer to the data we want to send, and sends it to the transmitter buffer via
 				  the data register 
 * @param       - Pointer to SPIx registers. 
 * @param       - Pointer to the data we want to send. It gets incremented depending on our length.
 *
 * @return      - none
 *
 * @note        - Data Manual Section 
 */
  
void SPI_SendData(SPIx_Registers_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

	// @study: This is a blocking call: waits until buffer is emtpy. Cannot continue until done

	while (len > 0) {

		// Step 1: Wait until TXE is set. 
		 
		while (!SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)) {
			//do nothing except for wait until TXE buffer is completly empty.
			// @study ^^^polling function: may produce an infinite loop.
		}

		//check the dff bit
		if ( (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)) )  {	// checks if 16-bit option is chosen
			// load the data
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);	//16-bit data will go inot data register now
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}
		else {
			// load 8-bits of data
			pSPIx->SPI_DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
	
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX) {

		// 1.) Save the Tx buffer address and length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2.) Mark the SPI state as busy in transmission so that no other code can take over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3.) Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE); // enables interrupt for transmission

		// 4.) Data transmission wil be handled by the ISR code (will implement later)

	}

	return state;

}

void SPI_RecieveData(SPIx_Registers_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {

	while (len > 0) {

		// Step 1: Wait until TXE is set.

		while (!SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)) {
			//do nothing except for wait until RXNE buffer is empty.
			// @study ^^^polling function: may produce an infinite loop.
		}

		//check the dff bit
		if ( (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)) )  {	// checks if 16-bit option is chosen
			// load the data from DR to Receive buffer
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++; // points to next free memory address.
		}
		else {
			// 8-bits
			(*pRxBuffer) = pSPIx->SPI_DR; // 8-bits from data register to recieve buffer.
			len--;
			pRxBuffer++;
		}
	}

}

uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {

	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {

		// 1.) Save the Rx buffer address and length information in some global variables
		//pSPIHandle->pRx = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2.) Mark the SPI state as busy in transmission so that no other code can take over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3.) Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE); // enables interrupt for transmission

		// 4.) Data transmission wil be handled by the ISR code (will implement later)

	}

	return state;

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); /* IRQHandling should already know what IRQ num and priority are so we just need pin number. */


}



void SPI_PeripheralControl(SPIx_Registers_t *pSPIx, uint8_t ENorDIS) {

	if (ENorDIS == ENABLE) {
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);	// enables the peripheral
	}
	else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);	// enables the peripheral
	}

}

/***************************************************************************************
 * @fn          - SPI_SSIConfig
 *
 * @brief       - When device in in master mode and SSM = 1, SSI needs to SSI = 1 to prevent mode fault.
 * @param       - Pointer to SPIx registers.
 * @param       - ENABLE or DISABLE macros
 *
 * @return      - none
 *
 * @note        - D
 */
void SPI_SSIConfig(SPIx_Registers_t *pSPIx, uint8_t ENorDIS) {

	if (ENorDIS == ENABLE) {
			pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);	// enables the peripheral
	}
	else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);	// disables the peripheral
	}

}

/***************************************************************************************
 * @fn          - SPI_SSOECOnfig()
 *
 * @brief       - When in hardware slave select mode (SSM = 0), when SPE = 1 or = 0, then we need to enable slave select output by SSOE = 0 or 1;
 * @param       - Pointer to SPIx registers.
 * @param       - ENABLE or DISABLE macros
 *
 * @return      - none
 *
 * @note        - SSOE Info in Data Manual Section 28.5.2
 */
void SPI_SSOEConfig(SPIx_Registers_t *pSPIx, uint8_t ENorDIS) {

	if (ENorDIS == ENABLE) {
			pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);	// Enable the software slave select enable
	}
	else {
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);	// Disable the software slave select enable
	}

}



void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENorDI);

void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority) { // same as the GPIO one.

	uint8_t iprx = IRQNum / 4;			// iprx = "interrupt priority register num

	uint8_t iprx_section = IRQNum % 4; 	// because each iprx is subdivded into 4 sections each with 8 bits.

	uint8_t shiftAmt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shiftAmt);

}

/***************************************************************************************
 * @fn          - SPI_IRQHandling
 *
 * @brief       - When device in in master mode and SSM = 1, SSI needs to SSI = 1 to prevent mode fault.
 * @param       - Pointer to SPIx registers.
 * @param       - ENABLE or DISABLE macros
 *
 * @return      - none
 *
 * @note        - D
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) { /* IRQHandling should already know what IRQ num and priority are so we just need pin number. */


	uint8_t temp1, temp2;
	// first lets check for TxE

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE); // testing the TXE in the SR. = 0 if reset
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {

		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);	// helper function

	}

	// check for RXNE

	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE); // testing the TXE in the SR. = 0 if reset
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {

		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);	// helper function
	}

	// Not doing MODF, CRC, or FRE interrupts

	//check for OVR flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {

		//handle the OVR error
		spi_ovr_err_interrupt_handle(pSPIHandle);

	}


}



void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//check the DFF bit
	if ( (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)) )  {	// checks if 16-bit option is chosen
		// load the data
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);	//16-bit data will go inot data register now
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else {
		// load 8-bits of data
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	// Close the communication

	if (!pSPIHandle->TxLen) { // if TxLen == 0 end of transmission.
		// TxLen is zero, close spi transmission and inform the applicaiton that TX is over.
		SPI_CloseTransmisson(pSPIHandle);

		/* Now we have to inform the application that interrupt is finished */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT); // Informs application that Tx is over.
	}

}


void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	//do rxing as per the dff
	if ( (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)) )  {	// checks if 16-bit option is chosen

		// 16-bit data
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->SPI_DR;	// reads from data register.
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;

		//#warning "below is the course's way, I think it can just be a single decrement with uint16_t"
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
		// (uint16_t*)pSPIHandle->pRxBuffer--; <-- my way of doing it.

	}
	else {
		// load 8-bits of data
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pRxBuffer;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	// Close the communication

	if (!pSPIHandle->RxLen) { // if TxLen == 0 end of transmission.
		// TxLen is zero, close spi transmission and inform the applicaiton that TX is over.

		SPI_CloseReception(pSPIHandle);

		/* Now we have to inform the application that interrupt is finished */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT); // Informs application that Tx is over.
	}

}

void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	// 1.) clear the OVR flag. Read DR then SR

	uint8_t temp; // dummy variable

	#ifdef STUDY_NOTE
		// We don't want to do
	#endif

	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) { // if busy in transmission this wont execute
		// Section 28.5.2: OVR is cleared by reading DR then SR.
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;

	}
	(void)temp;
	// 2.) Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR); // has to clear SPI_ClearOVRFlag()

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {

	// resets of fields and setting back to ready
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;


}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

	// resets of fields and setting back to ready
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPIx_Registers_t *pSPIx) {

	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;

}



	// this is a weak implementation. The applicaiton may override this function.
	// STUDY_NOTE: if this fucntino isn't implemented then the applcaiton will
	// call this weak function instead.



void SPI_IRQInterruptConfigIT(uint8_t IRQNum, uint8_t ENorDI);

void SPI_IRQPriorityConfigIT(uint8_t IRQNum, uint32_t IRQPriority);


