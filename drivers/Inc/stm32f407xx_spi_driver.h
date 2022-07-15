/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 29, 2022
 *      Author: jordanrainey
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h" //should also include the driver specific header file.
#include <stddef.h>
#include <stdint.h>

/*START*********structs**************************************/
typedef struct {

	uint8_t SPI_DeviceMode;			/* <  Decide whether device master or slave mode     > */
	uint8_t SPI_BusConfig;			/* <  full duplex, half duplex,    > */
	uint8_t SPI_SclkSpeed;;			/* <  values @SPI_SclkSpeed  > */
	uint8_t SPI_DFF;				/* <     > */
	uint8_t SPI_CPOL;				/* <     > */
	uint8_t SPI_CPHA;				/* <     > */
	uint8_t SPI_SSM;				/* <     > */

}SPI_Config_t;

typedef struct {

	SPIx_Registers_t *pSPIx;
	SPI_Config_t SPIConfig;

	uint8_t *pTxBuffer; 	/* !< To store the application Tx buffer address > */
	uint8_t *pRxBuffer; 	/* !< To store the application Rx buffer address > */
	uint32_t TxLen;			/* !<To store Tx Length> */
	uint32_t RxLen;			/* !<To store Rx Length> */
	uint8_t TxState;		/* !<To store Tx state> */
	uint8_t RxState;		/* !<To store Rx state> */

}SPI_Handle_t;	// Interrupt handler for SPI Peripherals

/*END*********structs****************************************/


/*START*********APIs Supported By This Driver******************************/

void SPI_Init(SPI_Handle_t *pSPIHandle); 

void SPI_DeInit(SPIx_Registers_t *pSPIx);    // Resets all peripherals of specified port using RCC reset 

/*
 * peripheral clock setup 
 */

void SPI_PeriClockControl(SPIx_Registers_t *pSPIx, uint8_t ENorDIS); //ENorDIS = "Enable or disable"

/*
 * Data Send and Receive
 */
void SPI_SSIConfig(SPIx_Registers_t *pSPIx, uint8_t ENorDIS);

void SPI_SendData(SPIx_Registers_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);			// "len" is how many bytes should be sent

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);

void SPI_RecieveData(SPIx_Registers_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);	// interrupt enabled

/*
 *	IRQ Configuration and ISR handling	
 *	@note: error flags in data manual section 28.4.8
 */

void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); /* IRQHandling should already know what IRQ num and priority are so we just need pin number. */

void SPI_IRQInterruptConfigIT(uint8_t IRQNum, uint8_t ENorDI);
void SPI_IRQPriorityConfigIT(uint8_t IRQNum, uint32_t IRQPriority);

/*
 *	SPI_SR flag getting function.
 * 	@note: Data Manual Section 28.5.3
 */

/*
 *	Other APIs supported by this device
 */

void SPI_PeripheralControl(SPIx_Registers_t *pSPIx, uint8_t ENorDIS);
void SPI_SSOEConfig(SPIx_Registers_t *pSPIx, uint8_t ENorDIS);

uint8_t SPI_GetFlagStatus(SPIx_Registers_t *pSPIx, uint32_t flagName);


void SPI_ClearOVRFlag(SPIx_Registers_t *pSPIx);

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 * @note: Has to be implement by the application.
 */

void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent); // Informs application that Tx is over.




/*END*********APIs Supported By This Driver******************************/





/*START*********SPI MACROS**********************************************/

/*
 * SPI application states
 * @SPI_CR2 Values
 */

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 *	Possible SPI application events
 *	@SPI_APPLICATION_EVENTS
 */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*
 *	@SPI_Device_Modes	
 */

#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1	//make sure in this mode so clk will be sent

/*
 *	@SPI_BusConfig	
 */

#define	SPI_BUS_CONFIG_FD						1 // Full Duplex
#define	SPI_BUS_CONFIG_HD						2 // Half-Duplex
#define	SPI_BUS_CONFIG_SIMPLEX_RXONLY			3 // Simplex Recieve Only

/*
 *	@SPI_SclkSpeed
 * 	Data Manual Section 28.5.1, BR register is what configure the SPI Sclk scaler.
 */

#define SPI_SCLK_SPEED_DIV2			0	// fPCLK/2 
#define SPI_SCLK_SPEED_DIV4			1	// fPCLK/4 
#define SPI_SCLK_SPEED_DIV8			2	// fPCLK/8 
#define SPI_SCLK_SPEED_DIV16		3	// fPCLK/16
#define SPI_SCLK_SPEED_DIV32		4	// fPCLK/32
#define SPI_SCLK_SPEED_DIV64		5	// fPCLK/64
#define SPI_SCLK_SPEED_DIV128		6	// fPCLK/128
#define SPI_SCLK_SPEED_DIV256		7	// fPCLK/256

/*
 *	@DFF_Values
 * 	Data Manual Section 28.5.1, Bit [11]
 *	@note: IMPORTANT! This bit should only be configured when SPE = 0 for correct operation
 */	

#define SPI_DFF_8Bits		0	// this is default value
#define SPI_DFF_16Bits		1

/*
 *	@CPOL_Values
 */

#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 *	@CPHA_Values
 */

#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 *	@SSM_Values
 */

 #define SPI_SSM_HW			0	//Slave select Management, Software (Default settting)
 #define SPI_SSM_SW			1 	//slave select management, hardware

/*
 *	@SPI_SR_TXE_VALUES 
 */

#define SPI_SR_TXE_EMPTY		1
#define SPI_SR_TXE_NOTEMPTY		0

/*
 *	@SPI_SR_FLAG_VALUES
 * 	@Note: Used for checking flag status of the bit position in conjunction with the '&' operator
 * 	used in getFlagStatus function.
 */

#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG		(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)

/*END***********SPI MACROS**********************************************/

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
