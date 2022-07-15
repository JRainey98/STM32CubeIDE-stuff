/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jun 25, 2022
 *      Author: jordanrainey
 */


#include "stm32f407xx.h"
#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

/***************************************************************************************
 * @fn          - RCC_GetPCLK1Value()
 *
 * @brief       - Gets the clock speed for APB1 and AHB1 Bus.
 * @param       - none
 *
 * @return      - the clock speed value in Mhz.
 *
 */
uint32_t RCC_GetPCLK1Value(void) {

	uint8_t clksrc;
	uint32_t sysClkSpeed; 	//system clock speed
	uint8_t AHBPrescaler;
	uint8_t APB1Prescaler;
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0) {	// HSI clock
		sysClkSpeed = 16000000;		// 16 Mhz
	}
	else if (clksrc == 1) {	//HSE clock
		sysClkSpeed = 8000000;		// 8 Mhz
	}
	else if (clksrc == 2) { //PLL clock
		// we don't use PLL in this course
	}

	/* Now to find out he prescaler values */

	uint8_t temp;
	temp = ((RCC->CFGR >> 4) & (0xF));

	if (temp < 8) {
		AHBPrescaler = 1; 	//no division occurs
	}
	else {
		AHBPrescaler = AHB_PreScaler[temp - 8];
	}

	// find out the APB1 prescaler

	temp = ((RCC->CFGR >> 10) & (0x7));

	if (temp < 4) {
		APB1Prescaler = 1; 	//no division occurs
	}
	else {
		APB1Prescaler = APB1_PreScaler[temp - 4];
	}

	return (sysClkSpeed/AHBPrescaler/APB1Prescaler); // the peripheral clock speed after prescalers and system clock taken into account.
}

/***************************************************************************************
 * @fn          - RCC_GetPCLK1Value()
 *
 * @brief       - Gets the clock speed for APB1 and AHB1 Bus.
 * @param       - none
 *
 * @return      - the clock speed value in Mhz.
 *
 */

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t systemClock = 0, temp, pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		systemClock = 16000000;
	}else
	{
		systemClock = 8000000;
	}
	temp = ( RCC->CFGR >> 4 ) & 0xF;

	// calculate the AHB1 prescaler
	if(temp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> 13 ) & 0x7;

	// calculate the APB2 prescaler
	if(temp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[temp-4];
	}

	pclk2 = (systemClock / ahbp )/ apb2p;

	return pclk2;
}

