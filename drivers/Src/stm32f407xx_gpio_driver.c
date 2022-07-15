/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 25, 2022
 *      Author: jordanrainey
 */

#include <stdint.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

void GPIO_PeriClockControl(GPIOx_Registers_t *pGPIOx, uint8_t ENorDI); // so that I can use this in GPIO_Init()

/* 
 * in-de initialization 
 */

/***************************************************************************************
 * @fn			- GPIO_Init
 * 
 * @brief		- 
 *
 * @param		-
 * @param		- 
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;

	// 1. Configure depending on if it is in Alternate Function mode.


	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//the non interrupt modes input, output, alternate function, and analog mode.
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 * (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 			// clears the bits
		pGPIOHandle->pGPIOx->MODER |= temp;																	// sets the bits
	}
	else {
		// the interrupt modes.
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
		
			//configure the EXTI_FTSR. IMPORTANT!!! Make sure the EXTI_RTSR is == LOW also!!!!
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}	
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// configure both EXTI_FTSR and EXTI_RTSR	
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2.) Configure the GPIO port selection in SYSCFG_EXTICR (aka system config external interrupt register.
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;		// determines which EXTI#
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;	// for bit shifting.
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portCode << (temp2 * 4));
		// 3.) Enable the exti interrupt delivery using IMR (aka interrupt mask register)
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//interrupt goes over this line.

	}

	temp = 0;

	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 * (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));			// clears the bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;																// sets the bits
	temp = 0;

	// 3. configure the pullup/pulldown settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 * (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));			// clears the bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;																	// sets te bits
    temp = 0;

	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	//clear the space
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);			// clears bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;															// sets bits
	temp = 0;

	// 5. configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) { // can only configure if not in a general purpose input/output mode
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; 			// can only be 0 or 1 in value. Determines if HIGH or LOW register is selected.
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;			// determines how many bit shifts to do. Each pin has 4 bit fields so *4 is a must
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));			// clears bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));			// x4 is important, each value has 4-bitfields per pin.
	}

}



/***************************************************************************************
 * @fn			- GPIO_DeInit
 * 
 * @brief		- enables or disables peripheral clock for a given GPIO port
 *
 * @param		- base address of GPIO peripheral 
 * @param		- ENABLE or DISABLE macros (1 or 0 value)
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_DeInit(GPIOx_Registers_t *pGPIOx) {    // Resets all peripherals of specified port using RCC reset 

	if(pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    }   
    else if (pGPIOx == GPIOB) {
       	GPIOB_REG_RESET(); 
    }   
    else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    }   
    else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    }   
    else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    }   
    else if (pGPIOx == GPIOF) {
        GPIOF_REG_RESET();
    }   
    else if (pGPIOx == GPIOG) {
        GPIOG_REG_RESET();
    }   
    else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    }   
    else if (pGPIOx == GPIOI) {
        GPIOI_REG_RESET();
    }

}
/*
 * peripheral clock setup 
 */

/***************************************************************************************
 * @fn			- GPIO_PeriClockControl
 * 
 * @brief		- enables or disables peripheral clock for a given GPIO port
 *
 * @param		- base address of GPIO peripheral 
 * @param		- ENABLE or DISABLE macros (1 or 0 value)
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_PeriClockControl(GPIOx_Registers_t *pGPIOx, uint8_t ENorDI) { //ENorDI = "Enable or disable"
	
	if(pGPIOx == GPIOA) {
		GPIOA_PCLK_EN();
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_PCLK_EN();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_PCLK_EN();
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_PCLK_EN();
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_PCLK_EN();
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_PCLK_EN();
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_PCLK_EN();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_PCLK_EN();
	}
	else if (pGPIOx == GPIOI) {
		GPIOI_PCLK_EN();
	}
	else { //otheriwse means it wants to be DISABLED macro
		if(pGPIOx == GPIOA) {
        	GPIOA_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOB) {
        	GPIOB_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOC) {
        	GPIOC_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOD) {
        	GPIOD_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOE) {
        	GPIOE_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOF) {
        	GPIOF_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOG) {
        	GPIOG_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOH) {
        	GPIOH_PCLK_DI();
    	}   
    	else if (pGPIOx == GPIOI) {
        	GPIOI_PCLK_DI();
    	}   


	}

}
/*
 * Data read and write 
 */

/***************************************************************************************
 * @fn			- GPIO_ReadFromInputPin
 * 
 * @brief		- Reads whether it is a zero or 1 from the specified inputpin from a specified port 
 *
 * @param		- base address of GPIO peripheral
 * @param		- pin number we want to be reading from
 *
 * @return 		- Reads a 0 or a 1
 *
 * @note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_Registers_t *pGPIOx, uint8_t pinNum) {/* read can only be 1 or 0 for input state, so we return a type of uint8_t */

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> pinNum) & 0x00000001);
	return value;

}


/***************************************************************************************
 * @fn			- GPIO_ReadFromInputPort()
 * 
 * @brief		- reads the entire input data register, all 16 bits 
 *
 * @param		- base address of GPIO peripheral 
 *
 * @return 		- 16-bit value that represents all bitfields in the IDR (Input data register)
 *
 * @note		- For IDR info: refer to the data manual for the MCU, Section 8.4.5
 */
uint16_t GPIO_ReadFromInputPort(GPIOx_Registers_t *pGPIOx) {

	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/***************************************************************************************
 * @fn			- GPIO_WriteToOutputPin()
 * 
 * @brief		- Sets pin on a GPIOx port to output 0 or 1 value.
 *
 * @param		- base address of GPIO peripheral 
 * @param		- Pin number we want to change
 * @param		- value we want to change specified pin to (0, or 1 only)
 *
 * @return 		- none
 *
 * @note		- For info on ODR (output data regsiter), refer to data manual section 8.4.6.
 */
void GPIO_WriteToOutputPin(GPIOx_Registers_t *pGPIOx, uint8_t pinNum, uint8_t value){

	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (value << pinNum);			// set the bit position
	}
	else {
		pGPIOx->ODR &= ~(value << pinNum);			// clears the bit position
	}


} 

/***************************************************************************************
 * @fn			- GPIO_WriteToOutputPort()
 * 
 * @brief		- Write to ODR (output data register) the whole port, all 16 pins.
 *
 * @param		- base address of GPIO peripheral 
 * @param		- values to occupy the bit fields 
 *
 * @return 		- none
 *
 * @note		- For info on ODR (output data regsiter), refer to data manual section 8.4.6.
 */
void GPIO_WriteToOutputPort(GPIOx_Registers_t *pGPIOx, uint16_t value) {			// output port have 16 pins, each with 1 bit per pin, so 16-bit type is used.
/*
	pGPIOx->ODR &= 0x0;			// clears all 16-bits
	pGPIOx->ODR |= value;		// sets all 16-bits
*/ // ^^^this way to too inefficient if we are setting all values. The below is the better way. No need to clear the bits with bitshift operators.

	pGPIOx->ODR = value;		//does the clearing and setting of bits automatically. Faster code!

} 

/***************************************************************************************
 * @fn			- GPIO_ToggleOutputPin()
 * 
 * @brief		- toggles value of specified pin in the ODR (output data register). 
 *
 * @param		- base address of GPIO peripheral 
 * @param		- pin number we want to toggle
 *
 * @return 		- none
 *
 * @note		- For info on ODR (output data regsiter), refer to data manual section 8.4.6
 */
void GPIO_ToggleOutputPin(GPIOx_Registers_t *pGPIOx, uint8_t pinNum) {

	pGPIOx->ODR ^= (1 << pinNum);

}

/*
 * IRQ configuration and ISR handling
 */


/***************************************************************************************
 * @fn			- GPIO_PeriClockControl
 * 
 * @brief		- enables or disables peripheral clock for a given GPIO port
 *
 * @param		- base address of GPIO peripheral 
 * @param		- ENABLE or DISABLE macros (1 or 0 value)
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENorDI){

	// interrupts only go up to 81 according to data manual, section 12.2
	if (ENorDI == ENABLE) {

		if (IRQNum <= 31) {
   			*NVIC_ISER0 |= (1 << IRQNum);     	// want to be different here, not modulus, makes processor slower.
        }
        else if (IRQNum <= 63) {
		// @change: The comparisions used to have 2 operands like (IRQNum >= 32 && <= 63). Removed to save space
       		*NVIC_ISER1 |= (1 << (IRQNum % 32));        
        }
        else if (IRQNum <= 81) {
          	*NVIC_ISER2 |= (1 << (IRQNum % 64));                                              
	  	}
	}
	else if (ENorDI == DISABLE) {
		if (IRQNum <= 31) {                                                                                       		
        	*NVIC_ISER0 &= (0 << IRQNum);  
        }
        else if (IRQNum <= 63) {
        	*NVIC_ISER1 &= (0 << (IRQNum % 32));        
        }
        else if (IRQNum <= 81) {
          	*NVIC_ISER2 &= (0 << (IRQNum % 64));                                              
        }
	
	} 
}

void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority) {

	uint8_t iprx = IRQNum / 4;			// iprx = "interrupt priority register num
	uint8_t iprx_section = IRQNum % 4; 	// because each iprx is subdivded into 4 sections each with 8 bits. 

	uint8_t shiftAmt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shiftAmt);
	

}
/***************************************************************************************
 * @fn			- GPIO_PeriClockControl
 * 
 * @brief		- enables or disables peripheral clock for a given GPIO port
 *
 * @param		- base address of GPIO peripheral 
 * @param		- ENABLE or DISABLE macros (1 or 0 value)
 *
 * @return 		- none
 *
 * @note		- none
 */
void GPIO_IRQHandling(uint8_t pinNum) { /* IRQHandling should already know what IRQ num and priority are so we just need pin number. */

	// clear the EXTI PR (pending register) corresponding to the pin number (Section 12.3.6 of the Data Manual)	
	// to clear a bit, you set it to 1 according to the data manual for this register.	

	if (EXTI->EXTI_PR & (1 << pinNum)) {
		//clear the PR bit by setting a 1 (remember, 1 is a trigger, not a value, so inputting a 1 will flip its switch)
		EXTI->EXTI_PR |= (1 << pinNum);
	}
}

