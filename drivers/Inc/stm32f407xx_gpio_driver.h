/*
p
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 25, 2022
 *      Author: jordanrainey
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

//#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h" //should also include the driver specific header file.
#include <stdint.h>

typedef struct {

	uint8_t GPIO_PinNumber;			/* !<Possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;			/* !<Possible values from @GPIO_PIN_MODES> */
	uint8_t GPIO_PinSpeed;			/* !<Possible values from @GPIO_PIN_SPEED> */
	uint8_t GPIO_PinPuPdControl;	/* !<Possible values from @GPIO_PUPD_VALUES>*/
	uint8_t GPIO_PinOPType;			/* !<Possible valule from @GPIO_OUTPUT_TYPE>*/
	uint8_t GPIO_PinAltFunMode;		/* !<Possible values from @GPIO_AF_VALUES>	*/

}GPIO_PinConfig_t;

typedef struct { // handle structure for a GPIO pin

	GPIOx_Registers_t *pGPIOx; 			// holds base address of GPIO port to which the pin belongs.
	GPIO_PinConfig_t GPIO_PinConfig; 	// holds GPIO pin configuartion settings

}GPIO_Handle_t;

/*************************************************************************
 *
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 *
 *************************************************************************/

/* 
 * in-de initialization 
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); 
void GPIO_DeInit(GPIOx_Registers_t *pGPIOx);	// Resets all peripherals of specified port using RCC reset 

/*
 * peripheral clock setup 
 */

void GPIO_PeriClockControl(GPIOx_Registers_t *pGPIOx, uint8_t ENorDIS); //ENorDIS = "Enable or disable"

/*
 * Data read and write 
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_Registers_t *pGPIOx, uint8_t pinNum);/* read can only be 1 or 0 for input state, so we return a type of uint8_t */
uint16_t GPIO_ReadFromInputPort(GPIOx_Registers_t *pGPIOx); /* a port has 16 pins, all which can be 1 or 0, so we return a 16-bit unsigned int value */
void GPIO_WriteToOutputPin(GPIOx_Registers_t *pGPIOx, uint8_t pinNum, uint8_t value); //write to single pins, GPIO_PIN_SET/GPIO_PIN_RESET
void GPIO_WriteToOutputPort(GPIOx_Registers_t *pGPIOx, uint16_t); /* 16 pins in a port, so uint16_t instead of uint8_t */
void GPIO_ToggleOutputPin(GPIOx_Registers_t *pGPIOx, uint8_t value);

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority); 
void GPIO_IRQHandling(uint8_t pinNum); /* IRQHandling should already know what IRQ num and priority are so we just need pin number. */

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Possible Modes (Data manual, Section 8.41) NOTE: We put these macros here instead of MCU header because this is GPIO specific!!!
 */

#define GPIO_MODE_INPUT			0x0			
#define GPIO_MODE_OUTPUT		0x1
#define GPIO_MODE_ALTFN			0x2			//AF = "Alternate function mode"
#define GPIO_MODE_ANALOG		0x3
#define GPIO_MODE_IT_FT			0x4			// FT = "Falling edge, this and the next two are part of intterupt code
#define GPIO_MODE_IT_RT			0x5			// RT = "Rising Edge"
#define GPIO_MODE_IT_RFT		0x6			//RFT = "Rising/Falling Edge"

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * GPIO Output Type Macros (Data Manual, Section 8.4.2)
 */

#define GPIO_OUTPUT_TYPE_PP		0x0			// PP = "Push-Pull"
#define GPIO_OUTPUT_TYPE_OD		0x1			// OD = "Open Drain"

/* @GPIO_OUTPUT_TYPE
 * GPIO Output Type Macros (Data Manual, Section 8.4.2)
 */

#define GPIO_OUTPUT_TYPE_PP		0x0			// PP = "Push-Pull"
#define GPIO_OUTPUT_TYPE_OD		0x1			// OD = "Open Drain"

/*
 * GPIO Speed Type Macros (Data Manual, Section 8.4.3)
 */

#define GPIO_SPEED_LOW			0		
#define GPIO_SPEED_MEDIUM		1		
#define GPIO_SPEED_HIGH			2		
#define GPIO_SPEED_VERYHIGH		3		

/*@GPIO_PUPD_VALUES
 * GPIO pin pull up AND pull down configuration macros
 */
 #define GPIO_NO_PUPD			0
 #define GPIO_PIN_PU			1
 #define GPIO_PIN_PD			2

/*
 *@GPIO_AF_VALUES
 *
 */

#define GPIO_AF0				0
#define GPIO_AF1				1
#define GPIO_AF2				2
#define GPIO_AF3				3
#define GPIO_AF4				4
#define GPIO_AF5				5
#define GPIO_AF6				6
#define GPIO_AF7				7
#define GPIO_AF8				8
#define GPIO_AF9				9
#define GPIO_AF10				10
#define GPIO_AF11				11
#define GPIO_AF12				12
#define GPIO_AF13				13
#define GPIO_AF14				14
#define GPIO_AF15				15


#endif /* STM32F407XX_GPIO_DRIVER_H_ */

