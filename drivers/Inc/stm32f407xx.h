/*
 * stm32f407xx.h
 *
 *  Created on: May 22, 2022
 *      Author: jordanrainey
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
//#include "stm32f407xx_spi_driver.h" <--- DON'T DO THIS, WILL CAUSE UNDEFINED ERROR!!!

#define __weak		 __attribute__((weak))

/*START*****************************Processor Specific Info, Coretx M4******************************/

/*
 *	NVIC_ISERx (interrupt set enable) register addresses 
 */
 
#define NVIC_ISER0			( (volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			( (volatile uint32_t*)0xE000E104)
#define NVIC_ISER2			( (volatile uint32_t*)0xE000E108)
#define NVIC_ISER3			( (volatile uint32_t*)0xE000E10C)

/*
 *	NVIC_ICERx (interrupt clear enable) register addresses 
 */

#define NVIC_ICER0			( (volatile uint32_t*)0xE000E180)
#define NVIC_ICER1			( (volatile uint32_t*)0xE000E184)
#define NVIC_ICER2			( (volatile uint32_t*)0xE000E188)
#define NVIC_ICER3			( (volatile uint32_t*)0xE000E18C)

/*
 *	priority register address calculation	
 */

#define NVIC_PR_BASE_ADDR			( (volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

/*END*****************************Processor Specific Info, Coretx M4******************************/




//define the base addresses of flash and SRAM

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U		// 112 KB
#define SRAM2_BASEADDR			0x2001C000U		// 16 KB
#define ROM						0x1FFF7800U		// "system memory"
#define SRAM 					SRAM1_BASEADDR

/* bus addresses */

#define PERIPH_BASE				0x40000000U		// peri base start 
#define APB1PERIPH_BASE			PERIPH_BASE 	// has same base
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

/* GPIOx base addresses */

#define GPIOA_BASEADDR			0x40020000U
#define GPIOB_BASEADDR			0x40020400U
#define GPIOC_BASEADDR			0x40020800U
#define GPIOD_BASEADDR			0x40020C00U
#define GPIOE_BASEADDR			0x40021000U
#define GPIOF_BASEADDR			0x40021400U
#define GPIOG_BASEADDR			0x40021800U
#define GPIOH_BASEADDR			0x40021C00U
#define GPIOI_BASEADDR			0x40022000U
#define GPIOJ_BASEADDR			0x40022400U
#define GPIOK_BASEADDR			0x40022800U

/* ABP1 peripherals */

#define I2C1_BASEADDR			((APB1PERIPH_BASE) + 0x5400)
#define I2C2_BASEADDR			((APB1PERIPH_BASE) + 0x5800)
#define I2C3_BASEADDR			((APB1PERIPH_BASE) + 0x5C00)
#define SPI2_BASEADDR			((APB1PERIPH_BASE) + 0x3800)
#define SPI3_BASEADDR			((APB1PERIPH_BASE) + 0x3c00)
#define USART2_BASEADDR			((APB1PERIPH_BASE) + 0x4400)
#define USART3_BASEADDR			((APB1PERIPH_BASE) + 0x4800)
#define UART4_BASEADDR			((APB1PERIPH_BASE) + 0x4c00)
#define UART5_BASEADDR			((APB1PERIPH_BASE) + 0x5000)

/* APB2 Peripherals */

#define SPI1_BASEADDR			((APB2PERIPH_BASE) + 0x3000)
#define SPI4_BASEADDR			((APB2PERIPH_BASE) + 0x3400)
#define SPI5_BASEADDR			((APB2PERIPH_BASE) + 0x5000)
#define SPI6_BASEADDR			((APB2PERIPH_BASE) + 0x5400)
#define EXTI_BASEADDR			((APB2PERIPH_BASE) + 0x3c00)
#define USART1_BASEADDR			((APB2PERIPH_BASE) + 0x1000)
#define USART6_BASEADDR			((APB2PERIPH_BASE) + 0x1400)
#define SYSCFG_BASEADDR			((APB2PERIPH_BASE) + 0x3800)

/* AHB1 peripherals */

#define RCCBASEADDR			((AHB1PERIPH_BASE) + 0x3800)

/*
 * 	@IRQ_EXTI_NUMBERS
 * 	IRQ numbers for stm32f407xx MCU
 *	@note: Data Manual, Section 12.2
 */

#define IRQ_NUM_EXTI0			6
#define IRQ_NUM_EXTI1			7
#define IRQ_NUM_EXTI2			8
#define IRQ_NUM_EXTI3			9
#define IRQ_NUM_EXTI4			10
#define IRQ_NUM_EXTI9_5			23
#define IRQ_NUM_EXTI15_10		40


#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
/*
 *	IRQ Priority Numbers
 *	@note: range [0, 15]
 */

#define NVIC_IRQ_PRIO_0		0
#define NVIC_IRQ_PRIO_1		1
#define NVIC_IRQ_PRIO_2		2
#define NVIC_IRQ_PRIO_3		3
#define NVIC_IRQ_PRIO_4		4
#define NVIC_IRQ_PRIO_5		5
#define NVIC_IRQ_PRIO_6		6
#define NVIC_IRQ_PRIO_7		7
#define NVIC_IRQ_PRIO_8		8
#define NVIC_IRQ_PRIO_9		9
#define NVIC_IRQ_PRIO_10	10
#define NVIC_IRQ_PRIO_11	11
#define NVIC_IRQ_PRIO_12	12
#define NVIC_IRQ_PRIO_13	13
#define NVIC_IRQ_PRIO_14	14
#define NVIC_IRQ_PRIO_15	15




/* SYSCFG (System configuation Controller) struct registers */

typedef struct {

	volatile uint32_t MEMRMP;		/* Memory remap register				offset: 0x00		*/
	volatile uint32_t PMC;			/* peripheral mode config register		offset: 0x04		*/
	volatile uint32_t EXTICR[4];	/* ext inter config reg 				offset: 0x08-0x14	*/
	volatile uint32_t SYSCFG_RES1;	/* reserved								offset: 0x18		*/
	volatile uint32_t SYSCFG_RES2;	/* reserved								offset: 0x1c		*/
	volatile uint32_t CMPCR;		/* compensation cell ctrl register		offset: 0x20		*/

} SYSCFG_Registers_t;

#define GPIO_BASEADDR_TO_CODE(x)	 	(((x) == GPIOA)?0 :\
										((x) == GPIOB)?1 :\
										((x) == GPIOC)?2 :\
										((x) == GPIOD)?3 :\
										((x) == GPIOE)?4 :\
										((x) == GPIOF)?5 :\
										((x) == GPIOG)?6 :\
										((x) == GPIOH)?7 :\
										((x) == GPIOI)?8 :\
										((x) == GPIOJ)?9 :\
										((x) == GPIOK)?10:0)


/* GPIO struct */

typedef struct {

	/* Info found in data manual (DM) section 8.4: GPIO Registers */

	volatile uint32_t MODER;	/* mode register						offset: 0x00 		*/
	volatile uint32_t OTYPER;	/* output type register 				offset: 0x04   		*/
	volatile uint32_t OSPEEDR;	/* output speed register				offset: 0x08   		*/
	volatile uint32_t PUPDR;	/* pull-up/pull-down register			offset: 0x0c    	*/
	volatile uint32_t IDR;		/* input data register					offset: 0x10    	*/
	volatile uint32_t ODR;		/* output data register					offset: 0x14    	*/
	volatile uint32_t BSRR;		/* bit set/reset register				offset: 0x18    	*/
	volatile uint32_t LCKR;		/* configuration lock regsiter	`		offset: 0x1c    	*/
	volatile uint32_t AFR[2];	/* [0] = LOW Reg, [1] = HIGH reg		offset: 0x20 + 0x24	*/

} GPIOx_Registers_t;

/* GPIOx address macros typecasted to register structure */ 

#define GPIOA					((GPIOx_Registers_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIOx_Registers_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIOx_Registers_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIOx_Registers_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIOx_Registers_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIOx_Registers_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIOx_Registers_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIOx_Registers_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIOx_Registers_t*)GPIOI_BASEADDR)
#define GPIOJ					((GPIOx_Registers_t*)GPIOJ_BASEADDR)
#define GPIOK					((GPIOx_Registers_t*)GPIOK_BASEADDR)

/* RCC member registers struct */

typedef struct {

	/* RCC info in DM (data manual), Section 6.3: RCC Registers */

	volatile uint32_t CR;			/* clock control register		  		offset: 0x00	*/
	volatile uint32_t PLL;			/* configuration register				offset: 0x04    */
	volatile uint32_t CFGR;			/* clock config register				offset: 0x08  	*/
	volatile uint32_t CIR;			/* clock interrupt register				offset: 0x0c   	*/
	volatile uint32_t AHB1RSTR;		/* peripheral reset register ahb1		offset: 0x10	!<value info @GPIO_REG_RESET_VALUES>*/
	volatile uint32_t AHB2RSTR;		/* peripheral reset register ahb2  		offset: 0x14	*/
	volatile uint32_t AHB3RSTR;		/*peri reset reg,[31,1] reserved			offset: 0x18  	*/
	volatile uint32_t RES_1;		/* reserved								offset: 0x1c	*/
	volatile uint32_t APB1RSTR;		/* peripheral reset register apb1		offset: 0x20  	*/
	volatile uint32_t APB2RSTR;		/* peripheral reset register apb2	  	offset: 0x24	*/
	volatile uint32_t RES_2;		/* reserved								offset: 0x28	*/
	volatile uint32_t RES_3;		/* reserved								offset: 0x2c	*/
	volatile uint32_t AHB1ENR;		/* peripheral clock register ahb1  		offset: 0x30	*/
	volatile uint32_t AHB2ENR;		/* peripheral clock enable reg			offset: 0x34	*/
	volatile uint32_t AHB3ENR;		/* peri clock reg,[31,1] reserved  		offset: 0x38	*/
	volatile uint32_t RES_4;		/* reserved								offset: 0x3c	*/
	volatile uint32_t APB1ENR;		/* peripheral clock enable reg  		offset: 0x40	*/
	volatile uint32_t APB2ENR;		/* peripheral clock enable reg  		offset: 0x44	*/
	volatile uint32_t RES_5;		/* reserved								offset: 0x48	*/
	volatile uint32_t RES_6;		/* reserved								offset: 0x4c	*/
	volatile uint32_t AHB1LPENR;	/* periph clk enable low power reg		offset: 0x50  	*/
	volatile uint32_t AHB2LPENR;	/* periph clk enable low power reg  	offset: 0x54	*/
	volatile uint32_t AHB3LPENR;	/* periph clk en lw pwr, [31,1] reserv	offset: 0x58  	*/
	volatile uint32_t RES_7;		/* reserved  							offset: 0x5c	*/
	volatile uint32_t APB1LPENR;	/* periph clk enable low power reg  	offset: 0x60	*/
	volatile uint32_t APB2LPENR;	/* periph clk enable low power reg  	offset: 0x64	*/
	volatile uint32_t RES_8;		/* reserved								offset: 0x68	*/
	volatile uint32_t RES_9;		/* reserved								offset: 0x6c	*/
	volatile uint32_t BDCR;			/* backup domain control register		offset: 0x70  	*/
	volatile uint32_t CSR;			/* clk control & status register		offset: 0x74  	*/
	volatile uint32_t RES_10;		/* reserved						  		offset: 0x78	*/
	volatile uint32_t RES_11;		/* reserved  							offset: 0x7c	*/
	volatile uint32_t SSCGR;		/* spread spectrum clk generation reg	offset: 0x80  	*/
	volatile uint32_t PLLI2SCFGR;	/*PLLI2S configuration regsiter   		offset: 0x84	*/
	volatile uint32_t PLLSAICFGR;	/*PLL configuration register   		offset: 0x88	*/
	volatile uint32_t DCKCFGR;		/* dedicated clock confif register  	offset: 0x8c	*/

} RCCRegisters_t;

typedef struct {
	
	volatile uint32_t EXTI_IMR;			/* interrupt mask register 		offset: 0x00	*/
	volatile uint32_t EXTI_EMR;			/* event mask register 			offset: 0x04	*/
	volatile uint32_t EXTI_RTSR;		/* rising trigger selection reg offset: 0x08	*/
	volatile uint32_t EXTI_FTSR;		/* falling trigger select reg	offset: 0x0c	*/
	volatile uint32_t EXTI_SWIER;		/* software interrupt event reg	offset: 0x10	*/
	volatile uint32_t EXTI_PR;			/* pending register 			offset: 0x14	*/ 

}EXTI_Registers_t;			/* Data manual, section 12.3 */ 


#define RCC				((RCCRegisters_t*)(RCCBASEADDR))
#define EXTI			((EXTI_Registers_t*)(EXTI_BASEADDR))
#define SYSCFG			((SYSCFG_Registers_t*)(SYSCFG_BASEADDR))


/*
 *	@GPIO_REG_RESET_VALUES
 *	@brief: Resets the GPIO port registers without keeping it in reset state. switches back to regular state. That is why there are two statements per function marco.
 *	GPIO register reset function macros
 */
 
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) 
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/* 
 * clock enable macros for GPIOx periphals 
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8) )




/* 
 * clock enable macros for SYSCFG peripheral 
 */

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))


/* 
 * GPIOx Peripheral Clock Disable macros
 */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 8) )

/* 
 * clock disable macros for SYSCFG peripheral 
 */

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/*
 *	some generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define HIGH			1
#define LOW				0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET
 
/*START**********SPI*****************************************************************************************/

/*
 *	SPI Register struct: Data Manual, Section 28.5.10
 *	@note: Each register is 16-bits, must be accessed half=word or word.
 */

typedef struct {

	uint32_t volatile SPI_CR1;			/* control register 	offset: 0x00	*/
	uint32_t volatile SPI_CR2;			/* control register 	offset: 0x04	*/
	uint32_t volatile SPI_SR;			/* status register  	offset: 0x08	*/
	uint32_t volatile SPI_DR;			/* data register      	offset: 0x0c	*/
	uint32_t volatile SPI_CRCPR;		/* CRC polynomial reg 	offset: 0x10  	*/
	uint32_t volatile SPI_RXCRCR;		/* RX CRC register		offset: 0x14	*/
	uint32_t volatile SPI_TXCRCR;		/* TX CRC register  	offset: 0x18	*/
	uint32_t volatile SPI_I2SCFGR;		/* config reg			offset: 0x1c	*/
	uint32_t volatile SPI_I2SPR;		/* prescaler reg		offset: 0x20	*/
	
}SPIx_Registers_t;

/*
 *	SPIx Base Addresses	
 */

#define SPI1		( (SPIx_Registers_t*)SPI1_BASEADDR )
#define SPI2		( (SPIx_Registers_t*)SPI2_BASEADDR )
#define SPI3		( (SPIx_Registers_t*)SPI3_BASEADDR )
#define SPI4		( (SPIx_Registers_t*)SPI4_BASEADDR )
#define SPI5		( (SPIx_Registers_t*)SPI5_BASEADDR )
#define SPI6		( (SPIx_Registers_t*)SPI6_BASEADDR )


/*
 *	SPI Peripheral Clock Enable Macro Functions
 *	Data Manual, Section 6.3.13, Section 6.3.14
 */

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13) )
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20) )
#define SPI6_PCLK_EN()			(RCC->APB2ENR |= (1 << 21) )

/*
 *	SPI Peripheral Clock Disable Macro Functions
 *	Data Manual, Section 6.3.13, Section 6.3.14
 */

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13) )
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 20) )
#define SPI6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 21) )

/*
 *	@SPI_REG_RESET function Macros. 
 *	@brief: Resets the bit fields in all the registers using the RCC.  
 *	
 */
 
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0) 
 
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)

/*
 *	@SPI_CR1 Bit position definition macros.
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3	// spans over bits [5:3]
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 *	@SPI_CR2 Bit position macros.
 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 *	@SPI_SR Bit position macros.
 * 	Data Manual Section 28.5.3
 */

#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3	// spans over bits [5:3]
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8

/*END**********SPI*****************************************************************************************/


/*START**********I2C*****************************************************************************************/

/*
 *	I2C Register struct: Data Manual, Section 27.6 
 *	@note: Each register is 16-bits, must be accessed half word or word.
 */

typedef struct {

	uint32_t volatile I2C_CR1;			/* Control Register 1		offset: 0x00	*/
	uint32_t volatile I2C_CR2;			/* Control Register 2		offset: 0x04	*/
	uint32_t volatile I2C_OAR1;			/* Own Address Register		offset: 0x08	*/
	uint32_t volatile I2C_OAR2;			/* Own Address Register 2	offset: 0x0C	*/
	uint32_t volatile I2C_DR;			/* Data Register			offset: 0x10	*/
	uint32_t volatile I2C_SR1;			/* Status Register 1		offset: 0x14	*/
	uint32_t volatile I2C_SR2;			/* Status Register 2		offset: 0x18	*/
	uint32_t volatile I2C_CCR;			/* Clock Control Register	offset: 0x1C	*/
	uint32_t volatile I2C_TRISE;		/* TRISE Register			offset: 0x20	*/
	uint32_t volatile I2C_FLTR;			/* FLTR Register			offset: 0x24	*/

}I2Cx_Registers_t;



/*
 *	I2Cx Base Addresses	
 */

#define I2C1		( (I2Cx_Registers_t*)I2C1_BASEADDR	)
#define I2C2		( (I2Cx_Registers_t*)I2C2_BASEADDR  )
#define I2C3		( (I2Cx_Registers_t*)I2C3_BASEADDR  )

/* 
 * clock enable macros for I2Cx peripherals 
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

/* 
 * clock disable macros for I2Cx peripherals 
 */

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))


 /*****I2C Register Bit Position Macros***********************************************/

/*
 * I2C Control Register 1 bit position macros
 */

#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
// bits [2] reserved
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
// bits [14] reserved
#define I2C_CR1_SWRST			15

/*
 * I2C Control Register 2 bit position macros
 */

#define I2C_CR2_FREQ			0	/* Occupies bits [5:0], 6-bits total */
// bits [7:6] reserved
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12
// bits [15:13] reserved

/*
 * I2C Status Register 1 bit position macros
 */

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
// [5] is reserved
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
// [13] is reserved
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/*
 * I2C Status Register 2 bit position macros
 */

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
// Bits [3] reserved
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8 /* PEC occupies bits [15:7], 8-bits */

/*
 * I2C Clock Control Register
 */

#define I2C_CCR_CCR				0 /* Occupies bits [11:0], 12 bits */
// bits [13:12] reserved
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15



/*END**********I2C*****************************************************************************************/

/*********USART**********START************************************************************************************/

/*
 *	Register structure for USARTx peripherals
 */

typedef struct
{
	uint32_t SR;         /*!< TODO,					Address offset: 0x00 */
	uint32_t DR;         /*!< TODO,					Address offset: 0x04 */
	uint32_t BRR;        /*!< TODO,					Address offset: 0x08 */
	uint32_t CR1;        /*!< TODO,					Address offset: 0x0C */
	uint32_t CR2;        /*!< TODO,					Address offset: 0x10 */
	uint32_t CR3;        /*!< TODO,    				Address offset: 0x14 */
	uint32_t GTPR;       /*!< TODO,    				Address offset: 0x18 */

} USARTx_Registers_t;

/*
 * USART base addresses
 * @USARTx_Reg_Values
 */

#define USART1		((USARTx_Registers_t*)USART1_BASEADDR)
#define USART2		((USARTx_Registers_t*)USART2_BASEADDR)
#define USART3		((USARTx_Registers_t*)USART3_BASEADDR)
#define USART6		((USARTx_Registers_t*)USART6_BASEADDR)

/*
 * UART base addresses
 * @USARTx_Reg_Values
 */

#define UART4		((USARTx_Registers_t*)UART4_BASEADDR)
#define UART5		((USARTx_Registers_t*)UART5_BASEADDR)


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 * clock enable macros for UARTx peripherals
 */

#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 5))

/*
 * clock enable macros for UARTx peripherals
 */
#define UART4_PCLK_DIS()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()			(RCC->APB1ENR &= ~(1 << 20))

/*
 * Bit position definitions USART_SR
 * Bits [31:10] are reserved
 */
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

/*
 * Bit position definition macros of USART CR1
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU 			1
#define USART_CR1_RE  			2
#define USART_CR1_TE 			3
#define USART_CR1_IDLEIE 		4
#define USART_CR1_RXNEIE  		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE 			8
#define USART_CR1_PS 			9
#define USART_CR1_PCE 			10
#define USART_CR1_WAKE  		11
#define USART_CR1_M 			12
#define USART_CR1_UE 			13
/* bit [14] is reserved */
#define USART_CR1_OVER8  		15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0 // occupies bits [0:3]
/* bit [4] is reserved  */
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
/* bit [7] is reserved  */
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12 // occupies bits [13:12]
#define USART_CR2_LINEN   				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11
/* bits [31:12] are reserved */

/*
 * relevant header files.
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_usart_driver.h"


/*END**********USART*****************************************************************************************/

#endif /*INC_STM32F407XX_H_ */
