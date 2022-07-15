/*
 * ds1307.c
 *
 *  Created on: Jul 1, 2022
 *      Author: jordanrainey
 */

#include "ds1307.h"
#include <string.h>
#include <stdint.h>
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

static void DS3231_I2C_Pin_Config(void);
static void DS3231_I2C_Config(void);
void DS3231_Write(uint8_t value, uint8_t reg_addr);
uint8_t DS3231_Read(uint8_t reg_addr);

I2C_Handle_t g_DS3231_I2C_Handle;	// g = "global"

/*
 *	@fn			- DS3231_init()
 *
 *	@brief		- Initializes the approprite pins and settings for I2C Communication
 *				  between MCU and DS3231 peripheral.
 *
 *	@param		- none
 *
 *	@return		- none
 *
 *	@note		- Changes to initialzation should be made in DS3231 .h file.
 */
void DS3231_init(void) {



	//1.) Initialize the i2c pins
	DS3231_I2C_Pin_Config();

	// 2.) initialize the i2c peripheral
	DS3231_I2C_Config();

	// 3.) Enable the I2C Peripheral
	I2C_PeripheralControl((I2Cx_Registers_t*)DS3231_I2C, ENABLE);

	// DS3231 oscillator is always when powered by Vcc power.
	DS3231_Write(0x00, DS3231_ADDR_SEC);

}

/*
 *	@fn			- DS3231_Get_Current_Time(RTC_time_t *rtc_time)
 *
 *	@brief		- Gets and stores seconds, minutes, hours, and hours format into
 *				  member elements of RTC_time_t structure.
 *
 *	@param		- RTC_Time_t struct address
 *
 *	@return		- none
 *
 *	@note		- Register map in DS3231 Manual, Pg. 11
 */

static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m, n;
	uint8_t bcd;

	bcd = value;
	if(value >= 10)
	{
		m = value /10;
		n = value % 10;
		bcd = (m << 4) | n ;
	}

	return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m , n;
	m = (uint8_t) ((value >> 4 ) * 10);
	n =  value & (uint8_t)0x0F;
	return (m+n);
}


void DS3231_Set_Current_Time(RTC_time_t *rtc_time) {

	uint8_t seconds, hrs;
	seconds = binary_to_bcd(rtc_time->seconds);

	//seconds &= ~( 1 << 7);

	DS3231_Write(seconds, DS3231_ADDR_SEC);

	DS3231_Write(binary_to_bcd(rtc_time->minutes), DS3231_ADDR_MIN);

	hrs = binary_to_bcd(rtc_time->hours);

	if(rtc_time->time_format == TIME_FORMAT_24HRS){
		hrs &= ~(1 << 6);
	}else{
		hrs |= (1 << 6);
		hrs = (rtc_time->time_format  == TIME_FORMAT_12HRS_PM) ? hrs | ( 1 << 5) :  hrs & ~( 1 << 5) ;
	}

	DS3231_Write(hrs,DS3231_ADDR_HRS);

}

/*
 *	@fn			- DS3231_Get_Current_Time(RTC_time_t *rtc_time)
 *
 *	@brief		- Gets and stores seconds, minutes, hours, and hours format into
 *				  member elements of RTC_time_t structure.
 *
 *	@param		- RTC_Time_t struct address
 *
 *	@return		- none
 *
 *	@note		- Register map in DS3231 Manual, Pg. 11
 */
void DS3231_Get_Current_Time(RTC_time_t *rtc_time) {

//	rtc_time->seconds = bcd_to_binary(DS3231_Read(DS3231_ADDR_SEC));
//	rtc_time->minutes = bcd_to_binary(DS3231_Read(DS3231_ADDR_MIN));
//
//	// 24hr, 12hrAM, or 12hrPM.
//	uint8_t hrs = DS3231_Read(DS3231_ADDR_HRS);
//	if (hrs & (1 << 6)) {
//		rtc_time->time_format = TIME_FORMAT_24HRS;
//	}
//	else {
//		// 12-hr format. 1 == PM, 0 == AM.
//		rtc_time->time_format = ( (hrs >> 5) & 0x01);
//	}
//
//	rtc_time->hours = bcd_to_binary(hrs);

	uint8_t seconds,hrs;

	seconds = DS3231_Read(DS3231_ADDR_SEC);

	//seconds &= ~( 1 << 7); not needed

	rtc_time->seconds = bcd_to_binary(seconds);
	rtc_time->minutes = bcd_to_binary(DS3231_Read(DS3231_ADDR_MIN));

	hrs = DS3231_Read(DS3231_ADDR_HRS);
	if(hrs & ( 1 << 6)){
		//12 hr format
		rtc_time->time_format =  !((hrs & ( 1 << 5)) == 0) ;
		hrs &= ~(0x3 << 5);//Clear 6 and 5
	}else{
		//24 hr format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}

	rtc_time->hours = bcd_to_binary(hrs);

}

void DS3231_Set_Current_Date(RTC_date_t *rtc_date) {


	DS3231_Write(binary_to_bcd(rtc_date->day), DS3231_ADDR_DAY);
	DS3231_Write(binary_to_bcd(rtc_date->date), DS3231_ADDR_DATE);
	DS3231_Write(binary_to_bcd(rtc_date->month), DS3231_ADDR_MONTH);
	DS3231_Write(binary_to_bcd(rtc_date->year), DS3231_ADDR_YEAR);

}
void DS3231_Get_Current_Date(RTC_date_t *rtc_date) {

	// (day) is the software defined WORD day (i.e. Monday, Tuesday, Wednesday, etc)
	rtc_date->day = bcd_to_binary(DS3231_Read(DS3231_ADDR_DAY));
	// (date) is the NUMBER day of the month 1-31.
	rtc_date->date = bcd_to_binary(DS3231_Read(DS3231_ADDR_DATE));
	rtc_date->month = bcd_to_binary(DS3231_Read(DS3231_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(DS3231_Read(DS3231_ADDR_YEAR));

}

// Helper function, configures the i2c pins for it
static void DS3231_I2C_Pin_Config(void) {

	GPIO_Handle_t i2c_sda, i2c_scl;

	/* Set all member elements to 0 */
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C1_SCL ==> PB6
	 * I2C1_SDA ==> PB7
	 */

	i2c_sda.pGPIOx = DS3231_I2C_GPIO_PORT; // should be GPIOB
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS3231_I2C_PUPD; 	// Should use internal pullup for our project
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS3231_I2C_SDA_PIN;		// should be PB7
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(&i2c_sda);	// Also enables RCC for GPIO Port


	i2c_scl.pGPIOx = DS3231_I2C_GPIO_PORT; // should be GPIOB
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS3231_I2C_PUPD; 	// Should use internal pullup for our project
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS3231_I2C_SCL_PIN;		// should be PB7
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS3231_I2C_SCL_PIN;		// should be PB6;

	GPIO_Init(&i2c_scl);

}

/*
 *	@fn			- DS3231_I2C_Config()
 *
 *	@brief		- helper function that initializes MCU for I2C communcation with
 *				  DS3231 RTC peripheral.
 *
 *	@param		- none
 *
 *	@return		- none
 */
static void DS3231_I2C_Config(void) {

	g_DS3231_I2C_Handle.pI2Cx = I2C2;
	g_DS3231_I2C_Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	g_DS3231_I2C_Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	// below doesn't really matter what we choose, we aren't in fast mode (FM)
	g_DS3231_I2C_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	g_DS3231_I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_ST;

	I2C_Init(&g_DS3231_I2C_Handle);
}

/*
 *	@fn			- DS3231_Write(uint8_t value, uint8_t reg_addr)
 *
 *	@brief		- Starts communication with DS3231 by sending start, slave address, then
 *				  it sends the word address.
 *
 *	@param		- value that you want to be written
 *	@param		- Address of register in DS3231 you want alter (read the Ref Manual for them)
 *
 *	@return		- none
 *	@note		- "Figure 3. Data Write--Slave Recieve Mode" in DS3231 RM pg. 16.
 */
void DS3231_Write(uint8_t value, uint8_t reg_addr) {

	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&g_DS3231_I2C_Handle, tx, 2, DS3231_SLAVE_ADDR, 0);
}

/*
 *	@fn			- DS3231_Read(uint8_t reg_addr)
 *
 *	@brief		- Reads from reg_addr of the DS3231 peripheral.
 *
 *	@param		- DS3231 register addr to read from.
 *
 *	@return		- 8-bit data from reg_addr for DS3231
 *	@note		- "Figure 5. Data Write/Read -- Slave recieve then transmit" DS3231 RF pg. 17
 */


uint8_t DS3231_Read(uint8_t reg_addr) {

	uint8_t data;

	// sends slave address, adds write bit, gets ACK back.

	/*
	 * Question: Why don't we send over word again?
	 * on pg. 17 it says we don't need to if a write already occured.
	 */
	I2C_MasterSendData(&g_DS3231_I2C_Handle, &reg_addr, 1, DS3231_SLAVE_ADDR, 0);
	// Now we just recieve a single byte.
	I2C_MasterReceiveData(&g_DS3231_I2C_Handle, &data, 1, DS3231_SLAVE_ADDR, 0);
	return data;

}











