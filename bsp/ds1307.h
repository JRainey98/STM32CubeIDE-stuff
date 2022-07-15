/*
 * ds1307.h
 *
 *  Created on: Jul 1, 2022
 *      Author: jordanrainey
 */

/* Notes
 *
 * DS 3231 uses BCD for writing and reading data.
 *
 *
 */
#ifndef DS1307_H_
#define DS1307_H_

#include<stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

/* Application Configurable Items */
#define DS3231_I2C				I2C2
#define DS3231_I2C_GPIO_PORT	GPIOB
#define DS3231_I2C_SDA_PIN		GPIO_PIN_11
#define DS3231_I2C_SCL_PIN		GPIO_PIN_10
#define DS3231_I2C_SPEED		I2C_SCL_SPEED_SM	//SM = "Standard Mode"
#define DS3231_I2C_PUPD			GPIO_PIN_PU		// use internal pullup for SDA and SCL lines.


/* Register Addresses */
#define DS3231_ADDR_SEC		0x00
#define DS3231_ADDR_MIN		0x01
#define DS3231_ADDR_HRS		0x02
#define DS3231_ADDR_DAY		0x03
#define DS3231_ADDR_DATE	0x04
#define DS3231_ADDR_MONTH	0x05
#define DS3231_ADDR_YEAR	0x06

#define DS3231_ADDR_SR		0x0F	// status register (SR)

#define TIME_FORMAT_12HRS_AM	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS		2

#define DS3231_SLAVE_ADDR		0x68	// 7-bit address = 1101000, followed by R/W that is put in by the I2C Send/recieve Functions.

#define SUNDAY			1
#define MONDAY			2
#define TUESDAY			3
#define WEDNESDAY		4
#define THURSDAY		5
#define FRIDAY			6
#define SATURDAY		7



typedef struct {

	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;

}RTC_date_t;

typedef struct {

	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;

}RTC_time_t;

/*
 * fucntion prototypes
 */

void DS3231_init(void);
void DS3231_Set_Current_Time(RTC_time_t* );
void DS3231_Get_Current_Time(RTC_time_t* );

void DS3231_Set_Current_Date(RTC_date_t* );
void DS3231_Get_Current_Date(RTC_date_t* );




uint8_t DS3231_Read(uint8_t reg_addr);
void DS3231_Write(uint8_t value, uint8_t reg_addr);


#endif /* DS1307_H_ */


/*
 * Register Addresses
 */


