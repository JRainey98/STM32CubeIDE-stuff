/*
 * 017_rtc_lcd.c
 *
 *  Created on: Jul 1, 2022
 *      Author: jordanrainey
 */
#include <stdint.h>
#include <stdio.h>
#include "ds1307.h"

char* get_day_of_week(uint8_t day);
char* time_to_string(RTC_time_t *rtc_time);
void number_to_string(uint8_t num , char* buf);
char* date_to_string(RTC_date_t *rtc_date);
void GPIO_ButtonInit(void);

#define SYSTICK_TIM_CLK   16000000UL

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

int main(void) {

 	initialise_monitor_handles();

	RTC_time_t currentTime;
	RTC_date_t currentDate;
	printf("RTC Test\n");

	//init_systick_timer(1); // 1 systick interrupt per second. enables the interrupt



	// the code below is not applicable to the DS3231, only the DS1307, which is  obsolete hardware.
//	if (DS3231_init()) {
//
//		printf("RTC has failed\n");
//		while(1);
//
//	}

	// don't put the below here.

	// Check the validity of the data.

	I2C_PeriClockControl(I2C1, ENABLE);
	GPIO_ButtonInit();
//	while( ! GPIO_ReadFromInputPin( GPIOA,GPIO_PIN_0) );
/********************START****************/
	DS3231_init();
	I2C_PeripheralControl(I2C2, ENABLE);

	uint8_t value = (uint8_t)(~(1 << 7)); // makes rue the bit is set to 0.
	DS3231_Write(value, DS3231_ADDR_SR); // status register (SR), clears the OSF flag.
	value = (uint8_t)(~(1 << 3));			// clears the 32Khz enable bit
	DS3231_Write(value, DS3231_ADDR_SR);

	uint8_t temp = DS3231_Read(0x0E);
	printf("0x%x\n", temp);
	temp = DS3231_Read(0x0F);
	printf("0x%x\n", temp);


	currentDate.day = FRIDAY;
	currentDate.date = 15;
	currentDate.month = 1;
	// year is only last two digits, can't do 2022, etc.
	currentDate.year = 21;

	currentTime.hours = 4;
	currentTime.minutes = 25;
	currentTime.seconds = 41;
	currentTime.time_format = TIME_FORMAT_12HRS_PM;

	DS3231_Set_Current_Date(&currentDate);
	DS3231_Set_Current_Time(&currentTime);
	// I think the error happens up above?

	DS3231_Get_Current_Date(&currentDate);
	DS3231_Get_Current_Time(&currentTime);

	// print time based on time format;
	char *am_pm;
	if (currentTime.time_format != TIME_FORMAT_24HRS) {
		//print with am/pm details. 0 == "AM", 1 == "PM"
		am_pm = ((currentTime.time_format == 1) ? "PM" : "AM");

		printf("Current Time = %s %s\n", time_to_string(&currentTime), am_pm); // 4:25:41 PM

	}
	else {
		// AM/PM not needed.
		printf("Current Time = %s\n", time_to_string(&currentTime)); // 4:25:41
	}

	// 15/01/21 <friday

	printf("Current Date = %s <%s>\n", date_to_string(&currentDate), get_day_of_week(currentDate.day));



	// busy loop
	while(1) {
		for (uint32_t i = 0; i < 250000; i++) {

			// do nothing.

		}
	}
	return 0;
}

/*
 * helper function
 */

char* get_day_of_week(uint8_t day) {


	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
	// day of the week macros start at 1 == "SUNDAY".
	return days[day - 1];

}


// HH:MM:SS
char* time_to_string(RTC_time_t *rtc_time) {

	static char buf[9]; // static to avoid dangling pointer
	buf[2] = ':';
	buf[5] = ':';
	number_to_string(rtc_time->hours, &buf[0]);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;

}

void number_to_string(uint8_t num , char* buf)
{

	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48;
		buf[1]= (num % 10) + 48;
	}
}

char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date,buf);
	number_to_string(rtc_date->month,&buf[3]);
	number_to_string(rtc_date->year,&buf[6]);

	buf[8]= '\0';

	return buf;
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	DS3231_Get_Current_Time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n",time_to_string(&current_time),am_pm); // 04:25:41 PM
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif

	}else{
#ifndef PRINT_LCD
		printf("Current time = %s\n",time_to_string(&current_time)); // 04:25:41
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	DS3231_Get_Current_Date(&current_date);

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_print_char('>');
#endif

}


