/*
 * main.c
 *
 *  Created on: May 28, 2022
 *      Author: jordanrainey
 */

#include "stm32f407xx.h"




int main(void) {

	
	return 0;
}

void EXTI0_IRQHandler(void) {
	GPIO_IRQHandling(0);
}

