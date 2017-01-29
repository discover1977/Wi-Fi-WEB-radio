/*
 * main.c
 *
 *  Created on: 26 џэт. 2017 у.
 *      Author: gavrilov.iv
 */

#include <avr/io.h>
#include <util/delay.h>
#include "bits_macros.h"
#include "i2c-soft.h"
#include "CS8406.h"

int main() {

	_delay_ms(1000);
	CS8406_Init();
	while(1) {

	}
}
