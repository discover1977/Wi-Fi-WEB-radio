/*
 * i2c-soft.h
 *
 *  Created on: 19.12.2009
 *      Author: Pavel V. Gololobov
 */

#ifndef TWI_SW_SW_H_
#define TWI_SW_SW_H_

#include <avr/io.h>

	#define NO_TWI_SW_ACK 0
	#define OK_TWI_SW_ACK 1

	#ifndef SDA
	#define I2COUT      PORTC	// Write to Port
	#define I2CIN       PINC	// Read from Port
	#define I2CDIR      DDRC	// Set Port Direction
	#define I2CSEL      PORTC	// Alternative Port Fuctions
	
	#define SDA       	(1 << 4)	// Serial Data Line
	#define SCL       	(1 << 5)	// Serial Clock Line
	#endif

// Init Bus	
void TWI_SW_Init();
// Start Transfer
void TWI_SW_Start();
// Stop Transfer
void TWI_SW_Stop();
// Write Transfer
unsigned int TWI_SW_Write(unsigned int a);
// Read Transfer
unsigned int TWI_SW_Read(unsigned int ack);

// Read Byte
unsigned int TWI_SW_ReadByte(unsigned int nAddress, unsigned int nRegister);
// Write Byte
void TWI_SW_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue);

#endif
