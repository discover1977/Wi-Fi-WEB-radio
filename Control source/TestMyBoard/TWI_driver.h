/*
 * TWI_driver.h
 *
 *  Created on: 30.05.2017
 *      Author: Gavrilov I.
 */

#ifndef TWI_DRIVER_H_
#define TWI_DRIVER_H_

//#define HW

#ifdef HW
	#include "TWI_HW_Master.h"
#else
	#include "TWI_SW_Master.h"
#endif

// Init Bus	
void TWI_Init();
// Start Transfer
void TWI_Start();
// Stop Transfer
void TWI_Stop();
// Write Transfer
unsigned int TWI_Write(unsigned int a);
// Read Transfer
unsigned int TWI_Read(unsigned int ack);

// Read Byte
unsigned int TWI_ReadByte(unsigned int nAddress, unsigned int nRegister);
// Write Byte
void TWI_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue);

#endif
