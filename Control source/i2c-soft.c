/*
 * i2c-soft.c
 *
 *  Created on: 19.12.2009
 *      Author: Pavel V. Gololobov
 */
#include <avr/io.h>
#include "i2c-soft.h"

//------------------------------------------------------------------
// I2C Speed Down
//------------------------------------------------------------------
#define I2CWAIT i2c_Wait(1);

//------------------------------------------------------------------
// I2C Delay
//------------------------------------------------------------------
void i2c_Wait(unsigned int n)
{
	do
	{
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
	while(--n);
}

//------------------------------------------------------------------
// I2C SDA SCL Control
//------------------------------------------------------------------
static void SetLowSDA()
{
	I2CDIR |= SDA;
	I2CWAIT
}
static void SetHighSDA()
{
	I2CDIR &= ~SDA;
	I2CWAIT
}

static void SetLowSCL()
{
	I2COUT &= ~SCL;
	I2CWAIT
}
static void SetHighSCL()
{
	I2COUT |= SCL;
	I2CWAIT
}

//------------------------------------------------------------------
// I2C Initialize Bus
//------------------------------------------------------------------
void i2c_Init()
{
	I2CSEL &= ~SDA;
	I2CSEL &= ~SCL;

	I2COUT &= ~SCL;
	I2COUT &= ~SDA;

	I2CDIR |= SCL;
	I2CDIR &= ~SDA;

	SetHighSCL();
	SetLowSDA();
	SetHighSDA();
}

//------------------------------------------------------------------
// I2C Start Data Transfer
//------------------------------------------------------------------
void i2c_Start()
{
	SetHighSCL();
	SetHighSDA();

	SetHighSCL();
	SetLowSDA();

	SetLowSCL();
	SetHighSDA();
}

//------------------------------------------------------------------
// I2C Stop  Transfer
//------------------------------------------------------------------
void i2c_Stop()
{
	SetLowSCL();
	SetLowSDA();

	SetHighSCL();
	SetLowSDA();

	SetHighSCL();
	SetHighSDA();
}

//------------------------------------------------------------------
// I2C Write  Transfer
//------------------------------------------------------------------
unsigned int i2c_Write(unsigned int a)
{
	unsigned int i;
	unsigned int return_ack;

	for (i = 0; i < 8; i++)
    {
		SetLowSCL();
		if (a & 0x80)
			SetHighSDA();
		else
			SetLowSDA();

		SetHighSCL();
		a <<= 1;
	}
	SetLowSCL();

	/* ack Read */
	SetHighSDA();
	SetHighSCL();

	if (I2CIN & SDA)
		return_ack = NO_I2C_ACK;
	else
		return_ack = OK_I2C_ACK;
	
    SetLowSCL();

	return (return_ack);
}

//------------------------------------------------------------------
// I2C Read  Transfer
//------------------------------------------------------------------
unsigned int i2c_Read(unsigned int ack)
{
	unsigned int i;
	unsigned int caracter = 0x00;

	SetLowSCL();
	SetHighSDA();

	for (i = 0; i < 8; i++)
    {
		caracter = caracter << 1;
		SetHighSCL();
		if (I2CIN & SDA)
			caracter = caracter  + 1;
        
		SetLowSCL();
	}

	if (ack)
    {
		SetLowSDA();
	}
	SetHighSCL();
	SetLowSCL();

	return (caracter);
}

//------------------------------------------------------------------
// I2C Read Byte
//------------------------------------------------------------------
unsigned int i2c_ReadByte(unsigned int nAddress, unsigned int nRegister)
{
	i2c_Start();
	i2c_Write(nAddress);
	i2c_Write(nRegister);
	i2c_Start();
	i2c_Write(nAddress | 0x01);
	uint8_t ret = i2c_Read(0);
	i2c_Stop();

	return ret;
}

//------------------------------------------------------------------
// I2C Write Byte
//------------------------------------------------------------------
void i2c_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue)
{
	i2c_Start();
	i2c_Write(nAddress);
	i2c_Write(nRegister);
	i2c_Write(nValue);
	i2c_Stop();
}

