/*
 * CS8406.c
 *
 *  Created on: 26 џэт. 2017 у.
 *      Author: gavrilov.iv
 */

#include "CS8406.h"
#include "i2c-soft.h"




void CS8406_Init()
{
	i2c_Init();

	i2c_Start();
	i2c_Write(WRITE_ADDR_CS8406);
	i2c_Write(REG_ADDR_ID_VERSION);
	i2c_Start();
	i2c_Write(READ_ADDR_CS8406);
	i2c_Read(0);
	i2c_Stop();

	CS8406_Registers.CLK_SOURCE_CTRL.RUN = 1;
	CS8406_Registers.CLK_SOURCE_CTRL.CLK = 3;
	i2c_Start();
	i2c_Write(WRITE_ADDR_CS8406);
	i2c_Write(REG_ADDR_CLOCK_SOURCE_CTRL);
	i2c_Write(CS8406_Registers.REG_04);
	i2c_Stop();

	CS8406_Registers.SER_IN_Format.SIMS = 0;
	CS8406_Registers.SER_IN_Format.SISF = 0;
	CS8406_Registers.SER_IN_Format.SIRES = 0;		// 2 bits
	CS8406_Registers.SER_IN_Format.SIJUST = 0;
	CS8406_Registers.SER_IN_Format.SIDEL = 1;
	CS8406_Registers.SER_IN_Format.SISPOL = 0;
	CS8406_Registers.SER_IN_Format.SILRPOL = 0;

	i2c_Start();
	i2c_Write(WRITE_ADDR_CS8406);
	i2c_Write(REG_ADDR_SERIAL_INPUT_FMRM);
	i2c_Write(CS8406_Registers.REG_05);
	i2c_Stop();
}

