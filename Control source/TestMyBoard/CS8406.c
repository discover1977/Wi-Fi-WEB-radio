/*
 * CS8406.c
 *
 *  Created on: 26 џэт. 2017 у.
 *      Author: gavrilov.iv
 */

#include "CS8406.h"
#include "TWI_SW_Master.h"

void CS8406_Init()
{
	TWI_SW_Init();

	TWI_SW_Start();
	TWI_SW_Write(WRITE_ADDR_CS8406);
	TWI_SW_Write(REG_ADDR_ID_VERSION);
	TWI_SW_Start();
	TWI_SW_Write(READ_ADDR_CS8406);
	TWI_SW_Read(0);
	TWI_SW_Stop();

	CS8406_Registers.CLK_SOURCE_CTRL.RUN = 1;
	CS8406_Registers.CLK_SOURCE_CTRL.CLK = 3;
	TWI_SW_Start();
	TWI_SW_Write(WRITE_ADDR_CS8406);
	TWI_SW_Write(REG_ADDR_CLOCK_SOURCE_CTRL);
	TWI_SW_Write(CS8406_Registers.REG_04);
	TWI_SW_Stop();

	CS8406_Registers.SER_IN_Format.SIMS = 0;
	CS8406_Registers.SER_IN_Format.SISF = 0;
	CS8406_Registers.SER_IN_Format.SIRES = 0;		// 2 bits
	CS8406_Registers.SER_IN_Format.SIJUST = 0;
	CS8406_Registers.SER_IN_Format.SIDEL = 1;
	CS8406_Registers.SER_IN_Format.SISPOL = 0;
	CS8406_Registers.SER_IN_Format.SILRPOL = 0;

	TWI_SW_Start();
	TWI_SW_Write(WRITE_ADDR_CS8406);
	TWI_SW_Write(REG_ADDR_SERIAL_INPUT_FMRM);
	TWI_SW_Write(CS8406_Registers.REG_05);
	TWI_SW_Stop();
}

