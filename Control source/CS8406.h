/*
 * CS8406.h
 *
 *  Created on: 26 џэт. 2017 у.
 *      Author: gavrilov.iv
 */

#ifndef CS8406_H_
#define CS8406_H_

#include <stdint.h>

#define		WRITE_ADDR_CS8406	0x20
#define		READ_ADDR_CS8406	0x21


/* Control port register address */
#define REG_ADDR_CONTROL_1			0x01
#define REG_ADDR_CONTROL_2			0x02
#define REG_ADDR_DATA_FLOW_CTRL		0x03
#define REG_ADDR_CLOCK_SOURCE_CTRL	0x04
#define REG_ADDR_SERIAL_INPUT_FMRM	0x05
#define REG_ADDR_INT_1_STATUS		0x07
#define REG_ADDR_INT_2_STATUS		0x08
#define REG_ADDR_INT_1_MASK			0x09
#define REG_ADDR_INT_1_MODE_MSB		0x0A
#define REG_ADDR_INT_1_MODE_LSB		0x0B
#define REG_ADDR_INT_2_MASK			0x0C
#define REG_ADDR_INT_2_MODE_MSB		0x0D
#define REG_ADDR_INT_2_MODE_LSB		0x0E
#define REG_ADDR_CS_DATA_BUFF_CTRL	0x12
#define REG_ADDR_U_DATA_BUFF_CTRL	0x13
#define REG_ADDR_ID_VERSION			0x7F

/* Register struct */
struct CS8406_Registers
{
	union
	{
		struct	// Bits order: first LSB bits
		{
			uint8_t RES0 : 4;
			uint8_t CLK : 2;
			uint8_t RUN : 1;
			uint8_t RES1 : 1;
		} CLK_SOURCE_CTRL;
		uint8_t REG_04;
	};
	union
	{
		struct
		{
			uint8_t SILRPOL :1;
			uint8_t SISPOL :1;
			uint8_t SIDEL :1;
			uint8_t SIJUST :1;
			uint8_t SIRES :2;
			uint8_t SISF :1;
			uint8_t SIMS :1;
		}SER_IN_Format;
		uint8_t REG_05;
	};
} CS8406_Registers;

void CS8406_Init();

#endif /* CS8406_H_ */
