#include "TWI_driver.h"

// Init Bus
void TWI_Init()
{
#ifdef HW
	
#else
	i2c_Init();
#endif
}

// Start Transfer
void TWI_Start()
{
#ifdef HW

#else
	i2c_Start();
#endif
}

// Stop Transfer
void TWI_Stop()
{
#ifdef HW

#else
	i2c_Stop();
#endif
}

// Write Transfer
unsigned int TWI_Write(unsigned int a)
{
#ifdef HW

#else
	unsigned int i2c_Write(unsigned int a);
#endif
}

// Read Transfer
unsigned int TWI_Read(unsigned int ack)
{
#ifdef HW

#else
	unsigned int i2c_Read(unsigned int ack);
#endif
}

// Read Byte
unsigned int TWI_ReadByte(unsigned int nAddress, unsigned int nRegister)
{
#ifdef HW

#else
	unsigned int i2c_ReadByte(unsigned int nAddress, unsigned int nRegister);
#endif
}

// Write Byte
void TWI_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue)
{
#ifdef HW

#else
	void i2c_WriteByte(unsigned int nAddress, unsigned int nRegister, unsigned int nValue)
#endif
}