#include "rtc.h"

#include "i2c-soft.h"

unsigned char BcdToBin(unsigned char Bcd)
{
	return ( ( ( Bcd >> 4 ) * 10 ) + ( Bcd & 0x0F ) );
}

unsigned char BinToBcd(unsigned char Bin)
{
	return ( ( Bin / 10 ) * 0x10 ) + ( Bin % 10 );
}

unsigned char rtc_read(unsigned char address)
{
	unsigned char data;
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(address);
	i2c_Start();
	i2c_Write(BASE_ADDR_RD);
	data = i2c_Read( NO_I2C_ACK );
	i2c_Stop();
	return data;
}

void rtc_write( unsigned char address, unsigned char data )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(address);
	i2c_Write(data);
	i2c_Stop();
}

void rtc_init ( unsigned char initValue )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(CONTROL_ADDR);
	i2c_Write(initValue);
	i2c_Stop();
}

void rtc_get_time( unsigned char *hour, unsigned char *min ,unsigned char *sec )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(SECONDS_ADDR);
	i2c_Start();
	i2c_Write(BASE_ADDR_RD);
	*sec=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*min=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*hour=BcdToBin( i2c_Read( NO_I2C_ACK ) );
	i2c_Stop();
}

void rtc_set_time( unsigned char hour, unsigned char min, unsigned char sec )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(SECONDS_ADDR);
	i2c_Write( BinToBcd( sec ) );
	i2c_Write( BinToBcd( min ) );
	i2c_Write( BinToBcd( hour ) );
	i2c_Stop();
}

void rtc_get_date( unsigned char *date, unsigned char *month, unsigned char *year )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(DATE_ADDR);
	i2c_Start();
	i2c_Write(BASE_ADDR_RD);
	*date=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*month=BcdToBin( i2c_Read( OK_I2C_ACK ) );
	*year=BcdToBin( i2c_Read( NO_I2C_ACK ) );
	i2c_Stop();
}

void rtc_set_date( unsigned char date, unsigned char month, unsigned char year )
{
	i2c_Start();
	i2c_Write(BASE_ADDR_WR);
	i2c_Write(DATE_ADDR);
	i2c_Write( BinToBcd( date ) );
	i2c_Write( BinToBcd( month ) );
	i2c_Write( BinToBcd( year ) );
	i2c_Stop();
}


#ifdef DS323x

union rtc_temp
{
	unsigned char b[2];
	signed int w;
};

TEMP_RET_TYPE rtc_get_temperature()
{
	union rtc_temp tt;
	i2c_Start();
	i2c_Write( BASE_ADDR_WR );
	i2c_Write( TEMP_MSB_ADDR );
	i2c_Start();
	i2c_Write( BASE_ADDR_RD );
	tt.b[1] = i2c_Read( OK_I2C_ACK );
	tt.b[0] = i2c_Read( NO_I2C_ACK );
	i2c_Stop();
#ifdef FLOAT_TEMPERATURE
	return tt.w / 256.0;
#else
	return tt.w / 256;
#endif
}
#endif
