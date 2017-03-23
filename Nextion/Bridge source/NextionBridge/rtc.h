
#ifndef _RTC_
#define _RTC_

//#define DS1307
#define DS323x

#define BASE_ADDR_WR	0xD0
#define BASE_ADDR_RD	0xD1

#define SECONDS_ADDR	0x00
#define MINUTES_ADDR	0x01
#define HOURS_ADDR		0x02
#define DAY_ADDR		0x03
#define DATE_ADDR		0x04
#define MONS_ADDR		0x05
#define YEAR_ADDR		0x06

#ifdef DS1307

#define CONTROL_ADDR	0x07

#define CTRL_OUT		7
#define CTRL_SQWE		4
#define CTRL_RS1		1
#define CTRL_RS0		0
#define CTRL_RS			0

#endif

#ifdef DS323x

#define CONTROL_ADDR	0x0E
#define TEMP_MSB_ADDR	0x11
#define TEMP_LSB_ADDR	0x12

#define CTRL_EOSC		7
#define CTRL_BBSQW		6
#define CTRL_CONV		5
#define CTRL_INTCN		2
#define CTRL_A2IE		1
#define CTRL_A1IE		0

//#define FLOAT_TEMPERATURE

#endif

unsigned char rtc_read(unsigned char address);
void rtc_write(unsigned char address, unsigned char data);
void rtc_init(unsigned char initValue);
void rtc_get_time(unsigned char *hour,unsigned char *min,unsigned char *sec);
void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec);
void rtc_get_date(unsigned char *date,unsigned char *month,unsigned char *year);
void rtc_set_date(unsigned char date,unsigned char month,unsigned char year);

#ifdef DS323x

#ifdef FLOAT_TEMPERATURE
	#define TEMP_RET_TYPE float
#else
	#define TEMP_RET_TYPE int
#endif

TEMP_RET_TYPE rtc_get_temperature();
#endif

#endif

