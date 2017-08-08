/* This source file is part of the ATMEL QTouch Library 5.0.8 */


/* == include files ========================================================== */
//#include <avr/sleep.h>
#include "touch_api_ptc.h"
#include "bits_macros.h"
#include "ssd1306.h"
#include "median.h"
#include "buttons.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define CS8406_USE	1
#define DEBUG		0

#if CS8406_USE
	#include "CS8406.h"
#endif

#define N_ELEMENTS(X) (sizeof(X)/sizeof(*(X)))

/* == #defines ================================================================ */
#define ON		1
#define OFF		0
#define DEF_MCU_CLOCK_PRESCALER		0u
#define DEF_TIME_PERIOD_1MSEC       32u
#define DEF_TIME_PERIOD_10MSEC		0x1AFF
#define ACC_SIZE					4
#define AccSizeMacro(val)			( 1 << val )
#define SET_VOLUME_STOP				600
#define USART1_RX_BUFFER_SIZE		128
#define USART1_TX_BUFFER_SIZE		32
#define KARADIO_STOP_BYTE			0x0D
#define BAUD1						9600
#define MYUBRR1						F_CPU/16/BAUD1-1
#define META_SIZE					100
#define REFRESH_TIMOUT				1000

/* == global variables ========================================================== */
// Buffer for KaRadio exchange
volatile uint8_t USART1TxIndex = 0;
char RxPacket[USART1_RX_BUFFER_SIZE];
char USART1TxBuffer[USART1_TX_BUFFER_SIZE];
volatile uint8_t 	ButtonCode = 0;
volatile uint8_t 	ButtonEvent = 0;
/****************************************************/

char METAMessage[META_SIZE];
char NameSet[22];
char DateString[9];
char FWVersionStr[22];
volatile uint8_t CurrentListIndex;
volatile uint8_t ListCount;
volatile uint8_t VolumeLevel;
volatile uint16_t SetVolumeCnt = 0u;
volatile uint16_t RefreshCurrentStationCnt = 0;
uint16_t Buffer[AccSizeMacro(ACC_SIZE)];
volatile uint16_t METAScrollCnt = 0;
volatile uint16_t SleepCnt = 0;

// String constant
const char CliListS[] PROGMEM = "#CLI.LIST#";
const char CliListE[] PROGMEM = "##CLI.LIST#";
const char CliListInfo[] PROGMEM = "LISTINFO#";
const char CliListNum[] PROGMEM = "LISTNUM#";
const char AnswerNameSet[] PROGMEM = "NAMESET#: ";
const char AnswerPLAYING[] PROGMEM = "PLAYING#";
const char AnswerMETA[] PROGMEM = "META#: ";
const char AnswerVOL[] PROGMEM = "VOL#: ";
const char Space21[] PROGMEM = "                     ";
const char PowerOffStr[] PROGMEM = "Power OFF            ";
const char SoftResetStr[] PROGMEM = "Soft reset";
const char KaRadioInitStr[] PROGMEM = "KaRadio init";
const char ReleaseStr[] PROGMEM = "Release: ";

void get_but()
{
	ButtonCode = BUT_GetBut();
	if ( ButtonCode )
	{
		ButtonEvent = BUT_GetBut();
	}
}

struct SensorState
{
	uint8_t ButPrev : 1;
	uint8_t ButStop : 1;
	uint8_t ButPlay : 1;
	uint8_t ButNext : 1;
	uint8_t Slider : 1;
} SensorState;

enum DisplayRow
{	
	StationRow,
	TitleRow = 2,
	StatusRow = 7
};

volatile struct Flag  
{
	uint8_t Sleep : 1;
	uint8_t RadioIsStarted : 1;
	uint8_t METAInfo : 1;
	uint8_t ScrollStep : 1;
	uint8_t VolumeIsGetting : 1;
	uint8_t Scroll : 1;
	uint8_t ReadingListComplete : 1;
	uint8_t ReadingNameSet : 1;
	uint8_t RefreshInfo : 1;
	uint8_t USART1RecieveCompleted : 1;
	uint8_t FWReadComplete : 1;
	uint8_t SecondTick : 1;
} Flag;

enum SensorName
{
	ButPrev,
	ButStop,
	ButPlay,
	ButNext,
	Slider
} SensorName;

/*! \brief MCU clock prescaler configuration routine
 *
 */
//static void configure_prescaler(const uint8_t MCU_Clock_Prescaler);

/*! \brief Timer initialization routine
 *
 */
static void timer_init(void);

static void vcc_enable(uint8_t val);

static void ports_init(void);

static void send_to_karadio(char* text);

/* == file-scope (static) functions defines =================================== */

/*! \brief Set the system prescaler to 1 to ensure main clock is running at 8Mhz
 *
 * \param Sys_Clock_Prescaler	MCU clock prescaler value
 * \param MCU_Clock_Prescaler   MCU main clock prescaler.
 *
 * \return void
 */



/*! \brief Configures the timer 2 for Output compare A interrupt at every 10 ms
 *
 * \param void
 * \return void
 */
static void timer_init(void)
{
	TCCR1A = 0x00;
	OCR1A = DEF_TIME_PERIOD_10MSEC;
	TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10);
	TCCR1C = 0x00;
	TCNT1 = 0x0000;
	TIMSK1 = (1 << OCIE1A);

}/*timer_init*/

void USART1_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1) | (1<<TXCIE1);
	/* Set frame format: 8 data, 1 stop bit */
	UCSR1C = (0<<USBS1) | (3<<UCSZ10);
}

void clear_buffer(char* buffer, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		buffer[i] = 0;
	}
}

uint16_t str2int(char* data)
{
	volatile uint16_t val = 0, i = 10000;
	uint8_t c = 0;

	for ( c = 0; c < (5 - strlen(data)); c++ ) i /= 10;

	for ( c = 0; c < strlen(data); c++ )
	{
		val += (data[c] - 0x30) * i;
		i /= 10;
	}
	return val;
}

void utf_to_cp1251(char* dest, char* src)
{
	char* pSrc = src;
	char* pDest = dest;
	int i;
	uint16_t UTFCharCode;
	for (i = 0; i < strlen(src); i++)
	{
		if (*pSrc <= 0x7F)
		{
			*pDest = *pSrc;
			pSrc++;
			pDest++;
		} 
		else
		{
			UTFCharCode = 0xFF00 & (*pSrc << 8); 
			UTFCharCode += *(pSrc + 1);
			if ((UTFCharCode >= 0xD090) && (UTFCharCode <= 0xD0BF))
			{
				*pDest = (uint8_t)(UTFCharCode - 0xCFD0/*CP-1251*/);	// 0xCFE0 - iso8859-5
				pSrc +=2;
				pDest++;
			} 
			else if ((UTFCharCode >= 0xD180) && (UTFCharCode <= 0xD18F))
			{
				*pDest = (uint8_t)(UTFCharCode - 0xD090/*CP-1251*/);			// 0xD0A0 - iso8859-5
				pSrc +=2;
				pDest++;
			}
			else
			{
				if (UTFCharCode == 0xD001)
				{
					*pDest = 0xA8; //  0xA1 - iso8859-5
					pSrc +=2;
					pDest++;
				}
				if (UTFCharCode == 0xD191)
				{
					*pDest = 0xB8; //  0xF1 - iso8859-5
					pSrc +=2;
					pDest++;
				}
			}
		}
	}
}

void vcc_enable(uint8_t val)
{
	if (val == ON)
	{
		SetBit(PORTC, 1);
	} 
	if (val == OFF)
	{
		ClearBit(PORTC, 1);
	}
}

static void ports_init(void)
{
	/* Vcc_CTRL pin init */
	SetBit(DDRC, 1);
	ClearBit(PORTC, 1);
	SetBit(PORTD, 4);
}

uint16_t float_window( uint16_t Value )
{
	static uint16_t Index = 0;
	static uint64_t Summ = 0;

	Summ += Value;

	Summ -= Buffer[Index];

	Buffer[Index++] = Value;

	if ( Index == AccSizeMacro( ACC_SIZE ) )
	{
		Index = 0;
	}

	return ( Summ >> ACC_SIZE );
}

void wait_release_sensor()
{
	uint8_t Exit = 1;
	while(Exit)
	{
		SetVolumeCnt = 0;
		touch_sensors_measure();
		if ((p_selfcap_measure_data->measurement_done_touch == 1u))
		{
			SensorState.ButPrev = GET_SELFCAP_SENSOR_STATE(ButPrev);
			SensorState.ButStop = GET_SELFCAP_SENSOR_STATE(ButStop);
			SensorState.ButPlay = GET_SELFCAP_SENSOR_STATE(ButPlay);
			SensorState.ButNext = GET_SELFCAP_SENSOR_STATE(ButNext);
			SensorState.Slider = GET_SELFCAP_SENSOR_STATE(Slider);

			// GET_SELFCAP_ROTOR_SLIDER_POSITION(0);

			if (!SensorState.ButPrev && 
				!SensorState.ButStop &&
				!SensorState.ButPlay &&
				!SensorState.ButNext &&
				!SensorState.Slider)
			{
				Exit = 0;
			}
		}
	}
}

void show_volume(uint8_t val, uint8_t with_big_num)
{
	char Text[4];
	if (with_big_num)
	{
		sprintf(Text, "%3d", val);
		LCD_Goto(24, 0);
		LCD_Printf(Text, 2, INV_OFF);
	}
	LCD_Volume(val / 2);
}

uint8_t check_val(int val)
{
	if (val < 0) return 0;
	else if (val > 255) return 255;
	else return (uint8_t)val;
}

uint8_t set_volume(uint8_t val)
{
	char Text[16];	
	uint8_t changeInSlider = 0;
	uint8_t slVal = 0;
	int lVal = val;
	LCD_Clear();
	show_volume(0, 1);
	show_volume(val, 1);

	wait_release_sensor();
	
	while(SetVolumeCnt != SET_VOLUME_STOP)
	{
		touch_sensors_measure();
		if ((p_selfcap_measure_data->measurement_done_touch == 1u))
		{
			SensorState.ButPrev = GET_SELFCAP_SENSOR_STATE(ButPrev);
			SensorState.ButNext = GET_SELFCAP_SENSOR_STATE(ButNext);
			SensorState.ButStop = GET_SELFCAP_SENSOR_STATE(ButStop);
			SensorState.ButPlay = GET_SELFCAP_SENSOR_STATE(ButPlay);
			SensorState.Slider = GET_SELFCAP_SENSOR_STATE(Slider);

			if (SensorState.ButPrev)
			{
				wait_release_sensor();
				SetVolumeCnt = 0;
				lVal -= 5;
				val = check_val(lVal);
				changeInSlider = 1;
			}
			if (SensorState.ButNext)
			{
				wait_release_sensor();
				SetVolumeCnt = 0;
				lVal += 5;
				val = check_val(lVal);
				changeInSlider = 1;
			}
		}
	
		if (SensorState.Slider)
		{
			SetVolumeCnt = 0;
			#define DELTA		2
			slVal = float_window(MedianFilter(GET_SELFCAP_ROTOR_SLIDER_POSITION(0)));
			if ((slVal < (lVal - DELTA)) || (slVal > (lVal + DELTA)))
			{
				val = lVal = slVal;
				changeInSlider = 1;
				show_volume(val, 0);
			}										
		}
		else if (changeInSlider == 1)
		{
			show_volume(val, 1);
		}

		if ((changeInSlider == 1) && (USART1TxIndex == 0))
		{
			show_volume(val, 1);
			sprintf(Text, "cli.vol(\"%d\")", val);
			send_to_karadio(Text);
			changeInSlider = 0;
		}
	}
	SetVolumeCnt = 0;
	LCD_Clear();
	return val;
}

void send_to_karadio(char* text)
{
	USART1TxIndex = strlen(text);
	strcpy((char*)USART1TxBuffer, text);
	USART1TxBuffer[strlen(text)] = 0x0d;
	USART1TxIndex++;
	UDR1 = USART1TxBuffer[0];
}

#define SCROLL_TEXT_LEN		22
char TitleString[META_SIZE + SCROLL_TEXT_LEN];
uint16_t ScrollIndex = 0;
uint16_t EndScrollIndex = 0;

void set_title_string(char* titleText)
{
	sprintf(TitleString, "                     %s ", titleText);
	ScrollIndex = 0;
	EndScrollIndex = strlen(TitleString);
}

void scroll_title_string()
{
	char ScrollText[SCROLL_TEXT_LEN];
	for (int i = 0; i < SCROLL_TEXT_LEN; i++) ScrollText[i] = 0;
	strncpy(ScrollText, TitleString + ScrollIndex, SCROLL_TEXT_LEN - 1);
	LCD_Goto(1, TitleRow);
	LCD_Printf(ScrollText, 0, INV_OFF);
	if(++ScrollIndex == EndScrollIndex) ScrollIndex = 0;
}

void draw_line(uint8_t page)
{
	LCD_Goto(0, page);
	for (int i = 0; i < 128; i++)
	{
		LCD_Commmand(DATA, 0x18);
	}
}

void karadio_parser(char* line)
{
	char IntBuff[4];
	char* ici;
	char* s;
	char* e;
	uint8_t d;

	if ((ici = strstr_P(line, ReleaseStr)) != NULL) 
	{
		strlcpy(FWVersionStr, ici + 9, 19);
		Flag.FWReadComplete = 1;
	}

	if ((ici = strstr_P(line, CliListNum)) != NULL)
	{
		ListCount++;
	}

	if (((ici = strstr_P(line, CliListE)) != NULL) && (!Flag.ReadingListComplete))
	{
		Flag.ReadingListComplete = 1;
	}

	if ((ici = strstr_P(line, CliListInfo)) != NULL)
	{
		strlcpy(NameSet, ici + 16, strchr(ici, ',') - (ici + 15));
		return;
	}

	if ((ici=strstr_P(line, AnswerNameSet)) != NULL)
	{
		clear_buffer(IntBuff, 4);
		s = ici + 10;
		e = strchr(s, ' ');
		d = (e - s);
		strlcpy(IntBuff, ici + 10, d + 1);
		CurrentListIndex = str2int(IntBuff);
		strcpy(NameSet, e + 1);
		Flag.ReadingNameSet = 1;
	}
	if ((ici = strstr_P(line, AnswerPLAYING)) != NULL)
	{
		Flag.RadioIsStarted = 1;
	}
	if ((ici = strstr_P(line, AnswerMETA)) != NULL)
	{
		clear_buffer(METAMessage, META_SIZE);
		strcpy(METAMessage, ici + 7);
		Flag.METAInfo = 1;
	}
	if ((ici = strstr_P(line, AnswerVOL)) != NULL)
	{
		if (!Flag.VolumeIsGetting)
		{
			strcpy(IntBuff, ici + 6);
			VolumeLevel = str2int(IntBuff);
			Flag.VolumeIsGetting = 1;
		}
	}
}

void show_string(char* text, uint8_t page, uint8_t inv)
{
	LCD_Goto(1, page);
	LCD_Printf(text, 0, inv);
}

void show_KaRadio_version()
{
	Flag.FWReadComplete = 0;
	send_to_karadio("sys.version");
	while(!Flag.FWReadComplete)
	{
		/* USART1 Rx *****************************************************************************/
		if (Flag.USART1RecieveCompleted == 1)
		{
			karadio_parser(RxPacket);
			clear_buffer(RxPacket, USART1_RX_BUFFER_SIZE);
			Flag.USART1RecieveCompleted = 0;
		}
		/*****************************************************************************************/
	}
	LCD_PageClear(StatusRow);
	show_string(FWVersionStr, StatusRow, OFF);
}

void startup_init()
{

	Flag.Sleep = 0;
	/* Configure the sys_clock prescaler */
	//configure_prescaler(DEF_MCU_CLOCK_PRESCALER);

	/* Configure the Timer 2 */
	timer_init();

	/* Initialize QTouch library and configure touch sensors. */
	touch_sensors_init();

	/* GPIO ports init */
	ports_init();

	/* Main board power ON */
	vcc_enable(ON);

	_delay_ms(100);

	/* Button init */
	BUT_Init();

	/* Configure the CS8406 */
	#if CS8406_USE
	CS8406_Init();
	#endif

	/* OLED display init */
	LCD_init();
	#if IMAGE_INCLUDE
	LCD_DrawImage(0);
	#endif

	/* Enable global interrupts */
	cpu_irq_enable();

	if (BitIsClear(PIND, 4))
	{

		#define UBRR115200				F_CPU/16/115200-1
		while(BitIsClear(PIND, 4));
		show_string("Init", StatusRow, ON);
		USART1_Init(UBRR115200);
		_delay_ms(100);
		send_to_karadio("sys.i2s(\"1\")");
		_delay_ms(100);
		send_to_karadio("sys.vol(\"127\")");
		_delay_ms(100);
		send_to_karadio("sys.uart(\"9600\")");
		_delay_ms(100);
		send_to_karadio("sys.boot");
	}

	_delay_ms(3000);
	LCD_PageClear(StatusRow);
	LCD_Clear();

	show_string("KaRadio is loading...", StatusRow, OFF);

	Flag.RadioIsStarted = 0;

	/* USART1 init */
	USART1_Init(MYUBRR1);

	/* Enable global interrupts */
	// cpu_irq_enable();
		
	while(!Flag.RadioIsStarted)
	{
		/* USART1 Rx *****************************************************************************/
		if (Flag.USART1RecieveCompleted == 1)
		{
			karadio_parser(RxPacket);
			clear_buffer(RxPacket, USART1_RX_BUFFER_SIZE);
			Flag.USART1RecieveCompleted = 0;
		}
		/*****************************************************************************************/
	}

	show_KaRadio_version();
	_delay_ms(2000);
	LCD_PageClear(StatusRow);

	show_string("Reading list count...", StatusRow, OFF);
	ListCount = 0;
	Flag.ReadingListComplete = 0;
	send_to_karadio("cli.list");
	while(!Flag.ReadingListComplete)
	{
		/* USART1 Rx *****************************************************************************/
		if (Flag.USART1RecieveCompleted == 1)
		{
			karadio_parser(RxPacket);
			clear_buffer(RxPacket, USART1_RX_BUFFER_SIZE);
			Flag.USART1RecieveCompleted = 0;
		}
		/*****************************************************************************************/
	}
		
	LCD_Clear();		
	draw_line(1);
	draw_line(3);	
}

int main(void)
{
	int8_t Temp = 0;
	char Text[100];
	uint8_t SleepTime = 0;
	uint16_t SleepVal[6] = {0, 5, 60, 300, 900, 1800};

	startup_init();

    while(1)
    {
		get_but();

		/* USART1 Rx *****************************************************************************/
		if (Flag.USART1RecieveCompleted == 1)
		{
			karadio_parser(RxPacket);
			clear_buffer(RxPacket, USART1_RX_BUFFER_SIZE);
			Flag.USART1RecieveCompleted = 0;		
		}
		/*****************************************************************************************/

		/* Sleep mode */
		if((ButtonCode == BUT_1_ID) && (ButtonEvent == BUT_RELEASED_CODE))
		{
			if (!Flag.Sleep)
			{
				if (++SleepTime == 6) SleepTime = 0;
				if (SleepTime == 0)
				{
					LCD_PageClear(StatusRow);
					SleepCnt = 0;
				}
				else
				{
					SleepCnt = SleepVal[SleepTime];
					sprintf(Text, "Sleep after: %2d.%02d", (SleepCnt / 60), (SleepCnt % 60));
					show_string(Text, StatusRow, ON);
				}				
			} 
			else
			{
				Flag.Sleep = 0;
				SleepTime = 0;
				vcc_enable(ON);
				startup_init();
			}
						
		}

		/* Show version */
		if((ButtonCode == BUT_1_ID) && (ButtonEvent == BUT_DOUBLE_CLICK_CODE))
		{
			show_KaRadio_version();
		}

		/* Update KaRadio FW */
		if((ButtonCode == BUT_1_ID) && (ButtonEvent == BUT_RELEASED_LONG_CODE))
		{
			show_string("  Update KaRadio FW  ", StatusRow, ON);
			send_to_karadio("sys.update");
			Flag.RadioIsStarted = 0;
			startup_init();
		}	
			
		/* Show sleep time */
		if ((SleepCnt > 0) && (Flag.SecondTick))
		{
			sprintf(Text, "Sleep after: %2d.%02d", (SleepCnt / 60), (SleepCnt % 60));
			show_string(Text, StatusRow, OFF);
		}

		if (Flag.RefreshInfo)
		{
			send_to_karadio("cli.info");
			Flag.RefreshInfo = 0;
		}

		if (Flag.Sleep)
		{	
			clear_buffer(METAMessage, META_SIZE);		
			LCD_Clear();
			LCD_DrawImage(0);	
			_delay_ms(100);
			vcc_enable(OFF);		
		}

		if (Flag.ReadingNameSet)
		{
			for(int i = 0; i < 100; i++) Text[i] = 0; 
			LCD_Goto(0, StationRow);
			LCD_Commmand(DATA, 0x00);
			LCD_PageClear(StationRow);
			LCD_Goto(1, StationRow);
			utf_to_cp1251(Text, NameSet);
			LCD_Printf(Text, 0, INV_OFF);
			Flag.ReadingNameSet = 0;
			Temp = CurrentListIndex;
		}

		if (Flag.ReadingListComplete)
		{
			LCD_Goto(0, StationRow);
			LCD_Commmand(DATA, 0xFF);
			LCD_PageClear(StationRow);
			LCD_Goto(1, StationRow);
			LCD_Printf(NameSet, 0, INV_ON);
			Flag.ReadingListComplete = 0;
		}

		/* META info ***********************************************************************/
		if (Flag.METAInfo == 1)
		{
			for(int i = 0; i < 100; i++) Text[i] = 0;
			utf_to_cp1251(Text, METAMessage);
			set_title_string(Text);
			Flag.METAInfo = 0;
		}
		/*********************************************************************** META info */

		/* Scroll **************************************************************************/
		if ((Flag.Scroll) && (!Flag.Sleep))
		{
			scroll_title_string();
			Flag.Scroll = 0;
		}
		/************************************************************************** Scroll */	

        if (!Flag.Sleep) touch_sensors_measure();
		if ((p_selfcap_measure_data->measurement_done_touch == 1u) && (!Flag.Sleep))
		{
			SensorState.ButPrev = GET_SELFCAP_SENSOR_STATE(ButPrev);
			SensorState.ButStop = GET_SELFCAP_SENSOR_STATE(ButStop);
			SensorState.ButPlay = GET_SELFCAP_SENSOR_STATE(ButPlay);
			SensorState.ButNext = GET_SELFCAP_SENSOR_STATE(ButNext);
			SensorState.Slider = GET_SELFCAP_SENSOR_STATE(Slider);

			if (SensorState.Slider)
			{
				VolumeLevel = set_volume(VolumeLevel);
				while(USART1TxIndex);
				send_to_karadio("cli.info");
				draw_line(1);
				draw_line(3);
			}
			if (SensorState.ButPrev)
			{
				wait_release_sensor();
				if (--Temp < 0)
				{
					Temp = ListCount - 1;
				}
				sprintf(Text, "List index: %3d/%d ", Temp + 1, ListCount);
				show_string(Text, StatusRow, INV_OFF);
				Flag.ReadingListComplete = 0;
				_delay_ms(10);
				sprintf(Text, "cli.list(\"%d\")\n", Temp);
				send_to_karadio(Text);
				RefreshCurrentStationCnt = REFRESH_TIMOUT;			
			}
			if (SensorState.ButStop)
			{
				wait_release_sensor();
				set_title_string("");
				send_to_karadio("cli.stop");
			}
			if (SensorState.ButPlay)
			{
				wait_release_sensor();
				set_title_string("");
				sprintf(Text, "cli.play(\"%d\")", Temp);
				send_to_karadio(Text);
				RefreshCurrentStationCnt = 0;
			}
			if (SensorState.ButNext)
			{
				wait_release_sensor();
				if (++Temp == ListCount)
				{
					Temp = 0;
				}				
				sprintf(Text, "List index: %3d/%d ", Temp + 1, ListCount);
				show_string(Text, StatusRow, INV_OFF);			
				Flag.ReadingListComplete = 0;
				_delay_ms(10);
				sprintf(Text, "cli.list(\"%d\")\n", Temp);
				send_to_karadio(Text);
				RefreshCurrentStationCnt = REFRESH_TIMOUT;
			}
		}
    }
}

/* == Interrupt =================================== */
ISR(USART1_RX_vect)
{
	static uint8_t USART1RxIndex = 0;
	uint8_t RxData = UDR1;
	static char USART1RxBuffer[USART1_RX_BUFFER_SIZE];
	if ((RxData != KARADIO_STOP_BYTE) && (Flag.USART1RecieveCompleted == 0))
	{
		USART1RxBuffer[USART1RxIndex] = RxData;
		if (USART1RxIndex < USART1_RX_BUFFER_SIZE) USART1RxIndex++;
	}
	else
	{
		for (int i = 0; i < USART1RxIndex; i++)
		{
			RxPacket[i] = USART1RxBuffer[i];
		}
		clear_buffer(USART1RxBuffer, USART1_RX_BUFFER_SIZE);
		Flag.USART1RecieveCompleted = 1;
		USART1RxIndex = 0;
	}
};

ISR(USART1_TX_vect)
{
	static uint8_t TxByteCnt = 0;

	if (++TxByteCnt == USART1TxIndex) {
		USART1TxIndex = TxByteCnt = 0;
		clear_buffer(USART1TxBuffer, USART1_TX_BUFFER_SIZE);
	}
	else {
		UDR1 = USART1TxBuffer[TxByteCnt];
	}
};

ISR(TIMER1_COMPA_vect)
{
    /* Clear the timer flag */
	TIFR1 |= OCF1A;
	TCNT1 = 0x0000;
	static uint16_t ScrollDevCnt = 0;
	static uint16_t SecondCnt = 0;
	static uint16_t touch_time_counter = 0u;

	BUT_Poll();

	if(SleepCnt > 0)
	{
		if(++SecondCnt == 200)
		{
			SecondCnt = 0;
			Flag.SecondTick = 1;
			if(--SleepCnt == 0) Flag.Sleep = 1;
		}
	}

	if(RefreshCurrentStationCnt > 0)
	{
		if (--RefreshCurrentStationCnt == 0)
		{
			Flag.RefreshInfo = 1;
		}
	}

	if (SetVolumeCnt < SET_VOLUME_STOP)
	{
		SetVolumeCnt++;
	}

	if (++ScrollDevCnt == 30)
	{
		Flag.Scroll = 1;
		ScrollDevCnt = 0;
	}

    /* Do something on RTC overflow here */
    if (touch_time_counter == touch_time.measurement_period_ms)
    {
        touch_time.time_to_measure_touch = 1u;
        touch_time.current_time_ms = touch_time.current_time_ms + touch_time.measurement_period_ms;
        touch_time_counter = 0u;
    }/*if*/
    else
    {
        touch_time_counter++;
    }/*else*/
}/*ISR(TIMER2_COMPA_vect)*/
