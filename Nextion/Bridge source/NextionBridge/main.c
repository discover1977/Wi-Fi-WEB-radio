/*
 * NextionBridge.c
 *
 * Created: 09.03.2017 10:05:15
 * Author : gavrilov.iv
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>

#include "bits_macros.h"
#include "buttons.h"
#include "i2c-soft.h"
#include "rtc.h"
#include "karadioutils.h"

#define USART0_BUFFER_SIZE		128		// Buffer for Nextion exchange
#define USART1_BUFFER_SIZE		128		// Buffer for KaRadio exchange
#define NEXTION_STOP_BYTE		0x0D
#define KARADIO_STOP_BYTE		0x0D

#define BAUD0		9600
#define MYUBRR0		F_CPU/16/BAUD0-1
#define BAUD1		9600
#define MYUBRR1		F_CPU/16/BAUD1-1

// Nextion parser errors
enum NextionParserStatus
{
	NextionParserError,
	NextionKaRadioCmd,
	NextionGetTime,
	NextionGetDate,
	NextionSetTime,
	NextionSetDate,
	NextionGetAlarm,
	NextionSetAlarm
};	

volatile uint8_t RTCReadData = 0;
volatile char Text[50];

// Buffer for Nextion exchange
volatile uint8_t USART0RecieveCompleted = 0;
volatile uint8_t USART0RxIndex = 0;
volatile uint8_t USART0TxIndex = 0;
volatile uint8_t USART0RxBuffer[USART0_BUFFER_SIZE];		
volatile uint8_t USART0TxBuffer[USART0_BUFFER_SIZE];	

// Buffer for KaRadio exchange
volatile uint8_t USART1RecieveCompleted = 0;
volatile uint8_t USART1RxIndex = 0;
volatile uint8_t USART1TxIndex = 0;
volatile uint8_t USART1RxBuffer[USART1_BUFFER_SIZE];	
volatile uint8_t USART1TxBuffer[USART0_BUFFER_SIZE];	

volatile struct Flag
{
	uint8_t METAInfo : 1;
	uint8_t DirectCmd : 1;
	uint8_t ClearUSART1RxBuffer : 1;
	uint8_t ReadListCount : 1;
	uint8_t ReadListCountComlete : 1;
	uint8_t GetListCount : 1;
	uint8_t GetListCountProc : 1;
	uint8_t ReadCurListStation : 1;
	uint8_t ReadCurListStationComlete : 1;
	uint8_t GetNameSet : 1;
	uint8_t GetVolLev : 1;
} Flag;

volatile uint8_t ListCount;
volatile uint8_t CurrentListIndex;
volatile uint8_t VolumeLevel;
volatile char NameSetList[20];
#define META_SIZE	100
volatile char METAMessage[META_SIZE];


//RTC struct
struct RTC
{
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hour;
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;
} RTC;	

void USART0_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0);
	/* Set frame format: 8 data, 1 stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

void USART1_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<TXCIE1);
	/* Set frame format: 8 data, 1 stop bit */
	UCSR1C = (0<<USBS1)|(3<<UCSZ10);
}

void clear_buffer(uint8_t* buffer, uint8_t size)
{
	// uint8_t* Pointer = buffer;
	for (uint8_t i = 0; i < size; i++)
	{
		buffer[i] = 0;
	}
}

uint16_t str2int(char* data)
{
	volatile uint16_t val = 0, i = 10000;

	for ( uint8_t c = 0; c < (5 - strlen(data)); c++ ) i /= 10;

	for ( uint8_t c = 0; c < strlen(data); c++ )
	{
		val += (data[c] - 0x30) * i;
		i /= 10;
	}
	return val;
}

volatile char* Pointer;

void send_to_nextion(char* text)
{
	clear_buffer(USART0TxBuffer, 128);
	sprintf(USART0TxBuffer, "%s%c%c%c", text, 0xFF, 0xFF, 0xFF);
	USART0TxIndex = strlen(USART0TxBuffer);
	UDR0 = USART0TxBuffer[0];
}

void send_to_karadio(char* text)
{
	clear_buffer(USART1TxBuffer, 128);
	USART1TxIndex = strlen(text);
	strcpy(USART1TxBuffer, text);
	USART1TxBuffer[strlen(text)] = 0x0d;
	USART1TxIndex++;
	UDR1 = USART1TxBuffer[0];
}

void send_time()
{	
	char SendData[32];
	sprintf(SendData, "varHour.val=%02d", RTC.Hour);
	send_to_nextion(SendData);
	while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));

	sprintf(SendData, "varMinute.val=%02d", RTC.Minute);
	send_to_nextion(SendData);
	while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));

	sprintf(SendData, "varSecond.val=%02d", RTC.Second);
	send_to_nextion(SendData);
	while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));
}

void send_date()
{
	char SendData[20];
	sprintf(SendData, "varDay.val=%02d", RTC.Day);
	send_to_nextion(SendData);
	while(USART0TxIndex);

	sprintf(SendData, "varMonth.val=%02d", RTC.Month);
	send_to_nextion(SendData);
	while(USART0TxIndex);

	sprintf(SendData, "varYear.val=%02d", RTC.Year);
	send_to_nextion(SendData);
	while(USART0TxIndex);
}

//volatile uint16_t UTFCharCode;
void utf_to_iso(char* src, char* dest)
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
				*pDest = (uint8_t)(UTFCharCode - 0xCFE0);
				pSrc +=2;
				pDest++;
			} 
			else if ((UTFCharCode >= 0xD180) && (UTFCharCode <= 0xD18F))
			{
				*pDest = (uint8_t)(UTFCharCode - 0xD0A0);
				pSrc +=2;
				pDest++;
			}
		}
	}
}

void nextion_parser(char* data)
{
	volatile char buff[24];
	volatile uint8_t* Index = 0;
	char* Pointer;

	strlcpy(buff, data, strlen(data) + 2);
	if (memcmp("bri", buff, 3) == 0)
	{
		if (memcmp("getVolLev", (buff + 4), 9) == 0) {
			Flag.GetVolLev = 1;
		}

		if (memcmp("getLCount", (buff + 4), 9) == 0) {
			Flag.GetListCount = 1;
		}

		if (memcmp("getCurSta", (buff + 4), 8) == 0) {
			Flag.GetNameSet = 1;
		}

		if (memcmp("getTime", (buff + 4), 7) == 0) {
			//return NextionGetTime;
		}
		if (memcmp("getDate", (buff + 4), 7) == 0) {
			//return NextionGetDate;
		}
		if (memcmp("setTime", (buff + 4), 7) == 0) {

			// Hours extraction
			Index = strchr(data, '"');
			strlcpy(buff, (uint8_t*)(Index + 1), 3);
			RTC.Hour = str2int(buff);

			// Minute extraction
			Index = strchr(data, ':');
			strlcpy(buff, (uint8_t*)(Index + 1), 3);
			RTC.Minute = str2int(buff);

			// Send to RTC
			rtc_set_time(RTC.Hour, RTC.Minute, 0);

			//return NextionSetTime;
		}
		if (memcmp("setDate", (data + 4), 7) == 0) {
			// Date extraction
			Index = strchr(data, '"');
			strlcpy(buff, (uint8_t*)(Index + 1), 3);
			RTC.Day = str2int(buff);
			// Month extraction
			Index += 3;
			strlcpy(buff, (uint8_t*)(Index + 1) , 3);
			RTC.Month = str2int(buff);
			// Year extraction
			Index += 3;
			strlcpy(buff, (uint8_t*)(Index + 1), 3);
			RTC.Year = str2int(buff);
			// Send to RTC
			rtc_set_date(RTC.Day, RTC.Month, RTC.Year);

			//return NextionSetDate;
		}
	}

	removeUtf8((char*)data);
	// KaRadio command parser block 
	if ((Pointer = strstr(data, "cli.")) != NULL) {
		asm("nop");
		if ((Pointer = strstr(data, "_____")) != NULL)
		{
			asm("nop");

		}
		else
		{
			asm("nop");
			Flag.DirectCmd = 1;
		}
	}
}

volatile char* ici;
volatile char* s;
volatile char* e;
volatile uint8_t d;

void karadio_parser(char* line)
{
	char IntBuff[4];
	char Buff[50];	
	//removeUtf8((char*)line);
	if ((ici=strstr(line,"NAMESET#: ")) != NULL)
	{
		clear_buffer(IntBuff, 4);
		s = ici + 10;
		e = strchr(s, ' ');
		d = (e - s);
		strlcpy(IntBuff, ici + 10, d + 1);
		CurrentListIndex = str2int(IntBuff) + 1;
		asm("nop");
	}
	if ((ici = strstr(line, "META#: ")) != NULL)
	{
		clear_buffer(METAMessage, META_SIZE);
		strcpy(METAMessage, ici + 7);
		Flag.METAInfo = 1;
	}
	if ((ici = strstr(line, "VOL ")) != NULL)
	{
		strcpy(IntBuff, ici + 4);
		VolumeLevel = str2int(IntBuff);
		if (VolumeLevel == 0) VolumeLevel = 1;
	}
	if ((ici = strstr(line, "VOL#: ")) != NULL)
	{
		strcpy(IntBuff, ici + 6);
		VolumeLevel = str2int(IntBuff);
		if (VolumeLevel == 0) VolumeLevel = 1;
	}
	if (Flag.GetListCountProc == 1)
	{
		// Get station list count
		if (Flag.ReadListCount == 1)
		{
			ListCount++;
		}
		if ((ici=strstr(line, "LIST#")) != NULL)
		{
			if (Flag.ReadListCount == 0)
			{
				Flag.ReadListCount = 1;
				ListCount = 0;
			}
			else
			{
				Flag.ReadListCount = 0;
				Flag.ReadListCountComlete = 1;
				Flag.GetListCountProc = 0;
				ListCount--;
			}
		}
	}
	else
	{
		if (Flag.ReadCurListStation == 1)
		{
			if ((ici=strstr(line, "  ")) != NULL)
			{
				strlcpy(NameSetList, ici + 5, strchr(ici, ',') - (ici + 4));
				Flag.ReadCurListStationComlete = 1;
			}	
		}
		if ((ici=strstr(line, "LIST#")) != NULL)
		{
			Flag.ReadCurListStation = 1;
		}
		else
		{
			Flag.ReadCurListStation = 0;
		}
	}
}

// Receive from Nextion
ISR(USART0_RX_vect)
{
	uint8_t RxData = UDR0;
	if (RxData != NEXTION_STOP_BYTE)
	{
		USART0RxBuffer[USART0RxIndex] = RxData;
		if (USART0RxIndex < USART0_BUFFER_SIZE) USART0RxIndex++;
	} 
	else
	{
		USART0RecieveCompleted = 1;
		USART0RxIndex = 0;
	}
};

// Transmit to Nextion
ISR(USART0_TX_vect)
{
	static uint8_t TxByteCnt = 0;

	if (++TxByteCnt == USART0TxIndex) {
		USART0TxIndex = TxByteCnt = 0;
	} 
	else {
		UDR0 = USART0TxBuffer[TxByteCnt];
	}
}

// Receive from KaRadio
ISR(USART1_RX_vect)
{
	uint8_t RxData = UDR1;
	if ((RxData != KARADIO_STOP_BYTE) && (USART1RecieveCompleted == 0))
	{
		USART1RxBuffer[USART1RxIndex] = RxData;
		if (USART1RxIndex < USART0_BUFFER_SIZE) USART1RxIndex++;
	}
	else
	{
		USART1RecieveCompleted = 1;
		USART1RxIndex = 0;
	}
};

volatile uint8_t dd;

ISR(USART1_TX_vect)
{
	static uint8_t TxByteCnt = 0;

	if (++TxByteCnt == USART1TxIndex) {
		USART1TxIndex = TxByteCnt = 0;
		Flag.ClearUSART1RxBuffer = 1;
	}
	else {
		dd = UDR1 = USART1TxBuffer[TxByteCnt];
	}
};

/*ISR(INT1_vect)
{
	RTCReadData = 1;
};*/

int main(void)
{
	// INT0 init
	//EIMSK = (1 << INT1);
	//EICRA = (1 << ISC11) | (0 << ISC10);

	// USART init
	USART0_Init(MYUBRR0);
	USART1_Init(MYUBRR1);

	// Soft I2C init
	i2c_Init();

	// RTC init
	/*rtc_init( ( 1 << CTRL_CONV ) | ( 0 << CTRL_EOSC ) | ( 1 << CTRL_BBSQW ) | ( 0 << CTRL_INTCN ) );	// DS323x init data
	rtc_get_time( RTC.Hour, RTC.Minute, RTC.Second);
	rtc_get_date( RTC.Day, RTC.Month, RTC.Year);*/

	// Local variable
	uint8_t NextionParseCode = NextionParserError;
	
	DDRB = (1<<5);
	sei();

	// _delay_ms(30000);

	Flag.GetListCountProc = 1;
	sprintf(Text, "cli.list");
	send_to_karadio(Text);

    while (1) 
    {
		if (RTCReadData == 1) {
			rtc_get_time(&RTC.Hour, &RTC.Minute, &RTC.Second);
			send_time();
			SetBit(PORTB, 5);
			_delay_ms(50);
			ClearBit(PORTB, 5);
			RTCReadData = 0;
		}

		// Check receive packet flag USART0
		if (USART0RecieveCompleted == 1) {
			// Parse Nextion packet
			nextion_parser(USART0RxBuffer);
			USART0RecieveCompleted = 0;
		}
		if (Flag.DirectCmd == 1) {			
			send_to_karadio(USART0RxBuffer);
			Flag.DirectCmd = 0;
		}

		if (Flag.ClearUSART1RxBuffer == 1)
		{
			clear_buffer(USART0RxBuffer, 128);
			Flag.ClearUSART1RxBuffer = 0;
		}

		if (Flag.ReadListCountComlete == 1)
		{
			Flag.ReadListCountComlete = 0;
			SetBit(PORTB, 5);
			_delay_ms(500);
			ClearBit(PORTB, 5);
		}

		if(Flag.GetListCount == 1)
		{
			if (ListCount > 0)
			{
				clear_buffer(Text, 50);
				sprintf(Text, "varListCount.val=%d", ListCount);
				send_to_nextion(Text);	
			}
			else
			{
				Flag.GetListCountProc = 1;
				sprintf(Text, "cli.list");
				send_to_karadio(Text);
			}
			Flag.GetListCount = 0;
		}

		if (Flag.ReadCurListStationComlete == 1)
		{
			clear_buffer(Text, 50);
			sprintf(Text, "t2.txt=\"%s\"", NameSetList);
			while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));
			send_to_nextion(Text);
			Flag.ReadCurListStationComlete = 0;
		}

		if (Flag.GetNameSet == 1)
		{
			clear_buffer(Text, 50);
			if (CurrentListIndex > 0)
			{
				sprintf(Text, "varCLIndex.val=%d", CurrentListIndex);
				while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));
				send_to_nextion(Text);				
			} 
			else
			{
				send_to_karadio("cli.start");
			}
			Flag.GetNameSet = 0;
		}

		if (Flag.GetVolLev == 1)
		{
			clear_buffer(Text, 50);
			if (VolumeLevel > 0)
			{
				sprintf(Text, "varVolumeLevel.val=%d", VolumeLevel);
				while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));				
				send_to_nextion(Text);
			} 
			else
			{
				send_to_karadio("cli.vol");
			}
			Flag.GetVolLev = 0;
		}

		// Check receive packet flag USART1
		if (USART1RecieveCompleted == 1) {
			karadio_parser(USART1RxBuffer);
			clear_buffer(USART1RxBuffer, 128);
			USART1RecieveCompleted = 0;
		}

		
		if (Flag.METAInfo == 1)
		{
			Flag.METAInfo = 0;
			clear_buffer(Text, 50);
			utf_to_iso(METAMessage, Text);
			clear_buffer(METAMessage, META_SIZE);
			sprintf(METAMessage, "qMETA.txt=\"");
			char* p = METAMessage + strlen(METAMessage);
			for (int i = 0; ; i++)
			{
				if (Text[i] == 0)
				{
					break;
				} 
				else
				{
					*p = Text[i];
					p++;
				}
			}
			*p = '"';
			while((USART0TxIndex) || (!( UCSR0A & (1<<UDRE0))));
			send_to_nextion(METAMessage);			
		}
    }
}

