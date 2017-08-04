//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: mega16
//
//  Compiler....: IAR, GCC, CodeVision 
//
//  Description.: Парсер данных 
//
//  Data........: 14.10.13
//
//***************************************************************************

#ifndef PARSER_H
#define PARSER_H

#include <stdint.h>

#ifdef __GNUC__
#include <avr/pgmspace.h>
#endif

/*вместимость приемного буфера*/
#define SIZE_RECEIVE_BUF  16

/*количество токенов*/
#define AMOUNT_PAR 2

/*функции парсера*/
void PARS_Init(void);
void PARS_Parser(char sym);

/*обработчик полученной строки. он определяется в другом месте*/
extern void PARS_Handler(uint8_t argc, char *argv[]);

/*дополнительные функции для работы со строками*/
uint8_t PARS_EqualStr(char *s1, char *s2);
uint8_t PARS_StrToUchar(char *s);
uint16_t PARS_StrToUint(char *s);

#ifdef __GNUC__
uint8_t PARS_EqualStrFl(char *s1, char const *s2);
#else
uint8_t PARS_EqualStrFl(char *s1, char __flash *s2);
#endif

#endif //PARSER_H