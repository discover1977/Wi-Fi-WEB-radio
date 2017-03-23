//***************************************************************************
//
//  Author(s)...: Павел Бобков  http://ChipEnable.Ru   
//
//  Target(s)...: avr
//
//  Compiler....: IAR
//
//  Description.: драйвер кнопок, концевых датчиков и т.д.
//
//  Data........: 12.12.13
//
//***************************************************************************
#ifndef BUT_H
#define BUT_H

#ifdef  __ICCAVR__ 
   #include <ioavr.h>
   #include <intrinsics.h>
#elif  __GNUC__
   #include <avr/io.h>
   #include <avr/interrupt.h>
#elif __CODEVISIONAVR__
   #include <io.h>
#endif

#include <stdint.h>

/****************** не менять ****************************/

#define BUT_EV_PRESSED       (1<<0)
#define BUT_EV_HELD          (1<<1) 
#define BUT_EV_RELEASED      (1<<2)
#define BUT_EV_RELEASED_LONG (1<<3)
#define BUT_EV_DOUBLE_CLICK  (1<<4)
#define BUT_EV_ALL           (BUT_EV_PRESSED|BUT_EV_HELD|BUT_EV_RELEASED|BUT_EV_RELEASED_LONG|BUT_EV_DOUBLE_CLICK)


/***************** настройки драйвера *********************/

/*количество кнопок*/

#define BUT_AMOUNT           1

/*сколько циклов опроса нужно удерживать 
кнопку, чтобы она считалась нажатой. 
должно быть меньше BUT_COUNT_HELD */

#define BUT_COUNT_THR        5

/*максимальное количество циклов между первым 
и вторым нажатием кнопки для двойного щелчка*/

#define BUT_COUNT_THR_2      50

/*сколько циклов опроса нужно удерживать кнопку,
чтобы она считалась длительно нажатой
должно быть больше BUT_COUNT_THR */

#define BUT_COUNT_HELD       100


/*размер буфера событий.
Его значение должно быть 
кратно степени двойки (2, 4, 8, 16...).*/

#define BUT_SIZE_BUF   2

/*циклический опрос или нет. 
0 - опрос всех кнопок за один вызов BUT_poll()
1 - опрос одной кнопки за один вызов BUT_poll()*/

#define BUT_POLL_ROTATION    0

/* события, которые фиксируются в буфере. 
0 - это событие не фиксируется
1 - событие фиксируется, если разрещено индивидуальной настройкой*/

#define BUT_PRESSED_EN          1
#define BUT_HELD_EN             1
#define BUT_RELEASED_EN         1
#define BUT_RELEASE_LONG_EN     1
#define BUT_DOUBLE_CLICK_EN     1

/* коды событий. могут принимать 
любые значение от 1 до 255 */

#define BUT_PRESSED_CODE        1
#define BUT_HELD_CODE           2 
#define BUT_RELEASED_CODE       3
#define BUT_RELEASED_LONG_CODE  4
#define BUT_DOUBLE_CLICK_CODE   5

/* настройки входов
BUT_1_ID     код кнопки. соответствует ее номеру. задается по порядку (1, 2, 3 .. 32)
BUT_1_DDRX   порт микроконтроллера, задающий направление работы пина
BUT_1_PORTX  порт микроконтроллера, где включается подтягивающий резистор
BUT_1_PINX   порт микроконтроллера, отображающий состояние пина
BUT_1_PIN    номер пина микроконтроллера
BUT_1_LEV    активный уровень пина
BUT_1_PULL   0 - не включать подтягивающий резистор, 1 - включать
BUT_1_EVEN   список событий, которые фиксируется в буфере (BUT_EV_PRESSED|BUT_EV_RELEASED|...)*/

#define BUT_1_ID     1
#define BUT_1_DDRX   DDRD
#define BUT_1_PORTX  PORTD
#define BUT_1_PINX   PIND
#define BUT_1_PIN    2
#define BUT_1_LEV    0
#define BUT_1_PULL   1
#define BUT_1_EVENT  (BUT_EV_RELEASED|BUT_EV_RELEASED_LONG|BUT_EV_DOUBLE_CLICK)


/**************** пользовательские функции *****************/

/*инициализация. 
вызывается в начале программы*/

void BUT_Init(void);

/*опрос кнопок/входов. 
вызывается периодически*/

void BUT_Poll(void);

/*взять событие из буфера.нужно вызывать два раза.
первый раз возвращается ID кнопки. второй раз возвращается код события*/

uint8_t BUT_GetBut(void);

#endif //BUT_H

