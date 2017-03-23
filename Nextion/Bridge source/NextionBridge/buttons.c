//***************************************************************************
//
//  Author(s)...: Павел Бобков  http://ChipEnable.Ru   
//
//  Target(s)...: avr
//
//  Compiler....: IAR
//
//  Description.: драйвер кнопок
//
//  Data........: 12.12.13
//
//***************************************************************************

#include "buttons.h"

#define FLAG_BUT_PRESSED    (1<<0)
#define FLAG_BUT_HOLD       (1<<1)
#define FLAG_BUT_RELEASED   (1<<2)


/*макрос проверки состояния пина. зависит от 
активного уровня в настройках*/
#define _TestBit1(var, bit)            ((var) & (1<<(bit)))
#define _TestBit0(var, bit)            (!((var) & (1<<(bit))))
#define _TestBit(var, bit, lev)        _TestBit##lev(var, bit)  
#define TestBitLev(var, bit, lev)      _TestBit(var, bit, lev)

/*настройка портов на вход, вкл/выкл подтяжки*/
#define ButtonInit_m(dir, port, pin, pull)   do{dir &= ~(1<<pin);                 \
                                                if (pull) {port |= (1<<pin);}     \
                                                else{port &= ~(1<<pin);}}while(0)

/*сохранение события во временной переменной, если оно разрешено*/
#define SaveEvent_m(settings, mask, curEvent, reg) do{if ((settings) & (mask)){reg = curEvent;}}while(0)

/*макрос для опроса. в одном случае реализует часть switch оператора, в другом просто опрос*/
#if (BUT_POLL_ROTATION > 0) 
#define CheckOneBut_m(id, port, pin, lev, settings, reg)     case ((id) - 1):                   \
                                                                if (TestBitLev(port, pin, lev)){\
                                                                   reg = 1;                     \
                                                                }                               \
                                                                else{                           \
                                                                   reg = 0;                     \
                                                                }                               \
                                                                BUT_Check(reg, id, settings);   \
                                                                break;

   #define Switch_m(x)  switch(x){
   #define End_m()      }
#else

#define CheckOneBut_m(id, port, pin, lev, settings, reg)        if (TestBitLev(port, pin, lev)){\
                                                                   reg = 1;                     \
                                                                }                               \
                                                                else{                           \
                                                                   reg = 0;                     \
                                                                }                               \
                                                                BUT_Check(reg, id, settings);   

   #define Switch_m(x)  
   #define End_m()         
#endif

/*границы для счетчика антидребезга и двойного клика*/
#define BUT_COUNT_MAX        (BUT_COUNT_HELD + 1)
#define BUT_COUNT_THR_2_MAX  (BUT_COUNT_THR_2 + 1)

/*антидребезговый счетчик*/
#if (BUT_COUNT_HELD <= 250)
  static uint8_t countDeb[BUT_AMOUNT];
  static uint8_t countDebTmp;
#else
  static uint16_t countDeb[BUT_AMOUNT];
  static uint16_t countDebTmp;
#endif 

/*счетчики для реализации двойного клика*/
#if (BUT_DOUBLE_CLICK_EN == 1)   
  #if (BUT_COUNT_THR_2 <= 253)
    static uint8_t countHold[BUT_AMOUNT];
    static uint8_t countHoldTmp;
  #else
    static uint16_t countHold[BUT_AMOUNT];
    static uint16_t countHoldTmp;
  #endif
#endif
  
/*буфер, в котором хрянятся флаги кнопок*/
static uint8_t stateBut[BUT_AMOUNT];

/*************** кольцевой буфер ******************/

static uint8_t buf[BUT_SIZE_BUF];
static uint8_t head, tail, count;

static void PutBut(uint8_t but)
{
  if (count < BUT_SIZE_BUF){
     buf[head] = but;
     count++;
     head++;
     head &= (BUT_SIZE_BUF - 1);    
  }
}

uint8_t BUT_GetBut(void)
{
  uint8_t but = 0;
    
  if (count){
     but = buf[tail];
     count--;
     tail++;
     tail &= (BUT_SIZE_BUF - 1);    
  }
  
  return but;
}

/************************************************/

static void BUT_Check(uint8_t state, uint8_t i, uint8_t settings)
{
  uint8_t stateTmp; 
  uint8_t event;
  
  i--;
   
  stateTmp = stateBut[i];
  event = 0;

#if (BUT_DOUBLE_CLICK_EN == 1)  
  countHoldTmp = countHold[i];
#endif

 countDebTmp = countDeb[i];
  
 if (state){
    if (countDebTmp < BUT_COUNT_MAX){
       countDebTmp++;

       if (countDebTmp > BUT_COUNT_THR){
          if (!(stateTmp & FLAG_BUT_PRESSED)){
             stateTmp |= FLAG_BUT_PRESSED;
             
#if (BUT_PRESSED_EN == 1)
             SaveEvent_m(settings, BUT_EV_PRESSED, BUT_PRESSED_CODE, event);  
#endif                   
          }
       }
       
       if (countDebTmp > BUT_COUNT_HELD){
         if (!(stateTmp & FLAG_BUT_HOLD)){
            stateTmp &= ~(FLAG_BUT_RELEASED);
            stateTmp |= FLAG_BUT_HOLD;

#if (BUT_HELD_EN == 1)
            SaveEvent_m(settings, BUT_EV_HELD, BUT_HELD_CODE, event);
#endif       
         }
       }    
    }    
  }
  else{

#if (BUT_DOUBLE_CLICK_EN == 1)     
     if ((stateTmp & FLAG_BUT_PRESSED)&&(!(stateTmp & FLAG_BUT_HOLD))){

       if (stateTmp & FLAG_BUT_RELEASED){
          stateTmp &= ~FLAG_BUT_RELEASED;
          SaveEvent_m(settings, BUT_EV_DOUBLE_CLICK, BUT_DOUBLE_CLICK_CODE, event);
       }
       else{
          countHoldTmp = 0;
          stateTmp |= FLAG_BUT_RELEASED;
       }    
     }       
 
     if (stateTmp & FLAG_BUT_RELEASED){   
        if (countHoldTmp > BUT_COUNT_THR_2){
           countHoldTmp = 0;
           stateTmp &= ~FLAG_BUT_RELEASED;
  #if (BUT_RELEASED_EN == 1)   
           SaveEvent_m(settings, BUT_EV_RELEASED, BUT_RELEASED_CODE, event);
  #endif           
        }
     }
#else
     if ((stateTmp & FLAG_BUT_PRESSED)&&(!(stateTmp & FLAG_BUT_HOLD))){
        SaveEvent_m(settings, BUT_EV_RELEASED, BUT_RELEASED_CODE, event);
     }       
#endif      
     
#if (BUT_RELEASE_LONG_EN == 1)
     if ((stateTmp & FLAG_BUT_PRESSED)&&(stateTmp & FLAG_BUT_HOLD)){
        SaveEvent_m(settings, BUT_EV_RELEASED_LONG, BUT_RELEASED_LONG_CODE, event);
     }
#endif     

     countDebTmp = 0;
     stateTmp &= ~(FLAG_BUT_PRESSED|FLAG_BUT_HOLD);
  }

 

#if (BUT_DOUBLE_CLICK_EN == 1)  
  if (stateTmp & FLAG_BUT_RELEASED){
     if (countHoldTmp < BUT_COUNT_THR_2_MAX){
        countHoldTmp++;
     }
  }
 
  countHold[i] = countHoldTmp;
#endif       
  
  if (event){
     PutBut(i+1);
     PutBut(event);
  }
  
  countDeb[i] = countDebTmp;
  stateBut[i] = stateTmp; 
}

/******************************************************/

void BUT_Init(void)
{
  uint8_t i;
  
  for(i = 0; i < BUT_AMOUNT; i++){
     countDeb[i] = 0;
     stateBut[i] = 0;
     
#if (BUT_DOUBLE_CLICK_EN == 1)      
     countHold[i] = 0;
#endif
     
  }
  
  for(i = 0; i < BUT_SIZE_BUF; i++){
     buf[i] = 0;    
  }
  
  head = 0;
  tail = 0;  
  count = 0;

#ifdef BUT_1_ID  
  ButtonInit_m(BUT_1_DDRX, BUT_1_PORTX, BUT_1_PIN, BUT_1_PULL);
#endif

#ifdef BUT_2_ID  
  ButtonInit_m(BUT_2_DDRX, BUT_2_PORTX, BUT_2_PIN, BUT_2_PULL);
#endif  

#ifdef BUT_3_ID  
  ButtonInit_m(BUT_3_DDRX, BUT_3_PORTX, BUT_3_PIN, BUT_3_PULL);
#endif  

#ifdef BUT_4_ID  
  ButtonInit_m(BUT_4_DDRX, BUT_4_PORTX, BUT_4_PIN, BUT_4_PULL);
#endif  

#ifdef BUT_5_ID  
  ButtonInit_m(BUT_5_DDRX, BUT_5_PORTX, BUT_5_PIN, BUT_5_PULL);
#endif

#ifdef BUT_6_ID  
  ButtonInit_m(BUT_6_DDRX, BUT_6_PORTX, BUT_6_PIN, BUT_6_PULL);
#endif  

#ifdef BUT_7_ID  
  ButtonInit_m(BUT_7_DDRX, BUT_7_PORTX, BUT_7_PIN, BUT_7_PULL);
#endif  

#ifdef BUT_8_ID  
  ButtonInit_m(BUT_8_DDRX, BUT_8_PORTX, BUT_8_PIN, BUT_8_PULL);
#endif  

#ifdef BUT_9_ID  
  ButtonInit_m(BUT_9_DDRX, BUT_9_PORTX, BUT_9_PIN, BUT_9_PULL);
#endif

#ifdef BUT_10_ID  
  ButtonInit_m(BUT_10_DDRX, BUT_10_PORTX, BUT_10_PIN, BUT_10_PULL);
#endif  

#ifdef BUT_11_ID  
  ButtonInit_m(BUT_11_DDRX, BUT_11_PORTX, BUT_11_PIN, BUT_11_PULL);
#endif  

#ifdef BUT_12_ID  
  ButtonInit_m(BUT_12_DDRX, BUT_12_PORTX, BUT_12_PIN, BUT_12_PULL);
#endif  

#ifdef BUT_13_ID  
  ButtonInit_m(BUT_13_DDRX, BUT_13_PORTX, BUT_13_PIN, BUT_13_PULL);
#endif

#ifdef BUT_14_ID  
  ButtonInit_m(BUT_14_DDRX, BUT_14_PORTX, BUT_14_PIN, BUT_14_PULL);
#endif  

#ifdef BUT_15_ID  
  ButtonInit_m(BUT_15_DDRX, BUT_15_PORTX, BUT_15_PIN, BUT_15_PULL);
#endif  

#ifdef BUT_16_ID  
  ButtonInit_m(BUT_16_DDRX, BUT_16_PORTX, BUT_16_PIN, BUT_16_PULL);
#endif  

#ifdef BUT_17_ID  
  ButtonInit_m(BUT_17_DDRX, BUT_17_PORTX, BUT_17_PIN, BUT_17_PULL);
#endif

#ifdef BUT_18_ID  
  ButtonInit_m(BUT_18_DDRX, BUT_18_PORTX, BUT_18_PIN, BUT_18_PULL);
#endif  

#ifdef BUT_19_ID  
  ButtonInit_m(BUT_19_DDRX, BUT_19_PORTX, BUT_19_PIN, BUT_19_PULL);
#endif  

#ifdef BUT_20_ID  
  ButtonInit_m(BUT_20_DDRX, BUT_20_PORTX, BUT_20_PIN, BUT_20_PULL);
#endif  

#ifdef BUT_21_ID  
  ButtonInit_m(BUT_21_DDRX, BUT_21_PORTX, BUT_21_PIN, BUT_21_PULL);
#endif

#ifdef BUT_22_ID  
  ButtonInit_m(BUT_22_DDRX, BUT_22_PORTX, BUT_22_PIN, BUT_22_PULL);
#endif  

#ifdef BUT_23_ID  
  ButtonInit_m(BUT_23_DDRX, BUT_23_PORTX, BUT_23_PIN, BUT_23_PULL);
#endif  

#ifdef BUT_24_ID  
  ButtonInit_m(BUT_24_DDRX, BUT_24_PORTX, BUT_24_PIN, BUT_24_PULL);
#endif  

#ifdef BUT_25_ID  
  ButtonInit_m(BUT_25_DDRX, BUT_25_PORTX, BUT_25_PIN, BUT_25_PULL);
#endif

#ifdef BUT_26_ID  
  ButtonInit_m(BUT_26_DDRX, BUT_26_PORTX, BUT_26_PIN, BUT_26_PULL);
#endif  

#ifdef BUT_27_ID  
  ButtonInit_m(BUT_27_DDRX, BUT_27_PORTX, BUT_27_PIN, BUT_27_PULL);
#endif  

#ifdef BUT_28_ID  
  ButtonInit_m(BUT_28_DDRX, BUT_28_PORTX, BUT_28_PIN, BUT_28_PULL);
#endif  

#ifdef BUT_29_ID  
  ButtonInit_m(BUT_29_DDRX, BUT_29_PORTX, BUT_29_PIN, BUT_29_PULL);
#endif

#ifdef BUT_30_ID  
  ButtonInit_m(BUT_30_DDRX, BUT_30_PORTX, BUT_30_PIN, BUT_30_PULL);
#endif  

#ifdef BUT_31_ID  
  ButtonInit_m(BUT_31_DDRX, BUT_31_PORTX, BUT_31_PIN, BUT_31_PULL);
#endif  

#ifdef BUT_32_ID  
  ButtonInit_m(BUT_32_DDRX, BUT_32_PORTX, BUT_32_PIN, BUT_32_PULL);
#endif  
}

/**********************************************/

void BUT_Poll(void)
{
#if (BUT_POLL_ROTATION > 0)
  static uint8_t i = 0;
#endif

  uint8_t state = 0;

  Switch_m(i);
  
#ifdef BUT_1_ID  
  CheckOneBut_m(BUT_1_ID, BUT_1_PINX, BUT_1_PIN, BUT_1_LEV, BUT_1_EVENT, state);
#endif

#ifdef BUT_2_ID  
  CheckOneBut_m(BUT_2_ID, BUT_2_PINX, BUT_2_PIN, BUT_2_LEV, BUT_2_EVENT, state);
#endif  

#ifdef BUT_3_ID  
  CheckOneBut_m(BUT_3_ID, BUT_3_PINX, BUT_3_PIN, BUT_3_LEV, BUT_3_EVENT, state);
#endif 
  
#ifdef BUT_4_ID  
  CheckOneBut_m(BUT_4_ID, BUT_4_PINX, BUT_4_PIN, BUT_4_LEV, BUT_4_EVENT, state);
#endif  
  
#ifdef BUT_5_ID  
  CheckOneBut_m(BUT_5_ID, BUT_5_PINX, BUT_5_PIN, BUT_5_LEV, BUT_5_EVENT, state);
#endif

#ifdef BUT_6_ID  
  CheckOneBut_m(BUT_6_ID, BUT_6_PINX, BUT_6_PIN, BUT_6_LEV, BUT_6_EVENT, state);
#endif  

#ifdef BUT_7_ID  
  CheckOneBut_m(BUT_7_ID, BUT_7_PINX, BUT_7_PIN, BUT_7_LEV, BUT_7_EVENT, state);
#endif 
  
#ifdef BUT_8_ID  
  CheckOneBut_m(BUT_8_ID, BUT_8_PINX, BUT_8_PIN, BUT_8_LEV, BUT_8_EVENT, state);
#endif  
    
#ifdef BUT_9_ID  
  CheckOneBut_m(BUT_9_ID, BUT_9_PINX, BUT_9_PIN, BUT_9_LEV, BUT_9_EVENT, state);
#endif

#ifdef BUT_10_ID  
  CheckOneBut_m(BUT_10_ID, BUT_10_PINX, BUT_10_PIN, BUT_10_LEV, BUT_10_EVENT, state);
#endif  

#ifdef BUT_11_ID  
  CheckOneBut_m(BUT_11_ID, BUT_11_PINX, BUT_11_PIN, BUT_11_LEV, BUT_11_EVENT, state);
#endif 
  
#ifdef BUT_12_ID  
  CheckOneBut_m(BUT_12_ID, BUT_12_PINX, BUT_12_PIN, BUT_12_LEV, BUT_12_EVENT, state);
#endif  
  
#ifdef BUT_13_ID  
  CheckOneBut_m(BUT_13_ID, BUT_13_PINX, BUT_13_PIN, BUT_13_LEV, BUT_13_EVENT, state);
#endif

#ifdef BUT_14_ID  
  CheckOneBut_m(BUT_14_ID, BUT_14_PINX, BUT_14_PIN, BUT_14_LEV, BUT_14_EVENT, state);
#endif  

#ifdef BUT_15_ID  
  CheckOneBut_m(BUT_15_ID, BUT_15_PINX, BUT_15_PIN, BUT_15_LEV, BUT_15_EVENT, state);
#endif 
  
#ifdef BUT_16_ID  
  CheckOneBut_m(BUT_16_ID, BUT_16_PINX, BUT_16_PIN, BUT_16_LEV, BUT_16_EVENT, state);
#endif   
  
#ifdef BUT_17_ID  
  CheckOneBut_m(BUT_17_ID, BUT_17_PINX, BUT_17_PIN, BUT_17_LEV, BUT_17_EVENT, state);
#endif

#ifdef BUT_18_ID  
  CheckOneBut_m(BUT_18_ID, BUT_18_PINX, BUT_18_PIN, BUT_18_LEV, BUT_18_EVENT, state);
#endif  

#ifdef BUT_19_ID  
  CheckOneBut_m(BUT_19_ID, BUT_19_PINX, BUT_19_PIN, BUT_19_LEV, BUT_19_EVENT, state);
#endif 
  
#ifdef BUT_20_ID  
  CheckOneBut_m(BUT_20_ID, BUT_20_PINX, BUT_20_PIN, BUT_20_LEV, BUT_20_EVENT, state);
#endif  
  
#ifdef BUT_21_ID  
  CheckOneBut_m(BUT_21_ID, BUT_21_PINX, BUT_21_PIN, BUT_21_LEV, BUT_21_EVENT, state);
#endif

#ifdef BUT_22_ID  
  CheckOneBut_m(BUT_22_ID, BUT_22_PINX, BUT_22_PIN, BUT_22_LEV, BUT_22_EVENT, state);
#endif  

#ifdef BUT_23_ID  
  CheckOneBut_m(BUT_23_ID, BUT_23_PINX, BUT_23_PIN, BUT_23_LEV, BUT_23_EVENT, state);
#endif 
  
#ifdef BUT_24_ID  
  CheckOneBut_m(BUT_24_ID, BUT_24_PINX, BUT_24_PIN, BUT_24_LEV, BUT_24_EVENT, state);
#endif  
   
#ifdef BUT_25_ID  
  CheckOneBut_m(BUT_25_ID, BUT_25_PINX, BUT_25_PIN, BUT_25_LEV, BUT_25_EVENT, state);
#endif

#ifdef BUT_26_ID  
  CheckOneBut_m(BUT_26_ID, BUT_26_PINX, BUT_26_PIN, BUT_26_LEV, BUT_26_EVENT, state);
#endif  

#ifdef BUT_27_ID  
  CheckOneBut_m(BUT_27_ID, BUT_27_PINX, BUT_27_PIN, BUT_27_LEV, BUT_27_EVENT, state);
#endif 
  
#ifdef BUT_28_ID  
  CheckOneBut_m(BUT_28_ID, BUT_28_PINX, BUT_28_PIN, BUT_28_LEV, BUT_28_EVENT, state);
#endif  
  
#ifdef BUT_29_ID  
  CheckOneBut_m(BUT_29_ID, BUT_29_PINX, BUT_29_PIN, BUT_29_LEV, BUT_29_EVENT, state);
#endif

#ifdef BUT_30_ID  
  CheckOneBut_m(BUT_30_ID, BUT_30_PINX, BUT_30_PIN, BUT_30_LEV, BUT_30_EVENT, state);
#endif  

#ifdef BUT_31_ID  
  CheckOneBut_m(BUT_31_ID, BUT_31_PINX, BUT_31_PIN, BUT_31_LEV, BUT_31_EVENT, state);
#endif 
  
#ifdef BUT_32_ID  
  CheckOneBut_m(BUT_32_ID, BUT_32_PINX, BUT_32_PIN, BUT_32_LEV, BUT_32_EVENT, state);
#endif    

   End_m();

#if (BUT_POLL_ROTATION > 0)
   i++;
   if (i >= BUT_AMOUNT){
     i = 0;
   }
#endif   
   
}
 
