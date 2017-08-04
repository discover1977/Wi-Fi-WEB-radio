#include "parser.h"

#define TRUE   1
#define FALSE  0

char buf[SIZE_RECEIVE_BUF];
char *argv[AMOUNT_PAR];
uint8_t argc;

uint8_t i = 0;
uint8_t flag = 0;

void PARS_Init(void)
{
  argc = 0;
  argv[0] = buf;
  flag = FALSE;
  i = 0;
}

void PARS_Parser(char sym)
{
   if (sym != '\r'){
     if (i < SIZE_RECEIVE_BUF - 1){
        if (sym != ' '){
           if (!argc){
              argv[0] = buf;
              argc++;  
           }
          
           if (flag){
              if (argc < AMOUNT_PAR){
                 argv[argc] = &buf[i];   
                 argc++;
              }
              flag = FALSE; 
            }
            
            buf[i] = sym;
            i++;
        }
        else{
           if (!flag){
              buf[i] = 0;
              i++;
              flag = TRUE;
           }
        }
     }
     buf[i] = 0;
     return;
   }
   else{
      buf[i] = 0;

      if (argc){
         PARS_Handler(argc, argv);
      }
      else{
         //сюда можно что-то добавить
      }
      
      argc = 0;
      flag = FALSE;
      i = 0;
   }     
}

#ifdef  __GNUC__

uint8_t PARS_EqualStrFl(char *s1, char const *s2)
{
  uint8_t i = 0;
  
  while(s1[i] == pgm_read_byte(&s2[i]) && s1[i] != '\0' && pgm_read_byte(&s2[i]) != '\0'){
     i++;  
  }
  
  if (s1[i] =='\0' && pgm_read_byte(&s2[i]) == '\0'){
     return TRUE;  
  }
  else{
     return FALSE;  
  }
}

#else

uint8_t PARS_EqualStrFl(char *s1, char __flash *s2)
{
  uint8_t i = 0;
  
  while(s1[i] == s2[i] && s1[i] != '\0' && s2[i] != '\0'){
     i++;  
  }
  
  if (s1[i] =='\0' && s2[i] == '\0'){
     return TRUE;  
  }
  else{
     return FALSE;  
  }
}

#endif


uint8_t PARS_EqualStr(char *s1, char *s2)
{
  uint8_t i = 0;
  
  while(s1[i] == s2[i] && s1[i] != '\0' && s2[i] != '\0'){
     i++;  
  }
  
  if (s1[i] =='\0' && s2[i] == '\0'){
     return TRUE;  
  }
  else{
     return FALSE;  
  }
}

uint8_t PARS_StrToUchar(char *s)
{
   uint8_t value = 0;
  
   while(*s == '0'){
     s++;
   }
   
   while(*s){ 
      value += (*s - 0x30);
      s++;
      if (*s){
         value *= 10;  
      }
   };
  
  return value;
}

uint16_t PARS_StrToUint(char *s)
{
   uint16_t value = 0;
  
   while(*s == '0'){
     s++;
   }
   
   while(*s){ 
      value += (*s - 0x30);
      s++;
      if (*s){
         value *= 10;  
      }
   };
  
  return value;
}