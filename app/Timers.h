/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Timers.h      
 *    Centeye,Inc
 *    Alison Leonard
 *    Aug 2, 2011
 **************************************************************************/

#ifndef TIMERS_H
#define TIMERS_H

#define TIMER_MASK      GPIO_Pin_1
#define TIMER_PORT      GPIOB

#include "stm32f2xx_tim.h"

class Timers {
 
public:
  Timers();
  
  unsigned char FPS[2]; 
  
  void Init(void);
  void GetFPS(void);
  void Delay_us(unsigned int nTime);
  
//private:
  unsigned int LastTime;
  unsigned int ElapsedTime;
  bool Toggle;
   
};

#endif