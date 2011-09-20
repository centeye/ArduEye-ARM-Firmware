/*
 
 Timers.cpp : FPS timer and delay functions
 Centeye, Inc
 Created by Alison Leonard. August, 2011

 ===============================================================================
 Copyright (c) 2011, Centeye, Inc.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Centeye, Inc. nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CENTEYE, INC. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ===============================================================================
*/

#include "Timers.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"

/*************************************************************************
 * Function Name: Constructor
 * Description: class initialization
 *
 *************************************************************************/
Timers::Timers()
{
  Toggle = false;
  TimerOutEnable = USE_TIME_PIN_OUT;
}

/*************************************************************************
 * Function Name: Init
 * Description: initialize FPS timer settings.  This timer has a step size of 100us
 *
 *************************************************************************/
void Timers::Init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
   /* Compute the prescaler value */
  uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000);
  
  //Timer Configuration
  TIM_TimeBaseStructure.TIM_Period = 65536;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  TIM_ARRPreloadConfig(TIM2, DISABLE);
 
  /* TIM Interrupts enable */
  TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

  /* TIM3 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  
  // enable debug output pin if active
  if(TimerOutEnable)
  {
    GPIO_InitStructure.GPIO_Pin =  TIMER_MASK;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(TIMER_PORT, &GPIO_InitStructure);
  }
}


/*************************************************************************
 * Function Name: GetFPS
 * Description: computes time elapsed since last called (max is 6.5s)
 *
 *************************************************************************/
void Timers::GetFPS()
{
  // get current time
  unsigned short CurrentTime = TIM2->CNT;
  // get elapsed time
  unsigned int ElapsedTime = CurrentTime - LastTime;
  // timer is in 100us steps
  unsigned int FPSInt = 10000 / ElapsedTime;
  // Store in FPS array
  FPS[0] = FPSInt >> 8;
  FPS[1] = FPSInt & 0xFF;
  // update LastTime
  LastTime = CurrentTime;
  
  // toggle output on timer pin if active
  if(TimerOutEnable)
  {
    Toggle = !Toggle;
    if(Toggle)
      TIMER_PORT->ODR |= TIMER_MASK;
    else
      TIMER_PORT->ODR &= ~TIMER_MASK;
  }
}

/*************************************************************************
 * Function Name: Delay_us
 * Description: uses instruction cycle burning to create delay - delay function is 
 * imprecise, but relatively close (checked on oscilloscope)
 *************************************************************************/
void Timers::Delay_us(unsigned int nTime)
{ 
  int TempVal;
  
  for( int n = 0; n < nTime; n++)
    for(int i = 0; i < 17; i++)
      TempVal = i;
 
}

/*************************************************************************
 * Function Name: Delay_s
 * Description: uses instruction cycle burning to create delay - delay function is 
 * imprecise, but relatively close (checked on oscilloscope)  
 *************************************************************************/
void Timers::Delay_s(unsigned int nTime)
{ 
  int TempVal;
   
  for( int n = 0; n < nTime; n++)
    for(int i = 0; i < 20000000; i++)
      TempVal = i;
 
}
