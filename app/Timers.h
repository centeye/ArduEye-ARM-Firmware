/*
 
 Timers.h: FPS timer and delay functions
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

#ifndef TIMERS_H
#define TIMERS_H

// Pin for FPS timer output: used for debugging purposes - uses the same pin
// as RDY2, so both signals cannot be used at the same time
#define TIMER_MASK      GPIO_Pin_4
#define TIMER_PORT      GPIOC

// Sets TimerOutEnable Flag: currently defaults to false 
#define USE_TIME_PIN_OUT 0

#include "stm32f2xx_tim.h"

// timer class manages fps timer and delay functions
class Timers {
 
public:
  // constructor
  Timers();
  
  // FPS dataset (unsigned short)
  unsigned char FPS[2]; 
  
  // initialize timers
  void Init(void);
  // GetFPS called each frame loop to compute frames per second
  void GetFPS(void);
  // Delay microseconds (delay function are imprecise)
  void Delay_us(unsigned int nTime);
  // Delay seconds  (delay functions are imprecise)
  void Delay_s(unsigned int nTime);
  
  //FPS computation variables
  // time record from last frame
  unsigned int LastTime;
  // toggle to change timer pin output (if used)
  bool Toggle;
  // true if timer output pin is used
  bool TimerOutEnable;
   
};

#endif