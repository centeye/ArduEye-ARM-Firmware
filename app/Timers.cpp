/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Timers.cpp      
 *    Centeye,Inc
 *    Alison Leonard
 *    Aug 2, 2011
 **************************************************************************/

#include "Timers.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"

/*************************************************************************
 * Function Name: Constructor
 * Parameters: none
 *
 * Return: none
 *
 * Description: class initialization
 *
 *************************************************************************/
Timers::Timers()
{
  Toggle = false;
}

/*************************************************************************
 * Function Name: Init
 * Parameters: none
 *
 * Return: none
 *
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
  //TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  
  // enable debug output pin
  GPIO_InitStructure.GPIO_Pin =  TIMER_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(TIMER_PORT, &GPIO_InitStructure);
}


/*************************************************************************
 * Function Name: GetFPS
 * Parameters: none
 *
 * Return: none
 *
 * Description: computes time elapsed since last called (max is 6.5s)
 *
 *************************************************************************/
void Timers::GetFPS()
{
  
  unsigned short CurrentTime = TIM2->CNT;
  ElapsedTime = CurrentTime - LastTime;
  unsigned int FPSInt = 10000 / ElapsedTime;
  FPS[0] = FPSInt >> 8;
  FPS[1] = FPSInt & 0xFF;
  LastTime = CurrentTime;
  
 // TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  Toggle = !Toggle;
  if(Toggle)
    TIMER_PORT->ODR |= TIMER_MASK;
  else
    TIMER_PORT->ODR &= ~TIMER_MASK;
  
}

/*************************************************************************
 * Function Name: Delay_us
 * Parameters: none
 *
 * Return: none
 *
 * Description: uses Timer3 to count a delay time
 *
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
 * Parameters: none
 *
 * Return: none
 *
 * Description: uses Timer3 to count a delay time
 *
 *************************************************************************/
void Timers::Delay_s(unsigned int nTime)
{ 
  int TempVal;
  
  
  for( int n = 0; n < nTime; n++)
    for(int i = 0; i < 20000000; i++)
      TempVal = i;
 
}
