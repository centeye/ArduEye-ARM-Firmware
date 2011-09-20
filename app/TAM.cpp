/*
 
 ImageProcessing.cpp : general class holding image processing functions
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

#include "VisionChip.h"
#include "TAM.h"

/*************************************************************************
 Constructor
 *************************************************************************/
TAM::TAM()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
}

/*************************************************************************
 * Function Name: Init
 * Description: intialize chip and local settings
 *
 *************************************************************************/
void TAM::Init()
{
  // clear counter
  CLB_PORT->ODR &= ~CLB_MASK;
  TimE.Delay_us(1);
  CLB_PORT->ODR |= CLB_MASK;
  
  ResRows = RAW_ROWS_DEFAULT;
  ResCols = RAW_COLS_DEFAULT;
}

/*************************************************************************
 * Function Name: ReadRawImage
 * Description: read raw pixels from vision chip
 *
 *************************************************************************/
void TAM::ReadRawImage()
{ 
  int row, col;
  unsigned short FPN;
  unsigned int FPNCount= FPNAddress;
  
  // indices count down from mask so that image is right side up for UI display
  PixIdx = ResRows*ResCols - 1;
  FPNCount += PixIdx;
  
    // clear counter (set to zeroth pixel)
  CLB_PORT->ODR &= ~CLB_MASK;
  TimE.Delay_us(1);
  CLB_PORT->ODR |= CLB_MASK;

  // loop through all pixels
  for(row = 0; row < ResRows; row++)
  {
          for(col = 0; col < ResCols; col++)
          {           
                // load fpn mask byte
                FPN = (*(__IO uint8_t*) FPNCount);  
                FPNCount--;
                
                TimE.Delay_us(1);
                ADC_SoftwareStartConv(ADC1);
                 // wait until conversion is finished
                 while(!PixFilled);
                 PixFilled = false;
                // apply FPN mask
                if(FPN_On)
                  RawImgBuf[PixIdx] -= FPN;
                PixIdx--;

                // increment pixel counter
                CLK_PORT->ODR |= CLK_MASK;
                TimE.Delay_us(1);
                CLK_PORT->ODR &= ~CLK_MASK;
          }
  }
}

/*************************************************************************
 * Function Name: RecordFPNMask
 * Description: compute new FPN mask and to flash memory
 * Input-    FlashAddress- Start Address in flash memory to write fixed pattern noise mask
 *************************************************************************/
void TAM::RecordFPNMask(unsigned int FlashAddress)
{
    unsigned char MinVal, MaxVal;
    int i, k;
    int RawImgSize = ResRows * ResCols;
    
    // erase sector(s) where FPN mask will be written
    FlashM.FlashErase(RawImgSize, FlashAddress); 

    MinVal = 255; MaxVal = 0;
    FPN_On = false;
  
    // clear FPNMask
    for(i = 0; i < RawImgSize; i++)
      FPNMaskBUILD[i] = 0;

    // iterate a number of image reads
    for(i = 0; i < 128; i++)
    {
      ReadRawImage();
      for(k = 0; k < RawImgSize; k++)
         FPNMaskBUILD[k] += (unsigned short)RawImgBuf[k];
    }

    // Get the average pixel (mask) value and min max array values
    for(k = 0; k < RawImgSize; k++)
    {
        FPNMaskBUILD[k] >>= 7;
       
        if(FPNMaskBUILD[k] > MaxVal)
                MaxVal = FPNMaskBUILD[k];
        if((FPNMaskBUILD[k] < MinVal) && (FPNMaskBUILD[k] > 0))
                MinVal = FPNMaskBUILD[k];
    }

    // subtract the min pixel value from the mask
    for (i = 0; i < RawImgSize; i++)     
       RawImgBuf[i] = (unsigned char)FPNMaskBUILD[i] - MinVal;

    // write fpn mask to flash memory
    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
    FPN_On = true;
}

/*************************************************************************
 * Function Name: InitGPIOS_Timers
 * Description: initialize GPIOs to input or output, define timer class reference
   Input: DelayTimer- reference to Timer class.  one timer class initiated for entire application
 *************************************************************************/
void TAM::InitGPIOs_Timers(Timers DelayTimer)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  CLB_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(CLB_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  CLK_MASK;
  GPIO_Init(CLK_PORT, &GPIO_InitStructure);
  
  TimE = DelayTimer;
}
