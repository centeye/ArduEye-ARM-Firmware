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
#include "Stonyman.h"

/*************************************************************************
 Constructor
 *************************************************************************/
Stonyman::Stonyman()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
  MinRows = RAW_ROWS_MIN;
  MinCols = RAW_COLS_MIN;
  
  AmpGain = AMP_GAIN_DEFAULT;
  SettlingTime = SETTLING_TIME_DEFAULT;
}

/*************************************************************************
 * Function Name: ReadRawImage
 * Description: read raw image pixels from vision chip
 *
 *************************************************************************/
void Stonyman::ReadRawImage()
{
  int x, y, inc;
  unsigned char FPN;
  int FPNCount = FPNAddress;
  
  // indices count down from mask so that image is right side up for UI display
  PixIdx = ResRows*ResCols - 1;
  FPNCount += PixIdx;
  
  // clear register pointer
   RESP_PORT->ODR |= RESP_MASK;
   RESP_PORT->ODR &= ~RESP_MASK;
  
  // loop through all pixels (process described in Stonyman data sheet)
  for (y = 0; y < RAW_ROWS_MAX; y+=PixStep[0]) 
  {
      // select row register (increment register by one)
      INCP_PORT->ODR |= INCP_MASK;
      INCP_PORT->ODR &= ~INCP_MASK;
      
      // if y is zero, clear row register value
      if(y == 0)
        { 
          RESV_PORT->ODR |= RESV_MASK;
          RESV_PORT->ODR &= ~RESV_MASK;
      
          // increment to middle of bin (could be anywhere in bin)
          for(inc = 0; inc < PixStep[0]/2; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
            INCV_PORT->ODR &= ~INCV_MASK;
          }
        }
       else{
          // increment column pointer to next bin
          for(inc = 0; inc < PixStep[0]; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
            INCV_PORT->ODR &= ~INCV_MASK;
          }
        }
    
      // select column register (reset to zero)
     RESP_PORT->ODR |= RESP_MASK;
      RESP_PORT->ODR &= ~RESP_MASK;
  
      //Reset column register value to zero
      RESV_PORT->ODR |= RESV_MASK;
      RESV_PORT->ODR &= ~RESV_MASK;
      
      
      // cycle through columns
      for (x = 0; x < RAW_COLS_MAX; x+=PixStep[1]) 
      {       
        // if x is zero, increment to column value to middle of first bin (could be anywhere in bin)
        if(x == 0)
        { 
          for(inc = 0; inc < PixStep[1]/2; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
            INCV_PORT->ODR &= ~INCV_MASK;
          }
        }
       else{
          // increment column pointer to next bin
          for(inc = 0; inc < PixStep[1]; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
            INCV_PORT->ODR &= ~INCV_MASK;
          }
        }
        //Toggle Amplifier if Active
        if(AmpGain > 0)
        {
          INPHI_PORT->ODR |= INPHI_MASK;
          INPHI_PORT->ODR &= ~INPHI_MASK;
        }
          
          // load fpn mask bytes
          FPN = (*(__IO uint8_t*) FPNCount);        

          TimE.Delay_us(SettlingTime);
          // start ADC conversion
          ADC_SoftwareStartConv(ADC1);
          // wait until conversion is complete
          while(!PixFilled);
          PixFilled = false;
          // apply fixed pattern noise mask
          if(FPN_On)
            RawImgBuf[PixIdx] += FPN;
          // update indices
          PixIdx--;        
          FPNCount--;      
      }
  } 
}

/*************************************************************************
 * Function Name: Init
 * Description: intialize chip and local settings
 *
 *************************************************************************/
void Stonyman::Init()
{
  SendCommand(NBIAS, NBIAS_DEFAULT);
  SendCommand(AOBIAS, AOBIAS_DEFAULT);
  SendCommand(VREF, VREF_DEFAULT);
  
  SetResolution(RAW_ROWS_DEFAULT, RAW_COLS_DEFAULT);
  
  //Amplifier OFF
  if(AmpGain == 0)
    SendCommand(CONFIG,16);
  //Amplifier ON
  else
    SendCommand(CONFIG, AmpGain + 8 + 16);
  
}

/******************************************************************************
* SetResolution - set local resolution and send binning commands to vision chip
  Input-    rows- new resolution rows
            cols- new resolution cols
  return-  true if row and column values are valid, else false
*******************************************************************************/
bool Stonyman::SetResolution(int rows, int cols)
{
 bool Invalid = false;

  //Set row dimmensions
  switch (cols)
  {
          case 112:
                  PixStep[0] = 1;
                  SendCommand(HSW, 0);
                  break;
          case 56:
                  PixStep[0] = 2;
                  SendCommand(HSW, 0x55 );
                  break;
          case 28:
                  PixStep[0] = 4;
                  SendCommand(HSW, 0x77 );
                  break;
          case 14:
                  PixStep[0] = 8;
                  SendCommand(HSW, 0x7F );
                  break;
          default:
                  Invalid = true;
                  break;
  }
  // set column dimensions
  switch (rows)
    {
            case 112:
                    PixStep[1] = 1;
                    SendCommand(VSW, 0);
                    break; 
            case 56:
                    PixStep[1] = 2;
                   SendCommand(VSW, 0xAA );
                    break;
            case 28:
                    PixStep[1] = 4;
                    SendCommand(VSW, 0xEE );
                    break;
            case 14:
                    PixStep[1] = 8;
                    SendCommand(VSW, 0xFE );
                    break;
            default:
                    Invalid = true;
                    break;
    }
  
 // if valid row and column values, apply local settings and get fpn mask address
  if(!Invalid)
  {
      ResRows = rows;
      ResCols = cols;
      GetFPNMaskAddr();
  }
  
  // returns true if row and column values are valid
  return Invalid;
}

/*************************************************************************
 * Function Name: RecordFPNMask
 * Description: compute new FPN mask and to flash memory
 * Input-    FlashAddress- Start Address in flash memory to write fixed pattern noise mask
 *************************************************************************/
void Stonyman::RecordFPNMask(unsigned int FlashAddr)
{
  int saveRows = ResRows, saveCols = ResCols;
  
  // find size for all fpn masks (for all resolutions)
  uint32_t FPNSize = GetFullFPNSize();
  // erase all sectors that will store fpn masks
  FlashM.FlashErase(FPNSize, FLASH_ADDR_0); 
  
  FPN_On = false;
  
  // cycle through all possible resolutions and generate fpn mask
    for (int r = RAW_ROWS_MIN; r <= RAW_ROWS_MAX; r<<= 1)
  {
    for (int c = RAW_COLS_MIN; c <= RAW_COLS_MAX; c<<=1)
    {
      SetResolution(r,c);
      FPNIteration(FlashAddr);
      FlashAddr += r*c;
    }
  }
  //reset resolution to calling resolution
  SetResolution(saveRows, saveCols);
  FPN_On = true;
}

/*************************************************************************
 * Function Name: FPNIteration
 * Description: Get FPN mask for one resolution setting
 * Input:   FlashAddress- Starting address to start fpn mask in flash memory
 *************************************************************************/
void Stonyman::FPNIteration(unsigned int FlashAddress)
{
    int MinVal, MaxVal;
    int i, k;
    int RawImgSize = ResRows * ResCols;

    MinVal = 255; MaxVal = 0;
  
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
    {       
       RawImgBuf[i] = (unsigned char)(-(FPNMaskBUILD[i] - MaxVal));
    }

    // write mask to flash memory
    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
}

/*************************************************************************
 * Function Name: SendCommand
 * Description: send commadn to vision chip
 *
 *************************************************************************/
void Stonyman::SendCommand(unsigned char cmd, unsigned char val)
{
  int i;
  
  //Reset Register Pointer
  RESP_PORT->ODR |= RESP_MASK;
  //TimE.Delay_us(CMD_DELAY);
  RESP_PORT->ODR &= ~RESP_MASK;
  
  //Increment Register Pointer
  for (i = 0; i < cmd; i++)
  {
    INCP_PORT->ODR |= INCP_MASK;
    //TimE.Delay_us(CMD_DELAY);
    INCP_PORT->ODR &= ~INCP_MASK;
    //TimE.Delay_us(CMD_DELAY);
  }
  
  //Reset Value Pointer
  RESV_PORT->ODR |= RESV_MASK;
  //TimE.Delay_us(CMD_DELAY);
  RESV_PORT->ODR &= ~RESV_MASK;
  
  //Increment Value Pointer
   for (i = 0; i < val; i++)
  {
    INCV_PORT->ODR |= INCV_MASK;
    //TimE.Delay_us(CMD_DELAY);
    INCV_PORT->ODR &= ~INCV_MASK;
    //TimE.Delay_us(CMD_DELAY);
  }
  
}

/*************************************************************************
 * Function Name: SetAmpGain
 * Description: set amplification gain level
 * Input-     Gain: desired amplification level
 *************************************************************************/
void Stonyman::SetAmpGain(unsigned char gain)
{
  AmpGain = gain;
  if(AmpGain > 0)
    SendCommand(CONFIG, AmpGain + 8 + 16);
  else
    SendCommand(CONFIG, 16);
}

/*************************************************************************
 * Function Name: SendPendingCommand
 * Description: Send command in PendingCmd, PendingVal
 *
 *************************************************************************/
void Stonyman::SendPendingCommand()
{
  SendCommand(PendingCmd, PendingVal);
}

/*************************************************************************
 * Function Name: InitGPIOS_Timers
 * Description: initialize GPIOs to input or output, define timer class reference
   Input: DelayTimer- reference to Timer class.  one timer class initiated for entire application
 *************************************************************************/
void Stonyman::InitGPIOs_Timers(Timers DelayTimer)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  RESP_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(RESP_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  INCP_MASK;
  GPIO_Init(INCP_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  RESV_MASK;
  GPIO_Init(RESV_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  INPHI_MASK;
  GPIO_Init(INPHI_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  INCV_MASK;
  GPIO_Init(INCV_PORT, &GPIO_InitStructure);  
  
  TimE = DelayTimer;
}