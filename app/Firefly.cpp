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
#include "Firefly.h"

/*************************************************************************
 Constructor
 *************************************************************************/
Firefly::Firefly()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
  MinRows = RAW_ROWS_MIN;
  MinCols = RAW_COLS_MIN;
}

/*************************************************************************
 * Function Name: Init
 * Description: initialize Vision Chip biases
 *
 *************************************************************************/
void Firefly::Init(void)
{
 
  	// Inititialize the linear regulator
	send(LINREG1, 0x24);
	send(LINREG2, 0x04);
	send(LINREG3, 0x05);
	send(LINREG4, 0x10);

	// Set the non-vaulted biases
	send(RESB, 63); // Bottom of ADC range is set to lowest voltage
	send(REST, 0); // Top of ADC range is set to highest voltage
	send(YUKNBIAS, 51); // This bias MUST be set before turning on bias generators

	// Open the vault
	send(BIASSWITCH1, 12); // 12 is first key
	send(BIASSWITCH2, 21); // 21 is second key
	send(BIASSWITCH3, 22); // 22 is third key

	// Set the vaulted biases
	send(NBIAS, 58);
	send(PRSUPPLY, 63 );
	send(ANALOGOUTBIAS, 56);
	send(VREF, 48);
	send(VREF, 48);
        
	// Turn on bias generators using CONNECTVDDA- The vault must be open to do this
	send(CONNECTVDDA, 2 ); // lowest three bits must be 010 to turn on biases

	// Close the vault
	send(BIASSWITCH1, 0 ); // Any value other than the key will close the vault
	send(BIASSWITCH2, 0 );
	send(BIASSWITCH3, 0 );

	// Set row amplifiers to straight output e.g. no amplification
	send(AMPCONF, 2);	// SHORTCAP=0, SELRAW=0, SELAMP=1, SELPBUF=0
	send(ADCOP, 2);
	send(AMPOP, 1 ); // TURN OFF SWITCH CAP AMPLIFIER
	// Configure focal plane
	send(FPCONF, 4 ); // RESETROW=0, READROW=1, ENABLERESET=0, NCONNPRS=0

	// Turn off all binning switches in vertical and horizontal directions
	send(VSW, 0);
	send(HSW, 0);
        
        // set local resolution definitions
        SetResolution(RAW_ROWS_DEFAULT, RAW_COLS_DEFAULT);
}

/*************************************************************************
 * Function Name: ReadRawImage
 * Description: read raw image pixels from vision chip
 *
 *************************************************************************/
void Firefly::ReadRawImage(void)
{
  int x, y;
  unsigned char FPN;
  int FPNCount = FPNAddress;

  // indices count down from mask so that image is right side up for UI display
  PixIdx = ResRows*ResCols - 1;
  FPNCount += PixIdx;
  
  // loop through all pixels (process described in firefly data sheet)
  for (y = 0; y < RAW_ROWS_MAX; y+=PixStep[0]) {
      send(ROWSEL, y); // Select row
      // amplify rows
      send (AMPOP,42); // Connect caps to VREF, set PHI=1 and NPHI=0 to short inverter
      TimE.Delay_us(1);
      send (AMPOP,41); // Set PHI=0 and NPHI=1. This should be performed BEFORE the next step
      TimE.Delay_us(1);
      send (AMPOP, 21); // Connect one or both caps to column output.

      // cycle through columns
      for (x = 0; x < RAW_COLS_MAX; x+=PixStep[1]) {
          // load pixel 1
        
          send(COLSEL, x); // Select column
          
          // load fpn mask bytes
          FPN = (*(__IO uint8_t*) FPNCount);        
          
          // toggle WNR and CS pins
          WNR_PORT->ODR &= ~WNR_MASK;
          CS_PORT->ODR |= CS_MASK;
          
          // delay to allow pixel value to settle
          TimE.Delay_us(1);
          
          // start ADC conversion
          ADC_SoftwareStartConv(ADC1);
          // wait until conversion is finished
          while(!PixFilled);
          PixFilled = false;
          // apply FPN mask
          if(FPN_On)
            RawImgBuf[PixIdx] -= FPN;
          
          //  update indices
          PixIdx--;        
          CS_PORT->ODR &= ~CS_MASK;
          FPNCount--;         
      }
  } 
}

/******************************************************************************
* SetResolution - set local resolution and send binning commands to vision chip
  Input-    rows- new resolution rows
            cols- new resolution cols
  return-  true if row and column values are valid, else false
*******************************************************************************/
bool Firefly::SetResolution(int rows, int cols)
{
  bool Invalid = false;

  //Set row dimmensions
  switch (rows)
  {
          case 128:
                  PixStep[0] = 1;
                  send(VSW, 0);
                  break;
          case 64:
                  PixStep[0] = 2;
                  send(VSW, 0x55 );
                  break;
          case 32:
                  PixStep[0] = 4;
                  send(VSW, 0x77 );
                  break;
          case 16:
                  PixStep[0] = 8;
                  send(VSW, 0x7F );
                  break;
          default:
                  Invalid = true;
                  break;
  }
  // set column dimensions
  switch (cols)
    {
            case 0:
            // full resolution is disable because of memory constraints
            case 256:
           /*         colIdx = 3;
                    PixStep[1] = 1;
                    cols = 256;
                    send(HSW, 0);
                    break; */
            case 128:
                    PixStep[1] = 2;
                    send(HSW, 0xAA );
                    cols = 128;
                    break;
            case 64:
                    PixStep[1] = 4;
                    send(HSW, 0xEE );
                    break;
            case 32:
                    PixStep[1] = 8;
                    send(HSW, 0xFE );
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
 * Function Name: FPNIteration
 * Description: Get FPN mask for one resolution setting
 * Input:   FlashAddress- Starting address to start fpn mask in flash memory
 *************************************************************************/
void Firefly::FPNIteration(unsigned int FlashAddress)
{
    unsigned char MinVal, MaxVal;
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
       RawImgBuf[i] = (unsigned char)FPNMaskBUILD[i] - MinVal;
    }

    // write the mask to memory
    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
      
}

/*************************************************************************
 * Function Name: SendCommand
 * Description: send command to the vision chip, opening and closing the vault
    in case command is a vaulted bias
 *************************************************************************/
void Firefly::SendCommand(unsigned char cmd, unsigned char val)
{ 
  // Open the vault
  send(BIASSWITCH1, 12); // 12 is first key
  send(BIASSWITCH2, 21); // 21 is second key
  send(BIASSWITCH3, 22); // 22 is third key
    
  // send command
  send((unsigned int)cmd, (unsigned int)val);
  
  // Close the vault
  send(BIASSWITCH1, 0 ); // Any value other than the key will close the vault
  send(BIASSWITCH2, 0 );
  send(BIASSWITCH3, 0 );
}

/*************************************************************************
 * Function Name: RecordFPNMask
 * Description: compute new FPN mask and to flash memory
 * Input-    FlashAddress- Start Address in flash memory to write fixed pattern noise mask
 *************************************************************************/
void Firefly::RecordFPNMask(unsigned int FlashAddr)
{ 
  
  int saveRows = ResRows, saveCols = ResCols;
  
  // find size for all fpn masks (for all resolutions)
  uint32_t FPNSize = GetFullFPNSize();
  // erase all sectors that will store fpn masks
  FlashM.FlashErase(FPNSize, FLASH_ADDR_0); 
  
  FPN_On = false;
  
  // cycle through all possible resolutions and generate fpn mask
    for (int r = 16; r <= RAW_ROWS_MAX; r<<= 1)
  {
    for (int c = 32; c <= RAW_COLS_TEMP_MAX; c<<=1)
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
 * Function Name: SendPendingCommand
 * Description: Send command in PendingCmd, PendingVal
 *
 *************************************************************************/
void Firefly::SendPendingCommand()
{
  SendCommand(PendingCmd, PendingVal);
}

/*************************************************************************
 * Function Name: InitGPIOS_Timers
 * Description: initialize GPIOs to input or output, define timer class reference
   Input: DelayTimer- reference to Timer class.  one timer class initiated for entire application
 *************************************************************************/
void Firefly::InitGPIOs_Timers(Timers DelayTimer)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  CMDIO_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(CMDIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  WNR_MASK;
  GPIO_Init(WNR_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  CS_MASK;
  GPIO_Init(CS_PORT, &GPIO_InitStructure);
  
  TimE = DelayTimer;
}

/*************************************************************************
 * Function Name: send
 * Description: send command to Vision Chip
   Input:  cmd- cmd to send
           val- value to be applied to command
 *************************************************************************/
void Firefly::send(unsigned int cmd, unsigned int val)
{ 
  // add upper two bits of val to lower bits of cmd
  cmd += (val >> 6) & 0x03;
  val &= 0x3F;
  
  // set WNR
  WNR_PORT->ODR |= WNR_MASK;

  // clear whatever was previously on pins
  ClearPins();
  // send command 1
  cmd += 0x80;
  CMDIO_PORT->ODR |= (CMDIO_MASK & cmd);
           
  // toggle chip select to set command
  CS_PORT->ODR |= CS_MASK;
  TimE.Delay_us(1);
  CS_PORT->ODR &= ~CS_MASK;
  
  // clear whatever was previously on pins
  ClearPins();
  // send command 2
  CMDIO_PORT->ODR |= (CMDIO_MASK & val);
 
  // toggle chip select to set command
  CS_PORT->ODR |= CS_MASK;
  TimE.Delay_us(1);
  CS_PORT->ODR &= ~CS_MASK;
  
}

/*************************************************************************
 * Function Name: ClearPins
 * Description: clear command pins for writing
 *
 *************************************************************************/
void Firefly::ClearPins(void)
{
  CMDIO_PORT->ODR &= ~CMDIO_MASK;                   
}


