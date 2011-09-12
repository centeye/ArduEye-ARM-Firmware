/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    FireFly.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "VisionChip.h"
#include "Stonyman.h"

Stonyman::Stonyman()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
  MinRows = RAW_ROWS_MIN;
  MinCols = RAW_COLS_MIN;
  
  AmpGain = AMP_GAIN_DEFAULT;
  SettlingTime = SETTLING_TIME_DEFAULT;
}

void Stonyman::ReadRawImage()
{
  
  int x, y, inc;
  unsigned char FPN;
  int FPNCount = FPNAddress;
  
  PixIdx = ResRows*ResCols - 1;
  FPNCount += PixIdx;
  
  // clear register pointer
   RESP_PORT->ODR |= RESP_MASK;
    //TimE.Delay_us(CMD_DELAY);
   RESP_PORT->ODR &= ~RESP_MASK;
  
  // loop through all pixels (process described in Stonyman data sheet)
  for (y = 0; y < RAW_ROWS_MAX; y+=PixStep[0]) 
  {
      // select row register (increment register by one)
      INCP_PORT->ODR |= INCP_MASK;
        //TimE.Delay_us(CMD_DELAY);
      INCP_PORT->ODR &= ~INCP_MASK;
      
      if(y == 0)
        { 
          RESV_PORT->ODR |= RESV_MASK;
            //TimE.Delay_us(CMD_DELAY);
          RESV_PORT->ODR &= ~RESV_MASK;
      
          for(inc = 0; inc < PixStep[0]/2; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
               //TimE.Delay_us(CMD_DELAY);
            INCV_PORT->ODR &= ~INCV_MASK;
              //TimE.Delay_us(CMD_DELAY);
          }
        }
       else{
          // increment column pointer
          for(inc = 0; inc < PixStep[0]; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
               //TimE.Delay_us(CMD_DELAY);
            INCV_PORT->ODR &= ~INCV_MASK;
              //TimE.Delay_us(CMD_DELAY);
          }
        }
    
      // select column register (reset to zero)
     RESP_PORT->ODR |= RESP_MASK;
        //TimE.Delay_us(CMD_DELAY);
      RESP_PORT->ODR &= ~RESP_MASK;
  
      //Reset column register value to zero
      RESV_PORT->ODR |= RESV_MASK;
        //TimE.Delay_us(CMD_DELAY);
      RESV_PORT->ODR &= ~RESV_MASK;
      // cycle through columns
      for (x = 0; x < RAW_COLS_MAX; x+=PixStep[1]) 
      {       
          
        if(x == 0)
        { 
          for(inc = 0; inc < PixStep[1]/2; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
               //TimE.Delay_us(CMD_DELAY);
            INCV_PORT->ODR &= ~INCV_MASK;
              //TimE.Delay_us(CMD_DELAY);
          }
        }
       else{
          // increment column pointer
          for(inc = 0; inc < PixStep[1]; inc++)
          {
            INCV_PORT->ODR |= INCV_MASK;
               //TimE.Delay_us(CMD_DELAY);
            INCV_PORT->ODR &= ~INCV_MASK;
              //TimE.Delay_us(CMD_DELAY);
          }
        }
        //If Amplifier Active
        // Toggle Amplifier
        if(AmpGain > 0)
        {
          INPHI_PORT->ODR |= INPHI_MASK;
          //TimE.Delay_us(CMD_DELAY);
          INPHI_PORT->ODR &= ~INPHI_MASK;
        }
          
          // load fpn mask bytes
          FPN = (*(__IO uint8_t*) FPNCount);        
          
          
          
          TimE.Delay_us(SettlingTime);
          ADC_SoftwareStartConv(ADC1);
          while(!PixFilled);
          PixFilled = false;
          if(FPN_On)
            RawImgBuf[PixIdx] += FPN;

          if(RawImgBuf[PixIdx] == ESC_CHAR)
            RawImgBuf[PixIdx] --;
          PixIdx--;        
          FPNCount--;
          
      }
  } 

  
}

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
  
  if(!Invalid)
  {
      ResRows = rows;
      ResCols = cols;
      GetFPNMaskAddr();
  }
  
  return Invalid;
}

void Stonyman::RecordFPNMask(unsigned int FlashAddr)
{
  int saveRows = ResRows, saveCols = ResCols;
  
  uint32_t FPNSize = GetFullFPNSize();
  FlashM.FlashErase(FPNSize, FLASH_ADDR_0); 
  
  FPN_On = false;
  
    for (int r = RAW_ROWS_MIN; r <= RAW_ROWS_MAX; r<<= 1)
  {
    for (int c = RAW_COLS_MIN; c <= RAW_COLS_MAX; c<<=1)
    {
      SetResolution(r,c);
      FPNIteration(FlashAddr);
      FlashAddr += r*c;
    }
  }
  SetResolution(saveRows, saveCols);
  FPN_On = true;
}

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

    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
}

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
void Stonyman::SetAmpGain(unsigned char gain)
{
  AmpGain = gain;
  if(AmpGain > 0)
    SendCommand(CONFIG, AmpGain + 8 + 16);
  else
    SendCommand(CONFIG, 16);
}
void Stonyman::SendPendingCommand()
{
  SendCommand(PendingCmd, PendingVal);
}