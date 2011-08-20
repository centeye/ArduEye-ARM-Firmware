/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    TAM.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "VisionChip.h"
#include "TAM.h"

TAM::TAM()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
}
void TAM::Init()
{
  CLB_PORT->ODR &= ~CLB_MASK;
  //DelayResolution1us(1);
  TimE.Delay_us(1);
  CLB_PORT->ODR |= CLB_MASK;
  
  ResRows = RAW_ROWS_DEFAULT;
  ResCols = RAW_COLS_DEFAULT;
}

void TAM::ReadRawImage()
{ 
  int row, col, PixIdx=0;
  unsigned short FPN;
  unsigned int FPNCount= FPNAddress;

  // loop through all pixels
  for(row = 0; row < ResRows; row++)
  {
          for(col = 0; col < ResCols; col++)
          {
            
                // load fpn mask bytes
                FPN = (*(__IO uint8_t*) FPNCount);  
                FPNCount++;
                
                //DelayResolution1us(1);
                TimE.Delay_us(1);
                ADC_SoftwareStartConv(ADC1);
                while(ADC_GetITStatus(ADC1, ADC_IT_EOC));
               // RawImgBuf[PixIdx] += 50;
                if(FPN_On)
                  RawImgBuf[PixIdx] -= FPN;
                if(RawImgBuf[PixIdx] == ESC_CHAR)
                  RawImgBuf[PixIdx] --;
                PixIdx++;

                CLK_PORT->ODR |= CLK_MASK;
                //DelayResolution1us(1);
                TimE.Delay_us(1);
                CLK_PORT->ODR &= ~CLK_MASK;
          }
  }
  CLB_PORT->ODR &= ~CLB_MASK;
  //DelayResolution1us(1);
  TimE.Delay_us(1);
  CLB_PORT->ODR |= CLB_MASK;
  
}

/*************************************************************************
 * Function Name: RecordFPNMask
 * Parameters: none
 *
 * Return: none
 *
 * Description: write new FPN mask to flash memory
 *
 *************************************************************************/
void TAM::RecordFPNMask(unsigned int FlashAddress)
{
    unsigned char MinVal, MaxVal;
    int i, k;
    int RawImgSize = ResRows * ResCols;
    
    FlashM.FlashErase(RawImgSize, FlashAddress); 

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

    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
      
}
