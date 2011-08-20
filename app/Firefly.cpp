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
#include "Firefly.h"

Firefly::Firefly()
{
  MaxRows = RAW_ROWS_MAX;
  MaxCols = RAW_COLS_MAX;
  MinRows = RAW_ROWS_MIN;
  MinCols = RAW_COLS_MIN;
}

/*************************************************************************
 * Function Name: Init
 * Parameters: none
 *
 * Return: none
 *
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
        
        SetResolution(RAW_ROWS_DEFAULT, RAW_COLS_DEFAULT);
}

/*************************************************************************
 * Function Name: ReadRawImage
 * Parameters: none
 *
 * Return: none
 *
 * Description: read raw image pixels from vision chip
 *
 *************************************************************************/
void Firefly::ReadRawImage(void)
{
  int x, y;
  unsigned char FPN;
  int FPNCount = FPNAddress;

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
          
          
          WNR_PORT->ODR &= ~WNR_MASK;
          CS_PORT->ODR |= CS_MASK;
                           
          //DelayResolution1us(2);
          TimE.Delay_us(1);
          ADC_SoftwareStartConv(ADC1);
          while(!PixFilled);
          PixFilled = false;
          if(FPN_On)
          {
 //           RawImgBuf[PixIdx] += 50;
            RawImgBuf[PixIdx] -= FPN;
          }
          if(RawImgBuf[PixIdx] == ESC_CHAR)
            RawImgBuf[PixIdx] --;
          PixIdx--;        
          CS_PORT->ODR &= ~CS_MASK;
          FPNCount--;
          
      }
  } 
}

/******************************************************************************
* SetRawSize: handler to update image size and row/column binning
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
  
  if(!Invalid)
  {
      ResRows = rows;
      ResCols = cols;
      GetFPNMaskAddr();
  }
  
  return Invalid;
}

/*************************************************************************
 * Function Name: RecordFPNMask
 * Parameters: none
 *
 * Return: none
 *
 * Description: clear command pins for writing
 *
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

    FlashM.WriteToFlash(RawImgBuf, RawImgSize, FlashAddress);
      
}

void Firefly::SendCommand(unsigned char cmd, unsigned char val)
{ 
  // Open the vault
  send(BIASSWITCH1, 12); // 12 is first key
  send(BIASSWITCH2, 21); // 21 is second key
  send(BIASSWITCH3, 22); // 22 is third key
        
  send((unsigned int)cmd, (unsigned int)val);
  
  // Close the vault
  send(BIASSWITCH1, 0 ); // Any value other than the key will close the vault
  send(BIASSWITCH2, 0 );
  send(BIASSWITCH3, 0 );
}

void Firefly::RecordFPNMask(unsigned int FlashAddr)
{ 
  
  int saveRows = ResRows, saveCols = ResCols;
  
  uint32_t FPNSize = GetFullFPNSize();
  FlashM.FlashErase(FPNSize, FLASH_ADDR_0); 
  
  FPN_On = false;
  
    for (int r = 16; r <= RAW_ROWS_MAX; r<<= 1)
  {
    for (int c = 32; c <= RAW_COLS_TEMP_MAX; c<<=1)
    {
      SetResolution(r,c);
      FPNIteration(FlashAddr);
      FlashAddr += r*c;
    }
  }
  SetResolution(saveRows, saveCols);
  FPN_On = true;
}

void Firefly::SendPendingCommand()
{
  SendCommand(PendingCmd, PendingVal);
}

