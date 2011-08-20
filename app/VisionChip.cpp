/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    VisionChip.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "VisionChip.h"

/*************************************************************************
 * Function Name: Default Constructor

 *************************************************************************/
VisionChip::VisionChip()
{
  
  FPNAddress = FLASH_ADDR_0;
  DRC_On = false;
  FPN_On = false;
  PixFilled = false;
  
  RawImgBuf = ImgBufA;
  Img1 = ImgBufB;
  Img2 = ImgBufA;
  
}

/*************************************************************************
 * Function Name: Deconstructor

 *************************************************************************/
VisionChip::~VisionChip()
{
  delete ImgBufA;
  delete ImgBufB;
  delete FPNMaskBUILD;
}

/*************************************************************************
 * Function Name: SwapBuffers
 * Parameters: none
 *
 * Return: none
 *
 * Description: switch pointer between two raw image buffers
 *************************************************************************/
void VisionChip::SwapBuffers()
{
  if(RawImgBuf == ImgBufA)
  {
    RawImgBuf = ImgBufB;
    Img1 = ImgBufA;
    Img2 = ImgBufB;
  }
  else
  {
    RawImgBuf = ImgBufA;
    Img1 = ImgBufB;
    Img2 = ImgBufA;
  }
}
/*************************************************************************
 * Function Name: ReadRawImage
 * Parameters: none
 *
 * Return: none
 *
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::ReadRawImage()
{
}
/*************************************************************************
 * Function Name: Init
 * Parameters: none
 *
 * Return: none
 *
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::Init()
{
}
/*************************************************************************
 * Function Name: SetResolution
 * Parameters: none
 *
 * Return: none
 *
 * Description: place holder for inheriting classes
 *************************************************************************/
bool VisionChip::SetResolution(int Rows, int Cols)
{
  return true;
}

/*************************************************************************
 * Function Name: SetResolution
 * Parameters: none
 *
 * Return: none
 *
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::SetNewResolution(int Rows, int Cols)
{
 NewRows = Rows;
 NewCols = Cols;
}

/*************************************************************************
 * Function Name: InitGPIOS
 * Parameters: none
 *
 * Return: none
 *
 * Description: initialize GPIOs to input or output
 *************************************************************************/
void VisionChip::InitGPIOs_Timers(Timers DelayTimer)
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
  
  GPIO_InitStructure.GPIO_Pin =  TEMP_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(TEMP_PORT, &GPIO_InitStructure);
}


/*************************************************************************
 * Function Name: ClearPins
 * Parameters: none
 *
 * Return: none
 *
 * Description: clear command pins for writing
 *
 *************************************************************************/
void VisionChip::ClearPins(void)
{
  CMDIO_PORT->ODR &= ~CMDIO_MASK;                   
}

/*************************************************************************
 * Function Name: send
 * Parameters: Command, Value
 *
 * Return: none
 *
 * Description: send command to Vision Chip
 *
 *************************************************************************/
void VisionChip::send(unsigned int cmd, unsigned int val)
{ 
  // add upper two bits of val to lower bits of cmd
  cmd += (val >> 6) & 0x03;
  val &= 0x3F;
  
  WNR_PORT->ODR |= WNR_MASK;

  ClearPins();
  // send command 1
  cmd += 0x80;
  CMDIO_PORT->ODR |= (CMDIO_MASK & cmd);
           
  
  CS_PORT->ODR |= CS_MASK;
  //DelayResolution1us(1);
  TimE.Delay_us(1);
  //cmd += 10;
  CS_PORT->ODR &= ~CS_MASK;
  
  // send command 2
  ClearPins();
  CMDIO_PORT->ODR |= (CMDIO_MASK & val);
 
  CS_PORT->ODR |= CS_MASK;
  //DelayResolution1us(1);
  TimE.Delay_us(1);
  //cmd -= 10;
  CS_PORT->ODR &= ~CS_MASK;
  
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
void VisionChip::RecordFPNMask(unsigned int FlashAddress)
{
}

void VisionChip::SendCommand(unsigned char cmd, unsigned char val)
{
  send((unsigned int)cmd, (unsigned int)val);
}

void VisionChip::ReadCommand(unsigned char cmd)
{
}

unsigned int VisionChip::GetFullFPNSize()
{
  
  unsigned int DataSize = 0;
  
  for (int r = MinRows; r <= MaxRows; r<<= 1)
  {
   for (int c = MinCols; c <= MaxCols; c<<=1)
    {
      DataSize += r*c;
    }
  }
}

void VisionChip::GetFPNMaskAddr()
{
  unsigned int FlashAddr = FLASH_ADDR_0;
   for (int r = MinRows; r <= MaxRows; r<<= 1)
  {
    for (int c = MinCols; c <= MaxCols; c<<=1)
    {
      if((r == ResRows) && (c == ResCols))
      {
        FPNAddress = FlashAddr;
        return;
      }
      FlashAddr += r*c;
    }
  }
}
void VisionChip::GetDRC()
{
  int RowIdx = 0;
  int MinPix = 255, MaxPix = 0;
  
  for (int r = 1; r < ResRows-1; r++)
  {
    RowIdx = r * ResCols;
    for(int c = 1; c < ResCols-1; c++)
    {
      if(RawImgBuf[RowIdx + c] < MinPix)
        MinPix = RawImgBuf[RowIdx + c];
      if(RawImgBuf[RowIdx + c] > MaxPix)
        MaxPix = RawImgBuf[RowIdx + c];
    }
  }
      
  for(int Idx = 0; Idx  < ResRows*ResCols; Idx++)
  {
    RawImgBuf[Idx]  = ((int)RawImgBuf[Idx] - MinPix) * 255 / (MaxPix - MinPix);
     if(RawImgBuf[PixIdx] == ESC_CHAR)
            RawImgBuf[PixIdx] --;
  }
}
void VisionChip::SetAmpGain(unsigned char gain)
{
}
void VisionChip::SetPendingCommand(int cmd, int val)
{
  PendingCmd = cmd;  PendingVal = val;
}
void VisionChip::SendPendingCommand()
{
  SendCommand(PendingCmd, PendingVal);
}

