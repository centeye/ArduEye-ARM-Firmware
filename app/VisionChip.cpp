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
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::ReadRawImage()
{
}
/*************************************************************************
 * Function Name: Init
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::Init()
{
}
/*************************************************************************
 * Function Name: SetResolution
 * Description: place holder for inheriting classes
 *************************************************************************/
bool VisionChip::SetResolution(int Rows, int Cols)
{
  return true;
}

/*************************************************************************
 * Function Name: SetResolution
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::SetNewResolution(int Rows, int Cols)
{
 NewRows = Rows;
 NewCols = Cols;
}

/*************************************************************************
 * Function Name: InitGPIOS_Timers
 * Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::InitGPIOs_Timers(Timers DelayTimer)
{
}

/*************************************************************************
 * Function Name: RecordFPNMask
 *  Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::RecordFPNMask(unsigned int FlashAddress)
{
}

/*************************************************************************
 * Function Name: SendCommand
 *  Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::SendCommand(unsigned char cmd, unsigned char val)
{
}

/*************************************************************************
 * Function Name: ReadCommand
*  Description: read command value - not currently implemented
 *************************************************************************/
void VisionChip::ReadCommand(unsigned char cmd)
{
}

/*************************************************************************
 * Function Name: GetFullFPNSize
*  Description: compute the size for all vision chip fpn masks (relevant for 
  visionchips with multiple resolutions) - use current resolution settings
return: size of all arrays
 *************************************************************************/
unsigned int VisionChip::GetFullFPNSize()
{
  
  unsigned int DataSize = 0;
  
  // cycle through possible resolutions
  for (int r = MinRows; r <= MaxRows; r<<= 1)
  {
   for (int c = MinCols; c <= MaxCols; c<<=1)
    {
      // compute array size and add to datasize variable
      DataSize += r*c;
    }
  }
  // return size of all arrays
  return DataSize;
}

/*************************************************************************
 * Function Name: GetFPNMaskAddr
*  Description: find flash memory location of fpn mask for a given array 
 *************************************************************************/
void VisionChip::GetFPNMaskAddr()
{
  unsigned int FlashAddr = FLASH_ADDR_0;
  // cycle through possible resolutions
   for (int r = MinRows; r <= MaxRows; r<<= 1)
  {
    for (int c = MinCols; c <= MaxCols; c<<=1)
    {
      // increment address until desired resolution is reached
      // (FPN masks are stored in order from smallest to largest)
      if((r == ResRows) && (c == ResCols))
      {
        FPNAddress = FlashAddr;
        return;
      }
      FlashAddr += r*c;
    }
  }
}

/*************************************************************************
 * Function Name: GetDRC
*  Description: compute Dynamic range compensation - expand pixel range to fill full
      0-255 range
 *************************************************************************/
void VisionChip::GetDRC()
{
  int RowIdx = 0;
  int MinPix = 255, MaxPix = 0;
  
  // find minimum and maximum pixels
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
  
  // apply DRC to current image buffer
  for(int Idx = 0; Idx  < ResRows*ResCols; Idx++)
    RawImgBuf[Idx]  = ((int)RawImgBuf[Idx] - MinPix) * 255 / (MaxPix - MinPix);
}

/*************************************************************************
 * Function Name: SetAmpGain
 *  Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::SetAmpGain(unsigned char gain)
{
}

/*************************************************************************
 * Function Name: SetPendingCommand
 *  Description: set cmd and val to send after current pixel acquisition is complete
 *************************************************************************/
void VisionChip::SetPendingCommand(int cmd, int val)
{
  PendingCmd = cmd;  PendingVal = val;
}

/*************************************************************************
 * Function Name: SendPendingCommand
 *  Description: place holder for inheriting classes
 *************************************************************************/
void VisionChip::SendPendingCommand()
{
}

