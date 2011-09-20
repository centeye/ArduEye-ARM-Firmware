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

#include "ImageProcessing.h"

/*---------------------------------------------------------------
 ImageProcessing: constructor - initialize variables
 ---------------------------------------------------------------*/
ImageProcessing::ImageProcessing()
{
  LWTAWinSize = LWTA_WINSIZE_DEFAULT;
  LWTAThresh = LWTA_THRESH_DEFAULT;
  
}
/*---------------------------------------------------------------
 ImageProcessing: destructor
 ---------------------------------------------------------------*/
ImageProcessing::~ImageProcessing()
{
  if(LowPass)
    delete LowPass;
}
/*---------------------------------------------------------------
 InitOpticFlow - initialize Optic Flow algorithm
 Input:  ImgRows- rows in raw Image
        ImgCols- cols in raw Image
 ---------------------------------------------------------------*/
void ImageProcessing::InitOpticFlow(int ImgRows, int ImgCols)
{
  // set optic flow bins to default
  NumBins[0] = OF_DEFAULT_BINS;
  NumBins[1] = OF_DEFAULT_BINS;
  
  // set local raw image resolution
  SetImageResolution(ImgRows, ImgCols);
  
  // set optic flow smoothing rate to default
  alpha = OF_ALPHA_DEFAULT;
}

/*---------------------------------------------------------------
 ComputeOpticFlow - compute optic flow using maryam srinivasans image interpolation algorithm
 Input:     Array1-  first raw Image
            Array2- second raw Image
 ---------------------------------------------------------------*/
void ImageProcessing::ComputeOpticFlow(unsigned char * Array1, unsigned char * Array2)
{
  int			m, n, RIdx, F3RIdx, F4RIdx;
  long long	        A, BD, C, E, F, Denom;
  int			r, c, cOne, rOne, F1c, F2c;
  int   		F2F1, F4F3, FCF0;
  int			InvScale, Delta, BinNum, BinRows;
  
  Delta = 1;
  
  // Compute 2D Optic Flow for each Bin
  //row bins
  for(m = 0; m < NumBins[0]; m++)
  {
    BinRows = m * NumBins[1];
    rOne = m * PixPerBin[0];
    
    // cycle through column bins
    for (n = 0; n < NumBins[1]; n++)
    {
            BinNum = BinRows + n;
            // clear accumulators
            A = BD = C = E = F = 0;

            // Process all Pix in bin
            for (r = Delta; r < PixPerBin[0]-Delta; r++)
            {
                    // set row indices
                    RIdx = (rOne + r) * ResCols;
                    F3RIdx = (rOne + r + Delta) * ResCols;
                    F4RIdx = (rOne + r - Delta) * ResCols;
                    
                    // set initial column
                    cOne = n*PixPerBin[1];
    
                    // cycle through columns
                    for (c = Delta; c < PixPerBin[1] - Delta; c++)
                    {
                            // set column indices
                            F1c = cOne + c + Delta;
                            F2c = cOne + c - Delta;
    
                            // compute common elements in calculations (requires 2^15 input range max)
                            F2F1 = (int)(Array1[RIdx + F1c]) - (int)(Array1[RIdx + F2c]);
                            F4F3 = (int)(Array1[F4RIdx + cOne + c]) - (int)(Array1[F3RIdx + cOne + c]);
                            FCF0 = (int)(Array2[RIdx + cOne + c]) - (int)(Array1[RIdx + cOne + c]);
    
                            // update summations
                            A += (F2F1 * F2F1);
                            BD += (F4F3 * F2F1);
                            C += (FCF0 * F2F1);
                            E += (F4F3 * F4F3);
                            F += (FCF0 * F4F3);
                    }
            }
    
            // compute [A,B;D,E]^-1 * [C;F] for X,Y displacement
            Denom = (A * E - BD * BD);
    
            // if denominator is non-zero, compute array inverse to assign optic flow for bin
            if(Denom)
            {
                    InvScale = (2 * Delta) << OF_UP_SCALE;
                    OpticFlowX[BinNum] += (int)((((InvScale * (C * E - F * BD)) / Denom) - OpticFlowX[BinNum]) / alpha);
                    OpticFlowY[BinNum] += (int)((((InvScale * (A * F - C * BD)) / Denom) - OpticFlowY[BinNum]) / alpha);
            }
            else
            {
                    OpticFlowX[BinNum] = 0;
                    OpticFlowY[BinNum] = 0;
            }
       }
  }
}
/*---------------------------------------------------------------
 ScaleOpticFlow - prep optic flow for txmission, convert to single byte array
 ---------------------------------------------------------------*/
void ImageProcessing::ScaleOpticFlow()
{
  int idx = 0;
  // fill optic flow for X direction
  for(int i = 0; i < OpticFlowSize; i++)
  {
    // trunctate measurement if optic flow bigger than one pixel
    if(OpticFlowX[i] > 1024)
      OpticFlowScale[idx] = 127;
    else if(OpticFlowX[idx] < -1024)
      OpticFlowScale[idx] = -127;
    else
      OpticFlowScale[idx] = OpticFlowX[i] >> OF_DOWN_SCALE;
    idx++;
  }
  // fill optic flow for Y direction
  for(int i = 0; i < OpticFlowSize; i++)
  {
    // trunctate measurement if optic flow bigger than one pixel
     if(OpticFlowY[i] > 1024)
      OpticFlowScale[idx] = 127;
    else if(OpticFlowY[i] < -1024)
      OpticFlowScale[idx] = -127;
    else
      OpticFlowScale[idx] = OpticFlowY[i] >> OF_DOWN_SCALE;  
    idx++;
  }
}

/*---------------------------------------------------------------
 SetImageResolution- set raw image resolution used in image processing functions
Input:    ImgRows- raw image rows
          ImgCols- raw image columns
 ---------------------------------------------------------------*/
void ImageProcessing::SetImageResolution(int ImgRows, int ImgCols)
{
  ResRows = ImgRows;
  ResCols = ImgCols;
  
  // adjust optic flow bin settings to match new raw image resolution
  SetBinResolution(NumBins[0], NumBins[1]);
}

/*---------------------------------------------------------------
 SetBinResolution- set optic flow bin resolution
Input:       rows - optic flow row bins
             cols - optic flow column bins
 ---------------------------------------------------------------*/
void ImageProcessing::SetBinResolution(int rows, int cols)
{
  // rows and columns cannot be negative
  if(rows < 0)
    rows = 1;
  if(cols < 0)
    cols = 1;
  
  NumBins[0] = rows;
  NumBins[1] = cols;
  
  // Limit numbins to max bins
  if(NumBins[0] > OF_MAX_BINS)
  {
    NumBins[0] = OF_MAX_BINS;
    PixPerBin[0] = ResRows / OF_MAX_BINS;
  }
   if(NumBins[1] > OF_MAX_BINS)
  {
    NumBins[1] = OF_MAX_BINS;
    PixPerBin[1] = ResCols / OF_MAX_BINS;
  }
  
  // compute pixels per bin
  PixPerBin[0] = ResRows / NumBins[0];
  PixPerBin[1] = ResCols / NumBins[1];
  
  // limit numbins to minimum PixPerBin
  if(PixPerBin[0] < 4)
  {
    NumBins[0] = ResRows / 4;
    PixPerBin[0] = 4;
  }
  if(PixPerBin[1] < 4)
  {
    NumBins[1] = ResCols / 4;
    PixPerBin[1] = 4;
  }
  
  // set 1D optic flow data set size
 OpticFlowSize = NumBins[0] * NumBins[1];
  
}

/*---------------------------------------------------------------
 SetAlpha - set optic flow smoothing rate
Input:    rate- new optic flow rate
 ---------------------------------------------------------------*/
void ImageProcessing::SetAlpha(unsigned char rate)
{
  alpha = (int)rate;
}

/*---------------------------------------------------------------
 HighPass- computed High Passed image
Input:    Buf- input raw image, high passed version returned in the same array
 ---------------------------------------------------------------*/
void ImageProcessing::HighPass(unsigned char * Buf)
{
  
  int n, Diff[2], InShift;

  // compute if highpass level is greater than zero
    if(HPShift > 0){
      // compute for each pixels
        for (n = 0; n < ResRows*ResCols; n++)
        {
              // update lowpass
              InShift = (int)Buf[n] << 16;
              Diff[0] = InShift - LowPass[n];
              Diff[1] = Diff[0] >> HPShift;
              LowPass[n] += Diff[1];

              // highpass
              InShift -= LowPass[n];
              Buf[n] = (unsigned char) ((InShift + 0x800000) >> 16);
        }
    } 
}

/*---------------------------------------------------------------
InitHighPass - intialize high pass algorithm
Input:    ImgRows- raw image rows
          ImgCols- raw image columns
 ---------------------------------------------------------------*/
void ImageProcessing::InitHighPass(int ImgRows, int ImgCols)
{
  ResRows = ImgRows;
  ResCols = ImgCols;

  // set high pass level to default
  HPShift =  HP_SHIFT_DEFAULT;
}

/*---------------------------------------------------------------
SetHighPass:  set high pass level
 ---------------------------------------------------------------*/
void ImageProcessing::SetHighPass(int cmd)
{
  HPShift = cmd;
}

/*---------------------------------------------------------------
SetWinSize: set window size for local winner take all algorithm
Input:  cmd- new window size
 ---------------------------------------------------------------*/
void ImageProcessing::SetWinSize(int cmd)
{
  LWTAWinSize = cmd;
  
    if(LWTAWinSize > MAX_LWTA_WINSIZE)
      LWTAWinSize = MAX_LWTA_WINSIZE;
   
    if(LWTAWinSize < MIN_LWTA_WINSIZE)
      LWTAWinSize = MIN_LWTA_WINSIZE;
  
}

/*---------------------------------------------------------------
SetLWTAThesh:  set threshold for local winner take all algorithm
  All winners must be above the set threshold
Input:  cmd- new threshold
 ---------------------------------------------------------------*/
void ImageProcessing::SetLWTAThresh(int cmd)
{
  LWTAThresh = cmd;
}

/*---------------------------------------------------------------
LocalWinners:  find local winners
Input:   Buf- raw Image to search for local winners
 ---------------------------------------------------------------*/
void ImageProcessing::LocalWinners(unsigned char * Buf)
{
  bool Fail;
  // compute buffer from image edge
  int EdgeBuf = LWTAWinSize/2 + 1;
  int Radius = LWTAWinSize / 2;
  unsigned char Pix;
  int r, c, m, n, RowIdx, rIdx;
  
  // first set of points is the RawImage resolution
  // used to tell potential UI how to plot the point indexes with respect to the
  // raw image
  MaxPoints[0] = ResRows;
  MaxPoints[1] = ResCols;
  NumLWTAPoints = 1;
  
  // cycle through all pix within buffer from edge
  for (r = EdgeBuf; r < ResRows - EdgeBuf; r++)
  {
    rIdx = r * ResRows;
      // cycle through columns within buffer from edge
      for(c = EdgeBuf; c < ResCols - EdgeBuf; c++)
      {
         //set pix to compare      
         Pix = Buf[rIdx + c];
         // set flag to true
	 Fail = true;
	
         // check right, left, up, down first and skip to next pixel if any fail
         if(Pix > LWTAThresh)
           if(Buf[rIdx  + c - 1] < Pix)
              if(Buf[rIdx + c + 1] < Pix)
                  if(Buf[(r - 1) * ResRows + c] < Pix)
                       // cycle through complete window of pixels                     
                       if(Buf[(r + 1) * ResRows + c] < Pix)
                       {     
                          Fail = 0;
                          for (m = r - Radius; m <= r + Radius; m++)
                          {
                                  RowIdx = m * ResRows;
                                  for ( n = c - Radius; n <= c + Radius; n++)
                                  {
                                          // pixels that are equal to the test pixel will succeed (not fail) in current setup
                                          if((Buf[RowIdx + n] > Pix) && !((r==m)&&(n==c)))
                                          {
                                                  Fail = 1;
                                                  break;
                                          }
                                  }
                                  // skip to next pixel if any comparison test fails
                                  if(Fail)
                                          break;
                          }
                       }
         
         // record winning points to MaxPoints array
         if(!Fail)
         {
           MaxPoints[NumLWTAPoints*2] = r;
           MaxPoints[NumLWTAPoints*2 + 1] = c;
           NumLWTAPoints++;          
         }   
      }
  }
}
