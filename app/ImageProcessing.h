/*
 
 ImageProcessing.h : general class holding image processing functions
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

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include "Common.h"

// scale factors for computing optic flow in decimal
#define OF_UP_SCALE            8
#define OF_DOWN_SCALE          1

// default optic flow bins
#define OF_DEFAULT_BINS        4

// min and max settings for optic flow bins
#define OF_MIN_PIX_PER_BIN     4
#define OF_MAX_BINS            16
// maximum optic flow data size
#define OF_MAX_DATA_SIZE       256 

// max/max/default local winner take all window size
#define MAX_LWTA_WINSIZE       13
#define MIN_LWTA_WINSIZE       5

// Image Processing is a general wrapper for multiple image processing functions
class ImageProcessing{
  
public:
  
  // constructor/desctructor
  ImageProcessing();
  ~ImageProcessing();
    
  // full size optic flow arrays
  int OpticFlowX[OF_MAX_DATA_SIZE], OpticFlowY[OF_MAX_DATA_SIZE];
  // byte optic flow array for txmission
  char OpticFlowScale[OF_MAX_DATA_SIZE*2];
  // number of optic flow bins in X and Y dimension
  int NumBins[2];
  // Data array for local winner points
  unsigned char MaxPoints[RAW_IMG_DEF_SIZE / (MIN_LWTA_WINSIZE * MIN_LWTA_WINSIZE)];
  // number of local winners (size of local winner points array to transmit)
  int NumLWTAPoints;
  
  // init optic flow algorithm
  void InitOpticFlow(int ImgRows, int ImgCols);
  // compute optic flow
  void ComputeOpticFlow(unsigned char *Array1, unsigned char *Array2);
  // scale optic flow to byte size from txmission
  void ScaleOpticFlow(void);
  // set raw image resolution record for image processing algorithms
  void SetImageResolution(int ImgRows, int ImgCols); 
  // set number of optic flow bins in X and Y dimension
  void SetBinResolution(int rows, int cols);
  // set smoothing rate for optic flow measurements
  void SetAlpha(unsigned char rate);
  // compute high-passed image
  void HighPass(unsigned char * Buf);
  // intialize high pass algorithm
  void InitHighPass(int ImgRows, int ImgCols);
  // set High Pass level
  void SetHighPass(int cmd);
  // compute local winners in rawImage
  void LocalWinners(unsigned char * buf);
  // set window size for local winner computation
  void SetWinSize(int cmd);
  // set threshold for local winner computation
  void SetLWTAThresh(int cmd);
  
private:
  // RawImage resolution, Size of optic flow array
  int ResRows, ResCols, OpticFlowSize;
  // smoothing rate for optic flow, number of pixels in each optic flow bin
  int alpha, PixPerBin[2];
  // low pass array used in  high pass computation
  int LowPass[RAW_IMG_DEF_SIZE];
  // HP shift level
  int HPShift;
  // local winner thereshold and window size
  int LWTAThresh, LWTAWinSize; 
  
};

#endif