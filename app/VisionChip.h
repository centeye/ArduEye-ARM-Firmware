/*
 
 VisionChip.h : vision chip base classs and specific vision chip inherited classes
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

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_adc.h"
#include "Timers.h"
#include "FlashManager.h"
#include "Common.h"

#ifndef VISONCHIP_H
#define VISONCHIP_H

// Base Vision Chip class
class VisionChip{

public: 
  VisionChip();
  ~VisionChip();
  
  // image resolution
  int ResRows, ResCols;
  // maximum raw image size
  int MaxSize;
  // new resolution - held until update resolution is called
  int NewRows, NewCols;
  // maximum raw Image resolution
  int MaxRows, MaxCols;
  // minimum rawImage resolution
  int MinRows, MinCols;
  // pixel index for read out, flash address for read reading fixed pattern noise value
  int PixIdx, FPNAddress;
  // pointer Current RawImage Buffer (image is double buffered, primarily because two image buffers are needed for optic flow computation)
  unsigned char *RawImgBuf;
  // pointers to keep track of buffer order for optic flow computation
  unsigned char *Img1, *Img2;
  // Instance of FlashManager class to manage memory reads and writes
  FlashManager FlashM;
  // flags for DRC, FPN, stored in Vision Chip class for convenience
  bool DRC_On;
  bool FPN_On;
  // flag that pixel acquisition is complete (set in ADC interrupt, cleared in Vision Chip function)
  bool PixFilled;
  // Amplification gain setting - currently used in Stonyman chip only
  int AmpGain;
  // Command and Value to send when SendPendingCommand is called
  int PendingCmd, PendingVal;
  // time to wait before converting pixel value (between setting pixel index and starting ADC)
  int SettlingTime;
  
  // pixel acquisition 
  void ReadRawImage();
  // initialize visionchip class
  void Init();
  // compute DRC
  void GetDRC();
  // initialize input/output pins and timer class
  void InitGPIOs_Timers(Timers DelayTimer);
  // set raw image resoluion
  bool SetResolution(int rows, int cols);
  // set NewRows, NewCols variables, to be updated after current pixel acquisition is complete
  void SetNewResolution(int rows, int cols);
  // Swap Raw Image Buffers (switch Img1, Img2 pointers)
  void SwapBuffers();
  // record fixed pattern noise mask
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  // send command to vision chip
  void SendCommand(unsigned char cmd, unsigned char val);
  // read command from vision chip -- not currently implemented
  void ReadCommand(unsigned char cmd);
  // find flash memory location of fpn mask for a given array 
  void GetFPNMaskAddr();
  // compute the size for all vision chip fpn masks (relevant for visionchips with multiple resolutions)
  unsigned int GetFullFPNSize();
  // Set amplification gain setting
  void SetAmpGain(unsigned char gain);
  // set a visionchip command to be sent after current pixel acquisition is complete
  void SetPendingCommand(int cmd, int val);
  // send command set by SetPendingCommand
  void SendPendingCommand();
  
protected:  
  // Two Buffers for Raw Image storage - image is double buffered primarily for optic
  // flow computation, as two buffers are needed to compute optic flow
  unsigned char ImgBufA[RAW_IMG_DEF_SIZE], ImgBufB[RAW_IMG_DEF_SIZE];
  // Array for computing FPN mask
  unsigned short FPNMaskBUILD[RAW_IMG_DEF_SIZE];
  
  // instance of timer class - one class instance used for entire application
  Timers TimE;
  
};

// Fifefly Vision Chip implementation
class Firefly : public VisionChip{

public:
  Firefly();
  
  // pixels between each row / colum read, used for reading binned pixels
  int PixStep[2];
  
  // pixel acquisition 
  void ReadRawImage();
  // initialize visionchip class
  void Init();
  // set raw image resoluion
  bool SetResolution(int rows, int cols);
  // record fixed pattern noise mask
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  // send command to vision chip
  void SendCommand(unsigned char cmd, unsigned char val);
  // cycle though every resolution configuation and compute fixed patter noise mask
  void FPNIteration(unsigned int FlashAddress);
  // set a visionchip command to be sent after current pixel acquisition is complete
  void SendPendingCommand();
  // initialize input/output pins and timer class
  void InitGPIOs_Timers(Timers DelayTimer);
  // send command to vision chip
  
//protected
  void send(unsigned int cmd, unsigned int val);
  // clear io pins for writing
  void ClearPins(void);
   
};

class TAM : public VisionChip{
  
public:
  
  TAM();
       
  // pixel acquisition 
  void ReadRawImage();
  // initialize visionchip class
  void Init();
  // record fixed pattern noise mask
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  // initialize input/output pins and timer class
  void InitGPIOs_Timers(Timers DelayTimer);

  
};

class Stonyman : public VisionChip{

public:
  Stonyman();
  
  int PixStep[2];
  
  // pixel acquisition 
  void ReadRawImage();
  // initialize visionchip class
  void Init();
  // set raw image resoluion
  bool SetResolution(int rows, int cols);
  // record fixed pattern noise mask
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  // send command to vision chip
  void SendCommand(unsigned char cmd, unsigned char val);
  // cycle though every resolution configuation and compute fixed patter noise mask
  void FPNIteration(unsigned int FlashAddress);
  // set a visionchip command to be sent after current pixel acquisition is complete
  void SendPendingCommand();
  // initialize input/output pins and timer class
  void InitGPIOs_Timers(Timers DelayTimer);
  // Set Amplification gain level
  void SetAmpGain(unsigned char gain);
   
};


#endif