/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    ImageProcessing.h
 *    Centeye,Inc
 *    Alison Leonard
 *    May 29, 2011
 **************************************************************************/

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include "Common.h"

#define OF_UP_SCALE            8
#define OF_DOWN_SCALE          1

#define OF_ALPHA_DEFAULT       1
#define OF_DEFAULT_BINS        4

#define OF_MIN_PIX_PER_BIN     4
#define OF_MAX_BINS            16
#define OF_MAX_DATA_SIZE       256
#define HP_SHIFT_DEFAULT       5     

#define MAX_LWTA_WINSIZE       13
#define MIN_LWTA_WINSIZE       5
#define LWTA_WINSIZE_DEFAULT   9

#define LWTA_THRESH_DEFAULT    120

class ImageProcessing{
  
public:
  
  ImageProcessing();
  ~ImageProcessing();
    
  int OpticFlowX[OF_MAX_DATA_SIZE], OpticFlowY[OF_MAX_DATA_SIZE];
  char OpticFlowScale[OF_MAX_DATA_SIZE*2];
  int NumBins[2];
  unsigned char MaxPoints[RAW_IMG_DEF_SIZE / (MIN_LWTA_WINSIZE * MIN_LWTA_WINSIZE)];
  int NumLWTAPoints;
  
  void InitOpticFlow(int ImgRows, int ImgCols);
  void ComputeOpticFlow(unsigned char *Array1, unsigned char *Array2);
  void ScaleOpticFlow(void);
  void SetImageResolution(int ImgRows, int ImgCols);  
  void SetBinResolution(int rows, int cols);
  void SetAlpha(unsigned char rate);
  void HighPass(unsigned char * Buf);
  void InitHighPass(int ImgRows, int ImgCols, int size);
  void SetHighPass(int cmd);
  void LocalWinners(unsigned char * buf);
  void SetWinSize(int cmd);
  void SetLWTAThresh(int cmd);
  
private:
  int ResRows, ResCols, OpticFlowSize;
  int alpha, PixPerBin[2];
  int LowPass[RAW_IMG_DEF_SIZE];
  int HPShift;
  int LWTAThresh, LWTAWinSize;
  
  
  
  
};

#endif