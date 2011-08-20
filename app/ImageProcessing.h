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

class ImageProcessing{
  
public:
  
  ~ImageProcessing();
    
  int OpticFlowX[OF_MAX_DATA_SIZE], OpticFlowY[OF_MAX_DATA_SIZE];
  char OpticFlowScaleX[OF_MAX_DATA_SIZE], OpticFlowScaleY[OF_MAX_DATA_SIZE];
  int NumBins[2];
  
  void InitOpticFlow(int ImgRows, int ImgCols);
  void ComputeOpticFlow(unsigned char *Array1, unsigned char *Array2);
  void ScaleOpticFlow(void);
  void SetImageResolution(int ImgRows, int ImgCols);  
  void SetBinResolution(int rows, int cols);
  void SetAlpha(unsigned char rate);
  void HighPass(unsigned char * Buf);
  void InitHighPass(int ImgRows, int ImgCols, int size);
  void SetHighPass(int cmd);
  
private:
  int ResRows, ResCols, OpticFlowSize;
  int alpha, PixPerBin[2];
  int LowPass[RAW_IMG_DEF_SIZE];
  int HPShift;
  
  
  
};

#endif