/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    ImageProcessing.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    May 29, 2011
 **************************************************************************/

#include "ImageProcessing.h"

ImageProcessing::~ImageProcessing()
{
  if(LowPass)
    delete LowPass;
}
void ImageProcessing::InitOpticFlow(int ImgRows, int ImgCols)
{
  
  NumBins[0] = OF_DEFAULT_BINS;
  NumBins[1] = OF_DEFAULT_BINS;
  
  SetImageResolution(ImgRows, ImgCols);
  
  alpha = OF_ALPHA_DEFAULT;
}

void ImageProcessing::ComputeOpticFlow(unsigned char * Array1, unsigned char * Array2)
{
  int			m, n, RIdx, F3RIdx, F4RIdx;
  long long	        A, BD, C, E, F, Denom;
  int			r, c, cOne, rOne, F1c, F2c;
  int   		F2F1, F4F3, FCF0;
  int			InvScale, Delta, BinNum, BinRows;
  
  Delta = 1;
  
  // Compute 2D Optic Flow for each Bin
  for(m = 0; m < NumBins[0]; m++)
  {
    BinRows = m * NumBins[1];
    rOne = m * PixPerBin[0];
    
    for (n = 0; n < NumBins[1]; n++)
    {
            BinNum = BinRows + n;
            // clear accumulators
            A = BD = C = E = F = 0;

            // Process all Pix in bin
            for (r = Delta; r < PixPerBin[0]-Delta; r++)
            {
                    RIdx = (rOne + r) * ResCols;
                    F3RIdx = (rOne + r + Delta) * ResCols;
                    F4RIdx = (rOne + r - Delta) * ResCols;
    
                    cOne = n*PixPerBin[1];
    
                    for (c = Delta; c < PixPerBin[1] - Delta; c++)
                    {
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
void ImageProcessing::ScaleOpticFlow()
{
  for(int i = 0; i < OpticFlowSize; i++)
  {
   // OpticFlowScaleX[i] = OpticFlowX[i] >> OF_DOWN_SCALE;
   // OpticFlowScaleY[i] = OpticFlowY[i] >> OF_DOWN_SCALE;
    if(OpticFlowX[i] > 1024)
      OpticFlowScaleX[i] = 127;
    else if(OpticFlowX[i] < -1024)
      OpticFlowScaleX[i] = -127;
    else
      OpticFlowScaleX[i] = OpticFlowX[i] >> OF_DOWN_SCALE;
    
     if(OpticFlowY[i] > 1024)
      OpticFlowScaleY[i] = 127;
    else if(OpticFlowY[i] < -1024)
      OpticFlowScaleY[i] = -127;
    else
      OpticFlowScaleY[i] = OpticFlowY[i] >> OF_DOWN_SCALE;
      
  }
}

void ImageProcessing::SetImageResolution(int ImgRows, int ImgCols)
{
  ResRows = ImgRows;
  ResCols = ImgCols;
  
  SetBinResolution(NumBins[0], NumBins[1]);
}
void ImageProcessing::SetBinResolution(int rows, int cols)
{
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
  
  PixPerBin[0] = ResRows / NumBins[0];
  PixPerBin[1] = ResCols / NumBins[1];
  
  // limit numbins to min pix per bin
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
void ImageProcessing::SetAlpha(unsigned char rate)
{
  alpha = (int)rate;
}

void ImageProcessing::HighPass(unsigned char * Buf)
{
  
  int n, Diff[2], InShift;

    if(HPShift > 0){
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
void ImageProcessing::InitHighPass(int ImgRows, int ImgCols, int size)
{
  ResRows = ImgRows;
  ResCols = ImgCols;

  HPShift =  HP_SHIFT_DEFAULT;
}
void ImageProcessing::SetHighPass(int cmd)
{
  HPShift = cmd;
}
