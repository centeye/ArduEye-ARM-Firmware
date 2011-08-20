/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    VisionChip.h      
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_adc.h"
//#include "Delays.h"
#include "Timers.h"
#include "FlashManager.h"
#include "Common.h"

#ifndef VISONCHIP_H
#define VISONCHIP_H

#define CMDIO_MASK      ((uint16_t)0x00FF)
#define CMDIO_PORT      GPIOA

#define WNR_MASK        GPIO_Pin_4
#define WNR_PORT        GPIOC

#define CS_MASK         GPIO_Pin_5
#define CS_PORT         GPIOC

#define TEMP_MASK       GPIO_Pin_10
#define TEMP_PORT       GPIOB

#define FLASH_ADDR_0    ((uint32_t)0x08008000)

extern const char ESC_CHAR;

class VisionChip{

public: 
  VisionChip();
  ~VisionChip();
  
  int ResRows, ResCols; 
  int MaxSize;
  int NewRows, NewCols;
  int MaxRows, MaxCols;
  int MinRows, MinCols;
  int PixIdx, FPNAddress;
  unsigned char *RawImgBuf, *Img1, *Img2;
  FlashManager FlashM;
  bool DRC_On;
  bool FPN_On;
  bool PixFilled;
  int AmpGain;
  int PendingCmd, PendingVal;
  int SettlingTime;
  
  void ReadRawImage();
  void Init();
  void GetDRC();
  void InitGPIOs_Timers(Timers DelayTimer);
  bool SetResolution(int rows, int cols);
  void SetNewResolution(int rows, int cols);
  void SwapBuffers();
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  void SendCommand(unsigned char cmd, unsigned char val);
  void ReadCommand(unsigned char cmd);
  void GetFPNMaskAddr();
  unsigned int GetFullFPNSize();
  void SetAmpGain(unsigned char gain);
  void SetPendingCommand(int cmd, int val);
  void SendPendingCommand();
  
protected:   
  void send(unsigned int cmd, unsigned int val);
  void ClearPins(void);
  
  unsigned char ImgBufA[RAW_IMG_DEF_SIZE], ImgBufB[RAW_IMG_DEF_SIZE];
  unsigned short FPNMaskBUILD[RAW_IMG_DEF_SIZE];
  
  Timers TimE;
  
};

class Firefly : public VisionChip{

public:
  Firefly();
  
  int PixStep[2];
  
  void ReadRawImage();
  void Init();
  bool SetResolution(int rows, int cols);
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  void FPNIteration(unsigned int FlashAddress);
  void SendCommand(unsigned char cmd, unsigned char val);
  void SendPendingCommand();
   
};

class TAM : public VisionChip{
  
public:
  
  TAM();
       
  void ReadRawImage();
  void Init();
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);

  
};

class Stonyman : public VisionChip{

public:
  Stonyman();
  
  int PixStep[2];
  
  void ReadRawImage();
  void Init();
  bool SetResolution(int rows, int cols);
  void RecordFPNMask(unsigned int FlashAddr= FLASH_ADDR_0);
  void FPNIteration(unsigned int FlashAddress);
  void SendCommand(unsigned char cmd, unsigned char val);
  void SetAmpGain(unsigned char gain);
  void SendPendingCommand();
   
};


#endif