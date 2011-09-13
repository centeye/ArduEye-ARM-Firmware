/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    VCInterface
 *    Centeye,Inc
 *    Alison Leonard
 *    May 29, 2011
 **************************************************************************/

#include "includes.h"


#define SPIS_MASK       ((uint16_t)0xB000)
#define SPIS_PORT       GPIOB

#define SPI_MOSI_SOURCE GPIO_PinSource15
#define SPI_MISO_SOURCE GPIO_PinSource14
#define SPI_SCK_SOURCE  GPIO_PinSource13

#define SPI_MISO_MASK   GPIO_Pin_14
#define SPI_PORT        GPIOB

#define AN_MASK         GPIO_Pin_2
#define AN_CH           ADC_Channel_12
#define AN_PORT         GPIOC

#define RDY_MASK        GPIO_Pin_6
#define RDY_PORT        GPIOC

#define CMD_MASK      GPIO_Pin_10
#define CMD_PORT      GPIOB

#define LED_MASK      GPIO_Pin_2
#define LED_PORT      GPIOB

#define RDY2_MASK     GPIO_Pin_4
#define RDY2_PORT     GPIOC

#define SDA_MASK      GPIO_Pin_11
#define SDA_PORT      GPIOB

#define SCL_MASK      GPIO_Pin_10
#define SCL_PORT      GPIOB

#define SPI_TIME_OUT    20000


DataManager Dat;
ImageProcessing ImgP;
Timers TimE;
unsigned int  EIdx = 0, CIdx = 0, DIdx = 0;
unsigned short  MinPix = 0xFFFF;
unsigned short  MaxPix  = 0;
bool SPI_Done = FALSE;
char Process = NULL_PROCESS;
bool RecordFPN = false;
bool Toggle = false;
bool ResolutionUpdateRequest = false, CMDUpdateRequest = false;
uint16_t CCR3_Val = 100;

Stonyman VC;

Int32U CriticalSecCntr;

/*************************************************************************
 * Function Name: ADCHandler
 * Parameters: void
 * Return: void
 *
 * Description:
 *		
 *************************************************************************/
void TimerHandler(void)
{
 //uint16_t capture = 0;
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
 /* Toggle = !Toggle;
  if(Toggle)
    TIMER_PORT->ODR |= TIMER_MASK;
  else
    TIMER_PORT->ODR &= ~TIMER_MASK;
  */
  
 /* if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + CCR3_Val);
  }*/
}


/*************************************************************************
 * Function Name: ADCHandler
 * Parameters: void
 * Return: void
 *
 * Description:
 *		
 *************************************************************************/
void ADCHandler(void)
{
 // unsigned short shortval = ADC_GetConversionValue(ADC1);
  if(VC.PixIdx < 0)
    VC.RawImgBuf[0] =  ADC_GetConversionValue(ADC1) >> 4;
  else
    VC.RawImgBuf[VC.PixIdx] = ADC_GetConversionValue(ADC1) >> 4;// - (FPN_On & FPNMask[PixIdx]);
  VC.PixFilled = true;
  
 // if(shortval > MaxPix)
 //   MaxPix = shortval;
 // if(shortval < MinPix)
 //   MinPix = shortval;
  
  
}

/*************************************************************************
 * Function Name: ParseCommand
 * Parameters: void
 * Return: void
 *
 * Description:
 *		
 *************************************************************************/
void ParseCommand(void)
{  
  switch(Dat.Cmd.Bytes[0])
  {
  case WRITE_CMD:
    switch (Dat.Cmd.Bytes[1]){
    case CALIBRATE_CMD:
      RecordFPN = true;
      //VC.RecordFPNMask();
      break;
    case RESOLUTION_CMD:
      VC.SetNewResolution(Dat.Cmd.Bytes[2], Dat.Cmd.Bytes[3]);
      ResolutionUpdateRequest = true;
      break;
    case OF_RESOLUTION_CMD:
      ImgP.SetBinResolution(Dat.Cmd.Bytes[2], Dat.Cmd.Bytes[3]);
      Dat.UpdateResolution(DATA_ID_OF, ImgP.NumBins[0], ImgP.NumBins[1]*2);
      break;
    case OF_SMOOTHING_CMD:
      ImgP.SetAlpha(Dat.Cmd.Bytes[2]);
      break;
    case DRC_ON_CMD:
      VC.DRC_On = (Dat.Cmd.Bytes[2]) ? true : false;
      break;
    case FPN_ON_CMD:
      VC.FPN_On = (Dat.Cmd.Bytes[2]) ? true : false;
      break;
    case AMP_GAIN_CMD:
      VC.SetAmpGain(Dat.Cmd.Bytes[2]);
      break;
    case HIGH_PASS_CMD:
      ImgP.SetHighPass(Dat.Cmd.Bytes[2]);
      break;
    case SETTLING_TIME_CMD:
      VC.SettlingTime = Dat.Cmd.Bytes[2];
      break;
    case LWTA_THRESH_CMD:
      ImgP.SetLWTAThresh(Dat.Cmd.Bytes[2]);
      break;
    case LWTA_WIN_CMD:
      ImgP.SetWinSize(Dat.Cmd.Bytes[2]);
      break;
    default:
      VC.SetPendingCommand(Dat.Cmd.Bytes[1] - 80, Dat.Cmd.Bytes[2]);
      CMDUpdateRequest = true;
      MinPix = 0xFFFF;
      MaxPix = 0;
      break;
    }
    break;
  case DISPLAY_CMD:
    Dat.SetDSActive(Dat.Cmd.Bytes[1], true);
    break;
  case STOP_CMD:
    Dat.SetDSActive(Dat.Cmd.Bytes[1], false);
    Dat.ResetTX();
    break;
  case READ_CMD:
    VC.ReadCommand(Dat.Cmd.Bytes[1]);
    break;
  case SOH_CHAR:
    Dat.InitHeader(Dat.Cmd.Bytes[1]);
    break;
  case SOD_CHAR:
    Dat.SetActiveArray(Dat.Cmd.Bytes[1]);  
    break;
  case EOD_CHAR:
    SPI_Done = true;
    break;
  default:
    break;
  }
  CIdx = 0;
}

/*************************************************************************
 * Function Name: ParseEsc
 * Parameters: void
 * Return: void
 *
 * Description:
 *		
 *************************************************************************/
void ParseESC(void)
{  
  switch(Dat.Cmd.ESCBytes[1])
  {
   // if escape char is duplicated, ignore it
  case ESC_CHAR:
    Dat.Cmd.Bytes[CIdx] = Dat.Cmd.ESCBytes[1];
    CIdx++;
    break;
   // set write mode
  case WRITE_CHAR:
    Dat.Mode = WRITE_MODE;
    break;
  // set read mode
  case READ_CHAR:
    Dat.Mode = READ_MODE;
    SPI_I2S_SendData(SPI2, Dat.Array[0]);
    DIdx = 1;
    break;
  //start packet flag is received, no action is needed
  case END_PCKT:
    ParseCommand();
    break;
  case START_PCKT:
  default:
    break;
  }
  EIdx = 0;
}

/*************************************************************************
 * Function Name: SPIHandler
 * Parameters: void
 * Return: void
 *
 * Description:
 *		
 *************************************************************************/
void SPIHandler(void)
{
  uint16_t ByteIn;
  
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE))
  {
    // read input byte
    ByteIn = SPI_I2S_ReceiveData(SPI2);
    // first byte is Start of CMD, HeaderRequest, or DataRequest
    // all other bytes are null for data/header request
    
    if((ByteIn == ESC_CHAR) || (EIdx > 0))
    {
      Dat.Cmd.ESCBytes[EIdx] = ByteIn;
      EIdx++;
      if(EIdx == 2)
        ParseESC();
    }
    else if(Dat.Mode == WRITE_MODE)
    {
      Dat.Cmd.Bytes[CIdx] = ByteIn;
      CIdx++;
      if(CIdx == MAX_COMMAND_SIZE)
        CIdx = 0;
    }
    else if (Dat.Mode == READ_MODE)
    {
      if(DIdx < Dat.TxDataSize)
      {
        SPI_I2S_SendData(SPI2, Dat.Array[DIdx]);
        DIdx++;
      }
    }
  }
}

/*************************************************************************
 * Function Name: InitGPIOs
 * Parameters: none
 *
 * Return: none
 *
 * Description: initialize GPIO ports
 *
 *************************************************************************/
void InitGPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin =  RDY_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(RDY_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  CMD_MASK;
  GPIO_Init(CMD_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  LED_MASK;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  RDY2_MASK;
  GPIO_Init(RDY2_PORT, &GPIO_InitStructure);
  
   GPIO_InitStructure.GPIO_Pin =  SDA_MASK;
  GPIO_Init(SDA_PORT, &GPIO_InitStructure);
  
   GPIO_InitStructure.GPIO_Pin =  SCL_MASK;
  GPIO_Init(SCL_PORT, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.GPIO_Pin =  AN_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(AN_PORT, &GPIO_InitStructure);
  
      /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(SPIS_PORT, SPI_SCK_SOURCE, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIS_PORT, SPI_MOSI_SOURCE, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIS_PORT, SPI_MISO_SOURCE, GPIO_AF_SPI2);

  
  GPIO_InitStructure.GPIO_Pin =  SPIS_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(SPIS_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  SPI_MISO_MASK;
  GPIO_Init(SPI_PORT, &GPIO_InitStructure);
  
}

/*************************************************************************
 * Function Name: InitIRQs
 * Parameters: none
 *
 * Return: none
 *
 * Description: initialize interrupts
 *
 *************************************************************************/
void InitIRQs(void)
{
  
  NVIC_InitTypeDef ADC_IRQInitStructure;
  NVIC_InitTypeDef SPI_IRQInitStructure; 
  NVIC_InitTypeDef TIM_IRQInitStructure; 
  
  ADC_IRQInitStructure.NVIC_IRQChannel = ADC_IRQn;
  ADC_IRQInitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  ADC_IRQInitStructure.NVIC_IRQChannelSubPriority = 0;
  ADC_IRQInitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&ADC_IRQInitStructure);
  
  SPI_IRQInitStructure.NVIC_IRQChannel = SPI2_IRQn;
  SPI_IRQInitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  SPI_IRQInitStructure.NVIC_IRQChannelSubPriority = 0;
  SPI_IRQInitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&SPI_IRQInitStructure);
  
  TIM_IRQInitStructure.NVIC_IRQChannel = TIM2_IRQn;
  TIM_IRQInitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  TIM_IRQInitStructure.NVIC_IRQChannelSubPriority = 0;
  TIM_IRQInitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&TIM_IRQInitStructure);
  
}

 int main()
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  SPI_InitTypeDef SPI_InitStructure;
  
  int Count, i, Offset = 0;

  ENTR_CRT_SECTION(); 
  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
  
  // enable peripheral clock for ADC
  RCC_APB2PeriphResetCmd( RCC_APB2Periph_ADC1, DISABLE);
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE);
 
  // enable peripheral clock for spi
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_TIM2, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_TIM2, ENABLE);
  
  // GPIO enable clock and release Reset
  RCC_AHB1PeriphResetCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
                           | RCC_AHB1Periph_GPIOC, DISABLE);
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
                           | RCC_AHB1Periph_GPIOC, ENABLE);
  
  EXT_CRT_SECTION();

  // initialize gpios and interrupts
  InitGPIOs();
  VC.InitGPIOs_Timers(TimE);
  InitIRQs();
  // set flash read latency
  FLASH_SetLatency(FLASH_Latency_3);
  
  // ADC configuration
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC1 regular channel14 configuration
  ADC_RegularChannelConfig(ADC1, AN_CH, 1, ADC_SampleTime_15Cycles);
  
  // enable ADC interrupts
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  // Disable ADC1 DMA
  ADC_DMACmd(ADC1, DISABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // delay to allow time for arduino to startup
  TimE.Delay_s(1);
  
  // initialize spi
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
  
  //SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Reset); 
  
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
  //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
  
  uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000);
  
  // initialize timer
  TimE.Init();
  // initialize vision chip interface
  VC.Init();
  // initialize image processing 
  ImgP.InitHighPass(VC.ResRows, VC.ResCols, VC.MaxSize); 
  ImgP.InitOpticFlow(VC.ResRows, VC.ResCols);
  // initialize datasets to transmit
  Dat.InitDS(DATA_ID_RAW, VC.ResRows, VC.ResCols, VC.RawImgBuf);
  Dat.InitDS(DATA_ID_OF, ImgP.NumBins[0], ImgP.NumBins[1]*2, (unsigned char *)ImgP.OpticFlowScale);
  Dat.InitDS(DATA_ID_FPS, 1, 2, TimE.FPS);
  Dat.InitDS(DATA_ID_MAXES, 1, 2, ImgP.MaxPoints);
  
  RDY_PORT->ODR |= RDY_MASK;
  TimE.Delay_us(50);
  
  while(1)
  {
  
    //****************** GET RAW IMAGE ******************//
    // clear data ready flag (pin)
    RDY_PORT->ODR &= ~RDY_MASK;
    LED_PORT->ODR &= ~LED_MASK;

    // if calibration requested, record new fpn mask
    if(RecordFPN)
    {
      VC.RecordFPNMask();
      RecordFPN = false;
    }
    // if change resolution command received, update vision chip class, data to transmit and image processing
    if(ResolutionUpdateRequest)
    {
      VC.SetResolution(VC.NewRows, VC.NewCols);
      Dat.UpdateResolution(DATA_ID_RAW, VC.ResRows, VC.ResCols);
      ImgP.SetImageResolution(VC.ResRows, VC.ResCols);
      ResolutionUpdateRequest = false;
    }
    // send pending commands (commands sent asyncronously could interfere with image readout)
    if(CMDUpdateRequest)
    {
      VC.SendPendingCommand();
      CMDUpdateRequest = false;
    }
    // get new frame
    VC.ReadRawImage();
    
    // ****************APPLY IMAGE PROCESSING ********** //
    ImgP.HighPass(VC.RawImgBuf);
    ImgP.ComputeOpticFlow(VC.Img1, VC.Img2);  
    // prep optic flow for output
    ImgP.ScaleOpticFlow();
   
    // raw image is double buffered
    VC.SwapBuffers();
    Dat.UpdateDataPointer(DATA_ID_RAW, VC.RawImgBuf);
    ImgP.LocalWinners(VC.RawImgBuf);
    Dat.UpdateResolution(DATA_ID_MAXES, ImgP.NumLWTAPoints, 2);
    
    // apply DRC if turned on 
    if(VC.DRC_On)
      VC.GetDRC();  
    // compute frames per second
    TimE.GetFPS();

    //******************** SEND DATA *********************//
    // set data ready flag (pin) if datasets are requested
     if(Dat.TxActive)
    {
      // flag data read done (read by master)
       RDY_PORT->ODR |= RDY_MASK;
       LED_PORT->ODR |= LED_MASK;
    
    // wait for data Tx to finish
     Count = 0;
     while((SPI_Done == false))// && (Count < SPI_TIME_OUT))
     {
       Count++;
        TimE.Delay_us(50);
     }
     // clear flags
     RDY_PORT->ODR &= ~RDY_MASK;
     SPI_Done = false;
     CMD_PORT->ODR &= ~CMD_MASK;
    }
  }
  return 0;
}
