 /*
 
 main.cpp : main loop, microcontroller setup, interrupt handling 
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
#include "includes.h"

//PORT DEFINITIONS
// Spi Pins
#define SPIS_MASK       ((uint16_t)0xB000)
#define SPI_MISO_MASK   GPIO_Pin_14
#define SPIS_PORT       GPIOB

// Spi Pin Sources for Alternate Function Assignment
#define SPI_MOSI_SOURCE GPIO_PinSource15
#define SPI_MISO_SOURCE GPIO_PinSource14
#define SPI_SCK_SOURCE  GPIO_PinSource13

// DataRdy Flag (goes high when frame capture + processing is done)
#define RDY_MASK        GPIO_Pin_6
#define RDY_PORT        GPIOC

// Analog In
#define AN_MASK         GPIO_Pin_2
#define AN_CH           ADC_Channel_12
#define AN_PORT         GPIOC

// connects to LED
#define LED_MASK      GPIO_Pin_2
#define LED_PORT      GPIOB

// RD2 Pin - not being used in firmware but is broken out on ArduEye
#define RDY2_MASK     GPIO_Pin_4
#define RDY2_PORT     GPIOC

// i2c pins - not implemented in firmware, but broken out on ArduEye
#define SDA_MASK      GPIO_Pin_11
#define SDA_PORT      GPIOB

#define SCL_MASK      GPIO_Pin_10
#define SCL_PORT      GPIOB

// keeps track of dataset settings and incoming cmd bytes
DataManager Dat;
// keeps track of command settings and command flash storage
CommandManager Cmds;
// class containing image processing algorithms
ImageProcessing ImgP;
// class containing timers and delay functions
Timers TimE;
// indices used for parsing incoming spi bytes
unsigned int  EIdx = 0, CIdx = 0, DIdx = 0;
// flag to alert main loop that SPI transfer is complete and new data capture can proceed
bool SPI_Done = FALSE;
// flag to record fpn mask on next data acquisition loop
bool RecordFPN = false;
// flag to send command, update resolution on next data acquisition loop
bool ResolutionUpdateRequest = false, CMDUpdateRequest = false, FlashCmdUpdateRequest = false;
// main Vision Chip class.  Currently implemented vision chips are StonyMan, Firefly, TAM
Stonyman VC;
// defined as an extern in arm_comm.h
Int32U CriticalSecCntr;

/*************************************************************************
 * Function Name: ADCHandler
 * Description: interrupt handler responds to ADC conversion done event
 *		
 *************************************************************************/
void ADCHandler(void)
{
 // debugging catch for potential errors in array address
  if(VC.PixIdx < 0)
    VC.RawImgBuf[0] =  ADC_GetConversionValue(ADC1) >> 4;
  // normal data array read
  else
    VC.RawImgBuf[VC.PixIdx] = ADC_GetConversionValue(ADC1) >> 4;
  // flag to alert VC class that pixel acquisition is complete
  VC.PixFilled = true;
 
  
}

/*************************************************************************
 * Function Name: ParseCommand
 * Description: Parse incoming commands and respond appropriately
 *		
 *************************************************************************/
void ParseCommand(void)
{  
  // GENERAL COMMAND FORMAT : [High Level Command] [Cmd] [Values ..]
  switch(Dat.Cmd.Bytes[0])
  {
    // WRITE_CMD precedes all settingss changes
  case WRITE_CMD:
     Cmds.SetPendingUpdate(Dat.Cmd.Bytes[1], Dat.Cmd.Bytes[2]);
     FlashCmdUpdateRequest = true;
    switch (Dat.Cmd.Bytes[1]){
    case CALIBRATE_CMD:
      // Set flag to record calibration mask on next process loop
      RecordFPN = true;
      break;
    case RESOLUTION_CMD:
      // Set flag to update image resolution on next process loop
      VC.SetNewResolution(Dat.Cmd.Bytes[2], Dat.Cmd.Bytes[3]);
      ResolutionUpdateRequest = true;
      break;
    case OF_RESOLUTION_CMD:
      // Set Optic Flow Bin resolution and update data record
      ImgP.SetBinResolution(Dat.Cmd.Bytes[2], Dat.Cmd.Bytes[3]);
      Dat.UpdateResolution(DATA_ID_OF, ImgP.NumBins[0], ImgP.NumBins[1]*2);
      break;
    case OF_SMOOTHING_CMD:
      // set optic flow smoothing level
      ImgP.SetAlpha(Dat.Cmd.Bytes[2]);
      break;
    case DRC_ON_CMD:
      // turn DRC on or off
      VC.DRC_On = (Dat.Cmd.Bytes[2]) ? true : false;
      break;
    case FPN_ON_CMD:
      // turn fixed pattern noise compensation on or off
      VC.FPN_On = (Dat.Cmd.Bytes[2]) ? true : false;
      break;
    case AMP_GAIN_CMD:
      // set amplification level (valid for stonyman chip only)
      VC.SetAmpGain(Dat.Cmd.Bytes[2]);
      break;
    case HIGH_PASS_CMD:
      // turn high pass filter on or off
      ImgP.SetHighPass(Dat.Cmd.Bytes[2]);
      break;
    case SETTLING_TIME_CMD:
      // set time to wait prior to pixel read-out
      VC.SettlingTime = Dat.Cmd.Bytes[2];
      break;
    case LWTA_THRESH_CMD:
      // set threshold for local winner algorithm (winners must exceed threshold)
      ImgP.SetLWTAThresh(Dat.Cmd.Bytes[2]);
      break;
    case LWTA_WIN_CMD:
      //  set window size for local winner algorithm
      ImgP.SetWinSize(Dat.Cmd.Bytes[2]);
      break;
    default:
      // send generic command to vision chip
      VC.SetPendingCommand(Dat.Cmd.Bytes[1] - 80, Dat.Cmd.Bytes[2]);
      CMDUpdateRequest = true;
      break;
    }
    break;
  case DISPLAY_CMD:
    // set data type to active
    Dat.SetDSActive(Dat.Cmd.Bytes[1], true);
    break;
  case STOP_CMD:
    // set data type to inActive
    Dat.SetDSActive(Dat.Cmd.Bytes[1], false);
    break;
  case SOH_CHAR:
    // start of header character - alerts sensor to send header data
    Dat.InitHeader(Dat.Cmd.Bytes[1]);
    break;
  case SOD_CHAR:
    // start of data character - alerts sensor to send header data
    Dat.SetActiveArray(Dat.Cmd.Bytes[1]);  
    break;
  case EOD_CHAR:
    // end of data character - alerts sensor that all data has been received, OK to start new frame acquisition
    SPI_Done = true;
    break;
  default:
    break;
  }
  // clear command index
  CIdx = 0;
}

/*************************************************************************
 * Function Name: ParseEsc
 * Description: Parse Esc Sequence and respond as necessary
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
  // end packet is received, process incoming command
  case END_PCKT:
    ParseCommand();
    break;
  //start packet flag is received, no action is needed
  case START_PCKT:
  default:
    break;
  }
  // reset escape sequence index
  EIdx = 0;
}

/*************************************************************************
 * Function Name: SPIHandler
 * Description: manage incoming spi communication.  called in response to 
 *  spi RX interrupt		
 *************************************************************************/
void SPIHandler(void)
{
  uint16_t ByteIn;
  
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE))
  {
    // read input byte
    ByteIn = SPI_I2S_ReceiveData(SPI2);
    
    // ESC_CHAR processing has priority 
    // escape sequences are two bytes long
    if((ByteIn == ESC_CHAR) || (EIdx > 0))
    {
      Dat.Cmd.ESCBytes[EIdx] = ByteIn;
      EIdx++;
      if(EIdx == 2)
        ParseESC();
    }
    // if spi handler is in write mode, record data bytes for processing
    else if(Dat.Mode == WRITE_MODE)
    {
      Dat.Cmd.Bytes[CIdx] = ByteIn;
      CIdx++;
      if(CIdx == MAX_COMMAND_SIZE)
        CIdx = 0;
    }
    // if spi handler is in read more, prepare next byte for txmission
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
 * Function Name: SetDefaultCommandValues
 * Description: Set Sensor settings to default values stored in flash memory
 *		
 *************************************************************************/
void SetDefaultCommandValues()
{
  // use the ParseCommand Function by storing command values in Dat.Cmd.Bytes
  Dat.Cmd.Bytes[0] = WRITE_CMD;
  // cycle through all commands stored in flash
  for (int i = 0; i < NUM_READABLE_COMMANDS; i++)
  {
    Dat.Cmd.Bytes[1] = Cmds.CommandDump[i*2];
    Dat.Cmd.Bytes[2] = Cmds.CommandDump[i*2 + 1];
    // the parse command function takes care of appropriate cmd setting actions
    ParseCommand();
    // send commands to vision chip if appropriate.  the ParseCommand function sets 
    // a flag to send the commands in the next computation loop, but for initialization
    // we want to send the command right away
    if(CMDUpdateRequest)
    {
      VC.SendPendingCommand();
      CMDUpdateRequest = false;
    }
  }
  // commands do not need to be stored in flash as they have not been updated
  FlashCmdUpdateRequest = false;
}

/*************************************************************************
 * Function Name: InitGPIOs
 * Description: initialize GPIO ports (Required for operating IO)
 *
 *************************************************************************/
void InitGPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Output pins initialized, set to maximum io speed
  GPIO_InitStructure.GPIO_Pin =  RDY_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(RDY_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  LED_MASK;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  RDY2_MASK;
  GPIO_Init(RDY2_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  SDA_MASK;
  GPIO_Init(SDA_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  SCL_MASK;
  GPIO_Init(SCL_PORT, &GPIO_InitStructure);
  
  // Analog input pin, remove pullup resistor
  GPIO_InitStructure.GPIO_Pin =  AN_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(AN_PORT, &GPIO_InitStructure);
  
  // Connect SPI pins to alternate function 5 (AF5) 
  GPIO_PinAFConfig(SPIS_PORT, SPI_SCK_SOURCE, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIS_PORT, SPI_MOSI_SOURCE, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIS_PORT, SPI_MISO_SOURCE, GPIO_AF_SPI2);

  // init spi pins
  GPIO_InitStructure.GPIO_Pin =  SPIS_MASK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(SPIS_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  SPI_MISO_MASK;
  GPIO_Init(SPIS_PORT, &GPIO_InitStructure);
  
}

/*************************************************************************
 * Function Name: InitIRQs
 * Description: initialize interrupts
 *
 *************************************************************************/
void InitIRQs(void)
{
  // initialize interrupts for SPI and ADC
  NVIC_InitTypeDef IRQInitStructure;

  IRQInitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  IRQInitStructure.NVIC_IRQChannelSubPriority = 0;
  IRQInitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  IRQInitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_Init(&IRQInitStructure);
  
  IRQInitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_Init(&IRQInitStructure);
}

/*************************************************************************
 * Function Name: main
 * Description: main program loop
 *
 *************************************************************************/
 int main()
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();
  
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
 
   // set flash read latency
  // latency setting defined by chip clock and voltage range
  FLASH_SetLatency(FLASH_Latency_3);
  // initialize command storage class (pull command values from flash memory
  Cmds.Init();
  // set command values to their defaults
  SetDefaultCommandValues();
  
  // initialize gpios 
  InitGPIOs();
  VC.InitGPIOs_Timers(TimE);
  // initialize interrupts
  InitIRQs();
  
  // ADC configuration
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; // fastest possible clock
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // shortest possibly sampling delay
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12 bit resolution
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // single conversion mode
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1; // one conversion per call
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC1 regular channel14 configuration
  ADC_RegularChannelConfig(ADC1, AN_CH, 1, ADC_SampleTime_3Cycles); // shortest possible sample time
  
  // enable ADC interrupts
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

  // Disable ADC1 DMA
  ADC_DMACmd(ADC1, DISABLE);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // delay to allow time for arduino to startup
  TimE.Delay_s(5);
  
  // initialize spi
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave; // slave
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  // "spi mode 3"
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard; //spi works with or without chip select
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
  
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
  
  // initialize timer
  TimE.Init();
  // initialize vision chip interface
  VC.Init();
  // initialize image processing 
  ImgP.InitHighPass(VC.ResRows, VC.ResCols); 
  ImgP.InitOpticFlow(VC.ResRows, VC.ResCols);
  // initialize datasets to transmit
  Dat.InitDS(DATA_ID_RAW, VC.ResRows, VC.ResCols, VC.RawImgBuf);
  Dat.InitDS(DATA_ID_OF, ImgP.NumBins[0], ImgP.NumBins[1]*2, (unsigned char *)ImgP.OpticFlowScale);
  Dat.InitDS(DATA_ID_FPS, 1, 2, TimE.FPS);
  Dat.InitDS(DATA_ID_MAXES, 1, 2, ImgP.MaxPoints);
  Dat.InitDS(DATA_CMD_VAL, 2,  NUM_READABLE_COMMANDS, (unsigned char *)Cmds.CommandDump);
  
  // raise flag to indicate that sensor boot is complete.  
  RDY_PORT->ODR |= RDY_MASK;
  TimE.Delay_us(50);
  
  // main process loop
  while(1)
  {
  
    //****************** GET RAW IMAGE ******************//
    // clear data ready flag (pin)
    RDY_PORT->ODR &= ~RDY_MASK;
    // init LED to low
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
    // store pending commands to flash (commands sent asyncronously could interfere with image readout)
    if(FlashCmdUpdateRequest)
    {
      Cmds.UpdatePending();
      FlashCmdUpdateRequest = false;
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
    // get local winners
    ImgP.LocalWinners(VC.RawImgBuf);
    // update size of local winners set (different every frame!)
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
       // LED copycat - TODO: set up LED to indicate different behaviors
       LED_PORT->ODR |= LED_MASK;
    
    // wait for data Tx to finish -- Wait for End of Data flag from comm bus
     while(SPI_Done == false)
        TimE.Delay_us(50);

     // clear flags
     RDY_PORT->ODR &= ~RDY_MASK;
     SPI_Done = false;
    }
  }

}
