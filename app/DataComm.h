/*
 
 DataComm.h : supports communication interface, keeps track of dataset characteristics
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

#include "arm_comm.h"

// packet flow control flags
const char ESC_CHAR =  38; 
#define EOD_CHAR      92
#define SOD_CHAR      95
#define SOH_CHAR      96
#define START_PCKT 90
#define END_PCKT 91
#define WRITE_CHAR 93
#define READ_CHAR 94

// packet / process types
#define CMD_RECEIVE   1
#define TX_HEAD       2
#define TX_DAT        3
#define NULL_PROCESS  -1
#define NULL_TX       -1

// array sizes
#define HEADER_SIZE   6
#define MAX_DATASETS  6
#define MAX_COMMAND_SIZE 8
#define MAX_ESC_SIZE 2

// cmd definitions
#define WRITE_CMD  32
#define DISPLAY_CMD 33
#define STOP_CMD 35
#define ACK_CMD 34
#define READ_CMD 36
#define CALIBRATE_CMD 70
#define RESOLUTION_CMD 71
#define OF_RESOLUTION_CMD 72
#define OF_SMOOTHING_CMD 73
#define DRC_ON_CMD  74
#define FPN_ON_CMD 75
#define AMP_GAIN_CMD 76
#define HIGH_PASS_CMD 77
#define SETTLING_TIME_CMD 78
#define LWTA_THRESH_CMD 79
#define LWTA_WIN_CMD 80
#define VREF_CMD  81

// data set ids
#define DATA_ID_RAW   48
#define DATA_ID_OF   50
#define DATA_ID_FPS   54
#define DATA_CMD_VAL  56
#define DATA_ID_MAXES 58

// mode definitions
#define WRITE_MODE 10
#define READ_MODE 11

// DataSet Structure - This structure keeps track of the current size and
// array pointer for each dataset
typedef struct DataSets{
 
  bool Active;
  char DSID;
  unsigned short Size, Rows, Cols;
  unsigned char * ArrayPointer;
  
}DataSets;

//CmdWrap Structure - This structure holds incoming bytes for processing
typedef struct CmdWrap{
 
  char Size;
  unsigned char Bytes[MAX_COMMAND_SIZE];
  unsigned char ESCBytes[MAX_ESC_SIZE];
}CmdWrap;
  
// DataManager class prepares data to be sent via spi and keeps track of
// dataset settings
class DataManager{
  
public:
  DataManager();
  ~DataManager();
  
  //FUNCTIONS
  // initialize dataset pointer and size
  bool InitDS(int inDSID, int Rows, int Cols, unsigned char * pointer);
  // update datset pointer
  void UpdateDataPointer(int inDSID, unsigned char * pointer);
  // initialize dataset for txmission
  void SetActiveArray(int inDSID);
  // initialize header for txmission
  void InitHeader(int inDSID);
  // set dataset active flag
  void SetDSActive(int inDSID, bool Enable);
  // update resolution for dataset
  void UpdateResolution(int inDSID, int Rows, int Cols);
  
  //VARIABLES
  // size of currently txmitting dataset
  int TxDataSize;
  // pointer to currently txmitting dataset
  unsigned char * Array;
  // hold header data, is initialized at the beginning of each header txmission
  unsigned char Header[HEADER_SIZE];
  // holds incoming bytes
  CmdWrap Cmd;
  // flag that at least one dataset is active
  bool TxActive;
  // current Tx index in dataset array
  int DSIdx;
  // read / write mode
  char Mode;
  
private:
  //VARIABLES
  // array tracking dataset settings
  DataSets DS[MAX_DATASETS]; 
  // InitIdx tracks number of initialized datasets, ActiveTx is the index
  // of the currently txmitting dataset
  int InitIdx, ActiveTx;
  
};
