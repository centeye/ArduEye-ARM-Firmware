/*
 
 CommandManager.h : holds command values and manages updates.  command values are stored in flash memory
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

#ifndef COMMANDS_H
#define COMMANDS_H

#include "Common.h"
#include "FlashManager.h"

// cmd definitions
#define WRITE_CMD  32
#define DISPLAY_CMD 33
#define STOP_CMD 35
#define ACK_CMD 34
#define CALIBRATE_CMD 70
#define RESOLUTION_CMD 71
#define OF_RESOLUTION_CMD 72

// ReadableCommands (Comamands stored in Flash Memory, these comprise the Command Dump Dataset)
typedef enum 
{
  OF_SMOOTHING_CMD = 73,
  DRC_ON_CMD = 74,
  FPN_ON_CMD = 75,
  AMP_GAIN_CMD = 76,
  HIGH_PASS_CMD = 77,
  SETTLING_TIME_CMD = 78,
  LWTA_THRESH_CMD = 79,
  LWTA_WIN_CMD = 80,
  VREF_CMD = 84,
  NBIAS_CMD = 86,
  AOBIAS_CMD = 87
}ReadableCommands;

#define FLASH_ADDR_CMDS_FLAG   FLASH_ADDR_11
#define FLASH_ADDR_CMDS        FLASH_ADDR_CMDS_FLAG + 1
#define COMMANDS_INITIALIZED  24
#define NUM_READABLE_COMMANDS 11

class CommandManager { 
  
public:
  CommandManager();
  ~CommandManager();
  
   // store most recent command value to flash memory
  void UpdateCommand(int CommandID, char value);
  // store command update to be activated at next computational loop
  void SetPendingUpdate(int CommandID, char value);
  // send pending command to flash
  void UpdatePending();
  // initialize command arrays
  void Init();
  
    // Dataset of command values
  char CommandDump[NUM_READABLE_COMMANDS * 2];
  
private:
  
    // arry to hold the current values for each command / bias / setting
  // this array is stored in flash memory
  char CommandValues[NUM_READABLE_COMMANDS];
  // array matching command indices to Command IDs - indices for command IDs match the Command Values array
  unsigned char CommandIndices[NUM_READABLE_COMMANDS];
  // find the index of a command value in the CommandValues array
  int GetCommandIndex(int CMDid); 
   // check if commands have been stored in flash memory
  void InitializeCommands(void); 
  // Fill CommandDump Array with current values
  void UpdateCommandDump();
  // store command defaults : used if no flash commands have been stored
  void StoreDefaultCommands(void);
  // initialize commandIndices Array
  void InitCommandIndices(void);
    // read command values from flash memory
  void ReadCommands(void);
  // write command values to flash memory and call UpdateCommandDump()
  void WriteCommands(void);
  
  // local copy of FlashManager class
  FlashManager FlashM;
  
  // Pending Cmd, Value pair, set for future update
  int pendingCmd;
  char pendingVal;
  
};

#endif