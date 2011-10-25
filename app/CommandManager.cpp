/*
 
 CommandManager.cpp : holds command values and manages updates.  command values are stored in flash memory
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

#include "CommandManager.h"

/*---------------------------------------------------------------
 CommandManager: constructor 
 ---------------------------------------------------------------*/
CommandManager::CommandManager()
{
}

/*---------------------------------------------------------------
 Init - load command values from flash memory
 or load defaults.  initialize CommandDump Array and CommandIndices Array
 ---------------------------------------------------------------*/
void CommandManager::Init()
{
  
   // initialize commandIndices Arrray
  InitCommandIndices();
  // load commands from flash or from defaults if flash is not yet stored
  InitializeCommands();
  // initialize CommandDump Arrau (transmit array)
  UpdateCommandDump();
 
}

/*---------------------------------------------------------------
 CommandManager: destructor
 ---------------------------------------------------------------*/
CommandManager::~CommandManager()
{
  //nothing to destroy
}

/*---------------------------------------------------------------
SetPendingUpdate: sendCommand to be updated at next computation loop
 Input:  CommandID : command ID (defined in CommandManager.h, table ReadableCommands)
          Value- new value for command
 ---------------------------------------------------------------*/
void CommandManager::SetPendingUpdate(int CommandID, char value)
{
  pendingCmd = CommandID;  pendingVal = value;
}

/*---------------------------------------------------------------
UpdatePending: sendCommand previously stored in pendingCmd, pendingVal
 ---------------------------------------------------------------*/
void CommandManager::UpdatePending()
{
    int Idx = GetCommandIndex(pendingCmd);
   
   if(Idx > 0)
   {
     CommandValues[Idx] = pendingVal;
     
     WriteCommands();
   }
}

/*---------------------------------------------------------------
 UpdateCommand:  Record new command value and write to Flash Memory
 Input:  CommandID : command ID (defined in CommandManager.h, table ReadableCommands)
          Value- new value for command
 ---------------------------------------------------------------*/
 void CommandManager::UpdateCommand(int CommandID, char value)
 {
   int Idx = GetCommandIndex(CommandID);
   
   // check that Idx is valid and write new data to flash memory
   if(Idx > 0)
   {
     CommandValues[Idx] = value;
     
     WriteCommands();
   }
 }

/*---------------------------------------------------------------
 ReadCommands: Write command values from  Command Values Array into Flash Memory and update CommandDump Array
 ---------------------------------------------------------------*/
 void CommandManager::WriteCommands(void)
 { 
   // prepare commands valid flag byte
    unsigned char InitByte[1];
    InitByte[0] = (unsigned char)COMMANDS_INITIALIZED;
    
   // must erase flash memory for each write
   FlashM.FlashErase(NUM_READABLE_COMMANDS+1, FLASH_ADDR_CMDS_FLAG);
   // after erase, cmds valid flag must be rewritten
   FlashM.WriteToFlash(InitByte, 1, FLASH_ADDR_CMDS_FLAG);
   // write commands to flash
   FlashM.WriteToFlash((unsigned char*)CommandValues, NUM_READABLE_COMMANDS, FLASH_ADDR_CMDS);
   // update CommandDump array (arry to transmit)
   UpdateCommandDump();
 }

/*---------------------------------------------------------------
 ReadCommands:  Read command values from Flash Memory into Command Values Array
 ---------------------------------------------------------------*/
 void CommandManager::ReadCommands(void)
 { 
   // read command values from flash  memory
   FlashM.ReadFromFlash((unsigned char*)CommandValues, NUM_READABLE_COMMANDS, FLASH_ADDR_CMDS);
 }

/*---------------------------------------------------------------
 InitializeCommands:  Check if command values have been stored to flash, and if not, load default values
  this function will load default values the first time the chip is programmed and will load
  flash values for all subsequent boots
 ---------------------------------------------------------------*/
void CommandManager::InitializeCommands(void)
{
  unsigned char InitByte[1];
  
  // check if flash commands are already stored by reading commands valid flag byte
  FlashM.ReadFromFlash(InitByte, 1, FLASH_ADDR_CMDS_FLAG);
  // if flag byte is not set, initialize commands with default values and stor to flash
  if(InitByte[0] != COMMANDS_INITIALIZED)
    StoreDefaultCommands();
  // else load command values from flash
  else
    ReadCommands();
}

/*---------------------------------------------------------------
InitCommandIndices: Init Command Indices Array.  Command Indices is a local
array that matches Command IDs to their index in the CommandValues array
 ---------------------------------------------------------------*/
void CommandManager::InitCommandIndices(void)
{
  // assign command values to Command Indices Array.  Values are defined in 
  // the ReadableCommands table in CommandManager.h
  CommandIndices[0] = OF_SMOOTHING_CMD;
  CommandIndices[1] = DRC_ON_CMD;
  CommandIndices[2] = FPN_ON_CMD;
  CommandIndices[3] = AMP_GAIN_CMD; 
  CommandIndices[4] = HIGH_PASS_CMD; 
  CommandIndices[5] = SETTLING_TIME_CMD;
  CommandIndices[6] = LWTA_THRESH_CMD;
  CommandIndices[7] = LWTA_WIN_CMD;
  CommandIndices[8] = VREF_CMD;
  CommandIndices[9] = NBIAS_CMD;
  CommandIndices[10] = AOBIAS_CMD;
  
}

/*---------------------------------------------------------------
StoreDefaultCommands : load default values for commands.  This function only runs
the first time the board is programmed.  On all subsequent boots, commands will be loaded from 
flash memory
 ---------------------------------------------------------------*/
void CommandManager::StoreDefaultCommands(void)
{
  // assign default values for commands.  These are stored in Common.h
  CommandValues[0] = OF_ALPHA_DEFAULT; //  OF_SMOOTHING_CMD
  CommandValues[1] = DRC_ON_DEFAULT; //DRC_ON_CMD
  CommandValues[2] = FPN_ON_DEFAULT; //FPN_ON_CMD 
  CommandValues[3] = AMP_GAIN_DEFAULT; //AMP_GAIN_CMD
  CommandValues[4] = HP_SHIFT_DEFAULT; //HIGH_PASS_CMD 
  CommandValues[5] = SETTLING_TIME_DEFAULT; //SETTLING_TIME_CMD 
  CommandValues[6] = LWTA_THRESH_DEFAULT; //LWTA_THRESH_CMD
  CommandValues[7] = LWTA_WINSIZE_DEFAULT; //LWTA_WIN_CMD 
  CommandValues[8] = VREF_DEFAULT; //VREF_CMD 
  CommandValues[9] = NBIAS_DEFAULT; //NBIAS_CMD 
  CommandValues[10] = AOBIAS_DEFAULT; //AOBIAS_CMD 
  
  // write commands to flash memory
  WriteCommands(); 
}

/*---------------------------------------------------------------
 UpdateCommandDump:  updates dataset of command values to transmit
 ---------------------------------------------------------------*/
void CommandManager::UpdateCommandDump()
{
  // update CommandDump arrray.  The array is a sequence of command-value pairs
  for( int i = 0; i < NUM_READABLE_COMMANDS; i++)
  {
    CommandDump[i*2] = CommandIndices[i];
    CommandDump[i*2 + 1] = CommandValues[i];
  }
}

/*---------------------------------------------------------------
 GetCommandIndex: find index of command in CommandValues Array
 Input:  CommandID : command ID (defined in CommandManager.h, table ReadableCommands)
 return:  returns the index of the Command, or if the command is not found, returns -1  
 ---------------------------------------------------------------*/
int CommandManager::GetCommandIndex(int CommandID)
{
  
  // scan through CommandIndices Array looking for a value that matches CommandID
  for (int i = 0; i < NUM_READABLE_COMMANDS; i++)
  {
    if(CommandIndices[i] == CommandID)
      return i;
  }
  
  // if no value is found to match, return error
  return -1;
}

