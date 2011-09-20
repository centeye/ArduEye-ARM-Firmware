/*
 
 DataComm.cpp : supports communication interface, keeps track of dataset characteristics
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

#include "DataComm.h"

/*---------------------------------------------------------------
 DataManager: constructor - initialize variables
 ---------------------------------------------------------------*/
DataManager::DataManager()
{
  InitIdx = 0;
  ActiveTx = NULL_TX;
  TxActive = false;
  DSIdx = 0;
  TxDataSize = 0;
  Mode = WRITE_MODE;
}

/*---------------------------------------------------------------
 DataManager: destructor
 ---------------------------------------------------------------*/
DataManager::~DataManager()
{
  //nothing to destroy
}
  
/*---------------------------------------------------------------
 InitDS:  assign the DatasetArray pointer and dataset size
 DataManager holds pointers to datasets but the actual arrays are elsewhere
 Input:    inDSID-  dataset ID to be initialized
           Rows-  Row dimension of dataset   
           Cols- Col dimension of dataset
           pointer- pointer to array holding dataset
 ---------------------------------------------------------------*/
bool DataManager::InitDS(int inDSID, int Rows, int Cols, unsigned char * pointer)
{
  // verify that idx is a valid possible dataset
  if(InitIdx > MAX_DATASETS)
    return false;
  
  DS[InitIdx].DSID = inDSID;
  DS[InitIdx].Size = Rows*Cols;
  DS[InitIdx].Rows = Rows;
  DS[InitIdx].Cols = Cols;
  DS[InitIdx].ArrayPointer = pointer;
  InitIdx++;
  
  return true;
}

/*---------------------------------------------------------------
 SetDSActive:  set the active flag for the dataset.  This function is not 
 strictly necessary for the implemented state machine.  But the TxActive flag is still referrenced in main.cpp
 This function is required in state machines that require the sensor to 
 track dataset communication order, so has not been removed in case such a state
 machine is implemented in the future.
 Input:   inDSID-  Dataset ID for set to be activated
         Enable- true for Active, false for inActive
 ---------------------------------------------------------------*/
void DataManager::SetDSActive(int inDSID, bool Enable)
{
  // get index of dataset in DataRecord array
  int Idx = 0;
  while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
  // make sure index represents a valid array
  if(Idx < MAX_DATASETS)
    // set active flag
   DS[Idx].Active = Enable;
  
  // set TxActive flag (global flag indicating at least one dataset is active)
  TxActive = false;
  for (Idx = 0; Idx < MAX_DATASETS; Idx++)
  {
    if(DS[Idx].Active)
    {
      TxActive = true;
      break;
    }
  } 
}

/*---------------------------------------------------------------
 UpdateDataPointer:  Change the data array pointer for a dataset
 Input:  inDSID- Dataset ID
         pointer- new pointer for dataset
 ---------------------------------------------------------------*/
void DataManager::UpdateDataPointer(int inDSID, unsigned char * pointer)
{
  // find index of dataset in DataRecord array
  int Idx = 0;
  while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
  //make sure index represents a valid array
  if(Idx < MAX_DATASETS)
    // set array pointer
    DS[Idx].ArrayPointer = pointer;
}

/*---------------------------------------------------------------
 UpdateResolution:  Change the data array pointer for a dataset
 Input:  inDSID- Dataset ID
         Rows - new row dimension of dataset
         Cols - new column dimension of dataset
 ---------------------------------------------------------------*/
void DataManager::UpdateResolution(int inDSID, int Rows, int Cols)
{
  // find index of dataset in DataRecord array
  int Idx = 0;
   while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
   //make sure index represents a valid array
  if(Idx < MAX_DATASETS)
  {
    // set size parameters
    DS[Idx].Size = Rows*Cols;
    DS[Idx].Rows = Rows;
    DS[Idx].Cols = Cols;
  }
}
/*---------------------------------------------------------------
 SetActiveArray:  set Tx array and Tx size (currently transmitting dataset)
 Input:  inDSID- Dataset ID
 ---------------------------------------------------------------*/
void DataManager::SetActiveArray(int inDSID)
{ 
  ActiveTx = NULL_TX;
  // find index that matches DSID
  for (int i = 0; i < MAX_DATASETS; i++)
    if(DS[i].DSID == inDSID)
    {
      ActiveTx = i;
      break;
    }
  
  // update data array pointer
   if(ActiveTx != NULL_TX)
   {
     // set currently transmitting array + size
      Array = DS[ActiveTx].ArrayPointer;
      TxDataSize = DS[ActiveTx].Size;
   }
}
  
/*---------------------------------------------------------------
 InitHeader:  Initialize dataset header and set as Tx array
 Header: 1 byte DataId, 2 byte rows, 2 byte cols, 1 byte CheckSum 
 Input:  inDSID- Dataset ID
 ---------------------------------------------------------------*/
void DataManager::InitHeader(int inDSID)
{  
  ActiveTx = NULL_TX;
  // find index that matches DSID
  for (int i = 0; i < MAX_DATASETS; i++)
    if(DS[i].DSID == inDSID)
    {
      // 
      ActiveTx = i;
      break;
    }
  // check that index is valid
  if(ActiveTx != NULL_TX)
  {
    // Init header for dataset, clear Dataset index
    Header[0] = inDSID + 1;
    Header[1] = DS[ActiveTx].Rows >> 8;
    Header[2] = DS[ActiveTx].Rows & 0xFF;
    Header[3] = DS[ActiveTx].Cols >> 8;
    Header[4] = DS[ActiveTx].Cols & 0xFF;
    
    //Compute Checksum Byte
    Header[5] = 0;
    for(int i = 0; i < 5; i++)
      Header[5] += Header[i];
    
    // Set TX array and Tx size
    Array = Header;
    TxDataSize = 6;
  }
}
