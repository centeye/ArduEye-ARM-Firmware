/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    DataComm.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "DataComm.h"

// The DataManger class prepares DataSets for txmission to an external board
DataManager::DataManager()
{
  InitIdx = 0;
  ActiveTx = NULL_TX;
  TxActive = false;
  DSIdx = 0;
  TxDataSize = 0;
  Mode = WRITE_MODE;
}

DataManager::~DataManager()
{
  //nothing to destroy
}
  
// InitDS is called by an external function to assign the DatasetArray pointer
// DataManager holds pointers to datasets but the actual arrays are elsewhere
bool DataManager::InitDS(int inDSID, int Rows, int Cols, unsigned char * pointer)
{
  // max datasets (MAX_DATASETS) is defined in DataComm.h
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

bool DataManager::TxIsNull(void)
{
  if(ActiveTx == NULL_TX)
    return true;
  else
    return false;
}

void DataManager::SetDSActive(int inDSID, bool Enable)
{
  int Idx = 0;
  while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
  if(Idx < MAX_DATASETS)
   DS[Idx].Active = Enable;
  
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

void DataManager::UpdateDataPointer(int inDSID, unsigned char * pointer)
{
  int Idx = 0;
  while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
  if(Idx < MAX_DATASETS)
    DS[Idx].ArrayPointer = pointer;
}

void DataManager::UpdateResolution(int inDSID, int Rows, int Cols)
{
  int Idx = 0;
   while((DS[Idx].DSID != inDSID) && (Idx < MAX_DATASETS))
    Idx++;
  
  if(Idx < MAX_DATASETS)
  {
    DS[Idx].Size = Rows*Cols;
    DS[Idx].Rows = Rows;
    DS[Idx].Cols = Cols;
  }
}

void DataManager::ResetTX()
{
  ActiveTx = NULL_TX;
  DSIdx = 0;
}
  
// UpdateTxData prepares a new packet for txmission.  Each packet contains a 
// header and a data section.  DataManager keeps track of the packet sequence
// and dataset sequence
void DataManager::SetActiveArray(int inDSID)
{ 
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
      Array = DS[ActiveTx].ArrayPointer;
      TxDataSize = DS[ActiveTx].Size;
   }
}
  
////Header: 1 byte DataId, 2 byte rows, 2 byte cols 
void DataManager::InitHeader(int inDSID)
{  
  // find index that matches DSID
  for (int i = 0; i < MAX_DATASETS; i++)
    if(DS[i].DSID == inDSID)
    {
      ActiveTx = i;
      break;
    }
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
    
    Array = Header;
    TxDataSize = 6;
  }
}
