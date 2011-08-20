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
  TxDataSize = 0;
  ActiveTx = NULL_TX;
  TxActive = false;
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
  UpdateTxData();
}

  
// UpdateTxData prepares a new packet for txmission.  Each packet contains a 
// header and a data section.  DataManager keeps track of the packet sequence
// and dataset sequence
void DataManager::UpdateTxData()
{ 
  // if ActiveTx is not set, find first active dataset
  if(ActiveTx == NULL_TX)
  {
    ActiveTx = 0;
    while((DS[ActiveTx].Active == false) && (ActiveTx < MAX_DATASETS))
      ActiveTx++;
    
    InitHeader(); 
  }  
  // if last datset is done sending, find next active dataset
  else if (Header[10] == 1)
  {
    ActiveTx++;
    while((DS[ActiveTx].Active == false) && (ActiveTx < MAX_DATASETS))
      ActiveTx++;
    
    InitHeader(); 
  }
 
  FillHeader();
  
  // update data array pointer
   if(ActiveTx != NULL_TX)
      Array = DS[ActiveTx].ArrayPointer + DS[ActiveTx].DSIdx;
}
  
////Header: ESC, 2bytes pckt size, 1 byte DataId, 1 byte rows, 1 byte cols , 2Bytes dataIndex, 1 Byte EODS
void DataManager::InitHeader()
{ 
// send null header (packet size = 0) if no active dataset  
  if(((ActiveTx >= MAX_DATASETS) || DS[ActiveTx].Active == false))
  {
    Header[1] = Header[2] = 0;
    
    ActiveTx = NULL_TX;
    return;
  }
  
  // Init header for dataset, clear Dataset index
  Header[0] = ESC_CHAR;
  Header[3] = DS[ActiveTx].DSID;
  Header[4] = DS[ActiveTx].Rows >> 8;
  Header[5] = DS[ActiveTx].Rows & 0xFF;
  Header[6] = DS[ActiveTx].Cols >> 8;
  Header[7] = DS[ActiveTx].Cols & 0xFF;
  DS[ActiveTx].DSIdx = 0;
  TxDataSize = 0;
}

void DataManager::FillHeader()
{
  // return if no active dataset
  if(ActiveTx == NULL_TX)
    return;
  
  // update DSIdx and fill header accordingly
  DS[ActiveTx].DSIdx += TxDataSize;
  if((DS[ActiveTx].Size - DS[ActiveTx].DSIdx) <= 0)
    DS[ActiveTx].DSIdx = 0;   

  if((DS[ActiveTx].Size - DS[ActiveTx].DSIdx) <= MAX_PCKT_SIZE)
  {
    TxDataSize =  DS[ActiveTx].Size - DS[ActiveTx].DSIdx;
    Header[10] = 1;
  }
  else
  {
    TxDataSize = MAX_PCKT_SIZE;
    Header[10] = 0;
  }
  Header[1] = (TxDataSize + HEADER_OFFSET) >> 8;
  Header[2] = (TxDataSize + HEADER_OFFSET) & 0xFF;
  Header[8] = DS[ActiveTx].DSIdx >> 8;
  Header[9] = DS[ActiveTx].DSIdx & 0xFF;

}
 