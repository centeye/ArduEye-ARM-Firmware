/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    DataComm.h
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "arm_comm.h"

// start packet flags
const char ESC_CHAR =  38; //0xFF
#define EOD_CHAR      0xFC
#define SOD_CHAR      0xFE
#define SOH_CHAR      0xFD
#define NULL_CHAR     0

// packet / process types
#define CMD_RECEIVE   1
#define TX_HEAD       2
#define TX_DAT        3
#define NULL_PROCESS  -1
#define NULL_TX       -1

// set parameters
#define HEADER_SIZE   5
#define MAX_DATASETS  4
#define MAX_PCKT_SIZE 512
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

#define START_PCKT 90
#define END_PCKT 91
#define WRITE_CHAR 93
#define READ_CHAR 94

#define VREF_CMD  81

// data set ids
#define DATA_ID_RAW   48
#define DATA_ID_OFX   49
#define DATA_ID_OFY   50
#define DATA_ID_FPS   51
#define DATA_CMD_VAL  52

#define WRITE_MODE 10
#define READ_MODE 11


typedef struct DataSets{
 
  bool Active;
  char DSID;
  unsigned short Size, DSIdx, Rows, Cols;
  unsigned char * ArrayPointer;
  
}DataSets;

typedef struct CmdWrap{
 
  char Size;
  unsigned char Bytes[MAX_COMMAND_SIZE];
  unsigned char ESCBytes[MAX_ESC_SIZE];
}CmdWrap;
  

class DataManager{
  
public:
  DataManager();
  ~DataManager();
  
  //functions
  bool InitDS(int inDSID, int Rows, int Cols, unsigned char * pointer);
  void UpdateDataPointer(int inDSID, unsigned char * pointer);
  void SetActiveArray(int inDSID);
  void InitHeader(int inDSID);
  void SetDSActive(int inDSID, bool Enable);
  bool TxIsNull(void);
  void UpdateResolution(int inDSID, int Rows, int Cols);
  void ResetTX();
  
  //variables
  int TxDataSize;
  unsigned char * Array;
  unsigned char Header[HEADER_SIZE];
  CmdWrap Cmd;
  bool TxActive;
  int DSIdx;
  char Mode;
  
private:
  //varaibles
  DataSets DS[MAX_DATASETS]; 
  int InitIdx, ActiveTx;
  
};
