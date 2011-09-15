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
