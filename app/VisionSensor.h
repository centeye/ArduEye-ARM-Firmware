/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    VisionChip.h      
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

class VisionChip{

public: 
  VisionChip();
  ~VisionChip();
  
  int ResRows, ResCols;
  
  void ReadRawImage();
  void Init();
  

};

class Firefly : public VisionChip{
  
  void ReadRawImage();
  void Init();
  
};