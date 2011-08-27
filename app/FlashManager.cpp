/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Flash.cpp
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "FlashManager.h"

FlashManager::FlashManager(){}
FlashManager::~FlashManager(){}
/*************************************************************************
 * Function Name: WriteToFlash
 * Parameters: none
 *
 * Return: none
 *
 * Description: write flash memory .. note that flash erase must be 
 *        performed prior to calling this function
 *
 *************************************************************************/
TestStatus FlashManager::WriteToFlash(unsigned char *Data, int DataSize, unsigned int StartAddr)
{
  unsigned int Address, EraseCounter, EndAddr;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  volatile TestStatus MemoryProgramStatus = PASSED;
  uint32_t StartSector, EndSector;
  int Idx;
  __IO uint16_t Dat;
  
   /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  
  EndAddr = StartAddr + DataSize;
  
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
  
  // Program
  Address = StartAddr;  Idx = 0;
  while((Address < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
  {
    FLASHStatus = FLASH_ProgramByte(Address, Data[Idx]);
    Address++;
    Idx++;
  }
  
  /* Check the corectness of written data */
  Address = StartAddr; Idx = 0;
  while((Address < EndAddr) && (MemoryProgramStatus != FAILED))
  {
    Dat = (*(__IO uint8_t*) Address);
    if((Dat != Data[Idx]))
    {
      MemoryProgramStatus = FAILED;
    }
  
    Address ++;
    Idx++;
  }
  
  return MemoryProgramStatus;
  
}

/*************************************************************************
 * Function Name: FlashErase
 * Parameters: none
 *
 * Return: none
 *
 * Description: erase flash sectors
 *
 *************************************************************************/
FLASH_Status FlashManager::FlashErase(unsigned int DataSize, unsigned int StartAddr)
{
  uint32_t StartSector, EndSector;
  uint32_t EndAddr, EraseCounter;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  
  EndAddr = StartAddr + DataSize;
  
  /* Get the number of the start and end sectors */
  StartSector = GetSector(StartAddr);
  EndSector = GetSector(EndAddr);
  
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

  /* Erase the FLASH pages */
  for(EraseCounter = StartSector; (EraseCounter < EndSector) && (FLASHStatus == FLASH_COMPLETE); EraseCounter+=8)
  {
    FLASHStatus = FLASH_EraseSector(EraseCounter, VoltageRange_3);
  }
  
  return FLASHStatus;
}
/*************************************************************************
 * Function Name: ReadFromFlash
 * Parameters: none
 *
 * Return: none
 *
 * Description: clear command pins for writing
 *
 *************************************************************************/
void FlashManager::ReadFromFlash(unsigned char *Data, int DataSize, unsigned int Address)
{
  int Idx;

  for (Idx = 0; Idx < DataSize; Idx++)
  {
    Data[Idx] = (*(__IO uint8_t*) Address);
    Address ++;
  }
}


/*************************************************************************
 * Function Name: GetSector
 * Parameters: none
 *
 * Return: none
 *
 * Description: Find sector where flash address is located
**************************************************************************/
uint32_t FlashManager::GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}