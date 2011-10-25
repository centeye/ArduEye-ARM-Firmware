/*
 
 FlashManager.cpp : on-chip flash memory read and write functions
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

#include "FlashManager.h"

FlashManager::FlashManager(){}
FlashManager::~FlashManager(){}

/*************************************************************************
 * Function Name: WriteToFlash
 * Description: write flash memory . flash erase must be 
 *        performed prior to calling this function
 * Input:   Data- Data to write
            DataSize- Size of data to write
            StartAddr- Address to start writing data
  return: true if data written successfully, false if not
 *************************************************************************/
TestStatus FlashManager::WriteToFlash(unsigned char *Data, int DataSize, unsigned int StartAddr)
{
  unsigned int Address, EndAddr;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  volatile TestStatus MemoryProgramStatus = PASSED;
  int Idx;
  __IO uint16_t Dat;
  
   /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  
  // get end address
  EndAddr = StartAddr + DataSize;
  
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
  
  // Write memory
  Address = StartAddr;  Idx = 0;
  while((Address < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
  {
    FLASHStatus = FLASH_ProgramByte(Address, Data[Idx]);
    Address++;
    Idx++;
  }
  
  // Check the corectness of written data
  // read data from flash and compare with input data
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
  
  // return true if data validation succeeds and false if not
  return MemoryProgramStatus;
  
}

/*************************************************************************
 * Function Name: FlashErase
 * Description: erase flash sectors
   Input:    DataSize- size of data to be written 
             StartAddr- desired start address in flash memory  
  return:   true if all sectors erase successfully, false if not
 *************************************************************************/
FLASH_Status FlashManager::FlashErase(unsigned int DataSize, unsigned int StartAddr)
{
  uint32_t StartSector, EndSector;
  uint32_t EndAddr, EraseCounter;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  
  //get end address
  EndAddr = StartAddr + DataSize;
  
  /* Get the index of the start and end sectors */
  StartSector = GetSector(StartAddr);
  EndSector = GetSector(EndAddr);
  
   /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

  /* Erase the FLASH pages */
  for(EraseCounter = StartSector; (EraseCounter <= EndSector) && (FLASHStatus == FLASH_COMPLETE); EraseCounter+=8)
  {
    FLASHStatus = FLASH_EraseSector(EraseCounter, VoltageRange_3);
  }
  
  // return true if all sectors erase successfully, false if not
  return FLASHStatus;
}
/*************************************************************************
 * Function Name: ReadFromFlash
 * Description: read data from flash memory
 * Input:   Data- Data array to write data into
            DataSize- Size of data to read
            StartAddr- Address to start reading data
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
 * Description: Find sector where flash address is located
   Input:    Address- address in flash memory
   return:   Sector number where address is located 
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