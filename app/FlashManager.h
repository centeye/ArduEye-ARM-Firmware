/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Flash.h
 *    Centeye,Inc
 *    Alison Leonard
 *    July 6, 2011
 **************************************************************************/

#include "stm32f2xx_flash.h"
#include "arm_comm.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Sector 11, 128 Kbytes */

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


class FlashManager{

public:
  FlashManager();
  ~FlashManager();
  
  void ReadFromFlash(unsigned char *Data, int DataSize, unsigned int Address);
  TestStatus WriteToFlash(unsigned char *Data, int DataSize, unsigned int StartAddr);
  uint32_t GetSector(uint32_t Address);
  FLASH_Status FlashErase(unsigned int DataSize, unsigned int StartAddr);
};