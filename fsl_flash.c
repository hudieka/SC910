#include "fsl_flash.h"
#include "adc_hw_trigger.h"

//LWord flash_buf[256];

void Kinetis_FlashInit()
{
  
  uint32_t  i = 0; 

  FLASH_Initialization();
  
#if TEST_FLASH
  FLASH_EraseSector(0xF800);
  
  for (i = 0; i < 256; i++)
  {
    flash_buf[i] = i;
  }
  
  FLASH_PROGRAM(0xF800, flash_buf, 256);
#endif
  
}
