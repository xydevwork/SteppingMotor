#include "flash.h"

uint32_t		count_number=0;
uint16_t  exti_flag=0;



void FLASHINIT_FLASH(uint32_t input_flash_data)
{	
	 
		HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = flash_adr;
    f.NbPages = 1;
    //??PageError
    uint32_t PageError = 0;
    //??????
    HAL_FLASHEx_Erase(&f, &PageError);

    //3??FLASH??
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_adr, input_flash_data);

    //4???FLASH
		HAL_FLASH_Lock();
	
	
	
}
