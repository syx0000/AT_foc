/**
  **************************************************************************
  * @file     flash.c
  * @brief    flash program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "flash.h"
#include "boot_usart.h"
#include "iap.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup USART_iap_bootloader
  * @{
  */

/**
  * @brief  flash erase/program operation.
  * @note   follow 2kb operation of ont time
  * @param  none
  * @retval none
  */
void flash_2kb_write(uint32_t write_addr, uint8_t* pbuffer)
{
    uint16_t index, write_data;
    flash_unlock();
    flash_sector_erase(write_addr);
    if(FLASH_SIZE < 0x100)  /* less than 256kb, 1kb/sector */
        flash_sector_erase(write_addr + 0x400);
    for(index = 0; index < 2048; index += 2)
    {
        write_data = (pbuffer[index+1] << 8) + pbuffer[index];
        flash_halfword_program(write_addr, write_data);
        write_addr += sizeof(uint16_t);
    }
    flash_lock();
}

/**
  * @brief  check flash upgrade flag.
  * @param  none
  * @retval none
  */
flag_status flash_upgrade_flag_read(void)
{
    //return SET;//ֱ�Ӵ���ota����
    if((*(uint32_t*)IAP_UPGRADE_FLAG_ADDR) == IAP_UPGRADE_FLAG)
        return SET;
    else
        return RESET;
}

/**
  * @}
  */

int flash_write(uint32_t base, uint8_t* buf, uint32_t size)
{
    int index;
    int length;
    uint32_t data;
    uint8_t* pbuf = buf;
    // ��������buffer���Ȳ���4��������������Ҫ���ȡ4���ֽڣ���֤���������ݵ�������
    length = size / 4;
    length += ((size % 4) > 0 ? 1 : 0);
    /* unlock the flash program/erase controller */
    flash_unlock();
    for(index = 0; index < length; index ++)
    {
        memcpy(&data, pbuf, sizeof(data));
        pbuf += sizeof(data);
        flash_word_program(base + sizeof(data) * index, data);
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    }
    /* lock the main FMC after the program operation */
    flash_lock();
    return 0;
}

void ReadMCUFlash(uint32_t Address, uint32_t* Data)
{
    *(Data) = (*(volatile uint32_t*)Address);
}

int flash_read(uint32_t base, uint8_t* buf, uint32_t size)
{
    int index = 0,length;
    uint32_t data = 0;
    uint8_t* pbuf = buf;

    length = size / 4;
    length += ((size % 4) > 0 ? 1 : 0);

    for(index = 0; index < length; index ++)
    {
        ReadMCUFlash((base + (sizeof(data) * index)), &data);
        memcpy(pbuf, &data, sizeof(data));
        pbuf += sizeof(data);
    }

    return index;
}

/**
  * @}
  */
int flash_erase(uint32_t base, int len)
{
    int index;
    int pages = 0;
    int result = 0;
    uint32_t address = 0;
    pages += (len / FLASH_PAGE_SIZE_OTA);
    pages += ((len % FLASH_PAGE_SIZE_OTA) > 0) ? 1 : 0;
    /* unlock the flash program/erase controller */
    flash_unlock();
    for(index = 0; index < pages; index ++)
    {
        address = (base + (index * FLASH_PAGE_SIZE_OTA));
        /* clear all pending flags */
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        /* erase the flash pages */
        result = flash_sector_erase(address);
        flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
        if(result != FLASH_OPERATE_DONE)
        {
            break;
        }
    }
    /* lock the main FMC after the erase operation */
    flash_lock();
    return result;
}

int flash_copy(uint32_t des, uint32_t	src, uint32_t size)
{
    int length = 0;
    uint8_t read_buf[1024] = {0,};

    flash_erase(des,size);

    for(int i = 0; i < size; i += sizeof(read_buf))
    {
        if((length = flash_read(src,read_buf,sizeof(read_buf))) > 0)
        {
            flash_write(des,read_buf,length*4);
            memset(read_buf,0,1024);
            src += sizeof(read_buf);
            des += sizeof(read_buf);
        }
        else
            break;
    }

    return 0;
}
