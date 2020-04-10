/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_flash.c
 * @brief     FLASH读写
 *
 * @history - V1.0, 2017-03-10, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_flash_pan159.h"

/*******************************************************************************
 * @brief      FLASH擦除扇区(512字节)
 * @param[in]  addr - 起始地址(512字节对齐)
 *             sectors - 要擦除的连续扇区数目
 * @param[out] 无
 * @return     无
 * @history  - V1.0, 2017-03-13, xiaoguolin, first implementation.
*******************************************************************************/
void flash_pan159_erase(uint32_t addr, uint16_t sectors)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_SP_UPDATE();
    while(sectors != 0)
    {
        FMC_Erase(addr);
        addr += 128;
        sectors--;
    }
    FMC_DISABLE_SP_UPDATE();
    FMC_Close();
    SYS_LockReg();
}

/*******************************************************************************
 * @brief      FLASH数据写入
 * @param[in]  addr   - 写入起始地址，要求4字节对齐
 *             p_addr - 数据指针
 *             len    - 数据长度(字节为单位)
 * @param[out] 无
 * @return     无
 * @history  - V1.0, 2017-03-13, xiaoguolin, first implementation.
*******************************************************************************/
void flash_pan159_write(uint32_t addr, void* p_data, uint16_t len)
{
    #define PTR32   ((uint32_t*)p_data)
    #define PTR8    ((uint8_t*)p_data)
    uint16_t i;
    union{
        uint32_t u32;
        struct{
            uint8_t x0;
            uint8_t x1;
            uint8_t x2;
            uint8_t x3;
        }u8;
    }tmp;
    tmp.u32 = 0xFFFFFFFF;

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_SP_UPDATE();
    for(i = 0; i < (len>>2); i++)
    {
        FMC_Write(addr,PTR32[i]);
        addr += 4;
    }
    i <<= 2;
    if(i < len)
    {
        tmp.u8.x0 = PTR8[i++];
        if(i < len)
        {
            tmp.u8.x1 = PTR8[i++];
            if(i < len)
            {
                tmp.u8.x2 = PTR8[i];
            }
        }
        FMC_Write(addr,tmp.u32);
    }
    FMC_DISABLE_SP_UPDATE();
    FMC_Close();
    SYS_LockReg();

    #undef  PTR8
    #undef  PTR32
}

/*******************************************************************************
 * @brief      FLASH数据读取
 * @param[in]  addr   - 读取起始地址，要求4字节对齐
 *             len    - 数据长度(字节为单位)
 * @param[out] p_data - 数据缓冲区指针，帯回数据
 * @return     无
 * @history  - V1.0, 2017-03-13, xiaoguolin, first implementation.
*******************************************************************************/
void flash_pan159_read(uint32_t addr, void* p_data, uint16_t len)
{
    #define PTR32   ((uint32_t*)p_data)
    #define PTR8    ((uint8_t*)p_data)
    #define TMP     (*((uint32_t*)tmp))
    uint16_t i;
    uint8_t tmp[4];

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_SP_UPDATE();
    for(i = 0; i < (len>>2); i++)
    {
        PTR32[i] = FMC_Read(addr);
        addr += 4;
    }
    i <<= 2;
    if(i < len)
    {
        TMP = FMC_Read(addr);
        PTR8[i++] = tmp[0];
        if(i < len)
        {
            PTR8[i++] = tmp[1];
            if(i < len)
            {
                PTR8[i] = tmp[2];
            }
        }
    }
    FMC_DISABLE_SP_UPDATE();
    FMC_Close();
    SYS_LockReg();

    #undef  TMP
    #undef  PTR8
    #undef  PTR32
}
