/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      drv_flash.c
 * @brief     FLASH��д
 *
 * @history - V1.0, 2017-03-10, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_flash_pan159.h"

/*******************************************************************************
 * @brief      FLASH��������(512�ֽ�)
 * @param[in]  addr - ��ʼ��ַ(512�ֽڶ���)
 *             sectors - Ҫ����������������Ŀ
 * @param[out] ��
 * @return     ��
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
 * @brief      FLASH����д��
 * @param[in]  addr   - д����ʼ��ַ��Ҫ��4�ֽڶ���
 *             p_addr - ����ָ��
 *             len    - ���ݳ���(�ֽ�Ϊ��λ)
 * @param[out] ��
 * @return     ��
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
 * @brief      FLASH���ݶ�ȡ
 * @param[in]  addr   - ��ȡ��ʼ��ַ��Ҫ��4�ֽڶ���
 *             len    - ���ݳ���(�ֽ�Ϊ��λ)
 * @param[out] p_data - ���ݻ�����ָ�룬��������
 * @return     ��
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
