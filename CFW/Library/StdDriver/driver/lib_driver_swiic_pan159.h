/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      driver_swiic_pan159.h
 * @brief     PAN159 GPIO IIC通用驱动
 *
 * @history - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
#ifndef __DRIVER_SWIIC_PAN159_H
#define __DRIVER_SWIIC_PAN159_H

#ifdef __cplusplus
extern "C"{
#endif

#include "lib_driver_delay_pan159.h"
    
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_578K    0x00000000
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_540K    0x00010000
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_515K    0x00020000
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_476K    0x00010001
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_457K    0x00020001
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_426K    0x00020002
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_412K    0x00030002
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_397K    0x00040002
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_385K    0x00030003
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_373K    0x00040003
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_361K    0x00050003
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_351K    0x00060003
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_341K    0x00050004
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_331K    0x00060004
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_322K    0x00070004
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_313K    0x00060005
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_306K    0x00070005
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_298K    0x00080005
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_291K    0x00090005
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_284K    0x00080006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_278K    0x00090006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_271K    0x000A0006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_265K    0x000B0006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_259K    0x000C0006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_254K    0x000D0006
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_249K    0x000C0007
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_244K    0x000D0007
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_239K    0x000E0007
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_234K    0x000F0007
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_230K    0x000E0008
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_225K    0x000F0008
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_220K    0x00100008
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_215K    0x00110008
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_200K    0x0010000B
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_190K    0x0011000C
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_181K    0x0010000E
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_171K    0x00100010
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_161K    0x00140010
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_151K    0x00190010
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_141K    0x001E0010
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_129K    0x001E0014
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_119K    0x001E0018
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_108K    0x001E001D
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_100K    0x00220020
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_81K     0x00380023
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_69K     0x00480028
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_54K     0x00580038
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_42K     0x00690050
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_31K     0x00880070
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_20K     0x010000A0
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_10K     0x02000160
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_1K      0x0FFF0F69
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_750     0x1A01123B
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_500     0x20001EDD
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_200     0x60004539
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_FASTEST 0x00000000
#define SWIIC_PAN159_SPEED_SYS_48M_IIC_DEFAULT 0xFFFFFFFF

typedef struct{
    volatile uint32_t *sda;
    volatile uint32_t *scl;
    union{
        uint32_t code;
        struct{
            uint16_t l;
            uint16_t h;
        }scl_tim_code;
    }speed;
}swiic_pan159_t;

/*******************************************************************************
 * @brief      GPIO IIC初始化
 * @param[in]  hiic       - IIC设备句柄
 *             iic_speed  - IIC速率
 *             sda_port_n - SDA端口号
 *             sda_pin_n  - SDA引脚号
 *             scl_port_n - SCL端口号
 *             scl_pin_n  - SCL引脚号
 * @return     无
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
void swiic_pan159_init(swiic_pan159_t* hiic, uint32_t iic_speed,
                       uint32_t sda_port_n, uint32_t sda_pin_n,
                       uint32_t scl_port_n, uint32_t scl_pin_n);

/*******************************************************************************
 * @brief      GPIO IIC读取(该函数读取具有一定的风险,无法区分读取失败和读到0)
 * @param[in]  dhiic     - 目标IIC设备句柄
 *             shiic     - 源IIC设备句柄
 *             iic_speed - IIC速率
 * @return     无
 * @history  - V1.0, 2017-09-12, xiaoguolin, first implementation.
*******************************************************************************/
void swiic_pan159_copy(swiic_pan159_t* dhiic, const swiic_pan159_t* shiic,
                       uint32_t iic_speed);

/*******************************************************************************
 * @brief      GPIO IIC读取(该函数读取具有一定的风险,无法区分读取失败和读到0)
 * @param[in]  hiic      - IIC设备句柄
 *             dev       - IIC设备地址
 *             reg       - 寄存器地址
 * @return     读取到的字节
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint8_t swiic_pan159_read_byte(const swiic_pan159_t* hiic,
                               uint8_t dev, uint8_t reg);

/*******************************************************************************
 * @brief      GPIO IIC读取
 * @param[in]  hiic      - IIC设备句柄
 *             dev       - IIC设备地址
 *             reg       - 寄存器地址
 *             len       - 要读取的数据长度
 * @param[out] _buf      - 数据缓冲区
 * @return     读取到的字节数
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint32_t swiic_pan159_read_bytes(const swiic_pan159_t* hiic,
                                 uint8_t  dev, uint8_t reg,
                                 uint8_t* buf, uint32_t len);

/*******************************************************************************
 * @brief      GPIO IIC写入
 * @param[in]  hiic      - IIC设备句柄
 *             dev       - IIC设备地址
 *             reg       - 寄存器地址
 *             ubyte     - 待写入字节
 * @return     1 - 成功
 *             0 - 失败
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint8_t swiic_pan159_write_byte(const swiic_pan159_t* hiic,
                                uint8_t dev, uint8_t reg, uint8_t ubyte);

/*******************************************************************************
 * @brief      GPIO IIC写入
 * @param[in]  hiic      - IIC设备句柄
 *             dev       - IIC设备地址
 *             reg       - 寄存器地址
 *             buf       - 数据缓冲区
 *             len       - 要写入的数据长度
 * @return     写入的字节数
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint32_t swiic_pan159_write_bytes(const swiic_pan159_t* hiic,
                                  uint8_t  dev, uint8_t reg,
                                  const uint8_t* buf, uint32_t len);

typedef void (*TCallback)(void);
void iic_pan159_init(void);
void iic_start_send_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk);
void iic_start_read_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk);


#ifdef __cplusplus
}
#endif

#endif  /*  __DRIVER_SWIIC_PAN159_H */

