/*******************************************************************************
 * @note      Copyright (C) 2017 Shanghai Panchip Microelectronics Co., Ltd.
 *            All rights reserved.
 *
 * @file      driver_swiic_pan159.c
 * @brief     PAN159 GPIO IICͨ������
 *
 * @history - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
#include "lib_driver_swiic_pan159.h"
#include "string.h"

#define __swiic_pan159_delay                  __delay_pan159

#define __swiic_pan159_mutex_lock(hiic)       __disable_irq()
#define __swiic_pan159_mutex_unlock(hiic)     __enable_irq()

#define __swiic_pan159_read_sda(hiic)         (*((hiic)->sda))
#define __swiic_pan159_write_sda(hiic,val)    (*((hiic)->sda)=(val))
#define __swiic_pan159_write_scl(hiic,val)    (*((hiic)->scl)=(val))

#define __swiic_pan159_set_speed(hiic,spd)    ((hiic)->speed.code=(spd))
#define __swiic_pan159_get_speed(hiic)        ((hiic)->speed.code)

//#define __swiic_pan159_echo_wclk(hiic)                         \
//    do{                                                        \
//        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
//        __swiic_pan159_write_scl((hiic),1);                    \
//        __swiic_pan159_delay((hiic)->scl_tim_code_h);          \
//        __swiic_pan159_write_scl((hiic),0);                    \
//        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
//    }while(0)
void __swiic_pan159_echo_wclk(const swiic_pan159_t* hiic)
{
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
    __swiic_pan159_write_scl(hiic,1);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.h);
    __swiic_pan159_write_scl(hiic,0);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
}

#define __swiic_pan159_echo_rclk(hiic,pret)                    \
    do{                                                        \
        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
        __swiic_pan159_write_scl((hiic),1);                    \
        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
        *(pret) = __swiic_pan159_read_sda((hiic));             \
        __swiic_pan159_write_scl((hiic),0);                    \
        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
    }while(0)
    
/**
 * IIC START SIGNAL: 
 *     ___
 * SDA    \_______
 *      ______
 * SCL /      \___
 */
//#define __swiic_pan159_start(hiic)                             \
//    do{                                                        \
//        __swiic_pan159_write_sda((hiic),1);                    \
//        __swiic_pan159_write_scl((hiic),1);                    \
//        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
//        __swiic_pan159_write_sda((hiic),0);                    \
//        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
//        __swiic_pan159_write_scl((hiic),0);                    \
//        __swiic_pan159_delay((hiic)->speed.scl_tim_code.l);    \
//    }while(0)
void __swiic_pan159_start(const swiic_pan159_t* hiic)                  
{
    __swiic_pan159_write_sda(hiic,1);
    __swiic_pan159_write_scl(hiic,1);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
    __swiic_pan159_write_sda(hiic,0);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
    __swiic_pan159_write_scl(hiic,0);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
}

/**
 * IIC STOP SIGNAL: 
 *         ___
 * SDA ___/
 *      ______
 * SCL /       
 */
//#define __swiic_pan159_stop(hiic)                        \
//    do{                                                  \
//        __swiic_pan159_write_sda((hiic),0);              \
//        __swiic_pan159_write_scl((hiic),1);              \
//        __swiic_pan159_delay((hiic)->scl_tim_code_l);    \
//        __swiic_pan159_write_sda((hiic),1);              \
//    }while(0)
void __swiic_pan159_stop(const swiic_pan159_t* hiic)
{
    __swiic_pan159_write_sda(hiic,0);
    __swiic_pan159_write_scl(hiic,1);
    __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
    __swiic_pan159_write_sda(hiic,1);
}

/**
 * IIC ACK SIGNAL: 
 *                  _
 * SDA \___________/
 *         ____
 * SCL ___/    \_____
 */
#define __swiic_pan159_write_ack(hiic)               \
    do{                                              \
        __swiic_pan159_write_sda((hiic),0);          \
        __swiic_pan159_echo_wclk((hiic));            \
        __swiic_pan159_write_sda((hiic),1);          \
    }while(0)

/**
 * IIC NACK SIGNAL: 
 *     ____________
 * SDA 
 *         ____
 * SCL ___/    \___
 */
#define __swiic_pan159_write_nack(hiic)               \
    do{                                               \
        __swiic_pan159_write_sda((hiic),1);           \
        __swiic_pan159_echo_wclk((hiic));             \
    }while(0)

void __swiic_pan159_write_byte(const swiic_pan159_t *hiic, uint8_t _ubyte)
{
    register uint8_t ubyte = _ubyte;
    register uint8_t i;
    for(i = 0; i < 8; i++){
        __swiic_pan159_write_sda(hiic,((ubyte & 0x80) ? 1 : 0));
        __swiic_pan159_echo_wclk(hiic);
        ubyte <<= 1;
    }
}

uint8_t __swiic_pan159_read_byte(const swiic_pan159_t* hiic)
{
    register uint8_t ubyte = 0;
    register uint8_t i;
    for(i = 0; i < 8; i++){
        ubyte <<= 1;
        __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
        __swiic_pan159_write_scl(hiic,1);
        __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
        ubyte |= __swiic_pan159_read_sda(hiic);
        __swiic_pan159_write_scl(hiic,0);
        __swiic_pan159_delay(hiic->speed.scl_tim_code.l);
    }
    return ubyte;
}

uint8_t __swiic_pan159_read_ack(const swiic_pan159_t* hiic)
{
    uint8_t ret;
    __swiic_pan159_write_sda(hiic,1);
    __swiic_pan159_echo_rclk(hiic,&ret);
    return ret;
}

/*******************************************************************************
 * @brief      GPIO IIC��ʼ��
 * @param[in]  hiic       - IIC�豸���
 *             iic_speed  - IIC����
 *             sda_port_n - SDA�˿ں�
 *             sda_pin_n  - SDA���ź�
 *             scl_port_n - SCL�˿ں�
 *             scl_pin_n  - SCL���ź�
 * @return     1 - �ɹ�, 0 - ʧ��
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
void swiic_pan159_init(swiic_pan159_t* hiic, uint32_t iic_speed,
                     uint32_t sda_port_n, uint32_t sda_pin_n,
                     uint32_t scl_port_n, uint32_t scl_pin_n)
{
    hiic->sda = &GPIO_PIN_ADDR(sda_port_n,sda_pin_n);
    hiic->scl = &GPIO_PIN_ADDR(scl_port_n,scl_pin_n);
    /**
     * us * 6 - 4 = code
     * ʱ�ӷֲ�Ϊ low : high = 2 : 1, ��˷�Ϊ 3 ��
     *                  ______
     *   \______ ______/      \
     *       low - 2   high - 1
     * ���code = us * 2 - 1
     */
    __swiic_pan159_set_speed(hiic, iic_speed);
    #define __PORT__(n)       ((GPIO_T *) (AHBPERIPH_BASE + (0x04000 | ((n) << 6))))
    #define __PIN__(n)        (1 << (n))
    GPIO_SetMode(__PORT__(sda_port_n),__PIN__(sda_pin_n),GPIO_MODE_OPEN_DRAIN);
    GPIO_SetMode(__PORT__(scl_port_n),__PIN__(scl_pin_n),GPIO_MODE_OPEN_DRAIN);
    #undef  __PORT__
    #undef  __PIN__
    __swiic_pan159_stop(hiic);
}

/*******************************************************************************
 * @brief      GPIO IIC��ȡ(�ú�����ȡ����һ���ķ���,�޷����ֶ�ȡʧ�ܺͶ���0)
 * @param[in]  dhiic     - Ŀ��IIC�豸���
 *             shiic     - ԴIIC�豸���
 *             iic_speed - IIC����
 * @return     ��
 * @history  - V1.0, 2017-09-12, xiaoguolin, first implementation.
*******************************************************************************/
void swiic_pan159_copy(swiic_pan159_t* dhiic, const swiic_pan159_t* shiic, uint32_t iic_speed)
{
    memcpy(dhiic,shiic,sizeof(swiic_pan159_t));
    if(iic_speed != SWIIC_PAN159_SPEED_SYS_48M_IIC_DEFAULT){
        __swiic_pan159_set_speed(dhiic,iic_speed);
    }
}

/*******************************************************************************
 * @brief      GPIO IIC��ȡ(�ú�����ȡ����һ���ķ���,�޷����ֶ�ȡʧ�ܺͶ���0)
 * @param[in]  hiic      - IIC�豸���
 *             iic_speed - IIC����
 *             dev       - IIC�豸��ַ
 *             reg       - �Ĵ�����ַ
 * @return     ��ȡ�����ֽ�
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint8_t swiic_pan159_read_byte(const swiic_pan159_t* hiic, uint8_t dev, uint8_t reg)
{
    register uint8_t ret = 0;
    __swiic_pan159_mutex_lock(hiic);                  /* ����MUTEX */
    __swiic_pan159_start(hiic);                       /* ����IIC */
    __swiic_pan159_write_byte(hiic,(dev << 1)|0);            /* д�豸��ַ(д) */
    if(__swiic_pan159_read_ack(hiic) == 0){           /* ��ȡACK */
        __swiic_pan159_write_byte(hiic,reg);          /* д�Ĵ�����ַ */
        if(__swiic_pan159_read_ack(hiic) == 0){       /* ��ȡACK */
            __swiic_pan159_start(hiic);               /* ����IIC */
            __swiic_pan159_write_byte(hiic,(dev << 1)|1);    /* д�豸��ַ(��) */
            if(__swiic_pan159_read_ack(hiic) == 0){   /* ��ȡACK */
                ret = __swiic_pan159_read_byte(hiic); /* ��ȡ�����ֽ� */
                __swiic_pan159_write_nack(hiic);      /* ��дNACK */
            }                                         /*  */
        }                                             /*  */
    }                                                 /*  */
    __swiic_pan159_stop(hiic);                        /* ֹͣIIC */
    __swiic_pan159_mutex_unlock(hiic);                /* �ͷ�MUTEX */
    return ret;
}

/*******************************************************************************
 * @brief      GPIO IIC��ȡ
 * @param[in]  hiic      - IIC�豸���
 *             iic_speed - IIC����
 *             dev       - IIC�豸��ַ
 *             reg       - �Ĵ�����ַ
 *             len       - Ҫ��ȡ�����ݳ���
 * @param[out] _buf      - ���ݻ�����
 * @return     ��ȡ�����ֽ���
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint32_t swiic_pan159_read_bytes(const swiic_pan159_t* hiic, uint8_t dev, uint8_t reg, uint8_t* buf, uint32_t len)
{
    register uint32_t i = 0;
    __swiic_pan159_mutex_lock(hiic);                         /* ����MUTEX */
    __swiic_pan159_start(hiic);                              /* ����IIC */
    __swiic_pan159_write_byte(hiic,(dev << 1)|0);                   /* д�豸��ַ(д) */
    if(__swiic_pan159_read_ack(hiic) == 0){                  /* ��ȡACK */
        __swiic_pan159_write_byte(hiic,reg);                 /* д�Ĵ�����ַ */
        if(__swiic_pan159_read_ack(hiic) == 0){              /* ��ȡACK */
            __swiic_pan159_start(hiic);                      /* ����IIC */
            __swiic_pan159_write_byte(hiic,(dev << 1)|1);           /* д�豸��ַ(��) */
            if(__swiic_pan159_read_ack(hiic) == 0){          /* ��ȡACK */
                for(len--; i < len; i++){                    /* len�м�����ȡ�����е����һ���ֽ� */
                    buf[i] = __swiic_pan159_read_byte(hiic); /* ��ȡ�ֽ� */
                    __swiic_pan159_write_ack(hiic);          /* ��дACK */
                }                                            /*  */
                buf[i++] = __swiic_pan159_read_byte(hiic);   /* ��ȡ���һ���ֽ� */
                __swiic_pan159_write_nack(hiic);             /* ��дNACK */
            }                                                /*  */
        }                                                    /*  */
    }                                                        /*  */
    __swiic_pan159_stop(hiic);                               /* ֹͣIIC */
    __swiic_pan159_mutex_unlock(hiic);                       /* �ͷ�MUTEX */
    return i;
}

/*******************************************************************************
 * @brief      GPIO IICд��
 * @param[in]  hiic      - IIC�豸���
 *             iic_speed - IIC����
 *             dev       - IIC�豸��ַ
 *             reg       - �Ĵ�����ַ
 *             ubyte     - ��д���ֽ�
 * @return     1 - �ɹ�
 *             0 - ʧ��
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint8_t swiic_pan159_write_byte(const swiic_pan159_t* hiic, uint8_t dev, uint8_t reg, uint8_t ubyte)
{
    __swiic_pan159_mutex_lock(hiic);                               /* ����MUTEX */
    __swiic_pan159_start(hiic);                                    /* ����IIC */
    __swiic_pan159_write_byte(hiic,(dev << 1)|0);                         /* д�豸��ַ(д) */
    if(__swiic_pan159_read_ack(hiic) == 0){                        /* ��ȡACK */
        __swiic_pan159_write_byte(hiic,reg);                       /* д�Ĵ�����ַ */
        if(__swiic_pan159_read_ack(hiic) == 0){                    /* ��ȡACK */
            __swiic_pan159_write_byte(hiic,ubyte);                 /* д�ֽ����� */
            ubyte = (__swiic_pan159_read_ack(hiic) == 0) ? 1 : 0;  /* ��ȡACK */
        }                                                          /*  */
    }                                                              /*  */
    __swiic_pan159_stop(hiic);                                     /* ֹͣIIC */
    __swiic_pan159_mutex_unlock(hiic);                             /* �ͷ�MUTEX */
    return ubyte;
}

/*******************************************************************************
 * @brief      GPIO IICд��
 * @param[in]  hiic      - IIC�豸���
 *             iic_speed - IIC����
 *             dev       - IIC�豸��ַ
 *             reg       - �Ĵ�����ַ
 *             buf       - ���ݻ�����
 *             len       - Ҫд������ݳ���
 * @return     д����ֽ���
 * @history  - V1.0, 2017-08-25, xiaoguolin, first implementation.
*******************************************************************************/
uint32_t swiic_pan159_write_bytes(const swiic_pan159_t* hiic, uint8_t dev, uint8_t reg, const uint8_t* buf, uint32_t len)
{
    register uint32_t i = 0;
    __swiic_pan159_mutex_lock(hiic);                      /* ����MUTEX */
    __swiic_pan159_start(hiic);                           /* ����IIC */
    __swiic_pan159_write_byte(hiic,(dev << 1)|0);                /* д�豸��ַ(д) */
    if(__swiic_pan159_read_ack(hiic) == 0){               /* ��ȡACK */
        __swiic_pan159_write_byte(hiic,reg);              /* д�Ĵ�����ַ */
        if(__swiic_pan159_read_ack(hiic) == 0){           /* ��ȡACK */
            for(; i < len; i++){                          /*  */
                __swiic_pan159_write_byte(hiic,buf[i]);   /* д�ֽ����� */
                if(__swiic_pan159_read_ack(hiic) != 0){   /* ��ȡACK */
                    break;                                /*  */
                }                                         /*  */
            }                                             /*  */
        }                                                 /*  */
    }                                                     /*  */
    __swiic_pan159_stop(hiic);                            /* ֹͣIIC */
    __swiic_pan159_mutex_unlock(hiic);                    /* �ͷ�MUTEX */
    return i;
}

swiic_pan159_t imu_iic;
swiic_pan159_t bar_iic;
//typedef void (*TCallback)(void);
void iic_pan159_init(void)
{
    swiic_pan159_init(&imu_iic,0x00030003,3,4,3,5);
    swiic_pan159_copy(&bar_iic,&imu_iic,0x00030003);
}

void iic_start_send_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk)
{
    if(swiic_pan159_write_bytes(&imu_iic, devAddr, regAddr, buf, len)){
        if(cbk){
            cbk();
        }
    }
    else{
        if(fail_cbk){
            fail_cbk();
        }
    }
}

void iic_start_read_bytes(uint16_t devAddr,uint8_t regAddr, uint8_t *buf, uint8_t len,TCallback cbk, TCallback fail_cbk)
{
    if(swiic_pan159_read_bytes(&imu_iic, devAddr, regAddr, buf, len)){
        if(cbk){
            cbk();
        }
    }
    else{
        if(fail_cbk){
            fail_cbk();
        }
    }
}
