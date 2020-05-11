#include <Mini58Series.h>
#include <stdbool.h>

#include <lib_driver_xn297l.h>

#define PIN_LED P53
#define PIN_POWER_LATCH P32
#define PIN_UNKNOWN_A P04

typedef struct gyro_obj
{
    int16_t ax, ay, az, gx, gy, gz;
    int16_t temp;
} gyro_obj;

static void clock_init(void)
{
    static uint32_t count = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    // Enable HIRC and wait for it to stablise
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0)
    {
        ++count;
    }

    // Configure the pll
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC, FREQ_50MHZ);
    // Set the HCLK divider to 1
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | CLK_CLKDIV_HCLK(1);
    // Select PLL as HCLK source
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    // Enable module clocks
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(PWMCH01_MODULE);
    CLK_EnableModuleClock(PWMCH23_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    // CLK_EnableModuleClock(UART0_MODULE);
    // CLK_EnableModuleClock(UART1_MODULE);
    // CLK_EnableModuleClock(ADC_MODULE);
    // CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_HCLK,CLK_CLKDIV_ADC(6));
    // CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADCSEL_HCLK, CLK_CLKDIV_ADC(180)); /** 276.480kHzÊḟÖÓ */
    // CLK_EnableModuleClock(PWMCH01_MODULE);
    // CLK_EnableModuleClock(PWMCH23_MODULE);

    //SystemCoreClockUpdate();
    //volatile uint32_t xx = CLK_GetPLLClockFreq();
    //volatile uint32_t yy = CLK_GetHCLKFreq();
    /* Lock protected registers */
    SYS_LockReg();
}

static void i2c_init(void)
{
    SYS->P3_MFP &= ~SYS_MFP_P34_Msk;
    SYS->P3_MFP |= SYS_MFP_P34_I2C0_SDA;
    SYS->P3_MFP &= ~SYS_MFP_P35_Msk;
    SYS->P3_MFP |= SYS_MFP_P35_I2C0_SCL;

    I2C_Open(I2C0, 400000);
    // for(uint8_t i=0x08; i<=0x77; ++i)
    // {
    //     I2C_START(I2C0);
    //     I2C_WAIT_READY(I2C0);
    //     I2C_SET_DATA(I2C0, (i << 1));
    //     I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    //     I2C_WAIT_READY(I2C0);

    //     if(I2C_GET_STATUS(I2C0) == 0x18)
    //     {
    //         // Got ack
    //         I2C_WAIT_READY(I2C0);
    //     }

    //     // send stop
    //     I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
    //     //I2C_WAIT_READY(I2C0);
    // }
}

bool i2c_gyro_init(void)
{
    // -------------- enable gyro
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, (0x69 << 1));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    if (I2C_GET_STATUS(I2C0) == 0x18)
    {
        // send reg address
        I2C_SET_DATA(I2C0, 0x6B); // pwr register
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        I2C_WAIT_READY(I2C0);
        if (I2C_GET_STATUS(I2C) == 0x28)
        {
            // send data
            I2C_SET_DATA(I2C0, 0x00); // 0 to wakeup
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
            I2C_WAIT_READY(I2C0);
        }
        else
        {
            // send stop
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            return false;
        }
    }
    else
    {
        // send stop
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
        return false;
    }

    // send stop
    I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);

    return true;
}

bool i2c_gyro_read(gyro_obj *gyro)
{
    // -------------- select reg for accel
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, (0x69 << 1));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);

    if (I2C_GET_STATUS(I2C0) == 0x18)
    {
        // send reg address
        I2C_SET_DATA(I2C0, 0x3B); // accel register
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        I2C_WAIT_READY(I2C0);
        if (I2C_GET_STATUS(I2C) != 0x28)
        {
            // send stop
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            return false;
        }
    }
    else
    {
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
        return false;
    }

    // send stop
    I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);

    // ------- read
    uint8_t buffer[14];
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    I2C_SET_DATA(I2C0, ((0x69 << 1) | 1));
    I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    I2C_WAIT_READY(I2C0);
    if (I2C_GET_STATUS(I2C0) == 0x40)
    {
        int i;
        for (i = 0; i < 14 - 1; i++)
        {
            I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
            I2C_WAIT_READY(I2C);

            /* Read data */
            buffer[i] = I2C_GET_DATA(I2C);
        }

        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);
        buffer[i] = I2C_GET_DATA(I2C);

        /* Send stop */
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
    }
    else
    {
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
        return false;
    }

    gyro->ax = (buffer[0] << 8) | buffer[1];
    gyro->ay = (buffer[2] << 8) | buffer[3];
    gyro->az = (buffer[4] << 8) | buffer[5];

    gyro->temp = (buffer[6] << 8) | buffer[7];

    gyro->gx = (buffer[8] << 8) | buffer[9];
    gyro->gy = (buffer[10] << 8) | buffer[11];
    gyro->gz = (buffer[12] << 8) | buffer[13];

    return true;
}

void pwm_init(void)
{
    SYS->P2_MFP = (SYS->P2_MFP & ~SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM0_CH0;
    SYS->P2_MFP = (SYS->P2_MFP & ~SYS_MFP_P23_Msk) | SYS_MFP_P23_PWM0_CH1;
    SYS->P2_MFP = (SYS->P2_MFP & ~SYS_MFP_P24_Msk) | SYS_MFP_P24_PWM0_CH2;
    SYS->P2_MFP = (SYS->P2_MFP & ~SYS_MFP_P25_Msk) | SYS_MFP_P25_PWM0_CH3;

    PWM->CLKPSC = (1 << PWM_CLKPSC_CLKPSC01_Pos) | (1 << PWM_CLKPSC_CLKPSC23_Pos);
    PWM->CLKDIV = (1 << PWM_CLKDIV_CLKDIV0_Pos) | (1 << PWM_CLKDIV_CLKDIV1_Pos) | (1 << PWM_CLKDIV_CLKDIV2_Pos) | (1 << PWM_CLKDIV_CLKDIV3_Pos);
    PWM->CTL = PWM_CTL_CNTMODE0_Msk | PWM_CTL_CNTMODE1_Msk | PWM_CTL_CNTMODE2_Msk | PWM_CTL_CNTMODE3_Msk;

    PWM->PERIOD0 = 1000;
    PWM->PERIOD1 = 1000;
    PWM->PERIOD2 = 1000;
    PWM->PERIOD3 = 1000;

    PWM->CMPDAT0 = 2 * 10;
    PWM->CMPDAT1 = 2 * 10;
    PWM->CMPDAT2 = 2 * 10;
    PWM->CMPDAT3 = 2 * 10;

    // PWM_ConfigOutputChannel(PWM, 0, 10000, 2);
    // PWM_ConfigOutputChannel(PWM, 1, 10000, 2);
    // PWM_ConfigOutputChannel(PWM, 2, 10000, 2);
    // PWM_ConfigOutputChannel(PWM, 3, 10000, 2);

    PWM_Start(PWM, 0b1111);

    PWM_EnableOutput(PWM, 0b1111);
}

static void rf_reciever_init(void)
{
    static uint8_t tx_addr[] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
    static uint8_t rx_addr[] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    //static uint8_t tx_addr[] = {0x37, 0x5d, 0x0b, 0xdf, 0x02};
    // static uint8_t tx_addr[] = {0xe7,0xe7,0xe7,0xe7,0xe7};
    // static uint8_t rx_addr[] = {0xe7,0xe7,0xe7,0xe7,0xe7};

    // static uint8_t rf_cal_data[]   = {0xf6, 0x37, 0x5d};
    // static uint8_t rf_cal_data_2[] = {0x40, 0x00, 0x0d, 0x1e, 0x66, 0x66};
    // static uint8_t bb_cal[]        = {0xd5, 0x21, 0xef, 0x2c, 0x5a};
    // static uint8_t dem_cal[]       = {0xff};
    // static uint8_t dem_cal2[]      = {0x0B,0xDF,0x02};

    // from eachine
    static uint8_t rf_cal_data[]   = {0xF6, 0x37, 0x5D};
    static uint8_t rf_cal_data_2[] = {0xd5, 0x21, 0xEF, 0x2C, 0x5A, 0x40};
    static uint8_t bb_cal[]        = {0x0a, 0x6c, 0x67, 0x9C, 0x46}; // diff
    static uint8_t dem_cal[]       = {0x01};
    static uint8_t dem_cal2[]      = {0x0B, 0xDF, 0x02};
	
    // from library
    // static uint8_t rf_cal_data[]   = {0xF6, 0x3F, 0x5D};
    // static uint8_t rf_cal_data_2[] = {0x45, 0x21, 0xEF, 0x2C, 0x5A, 0x40};
    // static uint8_t bb_cal[]        = {0x12, 0xED, 0x67, 0x9C, 0x46};
    // static uint8_t dem_cal[]       = {0x01};
    // static uint8_t dem_cal2[]      = {0x0B, 0xDF, 0x02};

    // delay_ms(200);
    // xn297l_write_reg(0x53, 0x5a);
    // delay_ms(10);
    // xn297l_write_reg(0x53, 0xa5);
    // delay_ms(10);

    // xn297l_read_buf(TX_ADDR, tx_addr, 5);
    // xn297l_read_buf(RX_ADDR_P0, rx_addr, 5);
    // xn297l_read_buf(BB_CAL, bb_cal, 5);
    // xn297l_read_buf(RF_CAL2, rf_cal_data_2, 6);
    // xn297l_read_buf(DEM_CAL, dem_cal, 1);
    // xn297l_read_buf(RF_CAL, rf_cal_data, 3);
    // xn297l_read_buf(DEM_CAL2, dem_cal2, 3);

    while (true)
    {
        delay_ms(200);
        xn297l_write_reg(0x53, 0x5a);
        delay_ms(10);
        xn297l_write_reg(0x53, 0xa5);
        delay_ms(10);
        xn297l_clear_fifo();
        xn297l_clear_status();
        xn297l_write_reg(0x3d, 0x20);
        xn297l_write_reg(0xfc, 0);
        xn297l_write_reg(0x22, 1);
        xn297l_write_reg(0x23, 3);
        xn297l_write_reg(0x25, 3);
        xn297l_write_reg(W_REGISTER + RX_PW_P0, 10);
        xn297l_write_buf(W_REGISTER + TX_ADDR, tx_addr, 5);
        xn297l_write_buf(W_REGISTER + RX_ADDR_P0, rx_addr, 5);
        xn297l_write_buf(W_REGISTER + BB_CAL, bb_cal, 5);
        xn297l_write_buf(W_REGISTER + RF_CAL2, rf_cal_data_2, 6);
        xn297l_write_buf(W_REGISTER + DEM_CAL, dem_cal, 1);
        xn297l_write_buf(W_REGISTER + RF_CAL, rf_cal_data, 3);
        xn297l_write_buf(W_REGISTER + DEM_CAL2, dem_cal2, 3);
        xn297l_write_reg(W_REGISTER + DYNPD, 0);
        xn297l_write_reg(W_REGISTER + RF_SETUP, 0xb);
        xn297l_write_reg(W_REGISTER + SETUP_RETR, 0);
        xn297l_write_reg(W_REGISTER + EN_AA, 0);
        uint8_t channel_read = xn297l_read_reg(RF_CH);
        if (channel_read == 3)
            break; // Check channel has been set correctly
        xn297l_write_reg(0x20, 0x8f);
        delay_ms(2);
        xn297l_write_reg(0xfd, 0);
        delay_ms(2);
    }
}

int main(void)
{
    SYS->P5_MFP &= SYS_MFP_P53_Msk;
    SYS->P0_MFP &= SYS_MFP_P04_Msk;
    SYS->P3_MFP &= SYS_MFP_P32_Msk;

    PIN_POWER_LATCH = 1;
    PIN_LED = 0;

    GPIO_SetMode(P5, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(P3, BIT2, GPIO_MODE_OUTPUT);

    clock_init();
    pwm_init();
    i2c_init();

    i2c_gyro_init();

    uint8_t data[32] = {0};

    rfspi_init();
    rf_reciever_init();

    // rf_init();
    // 0a 6c 67 9c 46
    // uint8_t addr[] = {0xe7,0xe7,0xe7,0xe7,0xe7};
    // rfspi_init();
    // while(1)
    // {
    //     delay_ms(200);
    //     xn297l_init(addr,5,3,10,XN297L_RF_DATA_RATE_1M);
    //     uint8_t x = xn297l_read_reg(RF_CH);
    //     if (x == 3)
    //     {
    //         break;
    //     }
    //     xn297l_write_reg(0x20,0x8f);
    //     delay_ms(2);
    //     xn297l_write_reg(0xfd,0);
    //     delay_ms(2);
    // }

    xn297l_rx_mode();

    // xn297l_read_buf(0, data, 32);

    // uint8_t status = xn297l_get_status();
    // xn297l_read_reg(RF_CH);
    // xn297l_read_reg(RF_SETUP);

    int chan = 0;
    while (xn297l_rx_data(data, 32) == 0)
    {
        // xn297l_set_channel(chan);
        // xn297l_rx_mode();
        // chan = (chan + 1) & 0x7f;
        delay_ms(100);
    }

    static gyro_obj gyro;

    for (;;)
    {
        i2c_gyro_read(&gyro);
        if (gyro.ax > 0)
        {
            PIN_LED = 1;
        }
        else
        {
            PIN_LED = 0;
        }
    }

    return 0;
}