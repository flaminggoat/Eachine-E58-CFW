#include <Mini58Series.h>

#define PIN_LED P53
#define PIN_POWER_LATCH P32
#define PIN_UNKNOWN_A P04

int main(void)
{
    SYS->P5_MFP &= SYS_MFP_P53_Msk;
    SYS->P0_MFP &= SYS_MFP_P04_Msk;
    SYS->P3_MFP &= SYS_MFP_P32_Msk;

    PIN_POWER_LATCH = 1;
    PIN_LED = 0;

    GPIO_SetMode(P5, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(P3, BIT2, GPIO_MODE_OUTPUT);

    PIN_LED = 1;

    for(;;);
    return 0;
}