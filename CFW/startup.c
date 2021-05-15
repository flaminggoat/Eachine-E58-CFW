#include <Mini58Series.h>

extern unsigned long _data_flash;
extern unsigned long _data_begin;
extern unsigned long _data_end;
extern unsigned long _bss_begin;
extern unsigned long _bss_end;
extern unsigned long _stack_end;

int main(void);
void handler_default(void);
void SysTick_Handler(void);

void reset_handler(void)
{
    unsigned long *source;
    unsigned long *destination;
    // Copying data from Flash to RAM
    source = &_data_flash;
    for (destination = &_data_begin; destination < &_data_end;)
    {
        *(destination++) = *(source++);
    }
    // default zero to undefined variables
    for (destination = &_bss_begin; destination < &_bss_end;)
    {
        *(destination++) = 0;
    }
    // starting main program
    main();
}

__attribute__((section(".interrupt_vector"))) void (*const table_interrupt_vector[])(void) =
    {
        (void *)&_stack_end, // 0 - stack
        reset_handler,       // 1
        handler_default,     // 2
        handler_default,     // 3
        handler_default,     // 4
        handler_default,     // 5
        handler_default,     // 6
        0,                   // 7
        0,                   // 8
        0,                   // 9
        0,                   // 10
        handler_default,     // 11
        handler_default,     // 12
        0,                   // 13
        handler_default,     // 14
        SysTick_Handler,     // 15
        // peripherals
        handler_default, // 0
        handler_default, // 1
        handler_default, // 2
        handler_default, // 3
        handler_default, // 4
        handler_default, // 5
        handler_default, // 6
        handler_default, // 7
        handler_default, // 8
        handler_default, // 9
        handler_default, // 10
        handler_default, // 11
        handler_default, // 12
        handler_default, // 13
        handler_default, // 14
        handler_default, // 15
        handler_default, // 16
        handler_default, // 17
        handler_default, // 18
        handler_default, // 19
        handler_default, // 20
        handler_default, // 21
        handler_default, // 22
        handler_default, // 23
        handler_default, // 24
        handler_default, // 25
        handler_default, // 26
        handler_default, // 27
        handler_default, // 28
        handler_default  // 29
};

/**
 * handler_default:
 * Alternative Hard Fault handler to help debug the reason for a fault.
 * To use, edit the vector table to reference this function in the HardFault vector
 * This code is suitable for Cortex-M3 and Cortex-M0 cores
 */

// Use the 'naked' attribute so that C stacking is not used.
__attribute__((naked)) void handler_default(void)
{
    /*
         * Get the appropriate stack pointer, depending on our mode,
         * and use it as the parameter to the C handler. This function
         * will never return
         */

    __asm(".syntax unified\n"
          "MOVS   R0, #4  \n"
          "MOV    R1, LR  \n"
          "TST    R0, R1  \n"
          "BEQ    _MSP    \n"
          "MRS    R0, PSP \n"
          "B      HardFault_HandlerC      \n"
          "_MSP:  \n"
          "MRS    R0, MSP \n"
          "B      HardFault_HandlerC      \n"
          ".syntax divided\n");
}

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void HardFault_HandlerC(unsigned long *hardfault_args)
{
    volatile unsigned long stacked_r0 __attribute__((unused));
    volatile unsigned long stacked_r1 __attribute__((unused));
    volatile unsigned long stacked_r2 __attribute__((unused));
    volatile unsigned long stacked_r3 __attribute__((unused));
    volatile unsigned long stacked_r12 __attribute__((unused));
    volatile unsigned long stacked_lr __attribute__((unused));
    volatile unsigned long stacked_pc __attribute__((unused));
    volatile unsigned long stacked_psr __attribute__((unused));
    volatile unsigned long _CFSR __attribute__((unused));
    volatile unsigned long _HFSR __attribute__((unused));
    volatile unsigned long _DFSR __attribute__((unused));
    volatile unsigned long _AFSR __attribute__((unused));
    volatile unsigned long _BFAR __attribute__((unused));
    volatile unsigned long _MMAR __attribute__((unused));

    stacked_r0 = ((unsigned long)hardfault_args[0]);
    stacked_r1 = ((unsigned long)hardfault_args[1]);
    stacked_r2 = ((unsigned long)hardfault_args[2]);
    stacked_r3 = ((unsigned long)hardfault_args[3]);
    stacked_r12 = ((unsigned long)hardfault_args[4]);
    stacked_lr = ((unsigned long)hardfault_args[5]);
    stacked_pc = ((unsigned long)hardfault_args[6]);
    stacked_psr = ((unsigned long)hardfault_args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28)));

    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30)));

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34)));
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38)));

    __asm("BKPT #0\n"); // Break into the debugger

    for (;;)
        ;
}

__attribute__((weak)) void SysTick_Handler(void)
{
}

