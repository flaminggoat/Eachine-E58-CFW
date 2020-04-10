#include <Mini58Series.h>

extern unsigned long _data_flash;
extern unsigned long _data_begin;
extern unsigned long _data_end;
extern unsigned long _bss_begin;
extern unsigned long _bss_end;
extern unsigned long _stack_end;

int main(void);
void handler_default(void);

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
        handler_default,     // 15
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

void handler_default(void)
{
    while (1)
    {
        //loop
    }
}