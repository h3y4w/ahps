
#include "stm32f4xx.h"

static void _delay_tick(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   _delay_tick\n"
          "    bx      lr");
}





// micro second 단위로 delay
void delay_us(unsigned long us)
{
    _delay_tick(us * (168000000 / 3 / 1000000));
}





// mili second 단위로 delay
void delay_ms(unsigned long ms)
{
    delay_us(ms * 1000);
}


