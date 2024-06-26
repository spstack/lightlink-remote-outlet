
#include <xc.h>
#include "main.h"
#include "config.h"
#include "timer.h"


bool timer_has_time_elapsed(uint32_t time, uint32_t num_ms)
{
    uint32_t cur_tick = timer_get_ticks();

    // Check for overflow condition - overflow if sys ticks is less than the start point 'time'
    if (cur_tick >= time)
    {
        if ((cur_tick - time) >= num_ms)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else 
    {
        // Overflow has occurred
        if ((cur_tick + (0xFFFFFFFF - time)) >= num_ms)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

void timer_wait_ms(uint32_t ms)
{
    uint32_t startTime = timer_get_ticks();
    
    while (!timer_has_time_elapsed(startTime, ms));
}



uint32_t timer_get_ticks(void)
{
    uint32_t temp_ticks = 0;

    // Getting timer ticks has to be an atomic operation because an interrupt can change it
    enter_critical_section();
    temp_ticks = SYSTEM_TICKS;
    exit_critical_section();
    
    return temp_ticks;
}

