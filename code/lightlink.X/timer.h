/* 
 * File:   timer.h
 * Author: Scott
 *
 * Created on July 13, 2019, 3:58 PM
 */

#ifndef TIMER_H
#define	TIMER_H
#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

// Function declarations
uint32_t timer_get_ticks(void);
void timer_wait_ms(uint32_t ms);
bool timer_has_time_elapsed(uint32_t time, uint32_t num_ms);


#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_H */

