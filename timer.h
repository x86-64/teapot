#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include	<avr/io.h>

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)
#define SAFE_INTERRUPT_DELAY 300

/*
clock stuff
*/
extern volatile uint8_t	clock_flag_10ms;
extern volatile uint8_t	clock_flag_250ms;
extern volatile uint8_t	clock_flag_1s;

// If the specific bit is set, execute the following block exactly once
// and then clear the flag.
#define	ifclock(F)	for (;F;F=0 )

typedef void (*timer_callback)(uint8_t id, void *userdata);

typedef struct timer_t {
	uint8_t                enabled;
	uint32_t               delay;
	timer_callback         callback;
	void                  *userdata;
	
	uint32_t               time_left;
} timer_t;

/*
timer stuff
*/
void timers_init(void) __attribute__ ((cold));
void timers_stop(void);
void timer_setup(uint8_t id, timer_callback callback, void *userdata);
void timer_enable(uint8_t id);
void timer_disable(uint8_t id);
void timer_charge(uint8_t id, uint32_t delay);



#endif	/* _TIMER_H */
