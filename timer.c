#include	"timer.h"

/** \file
	\brief Timer management - step pulse clock and system clock

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#include	<avr/interrupt.h>
#include	"memory_barrier.h"

#include	"arduino.h"
#include	"common.h"

#define NUM_TIMERS 3
timer_t timers[NUM_TIMERS];
volatile uint8_t        timers_used = 0; ///< timer_new counter

/// how often we overflow and update our clock; with F_CPU=16MHz, max is < 4.096ms (TICK_TIME = 65535)
#define		TICK_TIME			2 MS
/// convert back to ms from cpu ticks so our system clock runs properly if you change TICK_TIME
#define		TICK_TIME_MS	(TICK_TIME / (F_CPU / 1000))

/// time until next step, as output compare register is too small for long step times
uint32_t	next_step_time;
uint32_t	delay_time;

#ifdef ACCELERATION_TEMPORAL
/// unwanted extra delays, ideally always zero
uint32_t	step_extra_time = 0;
#endif /* ACCELERATION_TEMPORAL */

void timer_hardware_set(uint32_t delay);

void timers_gcode(void *next_target){
	if(! PARAMETER_SEEN(L_M))
		return;
	
	switch(PARAMETER_asint(L_M)){
		case 112:
			//? --- M112: Emergency Stop ---
			//?
			//? Example: M112
			//?
			//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
			//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.
			//?
			
			timers_stop();
			break;
	}
}

/// initialise timer and enable system clock interrupt.
/// step interrupt is enabled later when we start using it
void timers_init()
{
	// no outputs
	TCCR1A = 0;
	// Normal Mode
	TCCR1B = MASK(CS10);
	
	core_register(EVENT_GCODE_PROCESS, &timers_gcode);
}

void timers_update(uint32_t time_passed){
	uint8_t  i;
	
	// update time_left for all
	for(i=0; i<NUM_TIMERS; i++){
		if(timers[i].enabled == 0)
			continue;
		
		if(time_passed > timers[i].time_left){ // just in case, should not happen because time_passed defined by user's delay, not by real time passed
			timers[i].time_left = 0;
		}else{
			timers[i].time_left -= time_passed;                 // update time_left
		}
	}
}

void timers_recalc(void){
	uint8_t  i;
	uint32_t min = 0xFFFFFFFF;
	
redo:;
	for(i=0; i<NUM_TIMERS; i++){
		if(timers[i].enabled == 0)
			continue;
		
		if(timers[i].time_left < SAFE_INTERRUPT_DELAY){     // less than we can wait in hardware timer, fire it now
			timer_disable(i);
			timers[i].callback(i, timers[i].userdata);
			goto redo;                                  // since user could re-enable timer, we have to redo our calculations
		}
		
		if(min > timers[i].time_left){  // if this is first iteration, or this timer have less time to fire
			min = timers[i].time_left;
		}
	}
	if(min != 0xFFFFFFFF)
		timer_hardware_set(min);
}

/// comparator A is the step timer. It has higher priority then B.
ISR(TIMER1_COMPA_vect) {
	// save status register
	uint8_t sreg_save = SREG;

	// Check if this is a real step, or just a next_step_time "overflow"
	if (next_step_time < 65536) {
		// step!
		// disable this interrupt. if we set a new timeout, it will be re-enabled when appropriate
		TIMSK1 &= ~MASK(OCIE1A);
		
		timers_update(delay_time);
		timers_recalc();
		return;
	}

	next_step_time -= 65536;

	// similar algorithm as described in setTimer below.
	if (next_step_time < 65536) {
		OCR1A = (OCR1A + next_step_time) & 0xFFFF;
	} else if(next_step_time < 75536){
		OCR1A = (OCR1A - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	// leave OCR1A as it was

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

/*! Specify how long until the step timer should fire.
	\param delay in CPU ticks

	This enables the step interrupt, but also disables interrupts globally.
	So, if you use it from inside the step interrupt, make sure to do so
	as late as possible. If you use it from outside the step interrupt,
	do a sei() after it to make the interrupt actually fire.
*/
void timer_hardware_set(uint32_t delay)
{
	uint16_t step_start = 0;
	#ifdef ACCELERATION_TEMPORAL
	uint16_t current_time;
	uint32_t earliest_time, actual_time;
	#endif /* ACCELERATION_TEMPORAL */

	// re-enable clock interrupt in case we're recovering from emergency stop
	//TIMSK1 |= MASK(OCIE1B);

	// An interrupt would make all our timing calculations invalid,
	// so stop that here.
	cli();

	// Assume all steps belong to one move. Within one move the delay is
	// from one step to the next one, which should be more or less the same
	// as from one step interrupt to the next one. The last step interrupt happend
	// at OCR1A, so start delay from there.
	step_start = OCR1A;
	if (next_step_time == 0) {
		// new move, take current time as start value
		step_start = TCNT1;
	}
	next_step_time = delay;
	delay_time     = delay;

	#ifdef ACCELERATION_TEMPORAL
	// 300 = safe number of cpu cycles until the interrupt actually happens
	current_time = TCNT1;
	earliest_time = (uint32_t)current_time + 300;
	if (current_time < step_start) // timer counter did overflow recently
		earliest_time += 0x00010000;
	actual_time = (uint32_t)step_start + next_step_time;

	// Setting the interrupt earlier than it can happen obviously doesn't
	// make sense. To keep the "belongs to one move" idea, add an extra,
	// remember this extra and compensate the extra if a longer delay comes in.
	if (earliest_time > actual_time) {
		step_extra_time += (earliest_time - actual_time);
		next_step_time = earliest_time - (uint32_t)step_start;
	}
	else if (step_extra_time) {
		if (step_extra_time < actual_time - earliest_time) {
			next_step_time -= step_extra_time;
			step_extra_time = 0;
		}
		else {
			step_extra_time -= (actual_time - earliest_time);
			next_step_time -= (actual_time - earliest_time);
		}
	}
	#endif /* ACCELERATION_TEMPORAL */

	// Now we know how long we actually want to delay, so set the timer.
	if (next_step_time < 65536) {
		// set the comparator directly to the next real step
		OCR1A = (next_step_time + step_start) & 0xFFFF;
	}
	else if (next_step_time < 75536) {
		// Next comparator interrupt would have to trigger another
		// interrupt within a short time (possibly within 1 cycle).
		// Avoid the impossible by firing the interrupt earlier.
		OCR1A = (step_start - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	else {
		OCR1A = step_start;
	}

	// Enable this interrupt, but only do it after disabling
	// global interrupts (see above). This will cause push any possible
	// timer1a interrupt to the far side of the return, protecting the 
	// stack from recursively clobbering memory.
	TIMSK1 |= MASK(OCIE1A);
}

/// stop timers - emergency stop
void timers_stop() {
	// disable all interrupts
	TIMSK1 = 0;
}

uint8_t timer_new(void){
	uint8_t id = timers_used++;
	
	if(id >= NUM_TIMERS)
		return -1;
	
	return id;
}

void timer_setup(uint8_t id, timer_callback callback, void *userdata){
	timers[id].callback  = callback;
	timers[id].userdata  = userdata;
}
void timer_enable(uint8_t id){
	timers[id].enabled   = 1;
	timers[id].time_left = timers[id].delay;
}
void timer_disable(uint8_t id){
	timers[id].enabled   = 0;
}
void timer_charge(uint8_t id, uint32_t delay){
	timers[id].delay     = delay;
	timer_enable(id);
	
	timers_recalc();
}

/* 
 * usage example
 *

// try to not used delays inside timer code

uint8_t timer1_called = 0;
uint8_t timer2_called = 0;
uint8_t timer3_called = 0;

void mytimer1_callback(uint8_t id, void *userdata){
	timer1_called = 1;
	timer_enable(id);
}
void mytimer2_callback(uint8_t id, void *userdata){
	timer2_called = 1;
	timer_enable(id);
}
void mytimer3_callback(uint8_t id, void *userdata){
	timer3_called = 1;
	timer_enable(id);
}

int main(void){
	DDRB = 0xff;// (1<<DDB0)|(1<<DDB1);

	timers_init();
	
	timer_setup(0, &mytimer1_callback, 0);
	timer_setup(1, &mytimer2_callback, 0);
	timer_setup(2, &mytimer3_callback, 0);
	
	timer_charge(0, 10 MS);
	timer_charge(1, 25 MS);
	timer_charge(2, 14 MS);
	sei();
	while(1){
		if(timer1_called == 1){
			PORTB |= (1<<DDB1);
			_delay_us(500);
			PORTB &= ~(1<<DDB1);
			timer1_called = 0;
		}
		if(timer2_called == 1){
			PORTB |= (1<<DDB2);
			_delay_us(500);
			PORTB &= ~(1<<DDB2);
			timer2_called = 0;
		}
		if(timer3_called == 1){
			PORTB |= (1<<DDB2);
			_delay_us(500);
			PORTB &= ~(1<<DDB2);
			timer3_called = 0;
		}
		delay(10);
	}
}
*/
