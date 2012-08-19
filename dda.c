#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<avr/interrupt.h>

#include	"dda_maths.h"
#include	"timer.h"
#include	"serial.h"
#include	"gcode_parse.h"
#include	"debug.h"
#include	"pinio.h"
#include	"common.h"

#include	"delay.h"
#include	"memory_barrier.h"

//#include "graycode.c"

/*! Distribute a new dda->startpoint to DDA's internal structures without any movement.

	This is needed for example after homing or a G92. The new location must be in dda->startpoint already.
*/

/*! CREATE a dda given dda->current_position and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref dda->startpoint the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
uint8_t dda_create(DDA *dda, TARGET *target) {
	uint32_t	target_st, start_st, delta_um;
	uint32_t	distance, c_limit;

	// initialise DDA to a known state
	dda->allflags = 0;
	#ifdef ACCELERATION_RAMPING
		dda->move->ramping_n = 1;
		dda->move->ramping_c = ((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_M_X * ACCELERATION / 1000.)))) << 8;
	#endif

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("\n{DDA_CREATE: ["));

	// we end at the passed target
	dda->endpoint = *target;

/* FIXME	if (target->e_relative) {
		delta_um = labs(target->E);
		dda->delta_steps = labs(um_to_steps_e(target->E));
		dda->direction = (target->E >= 0)?1:0;
	}
	else {*/
		delta_um         = (uint32_t)labs(target->X - dda->startpoint.X);
		target_st        = um_to_steps_x(target->X);
		start_st         = um_to_steps_x(dda->startpoint.X);
		dda->delta_steps = labs(target_st - start_st);
		dda->direction   = (target->X >= dda->startpoint.X) ? 1 : 0;
	//}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("ts:%lu"), dda->delta_steps);

	distance = delta_um;

	if(distance == 0)
		return 1; // ERR, nothing to move

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR(",ds:%lu"), distance);

	#ifdef	ACCELERATION_TEMPORAL
		// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
		uint32_t move_duration = distance * ((60 * F_CPU) / (target->F * 1000UL));
	#else
		// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
		// mm (distance) * 60000000 us/min / step (delta) = mm.us per step.min
		//   note: um (distance) * 60000 == mm * 60000000
		// so in the interrupt we must simply calculate
		// mm.us per step.min / mm per min (F) = us per step

		// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
		// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
		// 2^32/6000 is about 715mm which should be plenty

		// changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
		// timer.c setTimer() routine altered for same reason

		// changed distance * 6000 .. * F_CPU / 100000 to
		//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
		uint32_t move_duration = ((distance * 2400) / dda->delta_steps) * (F_CPU / 40000);
	#endif

	// similarly, find out how fast we can run our axes.
	c_limit = ((delta_um * 2400L) / dda->delta_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8;

	#ifdef ACCELERATION_REPRAP
	// c is initial step time in IOclk ticks
	dda->c            = (move_duration / dda->startpoint.F) << 8;
	dda->c            = MAX(dda->c, c_limit);
	dda->reprap_end_c = (move_duration / target->F) << 8;
	dda->reprap_end_c = MAX(dda->reprap_end_c, c_limit);

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c >> 8);

	if (dda->c != dda->reprap_end_c) {
		uint32_t stF = dda->startpoint.F / 4;
		uint32_t enF = target->F / 4;
		// now some constant acceleration stuff, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		uint32_t ssq = (stF * stF);
		uint32_t esq = (enF * enF);
		int32_t dsq = (int32_t) (esq - ssq) / 4;

		uint8_t msb_ssq = msbloc(ssq);
		uint8_t msb_tot = msbloc(dda->delta_steps);

		// the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
		// at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
		// but if ssq-esq is small, ssq/dsq is only a few bits
		// we'll have to do it a few different ways depending on the msb locations of each
		if ((msb_tot + msb_ssq) <= 30) {
			// we have room to do all the multiplies first
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
				serial_writechar('A');
			dda->reprap_n = ((int32_t) (dda->delta_steps * ssq) / dsq) + 1;
		}
		else if (msb_tot >= msb_ssq) {
			// total steps has more precision
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
				serial_writechar('B');
			dda->reprap_n = (((int32_t) dda->delta_steps / dsq) * (int32_t) ssq) + 1;
		}
		else {
			// otherwise
			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
				serial_writechar('C');
			dda->reprap_n = (((int32_t) ssq / dsq) * (int32_t) dda->delta_steps) + 1;
		}

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->reprap_end_c >> 8, dda->reprap_n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

		dda->accel = 1;
	}
	else
		dda->accel = 0;
	#elif defined ACCELERATION_RAMPING
		// yes, this assumes always the x axis as the critical one regarding acceleration. If we want to implement per-axis acceleration, things get tricky ...
		dda->ramping_c_min = (move_duration / target->F) << 8;
		dda->ramping_c_min = MAX(dda->ramping_c_min, c_limit);
	// This section is plain wrong, like in it's only half of what we need. This factor 960000 is dependant on STEPS_PER_MM.
	// overflows at target->F > 65535; factor 16. found by try-and-error; will overshoot target speed a bit
		dda->rampup_steps = target->F * target->F / (uint32_t)(STEPS_PER_M_X * ACCELERATION / 960000.);
	//sersendf_P(PSTR("rampup calc %lu\n"), dda->rampup_steps);
		dda->rampup_steps = 100000; // replace mis-calculation by a safe value
	// End of wrong section.
		if (dda->rampup_steps > dda->delta_steps / 2)
			dda->rampup_steps = dda->delta_steps / 2;
		dda->rampdown_steps = dda->delta_steps - dda->rampup_steps;
	#elif defined ACCELERATION_TEMPORAL
		dda->temporal_step_interval = 0xFFFFFFFF;
		
		if (dda->delta_steps)
			dda->temporal_step_interval = move_duration / dda->delta_steps;
		
		dda->c = dda->temporal_step_interval;
		dda->c <<= 8;
	#else
		dda->c = (move_duration / target->F) << 8;
		dda->c = MAX(dda->c, c_limit);
	#endif

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));

	// next dda starts where we finish
	dda->startpoint = *target;

	return 0;
}

/*! Start a prepared DDA
	\param *dda pointer to entry in dda_queue to start

	This function actually begins the move described by the passed DDA entry.

	We set direction and enable outputs, and set the timer for the first step from the precalculated value.

	We also mark this DDA as running, so other parts of the firmware know that something is happening

	Called both inside and outside of interrupts.
*/
void dda_start(DDA *dda) {
	// initialise state variable
	dda->move->counter = -(dda->delta_steps >> 1);
	dda->move->steps   = dda->delta_steps;
	
	#ifdef ACCELERATION_RAMPING
		dda->move->ramping_step_no = 0;
	#endif
	#ifdef ACCELERATION_TEMPORAL
		dda->move->temporal_time = 0UL;
	#endif
	
	// ensure this dda starts
	dda->live = 1;
	
	// set timeout for first step
	#ifdef ACCELERATION_RAMPING
	if (dda->ramping_c_min > dda->move->ramping_c) // can be true when look-ahead removed all deceleration steps
		timer_charge(0, dda->ramping_c_min >> 8);
	else
		timer_charge(0, dda->move->ramping_c >> 8);
	#else
		timer_charge(0, dda->c >> 8);
	#endif
	// else just a speed change, keep dda->live = 0

	dda->current_position.F = dda->endpoint.F;
}

/*! STEP
	\param *dda the current move

	This is called from our timer interrupt every time a step needs to occur. Keep it as simple as possible!
	We first work out which axes need to step, and generate step pulses for them
	Then we re-enable global interrupts so serial data reception and other important things can occur while we do some math.
	Next, we work out how long until our next step using the selected acceleration algorithm and set the timer.
	Then we decide if this was the last step for this move, and if so mark this dda as dead so next timer interrupt we can start a new one.
	Finally we de-assert any asserted step pins.
*/
void dda_step(DDA *dda) {

#if ! defined ACCELERATION_TEMPORAL
	if ((dda->move->steps)) {
		dda->move->counter -= dda->delta_steps;
		if (dda->move->counter < 0) {
			x_step();
			//x_direction(dda->direction);
			dda->move->steps--;
			dda->move->counter += dda->delta_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	x_step();
	//x_direction(dda->direction);
	dda->move->steps--;
	
	dda->move->temporal_time += dda->temporal_step_interval;
	dda->move->all_time       = dda->move->temporal_time;
#endif


	#if STEP_INTERRUPT_INTERRUPTIBLE
		// Since we have sent steps to all the motors that will be stepping
		// and the rest of this function isn't so time critical, this interrupt
		// can now be interruptible by other interrupts.
		// The step interrupt is disabled before entering dda_step() to ensure
		// that we don't step again while computing the below.
		sei();
	#endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		if (dda->accel) {
			if ((dda->c > dda->reprap_end_c) && (dda->reprap_n > 0)) {
				uint32_t new_c = dda->c - (dda->c * 2) / dda->reprap_n;
				if (new_c <= dda->c && new_c > dda->reprap_end_c) {
					dda->c = new_c;
					dda->reprap_n += 4;
				}
				else
					dda->c = dda->reprap_end_c;
			}
			else if ((dda->c < dda->reprap_end_c) && (dda->reprap_n < 0)) {
				uint32_t new_c = dda->c + ((dda->c * 2) / -dda->reprap_n);
				if (new_c >= dda->c && new_c < dda->reprap_end_c) {
					dda->c = new_c;
					dda->reprap_n += 4;
				}
				else
					dda->c = dda->reprap_end_c;
			}
			else if (dda->c != dda->reprap_end_c) {
				dda->c = dda->reprap_end_c;
			}
			// else we are already at target speed
		}
	#endif
	#ifdef ACCELERATION_RAMPING
		// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		// - precalculate ramp lengths instead of counting them, see AVR446 tech note
		uint8_t recalc_speed;

		// debug ramping algorithm
		//if (dda->move->ramping_step_no == 0) {
		//	sersendf_P(PSTR("\r\nc %lu  ramping_c_min %lu  n %d"), dda->c, dda->ramping_c_min, dda->move->ramping_n);
		//}

		recalc_speed = 0;
		if (dda->move->ramping_step_no < dda->rampup_steps) {
			if (dda->move->ramping_n < 0) // wrong ramp direction
				dda->move->ramping_n = -((int32_t)2) - dda->move->ramping_n;
			recalc_speed = 1;
		}
		else if (dda->move->ramping_step_no >= dda->rampdown_steps) {
			if (dda->move->ramping_n > 0) // wrong ramp direction
				dda->move->ramping_n = -((int32_t)2) - dda->move->ramping_n;
			recalc_speed = 1;
		}
		if (recalc_speed) {
			dda->move->ramping_n += 4;
			// be careful of signedness!
			dda->move->ramping_c = (int32_t)dda->move->ramping_c - ((int32_t)(dda->move->ramping_c * 2) / (int32_t)dda->move->ramping_n);
		}
		dda->move->ramping_step_no++;
		// Print the number of steps actually needed for ramping up
		// Needed for comparing the number with the one calculated in dda_create()
		//static char printed = 0;
		//if (printed == 0 && dda->ramping_c_min >= dda->move->ramping_c) {
		//  sersendf_P(PSTR("speedup %lu steps\n"), dda->move->ramping_step_no);
		//  printed = 1;
		//}
		//if (dda->move->ramping_step_no < 3) printed = 0;

		// debug ramping algorithm
		// raise this 10 for higher speeds to avoid flooding the serial line
		//if (dda->move->ramping_step_no % 10 /* 10, 50, 100, ...*/ == 0)
		//	sersendf_P(PSTR("\r\nc %lu  ramping_c_min %lu  n %ld"),
		//	           dda->move->ramping_c, dda->ramping_c_min, dda->move->ramping_n);
	#endif

	#ifdef ACCELERATION_TEMPORAL
		/** How is this ACCELERATION TEMPORAL expected to work?

			All axes work independently of each other, as if they were on four different, synchronized timers. As we have not enough suitable timers, we have to share one for all axes.

			To do this, each axis maintains the time of its last step in dda->{xyze}_time. This time is updated as the step is done, see early in dda_step(). To find out which axis is the next one to step, the time of each axis' next step is compared to the time of the step just done. Zero means this actually is the axis just stepped, the smallest value > 0 wins.

			One problem undoubtly arising is, steps should sometimes be done at {almost,exactly} the same time. We trust the timer to deal properly with very short or even zero periods. If a step can't be done in time, the timer shall do the step as soon as possible and compensate for the delay later. In turn we promise here to send a maximum of four such short-delays consecutively and to give sufficient time on average.
		*/
		dda->c = 0xFFFFFFFF;
		if (dda->move->steps) {
			dda->c = dda->move->temporal_time + dda->temporal_step_interval - dda->move->all_time;
		}
		dda->c <<= 8;
	#endif

	// If there are no steps left, we have finished.
	if (dda->move->steps == 0) {
		dda->live = 0;
	}

	#ifdef ACCELERATION_RAMPING
		// we don't hit maximum speed exactly with acceleration calculation, so limit it here
		// the nice thing about _not_ setting dda->c to dda->ramping_c_min is, the move stops at the exact same c as it started, so we have to calculate c only once for the time being
		// TODO: set timer only if dda->c has changed
		if (dda->ramping_c_min > dda->move->ramping_c)
			timer_charge(0, dda->ramping_c_min >> 8);
		else
			timer_charge(0, dda->move->ramping_c >> 8);
	#else
		timer_charge(0, dda->c >> 8);
	#endif
}

/// update global dda->current_position struct
void update_current_position(DDA_QUEUE *dda_queue) {
	DDA *dda = &(dda_queue->movebuffer[dda_queue->mb_tail]);

	if (queue_empty(dda_queue)) {
		dda->current_position.X = dda->startpoint.X;
	}
	else if (dda->live) {
		/* FIXME if (dda->endpoint.e_relative) {
			dda->current_position.E = dda->move->steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
		}
		else {*/
		
		if (dda->direction)
			// (STEPS_PER_M_X / 1000) is a bit inaccurate for low STEPS_PER_M numbers
			dda->current_position.X = dda->endpoint.X -
			                     // should be: dda->move->steps * 1000000 / STEPS_PER_M_X)
			                     // but steps can be like 1000000 already, so we'd overflow
			                     dda->move->steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);
		else
			dda->current_position.X = dda->endpoint.X +
			                     dda->move->steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);



		// dda->current_position.F is updated in dda_start()
	}
}

// called from step timer when current move is complete
void next_move(DDA_QUEUE *queue) __attribute__ ((hot));


/// check if the queue is completely full
uint8_t queue_full(DDA_QUEUE *queue) {
	MEMORY_BARRIER();
	if (queue->mb_tail > queue->mb_head) {
		return ((queue->mb_tail - queue->mb_head - 1) == 0) ? 255 : 0;
	} else {
		return ((queue->mb_tail + MOVEBUFFER_SIZE - queue->mb_head - 1) == 0) ? 255 : 0;
	}
}

/// check if the queue is completely empty
uint8_t queue_empty(DDA_QUEUE *queue) {
	uint8_t save_reg = SREG;
	cli();
	
	uint8_t result = ((queue->mb_tail == queue->mb_head) && (queue->movebuffer[queue->mb_tail].live == 0))?255:0;

	MEMORY_BARRIER();
	SREG = save_reg;

	return result;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void queue_step(DDA_QUEUE *queue) {
	// do our next step
	DDA* current_movebuffer = &queue->movebuffer[queue->mb_tail];
	if (current_movebuffer->live) {
		// NOTE: dda_step makes this interrupt interruptible for some time,
		//       see STEP_INTERRUPT_INTERRUPTIBLE.
		dda_step(current_movebuffer);
	}

	// fall directly into dda_start instead of waiting for another step
	// the dda dies not directly after its last step, but when the timer fires and there's no steps to do
	if (current_movebuffer->live == 0)
		next_move(queue);
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
/// This is the only function that modifies queue->mb_head and it always called from outside an interrupt.
void queue_enqueue(DDA_QUEUE *queue, TARGET *t) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full(queue))
		delay(WAITING_DELAY);

	uint8_t h = queue->mb_head + 1;
	h &= (MOVEBUFFER_SIZE - 1);

	DDA* new_movebuffer = &(queue->movebuffer[h]);
	
	if (t != NULL) {
		if( dda_create(new_movebuffer, t) != 0) // null move
			return;
	}

	// make certain all writes to global memory
	// are flushed before modifying queue->mb_head.
	MEMORY_BARRIER();
	
	queue->mb_head = h;
	
	uint8_t save_reg = SREG;
	cli();

	uint8_t isdead = (queue->movebuffer[queue->mb_tail].live == 0);
	
	MEMORY_BARRIER();
	SREG = save_reg;
	
	if (isdead) {
		next_move(queue);
		// Compensate for the cli() in setTimer().
		sei(); // FIXME do we need it now?
	}
}

/// go to the next move.
/// be aware that this is sometimes called from interrupt context, sometimes not.
/// Note that if it is called from outside an interrupt it must not/can not by
/// be interrupted such that it can be re-entered from within an interrupt.
/// The timer interrupt MUST be disabled on entry. This is ensured because
/// the timer was disabled at the start of the ISR or else because the current
/// move buffer was dead in the non-interrupt case (which indicates that the 
/// timer interrupt is disabled).
void next_move(DDA_QUEUE *queue) {
	while ((queue_empty(queue) == 0) && (queue->movebuffer[queue->mb_tail].live == 0)) {
		// next item
		uint8_t t = queue->mb_tail + 1;
		t &= (MOVEBUFFER_SIZE - 1);
		DDA* current_movebuffer = &queue->movebuffer[t];
		// tail must be set before setTimer call as setTimer
		// reenables the timer interrupt, potentially exposing
		// queue->mb_tail to the timer interrupt routine. 
		queue->mb_tail = t;
		
		dda_start(current_movebuffer);
	} 
}

void queue_init(DDA_QUEUE *queue){
	queue->mb_head = 0;
	queue->mb_tail = 0;
}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void queue_print(DDA_QUEUE *queue) {
	sersendf_P(PSTR("Q%d/%d%c"), queue->mb_tail, queue->mb_head, (queue_full(queue)?'F':(queue_empty(queue)?'E':' ')));
}

/// dump queue for emergency stop.
/// \todo effect on startpoint is undefined!
void queue_flush(DDA_QUEUE *queue) {
	// Since the timer interrupt is disabled before this function
	// is called it is not strictly necessary to write the variables
	// inside an interrupt disabled block...
	uint8_t save_reg = SREG;
	cli();
	
	// flush queue
	queue->mb_tail = queue->mb_head;
	queue->movebuffer[queue->mb_head].live = 0;

	// disable timer
	timer_disable(0);
	
	MEMORY_BARRIER();
	SREG = save_reg;
}

/// wait for queue to empty
void queue_wait(DDA_QUEUE *queue) {
	for (;queue_empty(queue) == 0;) {
		ifclock(clock_flag_10ms) {
			clock_10ms();
		}
	}
}
