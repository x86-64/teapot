#include "common.h"
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

#include	"memory_barrier.h"


#define dda_queue_have_space()  queue_have_space (&dda_queue->mb_head, &dda_queue->mb_tail, MOVEBUFFER_SIZE)
#define dda_queue_have_item()   queue_have_item  (&dda_queue->mb_tail, &dda_queue->mb_head, MOVEBUFFER_SIZE)
#define dda_queue_push()        queue_push       (&dda_queue->mb_head, &dda_queue->mb_tail, MOVEBUFFER_SIZE)
#define dda_queue_pop()         queue_pop        (&dda_queue->mb_tail, &dda_queue->mb_head, MOVEBUFFER_SIZE)
#define dda_queue_curr_space()  queue_current    (&dda_queue->mb_head)
#define dda_queue_curr_item()   queue_current    (&dda_queue->mb_tail)

//#include "graycode.c"

/*! Distribute a new position_start to dda's internal structures without any movement.

	This is needed for example after homing or a G92. The new location must be in position_start already.
*/

void dda_create_acceleration_none(dda_t *dda, dda_target_t *position_start, dda_target_t *position_target){ // {{{
	uint32_t	       target_st;
	uint32_t               start_st;
	
	dda->allflags = 0;

/* FIXME	if (target->e_relative) {
		delta_um = labs(target->E);
		dda->delta_steps = labs(um_to_steps_e(target->E));
		dda->direction = (target->E >= 0)?1:0;
	}
	else {*/
		target_st        = um_to_steps_x(position_target->X);
		start_st         = um_to_steps_x(position_start->X);
		dda->delta_um    = (uint32_t)labs(position_target->X - position_start->X);
		dda->delta_steps = (uint32_t)labs(target_st - start_st);
		dda->direction   = (position_target->X >= position_start->X) ? 1 : 0;
	//}
	
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
	uint32_t move_duration = ((dda->delta_um * 2400) / dda->delta_steps) * (F_CPU / 40000);
	
	
	dda->c = (move_duration / position_target->F) << 8;
} // }}}

#ifdef ACCELERATION_TEMPORAL
/** How is this ACCELERATION TEMPORAL expected to work?

	All axes work independently of each other, as if they were on four different, synchronized timers. As we have not enough suitable timers, we have to share one for all axes.

	To do this, each axis maintains the time of its last step in dda->{xyze}_time. This time is updated as the step is done, see early in dda_step(). To find out which axis is the next one to step, the time of each axis' next step is compared to the time of the step just done. Zero means this actually is the axis just stepped, the smallest value > 0 wins.

	One problem undoubtly arising is, steps should sometimes be done at {almost,exactly} the same time. We trust the timer to deal properly with very short or even zero periods. If a step can't be done in time, the timer shall do the step as soon as possible and compensate for the delay later. In turn we promise here to send a maximum of four such short-delays consecutively and to give sufficient time on average.
*/
void dda_create_acceleration_temporal(dda_t *dda, dda_target_t *position_start, dda_target_t *position_target){ // {{{
	// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
	uint32_t move_duration = dda->delta_um * ((60 * F_CPU) / (position_target->F * 1000UL));
	
	dda->c = (move_duration / dda->delta_steps) << 8;
} // }}}

void dda_step_acceleration_temporal(dda_t *dda){ // {{{
	// do nothing, dda->c is okay
} // }}}
#endif

#ifdef ACCELERATION_RAMPING
void dda_create_acceleration_ramping(dda_t *dda, dda_target_t *position_start, dda_target_t *position_target){ // {{{
	
	dda->move->ramping_n = 1;
	dda->move->ramping_c = ((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_M_X * ACCELERATION / 1000.)))) << 8;
	
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
	uint32_t move_duration = ((dda->delta_um * 2400) / dda->delta_steps) * (F_CPU / 40000);
	
	// yes, this assumes always the x axis as the critical one regarding acceleration. If we want to implement per-axis acceleration, things get tricky ...
	dda->ramping_c_min = (move_duration / position_target->F) << 8;
	//dda->ramping_c_min = MAX(dda->ramping_c_min, c_limit);
// This section is plain wrong, like in it's only half of what we need. This factor 960000 is dependant on STEPS_PER_MM.
// overflows at position_target->F > 65535; factor 16. found by try-and-error; will overshoot target speed a bit
	dda->rampup_steps = position_target->F * position_target->F / (uint32_t)(STEPS_PER_M_X * ACCELERATION / 960000.);
//sersendf_P(PSTR("rampup calc %lu\n"), dda->rampup_steps);
	dda->rampup_steps = 100000; // replace mis-calculation by a safe value
// End of wrong section.
	if (dda->rampup_steps > dda->delta_steps / 2)
		dda->rampup_steps = dda->delta_steps / 2;
	dda->rampdown_steps = dda->delta_steps - dda->rampup_steps;
} // }}}
		
void dda_step_acceleration_ramping(dda_t *dda){ // {{{
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
		
	// we don't hit maximum speed exactly with acceleration calculation, so limit it here
	// the nice thing about _not_ setting dda->c to dda->ramping_c_min is, the move stops at the exact same c as it started, so we have to calculate c only once for the time being
	// TODO: set timer only if dda->c has changed
	
	if (dda->ramping_c_min > dda->move->ramping_c)
		dda->c = dda->ramping_c_min >> 8;
	else
		dda->c = dda->move->ramping_c >> 8;
} // }}}
#endif

#ifdef ACCELERATION_REPRAP
void dda_create_acceleration_reprap(dda_t *dda, dda_target_t *position_start, dda_target_t *position_target){ // {{{
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
	uint32_t move_duration = ((dda->delta_um * 2400) / dda->delta_steps) * (F_CPU / 40000);
	
	// c is initial step time in IOclk ticks
	dda->c            = (move_duration / position_start->F) << 8;
	//dda->c            = MAX(dda->c, c_limit);
	dda->reprap_end_c = (move_duration / position_target->F) << 8;
	//dda->reprap_end_c = MAX(dda->reprap_end_c, c_limit);

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c >> 8);

	if (dda->c != dda->reprap_end_c) {
		uint32_t stF = position_start->F / 4;
		uint32_t enF = position_target->F / 4;
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
			sersendf_P(PSTR("\n{dda:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->reprap_end_c >> 8, dda->reprap_n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

		dda->accel = 1;
	}
	else
		dda->accel = 0;
} // }}}

void dda_step_acceleration_reprap(dda_t *dda){ // {{{
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
} // }}}
#endif


/*! CREATE a dda given dda->position_current and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue_t entry to overwrite
	\param *target the target position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
uint8_t dda_create(dda_t *dda, dda_target_t *position_start, dda_target_t *position_target) {
	if(position_target->X == position_start->X)
		return 1; // ERR, nothing to move
	
	dda_create_acceleration_none(dda, position_start, position_target);
	#if defined ACCELERATION_REPRAP
		dda_create_acceleration_reprap(dda, position_start, position_target);
	#elif defined ACCELERATION_RAMPING
		dda_create_acceleration_ramping(dda, position_start, position_target);
	#elif defined ACCELERATION_TEMPORAL
		dda_create_acceleration_temporal(dda, position_start, position_target);
	#endif

	//c_limit = ((dda->delta_um * 2400L) / dda->delta_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8; // wtf?
	uint32_t c_limit = (F_CPU / um_to_steps_x(MAXIMUM_FEEDRATE_X)) * 10000L;
	
	// snap timer value to minimal
	dda->c = MAX(dda->c, c_limit);
	
	// next dda starts where we finish
	dda->status = DDA_READY;
	return 0;
}

/*! dda step routine, caltulate order to stepper and next time to call
 */
void dda_step(dda_t *dda, dda_order_t *order) {
	switch(dda->status){
		case DDA_READY:
			#ifdef ACCELERATION_RAMPING
				dda->move->ramping_step_no = 0;
			#endif
			
			dda->status = DDA_RUNNING;
			
			order->callme    = 1;   // ask for call
			order->c         = 0;   // as soon as you can
			break;
			
		case DDA_RUNNING:
			dda->delta_steps--;
			
			#if defined ACCELERATION_REPRAP
				dda_step_acceleration_reprap(dda);
			#elif defined ACCELERATION_RAMPING
				dda_step_acceleration_ramping(dda);
			#elif defined ACCELERATION_TEMPORAL
				dda_step_acceleration_temporal(dda);
			#endif
			
			// If there are no steps left, we have finished.
			if (dda->delta_steps == 0){
				dda->status = DDA_FINISHED;
				order->done = 1;
			}
			
			order->callme    = 1;                // ask for call
			order->c         = dda->c;           // next time to call
			
			order->step      = 1;                // ask for step
			order->direction = dda->direction;   // in this direction
			break;
		
		case DDA_FINISHED:
			break;
	}
}


/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void dda_queue_debug_print(dda_queue_t *dda_queue) {
	sersendf_P(PSTR("Q%d/%d"), dda_queue->mb_tail, dda_queue->mb_head);
}

void dda_queue_init(dda_queue_t *dda_queue){
	uint8_t i;
	
	dda_queue->mb_head = 0;
	dda_queue->mb_tail = 0;
	
	for(i=0; i<MOVEBUFFER_SIZE; i++){
		dda_queue->movebuffer[i].status = DDA_FINISHED;
	}
	MEMORY_BARRIER();
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void dda_queue_step(dda_queue_t *dda_queue, dda_order_t *order) {
	dda_t                 *dda_curr          = &dda_queue->movebuffer[ dda_queue_curr_item() ];
	
	// initialize order to empty value
	order->done      = 0;
	order->callme    = 0;
	order->step      = 0;
	
	switch(dda_curr->status){
		case DDA_READY:  // do our steps
		case DDA_RUNNING:
			dda_step(dda_curr, order);
			break;
			
		case DDA_FINISHED: // goto next queued move
			if(dda_queue_pop() == 0){                  // try to pop current item, if this is possible:
				order->callme = 1;                 //   call me
				order->c      = 0;                 //   as soon as possible
			}
			break;
	}
}

/// add a move to the movebuffer
void dda_queue_enqueue(dda_queue_t *dda_queue, dda_target_t *start, dda_target_t *target) {
	dda_t                  dda_new;
	dda_t                 *dda_curr          = &dda_queue->movebuffer[ dda_queue_curr_space() ];
	
	if( dda_create(&dda_new, start, target) != 0) // null move
		return;
	
	if(DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("dda_enqueue: x:%ld f:%ld\n"), target->X, target->F);
	
	*dda_curr = dda_new;

	while(dda_queue_push() != 0)
		delay(WAITING_DELAY);
}

