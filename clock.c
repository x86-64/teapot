#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"pinio.h"
#include	"sersendf.h"
#include	"dda_queue.h"
#include	"temp.h"
#include	"timer.h"
#include	"debug.h"
#include	"heater.h"
#include	"serial.h"
#include	"memory_barrier.h"

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms() {
	#ifndef	NO_AUTO_IDLE
	if (temp_all_zero())	{
		if (psu_timeout > (30 * 4)) {
			power_off();
		}
		else {
			uint8_t save_reg = SREG;
			cli();
			CLI_SEI_BUG_MEMORY_BARRIER();
			psu_timeout++;
			MEMORY_BARRIER();
			SREG = save_reg;
		}
	}
	#endif

	ifclock(clock_flag_1s) {
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			update_current_position();
			sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lq,%lu\n"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);

			// target position
			sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E, movebuffer[mb_tail].endpoint.F);

			// Queue
			print_queue();

			// newline
			serial_writechar('\n');
		}
		// temperature
		/*		if (temp_get_target())
		temp_print();*/
	}
}

/*! do stuff every 10 milliseconds

	call from ifclock(CLOCK_FLAG_10MS) in busy loops
*/
void clock_10ms() {
	// reset watchdog
	//wd_reset();
	
	// call callbacks

	temp_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}

