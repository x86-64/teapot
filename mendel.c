/** \file
	\brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
	\section intro_sec Introduction
		Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

		Please see README for a full introduction and long-winded waffle about this project
	\section install_sec	Installation
		\subsection step1 Step 1: Download
			\code git clone git://github.com/triffid/Teacup_Firmware \endcode
		\subsection step2 Step 2: configure
			\code cp config.[yourboardhere].h common.h \endcode
			Edit common.h to suit your machone
			Edit Makefile to select the correct chip and programming settings
		\subsection step3 Step 3: Compile
			\code make \endcode
			\code make program \endcode
		\subsection step4 Step 4: Test!
			\code ./func.sh mendel_reset
			./func.sh mendel_talk
			M115
			ctrl+d \endcode
*/

#define FEATURES_C

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"common.h"

#include	"serial.h"
#include	"timer.h"
#include	"debug.h"
#include	"pinio.h"
#include	"arduino.h"
#include	"memory_barrier.h"

/// Startup code, run when we come out of reset
void init(void) {
	arduino_init();

	debug_init();

	// set up serial
	serial_init();

	// set up timers
	timers_init();

	features_init();
	
	// read PID settings from EEPROM
	//heater_init();

	// enable interrupts
	sei();

	// reset watchdog
	//wd_reset();

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));

}

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms(void) {
	#ifndef	NO_AUTO_IDLE
/*	if (temp_all_zero())	{
		if (psu_timeout > (30 * 4)) {
			power_off();
		}
		else {
			uint8_t save_reg = SREG;
			cli();
			psu_timeout++;
			MEMORY_BARRIER();
			SREG = save_reg;
		}
	}*/
	// FIXME auto idle depend on temperature status
	#endif

	ifclock(clock_flag_1s) {
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			// FIXME update_current_position();
			//sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lq,%lu\n"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);

			// target position
			//sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E, movebuffer[mb_tail].endpoint.F);

			// Queue
			//print_queue();

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
void clock_10ms(void) {
	core_emit(EVENT_TICK_10MS, 0);
	
	temp_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}


/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
int main (void)
{
	init();

	// main loop
	for (;;)
	{
		// if queue is full, no point in reading chars- host will just have to wait
		if ((serial_rxchars() != 0)) {
			uint8_t c = serial_popchar();
			gcode_parse_char(c);
		}

		ifclock(clock_flag_10ms) {
			clock_10ms();
		}
	}
}


