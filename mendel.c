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

/// Startup code, run when we come out of reset
void init(void) {
	arduino_init();

	debug_init();

	// set up serial
	serial_init();

	// set up timers
	timers_init();

	features_init();

	// enable interrupts
	sei();

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));

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
	}
}


