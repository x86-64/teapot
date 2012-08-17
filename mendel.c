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
#include	"dda_queue.h"
#include	"dda.h"
#include	"timer.h"
#include	"debug.h"
#include	"pinio.h"
#include	"arduino.h"
#include	"dda_queue.h"
#include	"memory_barrier.h"

/// initialise all I/O - set pins as input or output, turn off unused subsystems, etc
void io_init(void) {
	// disable modules we don't use
	#ifdef PRR
		PRR = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
	#elif defined PRR0
		PRR0 = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
		#if defined(PRUSART3)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART3) | MASK(PRUSART2);
		#endif
		#if defined(PRUSART2)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART2);
		#endif
	#endif
	ACSR = MASK(ACD);

	// setup I/O pins

	// X Stepper
	WRITE(X_STEP_PIN, 0);	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_DIR_PIN,  0);	SET_OUTPUT(X_DIR_PIN);
	#ifdef X_MIN_PIN
		SET_INPUT(X_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MIN_PIN, 1);
		#else
			WRITE(X_MIN_PIN, 0);
		#endif
	#endif
	#ifdef X_MAX_PIN
		SET_INPUT(X_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MAX_PIN, 1);
		#else
			WRITE(X_MAX_PIN, 0);
		#endif
	#endif

	// Y Stepper
	WRITE(Y_STEP_PIN, 0);	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_DIR_PIN,  0);	SET_OUTPUT(Y_DIR_PIN);
	#ifdef Y_MIN_PIN
		SET_INPUT(Y_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Y_MIN_PIN, 1);
		#else
			WRITE(Y_MIN_PIN, 0);
		#endif
	#endif
	#ifdef Y_MAX_PIN
		SET_INPUT(Y_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Y_MAX_PIN, 1);
		#else
			WRITE(Y_MAX_PIN, 0);
		#endif
	#endif

	// Z Stepper
	#if defined Z_STEP_PIN && defined Z_DIR_PIN
		WRITE(Z_STEP_PIN, 0);	SET_OUTPUT(Z_STEP_PIN);
		WRITE(Z_DIR_PIN,  0);	SET_OUTPUT(Z_DIR_PIN);
	#endif
	#ifdef Z_MIN_PIN
		SET_INPUT(Z_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Z_MIN_PIN, 1);
		#else
			WRITE(Z_MIN_PIN, 0);
		#endif
	#endif
	#ifdef Z_MAX_PIN
		SET_INPUT(Z_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Z_MAX_PIN, 1);
		#else
			WRITE(Z_MAX_PIN, 0);
		#endif
	#endif

	#if defined E_STEP_PIN && defined E_DIR_PIN
		WRITE(E_STEP_PIN, 0);	SET_OUTPUT(E_STEP_PIN);
		WRITE(E_DIR_PIN,  0);	SET_OUTPUT(E_DIR_PIN);
	#endif

	// Common Stepper Enable
	#ifdef STEPPER_ENABLE_PIN
		#ifdef STEPPER_INVERT_ENABLE
			WRITE(STEPPER_ENABLE_PIN, 0);
		#else
			WRITE(STEPPER_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(STEPPER_ENABLE_PIN);
	#endif

	// X Stepper Enable
	#ifdef X_ENABLE_PIN
		#ifdef X_INVERT_ENABLE
			WRITE(X_ENABLE_PIN, 0);
		#else
			WRITE(X_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(X_ENABLE_PIN);
	#endif

	// Y Stepper Enable
	#ifdef Y_ENABLE_PIN
		#ifdef Y_INVERT_ENABLE
			WRITE(Y_ENABLE_PIN, 0);
		#else
			WRITE(Y_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(Y_ENABLE_PIN);
	#endif

	// Z Stepper Enable
	#ifdef Z_ENABLE_PIN
		#ifdef Z_INVERT_ENABLE
			WRITE(Z_ENABLE_PIN, 0);
		#else
			WRITE(Z_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(Z_ENABLE_PIN);
	#endif

	// E Stepper Enable
	#ifdef E_ENABLE_PIN
		#ifdef E_INVERT_ENABLE
			WRITE(E_ENABLE_PIN, 0);
		#else
			WRITE(E_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(E_ENABLE_PIN);
	#endif

	// setup PWM timers: fast PWM, no prescaler
	TCCR0A = MASK(WGM01) | MASK(WGM00);
	// PWM frequencies in TCCR0B, see page 108 of the ATmega644 reference.
	TCCR0B = MASK(CS00); // F_CPU / 256 (about 78(62.5) kHz on a 20(16) MHz chip)
	//TCCR0B = MASK(CS01);              // F_CPU / 256 / 8  (about 9.8(7.8) kHz)
	//TCCR0B = MASK(CS00) | MASK(CS01); // F_CPU / 256 / 64  (about 1220(977) Hz)
	//TCCR0B = MASK(CS02);              // F_CPU / 256 / 256  (about 305(244) Hz)
	#ifndef FAST_PWM
		TCCR0B = MASK(CS00) | MASK(CS02); // F_CPU / 256 / 1024  (about 76(61) Hz)
	#endif
	TIMSK0 = 0;
	OCR0A = 0;
	OCR0B = 0;

	TCCR2A = MASK(WGM21) | MASK(WGM20);
	// PWM frequencies in TCCR2B, see page 156 of the ATmega644 reference.
	TCCR2B = MASK(CS20); // F_CPU / 256  (about 78(62.5) kHz on a 20(16) MHz chip)
	//TCCR2B = MASK(CS21);              // F_CPU / 256 / 8  (about 9.8(7.8) kHz)
	//TCCR2B = MASK(CS20) | MASK(CS21); // F_CPU / 256 / 32  (about 2.4(2.0) kHz)
	//TCCR2B = MASK(CS22);              // F_CPU / 256 / 64  (about 1220(977) Hz)
	//TCCR2B = MASK(CS20) | MASK(CS22); // F_CPU / 256 / 128  (about 610(488) Hz)
	//TCCR2B = MASK(CS21) | MASK(CS22); // F_CPU / 256 / 256  (about 305(244) Hz)
	#ifndef FAST_PWM
		TCCR2B = MASK(CS20) | MASK(CS21) | MASK(CS22); // F_CPU / 256 / 1024
	#endif
	TIMSK2 = 0;
	OCR2A = 0;
	OCR2B = 0;

	#ifdef	TCCR3A
		TCCR3A = MASK(WGM30);
		TCCR3B = MASK(WGM32) | MASK(CS30);
		TIMSK3 = 0;
		OCR3A = 0;
		OCR3B = 0;
	#endif

	#ifdef	TCCR4A
		TCCR4A = MASK(WGM40);
		TCCR4B = MASK(WGM42) | MASK(CS40);
		TIMSK4 = 0;
		OCR4A = 0;
		OCR4B = 0;
	#endif

	#ifdef	TCCR5A
		TCCR5A = MASK(WGM50);
		TCCR5B = MASK(WGM52) | MASK(CS50);
		TIMSK5 = 0;
		OCR5A = 0;
		OCR5B = 0;
	#endif

	#ifdef	STEPPER_ENABLE_PIN
		power_off();
	#endif

	#ifdef	TEMP_MAX6675
		// setup SPI
		WRITE(SCK, 0);				SET_OUTPUT(SCK);
		WRITE(MOSI, 1);				SET_OUTPUT(MOSI);
		WRITE(MISO, 1);				SET_INPUT(MISO);
		WRITE(SS, 1);					SET_OUTPUT(SS);
	#endif
}

/// Startup code, run when we come out of reset
void init(void) {
	features_init();

	// set up serial
	serial_init();

	// set up inputs and outputs
	io_init();

	// set up timers
	timers_init();

	// read PID settings from EEPROM
	//heater_init();

	// set up dda
	dda_init();

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
		if ((serial_rxchars() != 0) && (queue_full() == 0)) {
			uint8_t c = serial_popchar();
			gcode_parse_char(c);
		}

		ifclock(clock_flag_10ms) {
			clock_10ms();
		}
	}
}


