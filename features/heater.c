#include "common.h"

#include	<stdlib.h>
#include	<avr/eeprom.h>
#include	<avr/pgmspace.h>

#include	"arduino.h"
#include	"debug.h"
#include	"crc.h"

/// default scaled P factor, equivalent to 8.0
#define		DEFAULT_P				8192
/// default scaled I factor, equivalent to 0.5
#define		DEFAULT_I				512
/// default scaled D factor, equivalent to 24
#define		DEFAULT_D				24576
/// default scaled I limit
#define		DEFAULT_I_LIMIT	384

/// \struct heater_t
/// \brief simply holds pinout data- port, pin, pwm channel if used
typedef struct heater_t {
	volatile uint8_t      *heater_port; ///< pointer to port. DDR is inferred from this pointer too
	uint8_t	               heater_pin;  ///< heater pin, not masked. eg for PB3 enter '3' here, or PB3_PIN or similar
	volatile uint8_t      *heater_pwm;  ///< pointer to 8-bit PWM register, eg OCR0A (8-bit) or ORC3L (low byte, 16-bit)
} heater_t;

/**
	\struct heater_pid_t
	\brief this struct holds the heater PID factors

	PID is a fascinating way to control any closed loop control, combining the error (P), cumulative error (I) and rate at which we're approacing the setpoint (D) in such a way that when correctly tuned, the system will achieve target temperature quickly and with little to no overshoot

	At every sample, we calculate \f$OUT = k_P (S - T) + k_I \int (S - T) + k_D \frac{dT}{dt}\f$ where S is setpoint and T is temperature.

	The three factors kP, kI, kD are chosen to give the desired behaviour given the dynamics of the system.

	See http://www.eetimes.com/design/embedded/4211211/PID-without-a-PhD for the full story
*/
typedef struct heater_pid_t {
	int32_t						p_factor; ///< scaled P factor
	int32_t						i_factor; ///< scaled I factor
	int32_t						d_factor; ///< scaled D factor
	int16_t						i_limit;  ///< scaled I limit, such that \f$-i_{limit} < i_{factor} < i_{limit}\f$
	uint16_t                                        crc;      ///< crc so we can use defaults if eeprom data is invalid
} heater_pid_t;
const uint8_t            heater_pid_EE_size = sizeof(heater_pid_t) - sizeof(uint16_t); // size of structure - sizeof(crc);

/// \brief this struct holds the runtime heater data- PID integrator history, temperature history, sanity checker
typedef struct heater_runtime_t {
	int16_t						heater_i; ///< integrator, \f$-i_{limit} < \sum{\Delta t} < i_{limit}\f$

	uint16_t					temp_history[TH_COUNT]; ///< store last TH_COUNT readings in a ring, so we can smooth out our differentiator
	uint8_t						temp_history_pointer;   ///< pointer to last entry in ring

	#ifdef	HEATER_SANITY_CHECK
		uint16_t					sanity_counter;				///< how long things haven't seemed sane
		uint16_t					sane_temperature;			///< a temperature we consider sane given the heater settings
	#endif

	uint8_t						heater_output;					///< this is the PID value we eventually send to the heater
} heater_runtime_t;

#undef DEFINE_HEATER
#define        DEFINE_HEATER(name, pin) { &(pin ## _WPORT), pin ## _PIN, (pin ## _PWM) },
const heater_t           heaters[] = {
/* FIXME	DEFINE_HEATER(bed, PB3)
	DEFINE_HEATER(bed, PB4)*/
};
#define NUM_HEATERS     (sizeof(heaters) / sizeof(heaters[0])) 
const uint8_t            heaters_count  = NUM_HEATERS;
      heater_pid_t       heaters_pid     [NUM_HEATERS];
      heater_pid_t EEMEM heaters_pid_EE  [NUM_HEATERS];
      heater_runtime_t   heaters_runtime [NUM_HEATERS];


#define	enable_heater()		heater_set(0, 64)
#define	disable_heater()	heater_set(0, 0)

API void heater_init(void);

/** \file
	\brief Manage heaters
*/


/// \brief initialise heater subsystem
/// Set directions, initialise PWM timers, read PID factors from eeprom, etc
void heater_init() {
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
	
	// set all heater pins to output
	do {
		#undef	DEFINE_HEATER
		#define	DEFINE_HEATER(name, pin) WRITE(pin, 0); SET_OUTPUT(pin);
			#include "common.h"
		#undef DEFINE_HEATER
	} while (0);
	
	uint8_t i;
	// setup pins
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters[i].heater_pwm) {
			*heaters[i].heater_pwm = 0;
			// this is somewhat ugly too, but switch() won't accept pointers for reasons unknown
			switch((uint16_t) heaters[i].heater_pwm) {
				case (uint16_t) &OCR0A:
					TCCR0A |= MASK(COM0A1);
					break;
				case (uint16_t) &OCR0B:
					TCCR0A |= MASK(COM0B1);
					break;
				case (uint16_t) &OCR2A:
					TCCR2A |= MASK(COM2A1);
					break;
				case (uint16_t) &OCR2B:
					TCCR2A |= MASK(COM2B1);
					break;
				#ifdef TCCR3A
				case (uint16_t) &OCR3AL:
					TCCR3A |= MASK(COM3A1);
					break;
				case (uint16_t) &OCR3BL:
					TCCR3A |= MASK(COM3B1);
					break;
				#ifdef COM3C1
				case (uint16_t) &OCR3CL:
					TCCR3A |= MASK(COM3C1);
					break;
				#endif
				#endif
				#ifdef	TCCR4A
				case (uint16_t) &OCR4AL:
					TCCR4A |= MASK(COM4A1);
					break;
				case (uint16_t) &OCR4BL:
					TCCR4A |= MASK(COM4B1);
					break;
				case (uint16_t) &OCR4CL:
					TCCR4A |= MASK(COM4C1);
					break;
				#endif
				#ifdef	TCCR5A
				case (uint16_t) &OCR5AL:
					TCCR5A |= MASK(COM5A1);
					break;
				case (uint16_t) &OCR5BL:
					TCCR5A |= MASK(COM5B1);
					break;
				case (uint16_t) &OCR5CL:
					TCCR5A |= MASK(COM5C1);
					break;
				#endif
			}
		}

		#ifdef	HEATER_SANITY_CHECK
			// 0 is a "sane" temperature when we're trying to cool down
			heaters_runtime[i].sane_temperature = 0;
		#endif

		#ifndef BANG_BANG
			// read factors from eeprom
			heaters_pid[i].p_factor = eeprom_read_dword((uint32_t *) &heaters_pid_EE[i].p_factor);
			heaters_pid[i].i_factor = eeprom_read_dword((uint32_t *) &heaters_pid_EE[i].i_factor);
			heaters_pid[i].d_factor = eeprom_read_dword((uint32_t *) &heaters_pid_EE[i].d_factor);
			heaters_pid[i].i_limit  = eeprom_read_word ((uint16_t *) &heaters_pid_EE[i].i_limit);

// 			if ((heaters_pid[i].p_factor == 0) && (heaters_pid[i].i_factor == 0) && (heaters_pid[i].d_factor == 0) && (heaters_pid[i].i_limit == 0)) {
			if (crc_block(&heaters_pid[i].p_factor, heater_pid_EE_size) != eeprom_read_word((uint16_t *) &heaters_pid_EE[i].crc)) {
				heaters_pid[i].p_factor = DEFAULT_P;
				heaters_pid[i].i_factor = DEFAULT_I;
				heaters_pid[i].d_factor = DEFAULT_D;
				heaters_pid[i].i_limit  = DEFAULT_I_LIMIT;
			}
		#endif /* BANG_BANG */
	}
}

/** \brief manually set PWM output
	\param id the heater we're setting the output for
	\param value the PWM value to write

	anything done by this function is overwritten by heater_tick above if the heater has an associated temp sensor
*/
void heater_set(uint8_t id, uint8_t value) {
	if (id >= NUM_HEATERS)
		return;

	heaters_runtime[id].heater_output = value;

	if (heaters[id].heater_pwm) {
		*(heaters[id].heater_pwm) = value;
		#ifdef	DEBUG
		if (DEBUG_PID && (debug_flags & DEBUG_PID))
			sersendf_P(PSTR("PWM{%u = %u}\n"), id, OCR0A);
		#endif
	}
	else {
		if (value >= 8)
			*(heaters[id].heater_port) |= MASK(heaters[id].heater_pin);
		else
			*(heaters[id].heater_port) &= ~MASK(heaters[id].heater_pin);
	}
}

/// \brief Write PID factors to eeprom
void heater_save_settings(void) {
	#ifndef BANG_BANG
		uint8_t i;
		for (i = 0; i < NUM_HEATERS; i++) {
			eeprom_write_dword((uint32_t *) &heaters_pid_EE[i].p_factor, heaters_pid[i].p_factor);
			eeprom_write_dword((uint32_t *) &heaters_pid_EE[i].i_factor, heaters_pid[i].i_factor);
			eeprom_write_dword((uint32_t *) &heaters_pid_EE[i].d_factor, heaters_pid[i].d_factor);
			eeprom_write_word ((uint16_t *) &heaters_pid_EE[i].i_limit,  heaters_pid[i].i_limit);
			eeprom_write_word ((uint16_t *) &heaters_pid_EE[i].crc,      crc_block(&heaters_pid[i].p_factor, heater_pid_EE_size));
		}
	#endif /* BANG_BANG */
}

/** \brief run heater PID algorithm
	\param id which heater we're running the loop for
	\param type which temp sensor type this heater is attached to
	\param current_temp the temperature that the associated temp sensor is reporting
	\param target_temp the temperature we're trying to achieve
*/
void heater_tick(uint8_t id /*, temp_type_t type*/, uint16_t current_temp, uint16_t target_temp) {
	uint8_t		pid_output;

	#ifndef	BANG_BANG
		int16_t		heater_p;
		int16_t		heater_d;
		int16_t		t_error = target_temp - current_temp;
	#endif	/* BANG_BANG */

	if (id >= NUM_HEATERS)
		return;

	if (target_temp == 0) {
		heater_set(id, 0);
		return;
	}

/*	#ifdef TEMP_NONE
		if (type == TT_NONE) {
			// it's something like a milling spindle
			heater_set(id, target_temp >> 2);
			return;
		}
	#endif  TEMP_NONE */

	#ifndef	BANG_BANG
		heaters_runtime[id].temp_history[heaters_runtime[id].temp_history_pointer++] = current_temp;
		heaters_runtime[id].temp_history_pointer &= (TH_COUNT - 1);

		// PID stuff
		// proportional
		heater_p = t_error;

		// integral
		heaters_runtime[id].heater_i += t_error;
		// prevent integrator wind-up
		if (heaters_runtime[id].heater_i > heaters_pid[id].i_limit)
			heaters_runtime[id].heater_i = heaters_pid[id].i_limit;
		else if (heaters_runtime[id].heater_i < -heaters_pid[id].i_limit)
			heaters_runtime[id].heater_i = -heaters_pid[id].i_limit;

		// derivative
		// note: D follows temp rather than error so there's no large derivative when the target changes
		heater_d = heaters_runtime[id].temp_history[heaters_runtime[id].temp_history_pointer] - current_temp;

		// combine factors
		int32_t pid_output_intermed = (
			(
				(((int32_t) heater_p) * heaters_pid[id].p_factor) +
				(((int32_t) heaters_runtime[id].heater_i) * heaters_pid[id].i_factor) +
				(((int32_t) heater_d) * heaters_pid[id].d_factor)
			) / PID_SCALE
		);

		// rebase and limit factors
		if (pid_output_intermed > 255)
			pid_output = 255;
		else if (pid_output_intermed < 0)
			pid_output = 0;
		else
			pid_output = pid_output_intermed & 0xFF;

		#ifdef	DEBUG
		if (DEBUG_PID && (debug_flags & DEBUG_PID))
			sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, heaters_pid[id].p_factor, (int32_t) heater_p * heaters_pid[id].p_factor / PID_SCALE, heaters_runtime[id].heater_i, heaters_pid[id].i_factor, (int32_t) heaters_runtime[id].heater_i * heaters_pid[id].i_factor / PID_SCALE, heater_d, heaters_pid[id].d_factor, (int32_t) heater_d * heaters_pid[id].d_factor / PID_SCALE, pid_output_intermed, pid_output);
		#endif
	#else
		if (current_temp >= target_temp)
			pid_output = BANG_BANG_OFF;
		else
			pid_output = BANG_BANG_ON;
	#endif

	#ifdef	HEATER_SANITY_CHECK
	// check heater sanity
	// implementation is a moving window with some slow-down to compensate for thermal mass
	if (target_temp > (current_temp + (TEMP_HYSTERESIS*4))) {
		// heating
		if (current_temp > heaters_runtime[id].sane_temperature)
			// hotter than sane- good since we're heating unless too hot
			heaters_runtime[id].sane_temperature = current_temp;
		else {
			if (heaters_runtime[id].sanity_counter < 40)
				heaters_runtime[id].sanity_counter++;
			else {
				heaters_runtime[id].sanity_counter = 0;
				// ratchet up expected temp
				heaters_runtime[id].sane_temperature++;
			}
		}
		// limit to target, so if we overshoot by too much for too long an error is flagged
		if (heaters_runtime[id].sane_temperature > target_temp)
			heaters_runtime[id].sane_temperature = target_temp;
	}
	else if (target_temp < (current_temp - (TEMP_HYSTERESIS*4))) {
		// cooling
		if (current_temp < heaters_runtime[id].sane_temperature)
			// cooler than sane- good since we're cooling
			heaters_runtime[id].sane_temperature = current_temp;
		else {
			if (heaters_runtime[id].sanity_counter < 125)
				heaters_runtime[id].sanity_counter++;
			else {
				heaters_runtime[id].sanity_counter = 0;
				// ratchet down expected temp
				heaters_runtime[id].sane_temperature--;
			}
		}
		// if we're at or below 60 celsius, don't freak out if we can't drop any more.
		if (current_temp <= 240)
			heaters_runtime[id].sane_temperature = current_temp;
		// limit to target, so if we don't cool down for too long an error is flagged
		else if (heaters_runtime[id].sane_temperature < target_temp)
			heaters_runtime[id].sane_temperature = target_temp;
	}
	// we're within HYSTERESIS of our target
	else {
		heaters_runtime[id].sane_temperature = current_temp;
		heaters_runtime[id].sanity_counter = 0;
	}

	// compare where we're at to where we should be
	if (labs((int16_t)(current_temp - heaters_runtime[id].sane_temperature)) > (TEMP_HYSTERESIS*4)) {
		// no change, or change in wrong direction for a long time- heater is broken!
		pid_output = 0;
		sersendf_P(PSTR("!! heater %d or its temp sensor broken - temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), id, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[id].sane_temperature >> 2, (heaters_runtime[id].sane_temperature & 3) * 25, heaters_runtime[id].sanity_counter);
	}
	#endif /* HEATER_SANITY_CHECK */

	heater_set(id, pid_output);
}


/** \brief turn off all heaters

	for emergency stop
*/
// FIXME catch estop
uint8_t heaters_all_off(void) {
	uint8_t i;
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters_runtime[i].heater_output > 0)
			return 0;
	}

	return 255;
}

/** \brief set heater P factor
	\param id heater to change factor for
	\param p scaled P factor
*/
void pid_set_p(uint8_t id, int32_t p) {
	#ifndef	BANG_BANG
		if (id >= NUM_HEATERS)
			return;

		heaters_pid[id].p_factor = p;
	#endif /* BANG_BANG */
}

/** \brief set heater I factor
	\param id heater to change I factor for
	\param i scaled I factor
*/
void pid_set_i(uint8_t id, int32_t i) {
	#ifndef	BANG_BANG
		if (id >= NUM_HEATERS)
			return;

		heaters_pid[id].i_factor = i;
	#endif /* BANG_BANG */
}

/** \brief set heater D factor
	\param id heater to change D factor for
	\param d scaled D factor
*/
void pid_set_d(uint8_t id, int32_t d) {
	#ifndef	BANG_BANG
		if (id >= NUM_HEATERS)
			return;

		heaters_pid[id].d_factor = d;
	#endif /* BANG_BANG */
}

/** \brief set heater I limit
	\param id heater to set I limit for
	\param i_limit scaled I limit
*/
void pid_set_i_limit(uint8_t id, int32_t i_limit) {
	#ifndef	BANG_BANG
		if (id >= NUM_HEATERS)
			return;

		heaters_pid[id].i_limit = i_limit;
	#endif /* BANG_BANG */
}

#ifndef	EXTRUDER
/** \brief send heater debug info to host
	\param i id of heater to send info for
*/
void heater_print(uint16_t i) {
	sersendf_P(PSTR("P:%ld I:%ld D:%ld Ilim:%u crc:%u "), heaters_pid[i].p_factor, heaters_pid[i].i_factor, heaters_pid[i].d_factor, heaters_pid[i].i_limit, crc_block(&heaters_pid[i].p_factor, heater_pid_EE_size));
}
#endif

/*
#if E_STARTSTOP_STEPS > 0
/// move E by a certain amount at a certain speed
static void SpecialMoveE(int32_t e, uint32_t f) {
	TARGET t = { 0L, 0L, 0L, e, f, 1 };
	enqueue(&t);
}
#endif  E_STARTSTOP_STEPS > 0 

void heater_gcode_process(void){
	case 'M':
		case 2:
			// FIXME heater off
			for (i = 0; i < NUM_HEATERS; i++)
				temp_set(i, 0);
	
		// M3/M101- extruder on
		case 3:
		case 101:
			//? --- M101: extruder on ---
			//?
			//? Undocumented.
			if (temp_achieved() == 0) {
				enqueue(NULL);
			}
			#ifdef DC_EXTRUDER
				// FIXME heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
			#elif E_STARTSTOP_STEPS > 0
				do {
					// backup feedrate, move E very quickly then restore feedrate
					backup_f = startpoint.F;
					startpoint.F = MAXIMUM_FEEDRATE_E;
					SpecialMoveE(E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
					startpoint.F = backup_f;
				} while (0);
			#endif
			break;

			// M102- extruder reverse

			// M5/M103- extruder off
			case 5:
			case 103:
				//? --- M103: extruder off ---
				//?
				//? Undocumented.
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, 0);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = startpoint.parameters[L_F];
						startpoint.parameters[L_F] = MAXIMUM_FEEDRATE_E;
						SpecialMoveE(-E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						startpoint.parameters[L_F] = backup_f;
					} while (0);
				#endif
				break;
			// FIXME extruder temp 
			case 104:
				//? --- M104: Set Extruder Temperature (Fast) ---
				//?
				//? Example: M104 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and return control to the host immediately (''i.e.'' before that temperature has been reached by the extruder).  See also M109.
				//? Teacup supports an optional P parameter as a sensor index to address (eg M104 P1 S100 will set the bed temperature rather than the extruder temperature).
				//?
				if ( ! (next_target.seen & (1<<L_S)) != 0)
					break;
				if ( ! (next_target.seen & (1<<L_P)) != 0)
					next_target.parameters[L_P] = HEATER_EXTRUDER;
				temp_set(next_target.parameters[L_P], next_target.parameters[L_S]);
				if (next_target.parameters[L_S])
					power_on();
				break;
			case 105:
				//? --- M105: Get Extruder Temperature ---
				//?
				//? Example: M105
				//?
				//? Request the temperature of the current extruder and the build base in degrees Celsius.  The temperatures are returned to the host computer.  For example, the line sent to the host in response to this command looks like
				//?
				//? <tt>ok T:201 B:117</tt>
				//?
				//? Teacup supports an optional P parameter as a sensor index to address.
				//?
				#ifdef ENFORCE_ORDER
					queue_wait();
				#endif
				temp_print();
				break;
			 FIXME extruder temp
			case 109:
				//? --- M109: Set Extruder Temperature ---
				//?
				//? Example: M109 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and wait for it to reach that value before sending an acknowledgment to the host.  In fact the RepRap firmware waits a while after the temperature has been reached for the extruder to stabilise - typically about 40 seconds.  This can be changed by a parameter in the firmware configuration file when the firmware is compiled.  See also M104 and M116.
				//?
				//? Teacup supports an optional P parameter as a sensor index to address.
				//?
				if ( ! (next_target.seen & (1<<L_S)) != 0)
					break;
				if ( ! (next_target.seen & (1<<L_P)) != 0)
					next_target.parameters[L_P] = HEATER_EXTRUDER;
				temp_set(next_target.parameters[L_P], next_target.parameters[L_S]);
				if (next_target.parameters[L_S]) {
					power_on();
					// FIXME enable_heater();
				}
				else {
					// FIXME disable_heater();
				}
				enqueue(NULL);
				break;
		case 109:
			//? --- M109: Set Extruder Temperature ---
			//?
			//? Example: M109 S190
			//?
			//? Set the temperature of the current extruder to 190<sup>o</sup>C and wait for it to reach that value before sending an acknowledgment to the host.  In fact the RepRap firmware waits a while after the temperature has been reached for the extruder to stabilise - typically about 40 seconds.  This can be changed by a parameter in the firmware configuration file when the firmware is compiled.  See also M104 and M116.
			//?
			//? Teacup supports an optional P parameter as a sensor index to address.
			//?
			if (next_target.seen_S)
				temp_set(next_target.P, next_target.S);
			if (next_target.S) {
				power_on();
				// FIXME enable_heater();
			}
			else {
				// FIXME disable_heater();
			}
			enqueue(NULL);
			break;

		case 130:
			//? --- M130: heater P factor ---
			//? Undocumented.
			if (next_target.seen_S)
				pid_set_p(next_target.P, next_target.S);
			break;

		case 131:
			//? --- M131: heater I factor ---
			//? Undocumented.
			if (next_target.seen_S)
				pid_set_i(next_target.P, next_target.S);
			break;

		case 132:
			//? --- M132: heater D factor ---
			//? Undocumented.
			if (next_target.seen_S)
				pid_set_d(next_target.P, next_target.S);
			break;

		case 133:
			//? --- M133: heater I limit ---
			//? Undocumented.
			if (next_target.seen_S)
				pid_set_i_limit(next_target.P, next_target.S);
			break;

		case 134:
			//? --- M134: save PID settings to eeprom ---
			//? Undocumented.
			heater_save_settings();
			break;

		case 135:
			//? --- M135: set heater output ---
			//? Undocumented.
			if (next_target.seen_S) {
				heater_set(next_target.P, next_target.S);
				power_on();
			}
			break;

		#ifdef	DEBUG
		case 136:
			//? --- M136: PRINT PID settings to host ---
			//? Undocumented.
			//? This comand is only available in DEBUG builds.
			heater_print(next_target.P);
			break;
		#endif
		case 140:
			//? --- M140: Set heated bed temperature ---
			//? Undocumented.
			#ifdef	HEATER_BED
				temp_set(HEATER_BED, next_target.S);
				if (next_target.S)
					power_on();
			#endif
			break;
			
		case 115:
			//? --- M115: Get Firmware Version and Capabilities ---
			//?
			//? Example: M115
			//?
			//? Request the Firmware Version and Capabilities of the current microcontroller
			//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
			//?
			//? sample data from firmware:
			//?  FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:1 HEATER_COUNT:1
			//?

			sersendf_P(PSTR("HEATER_COUNT:%d"), NUM_HEATERS);
			// FIXME number of temp sensors, number of heaters
			// newline is sent from gcode_parse after we return
			break;
	}

			// M113- extruder PWM
}
*/

