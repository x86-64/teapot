#include	"common.h"
#include        "temperature.h"

extern const temp_sensor_t           temp_sensors[];
extern const uint8_t                 temp_sensors_count;
extern       temp_sensor_runtime_t   temp_sensors_runtime[];

/*
NOTES

no point in specifying a port- all the different temp sensors we have must be on a particular port. The MAX6675 must be on the SPI, and the thermistor and AD595 must be on an analog port.

we still need to specify which analog pins we use in machine.h for the analog sensors however, otherwise the analog subsystem won't read them.
*/

/** \file
	\brief Manage temperature sensors

	\note \b ALL temperatures are stored as 14.2 fixed point in teacup, so we have a range of 0 - 16383.75 celsius and a precision of 0.25 celsius. That includes the ThermistorTable, which is why you can't copy and paste one from other firmwares which don't do this.
*/

API void temp_tick(void);
API uint8_t	temp_achieved(void);
API void temp_set(uint8_t index, uint16_t temperature);
API uint16_t temp_get(uint8_t index);
API uint8_t temp_all_zero(void);
API void temp_print(void);


#include	<stdlib.h>

#include	"debug.h"


/// called every 10ms from clock.c - check all temp sensors that are ready for checking
void temp_tick() {
	uint8_t i = 0;
	for (; i < temp_sensors_count; i++) {
		if (temp_sensors_runtime[i].next_read_time) {
			temp_sensors_runtime[i].next_read_time--;
			continue;
		}
		//time to deal with this temp sensor
		
		temp_sensors[i].sensor_read(&temp_sensors[i], &temp_sensors_runtime[i]);
		
		if (labs((int16_t)(temp_sensors_runtime[i].last_read_temp - temp_sensors_runtime[i].target_temp)) < (TEMP_HYSTERESIS*4)) {
			if (temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*100))
				temp_sensors_runtime[i].temp_residency++;
		}
		else {
			temp_sensors_runtime[i].temp_residency = 0;
		}

		//if (temp_sensors[i].heater < NUM_HEATERS) {
		//	heater_tick(temp_sensors[i].heater/*, temp_sensors[i].temp_type*/, temp_sensors_runtime[i].last_read_temp, temp_sensors_runtime[i].target_temp);
		//}
	}
}

/// report whether all temp sensors are reading their target temperatures
/// used for M109 and friends
uint8_t	temp_achieved() {
	uint8_t i;
	uint8_t all_ok = 255;

	for (i = 0; i < temp_sensors_count; i++) {
		if (temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*100))
			all_ok = 0;
	}
	return all_ok;
}

/// specify a target temperature
/// \param index sensor to set a target for
/// \param temperature target temperature to aim for
void temp_set(uint8_t index, uint16_t temperature) {
	if (index >= temp_sensors_count)
		return;

	// only reset residency if temp really changed
	if (temp_sensors_runtime[index].target_temp != temperature) {
		temp_sensors_runtime[index].target_temp = temperature;
		temp_sensors_runtime[index].temp_residency = 0;
	}
}

/// return most recent reading for a sensor
/// \param index sensor to read
uint16_t temp_get(uint8_t index) {
	if (index >= temp_sensors_count)
		return 0;

	return temp_sensors_runtime[index].last_read_temp;
}

/// send temperatures to host
void temp_print() {
	#if REPRAP_HOST_COMPATIBILITY < 20110509
		sersendf_P(PSTR("\n"));
	#endif
	
	uint8_t i;
	for (i = 0; i < temp_sensors_count; i++) {
		sersendf_P(PSTR("%c:%u.%u "),
			temp_sensors[i].rep_char,
			(temp_sensors_runtime[i].last_read_temp >> 2),
			(temp_sensors_runtime[i].last_read_temp  & 3) * 25
		);
	}
}
/* FIXME temperature conversion from gcode
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if ((next_target.M == 104) || (next_target.M == 109) || (next_target.M == 140))
						next_target.S = decfloat_to_int(&read_digit, 4);
					// if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
					else if ((next_target.M >= 130) && (next_target.M <= 132))
						next_target.S = decfloat_to_int(&read_digit, PID_SCALE);
					else
						next_target.S = decfloat_to_int(&read_digit, 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint16(next_target.S);
					break;
*/
