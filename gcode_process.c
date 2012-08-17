/** \file
	\brief Work out what to do with received G-Code commands
*/
#include	"common.h"

/***************************************************************************\
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* Relies on the global variable PARAMETER(L_N) being valid.                  *
*                                                                           *
\***************************************************************************/

void request_resend(void *next_target) {
	serial_writestr_P(PSTR("rs "));
	serwrite_uint8(PARAMETER(L_N));
	serial_writechar('\n');
}

uint32_t N_expected;

/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command(void *next_target) {
	#ifdef	REQUIRE_LINENUMBER
		if( 
			((PARAMETER(L_N) >= N_expected) && (PARAMETER_SEEN(L_N))) ||
			(PARAMETER_SEEN(L_M) && (PARAMETER(L_M) == 110))){
		}else {
			sersendf_P(PSTR("rs N%ld Expected line number %ld\n"), PARAMETER(L_N), N_expected);
			//request_resend();
		}
			
		// expect next line number
		if (PARAMETER_SEEN(L_N))
			N_expected = PARAMETER(L_N) + 1;
	#endif
	
	#ifdef	REQUIRE_CHECKSUM
		if( (PARAMETER_SEEN(L_CHECKSUM)) || (next_target->checksum != PARAMETER(L_CHECKSUM)) ){
			sersendf_P(PSTR("rs N%ld Expected checksum %d\n"), N_expected, next_target->checksum);
			//request_resend();
		}
	#endif
	
	core_emit(EVENT_GCODE_PROCESS, next_target);
	
	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .
	
	if (PARAMETER_SEEN(L_M)) {
		switch (PARAMETER(L_M)) {
			case 112:
				//? --- M112: Emergency Stop ---
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.
				//?

				cli();
				for (;;)
					core_emit(EVENT_TICK, 0);
				break;

			case 110:
				//? --- M110: Set Current Line Number ---
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? This is a no-op in Teacup.
				//?
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

				sersendf_P(PSTR("FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:%d.0 MACHINE_TYPE:Mendel "), 1);
				break;
		} // switch (PARAMETER(L_M))
	} // else if (PARAMETER_SEEN(L_M))
} // process_gcode_command()
