#include "common.h"
#include "delay.h"

static char ps_is_on = 0;

/// step/psu timeout
volatile uint8_t	psu_timeout = 0;

void power_on(void) {
	if (ps_is_on == 0) {
		#ifdef	PS_ON_PIN
			WRITE(PS_ON_PIN, 0);
			SET_OUTPUT(PS_ON_PIN);
			_delay_ms(500);
		#endif
		ps_is_on = 1;
	}

	psu_timeout = 0;
}

void power_off(void) {
	#ifdef	PS_ON_PIN
		SET_INPUT(PS_ON_PIN);
	#endif

	ps_is_on = 0;
}


void power_gcode_process(void *next_target){
	if(! PARAMETER_SEEN(L_M) )
		return;
	
	switch(PARAMETER(L_M)){
		case 0:
			//? --- M0: machine stop ---
			//?
			//? Example: M0
			//?
			//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
			//? Unimplemented, especially the restart after the stop. Fall trough to M2.
			//?

		case 2:
			//? --- M2: program end ---
			//?
			//? Example: M2
			//?
			//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
			//?
		
		case 191:
			//? --- M191: Power Off ---
			//? Undocumented.
			//? Same as M2. RepRap obviously prefers to invent new numbers instead of looking into standards. 
			power_off();
			break;
		
		case 190:
			//? --- M190: Power On ---
			//? Undocumented.
			//? This one is pointless in Teacup. Implemented to calm the RepRap gurus.
			//?
			power_on();
			break;

		case 112:
			//? --- M112: Emergency Stop ---
			//?
			//? Example: M112
			//?
			//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
			//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.
			//?
			power_off();
			break;
	}
}

void power_init(void){
	core_register(EVENT_GCODE_PROCESS, &power_gcode_process);
}

