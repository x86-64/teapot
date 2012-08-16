#define FEATURE
#include "common.h"

API void fan_init(void);

void fan_gcode_process(void *next_target){
	if(! PARAMETER_SEEN(L_M) )
		return;
	
	switch(PARAMETER(L_M)){
		case 7:
		case 106:
			//? --- M106: Fan On ---
			//?
			//? Example: M106
			//?
			//? Turn on the cooling fan (if any).
			//?
			
			// core_emit(EVENT_IDLE);
			/* FIXME #ifdef HEATER_FAN
				heater_set(HEATER_FAN, 255);
			#endif
			*/
			break;

		case 9:
		case 107:
			//? --- M107: Fan Off ---
			//?
			//? Example: M107
			//?
			//? Turn off the cooling fan (if any).
			//?
			
			// core_emit(EVENT_IDLE);
			/* FIXME #ifdef HEATER_FAN
				heater_set(HEATER_FAN, 0);
			#endif
			*/
			break;
	}
}

void fan_init(void){
	core_register(EVENT_GCODE_PROCESS, &fan_gcode_process);
}

