#define FEATURE
#include	"common.h"

API void time_init(void);

void time_gcode_process(void *next_target){
	if (PARAMETER_SEEN(L_G)) {
		switch (PARAMETER_asint(L_G)) {
			case 4:
				//? --- G4: Dwell ---
				//?
				//? Example: G4 P200
				//?
				//? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
				//?
				
				M400_WAIT();
				// delay
				if (PARAMETER_SEEN(L_P)) {
					int32_t delay_time;
					
					for (delay_time = PARAMETER_asint(L_P); delay_time > 0; delay_time--){
						core_emit(EVENT_TICK, 0);
						delay(1);
					}
				}
				break;
		}
	}
}

void time_init(void){
	core_register(EVENT_GCODE_PROCESS, &time_gcode_process);
}

