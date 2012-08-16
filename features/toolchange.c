#define FEATURE
#include "common.h"

API void toolchange_init(void);

uint8_t tool;      ///< the current tool
uint8_t next_tool; ///< the tool to be changed when we get an M6

void toolchange_gcode_process(void *next_target){
	
	if(PARAMETER_SEEN(L_T)) {
		//? --- T: Select Tool ---
		//?
		//? Example: T1
		//?
		//? Select extruder number 1 to build with.  Extruder numbering starts at 0.
		
		next_tool = PARAMETER(L_T);
	}
	
	if(PARAMETER_SEEN(L_M)) {
		switch(PARAMETER(L_M)){
			case 6:
				//? --- M6: tool change ---
				//?
				//? Undocumented.
				tool = next_tool;
				break;
		}
	}
}

void toolchange_init(void){
	core_register(EVENT_GCODE_PROCESS, &toolchange_gcode_process);
}

