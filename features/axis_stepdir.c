#define FEATURE
#include <common.h>
#include "axes.h"

API typedef struct axis_stepdir_userdata         { uint8_t pin_step; uint8_t pin_dir; } axis_stepdir_userdata;
API typedef struct axis_runtime_stepdir_userdata { } axis_runtime_stepdir_userdata;

API void           axis_stepdir_gcode(const axis_t *axis, axis_runtime_t *axis_runtime, void *next_target);


void axis_stepdir_gcode(const axis_t *axis, axis_runtime_t *axis_runtime, void *next_target){
	if(PARAMETER_SEEN(L_G)){
		switch(PARAMETER(L_G)){
			case 0:
				break;
			// TODO position_min/max
		}
	}
}

