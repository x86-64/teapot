#define FEATURE
#include "common.h"
#include "axes.h"

API typedef struct axis_stepdir_userdata         { uint8_t       pin_step; uint8_t pin_dir; } axis_stepdir_userdata;
API typedef struct axis_runtime_stepdir_userdata { dda_queue_t   queue;                     } axis_runtime_stepdir_userdata;

API void           axis_stepdir_init(const axis_t *axis, axis_runtime_t *axis_runtime);
API void           axis_stepdir_gcode(const axis_t *axis, axis_runtime_t *axis_runtime, void *next_target);

void axis_stepdir_init(const axis_t *axis, axis_runtime_t *axis_runtime){
	axis_runtime_stepdir_userdata *userdata = (axis_runtime_stepdir_userdata *)axis_runtime->userdata;
	
	dda_queue_init(&userdata->queue);
}

void axis_stepdir_gcode(const axis_t *axis, axis_runtime_t *axis_runtime, void *next_target){
	if(PARAMETER_SEEN(L_G)){
		switch(PARAMETER_asint(L_G)){
			case 0:
				
	sersendf_P(PSTR(
		"{AXIS_RUN: letter: '%c', "
			"relative: %d, "
			"inches: %d"
		"}"),
		gcode_convert_letter(axis->letter),
		axis_runtime->relative,
		axis_runtime->inches
	);

				break;
			// TODO position_min/max
		}
	}
}

