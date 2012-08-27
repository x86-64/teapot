#define FEATURE
#include "common.h"
#include "axes.h"

#define IDLE_TIME 100 MS

API typedef struct axis_stepdir_userdata         { uint8_t pin_step; uint8_t pin_dir; dda_queue_t queue; uint8_t timer_id; } axis_stepdir_userdata;

API void           axis_stepdir_init(axis_t *axis);
API void           axis_stepdir_gcode(axis_t *axis, void *next_target);
API axis_proto_t   axis_stepdir_proto;

void axis_stepdir_timer(uint8_t id, void *paxis){
	dda_order_t            order;
	axis_t                *axis              = (axis_t *)paxis;
	axis_stepdir_userdata *userdata          = (axis_stepdir_userdata *)axis->userdata;
	
	// 1. make dda step to calculate our next move
	do{
		dda_queue_step(&userdata->queue, &order);
	}while( order.callme == 1 && order.c == 0 ); // if dda request callback immediatly - do it
	
	// 2. check dda orders:
	
	// - dda ask to step?
	if(order.step){
		sersendf_P(PSTR("%c"), order.direction ? 'F' : 'B');
	}
	
	// - dda ask to callback?
	timer_charge(id,
		order.callme ?
			order.c :    // for specified by dda time
			IDLE_TIME    // nothing to do, idle wait
	);
}

void axis_stepdir_init(axis_t *axis){
	axis_stepdir_userdata *userdata          = (axis_stepdir_userdata *)axis->userdata;
	
	// init dda queue
	dda_queue_init(&userdata->queue);
	
	// init timer
	userdata->timer_id = timer_new();
	timer_setup  (userdata->timer_id, &axis_stepdir_timer, axis);
	timer_charge (userdata->timer_id, IDLE_TIME);
}

void axis_stepdir_gcode(axis_t *axis, void *next_target){
	dda_target_t           target;
	axis_stepdir_userdata *userdata          = (axis_stepdir_userdata *)axis->userdata;
	
	if(PARAMETER_SEEN(L_G)){
		switch(PARAMETER_asint(L_G)){
			case 0:	
				target.X = PARAMETER_asint(axis->letter);
				target.F = axis->feedrate_max;
				
				dda_queue_enqueue(&userdata->queue, &target);
				
				//axis_runtime->position_curr = PARAMETER_asint(axis->letter);
				break;
			// TODO position_min/max
		}
	}
}


axis_proto_t  axis_stepdir_proto = {
	.func_gcode = &axis_stepdir_gcode,
	.func_init  = &axis_stepdir_init,
};

