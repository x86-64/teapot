#define FEATURE
#include "common.h"
#include "axes.h"

#define IDLE_TIME      100 MS
#define MIN_STEP_TIME    1 // US

API typedef struct axis_stepdir_userdata         { uint8_t pin_step; uint8_t pin_dir; dda_queue_t queue; uint8_t timer_id; } axis_stepdir_userdata;

API void           axis_stepdir_init(axis_t *axis);
API void           axis_stepdir_gcode(axis_t *axis, void *next_target);
API axis_proto_t   axis_stepdir_proto;

void axis_stepdir_step(axis_t *axis, dda_order_t *order){
	WRITE(axis->pin_dir,  order->direction);
	WRITE(axis->pin_step, 1);
	delay_us(MIN_STEP_TIME);
	WRITE(axis->pin_step, 0);
}

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
	if(order.step)
		axis_stepdir_step(axis, &order);
	
	// - dda ask to callback?
	timer_charge(id,
		order.callme ?
			order.c :    // for specified by dda time
			IDLE_TIME    // nothing to do, idle wait
	);
	
	//sersendf_P(PSTR("O%c s%d "), order.step ? (order.direction ? 'F' : 'B') : ' ',  order.c);
}

void axis_stepdir_init(axis_t *axis){
	axis_stepdir_userdata *userdata          = (axis_stepdir_userdata *)axis->userdata;
	
	// init dda queue
	dda_queue_init(&userdata->queue);
	
	// init timer
	userdata->timer_id = timer_new();
	timer_setup  (userdata->timer_id, &axis_stepdir_timer, axis);
	timer_charge (userdata->timer_id, IDLE_TIME);

	// init outputs
	SET_OUTPUT(axis->pin_dir);
	SET_OUTPUT(axis->pin_step);
	
	// X Stepper
	WRITE(X_STEP_PIN, 0);	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_DIR_PIN,  0);	SET_OUTPUT(X_DIR_PIN);
	#ifdef X_MIN_PIN
		SET_INPUT(X_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MIN_PIN, 1);
		#else
			WRITE(X_MIN_PIN, 0);
		#endif
	#endif
	#ifdef X_MAX_PIN
		SET_INPUT(X_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MAX_PIN, 1);
		#else
			WRITE(X_MAX_PIN, 0);
		#endif
	#endif
}

void axis_stepdir_gcode(axis_t *axis, void *next_target){
	dda_target_t           start;
	dda_target_t           target;
	axis_stepdir_userdata *userdata          = (axis_stepdir_userdata *)axis->userdata;
	
	if(PARAMETER_SEEN(L_G)){
		switch(PARAMETER_asint(L_G)){
			case 0:	
				start.X  = axis->runtime.position_curr;
				start.F  = 0;                                // we start new gcode, so we assume that axis was stopped
				                                             // if dda look-ahead need feedrate for some move - it will use values from queue
				target.X = PARAMETER_asint(axis->letter);
				target.F = axis->feedrate_max;
				
				dda_queue_enqueue(&userdata->queue, &start, &target);
				
				axis->runtime.position_curr = target.X;
				break;
			// TODO position_min/max
		}
	}
}


axis_proto_t  axis_stepdir_proto = {
	.func_gcode = &axis_stepdir_gcode,
	.func_init  = &axis_stepdir_init,
};

