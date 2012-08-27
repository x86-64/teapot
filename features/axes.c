#include "common.h"
#include "axes.h"

API void axes_init(void);

void axis_debug_print(axis_t *axis){
	sersendf_P(PSTR(
		"{AXIS: letter: '%c', "
			"sticky: %d, "
			"have_min: %d, have_max: %d, "
			"feed_search: %lu, feed_max: %lu, "
			"pos_min: %ld, pos_max: %ld, "
			"letter: '%c', "
			"position: %ld, "
			"relative: %d, "
			"inches: %d"
		"}"),
		gcode_convert_letter(axis->letter),
		axis->sticky_relative,
		axis->have_position_min, axis->have_position_max,
		axis->feedrate_search,   axis->feedrate_max,
		axis->position_min,      axis->position_max,
		axis->runtime.position_curr,
		axis->runtime.relative,
		axis->runtime.inches
	);
}

void axes_debug_print(void){
	uint8_t i;

	for(i=0; i<axes_count; i++){
		axis_debug_print(&axes[i]);
		serial_writechar('\n');
	}
}

/// This function apply to any axis
void axis_gcode_universal(axis_t *axis, void *next_target){
	if(PARAMETER_SEEN(L_G)){
		switch(PARAMETER_asint(L_G)){
			case 0: // G0,1 movements
			case 1:
				// convert coordinate to um with respect to current measurment mode
				if(axis->runtime.inches){
					PARAMETER_SET(next_target, axis->letter,
						PARAMETER_asmult(axis->letter, 25400)
					); 
				}else{
					PARAMETER_SET(next_target, axis->letter,
						PARAMETER_asmult(axis->letter, 1000)
					); 
				}
				
				// if relative mode is on - convert relative coordinates to absolute
				if(axis->runtime.relative){
					PARAMETER_SET(next_target, axis->letter,
						axis->runtime.position_curr + PARAMETER_asint(axis->letter)
					);
				}
				break;
				
			case 20:
				//? --- G20: Set Units to Inches ---
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				axis->runtime.inches = 1;
				break;

			case 21:
				//? --- G21: Set Units to Millimeters ---
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				axis->runtime.inches = 0;
				break;

			case 90:
				//? --- G90: Set to Absolute Positioning ---
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin
				//? of the machine. This is the RepRap default.
				//?
				//? If you ever want to switch back and forth between relative and
				//? absolute movement keep in mind, X, Y and Z follow the machine's
				//? coordinate system while E doesn't change it's position in the
				//? coordinate system on relative movements.
				//?

				// No wait_queue() needed.
				if(axis->sticky_relative == 0)
					axis->runtime.relative = 0;
				break;

			case 91:
				//? --- G91: Set to Relative Positioning ---
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				//?
				
				// No wait_queue() needed.
				if(axis->sticky_relative == 0)
					axis->runtime.relative = 1;
				break;
		}
	}
	if(PARAMETER_SEEN(L_M)){
		switch(PARAMETER_asint(L_M)){
			case 82:
				//? --- M82 - Set E codes absolute ---
				//?
				//? This is the default and overrides G90/G91.
				//? M82/M83 is not documented in the RepRap wiki, behaviour
				//? was taken from Sprinter as of March 2012.
				//?
				//? While E does relative movements, it doesn't change its
				//? position in the coordinate system. See also comment on G90.
				//?

				// No wait_queue() needed.
				if(axis->sticky_relative == 1)
					axis->runtime.relative = 0;
				break;

			case 83:
				//? --- M83 - Set E codes relative ---
				//?
				//? Counterpart to M82.
				//?

				// No wait_queue() needed.
				if(axis->sticky_relative == 1)
					axis->runtime.relative = 1;
				break;
			
			case 400:
				//? --- M400 - Wait for all operations to complete (could be same as M116 but not sure)
				// FIXME queue_wait();
				break;

			#ifdef DEBUG
			case 401:
				//? --- M401 - Show debug info
				axes_debug_print();
				break;
			#endif
		}
	}
}

// This function is per-axis only
void axis_gcode_letter(axis_t *axis, void *next_target){
	axis->proto->func_gcode(axis, next_target);
}

void axes_gcode(void *next_target){
	uint8_t                i;
	
	for(i=0; i<axes_count; i++){
		axis_gcode_universal(&axes[i], next_target);
		
		if(PARAMETER_SEEN(axes[i].letter))
			axis_gcode_letter(&axes[i], next_target);
	}
}

void axes_init(void){
	uint8_t                i;
	
	core_register(EVENT_GCODE_PROCESS, &axes_gcode);
	
	for(i=0; i<axes_count; i++){
		axes[i].proto->func_init(&axes[i]);
	}
}

/*
	uint8_t endstop_stop; ///< Stop due to endstop trigger
	uint8_t endstop_not_done = 0; ///< Which axes haven't finished homing

#if defined X_MIN_PIN || defined X_MAX_PIN
	if (dda->endstop_check) {
#if defined X_MIN_PIN
		if (x_min() == dda->endstop_stop_cond)
			dda->move->debounce_count_min++;
		else
			dda->move->debounce_count_min = 0;
#endif

#if defined X_MAX_PIN
		if (x_max() == dda->endstop_stop_cond)
			dda->move->debounce_count_max++;
		else
			dda->move->debounce_count_max = 0;
#endif

		endstop_stop = dda->move->debounce_count_min >= ENDSTOP_STEPS ||
		               dda->move->debounce_count_max >= ENDSTOP_STEPS;
		if (!endstop_stop)
			endstop_not_done |= 0x1;
	} else
#endif
		endstop_stop = 0;

	#if defined Z_MIN_PIN
		home_z_negative();
	#elif defined	Z_MAX_PIN
		home_z_positive();
	#endif
/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
		TARGET t = startpoint;

		t.Z = -1000000;
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Z;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Z;
		#endif
		enqueue_home(&t, 0x4, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Z = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
		#endif

		// set Z home
		queue_wait();
		#ifdef Z_MIN
			startpoint.Z = next_target.target.Z = (int32_t)(Z_MIN * 1000.);
		#else
			startpoint.Z = next_target.target.Z = 0;
		#endif
		dda_new_startpoint(dda);
		z_disable();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
		TARGET t = startpoint;

		t.Z = +1000000;
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Z;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Z;
		#endif
		enqueue_home(&t, 0x4, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Z = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
		#endif

		// set Z home
		queue_wait();
		// set position to MAX
		startpoint.Z = next_target.target.Z = (int32_t)(Z_MAX * 1000.);
		dda_new_startpoint(dda);
		// go to zero
		t.Z = 0;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue(&t);
	#endif
}
	switch (next_target.G) {
		case 0:
			//? G0: Rapid Linear Motion
			//?
			//? Example: G0 X12
			//?
			//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
			//?
			backup_f = next_target.target.F;
			next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
			enqueue(&next_target.target);
			next_target.target.F = backup_f;
			break;

		case 1:
			//? --- G1: Linear Motion at Feed Rate ---
			//?
			//? Example: G1 X90.6 Y13.8 E22.4
			//?
			//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
			//?
			enqueue(&next_target.target);
			break;

			//	G2 - Arc Clockwise
			// unimplemented

			//	G3 - Arc Counter-clockwise
			// unimplemented


		case 30:
			//? --- G30: Go home via point ---
			//?
			//? Undocumented.
			enqueue(&next_target.target);
			// no break here, G30 is move and then go home

		case 28:
			//? --- G28: Home ---
			//?
			//? Example: G28
			//?
			//? This causes the RepRap machine to move back to its X, Y and Z zero endstops.  It does so accelerating, so as to get there fast.  But when it arrives it backs off by 1 mm in each direction slowly, then moves back slowly to the stop.  This ensures more accurate positioning.
			//?
			//? If you add coordinates, then just the axes with coordinates specified will be zeroed.  Thus
			//?
			//? G28 X0 Y72.3
			//?
			//? will zero the X and Y axes, but not Z.  The actual coordinate values are ignored.
			//?

			queue_wait();

			if ((next_target.seen & (1<<L_X)) != 0) {
				#if defined	X_MIN_PIN
					home_x_negative();
				#elif defined X_MAX_PIN
					home_x_positive();
				#endif
				axisSelected = 1;
			}
			if ((next_target.seen & (1<<L_Y)) != 0) {
				#if defined	Y_MIN_PIN
					home_y_negative();
				#elif defined Y_MAX_PIN
					home_y_positive();
				#endif
				axisSelected = 1;
			}
			if ((next_target.seen & (1<<L_Z)) != 0) {
				#if defined Z_MAX_PIN
					home_z_positive();
				#elif defined	Z_MIN_PIN
					home_z_negative();
				#endif
				axisSelected = 1;
			}
			// there's no point in moving E, as E has no endstops

			if (!axisSelected) {
				home();
			}
			break;


		case 92:
			//? --- G92: Set Position ---
			//?
			//? Example: G92 X10 E90
			//?
			//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
			//?

			queue_wait();

			if ((next_target.seen & (1<<L_X)) != 0) {
				startpoint.X = next_target.target.X;
				axisSelected = 1;
			}
			if ((next_target.seen & (1<<L_Y)) != 0) {
				startpoint.Y = next_target.target.Y;
				axisSelected = 1;
			}
			if ((next_target.seen & (1<<L_Z)) != 0) {
				startpoint.Z = next_target.target.Z;
				axisSelected = 1;
			}
			if ((next_target.seen & (1<<L_E)) != 0) {
				startpoint.E = next_target.target.E;
				axisSelected = 1;
			}

			if (axisSelected == 0) {
				startpoint.X = next_target.target.X =
				startpoint.Y = next_target.target.Y =
				startpoint.Z = next_target.target.Z =
				startpoint.E = next_target.target.E = 0;
			}

			dda_new_startpoint(dda);
			break;

		case 161:
			//? --- G161: Home negative ---
			//?
			//? Find the minimum limit of the specified axes by searching for the limit switch.
			//?
			if ((next_target.seen & (1<<L_X)) != 0)
				home_x_negative();
			if ((next_target.seen & (1<<L_Y)) != 0)
				home_y_negative();
			if ((next_target.seen & (1<<L_Z)) != 0)
				home_z_negative();
			break;

		case 162:
			//? --- G162: Home positive ---
			//?
			//? Find the maximum limit of the specified axes by searching for the limit switch.
			//?
			if ((next_target.seen & (1<<L_X)) != 0)
				home_x_positive();
			if ((next_target.seen & (1<<L_Y)) != 0)
				home_y_positive();
			if ((next_target.seen & (1<<L_Z)) != 0)
				home_z_positive();
			break;
	}

	case 'M'

			case 114:
				//? --- M114: Get Current Position ---
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
				//?
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				update_current_position(dda);
				sersendf_P(PSTR("X:%lq,Y:%lq,Z:%lq,E:%lq,F:%ld"), current_position.parameters[L_X], current_position.parameters[L_Y], current_position.parameters[L_Z], current_position.parameters[L_E], current_position.parameters[L_F]);
				// newline is sent from gcode_parse after we return
				break;
			case 250:
				//? --- M250: return current position, end position, queue ---
				//? Undocumented
				//? This command is only available in DEBUG builds.
				update_current_position(dda);
				sersendf_P(PSTR("{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t"), current_position.parameters[L_X], current_position.parameters[L_Y], current_position.parameters[L_Z], current_position.parameters[L_E], current_position.parameters[L_F], movebuffer[mb_tail].c, movebuffer[mb_tail].endpoint.parameters[L_X], movebuffer[mb_tail].endpoint.parameters[L_Y], movebuffer[mb_tail].endpoint.parameters[L_Z], movebuffer[mb_tail].endpoint.parameters[L_E], movebuffer[mb_tail].endpoint.parameters[L_F],
					#ifdef ACCELERATION_REPRAP
						movebuffer[mb_tail].end_c
					#else
						movebuffer[mb_tail].c
					#endif
					);

				print_queue();
				break;

			// M84- stop idle hold
			case 84:
				stepper_disable();
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				break;
			case 190:
				//? --- M190: Power On ---
				//? Undocumented.
				//? This one is pointless in Teacup. Implemented to calm the RepRap gurus.
				//?
				power_on();
				stepper_enable();
				x_enable();
				y_enable();
				z_enable();
				e_enable();
				break;

			case 200:
				//? --- M200: report endstop status ---
				//? Report the current status of the endstops configured in the firmware to the host.
				power_on();
				#if defined(X_MIN_PIN)
					sersendf_P(PSTR("x_min:%d "), x_min());
				#endif
				#if defined(X_MAX_PIN)
					sersendf_P(PSTR("x_max:%d "), x_max());
				#endif
				#if defined(Y_MIN_PIN)
					sersendf_P(PSTR("y_min:%d "), y_min());
				#endif
				#if defined(Y_MAX_PIN)
					sersendf_P(PSTR("y_max:%d "), y_max());
				#endif
				#if defined(Z_MIN_PIN)
					sersendf_P(PSTR("z_min:%d "), z_min());
				#endif
				#if defined(Z_MAX_PIN)
					sersendf_P(PSTR("z_max:%d "), z_max());
				#endif
				#if !(defined(X_MIN_PIN) || defined(X_MAX_PIN) || defined(Y_MIN_PIN) || defined(Y_MAX_PIN) || defined(Z_MIN_PIN) || defined(Z_MAX_PIN))
					sersendf_P(PSTR("no endstops defined"));
				#endif
				break;
			case 116:
				//? --- M116: Wait ---
				//?
				//? Example: M116
				//?
				//? Wait for ''all'' temperatures and other slowly-changing variables to arrive at their set values.  See also M109.

				enqueue(NULL);
				break;
			case 191:
				//? --- M191: Power Off ---
				//? Undocumented.
				//? Same as M2. RepRap obviously prefers to invent new numbers instead of looking into standards. 
				queue_wait();
				power_off();
				break;
			case 112:
				//? --- M112: Emergency Stop ---
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.
				//?

				queue_flush();
				break;
*/
