#ifndef AXES_H
#define AXES_H

typedef struct axis_t         axis_t;

typedef void (*func_axis_init)(axis_t *axis);
typedef void (*func_axis_gcode)(axis_t *axis, void *next_target);

typedef struct axis_runtime_t {
	uint8_t                relative :1;          ///< Use relative mode
	uint8_t                inches   :1;          ///< Use inches (1) or mm (0)
	void                  *userdata;             ///< Stepper runtime userdata

	 int32_t               position_curr;        ///< Current position on axis
} axis_runtime_t;

typedef struct axis_proto_t {
	func_axis_init         func_init;            ///< Function to call on start
	func_axis_gcode        func_gcode;           ///< Function to handle gcodes
} axis_proto_t;

typedef struct axis_t {
	uint8_t                sticky_relative   :1; ///< Ignore G90/91 requests
	uint8_t                have_position_min :1; ///< Do we have minimal position value?
	uint8_t                have_position_max :1; ///< Do we have minimal position value?
	
	uint8_t                letter;               ///< Letter for this axis (L_X, L_Y, L_Z, ...)
	uint32_t               feedrate_search;      ///< Search feedrate for this axis (mm/min)
	uint32_t               feedrate_max;         ///< Maximum feedrate value for this axis (mm/min)
	 int32_t               position_min;         ///< Minimal position value (um)
	 int32_t               position_max;         ///< Maximal position value (um)
	const void            *userdata;             ///< Stepper userdata
	
	axis_proto_t          *proto;                ///< Prototype functions
	axis_runtime_t         runtime;              ///< Runtime data
} axis_t;

extern       axis_t          axes[];
extern const uint8_t         axes_count;
#endif
