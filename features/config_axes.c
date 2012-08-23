#include "common.h"
#include "axes.h"

const axis_t          axes         [] = {
	{ 0, 0, 0, L_X, 50, 200, 0, 1000000, (const axis_stepdir_userdata []){{ 0, 0 }}, &axis_stepdir_gcode },
	{ 0, 0, 0, L_Y, 50, 200, 0, 1000000, (const axis_stepdir_userdata []){{ 0, 0 }}, &axis_stepdir_gcode },
	{ 0, 0, 0, L_Z, 50, 200, 0, 1000000, (const axis_stepdir_userdata []){{ 0, 0 }}, &axis_stepdir_gcode },
};
const uint8_t         axes_count = (sizeof(axes) / sizeof(axes[0]));

      axis_runtime_t  axes_runtime [(sizeof(axes) / sizeof(axes[0]))] = {
        { 0, 0, (axis_runtime_stepdir_userdata []){{ }} },
        { 0, 0, (axis_runtime_stepdir_userdata []){{ }} },
        { 0, 0, (axis_runtime_stepdir_userdata []){{ }} },
};


