#include "common.h"
#include "axes.h"

      axis_t          axes         [] = {
	{ 0, 0, 0, L_X, 200, 1200, 0, 1000000, (axis_stepdir_userdata []){{ 17, 16, 2, 1 }}, &axis_stepdir_proto },
	{ 0, 0, 0, L_Y, 200, 1200, 0, 1000000, (axis_stepdir_userdata []){{ 15, 14, 2, 1 }}, &axis_stepdir_proto },
	{ 0, 0, 0, L_Z, 200, 1200, 0, 1000000, (axis_stepdir_userdata []){{ 0, 0 }}, &axis_stepdir_proto },
};
const uint8_t         axes_count = (sizeof(axes) / sizeof(axes[0]));

