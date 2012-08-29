#include "common.h"
#include "axes.h"

      axis_t          axes         [] = {
	{ 0, 0, 0, L_X, 50, 200, 0, 1000000, (axis_stepdir_userdata []){{ PC2, PC3 }}, &axis_stepdir_proto },
	{ 0, 0, 0, L_Y, 50, 200, 0, 1000000, (axis_stepdir_userdata []){{ PC0, PC1 }}, &axis_stepdir_proto },
	{ 0, 0, 0, L_Z, 50, 200, 0, 1000000, (axis_stepdir_userdata []){{ 0, 0 }}, &axis_stepdir_proto },
};
const uint8_t         axes_count = (sizeof(axes) / sizeof(axes[0]));

