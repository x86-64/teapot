#include "common.h"

const temp_sensor_t           temp_sensors[] = {
	{ 0, 'T', &max6675_init, &max6675_read, (sensor_max6675_userdata []){ { } } }
	
};
const uint8_t                 temp_sensors_count = (sizeof(temp_sensors) / sizeof(temp_sensors[0]));
      temp_sensor_runtime_t   temp_sensors_runtime[(sizeof(temp_sensors) / sizeof(temp_sensors[0]))];
