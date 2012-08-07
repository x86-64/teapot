#include "common.h"

#ifndef FEATURES_H
API typedef struct sensor_dummy_userdata { } sensor_dummy_userdata;
API void dummy_init(void);
API void dummy_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void dummy_init(void){
	
}
void dummy_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	uint16_t	temp = 0;
	
	temp = runtime->last_read_temp;

	if (runtime->target_temp > temp)
		temp++;
	else if (runtime->target_temp < temp)
		temp--;

	runtime->next_read_time = 0;
}
