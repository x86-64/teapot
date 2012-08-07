#include "common.h"

#ifndef FEATURES_H
API typedef struct sensor_none_userdata { } sensor_none_userdata;
API void none_init(void);
API void none_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void none_init(void){
	
}
void none_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	runtime->last_read_temp = runtime->target_temp; // for get_temp()
	runtime->next_read_time = 25;
}
