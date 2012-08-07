#include "common.h"

#ifndef FEATURES_H
API typedef struct sensor_pt100_userdata { } sensor_pt100_userdata;
API void pt100_init(void);
API void pt100_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void pt100_init(void){
	
}
void pt100_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	#warning TODO: PT100 code
}
