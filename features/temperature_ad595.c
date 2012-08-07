#include "common.h"

#ifndef FEATURES_H
API typedef struct sensor_ad595_userdata { uint8_t pin; } sensor_ad595_userdata;
API void ad595_init(void);
API void ad595_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void ad595_init(void){
	// initialised when read
}
void ad595_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	uint16_t               temp     = 0;
	sensor_ad595_userdata *userdata = sensor->userdata;
	
	temp = analog_read(userdata->pin);
	// convert
	// >>8 instead of >>10 because internal temp is stored as 14.2 fixed point
	temp = (temp * 500L) >> 8;
	
	runtime->next_read_time = 0;
	runtime->last_read_temp = temp;
}
