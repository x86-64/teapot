#include "common.h"
#include "ThermistorTable.h"

#ifndef FEATURES_H
API typedef struct sensor_thermistor_userdata { uint8_t pin; uint8_t table_num; } sensor_thermistor_userdata;
API void thermistor_init(void);
API void thermistor_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void thermistor_init(void){
	
}
void thermistor_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	uint8_t                      j, table_num;
	uint16_t                     temp     = 0;
	sensor_thermistor_userdata  *userdata = sensor->userdata;
	
	//Read current temperature
	temp = analog_read(userdata->pin);
	// for thermistors the thermistor table number is in the additional field
	table_num = userdata->table_num;

	//Calculate real temperature based on lookup table
	for (j = 1; j < NUMTEMPS; j++) {
		if (pgm_read_word(&(temptable[table_num][j][0])) > temp) {
			// Thermistor table is already in 14.2 fixed point
			//#ifndef	EXTRUDER
			//if (DEBUG_PID && (debug_flags & DEBUG_PID))
			//	sersendf_P(PSTR("pin:%d Raw ADC:%d table entry: %d"),userdata->pin,temp,j);
			//#endif
			// Linear interpolating temperature value
			// y = ((x - x₀)y₁ + (x₁-x)y₀ ) / (x₁ - x₀)
			// y = temp
			// x = ADC reading
			// x₀= temptable[j-1][0]
			// x₁= temptable[j][0]
			// y₀= temptable[j-1][1]
			// y₁= temptable[j][1]
			// y =
			// Wikipedia's example linear interpolation formula.
			temp = (
			//     ((x - x₀)y₁
				((uint32_t)temp - pgm_read_word(&(temptable[table_num][j-1][0]))) * pgm_read_word(&(temptable[table_num][j][1]))
			//                 +
				+
			//                   (x₁-x)
				(pgm_read_word(&(temptable[table_num][j][0])) - (uint32_t)temp)
			//                         y₀ )
				* pgm_read_word(&(temptable[table_num][j-1][1])))
			//                              /
				/
			//                                (x₁ - x₀)
				(pgm_read_word(&(temptable[table_num][j][0])) - pgm_read_word(&(temptable[table_num][j-1][0])));
			//#ifndef	EXTRUDER
			//if (DEBUG_PID && (debug_flags & DEBUG_PID))
			//	sersendf_P(PSTR(" temp:%d.%d"),temp/4,(temp%4)*25);
			//#endif
			return;
		}
	}
	
	//Clamp for overflows
	if (j == NUMTEMPS)
		temp = temptable[table_num][NUMTEMPS-1][1];

	runtime->next_read_time = 0;
	runtime->last_read_temp = temp;
}
