#ifndef TEMPERATURE_H
#define TEMPERATURE_H

typedef struct temp_sensor_t         temp_sensor_t;
typedef struct temp_sensor_runtime_t temp_sensor_runtime_t;

typedef void (*func_sensor_init)(void);
typedef void (*func_sensor_read)(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);

struct temp_sensor_t {
	uint8_t                id;
	uint8_t                rep_char;       ///< character sensor will have in M105 query, 0 - don't print
	func_sensor_init       sensor_init;
	func_sensor_read       sensor_read;

	void                  *userdata;
};

/// this struct holds the runtime sensor data- read temperatures, targets, etc
struct temp_sensor_runtime_t {
	uint16_t               last_read_temp; ///< last received reading
	uint16_t               target_temp;    ///< manipulate attached heater to attempt to achieve this value

	uint16_t               temp_residency; ///< how long have we been close to target temperature in temp ticks?

	uint16_t               next_read_time; ///< how long until we can read this sensor again?
};

extern const temp_sensor_t           temp_sensors[];
extern const uint8_t                 temp_sensors_count;
extern       temp_sensor_runtime_t   temp_sensors_runtime[];

#endif
