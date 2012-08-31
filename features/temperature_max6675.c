#include "common.h"

#ifndef FEATURES_H
API typedef struct sensor_max6675_userdata { } sensor_max6675_userdata;
API void max6675_init(void);
API void max6675_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime);
#endif

void max6675_init(void){
	// initialised when read
	// setup SPI
/*	WRITE(SCK, 0);				SET_OUTPUT(SCK);
	WRITE(MOSI, 1);				SET_OUTPUT(MOSI);
	WRITE(MISO, 1);				SET_INPUT(MISO);
	WRITE(SS, 1);				SET_OUTPUT(SS);*/
}
void max6675_read(const temp_sensor_t *sensor, temp_sensor_runtime_t *runtime){
	uint16_t	temp = 0;
	
	#ifdef	PRR
		PRR &= ~MASK(PRSPI);
	#elif defined PRR0
		PRR0 &= ~MASK(PRSPI);
	#endif

	SPCR = MASK(MSTR) | MASK(SPE) | MASK(SPR0);
	
	// enable TT_MAX6675
	// FIXME WRITE(SS, 0);

	// No delay required, see
	// https://github.com/triffid/Teacup_Firmware/issues/22

	// read MSB
	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp = SPDR;
	temp <<= 8;

	// read LSB
	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp |= SPDR;

	// disable TT_MAX6675
	// FIXME WRITE(SS, 1);

	// FIXME initialized but not used: runtime->temp_flags = 0;
	if ((temp & 0x8002) == 0) {
		// got "device id"
		// FIXME initialized but not used: runtime->temp_flags |= PRESENT;
		if (temp & 4) {
			// thermocouple open
			// FIXME initialized but not used: runtime->temp_flags |= TCOPEN;
		}
		else {
			temp = temp >> 3;
		}
	}

	// this number depends on how frequently temp_tick is called. the MAX6675 can give a reading every 0.22s, so set this to about 250ms
	runtime->next_read_time = 25;
	runtime->last_read_temp = temp;
}
