#define API

#include <stdint.h>

/*

#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay_basic.h>

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/version.h>
#include <avr/wdt.h>
*/
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "config.h"

#include "arduino.h"

#include "core.h"
#include "dda.h"
#include "utils.h"

#include "gcode_parse.h"
#include "gcode_process.h"
#include "serial.h"


#ifndef FEATURE
#include "features.h"
#endif

