#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

// wether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// wether to insist on a checksum
//#define	REQUIRE_CHECKSUM

typedef enum letters { ///< This enum is used to reduce size of arrays which use letters as index
	L_G,
	L_M,
	L_X,
	L_Y,

	L_Z,
	L_E,
	L_F,
	L_S,
	
	L_P,
	L_T,
	L_N,
	L_CHECKSUM,
	
	MAX_LETTER
} letters;

/// this holds all the possible data from a received command
typedef struct {
	uint8_t                checksum;                 ///< checksum we calculated
	uint16_t               seen;                     ///< bit field for parameters
	decfloat               parameters[MAX_LETTER];   ///< array with all parameters
} GCODE_COMMAND;

#define PARAMETER_asint(_letter)        ( decfloat_to_int( &(((GCODE_COMMAND *) next_target)->parameters[_letter]), 1) )
#define PARAMETER_asmult(_letter,_mult) ( decfloat_to_int( &(((GCODE_COMMAND *) next_target)->parameters[_letter]), _mult) )
#define PARAMETER_SEEN(_letter)         ( (((GCODE_COMMAND *)next_target)->seen & (1<<_letter)) != 0 )
#define PARAMETER_SET(gcode,letter,value)    do {                         \
	decfloat_set_int(                                                 \
		&( ((GCODE_COMMAND *)gcode)->parameters[letter]), value); \
		((GCODE_COMMAND *)gcode)->seen |= (1<<letter);            \
	} while(0);

void gcode_init(void);

/// accept the next character and process it
void gcode_parse_char(uint8_t c);

uint8_t      gcode_convert_letter(letters c);
letters      gcode_convert_char(uint8_t c);
				
#define M400_WAIT() do { GCODE_COMMAND wait; PARAMETER_SET(&wait, L_M, 400); core_emit(EVENT_GCODE_PROCESS, &wait); } while(0);

#endif	/* _GCODE_PARSE_H */
