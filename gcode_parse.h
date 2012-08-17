#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#include	"dda.h"

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
	uint8_t						checksum;                 ///< checksum we calculated
	uint16_t          seen;                     ///< bit field for parameters
  int32_t           parameters[MAX_LETTER];   ///< array with all parameters
} GCODE_COMMAND;

/// the command being processed
extern GCODE_COMMAND next_target;

#define PARAMETER(letter)       ( ((GCODE_COMMAND *) next_target)->parameters[letter])
#define PARAMETER_SEEN(letter)  ( (((GCODE_COMMAND *)next_target)->seen & (1<<letter)) != 0 )

void gcode_init(void);

/// accept the next character and process it
void gcode_parse_char(uint8_t c);

#endif	/* _GCODE_PARSE_H */
