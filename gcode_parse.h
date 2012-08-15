#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#include	"dda.h"

// wether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// wether to insist on a checksum
//#define	REQUIRE_CHECKSUM

/// this is a very crude decimal-based floating point structure.
/// a real floating point would at least have signed exponent.\n
/// resulting value is \f$ mantissa * 10^{-(exponent - 1)} * ((sign * 2) - 1)\f$
typedef struct {
	uint32_t	mantissa;		///< the actual digits of our floating point number
	uint8_t	exponent	:7;	///< scale mantissa by \f$10^{-exponent}\f$
	uint8_t	sign			:1; ///< positive or negative?
} decfloat;

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

void gcode_init(void);

/// accept the next character and process it
void gcode_parse_char(uint8_t c);

// uses the global variable next_target.N
void request_resend(void);

#endif	/* _GCODE_PARSE_H */
