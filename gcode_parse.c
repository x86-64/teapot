#include	"gcode_parse.h"

/** \file
	\brief Parse received G-Codes
*/

#include	<string.h>

#include	"serial.h"
#include	"dda_queue.h"
#include	"debug.h"

#include	"gcode_process.h"

/// current or previous gcode word
/// for working out what to do with data just received
uint8_t last_field = 0;

/// crude crc macro
#define crc(a, b)		(a ^ b)

/// crude floating point data storage
decfloat read_digit					__attribute__ ((__section__ (".bss")));

/// this is where we store all the data for the current command before we work out what to do with it
GCODE_COMMAND next_target		__attribute__ ((__section__ (".bss")));

/*
	decfloat_to_int() is the weakest subject to variable overflow. For evaluation, we assume a build room of +-1000 mm and STEPS_PER_MM_x between 1.000 and 4096. Accordingly for metric units:

		df->mantissa:  +-0..1048075    (20 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  1000            (10 bit)

	imperial units:

		df->mantissa:  +-0..32267      (15 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  25400           (15 bit)
*/
// decfloat_to_int() can handle a bit more:
#define	DECFLOAT_EXP_MAX 3 // more is pointless, as 1 um is our presision
// (2^^32 - 1) / multiplicand - powers[DECFLOAT_EXP_MAX] / 2 =
// 4294967295 / 1000 - 5000 =
#define	DECFLOAT_MANT_MM_MAX 4289967  // = 4290 mm
// 4294967295 / 25400 - 5000 =
#define	DECFLOAT_MANT_IN_MAX 164093   // = 164 inches = 4160 mm

/*
	utility functions
*/
extern const uint32_t powers[];  // defined in sermsg.c

/// convert a floating point input value into an integer with appropriate scaling.
/// \param *df pointer to floating point structure that holds fp value to convert
/// \param multiplicand multiply by this amount during conversion to integer
///
/// Tested for up to 42'000 mm (accurate), 420'000 mm (precision 10 um) and
/// 4'200'000 mm (precision 100 um).
static int32_t decfloat_to_int(decfloat *df, uint16_t multiplicand) {
	uint32_t	r = df->mantissa;
	uint8_t	e = df->exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
	if (e)
		e--;

	// This raises range for mm by factor 1000 and for inches by factor 100.
	// It's a bit expensive, but we should have the time while parsing.
	while (e && multiplicand % 10 == 0) {
		multiplicand /= 10;
		e--;
	}

	r *= multiplicand;
	if (e)
		r = (r + powers[e] / 2) / powers[e];

	return df->sign ? -(int32_t)r : (int32_t)r;
}


typedef enum lexer_token {
	T_CHAR,
	T_DIGIT,
	T_SPACE,
	T_NEWLINE,
	T_SIGN,
	T_DOT,
	T_COMMENT_SEMICOLON,
	T_COMMENT_START,
	T_COMMENT_END,

	T_INVALID
} lexer_token;

typedef enum parser_state {
	S_PARSE_CHAR,
	S_PARSE_NUMBER,
	S_COMMENT_SEMI,
	S_COMMENT_BRACKET,
} parser_state;

parser_state gcode_parser_state       = S_PARSE_CHAR;
uint8_t      gcode_parser_char        = MAX_LETTER;

lexer_token  gcode_lexer(uint8_t c){
	      if(c >= 'A' && c <= 'Z'){      return T_CHAR;
	}else if(c == '*'){                  return T_CHAR;
	}else if(c >= '0' || c <= '9'){      return T_DIGIT;
	}else if(c == ' ' || c == '\t'){     return T_SPACE;
	}else if(c == '\r' || c == '\n'){    return T_NEWLINE;
	}else if(c == '-'){                  return T_SIGN;
	}else if(c == '.'){                  return T_DOT;
	}else if(c == ';'){                  return T_COMMENT_SEMICOLON;
	}else if(c == '('){                  return T_COMMENT_START;
	}else if(c == ')'){                  return T_COMMENT_END;
	}
	return T_INVALID;
}

letters      gcode_convert_char(uint8_t c){
	switch(c){
		case 'G': return L_G;
		case 'M': return L_M;
		case 'X': return L_X;
		case 'Y': return L_Y;
		case 'Z': return L_Z;
		case 'E': return L_E;
		case 'F': return L_F;
		case 'S': return L_S;
		case 'P': return L_P;
		case 'T': return L_T;
		case 'N': return L_N;
		case '*': return L_CHECKSUM;
	}
	return MAX_LETTER;
}

/// Character Received - add it to our command
/// \param c the next character to process
void gcode_parse_char(uint8_t c){
	lexer_token            type;
	
	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;
	
	type = gcode_lexer(c);
	switch(gcode_parser_state){
		case S_PARSE_CHAR: // wait for parameter name mode
			switch(type){
				// allow comments to start between parameters 
				case T_COMMENT_SEMICOLON: gcode_parser_state = S_COMMENT_SEMI;    break;
				case T_COMMENT_START:     gcode_parser_state = S_COMMENT_BRACKET; break;
				
				// this is axis or some other parameter
				case T_CHAR:
					gcode_parser_char  = gcode_convert_char(c);
					gcode_parser_state = S_PARSE_NUMBER;
					
					// clean read_digit before parsing anything
					read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;

					// set seen flag
					next_target.seen |= 1 << gcode_parser_char;
					break;
				
				// ignore spaces
				case T_SPACE:
					break;

				// newline means end of current command - start executing
				case T_NEWLINE:
					// process
					serial_writestr_P(PSTR("ok "));
					process_gcode_command();
					serial_writechar('\n');
					
					// reset variables
					next_target.seen     = 0;
					next_target.checksum = 0;
					break; 
				
				default:
					goto error;
			}
			break;
			
		case S_PARSE_NUMBER: // wait for parameter value mode
			switch(type){
				case T_DIGIT:
					if (read_digit.exponent < DECFLOAT_EXP_MAX + 1){
						// this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
						read_digit.mantissa = (read_digit.mantissa << 3) + (read_digit.mantissa << 1) + (c - '0');
						if (read_digit.exponent)
							read_digit.exponent++;
					}
					break;
					
				case T_SIGN:
					read_digit.sign     = 1;
					// force sign to be at start of number, so 1-2 = -2 instead of -12
					read_digit.exponent = 0;
					read_digit.mantissa = 0;
					break;

				case T_DOT:
					if (read_digit.exponent == 0)
						read_digit.exponent = 1;
					break;
					
				case T_SPACE: // we finished
					// since we use universal parameters table - all conversions to inch or mm goes to according modules
					next_target.parameters[gcode_parser_char] = decfloat_to_int(&read_digit, 1);
					
					gcode_parser_state = S_PARSE_CHAR;
					gcode_parser_char  = MAX_LETTER;
					break;
					
				default:
					goto error;
			}
		
		// comments
		case S_COMMENT_SEMI:
			switch(type){
				case T_NEWLINE:           gcode_parser_state = S_PARSE_CHAR;      break;
				default: break;
			}
			break;
		case S_COMMENT_BRACKET:
			switch(type){
				case T_COMMENT_END:       gcode_parser_state = S_PARSE_CHAR;      break;
				default: break;
			}
			break;
	}
resume:
	
	if(gcode_parser_char != L_CHECKSUM)
		next_target.checksum = crc(next_target.checksum, c);
	
	//if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
	//	serial_writechar(c);
	return;

error:
	#ifdef	DEBUG
		// emit error
		serial_writechar('?');
		serial_writechar(c);
		serial_writechar('?');
	#endif
	goto resume;
}

