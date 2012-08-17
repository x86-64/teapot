
/** \file
	\brief Parse received G-Codes
*/
#include        "common.h"
#include	<string.h>

#include	"serial.h"
#include	"debug.h"
#include        "utils.h"

#include	"gcode_parse.h"
#include	"gcode_process.h"

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

/// crude crc macro
#define crc(a, b)		(a ^ b)

/// crude floating point data storage
decfloat      read_digit					__attribute__ ((__section__ (".bss")));

/// this is where we store all the data for the current command before we work out what to do with it
GCODE_COMMAND next_gcode		__attribute__ ((__section__ (".bss")));

parser_state  gcode_parser_state       = S_PARSE_CHAR;
uint8_t       gcode_parser_char        = MAX_LETTER;

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

uint8_t      gcode_convert_letter(letters c){
	switch(c){
		case L_G: return 'G';
		case L_M: return 'M';
		case L_X: return 'X';
		case L_Y: return 'Y';
		case L_Z: return 'Z';
		case L_E: return 'E';
		case L_F: return 'F';
		case L_S: return 'S';
		case L_P: return 'P';
		case L_T: return 'T';
		case L_N: return 'N';
		case L_CHECKSUM: return '*';
		case MAX_LETTER:
			break;
	}
	return 0;
}

/// Character Received - add it to our command
/// \param c the next character to process
void gcode_parse_char(uint8_t c){
	lexer_token            type;
	
	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;
	
	type = gcode_lexer(c);
redo:;
	switch(gcode_parser_state){
		case S_PARSE_CHAR: // wait for parameter name mode
			switch(type){
				// allow comments to start between parameters 
				case T_COMMENT_SEMICOLON: gcode_parser_state = S_COMMENT_SEMI;    break;
				case T_COMMENT_START:     gcode_parser_state = S_COMMENT_BRACKET; break;
				
				// this is axis or some other parameter
				case T_CHAR:
					if( (gcode_parser_char = gcode_convert_char(c)) == MAX_LETTER)
						goto error; // unknown parameter letter
					gcode_parser_state = S_PARSE_NUMBER;
					
					// clean read_digit before parsing anything
					read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
					
					// set seen flag
					next_gcode.seen |= 1 << gcode_parser_char;
					break;
				
				// ignore spaces
				case T_SPACE:
					break;

				// newline means end of current command - start executing
				case T_NEWLINE:
					// process
					serial_writestr_P(PSTR("ok "));
					process_gcode_command(&next_gcode);
					serial_writechar('\n');
					
					// reset variables
					next_gcode.seen     = 0;
					next_gcode.checksum = 0;
					break; 
				
				default:
					goto error; // ERR unknown token for this mode
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
					
				case T_NEWLINE: // we finished
				case T_SPACE:   
					// since we use universal parameters table - all conversions to inch or mm goes to according modules
					next_gcode.parameters[gcode_parser_char] = decfloat_to_int(&read_digit, 1);
					
					gcode_parser_state = S_PARSE_CHAR;
					gcode_parser_char  = MAX_LETTER;
					
					if(type == T_NEWLINE) goto redo; // if this is newline - then S_PARSE_CHAR would be interested in this, redo switching
					break;
					
				default:
					goto error; // ERR unknown token for this mode
			}
			break;
		
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
		next_gcode.checksum = crc(next_gcode.checksum, c);
	
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

