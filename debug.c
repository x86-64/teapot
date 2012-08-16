#include "common.h"
#include "debug.h"

volatile uint8_t	debug_flags;

void debug_gcode_process(void){
	if(! PARAMETER_SEEN(L_M) )
		return;
	
	switch(PARAMETER(L_M)){
		case 240:
			//? --- M240: echo off ---
			//? Disable echo.
			//? This command is only available in DEBUG builds.
			debug_flags &= ~DEBUG_ECHO;
			serial_writestr_P(PSTR("Echo off"));
			// newline is sent from gcode_parse after we return
			break;

		case 241:
			//? --- M241: echo on ---
			//? Enable echo.
			//? This command is only available in DEBUG builds.
			debug_flags |= DEBUG_ECHO;
			serial_writestr_P(PSTR("Echo on"));
			// newline is sent from gcode_parse after we return
			break;

		case 111:
			//? --- M111: Set Debug Level ---
			//?
			//? Example: M111 S6
			//?
			//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
			//?
			//? <Pre>
			//? #define         DEBUG_PID       1
			//? #define         DEBUG_DDA       2
			//? #define         DEBUG_POSITION  4
			//? </pre>
			//?
			//? This command is only available in DEBUG builds of Teacup.

			if ( ! PARAMETER_SEEN(L_S) )
				break;
			
			debug_flags = PARAMETER(L_S);
			break;
		
		case 253:
			//? --- M253: read arbitrary memory location ---
			//? Undocumented
			//? This command is only available in DEBUG builds.
			
			if ( ! PARAMETER_SEEN(L_S))
				break;
			
			uint8_t *from = (void *)( (uint16_t) PARAMETER(L_S) );
			uint32_t size = PARAMETER_SEEN(L_P) ? PARAMETER(L_P) : 1;
			
			for (; size; size--, from++)
				serwrite_hex8( *(volatile uint8_t *)from );
			break;

		case 254:
			//? --- M254: write arbitrary memory location ---
			//? Undocumented
			//? This command is only available in DEBUG builds.
			if ( ! PARAMETER_SEEN(L_S) || ! PARAMETER_SEEN(L_P))
				break;
			
			uint8_t *to   = (void *)( (uint16_t) PARAMETER(L_S) );
			uint8_t  what = PARAMETER(L_P);
			
			sersendf_P(PSTR("%x:%x->%x"), PARAMETER(L_S), *(volatile uint8_t *)to, what);
			(*(volatile uint8_t *)to) = what;
			break;
	}
}

void debug_init(void){
	#ifdef	DEBUG
		core_register(EVENT_GCODE_PROCESS, &debug_gcode_process);
	#endif /* DEBUG */
}

