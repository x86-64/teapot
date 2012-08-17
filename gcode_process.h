#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

// when we have a whole line, feed it to this
void process_gcode_command(void *next_target);

void request_resend(void *next_target);

#endif	/* _GCODE_PROCESS_H */
