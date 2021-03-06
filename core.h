#ifndef CORE_H
#define CORE_H

#define MAX_FUNCS 7

typedef void (*event_func)(void *userdata);

typedef enum core_event_type {
	EVENT_INIT,
	EVENT_TICK,
	EVENT_TICK_10MS,
	EVENT_GCODE_PROCESS,

	MAX_EVENT,
} core_event_type;

int  core_register(core_event_type type, event_func core_event);
void core_emit(core_event_type type, void *userdata);

#endif
