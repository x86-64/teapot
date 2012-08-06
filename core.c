#include <core.h>

event_func  core_events[MAX_EVENT][MAX_FUNCS];

int core_register(core_event_type type, event_func core_event){
	int i;

	for(i=0; i<MAX_FUNCS; i++){
		if(!core_events[type][i]){
			core_events[type][i] = core_event;
			return 0;
		}
	}
	return 1;
}

void core_emit(core_event_type type){
	int i;

	for(i=0; i<MAX_FUNCS; i++){
		if(core_events[type][i])
			core_events[type][i]();
	}
}

