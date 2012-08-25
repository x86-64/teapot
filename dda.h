#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>
#include	"common.h"


#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/*
	types
*/

typedef enum dda_status {
	DDA_EMPTY,
	DDA_READY,
	DDA_RUNNING,
	DDA_FINISHED
} dda_status;

/**
	\struct dda_target_t
	\brief target is simply a point in space/time

	X in micrometers unless explcitely stated. F is in mm/min.
*/
typedef struct {
	int32_t						X;
	
	uint32_t					F;
} dda_target_t;

typedef struct dda_move_t {
	uint32_t               steps;                      ///< number of steps on axis
	
	#ifdef ACCELERATION_RAMPING
	uint32_t               ramping_step_no;            ///< counts actual steps done
	uint32_t               ramping_c;                  ///< time until next step
	int32_t                ramping_n;                  ///< tracking variable
	#endif
} dda_move_t;

/**
	\struct dda
	\brief this is a digital differential analyser data struct

	This struct holds all the details of an individual multi-axis move, including pre-calculated acceleration data.
	This struct is filled in by dda_create(), called from enqueue(), called mostly from gcode_process() and from a few other places too (eg \file homing.c)
*/
typedef struct dda_t {
	dda_status                                      status;
	
	/// this is where we should finish
	dda_target_t                                    position_start;
	dda_target_t                                    position_target;

	union {
		struct {
			uint8_t						direction		:1; ///< direction flag for axis
			
			#ifdef ACCELERATION_REPRAP
			uint8_t						accel					:1; ///< bool: speed changes during this move, run accel code
			#endif
		};
		uint8_t							allflags;	///< used for clearing all flags
	};

	// distances
	uint32_t					delta_um; ///< number of um on axis to do
	uint32_t					delta_steps; ///< number of steps on axis to do
	uint32_t					c; ///< time until next step, 24.8 fixed point

	#ifdef ACCELERATION_REPRAP
	uint32_t					reprap_end_c; ///< time between 2nd last step and last step
	 int32_t					reprap_n; ///< precalculated step time offset variable. At every step we calculate \f$c = c - (2 c / n)\f$; \f$n+=4\f$. See http://www.embedded.com/columns/technicalinsights/56800129?printable=true for full description
	#endif
	#ifdef ACCELERATION_RAMPING
	uint32_t					rampup_steps; ///< number of steps accelerating
	uint32_t					rampdown_steps; ///< number of last step before decelerating
	uint32_t					ramping_c_min; ///< 24.8 fixed point timer value, maximum speed
	#endif
	
	dda_move_t                                     *move;
} dda_t;

typedef struct dda_queue_t {
	/// movebuffer head pointer. Points to the last move in the queue.
	/// this variable is used both in and out of interrupts, but is
	/// only written outside of interrupts.
	uint8_t	mb_head;
	
	/// movebuffer tail pointer. Points to the currently executing move
	/// this variable is read/written both in and out of interrupts.
	uint8_t	mb_tail;
	
	/// move buffer.
	/// holds move queue
	/// contents are read/written both in and out of interrupts, but
	/// once writing starts in interrupts on a specific slot, the
	/// slot will only be modified in interrupts until the slot is
	/// is no longer live.
	/// The size does not need to be a power of 2 anymore!
	dda_t movebuffer[MOVEBUFFER_SIZE]; ///< this is the ringbuffer that holds the current and pending moves.
} dda_queue_t;

typedef struct dda_order_t {
	uint8_t                callme    :1;  ///< make a call in given time
	uint8_t                step      :1;  ///< make a step
	
	uint32_t               c;             ///< time until next step
	uint8_t                direction :1;  ///< direction to step
} dda_order_t;

/*
	methods
*/

// queue status methods
uint8_t queue_full(dda_queue_t *queue);
uint8_t queue_empty(dda_queue_t *queue);

void queue_init(dda_queue_t *queue);

// print queue status
void queue_debug_print(dda_queue_t *queue);

// add a new target to the queue
void queue_enqueue(dda_queue_t *queue, dda_target_t *t);

// take one step
void queue_step(dda_queue_t *queue, dda_order_t *order);

#endif	/* _dda_queue_t */
