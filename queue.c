#include "common.h"
#include "queue.h"
#include "memory_barrier.h"

#define NO_INTERRUPTS_BLOCK()   uint8_t s_reg = SREG; cli();
#define END_BLOCK()             SREG = s_reg;

uint8_t queue_have_space(uint8_t *p_pointer, uint8_t *p_behind, uint8_t queue_size){
	NO_INTERRUPTS_BLOCK();
	MEMORY_BARRIER();
	
	uint8_t                ret;
	uint8_t                pointer           = *p_pointer;
	uint8_t                behind            = *p_behind;
	
	pointer += 1;
	pointer -= (pointer >= queue_size) ? queue_size : 0;
	
	ret = (pointer == behind) ?
		255 : // error
		0;
	
	END_BLOCK()
	return ret;
}

uint8_t queue_push(uint8_t *p_pointer, uint8_t *p_behind, uint8_t queue_size){
	NO_INTERRUPTS_BLOCK();
	MEMORY_BARRIER();
	
	uint8_t                ret;
	uint8_t                pointer           = *p_pointer;
	uint8_t                behind            = *p_behind;
	
	pointer += 1;
	pointer -= (pointer >= queue_size) ? queue_size : 0;
	
	if(pointer == behind){
		ret = 255; // error
	}else{
		*p_pointer = pointer;
		ret = 0;   // success
	}
	
	MEMORY_BARRIER();
	END_BLOCK();
	return ret;
}

uint8_t queue_have_item(uint8_t *p_pointer, uint8_t *p_ahead, uint8_t queue_size){
	NO_INTERRUPTS_BLOCK();
	MEMORY_BARRIER();
	
	uint8_t                ret;
	uint8_t                pointer           = *p_pointer;
	uint8_t                ahead             = *p_ahead;
	
	ret =  (pointer == ahead) ?
		255 :
		0;
	
	END_BLOCK();
	return ret;
}

uint8_t queue_pop(uint8_t *p_pointer, uint8_t *p_ahead, uint8_t queue_size){
	NO_INTERRUPTS_BLOCK();
	MEMORY_BARRIER();
	
	uint8_t                ret;
	uint8_t                pointer           = *p_pointer;
	uint8_t                ahead             = *p_ahead;
	
	if(pointer == ahead){
		ret = 255; // error
	}else{
		pointer += 1;
		pointer -= (pointer >= queue_size) ? queue_size : 0;
		
		*p_pointer = pointer;
		ret = 0;   // success
	}
	
	MEMORY_BARRIER();
	END_BLOCK();
	return ret;
}

uint8_t queue_current(uint8_t *p_pointer){
	MEMORY_BARRIER();
	return *p_pointer;
}

/* usage example:
 *

typedef struct myqueue_t {
	uint8_t      mb_head;
	uint8_t      mb_tail;
	something_t  mb[QUEUE_SIZE];
} myqueue_t;

myqueue_t  myqueue;

#define myqueue_have_space()  queue_have_space (&myqueue->mb_head, &myqueue->mb_tail, QUEUE_SIZE)
#define myqueue_have_item()   queue_have_item  (&myqueue->mb_tail, &myqueue->mb_head, QUEUE_SIZE)
#define myqueue_push()        queue_push       (&myqueue->mb_head, &myqueue->mb_tail, QUEUE_SIZE)
#define myqueue_pop()         queue_pop        (&myqueue->mb_tail, &myqueue->mb_head, QUEUE_SIZE)
#define myqueue_curr_space()  queue_current    (&myqueue->mb_head)
#define myqueue_curr_item()   queue_current    (&myqueue->mb_tail)

int main(void){
	// insert item
	uint8_t i = myqueue_curr_space();
	myqueue.mb[i] = something;
	myqueue_push();
	
	// get item
	uint8_t i = myqueue_curr_item();
	print( myqueue.mb[i] );
	myqueue_pop();
}

 */
