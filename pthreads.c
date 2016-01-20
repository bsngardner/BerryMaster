//	pthreads.c	12/08/2015
//*******************************************************************************
//  STATE        Task Control Block        Stacks (malloc'd)
//                          ______                                       T0 Stack
//  Running tcbs[0].thread | xxxx-+------------------------------------->|      |
//                 .stack  | xxxx-+-------------------------------       |      |
//                 .block  | 0000 |                               \      |      |
//                 .retval |_0000_|                       T1 Stack \     |      |
//  Ready   tcbs[1].thread | xxxx-+---------------------->|      |  \    |      |
//                 .stack  | xxxx-+------------------     |      |   \   |      |
//                 .block  | 0000 |                  \    |      |    -->|(exit)|
//                 .retval |_0000_|        T2 Stack   --->|r4-r15|       |------|
//  Blocked tcbs[2].thread | xxxx-+------->|      |       |  SR  |
//                 .stack  | xxxx-+---     |      |       |  PC  |
//                 .block  |(sem) |   \    |      |       |(exit)|
//                 .retval |_0000_|    --->|r4-r15|       |------|
//  Free    tcbs[3].thread | 0000 |        |  SR  |
//                 .stack  | ---- |        |  PC  |
//                 .block  | ---- |        |(exit)|
//                 .retval |_----_|        |------|
//
//*******************************************************************************
#include <msp430.h>
#include <stdlib.h>
#include <string.h>
#include "BerryMaster.h"
#include "pthreads.h"

Tcb tcbs[MAX_THREADS];						// thread control blocks
volatile pthread_t ctid;					// current task id

extern int __STACK_SIZE;					// --stack_size=xxx
extern char* __STACK_END;					// top of stack
//extern void TA_isr();

//*******************************************************************************
//--init TimerA for context switching--------------------------------------------
//
int timerA_init(void)
{
	TA0R = 0;								// reset timerA
	TA0CTL = TA_CTL;						// set timerA control reg
	TA0CCR0 = TA_FREQ;						// set interval (frequency)
	return 0;
} // end timerA_init


//*******************************************************************************
//--init pthread stack-----------------------------------------------------------

void init_stack(int* stack, int size)
{
	while (((int)stack != __get_SP_register()) && (size -= 2) > 0)
		*stack++ = STACK_FILLER;
	return;
} // end init_stack


//*******************************************************************************
//--init pthreads for context switching------------------------------------------

int pthread_init(pthread_attr_t* parms)
{
	__disable_interrupt();					// ** INTERRUPTS MUST BE DISABLED **
	memset(tcbs, 0, sizeof(tcbs));			// clear tcb's
	ctid = 0;								// thread 0
//	tcbs[ctid].thread = (void*)(__STACK_END - (parms ? parms->stack_size : __STACK_SIZE));
#if 1
	long temp = (0x2000 - (parms ? parms->stack_size : 0x8c));
	tcbs[ctid].thread = (void*)temp;
#else
	tcbs[ctid].thread = (void*)((long)__STACK_END - (parms ? parms->stack_size : (long)__STACK_SIZE));
#endif
	init_stack(tcbs[ctid].thread, (int)__get_SP_register() - (int)tcbs[ctid].thread - 2);
	tcbs[ctid].stack = (void*)(__get_SP_register() + 2);
	timerA_init();							// start scheduling clock
	return 0;
} // end thread_init


//*******************************************************************************
//--create new thread------------------------------------------------------------

int pthread_create(	pthread_t* tid_p,		// ptr to returned tid (NULL -> none)
					pthread_attr_t* attr,	// stack size (NULL -> default)
					void* (*func)(void*),	// new thread function ptr
					void* arg)				// new thread argument
{
	uint16_t tid, sr, stack_size;
	uint16_t* stack;
	// get new thread stack size
	stack_size = (attr) ? (uint16_t)attr->stack_size : DEFAULT_STACK_SIZE;
	sr = __get_SR_register();				// save current interrupt mode
	__disable_interrupt();					// disable interrupts while creating
	for (tid = 0; tid < MAX_THREADS; tid++)
	{
		if (tcbs[tid].thread == 0)			// find free tcb
		{
			if (tid_p) *tid_p = tid;						// return thread id
			stack = (uint16_t*)malloc(stack_size);			// malloc stack memory
			if (stack == NULL) return SYS_ERR_MALLOC;
			init_stack((int*)stack, stack_size);			// clear stack
			tcbs[tid].thread = (void*)stack;				// create tcb entry
			stack = (void*)((unsigned int)stack + stack_size);	// set TOS
			*--stack = (uint16_t)pthread_exit;				// default exit function
			*--stack = (uint16_t)func;						// PC
			*--stack = GIE;									// SR
			stack = (void*)((unsigned int)stack - (12*2));	// r4-r15
			*(stack + R12_INDEX) = (uint16_t)arg;				// r12 = argument
			tcbs[tid].stack = (void*)stack;					// save stack pointer
			tcbs[tid].block = 0;							// no pending events
			__set_interrupt_state(sr);						// restore interrupt state
			return 0;
		}
	}
	__set_interrupt_state(sr);				// error, too many threads
	return _ERR_PTHREADS;
} // end pthread_create


//*******************************************************************************
//--access current thread id-----------------------------------------------------

pthread_t pthread_self(void)
{
	return ctid;							// return current thread id
} // end pthread_self


int pthread_equal(pthread_t tid1, pthread_t tid2)
{
	return (tid1 == tid2);
} // end pthread_equal


//*******************************************************************************
//--context switch to next thread------------------------------------------------

int pthread_yield(void)
{
	TA0CTL |= TAIFG;							// force interrupt
	return 0;
} // end pthread_yield


//*******************************************************************************
//--join 2 threads---------------------------------------------------------------

int pthread_join(pthread_t tid,				// thread to joint with
				 void** return_value)		// ptr to ptr to return value
{
	while (tcbs[tid].thread) pthread_yield();
	if (return_value)
	{
		*return_value = (void*)&tcbs[tid].retval;
	}
	return 0;
} // end pthread_join


//*******************************************************************************
//--exit thread (kill self)------------------------------------------------------

void pthread_exit(void* return_value)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();
//	if (tcbs[ctid].thread == 0) ERROR2(_USER, _ERR_THREADID);
	if (tcbs[ctid].thread == 0) {
		//reportError("Err-pthread_exit!", _ERR_THREADID);
		handleError(); // todo: make a better error mechanism.
	}
	free ((void*)tcbs[ctid].thread);
	tcbs[ctid].thread = 0;
	tcbs[ctid].retval = (int)return_value;
	pthread_yield();						// force context switch
	__set_interrupt_state(sr);
	return;
} // end thread_exit


//*******************************************************************************
//--produce a pthread mutex------------------------------------------------------

int pthread_mutex_init(pthread_mutex_t *mutex,		// pointer to mutex struct
					   pthread_mutexattr_t *attr)	// initial (unlocked or locked)
{
	memset((char*)mutex, 0, sizeof(pthread_mutex_t));
	mutex->tid = (uint8_t)-1;					// invalidate tid
	mutex->state = attr ? attr->state : unblocked;
	// TODO: why not set mutex->size to zero?
	return 0;
} // end pthread_mutex_init


//*******************************************************************************
//--consume a pthread mutex------------------------------------------------------

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();
	if (mutex->state == unblocked)
	{
		mutex->state = blocked;				// consume mutex
		mutex->tid = ctid;					// save tid of locking thread
	}
	else
	{
		// mutex is locked - block thread
		tcbs[ctid].block = (void*)mutex;
		mutex->queue[mutex->size++] = ctid;	// queue tid
		pthread_yield();					// context switch
	}
	__set_interrupt_state(sr);				// return here when mutex acquired
	return 0;
} // end pthread_mutex_lock


//*******************************************************************************
//--produce a pthread mutex------------------------------------------------------

int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();

	// check for valid mutex operation
	if ((mutex->tid != ctid) || (mutex->state != blocked))
	{
		__set_interrupt_state(sr);			// different thread or not locked
		return _ERR_MUTEX;					// mutex error
	}
	if (mutex->size)
	{
		uint16_t i;
		uint16_t tid = mutex->queue[0];
		tcbs[tid].block = 0;				// wakeup task
		mutex->state = blocked;				// consume mutex again
		mutex->tid = tid;					// save tid of locking thread
		for (i = 1; i < mutex->size; i++)
		{
			mutex->queue[i-1] = mutex->queue[i];
		}
		mutex->size--;
	}
	else
	{
		mutex->state = unblocked;			// produce mutex
		mutex->tid = (uint8_t)-1;				// set invalidate tid
	}
	pthread_yield();						// context switch
	__set_interrupt_state(sr);
	return 0;
} // end pthread_mutex_unlock


//*******************************************************************************
//--destroy a pthread mutex------------------------------------------------------

int pthread_mutex_destroy(pthread_mutex_t *mutex)
{
	mutex->tid = (uint8_t)-1;					// set invalid mutex
	return 0;
} // end pthread_mutex_destroy


//*******************************************************************************
//--produce a semaphore----------------------------------------------------------

int sem_signal(sem_t* sem)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();
	if (++sem->state <= 0)					// produce semaphore
	{
		uint16_t i;
		tcbs[sem->queue[0]].block = 0;		// wakeup task
		for (i = 1; i < sem->size; i++)
		{
			sem->queue[i-1] = sem->queue[i];
		}
		sem->size--;
	}
	pthread_yield();						// context switch
	__set_interrupt_state(sr);
	return 0;
} // end sem_signal


//*******************************************************************************
//--consume a semaphore----------------------------------------------------------

int sem_wait(sem_t* sem)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();
	if (--sem->state < 0)					// consume semaphore
	{
		tcbs[ctid].block = (void*)sem;		// block task
		sem->queue[sem->size++] = ctid;		// queue tid
	}
	pthread_yield();						// context switch
	__set_interrupt_state(sr);				// return here when sem acquired
	return 0;
} // end sem_wait


//*******************************************************************************
//--conditionally consume a semaphore--------------------------------------------

int sem_trywait(sem_t* sem)
{
	uint16_t sr = __get_SR_register();
	__disable_interrupt();
	if (sem->state >= 0)
	{
		--sem->state;						// success, consume semaphore
		__set_interrupt_state(sr);			// restore status
		return 0;
	}
	__set_interrupt_state(sr);				// return here when sem acquired
	return -1;								// unsuccessful
} // end sem_try


//*******************************************************************************
//--create a new semaphore-------------------------------------------------------

int semaphore_create(sem_t* sem,			// ptr to semaphore struct
					 int value)				// initial semaphore value
{
	memset((char*)sem, 0, sizeof(sem_t));	// zero out struct
	sem->state = value;						// set initial value
	sem->tid = ctid;						// tid of creating thread
	return 0;
} // end semaphore_create

//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A (void) {
//	TA_isr();
//	return;
//}
