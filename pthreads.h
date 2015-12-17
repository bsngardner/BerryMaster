//	pthread.h	12/08/2015

#ifndef PTHREAD_H_
#define PTHREAD_H_

//******************************************************************************
//	System
//				 xxxxxx10  11   01  xxx1
#define TA_CTL	(TASSEL_2|ID_3|MC_1|TAIE)	// SMCLK,/8,UP,IE
#ifndef	myClock
#define	myClock	24000000
#endif
#ifndef TA_CLK
#define TA_CLK	100						// 10ms (100 clks/sec)
#endif
#define	TA_FREQ	(myClock/8/TA_CLK)

enum _PTHREAD_USER_ERRORS				// ERROR2 LED codes
{
	_ERR_PTHREADS = 2,					// 2 too many pthreads
	_ERR_THREADID,						// 3 illegal thread id
	_ERR_MUTEX,							// 4 WDT mutex fault
	_ERR_SEMAPHORE						// 5 illegal semaphore
};

#define R12_INDEX		8
int timerA_init(void);

//******************************************************************************
// Thread data

#define MAX_THREADS	2					// room for 2 threads
#define DEFAULT_STACK_SIZE	126			// default stack size
#define STACK_FILLER	0xdead

typedef struct							// thread control block
{
	void* thread;						// thread stack (NULL->no thread)
	void* stack;						// current stack pointer
	void* block;						// blocking mutex/semaphore
	int	retval;							// return value
} Tcb;

typedef struct							// POSIX thread attribute
{
	int stack_size;						// stack size (NULL->default)
	int priority;						// pthread priority (not supported)
} pthread_attr_t;						// pthread attribute

enum thread_index_t {tcb_thread = 0, tcb_stack = 2, tcb_block = 4, tcb_retval = 6};
int pthread_init(pthread_attr_t*);		// POSIX thread initialization

//******************************************************************************
//	Thread Identifiers

typedef unsigned int pthread_t;			// POSIX thread identifier

pthread_t pthread_self(void);
int pthread_equal(pthread_t tid1, pthread_t tid2);

//******************************************************************************
// Thread Management Functions

int pthread_create(pthread_t*, pthread_attr_t*, void* (*func)(void*), void*);
void pthread_exit(void* return_value);
int pthread_join(pthread_t tid, void** return_value);
int pthread_yield(void);

#define sched_yield pthread_yield

//******************************************************************************
//	Thread Mutex

typedef struct							// POSIX mutex
{
	int state;							// mutex value
	unsigned char tid;					// creator thread tid
	unsigned char size;					// # of blocked threads in queue
	unsigned char queue[MAX_THREADS];	// blocked queue
} pthread_mutex_t;

typedef struct							// POSIX mutex attribute
{
	int state;
} pthread_mutexattr_t;

enum mutexattr { unblocked = 0, blocked };

int pthread_mutex_init(pthread_mutex_t *mutex, pthread_mutexattr_t *attr);
int pthread_mutex_lock(pthread_mutex_t *mutex);
int pthread_mutex_unlock(pthread_mutex_t *mutex);
int pthread_mutex_destroy(pthread_mutex_t *mutex);

//******************************************************************************
//	Semaphores

typedef struct							// POSIX semaphore
{
	int state;							// semaphore value
	unsigned char tid;					// creator thread tid
	unsigned char size;					// # of blocked threads in queue
	unsigned char queue[MAX_THREADS];	// blocked queue
} sem_t;

int semaphore_create(sem_t* sem, int value);
int sem_wait(sem_t* sem);
int sem_trywait(sem_t* sem);
int sem_signal(sem_t* sem);

#endif /* PTHREAD_H_ */
