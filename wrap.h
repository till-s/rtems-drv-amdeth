#ifndef TILL_WRAPPER_H
#define TILL_WRAPPER_H
/* $Id$ */

/* portability wrapper for semaphores and tasks
 * Author: Till Straumann <strauman@slac.stanford.edu>, 10/2001
 */
#ifndef __linux__
#include <stdio.h>
#define MY_PRINTF printf
#else
#include <linux/kernel.h>
#define MY_PRINTF printk
#endif

#if defined(__vxworks)
#include <vxWorks.h>
#elif defined(__rtems)
#include <rtems.h>
#elif defined(__linux__)
#ifndef USE_PTHREAD
#define USE_PTHREAD
#endif
#ifndef USE_PSEMA
#define USE_PSEMA
#endif
#else
#error "no OS type defined"
#endif

/* Use POSIX semaphores */
#ifdef  USE_PSEMA
#include <semaphore.h>
typedef sem_t		PSemaId;

inline int
pSemCreate(int binary, unsigned long init_count, PSemaId *ppsem)
{
	return sem_init(ppsem,0,init_count);
}

inline int
pSemDestroy(PSemaId *ppsem)
{
	return sem_destroy(ppsem);
}

inline int
pSemPost(PSemaId *ppsem)
{
	return sem_post(ppsem);
}

inline int
pSemWait(PSemaId *ppsem)
{
	return sem_wait(ppsem);
}

#elif defined(__vxworks)
/* vxworks native semaphores */

#include <vxWorks.h>
#include <semLib.h>
typedef SEM_ID		PSemaId;

int
pSemCreate(int binary, unsigned long init_count, PSemaId *ppsem)
{
	return !(*ppsem = (binary ?
		semBCreate(SEM_Q_FIFO,init_count ? SEM_FULL : SEM_EMPTY) :
		semCCreate(SEM_Q_FIFO,init_count)));
}

inline int
pSemDestroy(PSemaId *ppsem)
{
	return OK!=semDelete(*ppsem);
}

inline int
pSemPost(PSemaId *ppsem)
{
	return OK!=semGive(*ppsem);
}

inline int
pSemWait(PSemaId *ppsem)
{
	return OK!=semTake(*ppsem,WAIT_FOREVER);
}

#elif defined(__rtems)
/* rtems native semaphores */
typedef rtems_id	PSemaId;

inline int
pSemCreate(int binary, unsigned long init_count, PSemaId *ppsem)
{
	rtems_name sname=rtems_build_name('S',' ','0','0');

	return RTEMS_SUCCESSFUL!=rtems_semaphore_create(
			sname++,
			init_count,
			(binary ? RTEMS_BINARY_SEMAPHORE : RTEMS_COUNTING_SEMAPHORE)
				| RTEMS_FIFO,
			0,
			ppsem);
}
	
inline int
pSemDestroy(PSemaId *ppsem)
{
	return RTEMS_SUCCESSFUL != rtems_semaphore_delete(*ppsem);
}

inline int
pSemPost(PSemaId *ppsem)
{
	return RTEMS_SUCCESSFUL != rtems_semaphore_release(*ppsem);
}

inline int
pSemWait(PSemaId *ppsem)
{
	return RTEMS_SUCCESSFUL !=
		rtems_semaphore_obtain(*ppsem,
				RTEMS_WAIT,
				RTEMS_NO_TIMEOUT);
}

#endif

/* wrapper for task creation */

/* entry routine definition */
typedef void * (*Task_T)(void*);

/* priority ranges from 0 (max) to 255 (min)
 *
 * On systems that support a different range
 * the number is scaled appropriately
 */


#define SCALE_PRIO(pri,min,max) (((max)*(255-(pri))-(min)*(pri))/255)

#ifdef USE_PTHREAD

#include <pthread.h>
#ifndef __linux__
#include <sched.h>
#else
#include <rtl_sched.h>
#endif
typedef pthread_t	PTaskId;

typedef void*		PTaskArg;

#define PTASK_DECL(entry_point,arg) void* entry_point(PTaskArg arg)
#define PTASK_LEAVE                 do { return 0; } while (0)

#elif defined (__vxworks)

#include <taskLib.h>
typedef int		PTaskId;

typedef int		PTaskArg;

#define PTASK_DECL(entry_point,arg) int entry_point(PTaskArg arg, \
		int arg1, int arg2, int arg3, int arg4, \
		int arg5, int arg6, int arg7, int arg8, int arg9)

#define PTASK_LEAVE                 do { return 0; } while (0)

#elif defined(__rtems)
#include <string.h>
typedef rtems_id	PTaskId;

typedef rtems_task_argument PTaskArg;

#define PTASK_DECL(entry_point,arg) rtems_task entry_point(PTaskArg arg)
/* TODO: I believe the RTEMS native task should delete itself when leaving */
#define PTASK_LEAVE                 do { } while (0)

#endif

typedef PTASK_DECL( (*PTaskEntryPoint), dummy);

inline int
pTaskSpawn(char *name, int prio, int stacksize, int fpTask,
	   PTaskEntryPoint entry, PTaskArg arg, PTaskId *ptask)
{
	int	np;
#ifdef USE_PTHREAD
	pthread_attr_t 		attr;
	struct sched_param	schedparam;

	schedparam.sched_priority=np=SCALE_PRIO(prio,
					sched_get_priority_min(SCHED_FIFO),
					sched_get_priority_max(SCHED_FIFO));
#elif defined(__vxworks)
#if 1
	np = SCALE_PRIO(prio,255,0);
#else
	np = SCALE_PRIO(prio,255,1);	/* reserve 0 for the exec task */
#endif
#elif defined(__rtems)
	char	tmp[4]={0};
	rtems_name rn;
	np = SCALE_PRIO(prio,255,1);
#endif

	MY_PRINTF("spawning task at prio %i\n",np);

#ifdef USE_PTHREAD
	if (pthread_attr_init(&attr) ||
#ifndef __linux__ /* seems to be unimplemented */
		pthread_attr_setschedpolicy(&attr,SCHED_FIFO) ||
#endif
		pthread_attr_setdetachstate(&attr,
#if 0
			PTHREAD_CREATE_DETACHED
#else
			PTHREAD_CREATE_JOINABLE
#endif
			) ||
		pthread_attr_setschedparam(&attr,&schedparam)) {
		MY_PRINTF("unable to set scheduling parameters\n");
		goto errout;
	}
	if (pthread_create(ptask,&attr,entry,arg)){
		MY_PRINTF("unable to create interlock thread\n");
		goto errout;
	}
#elif defined(__vxworks)
	if (ERROR==
		(*ptask=taskSpawn(
			name,
			np,
			fpTask ? VX_FP_TASK : 0,
			stacksize,
			entry,
			arg,0,0,0,0,0,0,0,0,0))) {
		goto errout;
	}
#elif defined(__rtems)
	strncpy(tmp,name,4);
	rn=rtems_build_name(tmp[0],tmp[1],tmp[2],tmp[3]);
	if (stacksize<RTEMS_MINIMUM_STACK_SIZE)
		stacksize=RTEMS_MINIMUM_STACK_SIZE;
	if (RTEMS_SUCCESSFUL != rtems_task_create(
			rn,
			np,
			stacksize,
			0,/*default task mode*/
			fpTask ? RTEMS_FLOATING_POINT : 0,
			ptask) ||
		RTEMS_SUCCESSFUL != rtems_task_start(
			*ptask,
			entry,
			arg))
		goto errout;

#else
#error "I don't know how to spawn a task on this OS"
#endif
	return 0;
errout:
	*ptask=0;
	return -1;
}

#endif
