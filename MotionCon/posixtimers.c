/*******************************************************************
*
*	Revised from posixtimer.c(上地公司)
*	Realize a timer to ditermine the servo cycle
*/

#include "posixtimer.h"
#include "rtcontrol.h"


SEM_ID	servotimeSemId;
	
timer_t	timerId ;

int count;
//int Time1 = 0,Time2 = 0;

void TimerHandler()
{
	count++;
	semGive(servotimeSemId);
}

void TimerInit(void)
{
	struct itimerspec   timeToSet ;             /* time to be set */
	STATUS              	retval = OK;	

	sysClkRateSet(5000);
	timeToSet.it_value.tv_sec     = 0 ;         /* initial (one shot) value */
	timeToSet.it_value.tv_nsec    = 200000;
	timeToSet.it_interval.tv_sec  = 0 ;         /* reload (repetitive) value */
	timeToSet.it_interval.tv_nsec = 200000;	

	servotimeSemId=semBCreate(SEM_Q_FIFO,SEM_EMPTY);//返回信号值得指针

	if( timer_create( CLOCK_REALTIME, NULL, &timerId )  == ERROR )
	{
	    perror( "timedemo: Error in creating the timer\n" );
	    retval = ERROR;
	}
	/*关联时间中断处理函数*/
	if( timer_connect( timerId, &TimerHandler, 2)  == ERROR )
	{
	    perror( "timedemo: Error in connecting the timer\n" );
	    retval = ERROR;
	}
	else if( timer_settime( timerId, CLOCK_REALTIME, &timeToSet, NULL )  == ERROR )
	{
	    perror( "timedemo: Error in setting the timer\n" );
	    retval = ERROR;
	}

	printf("Timer created successfully!\n");

	
	while(1)
	{
		taskDelay(sysClkRateGet());

	}

	return;	
}

void TimerDelete(void)
{
	timer_delete( timerId );
	semDelete(servotimeSemId);
}

void GetCount()
{
	int t1,t2;
	t1=count;
	/*printf("count = %d\n",count);*/
	taskDelay(5*sysClkRateGet());
	t2=count;
	/*printf("count = %d\n",count);*/
	printf("delta count = %d\n",t2-t1);
}
