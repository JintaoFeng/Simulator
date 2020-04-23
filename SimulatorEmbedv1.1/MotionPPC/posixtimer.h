#ifndef	POSIXTIMER_H
#define	POSIXTIMER_H


#include <vxWorks.h>        /* our OS */
#include <taskLib.h>        /* task control */
#include <signal.h>         /* signals */
#include <stdio.h>          /* printf */
#include <sysLib.h>         /* sysClkRateGet */
#include <logLib.h>         /* logMsg */
#include <wdLib.h>          /* watchdog routines */
#include <stdlib.h>
#include <string.h>
#include <timers.h>         /* timers */

#include "semLib.h"

void TimerInit(void);
extern SEM_ID	servotimeSemId;


#endif