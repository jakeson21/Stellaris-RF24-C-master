
#include "compatibility.h"

// 3 ticks per call to SysCtlDelay plus about 3 more to compute tick parameter
#define TICKS_PER_mSEC 13333.333333 // 80000000.0/3.0/2
#define TICKS_PER_uSEC 13.333333333 // 80000000.0/3.0/2

clock_t start_time, end_time;
static struct tm start, end;
//static struct timeval start, end;
//static long mtime, seconds, useconds;

/**********************************************************************/
/**
 * This function is added in order to simulate arduino delay() function
 * @param milisec
 */
void __msleep(int milisec)
{
	//uint32_t ticks = ROM_SysCtlClockGet(); //
	unsigned long count = TICKS_PER_mSEC * milisec;
	count = max(1,count);
	SysCtlDelay(count); //The loop takes 3 cycles/loop.
	//usleep(milisec*1000);
}

void __usleep(int microsec)
{
	//uint32_t ticks = ROM_SysCtlClockGet();
	unsigned long count = TICKS_PER_uSEC * microsec;
	count = max(1,count);
	SysCtlDelay(count); //The loop takes 3 cycles/loop.
	//usleep(milisec);
}

/**
 * This function is added in order to simulate arduino millis() function
 */
void __start_timer()
{
	//gettimeofday(&start, NULL);
	start_time = clock();
}

long __millis()
{
    static long mtime, seconds, useconds;

    end_time = clock();

	//gettimeofday(&end, NULL);
    //seconds  = end.tv_sec  - start.tv_sec;
    //useconds = end.tv_usec - start.tv_usec;

    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;	
	return mtime;
}
