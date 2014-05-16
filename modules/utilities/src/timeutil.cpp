#include <stdio.h>
#include <math.h>
#include "utilities/timeutil.h"

#define USEC_PER_SEC 1e6

static struct timeval TimeOffset = {0, 0};

struct timeval getCurrentTimeTS()
{
   struct timeval ts; 
   gettimeofday(&ts,0);
   return ts;
}


timeval convertTS(double secs)
{
   timeval ts;
   double intpart;
   double frac = modf(secs, &intpart);
   ts.tv_sec = (long) intpart;
   ts.tv_usec = (long) (frac * (double) USEC_PER_SEC);
   return ts;
}

int TimevalInitialized(struct timeval * time)
{
   if((time->tv_sec == 0) && (time->tv_usec == 0))
      return 0;
   return 1;
}

void TimeutilPrint(FILE* stream, struct timeval *time)
{
   fprintf(stream,"%1.2f",(double)time->tv_sec 
         + (double)time->tv_usec/1E6);
}


void TimeutilGetCurrentTime(struct timeval *currentTime)
{
  if(currentTime == NULL)
    {
      fprintf(stderr, "TimeutiGetCurrentTime: Error\n");
      return;
    }

  gettimeofday(currentTime, NULL);
  AddTimeval(currentTime, &TimeOffset, currentTime); 
}


void TimeutilSetTimeOffset(const struct timeval *timeOffset)
{
  TimeOffset = *timeOffset;
}

void NormTimeval(struct timeval *tv)
{
    if(tv != NULL)
    {
	long us = tv->tv_usec, s = us / USEC_PER_SEC;

	if(us < 0)
 	{
	    s--;
	}
	tv->tv_sec += s;
	tv->tv_usec = us - s * USEC_PER_SEC;
    }
}
	    
void AddTimeval(const struct timeval *tv1, const struct timeval *tv2, 
	struct timeval *tv3)
{
    if(tv1 != NULL && tv2 != NULL && tv3 != NULL)
    {
	tv3->tv_sec = tv1->tv_sec + tv2->tv_sec;
	tv3->tv_usec = tv1->tv_usec + tv2->tv_usec;
	NormTimeval(tv3);
    }
}

void SubTimeval(const struct timeval *tv1, const struct timeval *tv2, 
	struct timeval *tv3)
{
    if(tv1 != NULL && tv2 != NULL && tv3 != NULL)
    {
	tv3->tv_sec = tv1->tv_sec - tv2->tv_sec;
	tv3->tv_usec = tv1->tv_usec - tv2->tv_usec;
	NormTimeval(tv3);
    }
}

void DivTimeval(struct timeval *tv, long div)
{
    if(tv != NULL)
    {
	tv->tv_usec = (tv->tv_usec + (tv->tv_sec % div) * USEC_PER_SEC) / div;
	tv->tv_sec /= div;
	NormTimeval(tv);
    }
}

int TimevalCmp(const struct timeval *tv1, const struct timeval *tv2)
{
  if (timercmp(tv1, tv2, <))
    return -1;
  else if (timercmp(tv1, tv2, >))
    return 1;
  else
    return 0;
}

double TimevalSecs(const struct timeval *tv)
{
    if(tv == NULL)
	return(-1.0);
    else
	return(tv->tv_sec + (double)tv->tv_usec / USEC_PER_SEC);
}

double TimevalUsecs(const struct timeval *tv)
{
    if(tv == NULL)
        return(-1.0);
    else
	return(tv->tv_sec*USEC_PER_SEC+(double)tv->tv_usec);
}

/*
** return time difference in seconds
*/
double TimevalDiff(const struct timeval *t1, const struct timeval *t2)
{
  double time;
 
  time = t1->tv_sec - t2->tv_sec + (t1->tv_usec - t2->tv_usec) / 1E6;
  return(time);
}

/* 
** return time difference in miliseconds
*/
double TimevalDiffMS(const struct timeval *t1, const struct timeval *t2)
{
  double time;
   
  time = (t1->tv_sec - t2->tv_sec) * 1E3 + (t1->tv_usec - t2->tv_usec) / 1E3;
  return(time);
}


void AddDoubleToTimeval(double sec, const struct timeval *tv1, 
			struct timeval *tv2)
{
  tv2->tv_sec  = tv1->tv_sec + (long) sec;
  tv2->tv_usec = tv1->tv_usec + (long) ((sec-(int)sec) * USEC_PER_SEC);
  NormTimeval(tv2);
}

double TimeAgo(const struct timeval *t) {
   struct timeval now;
   gettimeofday(&now, 0);
   return TimevalDiff(&now, t);
}

