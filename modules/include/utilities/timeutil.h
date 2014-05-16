#ifndef TIMEUTIL_H
#define TIMEUTIL_H

#include <stdio.h>
#include "utilities/misc.h"

#ifdef _WIN32
#include "utilities/gettimeofday.h"
#else
#include <sys/time.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct timeval getCurrentTimeTS(); 
timeval convertTS(double secs);

// return 1, if initialized; 0 if not (tv_sec == tv_usec == 0)
int TimevalInitialized(struct timeval * time);
void TimeutilGetCurrentTime(struct timeval *currentTime);
void TimeutilPrint(FILE* stream, struct timeval *time);
void TimeutilSetTimeOffset(const struct timeval *timeOffset);
void NormTimeval(struct timeval *tm);
void AddTimeval(const struct timeval *tm1, const struct timeval *tm2, 
	struct timeval *tm3);
void SubTimeval(const struct timeval *tm1, const struct timeval *tm2, 
	struct timeval *tm3);
void DivTimeval(struct timeval *tm, long div);
int TimevalCmp(const struct timeval *tv1, const struct timeval *tv2);
double TimevalSecs(const struct timeval *tv);
double TimevalUsecs(const struct timeval *tv);

double TimevalDiff(const struct timeval *t1, const struct timeval *t2);
double TimevalDiffMS(const struct timeval *t1, const struct timeval *t2);
/* returns difference of t1 - t2 in seconds. */

void AddDoubleToTimeval(double sec, const struct timeval *tv1,
			struct timeval *tv2);

double TimeAgo(const struct timeval *t);

#ifdef __cplusplus
}
#endif

#endif

