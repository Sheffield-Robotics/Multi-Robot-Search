#ifndef MISC_H
#define MISC_H

#include <math.h>
#include <limits.h> 
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/time.h>

#define GLOBAL_UTM_ZONE "17T"
#define COLOR_FILE_EXTENSIOM "_color.tiff"
#define MASK_FILE_EXTENSIOM "_mask.tiff"
#define DEM_FILE_EXTENSIOM "_dem.tiff"

#ifdef	__cplusplus
extern "C" {
#endif

/// Clamps x between a and b.
#ifndef CLAMP
#define CLAMP(x, a, b) ((x) < (a) ? (a) : (((x) > (b)) ? (b) : (x)))
#endif
#ifndef SIGN
#define SIGN(a) (((a) >= 0) ? (1) : (-1))
#endif
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#endif
#ifndef MIN
#define MIN(a, b) (((a) < (b))? (a) : (b))    
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b))? (a) : (b))
#endif
#ifndef ROUND
#define ROUND(x)   (floor(x+0.5))
#endif
#ifndef ABS
#define ABS(x)     (x>=0 ? x : -x)
#endif
#ifndef ALMOST_EQUAL
#define ALMOST_EQUAL(x, y) (ABS(x-y) < 1e-6 ? true : false)
#endif
#ifndef SQR
#define SQR(x) ((x)*(x))
#endif
 
#ifndef COLOR_BOUND
#define COLOR_BOUND(a) (MAX(MIN(a,255),0))    
#endif

#ifndef PREC
#define PREC 1e-12
#endif

/* take sign of a, either -1, 0, or 1 */
#ifndef ZSGN
#define ZSGN(a) (((a)<0) ? -1 : (a)>0 ? 1 : 0)
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) \
if (1) {\
   static double lastDone = 0.0; \
   double nowX = getCurrentTime(); \
   if(nowX - lastDone > (secs)) { \
      code; \
      lastDone = nowX; \
   } \
} else \
  (void)0
#endif

#ifndef PVAR
#define PVAR(var) \
   #var << " = " << (var)
#endif

#ifdef	__cplusplus
}
#endif




short clampToShort(double d);

inline double StraightUp(double x, double min, double max)
{
   if(max == min) {
      return(0.0);
   }

   if(x < min) {
      return(0.0);
   }

   if(x > max) {
      return(1.0);
   }

   return((x - min) / (max - min));
}                               

inline double StraightDown(double x, double min, double max)
{
   if(max == min) {
      return(0.0);
   }

   if(x < min) {
      return(1.0);
   }

   if(x > max) {
      return(0.0);
   }

   return((max - x) / (max - min));
}              


// font attributes
#define FT_BOLD      "\033[1m"
#define FT_UNDERLINE "\033[4m"

//background color
#define BG_BLACK     "\033[40m"
#define BG_RED       "\033[41m"
#define BG_GREEN     "\033[42m"
#define BG_YELLOW    "\033[43m"
#define BG_LIGHTBLUE "\033[44m"
#define BG_MAGENTA   "\033[45m"
#define BG_BLUE      "\033[46m"
#define BG_WHITE     "\033[47m"

// font color
#define CL_BLACK(s)     "\033[30m" << s << "\033[0m"
#define CL_RED(s)       "\033[31m" << s << "\033[0m"
#define CL_GREEN(s)     "\033[32m" << s << "\033[0m"
#define CL_YELLOW(s)    "\033[33m" << s << "\033[0m"
#define CL_LIGHTBLUE(s) "\033[34m" << s << "\033[0m"
#define CL_MAGENTA(s)   "\033[35m" << s << "\033[0m"
#define CL_BLUE(s)      "\033[36m" << s << "\033[0m"
#define CL_WHITE(s)     "\033[37m" << s << "\033[0m"

#define FG_BLACK     "\033[30m"
#define FG_RED       "\033[31m"
#define FG_GREEN     "\033[32m"
#define FG_YELLOW    "\033[33m"
#define FG_LIGHTBLUE "\033[34m"
#define FG_MAGENTA   "\033[35m"
#define FG_BLUE      "\033[36m"
#define FG_WHITE     "\033[37m"

#define FG_NORM      "\033[0m"


inline double getCurrentTime()
{
#ifdef WIN32
	double time = clock();		// Get current processor time
	#if (CLOCKS_PER_SEC != 1000)
		time /= (CLOCKS_PER_SEC / 1000);
	#endif
        return(time / 1000.0);
#else
        struct timeval tv; 
        gettimeofday(&tv,0);
        return(tv.tv_sec + (double)tv.tv_usec / 1e6);
#endif
}

inline void M_ERR(const char fmt[], ... )
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_RED, p, FG_NORM); 
   free(p);
}

inline void M_DEBUG(const char fmt[], ... ) 
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_YELLOW, p, FG_NORM); 
   free(p);
}

inline void M_WARN(const char fmt[], ... ) 
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_BLUE, p, FG_NORM);
   free(p);
}


inline void M_INFO1(const char fmt[], ... ) 
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_GREEN, p, FG_NORM); 
   free(p);
}

inline void M_INFO2(const char fmt[], ... ) 
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_MAGENTA, p, FG_NORM); 
   free(p);
}

inline void M_INFO1_D(int lvl, int l, const char fmt[], ... ) 
{
    if (l < lvl ) {
        char *p;
        va_list ap;
        p = NULL;
        va_start(ap, fmt);
        int dummy = vasprintf(&p, fmt, ap);
        (void) dummy;
        va_end(ap);
        printf("%s%s%s",FG_GREEN, p, FG_NORM); 
        free(p);
    }
}

inline void M_INFO2_D(int lvl, int l, const char fmt[], ... ) 
{
    if (l < lvl ) {
        char *p;
        va_list ap;
        p = NULL;
        va_start(ap, fmt);
        int dummy = vasprintf(&p, fmt, ap);
        (void) dummy;
        va_end(ap);
        printf("%s%s%s",FG_MAGENTA, p, FG_NORM); 
        free(p);
    }
}
inline void M_INFO3_D(int lvl, int l, const char fmt[], ... ) 
{
    if (l < lvl ) {
        char *p;
        va_list ap;
        p = NULL;
        va_start(ap, fmt);
        int dummy = vasprintf(&p, fmt, ap);
        (void) dummy;
        va_end(ap);
        printf("%s%s%s",FG_BLUE, p, FG_NORM); 
        free(p);
    }
}

inline void M_INFO3(const char fmt[], ... ) 
{
   char *p;
   va_list ap;
   p = NULL;
   va_start(ap, fmt);
   int dummy = vasprintf(&p, fmt, ap);
   (void) dummy;
   va_end(ap);
   printf("%s%s%s",FG_BLUE, p, FG_NORM); 
   free(p);
}

inline double rad2deg(double theta)
{
  return (theta * 180.0 / M_PI);
}

inline double deg2rad(double theta)
{
  return (theta * M_PI / 180.0);
}

inline double normalize_radian(double theta)
{
  int multiplier;
  
  if (theta >= -M_PI && theta < M_PI)
    return theta;
  
  multiplier = (int)(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}


#endif
