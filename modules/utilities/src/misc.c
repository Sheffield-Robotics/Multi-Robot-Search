#include "utilities/misc.h"
#include <limits.h>

short clampToShort(double d)
{
   if(d > SHRT_MAX)
      return SHRT_MAX;
   if(d < SHRT_MIN)
      return SHRT_MIN;

   return (short)d;
}

