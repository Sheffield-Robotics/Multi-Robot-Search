#include <stdio.h>
#include <math.h>
#include "utilities/angles.h"

double AnglesSin(double th)
{
  return(sin(DEG2RAD(th)));
}

double AnglesCos(double th)
{
  return(cos(DEG2RAD(th)));
}

double AnglesTan(double th)
{
  return(tan(DEG2RAD(th)));
}
  
double AnglesAtan2(double y, double x)
{
  return(RAD2DEG(atan2(y, x)));
}

double AnglesAsin(double x)
{
  return(RAD2DEG(asin(x)));
}

double AnglesAcos(double x)
{
  return(RAD2DEG(acos(x)));
}

double AnglesNormAngle(double th)
{
    if (fabs(th) > 180.0 * 3)
    {
	if (fabs(th) > 1e10)
	{
	    fprintf(stderr, "AnglesNormAngle: strange angle: %g\n", th);
	}

	th = th - 360.0 * floor(th / (360.0));
    }

    while (th < -180.0)
    {
	th += 360.0;
    }
    while (th >= 180.0)
    {
	th -= 360.0;
    }
  
    return(th);
}

double AnglesAddAngles(double th1, double th2)
{
  return(AnglesNormAngle(th1 + th2));
}

double AnglesSubAngles(double th1, double th2)
{
  return(AnglesNormAngle(th1 - th2));
}

double AnglesDiffDEG(double alpha, double beta)
{
   double diff = alpha - beta;
   
   if (diff<-180) diff += 360;
   if (diff>180) diff -= 360;
   
   return diff;
}

double AnglesDiffRAD(double alpha, double beta)
{
   double diff = alpha - beta;
   
   if (diff<-M_PI) diff += 2*M_PI;
   if (diff>M_PI) diff -= 2*M_PI;
   
   return diff;
}
