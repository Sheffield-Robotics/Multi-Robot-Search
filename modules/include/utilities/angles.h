#ifndef ANGLES_H
#define ANGLES_H

#include "utilities/misc.h"
#include "math.h"

#define norm_rad rescue_normalize_radian

#ifdef	__cplusplus
extern "C" {
#endif

const double M_PI2 = 2*M_PI;

double AnglesSin(double th);
double AnglesCos(double th);
double AnglesTan(double th);
double AnglesAsin(double x);
double AnglesAcos(double x);
double AnglesAtan2(double y, double x);
double AnglesNormAngle(double th);
double AnglesAddAngles(double th1, double th2);
double AnglesSubAngles(double th1, double th2);
double AnglesDiffDEG(double alpha, double beta);
double AnglesDiffRAD(double alpha, double beta);

inline double rescue_normalize_radian(double theta)
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

inline double norm_angle(double angle)
{
  if (angle >= 0)
    return fmod((angle+M_PI),M_PI2)-M_PI;
  else
    return -(fmod((-angle+M_PI),M_PI2)-M_PI);
  return angle;
}

inline double diff_angles(double a1, double a2) {
  double diff = a1 - a2;
  if (diff < -M_PI) diff += M_PI2;
  if (diff >  M_PI) diff -= M_PI2;
  return diff;
}




#ifdef	__cplusplus
}
#endif

#endif
