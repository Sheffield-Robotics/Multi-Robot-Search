#include "utilities/math/bresenham.h"
#include <math.h>
#include <iostream>

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  //printf("Get params for (x1,y1) --> (x2,y2) (%d,%d) --> (%d,%d)\n",p1x,p1y,p2x,p2y);
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex) 
  {
    params->Y1=p1x;
    params->X1=p1y;
    params->Y2=p2x;
    params->X2=p2y;
  } 
  else 
  {
    params->X1=p1x;
    params->Y1=p1y;
    params->X2=p2x;
    params->Y2=p2y;
  }

  if ((p2x - p1x) * (p2y - p1y) < 0) 
  {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  } 
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}
