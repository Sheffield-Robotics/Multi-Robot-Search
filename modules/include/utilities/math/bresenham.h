#ifndef BRESENHAM_H
#define BRESENHAM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);

inline void get_current_point(bresenham_param_t *params, int *x, int *y) 
{ 
  if (params->UsingYIndex) 
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  } 
  else 
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

inline int get_next_point(bresenham_param_t *params) 
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))     
    params->DTerm += params->IncrE;
  else 
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

#ifdef __cplusplus
}
#endif

#endif
