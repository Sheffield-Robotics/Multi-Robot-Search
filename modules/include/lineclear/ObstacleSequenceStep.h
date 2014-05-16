#ifndef OBSTACLESEQUENCESTEP_H
#define OBSTACLESEQUENCESTEP_H

class ObstacleSequenceStep {  
  public:    
    int i;     // obstacle index 
    short c;   // cost  
    short b;   // subsequent blocking 
    short rho; // ordering c- b_{i-1} 
    bool crit; // flag to identify critical steps 
};

#endif