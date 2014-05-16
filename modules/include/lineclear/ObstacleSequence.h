#ifndef OBSTACLESEQUENCE_H
#define OBSTACLESEQUENCE_H

#include <vector>
#include "ObstacleSequenceStep.h"

class ObstacleSequence {    
  public:

    /*
     * Determine whether the sequence is dominated by
     * other - domination criteria dependent on b, rho
     */
    bool is_dominated_by(ObstacleSequence &other);
    
    int length() { return _steps.size(); };
    int obstacle(int i) { return _steps[i].i;};
    int rho(int i) { return _steps[i].rho;};
    int c(int i) { return _steps[i].c;};
    int b(int i) { return _steps[i].b;};
    
  private:
    
    /* steps of the sequence */
    std::vector<ObstacleSequenceStep> _steps;
    
    
};

#endif