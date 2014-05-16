#ifndef SurveillanceGraphNode_H
#define SurveillanceGraphNode_H

#include <cstddef>

class SurveillanceGraphNode {  
  public:
      SurveillanceGraphNode() {
          children = new SurveillanceGraphNode*[2];
          children[0] = NULL;
          children[1] = NULL;
          _n_children = 2;
          _depth = 0;
          parallel_cost = 0;
          sequential_cost = 0;
          obstacle_index = -1;
          split_cost = 0;
          incoming_blocking_cost = 0;
          delay = 0.0;
          time = 0.0;
      }

      SurveillanceGraphNode(int n) {
          children = new SurveillanceGraphNode*[n];
          _n_children = n;
      }
      
      ~SurveillanceGraphNode() {
          delete children;
      }
      
      int parallel_cost;
      int sequential_cost;
      int obstacle_index;
      int incoming_blocking_cost;
      int split_cost;
      int _n_children;
      int _depth;
      float delay;
      float time;
      SurveillanceGraphNode **children;
};

#endif