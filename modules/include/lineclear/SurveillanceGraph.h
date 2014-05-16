#ifndef SurveillanceGraph_H
#define SurveillanceGraph_H

#include "SurveillanceGraphNode.h"

#define DEBUG_DEPTH_FIRST 0

class SurveillanceGraph {  
  public:    
      SurveillanceGraph(int n);
      int _n;
      SurveillanceGraphNode *root;
      SurveillanceGraphNode *nodes;
      
      void print();
      void print_children(SurveillanceGraphNode* node,int depth);
      void compute();
      
      void compute_sequential_cost();
      void compute_sequential_cost_node(SurveillanceGraphNode* node );

      void compute_parallel_cost();
      void compute_parallel_cost_node(SurveillanceGraphNode* node );
      
      void compute_parallel_delays();

      void compute_depth_first(int start_at, int end_at);
      float depth_first_par(SurveillanceGraphNode* node, int k, float delta);
      
      int get_parallel_cost();
      
};

#endif