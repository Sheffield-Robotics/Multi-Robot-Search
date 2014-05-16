#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include "planner/priorityQueue.h"
#include "planner/aStarNode.h"
#include "planner/planningMap.h"
#include <deque>
#include <set>

using std::deque;
using std::set;

struct PlanNode {
   double x;
   double z;
   double th;
   double narrowness;   // a factor between [0,1], 0=free, 1=narrow 
   double distance;      // Distance to the next obstacle
   double color[3]; // For drawing
};


/// A* Algorithm.
class AStarSearch
{
   public:
      AStarSearch();
      ~AStarSearch();

      /// Search a path from start to goal in planningMap
      deque<PlanNode>* search(PlanningMap* planningMap, int startX, int startZ, int goalX, int goalZ);
      void deleteSearchSpace();

   protected:
      deque<PlanNode>* createPathFromGoal(AStarNode* goalNode, PlanningMap* map);
      double heuristic(AStarNode* n, int gx, int gz);
      bool goalTest(AStarNode* n, int gx, int gz);

   protected:
      set<AStarNode*, AStarNode::PtrLower> * _closedList;
      PriorityQueue* _priorityQueue;
      bool _skillCellExploration;
      double _maxDistance;
      double _minDistance;
      double _greedyFactor;
};

#endif

