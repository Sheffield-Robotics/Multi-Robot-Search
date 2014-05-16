#include "planner/aStarSearch.h"
#include "utilities/misc.h"

#define DEBUG_ASTAR_SMALL (0)
#define DEBUG_ASTAR (0)
#define DEBUG_ASTAR_MAX_NUM (5)    ///< how many expansions to debug.

#define DEFAULT_GREEDY_FACTOR 10.0

// #define NUM_MAX_EXPANSIONS 500000

AStarSearch::AStarSearch()
{
   _closedList = NULL;
   _priorityQueue = NULL;
   _maxDistance = Params::g_obstacleMaxDist * 1000.0;
   _minDistance = Params::g_obstacleMinDist * 1000.0;
   _skillCellExploration = Params::g_skillExploration;
   if (Params::g_greedyExploration)
      _greedyFactor = DEFAULT_GREEDY_FACTOR;
   else
      _greedyFactor = 0.0;
}

AStarSearch::~AStarSearch()
{
}

deque<PlanNode>* AStarSearch::search(PlanningMap* planningMap, int startX, int startZ, int goalX, int goalZ)
{
   if (!planningMap->pointInMap(startX, startZ)) {
      M_ERR("Start %d %d  is not on map!\n",startX, startZ);
      return NULL;
   }
   if (!planningMap->pointInMap(goalX, goalZ)) {
      M_ERR("Goal %d %d  is not on map!\n",goalX, goalZ);
      return NULL;
   }
 
   // Reading parameters from param daemon
   _maxDistance = Params::g_obstacleMaxDist * 1000.0;
   _minDistance = Params::g_obstacleMinDist * 1000.0;
   _skillCellExploration = Params::g_skillExploration;
   if (Params::g_greedyExploration)
      _greedyFactor = DEFAULT_GREEDY_FACTOR;
   else
      _greedyFactor = 0.0;

   if (planningMap->hasVegetation(goalX, goalZ)) {
      M_INFO1("WARNING: Goal Node %d %d has vegetation\n",goalX, goalZ);
   }

   if (!planningMap->isTraversable(goalX, goalZ, _skillCellExploration)) {
      M_ERR("WARNING: Goal Node %d %d is not traversable!\n",goalX, goalZ);
      return NULL;
   }

   const int maxUntraversableSearchDepth = 15;

   AStarNode* goalNode = NULL;

   deleteSearchSpace();

   // Create queue and closed list, only if they do not already exist from former planning calls.
   if(_closedList == NULL)
      _closedList = new set<AStarNode*, AStarNode::PtrLower>();
   if(_priorityQueue == NULL)
      _priorityQueue = new PriorityQueue();

   AStarNode* node = new AStarNode(startX, startZ, 0.0, NULL);
   _priorityQueue->addNode(heuristic(node, goalX, goalZ), node);

   int nExpansions = 0;
   while( goalNode == NULL /*&& (nExpansions < NUM_MAX_EXPANSIONS)*/) {
      if(_priorityQueue->empty()) {
         AStarNode* current = _priorityQueue->popFront();
         goalNode = current;
         if(DEBUG_ASTAR_SMALL)
            printf("AStar: Queue empty: No Path found.\n");
         break;
      }
      AStarNode* current = _priorityQueue->popFront();
      if(goalTest(current, goalX, goalZ)) {
         _closedList->insert(current);
         goalNode = current;
         break;
      }

      if(DEBUG_ASTAR && nExpansions < DEBUG_ASTAR_MAX_NUM) {
         printf("AStar: Closing: %d %d.\n", current->_x, current->_z);
      }

      if(_closedList->find(current) != _closedList->end()) {    // already have a better way to here.
         delete current;
         continue;
      }

      if(DEBUG_ASTAR && nExpansions < DEBUG_ASTAR_MAX_NUM)
         printf("AStar: EXPANDING: %d %d.\n", current->_x, current->_z);

      // close and expand current 
      _closedList->insert(current);
  
      bool currentTraverable = planningMap->isTraversable(current->_x, current->_z, _skillCellExploration);
      nExpansions++;
      for(int i = -1; i <= 1; i++) {
         if( ((current->_x + i) < 0) || ((current->_x + i) >= planningMap->sizeX()) )
            continue;
         for(int j = -1; j <= 1; j++) {
             if( ((current->_z + j) < 0) || ((current->_z + j) >= planningMap->sizeZ()) )
               continue;
            if((i == 0) && (j == 0))   // No need to look at current node
               continue;
             // Do no plan from traversable cells to untraversable cells (but allow planning from untraversable cells)
            if(currentTraverable && (!planningMap->isTraversable(current->_x + i, current->_z + j, _skillCellExploration)))  // traversable -> untraversable
               continue;
            // untraversable -> untraversable, only possible if startNode was untraversable
            if((!currentTraverable) && (!planningMap->isTraversable(current->_x + i, current->_z + j, _skillCellExploration))) {   
               if(current->_depth >= maxUntraversableSearchDepth)    // Do no expand more than maxUntraversableSearchDepth over untraversable nodes -> Forces to plan out of walls
                  continue;
            }

            double dirCost = 1.0;
            if(abs(i) + abs(j) > 1)
               dirCost -= 0.1;

            // Only prefer to plan over explored terrain if close to the robot
            double d1 = hypot(current->_x+i - startX, current->_z+j - startZ) * planningMap->resolution();
            double cellTypeCost = 1.0;

            if (!planningMap->pointInMap(current->_x+1, current->_z+j)) 
               cellTypeCost += 200.0;

            if (d1 < 5000.0 && planningMap->isUnclassified(current->_x+i, current->_z+j))
               cellTypeCost += 0.9;


            double distCost = 5.0 / MIN(5.0, planningMap->getPlanningCells()[current->_x+i][current->_z+j].getDistance()/_maxDistance);
            double cost = current->_cost + (abs(i) + abs(j)) * distCost * dirCost * cellTypeCost;
            //double cost = current->_cost + (abs(i) + abs(j)) * (1 + 10 * planningMap->getPlanningCells()[current->_x + i][current->_z + j].getCost());
            AStarNode* n = new AStarNode(current->_x + i, current->_z + j, cost, current);
            if(_closedList->find(n) != _closedList->end()) {    // Node already closed
               if(DEBUG_ASTAR && nExpansions < DEBUG_ASTAR_MAX_NUM)
                  printf("Already closed: %d %d \n", n->_x, n->_z);
               delete n;
            } else {
               double f = n->_cost + heuristic(n, goalX, goalZ);
               if(DEBUG_ASTAR && nExpansions < DEBUG_ASTAR_MAX_NUM)
                  printf("Adding in queue f(%.2f): %d %d cost:(%.2f)\n", f, n->_x, n->_z, n->_cost);
               _priorityQueue->addNode(f, n);
            }
         }
      }
   }

   /*if(nExpansions >= NUM_MAX_EXPANSIONS) {
      M_WARN("AStar: Max Expansions %d reached.\n", NUM_MAX_EXPANSIONS);
      // Do not generate partial plan here, because the action might still have an older suitable plan.
      // No need to interfere with bad plan here.
   }*/
   if(DEBUG_ASTAR_SMALL)
      printf("nExp: %d\n", nExpansions);
   
   deque<PlanNode>* path = NULL;
   if(goalNode != NULL) {
      path = createPathFromGoal(goalNode, planningMap);
   }

   if(path != NULL) {
      if(DEBUG_ASTAR_SMALL)
         printf("AStar: Path length: %d\n", (int)path->size());
   } else {
      if(DEBUG_ASTAR_SMALL)
         printf("AStar: No path found.\n");
   }

   deleteSearchSpace();

   return path;
}

deque<PlanNode>* AStarSearch::createPathFromGoal(AStarNode* goalNode, PlanningMap* map)
{
   _minDistance = Params::g_obstacleMinDist * 1000.0;
   deque<PlanNode>* path = new deque<PlanNode>();
   while(goalNode != NULL) {
      PlanNode p;
      // Compute narrowness
      double d = map->getPlanningCells()[goalNode->_x][goalNode->_z].getDistance();
      double narrow;
      if (d<=_minDistance)
         narrow = 1.0;
      else if (d>=_maxDistance)
         narrow = 0.0;
      else 
         narrow = 1.0 - (d / _maxDistance); 
      p.narrowness = narrow; 
      p.distance = d;
      //map->grid2worldMM(p.x, p.z, goalNode->_x, goalNode->_z);
      p.x = (double) goalNode->_x;
      p.z = (double) goalNode->_z;
      path->push_front(p);
      goalNode = goalNode->_parent;
   }

   return path;
}

void AStarSearch::deleteSearchSpace()
{
   // clean up closed list
   set<AStarNode*, AStarNode::PtrLower>::iterator it;
   if(_closedList != NULL) {
      for(it = _closedList->begin(); it != _closedList->end(); it++) {
         delete (*it);
      }
   }
   delete _closedList;
   _closedList = NULL;

   // clean up queue
   PriorityQueue::iterator queueIt;
   if(_priorityQueue != NULL) {
      for(queueIt = _priorityQueue->begin(); queueIt != _priorityQueue->end(); queueIt++) {
         delete (*queueIt).second;
      }
   }
   delete _priorityQueue;
   _priorityQueue = NULL;
}

double AStarSearch::heuristic(AStarNode* n, int gx, int gz)
{
   return  (_greedyFactor + 1.0) * hypot(n->_x - gx, n->_z - gz);
}

bool AStarSearch::goalTest(AStarNode* n, int gx, int gz)
{
   if((n->_x == gx) && (n->_z == gz))
      return true;
   return false;
}



