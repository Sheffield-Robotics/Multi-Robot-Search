#include "planner/priorityQueue.h"

PriorityQueue::PriorityQueue()
{
}
   
PriorityQueue::~PriorityQueue()
{
}
   
PriorityQueue::iterator PriorityQueue::begin()
{
   return _queue.begin();
}
      
PriorityQueue::iterator PriorityQueue::end()
{
   return _queue.end();
}

void PriorityQueue::clear()
{
   _queue.clear();
}
   
bool PriorityQueue::empty()
{
   return _queue.empty();
}
   
void PriorityQueue::addNode(double p, AStarNode* n)
{
   pair<double, AStarNode*> aux(p, n);
   _queue.insert(aux);
}
   
AStarNode* PriorityQueue::popFront()
{
   if(empty())
      return 0;
   AStarNode* front = (*_queue.begin()).second;
   _queue.erase(_queue.begin());
   return front;
}

