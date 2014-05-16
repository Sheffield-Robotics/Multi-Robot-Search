#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <map>
using std::multimap;
using std::pair;

class AStarNode;

/// MultiMap based priority queue.
class PriorityQueue
{
   public:     
      PriorityQueue();
      ~PriorityQueue();

      /// Accessor functions.
      typedef multimap<double, AStarNode*>::iterator iterator;
      iterator begin();
      iterator end();

      void clear();
      bool empty();

      /// A Node n with priority p.
      void addNode(double p, AStarNode* n);

      /// Returns the Node in the queue with the smallest priority
      AStarNode* popFront();

   private:
      multimap<double, AStarNode*> _queue;
};

#endif

