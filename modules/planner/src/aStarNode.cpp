#include "planner/aStarNode.h"
#include <stdio.h>   // for NULL

AStarNode::AStarNode(int x, int z, double cost, AStarNode* parent)
{
   _x = x;
   _z = z;
   _cost = cost;
   _parent = parent;
   if(_parent == NULL)
      _depth = 0;
   else
      _depth = _parent->_depth + 1;
}
   
AStarNode::~AStarNode()
{
}

