#ifndef A_STAR_NODE_H
#define A_STAR_NODE_H

/// Node class for A* search in planning map.
class AStarNode
{
   public:
   /// Comparator for A* node pointers (for closed list).
   struct PtrLower
   {
      bool operator()(const AStarNode * n1, const AStarNode * n2) const
      {
         if(n1->_x < n2->_x)
            return true;
         if(n1->_x > n2->_x)
            return false;
         return (n1->_z < n2->_z);
      }
   };
   public:
      AStarNode(int x, int z, double cost, AStarNode* parent);
      ~AStarNode();

   public:
      int _x, _z;
      double _cost;
      AStarNode* _parent;
      int _depth;
};

#endif

