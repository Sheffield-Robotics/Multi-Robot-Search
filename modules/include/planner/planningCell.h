#ifndef PLANNING_CELL_H
#define PLANNING_CELL_H

#include "heightmap/heightcell.h"
#include <string>

class ObstacleEdge;

/// A cell to use for planning, having an occupancy.
class PlanningCell : public HeightCell
{
   public:
      enum SKILL_CLASSES {       // this cell has been marked as belonging to a special obstacle, that the robot might be able to overcome
         SC_NONE,
         SC_PALETTE,
         SC_RAMP,
         SC_STAIRS
      };


      PlanningCell();
      ~PlanningCell();

      // For sorting frontier cells
      bool operator()(const PlanningCell* c1, const PlanningCell* c2) {
         return c1->getUtility() > c2->getUtility();
      }

      std::string getSkillClassName() const {
         switch (_skillClass) {
            case SC_NONE:
               return "NONE";
               break;
            case SC_PALETTE:
               return "PALETTE";
               break;
            case SC_RAMP:
               return "RAMP";
               break;
            case SC_STAIRS:
               return "STAIRS";
               break;
         }
         return "";
      }

      void setCoords(int x, int z);
      int x();
      int z();

      double getOccupancy() const;
      void setOccupancy(double occ);

      int getVegetation();
      void setVegetation(int v);

      double getDistance() const;
      void setDistance(double dist);

      double getUtility() const;
      void setUtility(double ut);

      enum SKILL_CLASSES getSkillClass() const;
      void setSkillClass(enum SKILL_CLASSES cls);

      void setObstacleEdge(ObstacleEdge * oe);
      ObstacleEdge* getObstacleEdge() const;

      void setSkillCost(double cost);
      double getSkillCost() const;

      /// The cost to traverse this cell.
      double getCost() const;
   protected:
      enum SKILL_CLASSES _skillClass;
      ObstacleEdge* _obstacleEdge;
      double _distance;
      double _occupancy;
      double _skillCost;   ///< Factor that is determined by the skill if _skillClass is set.
      double _utility;
      int _x, _z; ///< x,z coordinates of this cell in the grid.
      int _vegetation;
};

inline int PlanningCell::x()
{
   return _x;
}
      
inline int PlanningCell::z()
{
   return _z;
}

inline int PlanningCell::getVegetation()
{
   return _vegetation;
}

inline void PlanningCell::setVegetation(int v)
{
   _vegetation  = v;
}

inline double PlanningCell::getOccupancy() const
{
   return _occupancy;
}

inline void PlanningCell::setOccupancy(double occ)
{
   _occupancy = occ;
}

inline double PlanningCell::getUtility() const
{
   return _utility;
}

inline void PlanningCell::setUtility(double ut)
{
   _utility = ut;
}

inline enum PlanningCell::SKILL_CLASSES PlanningCell::getSkillClass() const
{
   return _skillClass;
}

/**
 * For now, uses occupancy.
 */
inline double PlanningCell::getCost() const
{
   return getSkillCost() * getOccupancy();
}

#endif

