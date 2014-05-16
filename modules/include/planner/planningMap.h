#ifndef PLANNING_MAP_H
#define PLANNING_MAP_H

#include "heightmap/heightmap.h"
#include "planner/planningCell.h"

/// A map used for planning, consists of PlanningCells.
class PlanningMap : public HeightMap
{
   public:
      PlanningMap();
      ~PlanningMap();

      /// Creates blurred planning map from map.
      void createFromHeightMap(HeightMap* map);

      PlanningCell** getPlanningCells();

      /// Returns if cell at x, z is traversable.
      bool isTraversable(int x, int z, bool skillCellIsTraversable) const;

      bool isUnclassified(int x, int z) const;

      double getDistance(int x, int z) const;

      bool hasVegetation(int x, int z) const;

   protected:
      /// Computes the distance map
      void computeDistanceMap();

      /// Returns occupancy soley based on the cells class.
      double getClassOccupancy(HeightMap* hm, int x, int z) const;

      /// deletes and reallocates (if necessary) all PlanningCell arrays
      void reCreateNewPlanningCells(int sx, int sz);
      PlanningCell** createNewPlanningMap(int sx, int sz) const;

      /// For debugging
      void writeOccupancyPPM(string filename);
      void writeUtilityPPM(string filename);
      void writeClassPPM(string filename);
      void writeDistancePPM(string filename);

   protected:
      PlanningCell** _planningCells;   
};

inline PlanningCell** PlanningMap::getPlanningCells()
{
   return _planningCells;
}

inline bool PlanningMap::isTraversable(int x, int z, bool skillCellIsTraversable) const
{
  //          if (_planningCells[x][z].getVegetation() > 0) 
   //            printf("v");
   if(!skillCellIsTraversable && (_planningCells[x][z].getSkillClass() != PlanningCell::SC_NONE)) {   // skillcells not traversable and this is skillcell
      if((_planningCells[x][z].getClass() == HeightCell::ELC_DRIVABLE_OBSTACLE) || 
            _planningCells[x][z].getClass() == HeightCell::ELC_RAMPED_GROUND || 
            _planningCells[x][z].getVegetation() > 0) {
         // these might be set traversable, because they are skill cells -> dont allow
         return false;
      }
   }
   if (_planningCells[x][z].getVegetation() > 0) 
      return false;
   return (_planningCells[x][z].getDistance() > Params::g_obstacleMinDist * 1000.0);
}

inline bool PlanningMap::hasVegetation(int x, int z) const
{
   if (_planningCells[x][z].getVegetation() > 0) 
      return true;
   else
      return false;
}


inline bool PlanningMap::isUnclassified(int x, int z) const
{
   enum HeightCell::ELEVATION_CLASSES cls = _planningCells[x][z].getClass();
   return (cls == HeightCell::ELC_UNCLASSIFIED || cls == HeightCell::ELC_NOT_INITIALIZED);
}

inline double PlanningMap::getDistance(int x, int z) const
{
   return _planningCells[x][z].getDistance();
}


#endif

