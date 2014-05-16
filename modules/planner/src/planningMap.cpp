#include "planner/planningMap.h"
#include "utilities/timing.h"

#include <iostream>
#include <fstream>

#define DEBUG_PLANNING_MAP (0)

using namespace std;

PlanningMap::PlanningMap()
{
   _planningCells = NULL;
}
   
PlanningMap::~PlanningMap()
{
   if(_planningCells != NULL) {
      for(int i = 0; i < _sizeX; i++)
         delete [] _planningCells[i];
      delete [] _planningCells;
   }
}

void PlanningMap::createFromHeightMap(HeightMap* map)
{

   //Timing t("Create Planning Map");
   reCreateNewPlanningCells(map->sizeX(), map->sizeZ());

   _lastUpdateTime = map->lastUpdateTime();
   _id = map->id();
   _resolution = map->resolutionMM();
   _worldOffsetX = map->worldOffsetXMM();
   _worldOffsetZ = map->worldOffsetZMM();

   for(int x = 0; x < _sizeX; x++) {
      for(int z = 0; z < _sizeZ; z++) {
         _planningCells[x][z].integrateMeasurement(map->getCellsMM()[x][z].getHeight(), map->getCellsMM()[x][z].getVariance(), map->getCellsMM()[x][z].getLastUpdateTime(), true);
         _planningCells[x][z].setClass(map->getCellsMM()[x][z].getClass());
         _planningCells[x][z].setSkillClass(PlanningCell::SC_NONE);
         _planningCells[x][z].setOccupancy(getClassOccupancy(map, x, z));
         _planningCells[x][z].setVegetation(map->getCellsMM()[x][z].getVegetation());
      }
   }
   //t.printInfo();

   //Timing t2("Compute distance map");
   computeDistanceMap();
   //t2.printInfo();

   if( DEBUG_PLANNING_MAP) {
      writeOccupancyPPM("/tmp/occ.ppm");
      writeDistancePPM("/tmp/dist.ppm");
      writeUtilityPPM("/tmp/util.ppm");
      // _planningMap->writeClassPPM("/tmp/class.ppm");
   }
}


// Compute Distance Map for planning
void PlanningMap::computeDistanceMap()
{
   double distanceThreshold = Params::g_obstacleMaxDist * 1000.0;
   // Set initial distances
   for (int x=0; x<_sizeX; x++){
      for (int z=0; z<_sizeZ; z++){
         enum HeightCell::ELEVATION_CLASSES cls = _planningCells[x][z].getClass();
         bool isObstacle = !(cls == HeightCell::ELC_FLAT_GROUND || 
                cls == HeightCell::ELC_NOT_INITIALIZED) ||
            (_planningCells[x][z].getVegetation() > 0);
         //bool isObstacle = (cls == HeightCell::ELC_WALL);
         if (isObstacle)
            _planningCells[x][z].setDistance(0.0);
         else 
            _planningCells[x][z].setDistance(distanceThreshold);
      }
   }
   double ds = sqrt(2.0) * _resolution;
   double ls = _resolution;

   // compute distances
   for (int x=2; x<_sizeX-1; x++)
      for (int z=2; z<_sizeZ-1; z++)
      {
         float mval = distanceThreshold;
         mval=mval<_planningCells[x-1][z-1].getDistance() + ds ? mval:  _planningCells[x-1][z-1].getDistance() + ds;
         mval=mval<_planningCells[x-1][z].getDistance()   + ls ? mval:  _planningCells[x-1][z].getDistance()   + ls;
         mval=mval<_planningCells[x-1][z+1].getDistance() + ds ? mval:  _planningCells[x-1][z+1].getDistance() + ds;
         mval=mval<_planningCells[x][z-1].getDistance()   + ls ? mval:  _planningCells[x][z-1].getDistance()   + ls;
         mval=mval<_planningCells[x][z+1].getDistance()   + ls ? mval:  _planningCells[x][z+1].getDistance()   + ls;
         mval=mval<_planningCells[x+1][z-1].getDistance() + ds ? mval:  _planningCells[x+1][z-1].getDistance() + ds;
         mval=mval<_planningCells[x+1][z].getDistance()   + ls ? mval:  _planningCells[x+1][z].getDistance()   + ls;
         mval=mval<_planningCells[x+1][z+1].getDistance() + ds ? mval:  _planningCells[x+1][z+1].getDistance() + ds;
         mval=mval<distanceThreshold?mval:distanceThreshold;
         double d = _planningCells[x][z].getDistance() < mval ? _planningCells[x][z].getDistance() : mval;
         _planningCells[x][z].setDistance(d);
      }

   for (int x=_sizeX-2; x>1; x--)
      for (int z=_sizeZ-2; z>1; z--)
      {
         float mval=distanceThreshold;
         mval=mval<_planningCells[x-1][z-1].getDistance() + ds ? mval:  _planningCells[x-1][z-1].getDistance() + ds;
         mval=mval<_planningCells[x-1][z].getDistance()   + ls ? mval:  _planningCells[x-1][z].getDistance()   + ls;
         mval=mval<_planningCells[x-1][z+1].getDistance() + ds ? mval:  _planningCells[x-1][z+1].getDistance() + ds;
         mval=mval<_planningCells[x][z-1].getDistance()   + ls ? mval:  _planningCells[x][z-1].getDistance()   + ls;
         mval=mval<_planningCells[x][z+1].getDistance()   + ls ? mval:  _planningCells[x][z+1].getDistance()   + ls;
         mval=mval<_planningCells[x+1][z-1].getDistance() + ds ? mval:  _planningCells[x+1][z-1].getDistance() + ds;
         mval=mval<_planningCells[x+1][z].getDistance()   + ls ? mval:  _planningCells[x+1][z].getDistance()   + ls;
         mval=mval<_planningCells[x+1][z+1].getDistance() + ds ? mval:  _planningCells[x+1][z+1].getDistance() + ds;
         mval=mval<distanceThreshold?mval:distanceThreshold;
         double d = _planningCells[x][z].getDistance()<mval ? _planningCells[x][z].getDistance() : mval;
         _planningCells[x][z].setDistance(d); 
      }
}

double PlanningMap::getClassOccupancy(HeightMap* hm, int x, int z) const
{
   enum HeightCell::ELEVATION_CLASSES cls = hm->getCellsMM()[x][z].getClass();
   if(cls == HeightCell::ELC_FLAT_GROUND)
      return 0.0;
   if(cls == HeightCell::ELC_NOT_INITIALIZED)
      return 0.5;
   if(cls == HeightCell::ELC_UNCLASSIFIED)
      return 0.5;
   if(_planningCells[x][z].getSkillClass() == PlanningCell::SC_PALETTE)
      return 0.0;
   if(_planningCells[x][z].getSkillClass() == PlanningCell::SC_RAMP)
      return 0.0;
   if(_planningCells[x][z].getSkillClass() == PlanningCell::SC_STAIRS)
      return 0.0;
   if(cls == HeightCell::ELC_DRIVABLE_OBSTACLE)
      return 0.8;
   if(cls == HeightCell::ELC_RAMPED_GROUND)
      return 0.8;
   if(cls == HeightCell::ELC_STAIRS)
      return 0.8;
   return 1.0;
}

void PlanningMap::reCreateNewPlanningCells(int sx, int sz)
{
   if((sx == 0) || (sz == 0)) {
      return;
   }
   if((_sizeX != sx) || (_sizeZ != sz)) {
      if(_planningCells != NULL) {
         for(int i = 0; i < _sizeX; i++)
            delete [] _planningCells[i];
         delete [] _planningCells;
      }
      _planningCells = createNewPlanningMap(sx, sz);

      _sizeX = sx;
      _sizeZ = sz;
   }
}

PlanningCell** PlanningMap::createNewPlanningMap(int sx, int sz) const
{
   PlanningCell** map = new PlanningCell*[sx];
   for(int x = 0; x < sx; x++) {
      map[x] = new PlanningCell[sz];
      for(int z = 0; z < sz; z++) {
         map[x][z].setCoords(x, z);
      }
   }
   return map;
}

// For debugging
void PlanningMap::writeOccupancyPPM(string filename)
{
   printf("Save Occupancy PGM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << _sizeX << " " << _sizeZ << " " << 255 << endl;

   for(int z = 0; z < _sizeZ; z++) {
      for(int x = 0; x < _sizeX; x++) {
         char c = 255 - (int) (255.0 * _planningCells[x][z].getOccupancy());
         os.put(c);
      }
   }
}

void PlanningMap::writeUtilityPPM(string filename)
{
   printf("Save Utility PGM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << _sizeX << " " << _sizeZ << " " << 255 << endl;

   for(int z = 0; z < _sizeZ; z++) {
      for(int x = 0; x < _sizeX; x++) {
         char c = 255 - (int) (255.0 * _planningCells[x][z].getUtility());
         os.put(c);
      }
   }
}

void PlanningMap::writeClassPPM(string filename)
{
   printf("Save Class PGM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << _sizeX << " " << _sizeZ << " " << 255 << endl;

   for(int z = 0; z < _sizeZ; z++) {
      for(int x = 0; x < _sizeX; x++) {
         char c = (int) (255.0 * _planningCells[x][z].getSkillClass());
         c *= 50;
         os.put(c);
      }
   }
}

void PlanningMap::writeDistancePPM(string filename)
{
   printf("Save Distance PGM to %s\n",filename.c_str());
   ofstream os;
   os.open(filename.c_str());

   os << "P5" << endl;
   os << _sizeX << " " << _sizeZ << " " << 255 << endl;

   // Find max
   double max = 0.0;
   double min = HUGE_VAL;
   for(int z = 0; z < _sizeZ; z++) {
      for(int x = 0; x < _sizeX; x++) {
         double d = _planningCells[x][z].getDistance();
         if (d>max)
            max=d;
         if (d<min)
            min=d;
      }
   }

   printf("MAX DIST IS %lf\n",max);
   printf("MIN DIST IS %lf\n",min);
   for(int z = 0; z < _sizeZ; z++) {
      for(int x = 0; x < _sizeX; x++) {
         double f = 255.0 * (_planningCells[x][z].getDistance() / max);
         char c = (unsigned char) f;
         os.put(c);
      }
   }
}

